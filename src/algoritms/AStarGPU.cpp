// File: kernels/AStarGPU.cpp

#include "algoritms/AStarGPU.hpp"             // Include the function declaration
#include "map/PathfindingUtils.hpp"     // For GridPoint
#include "map/MapProcessingCommon.h"    // For Grid_V3, GridFlags

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <vector_types.h>
#include <limits>
#include <stdexcept>
#include <iostream>
#include <cmath>
#include <cstdio>
#include <vector>
#include <chrono>
#include <algorithm> // For std::max, std::reverse


//-------------------------------------------------------------------
// CUDA Error Checking Macro
//-------------------------------------------------------------------
#ifndef CUDA_CHECK
#define CUDA_CHECK(call) /* ... Same definition as before ... */ \
do { cudaError_t err = call; if (err != cudaSuccess) { fprintf(stderr, "CUDA Error in %s at %s %d: %s (%d)\n", __func__, __FILE__, __LINE__, cudaGetErrorString(err), err); fflush(stderr); return {}; }} while (0)
#endif // CUDA_CHECK


//-------------------------------------------------------------------
// Forward declarations for Device Data Structures
//-------------------------------------------------------------------
// Should match definitions in AStarKernels.cu
typedef struct { float g; float f; int parent_idx; uint8_t visited; } PathNodeGPU_fwd;
typedef struct { int width; int height; float resolution; float* values; uint8_t* flags; } LogicalGridInfoGPU_fwd;
typedef struct { int width; int height; float resolution; float inv_resolution; float origin_offset_x; float origin_offset_y; float* values; } ElevationGridInfoGPU_fwd;


//-------------------------------------------------------------------
// Extern "C" declarations for HOST-CALLABLE WRAPPERS from AStarKernels.cu
//-------------------------------------------------------------------
extern "C" {
    cudaError_t launch_initAStarNodes(dim3 gridDim, dim3 blockDim, cudaStream_t stream, PathNodeGPU_fwd* nodes, int width, int height, int start_idx, int goal_x, int goal_y, float heuristic_weight);
    cudaError_t launch_findLocalBestAStarNodes(dim3 gridDim, dim3 blockDim, size_t sharedMemBytes, cudaStream_t stream, PathNodeGPU_fwd* nodes, int node_count, int* block_bests_idx, float* block_bests_f, int* d_any_open_flag);
    cudaError_t launch_findGlobalAndExpandAStar(dim3 gridDim, dim3 blockDim, size_t sharedMemBytes, cudaStream_t stream, PathNodeGPU_fwd* nodes, int* block_bests_idx, float* block_bests_f, int num_reduce_blocks, int* d_global_best_idx, int* d_path_found_flag, int* d_no_open_nodes_flag, const LogicalGridInfoGPU_fwd* logInfo, const ElevationGridInfoGPU_fwd* elevInfo, int width, int height, int goal_idx, int goal_x, int goal_y, float heuristic_weight);
} // End extern "C"


//-------------------------------------------------------------------
// Host Wrapper Function Implementation
//-------------------------------------------------------------------
std::vector<int> findPathGPU_AStar(
    const mapgeo::Grid_V3& logical_grid,
    const std::vector<float>& elevation_values,
    int elevation_width,
    int elevation_height,
    float log_cell_resolution,
    float elev_cell_resolution,
    float origin_offset_x,
    float origin_offset_y,
    const GridPoint& start,
    const GridPoint& end,
    // --- Tuning Parameters ---
    float heuristic_weight,
    int max_iterations) // Default: calculate based on grid size
{
    // --- 1. Input Validation ---
    if (!logical_grid.isValid() || log_cell_resolution <= 1e-6f) return {};
    if (elevation_values.empty() || elevation_width <= 0 || elevation_height <= 0 || elev_cell_resolution <= 1e-6f) return {};
    int log_width = static_cast<int>(logical_grid.width());
    int log_height = static_cast<int>(logical_grid.height());
    size_t log_size = static_cast<size_t>(log_width) * log_height;
    size_t elev_size = elevation_values.size();
    if (static_cast<size_t>(elevation_width * elevation_height) != elev_size) return {};
    if (!logical_grid.inBounds(start.x, start.y) || !logical_grid.inBounds(end.x, end.y)) return {};
    int startIdx = start.y * log_width + start.x;
    int endIdx = end.y * log_width + end.x;
    const mapgeo::GridCellData& startCell = logical_grid.at(start.x, start.y);
    const mapgeo::GridCellData& endCell = logical_grid.at(end.x, end.y);
    if (startCell.value <= 0.0f || startCell.hasFlag(mapgeo::GridFlags::FLAG_IMPASSABLE) || endCell.value <= 0.0f || endCell.hasFlag(mapgeo::GridFlags::FLAG_IMPASSABLE)) return {};
    if (startIdx == endIdx) return { startIdx };
    if (heuristic_weight <= 0.0f) heuristic_weight = 1.0f; // Ensure positive weight
    if (max_iterations <= 0) {
        max_iterations = static_cast<int>(log_size) * 2; // Default safety limit
    }


    // --- 2. Prepare Host Data (Logical Grid Split) ---
    std::vector<float> h_logical_values(log_size);
    std::vector<uint8_t> h_logical_flags(log_size);
    for (size_t i = 0; i < log_size; ++i) {
        h_logical_values[i] = logical_grid.data()[i].value;
        h_logical_flags[i] = logical_grid.data()[i].flags;
    }
    std::vector<int> h_parents(log_size, -1); // For host reconstruction later


    // --- 3. Allocate GPU Memory ---
    float* d_logical_values = nullptr;
    uint8_t* d_logical_flags = nullptr;
    float* d_elevation_values = nullptr;
    PathNodeGPU_fwd* d_nodes = nullptr; // Array of A* nodes
    int* d_block_bests_idx = nullptr;  // Temp storage for best index per block
    float* d_block_bests_f = nullptr;  // Temp storage for best f-score per block
    int* d_path_found_flag = nullptr; // = 1 if goal reached
    int* d_any_open_flag = nullptr;   // = 1 if any node is in open set
    int* d_no_open_nodes_flag = nullptr; // = 1 if reduction finds no open nodes
    int* d_global_best_idx = nullptr; // Result of reduction
    LogicalGridInfoGPU_fwd* d_logInfoStruct = nullptr;
    ElevationGridInfoGPU_fwd* d_elevInfoStruct = nullptr;

    // Determine kernel launch parameters
    int threadsPerBlock = 256;
    int node_count = static_cast<int>(log_size); // Use int for counts passed to kernels
    int find_blocks = std::min(256, (node_count + threadsPerBlock - 1) / threadsPerBlock); // Limit blocks for reduction
    size_t find_shared_mem = threadsPerBlock * (sizeof(int) + sizeof(float)); // idx + f

    int process_blocks = 1; // Single block for global reduction and expansion
    int process_threads = find_blocks; // Threads needed for reduction = blocks from previous step
    size_t process_shared_mem = process_threads * (sizeof(int) + sizeof(float)); // idx + f

    // --- Helper lambda for cleanup ---
    auto cleanup = [&]() { /* ... free all allocated buffers ... */
        cudaFree(d_logInfoStruct); cudaFree(d_elevInfoStruct); cudaFree(d_logical_values);
        cudaFree(d_logical_flags); cudaFree(d_elevation_values); cudaFree(d_nodes);
        cudaFree(d_block_bests_idx); cudaFree(d_block_bests_f); cudaFree(d_path_found_flag);
        cudaFree(d_any_open_flag); cudaFree(d_no_open_nodes_flag); cudaFree(d_global_best_idx);
        };

    // Allocate memory
    CUDA_CHECK(cudaMalloc(&d_logical_values, log_size * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_logical_flags, log_size * sizeof(uint8_t)));
    CUDA_CHECK(cudaMalloc(&d_elevation_values, elev_size * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_nodes, log_size * sizeof(PathNodeGPU_fwd))); // Main A* node array
    CUDA_CHECK(cudaMalloc(&d_block_bests_idx, find_blocks * sizeof(int)));
    CUDA_CHECK(cudaMalloc(&d_block_bests_f, find_blocks * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_path_found_flag, sizeof(int)));
    CUDA_CHECK(cudaMalloc(&d_any_open_flag, sizeof(int)));
    CUDA_CHECK(cudaMalloc(&d_no_open_nodes_flag, sizeof(int)));
    CUDA_CHECK(cudaMalloc(&d_global_best_idx, sizeof(int)));
    CUDA_CHECK(cudaMalloc(&d_logInfoStruct, sizeof(LogicalGridInfoGPU_fwd)));
    CUDA_CHECK(cudaMalloc(&d_elevInfoStruct, sizeof(ElevationGridInfoGPU_fwd)));


    // --- 4. Prepare Info Structs on Host & Copy Data to GPU ---
    LogicalGridInfoGPU_fwd h_logInfoStruct; /* ... setup ... */
    h_logInfoStruct.width = log_width; h_logInfoStruct.height = log_height;
    h_logInfoStruct.resolution = log_cell_resolution;
    h_logInfoStruct.values = d_logical_values; h_logInfoStruct.flags = d_logical_flags;

    ElevationGridInfoGPU_fwd h_elevInfoStruct; /* ... setup ... */
    h_elevInfoStruct.width = elevation_width; h_elevInfoStruct.height = elevation_height;
    h_elevInfoStruct.resolution = elev_cell_resolution;
    if (std::fabs(elev_cell_resolution) < 1e-9f) { cleanup(); return {}; }
    h_elevInfoStruct.inv_resolution = 1.0f / elev_cell_resolution;
    h_elevInfoStruct.origin_offset_x = origin_offset_x; h_elevInfoStruct.origin_offset_y = origin_offset_y;
    h_elevInfoStruct.values = d_elevation_values;

    cudaStream_t stream = 0;
    CUDA_CHECK(cudaMemcpyAsync(d_logical_values, h_logical_values.data(), log_size * sizeof(float), cudaMemcpyHostToDevice, stream));
    CUDA_CHECK(cudaMemcpyAsync(d_logical_flags, h_logical_flags.data(), log_size * sizeof(uint8_t), cudaMemcpyHostToDevice, stream));
    CUDA_CHECK(cudaMemcpyAsync(d_elevation_values, elevation_values.data(), elev_size * sizeof(float), cudaMemcpyHostToDevice, stream));
    CUDA_CHECK(cudaMemcpyAsync(d_logInfoStruct, &h_logInfoStruct, sizeof(LogicalGridInfoGPU_fwd), cudaMemcpyHostToDevice, stream));
    CUDA_CHECK(cudaMemcpyAsync(d_elevInfoStruct, &h_elevInfoStruct, sizeof(ElevationGridInfoGPU_fwd), cudaMemcpyHostToDevice, stream));
    CUDA_CHECK(cudaMemsetAsync(d_path_found_flag, 0, sizeof(int), stream)); // Init flags to 0
    CUDA_CHECK(cudaMemsetAsync(d_no_open_nodes_flag, 0, sizeof(int), stream));


    // --- 5. Initialize A* Nodes ---
    dim3 initGridDim((node_count + threadsPerBlock - 1) / threadsPerBlock);
    dim3 initBlockDim(threadsPerBlock);
    CUDA_CHECK(launch_initAStarNodes(
        initGridDim, initBlockDim, stream,
        d_nodes, log_width, log_height, startIdx, end.x, end.y, heuristic_weight));


    // --- 6. A* Main Loop ---
    CUDA_CHECK(cudaStreamSynchronize(stream)); // Ensure init is done

    int h_path_found = 0;
    int h_no_open = 0;
    int current_iteration = 0;

    dim3 findGridDim(find_blocks);
    dim3 findBlockDim(threadsPerBlock);
    dim3 processGridDim(process_blocks);
    dim3 processBlockDim(process_threads);


    while (current_iteration < max_iterations) {
        current_iteration++;

        // Reset flags for this iteration
        int zero = 0;
        CUDA_CHECK(cudaMemcpyAsync(d_any_open_flag, &zero, sizeof(int), cudaMemcpyHostToDevice, stream));
        // No need to reset d_no_open_nodes_flag here, it's set by the kernel if needed

        // Find best candidates in each block
        CUDA_CHECK(launch_findLocalBestAStarNodes(
            findGridDim, findBlockDim, find_shared_mem, stream,
            d_nodes, node_count, d_block_bests_idx, d_block_bests_f, d_any_open_flag
        ));

        // Find global best and expand neighbors
        CUDA_CHECK(launch_findGlobalAndExpandAStar(
            processGridDim, processBlockDim, process_shared_mem, stream,
            d_nodes, d_block_bests_idx, d_block_bests_f, find_blocks,
            d_global_best_idx, d_path_found_flag, d_no_open_nodes_flag,
            d_logInfoStruct, d_elevInfoStruct,
            log_width, log_height, endIdx, end.x, end.y, heuristic_weight
        ));

        // Check termination conditions (can be done less frequently for perf boost)
        CUDA_CHECK(cudaMemcpyAsync(&h_path_found, d_path_found_flag, sizeof(int), cudaMemcpyDeviceToHost, stream));
        CUDA_CHECK(cudaMemcpyAsync(&h_no_open, d_no_open_nodes_flag, sizeof(int), cudaMemcpyDeviceToHost, stream));

        // Synchronize needed to get flags for host logic
        CUDA_CHECK(cudaStreamSynchronize(stream));

        if (h_path_found) {
            // printf("A* Info: Path found after %d iterations.\n", current_iteration); // Debug
            break;
        }
        if (h_no_open) {
            // printf("A* Info: No open nodes left after %d iterations.\n", current_iteration); // Debug
            break; // No path possible
        }
    } // End while loop

    if (current_iteration >= max_iterations) {
        fprintf(stderr, "A* Warning: Exceeded max iterations (%d).\n", max_iterations);
    }

    // --- 7. Copy Parent Info Back for Reconstruction ---
    std::vector<int> h_path_indices; // Result path
    if (h_path_found) {
        // Allocate host memory for parent indices AND visited flags (needed for reconstruction logic)
        std::vector<int> temp_parents(log_size);
        std::vector<uint8_t> temp_visited(log_size); // May not be strictly needed if parent traversal works
        std::vector<float> temp_g_scores(log_size); // Needed to confirm goal reachability

        // Create temporary host struct to read back PathNodeGPU data
        std::vector<PathNodeGPU_fwd> h_nodes(log_size);
        CUDA_CHECK(cudaMemcpy(h_nodes.data(), d_nodes, log_size * sizeof(PathNodeGPU_fwd), cudaMemcpyDeviceToHost));

        // Check if goal was actually reached (g score should be finite)
        if (h_nodes[endIdx].g < FLT_MAX) {
            // --- 8. Path Reconstruction (on Host) ---
            std::vector<int> path_reversed;
            int current = endIdx;
            size_t safety_count = 0;
            const size_t max_path_len = log_size + 1;

            while (current != -1 && safety_count < max_path_len) {
                path_reversed.push_back(current);
                if (current == startIdx) break; // Found start node

                if (current < 0 || static_cast<size_t>(current) >= log_size) {
                    fprintf(stderr, "A* Error: Invalid current index (%d) during reconstruction.\n", current);
                    current = -1; break;
                }
                int parent_val = h_nodes[current].parent_idx; // Get parent from copied data
                if (parent_val < -1 || (parent_val != -1 && static_cast<size_t>(parent_val) >= log_size)) {
                    fprintf(stderr, "A* Error: Invalid parent index (%d -> %d) during reconstruction.\n", current, parent_val);
                    current = -1; break;
                }
                current = parent_val;
                safety_count++;
            }

            if (current == startIdx) {
                std::reverse(path_reversed.begin(), path_reversed.end());
                h_path_indices = std::move(path_reversed);
            }
            else if (safety_count >= max_path_len) {
                fprintf(stderr, "A* Warning: Path reconstruction stopped due to cycle or excessive length.\n");
            }
            else {
                fprintf(stderr, "A* Warning: Path reconstruction failed (start node %d not reached).\n", startIdx);
            }
        }
        else {
            fprintf(stderr, "A* Warning: Path found flag set, but goal node G-score is FLT_MAX.\n");
        }
    }

    // --- 9. Free GPU Memory ---
    cleanup();

    return h_path_indices;
}
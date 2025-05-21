// File: kernels/HADS_GPU.cpp

#include "algoritms/HADS_GPU.hpp"             // Include the function declaration
#include "map/PathfindingUtils.hpp"     // Include ORIGINAL C++ header for GridPoint
#include "map/MapProcessingCommon.h"    // Include for Grid_V3 and GridFlags enum for checks

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <vector_types.h> // May not be strictly needed here
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
#define CUDA_CHECK(call)                                                     \
do {                                                                         \
    cudaError_t err = call;                                                  \
    if (err != cudaSuccess) {                                                \
        fprintf(stderr, "CUDA Error in %s at %s %d: %s (%d)\n",             \
                __func__, __FILE__, __LINE__, cudaGetErrorString(err), err); \
        fflush(stderr);                                                      \
        /* Consider adding cleanup before returning */                       \
        return {}; /* Return empty vector on error */                        \
    }                                                                        \
} while (0)
#endif // CUDA_CHECK

//-------------------------------------------------------------------
// Forward declarations for Device Data Structures
//-------------------------------------------------------------------
typedef struct { int width; int height; float resolution; float* values; uint8_t* flags; } LogicalGridInfoGPU_fwd;
typedef struct { int width; int height; float resolution; float inv_resolution; float origin_offset_x; float origin_offset_y; float* values; } ElevationGridInfoGPU_fwd;

//-------------------------------------------------------------------
// Extern "C" declarations for kernels from HADSKernels.cu AND common kernels
//-------------------------------------------------------------------
extern "C" {
    // --- HADS Launch Wrappers ---
    cudaError_t launch_precompute_local_heuristic(dim3 gridDim, dim3 blockDim, cudaStream_t stream, float* d_heuristic, int width, int height, int goal_x, int goal_y, int radius_cells, float W);
    cudaError_t launch_relaxLightEdgesHADS(dim3 gridDim, dim3 blockDim, cudaStream_t stream, const LogicalGridInfoGPU_fwd* logInfoPtr, const ElevationGridInfoGPU_fwd* elevInfoPtr, float* d_distance, int* d_bucket, int* d_nextBucket, int currentBucket, int* d_changed, int* d_parents, float delta_arg, float threshold_arg, const float* d_heuristic, int goal_x, int goal_y, float W, float PRUNE_FACTOR);
    cudaError_t launch_relaxHeavyEdgesHADS(dim3 gridDim, dim3 blockDim, cudaStream_t stream, const LogicalGridInfoGPU_fwd* logInfoPtr, const ElevationGridInfoGPU_fwd* elevInfoPtr, float* d_distance, int* d_bucket, int* d_nextBucket, int currentBucket, int* d_changed, int* d_parents, float delta_arg, float threshold_arg, const float* d_heuristic, int goal_x, int goal_y, float W, float PRUNE_FACTOR);

    // --- Common Utility Launch Wrappers (assuming defined in corresponding .cu file) ---
    cudaError_t launch_updateBuckets(dim3 gridDim, dim3 blockDim, cudaStream_t stream, int* bucket, int* nextBucket, int size);
    cudaError_t launch_findNextBucket(dim3 gridDim, dim3 blockDim, size_t sharedMemBytes, cudaStream_t stream, int* bucket, int size, int currentBucket, int* nextNonEmptyBucket);
    cudaError_t launch_isBucketEmpty(dim3 gridDim, dim3 blockDim, cudaStream_t stream, int* bucket, int size, int bucketNum, int* isEmptyFlag);

}  // End extern "C"


//-------------------------------------------------------------------
// Host Wrapper Function Implementation
//-------------------------------------------------------------------
std::vector<int> findPathGPU_HADS(
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
    float delta_param,
    float light_edge_threshold_param,
    int heuristic_radius_cells,
    float prune_factor,
    float heuristic_weight)
{
    // --- 1. Input Validation ---
    if (!logical_grid.isValid() || log_cell_resolution <= 1e-6f) {
        fprintf(stderr, "HADS_GPU Error: Invalid logical grid or resolution.\n"); return {};
    }
    if (elevation_values.empty() || elevation_width <= 0 || elevation_height <= 0 || elev_cell_resolution <= 1e-6f) {
        fprintf(stderr, "HADS_GPU Error: Invalid elevation data parameters.\n"); return {};
    }
    int log_width = static_cast<int>(logical_grid.width());
    int log_height = static_cast<int>(logical_grid.height());
    size_t log_size = static_cast<size_t>(log_width) * log_height;
    size_t elev_size = elevation_values.size();

    if (static_cast<size_t>(elevation_width * elevation_height) != elev_size) {
        fprintf(stderr, "HADS_GPU Error: Elevation data size mismatch.\n"); return {};
    }
    if (!logical_grid.inBounds(start.x, start.y) || !logical_grid.inBounds(end.x, end.y)) {
        fprintf(stderr, "HADS_GPU Error: Start or End point out of logical grid bounds.\n"); return {};
    }

    int startIdx = start.y * log_width + start.x;
    int endIdx = end.y * log_width + end.x;

    const mapgeo::GridCellData& startCell = logical_grid.at(start.x, start.y);
    const mapgeo::GridCellData& endCell = logical_grid.at(end.x, end.y);
    if (startCell.value <= 0.0f || startCell.hasFlag(mapgeo::GridFlags::FLAG_IMPASSABLE) ||
        endCell.value <= 0.0f || endCell.hasFlag(mapgeo::GridFlags::FLAG_IMPASSABLE)) {
        fprintf(stderr, "HADS_GPU Error: Start or End point is on an impassable cell.\n"); return {};
    }
    if (startIdx == endIdx) return { startIdx };

    // Validate HADS parameters
    if (delta_param <= 1e-6f) { fprintf(stderr, "HADS_GPU Warning: delta_param is very small.\n"); delta_param = 1.0f; }
    if (light_edge_threshold_param <= 0.0f) { fprintf(stderr, "HADS_GPU Warning: light_edge_threshold_param should be positive.\n"); light_edge_threshold_param = delta_param; }
    if (heuristic_radius_cells < 0) { fprintf(stderr, "HADS_GPU Warning: heuristic_radius_cells cannot be negative.\n"); heuristic_radius_cells = 0; }
    if (prune_factor < 1.0f) { fprintf(stderr, "HADS_GPU Warning: prune_factor < 1.0 might prune optimal paths.\n"); }
    if (heuristic_weight <= 0.0f) { fprintf(stderr, "HADS_GPU Warning: heuristic_weight should be positive.\n"); heuristic_weight = 1.0f; }


    // --- 2. Prepare Host Data ---
    std::vector<int> h_parents(log_size, -1);
    std::vector<float> h_distance(log_size, std::numeric_limits<float>::max());
    std::vector<int> h_bucket(log_size, -1);
    std::vector<int> h_nextBucket(log_size, -1);
    std::vector<float> h_heuristic(log_size, -1.0f); // Initialize host heuristic buffer

    h_distance[startIdx] = 0.0f;
    h_bucket[startIdx] = 0;

    std::vector<float> h_logical_values(log_size);
    std::vector<uint8_t> h_logical_flags(log_size);
    for (size_t i = 0; i < log_size; ++i) {
        h_logical_values[i] = logical_grid.data()[i].value;
        h_logical_flags[i] = logical_grid.data()[i].flags;
    }

    // --- 3. Allocate GPU Memory ---
    float* d_logical_values = nullptr;
    uint8_t* d_logical_flags = nullptr;
    float* d_elevation_values = nullptr;
    float* d_distance = nullptr;
    int* d_parents = nullptr;
    int* d_bucket = nullptr;
    int* d_nextBucket = nullptr;
    float* d_heuristic = nullptr; // *** NEW for HADS ***
    int* d_changed = nullptr;
    int* d_nextNonEmptyBucket = nullptr;
    int* d_isBucketEmptyFlag = nullptr;
    LogicalGridInfoGPU_fwd* d_logInfoStruct = nullptr;
    ElevationGridInfoGPU_fwd* d_elevInfoStruct = nullptr;

    auto cleanup = [&]() { /* ... free all allocated buffers ... */
        cudaFree(d_logInfoStruct); cudaFree(d_elevInfoStruct); cudaFree(d_logical_values);
        cudaFree(d_logical_flags); cudaFree(d_elevation_values); cudaFree(d_distance);
        cudaFree(d_parents); cudaFree(d_bucket); cudaFree(d_nextBucket);
        cudaFree(d_heuristic); // *** Free HADS buffer ***
        cudaFree(d_changed); cudaFree(d_nextNonEmptyBucket); cudaFree(d_isBucketEmptyFlag);
        };

    CUDA_CHECK(cudaMalloc(&d_logical_values, log_size * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_logical_flags, log_size * sizeof(uint8_t)));
    CUDA_CHECK(cudaMalloc(&d_elevation_values, elev_size * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_distance, log_size * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_parents, log_size * sizeof(int)));
    CUDA_CHECK(cudaMalloc(&d_bucket, log_size * sizeof(int)));
    CUDA_CHECK(cudaMalloc(&d_nextBucket, log_size * sizeof(int)));
    CUDA_CHECK(cudaMalloc(&d_heuristic, log_size * sizeof(float))); // *** Allocate HADS buffer ***
    CUDA_CHECK(cudaMalloc(&d_changed, sizeof(int)));
    CUDA_CHECK(cudaMalloc(&d_nextNonEmptyBucket, sizeof(int)));
    CUDA_CHECK(cudaMalloc(&d_isBucketEmptyFlag, sizeof(int)));
    CUDA_CHECK(cudaMalloc(&d_logInfoStruct, sizeof(LogicalGridInfoGPU_fwd)));
    CUDA_CHECK(cudaMalloc(&d_elevInfoStruct, sizeof(ElevationGridInfoGPU_fwd)));


    // --- 4. Prepare Info Structs on Host & Copy Data to GPU ---
    LogicalGridInfoGPU_fwd h_logInfoStruct; /* ... */
    h_logInfoStruct.width = log_width; h_logInfoStruct.height = log_height;
    h_logInfoStruct.resolution = log_cell_resolution;
    h_logInfoStruct.values = d_logical_values; h_logInfoStruct.flags = d_logical_flags;

    ElevationGridInfoGPU_fwd h_elevInfoStruct; /* ... */
    h_elevInfoStruct.width = elevation_width; h_elevInfoStruct.height = elevation_height;
    h_elevInfoStruct.resolution = elev_cell_resolution;
    if (std::fabs(elev_cell_resolution) < 1e-9f) { /* error */ cleanup(); return {}; }
    h_elevInfoStruct.inv_resolution = 1.0f / elev_cell_resolution;
    h_elevInfoStruct.origin_offset_x = origin_offset_x; h_elevInfoStruct.origin_offset_y = origin_offset_y;
    h_elevInfoStruct.values = d_elevation_values;

    cudaStream_t stream = 0;
    CUDA_CHECK(cudaMemcpyAsync(d_logical_values, h_logical_values.data(), log_size * sizeof(float), cudaMemcpyHostToDevice, stream));
    CUDA_CHECK(cudaMemcpyAsync(d_logical_flags, h_logical_flags.data(), log_size * sizeof(uint8_t), cudaMemcpyHostToDevice, stream));
    CUDA_CHECK(cudaMemcpyAsync(d_elevation_values, elevation_values.data(), elev_size * sizeof(float), cudaMemcpyHostToDevice, stream));
    CUDA_CHECK(cudaMemcpyAsync(d_distance, h_distance.data(), log_size * sizeof(float), cudaMemcpyHostToDevice, stream));
    CUDA_CHECK(cudaMemcpyAsync(d_parents, h_parents.data(), log_size * sizeof(int), cudaMemcpyHostToDevice, stream));
    CUDA_CHECK(cudaMemcpyAsync(d_bucket, h_bucket.data(), log_size * sizeof(int), cudaMemcpyHostToDevice, stream));
    CUDA_CHECK(cudaMemcpyAsync(d_nextBucket, h_nextBucket.data(), log_size * sizeof(int), cudaMemcpyHostToDevice, stream));
    CUDA_CHECK(cudaMemcpyAsync(d_heuristic, h_heuristic.data(), log_size * sizeof(float), cudaMemcpyHostToDevice, stream)); // Copy initialized heuristic
    CUDA_CHECK(cudaMemcpyAsync(d_logInfoStruct, &h_logInfoStruct, sizeof(LogicalGridInfoGPU_fwd), cudaMemcpyHostToDevice, stream));
    CUDA_CHECK(cudaMemcpyAsync(d_elevInfoStruct, &h_elevInfoStruct, sizeof(ElevationGridInfoGPU_fwd), cudaMemcpyHostToDevice, stream));

    // --- 5. Precompute Heuristics ---
    int threadsPerBlock = 256;
    int blocksPerGrid = (static_cast<int>(log_size) + threadsPerBlock - 1) / threadsPerBlock;
    dim3 gridDim(blocksPerGrid);
    dim3 blockDim(threadsPerBlock);

    // --- Use Wrapper Function ---
    CUDA_CHECK(launch_precompute_local_heuristic(
        gridDim, blockDim, stream,
        d_heuristic, log_width, log_height, end.x, end.y, heuristic_radius_cells, heuristic_weight
    ));
    // No need for cudaGetLastError() here if wrapper returns it and CUDA_CHECK handles it

    // --- 6. HADS Main Loop ---
    CUDA_CHECK(cudaStreamSynchronize(stream));
    size_t sharedMemBytes = static_cast<size_t>(threadsPerBlock) * sizeof(int);
    int currentBucket = 0;
    int h_changed_flag = 0; // Host copy of the changed flag
    int MAX_BUCKET_SAFETY = (log_width + log_height) * 200 / static_cast<int>(std::max(1.0f, floorf(delta_param))); // Adjusted safety limit
    MAX_BUCKET_SAFETY = std::max(100000, MAX_BUCKET_SAFETY); // Ensure reasonable minimum

    while (currentBucket < MAX_BUCKET_SAFETY) {
        int h_isEmpty = 1;
        CUDA_CHECK(cudaMemcpyAsync(d_isBucketEmptyFlag, &h_isEmpty, sizeof(int), cudaMemcpyHostToDevice, stream));
        // --- Use Wrapper Function ---
        CUDA_CHECK(launch_isBucketEmpty(
            gridDim, blockDim, stream,
            d_bucket, static_cast<int>(log_size), currentBucket, d_isBucketEmptyFlag
        ));
        CUDA_CHECK(cudaStreamSynchronize(stream));
        CUDA_CHECK(cudaMemcpy(&h_isEmpty, d_isBucketEmptyFlag, sizeof(int), cudaMemcpyDeviceToHost));

        if (h_isEmpty) {
            int h_nextBucketIdx = std::numeric_limits<int>::max();
            CUDA_CHECK(cudaMemcpyAsync(d_nextNonEmptyBucket, &h_nextBucketIdx, sizeof(int), cudaMemcpyHostToDevice, stream));
            // --- Use Wrapper Function ---
            // Note: Pass sharedMemBytes to wrapper
            CUDA_CHECK(launch_findNextBucket(
                gridDim, blockDim, sharedMemBytes, stream,
                d_bucket, static_cast<int>(log_size), currentBucket, d_nextNonEmptyBucket
            ));
            CUDA_CHECK(cudaStreamSynchronize(stream));
            CUDA_CHECK(cudaMemcpy(&h_nextBucketIdx, d_nextNonEmptyBucket, sizeof(int), cudaMemcpyDeviceToHost));
            if (h_nextBucketIdx == std::numeric_limits<int>::max()) break;
            currentBucket = h_nextBucketIdx;
            continue;
        }

        do { // Light Edge Relaxation Phase
            h_changed_flag = 0;
            CUDA_CHECK(cudaMemcpyAsync(d_changed, &h_changed_flag, sizeof(int), cudaMemcpyHostToDevice, stream));
            // --- Use Wrapper Function ---
            CUDA_CHECK(launch_relaxLightEdgesHADS(
                gridDim, blockDim, stream,
                d_logInfoStruct, d_elevInfoStruct, d_distance, d_bucket, d_nextBucket, currentBucket, d_changed, d_parents,
                delta_param, light_edge_threshold_param, d_heuristic, end.x, end.y, heuristic_weight, prune_factor
            ));
            // --- Use Wrapper Function ---
            CUDA_CHECK(launch_updateBuckets(
                gridDim, blockDim, stream,
                d_bucket, d_nextBucket, static_cast<int>(log_size)
            ));
            CUDA_CHECK(cudaStreamSynchronize(stream));
            CUDA_CHECK(cudaMemcpy(&h_changed_flag, d_changed, sizeof(int), cudaMemcpyDeviceToHost));
        } while (h_changed_flag);

        // Heavy Edge Relaxation Phase
        h_changed_flag = 0;
        CUDA_CHECK(cudaMemcpyAsync(d_changed, &h_changed_flag, sizeof(int), cudaMemcpyHostToDevice, stream));
        // --- Use Wrapper Function ---
        CUDA_CHECK(launch_relaxHeavyEdgesHADS(
            gridDim, blockDim, stream,
            d_logInfoStruct, d_elevInfoStruct, d_distance, d_bucket, d_nextBucket, currentBucket, d_changed, d_parents,
            delta_param, light_edge_threshold_param, d_heuristic, end.x, end.y, heuristic_weight, prune_factor
        ));
        // --- Use Wrapper Function ---
        CUDA_CHECK(launch_updateBuckets(
            gridDim, blockDim, stream,
            d_bucket, d_nextBucket, static_cast<int>(log_size)
        ));
        CUDA_CHECK(cudaStreamSynchronize(stream));

        currentBucket++;
    }

    if (currentBucket >= MAX_BUCKET_SAFETY) {
        fprintf(stderr, "HADS Warning: Exceeded MAX_BUCKET_SAFETY limit (%d). Pathfinding may be incomplete.\n", MAX_BUCKET_SAFETY);
    }

    // --- 7. Copy Results Back to Host ---
    std::vector<int> h_path_indices;
    float final_end_distance;
    CUDA_CHECK(cudaDeviceSynchronize()); // Sync before final copies
    CUDA_CHECK(cudaMemcpy(&final_end_distance, d_distance + endIdx, sizeof(float), cudaMemcpyDeviceToHost));

    if (final_end_distance < std::numeric_limits<float>::max()) {
        CUDA_CHECK(cudaMemcpy(h_parents.data(), d_parents, log_size * sizeof(int), cudaMemcpyDeviceToHost));

        // --- 8. Path Reconstruction (on Host) ---
        std::vector<int> path_reversed;
        int current = endIdx;
        size_t safety_count = 0;
        const size_t max_path_len = log_size + 1;
        while (current != -1 && safety_count < max_path_len) {
            path_reversed.push_back(current);
            if (current == startIdx) break;
            if (current < 0 || static_cast<size_t>(current) >= log_size) {
                fprintf(stderr, "HADS_GPU Error: Invalid current index (%d) during reconstruction.\n", current);
                current = -1; break;
            }
            int parent_val = h_parents[current];
            if (parent_val < -1 || (parent_val != -1 && static_cast<size_t>(parent_val) >= log_size)) {
                fprintf(stderr, "HADS_GPU Error: Invalid parent index (%d -> %d) during reconstruction.\n", current, parent_val);
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
            fprintf(stderr, "HADS_GPU Warning: Path reconstruction stopped due to cycle or excessive length.\n");
            h_path_indices.clear();
        }
        else {
            fprintf(stderr, "HADS_GPU Warning: Path reconstruction failed (start node %d not reached).\n", startIdx);
            h_path_indices.clear();
        }
    }

    // --- 9. Free GPU Memory ---
    cleanup();

    return h_path_indices;
}
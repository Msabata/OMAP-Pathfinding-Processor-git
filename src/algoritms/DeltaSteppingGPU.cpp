// File: DeltaSteppingGPU.cpp

#include "algoritms/DeltaSteppingGPU.hpp"    // Include the function declaration
#include "map/PathfindingUtils.hpp"    // Include ORIGINAL C++ header for GridPoint
#include "map/MapProcessingCommon.h"   // Include for Grid_V3 and GridFlags enum for checks

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <vector_types.h>
#include <limits>
#include <stdexcept>
#include <iostream> // Keep for final status messages if desired
#include <cmath>
#include <cstdio> // Keep for fprintf on error
#include <vector>
#include <chrono>
#include <algorithm> // For std::max

//-------------------------------------------------------------------
// CUDA Error Checking Macro
//-------------------------------------------------------------------
#define CUDA_CHECK(call)                                                     \
do {                                                                         \
    cudaError_t err = call;                                                  \
    if (err != cudaSuccess) {                                                \
        fprintf(stderr, "CUDA Error at %s %d: %s (%d)\n", __FILE__, __LINE__, \
                cudaGetErrorString(err), err);                               \
        fflush(stderr);                                                      \
        /* TODO: Add cleanup for allocated memory before returning */        \
        return {};                                                           \
    }                                                                        \
} while (0)

//-------------------------------------------------------------------
// Extern "C" declarations for kernels from .cu file (Updated Signatures)
//-------------------------------------------------------------------
typedef struct { int width; int height; float resolution; float* values; uint8_t* flags; } LogicalGridInfoGPU_fwd;
typedef struct { int width; int height; float resolution; float inv_resolution; float origin_offset_x; float origin_offset_y; float* values; } ElevationGridInfoGPU_fwd;

extern "C" {
    __global__ void relaxLightEdgesKernel(const LogicalGridInfoGPU_fwd* logInfoPtr, const ElevationGridInfoGPU_fwd* elevInfoPtr, float* d_distance, int* d_bucket, int* d_nextBucket, int currentBucket, int* d_changed, int* d_parents, float delta_arg, float threshold_arg);
    __global__ void relaxHeavyEdgesKernel(const LogicalGridInfoGPU_fwd* logInfoPtr, const ElevationGridInfoGPU_fwd* elevInfoPtr, float* d_distance, int* d_bucket, int* d_nextBucket, int currentBucket, int* d_changed, int* d_parents, float delta_arg, float threshold_arg);
    __global__ void updateBucketsKernel(int* bucket, int* nextBucket, int size);
    __global__ void findNextBucketKernel(int* bucket, int size, int currentBucket, int* nextNonEmptyBucket);
    __global__ void isBucketEmptyKernel(int* bucket, int size, int bucketNum, int* isEmptyFlag);
} // End extern "C"

// --- Constant declarations REMOVED ---


//-------------------------------------------------------------------
// Host Wrapper Function Implementation (Cleaned)
//-------------------------------------------------------------------
std::vector<int> findPathGPU_DeltaStepping(
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
    float delta_param,
    float light_edge_threshold_param)
{
    // --- 1. Input Validation ---
    if (!logical_grid.isValid() || log_cell_resolution <= 1e-6f) {
        fprintf(stderr, "DeltaSteppingGPU Error: Invalid logical grid or resolution.\n"); return {};
    }
    if (elevation_values.empty() || elevation_width <= 0 || elevation_height <= 0 || elev_cell_resolution <= 1e-6f) {
        fprintf(stderr, "DeltaSteppingGPU Error: Invalid elevation data parameters.\n"); return {};
    }
    int log_width = static_cast<int>(logical_grid.width());
    int log_height = static_cast<int>(logical_grid.height());
    size_t log_size = static_cast<size_t>(log_width) * log_height;
    size_t elev_size = elevation_values.size();

    if (static_cast<size_t>(elevation_width * elevation_height) != elev_size) {
        fprintf(stderr, "DeltaSteppingGPU Error: Elevation data size (%zu) mismatch with dimensions (%d x %d = %zu).\n",
            elev_size, elevation_width, elevation_height, static_cast<size_t>(elevation_width * elevation_height)); return {};
    }
    if (!logical_grid.inBounds(start.x, start.y) || !logical_grid.inBounds(end.x, end.y)) {
        fprintf(stderr, "DeltaSteppingGPU Error: Start or End point out of logical grid bounds.\n"); return {};
    }

    int startIdx = start.y * log_width + start.x;
    int endIdx = end.y * log_width + end.x;

    const mapgeo::GridCellData& startCell = logical_grid.at(start.x, start.y);
    const mapgeo::GridCellData& endCell = logical_grid.at(end.x, end.y);
    if (startCell.value <= 0.0f || startCell.hasFlag(mapgeo::GridFlags::FLAG_IMPASSABLE) ||
        endCell.value <= 0.0f || endCell.hasFlag(mapgeo::GridFlags::FLAG_IMPASSABLE)) {
        fprintf(stderr, "DeltaSteppingGPU Error: Start or End point is on an impassable cell.\n"); return {};
    }
    if (startIdx == endIdx) return { startIdx };


    // --- 2. Prepare Host Data ---
    std::vector<int> h_parents(log_size, -1);
    std::vector<float> h_distance(log_size, std::numeric_limits<float>::max());
    std::vector<int> h_bucket(log_size, -1);
    std::vector<int> h_nextBucket(log_size, -1);

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
    int* d_changed = nullptr;
    int* d_nextNonEmptyBucket = nullptr;
    int* d_isBucketEmptyFlag = nullptr;
    LogicalGridInfoGPU_fwd* d_logInfoStruct = nullptr;
    ElevationGridInfoGPU_fwd* d_elevInfoStruct = nullptr;

    // --- Helper lambda for cleanup ---
    auto cleanup = [&]() {
        cudaFree(d_logInfoStruct); cudaFree(d_elevInfoStruct); cudaFree(d_logical_values);
        cudaFree(d_logical_flags); cudaFree(d_elevation_values); cudaFree(d_distance);
        cudaFree(d_parents); cudaFree(d_bucket); cudaFree(d_nextBucket); cudaFree(d_changed);
        cudaFree(d_nextNonEmptyBucket); cudaFree(d_isBucketEmptyFlag);
        };

    CUDA_CHECK(cudaMalloc(&d_logical_values, log_size * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_logical_flags, log_size * sizeof(uint8_t)));
    CUDA_CHECK(cudaMalloc(&d_elevation_values, elev_size * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_distance, log_size * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_parents, log_size * sizeof(int)));
    CUDA_CHECK(cudaMalloc(&d_bucket, log_size * sizeof(int)));
    CUDA_CHECK(cudaMalloc(&d_nextBucket, log_size * sizeof(int)));
    CUDA_CHECK(cudaMalloc(&d_changed, sizeof(int)));
    CUDA_CHECK(cudaMalloc(&d_nextNonEmptyBucket, sizeof(int)));
    CUDA_CHECK(cudaMalloc(&d_isBucketEmptyFlag, sizeof(int)));
    CUDA_CHECK(cudaMalloc(&d_logInfoStruct, sizeof(LogicalGridInfoGPU_fwd)));
    CUDA_CHECK(cudaMalloc(&d_elevInfoStruct, sizeof(ElevationGridInfoGPU_fwd)));


    // --- 4. Prepare Info Structs on Host & Copy Data to GPU ---
    LogicalGridInfoGPU_fwd h_logInfoStruct;
    h_logInfoStruct.width = log_width;
    h_logInfoStruct.height = log_height;
    h_logInfoStruct.resolution = log_cell_resolution;
    h_logInfoStruct.values = d_logical_values;
    h_logInfoStruct.flags = d_logical_flags;

    ElevationGridInfoGPU_fwd h_elevInfoStruct;
    h_elevInfoStruct.width = elevation_width;
    h_elevInfoStruct.height = elevation_height;
    h_elevInfoStruct.resolution = elev_cell_resolution;
    if (std::fabs(elev_cell_resolution) < 1e-9f) {
        fprintf(stderr, "DeltaSteppingGPU Error: Elevation cell resolution is zero or too small.\n");
        cleanup(); return{};
    }
    h_elevInfoStruct.inv_resolution = 1.0f / elev_cell_resolution;
    h_elevInfoStruct.origin_offset_x = origin_offset_x;
    h_elevInfoStruct.origin_offset_y = origin_offset_y;
    h_elevInfoStruct.values = d_elevation_values;

    CUDA_CHECK(cudaMemcpy(d_logical_values, h_logical_values.data(), log_size * sizeof(float), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_logical_flags, h_logical_flags.data(), log_size * sizeof(uint8_t), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_elevation_values, elevation_values.data(), elev_size * sizeof(float), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_distance, h_distance.data(), log_size * sizeof(float), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_parents, h_parents.data(), log_size * sizeof(int), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_bucket, h_bucket.data(), log_size * sizeof(int), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_nextBucket, h_nextBucket.data(), log_size * sizeof(int), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_logInfoStruct, &h_logInfoStruct, sizeof(LogicalGridInfoGPU_fwd), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_elevInfoStruct, &h_elevInfoStruct, sizeof(ElevationGridInfoGPU_fwd), cudaMemcpyHostToDevice));

    // --- REMOVED cudaMemcpyToSymbol calls ---


    // --- 5. Delta-Stepping Main Loop ---
    int threadsPerBlock = 256;
    int blocksPerGrid = (static_cast<int>(log_size) + threadsPerBlock - 1) / threadsPerBlock;
    size_t sharedMemBytes = static_cast<size_t>(threadsPerBlock) * sizeof(int);

    int currentBucket = 0;
    int h_changed_flag = 0;
    int delta_int_floor = (delta_param > 1e-6f) ? static_cast<int>(std::floor(delta_param)) : 1;
    int MAX_BUCKET_SAFETY = (log_width + log_height) * 100 / delta_int_floor;
    MAX_BUCKET_SAFETY = std::max(100000, MAX_BUCKET_SAFETY); // Ensure reasonable minimum


    while (currentBucket < MAX_BUCKET_SAFETY) {
        // --- Check if current bucket is empty ---
        int h_isEmpty = 1;
        CUDA_CHECK(cudaMemcpy(d_isBucketEmptyFlag, &h_isEmpty, sizeof(int), cudaMemcpyHostToDevice));
        isBucketEmptyKernel << <blocksPerGrid, threadsPerBlock >> > (d_bucket, log_size, currentBucket, d_isBucketEmptyFlag);
        CUDA_CHECK(cudaGetLastError());
        CUDA_CHECK(cudaDeviceSynchronize());
        CUDA_CHECK(cudaMemcpy(&h_isEmpty, d_isBucketEmptyFlag, sizeof(int), cudaMemcpyDeviceToHost));

        if (h_isEmpty) {
            // --- Find the next non-empty bucket ---
            int h_nextBucketIdx = std::numeric_limits<int>::max();
            CUDA_CHECK(cudaMemcpy(d_nextNonEmptyBucket, &h_nextBucketIdx, sizeof(int), cudaMemcpyHostToDevice));
            findNextBucketKernel << <blocksPerGrid, threadsPerBlock, sharedMemBytes >> > (
                d_bucket, log_size, currentBucket, d_nextNonEmptyBucket);
            CUDA_CHECK(cudaGetLastError());
            CUDA_CHECK(cudaDeviceSynchronize());
            CUDA_CHECK(cudaMemcpy(&h_nextBucketIdx, d_nextNonEmptyBucket, sizeof(int), cudaMemcpyDeviceToHost));
            if (h_nextBucketIdx == std::numeric_limits<int>::max()) {
                break; // No more reachable nodes
            }
            currentBucket = h_nextBucketIdx;
            continue;
        }

        // --- Process Current Bucket (Relaxations) ---
        // Light Edge Relaxation Phase
        do {
            h_changed_flag = 0;
            CUDA_CHECK(cudaMemcpy(d_changed, &h_changed_flag, sizeof(int), cudaMemcpyHostToDevice));
            relaxLightEdgesKernel << <blocksPerGrid, threadsPerBlock >> > (
                d_logInfoStruct, d_elevInfoStruct,
                d_distance, d_bucket, d_nextBucket, currentBucket, d_changed, d_parents,
                delta_param, light_edge_threshold_param);
            CUDA_CHECK(cudaGetLastError());
            CUDA_CHECK(cudaDeviceSynchronize());
            updateBucketsKernel << <blocksPerGrid, threadsPerBlock >> > (d_bucket, d_nextBucket, log_size);
            CUDA_CHECK(cudaGetLastError());
            CUDA_CHECK(cudaDeviceSynchronize());
            CUDA_CHECK(cudaMemcpy(&h_changed_flag, d_changed, sizeof(int), cudaMemcpyDeviceToHost));
        } while (h_changed_flag);

        // Heavy Edge Relaxation Phase
        h_changed_flag = 0;
        CUDA_CHECK(cudaMemcpy(d_changed, &h_changed_flag, sizeof(int), cudaMemcpyHostToDevice));
        relaxHeavyEdgesKernel << <blocksPerGrid, threadsPerBlock >> > (
            d_logInfoStruct, d_elevInfoStruct,
            d_distance, d_bucket, d_nextBucket, currentBucket, d_changed, d_parents,
            delta_param, light_edge_threshold_param);
        CUDA_CHECK(cudaGetLastError());
        CUDA_CHECK(cudaDeviceSynchronize());
        updateBucketsKernel << <blocksPerGrid, threadsPerBlock >> > (d_bucket, d_nextBucket, log_size);
        CUDA_CHECK(cudaGetLastError());
        CUDA_CHECK(cudaDeviceSynchronize());

        // --- Check if Target Reached ---
        float h_endDistance;
        CUDA_CHECK(cudaMemcpy(&h_endDistance, d_distance + endIdx, sizeof(float), cudaMemcpyDeviceToHost));
        if (h_endDistance < std::numeric_limits<float>::max()) {
            break; // Target reached
        }

        // --- Find the next bucket ---
        int h_nextBucketIdx = std::numeric_limits<int>::max();
        CUDA_CHECK(cudaMemcpy(d_nextNonEmptyBucket, &h_nextBucketIdx, sizeof(int), cudaMemcpyHostToDevice));
        findNextBucketKernel << <blocksPerGrid, threadsPerBlock, sharedMemBytes >> > (
            d_bucket, log_size, currentBucket, d_nextNonEmptyBucket);
        CUDA_CHECK(cudaGetLastError());
        CUDA_CHECK(cudaDeviceSynchronize());
        CUDA_CHECK(cudaMemcpy(&h_nextBucketIdx, d_nextNonEmptyBucket, sizeof(int), cudaMemcpyDeviceToHost));
        if (h_nextBucketIdx == std::numeric_limits<int>::max()) {
            break; // No more buckets
        }
        currentBucket = h_nextBucketIdx;

    } // End while loop


    // --- 6. Copy Results Back to Host ---
    std::vector<int> h_path_indices; // Initialize empty path vector
    float final_end_distance;
    CUDA_CHECK(cudaMemcpy(&final_end_distance, d_distance + endIdx, sizeof(float), cudaMemcpyDeviceToHost));

    if (final_end_distance < std::numeric_limits<float>::max()) {
        CUDA_CHECK(cudaMemcpy(h_parents.data(), d_parents, log_size * sizeof(int), cudaMemcpyDeviceToHost));

        // --- 7. Path Reconstruction (on Host) ---
        std::vector<int> path_reversed;
        int current = endIdx;
        size_t safety_count = 0;
        const size_t max_path_len = log_size + 1;
        while (current != -1 && safety_count < max_path_len) {
            path_reversed.push_back(current);
            if (current == startIdx) break;
            if (current < 0 || static_cast<size_t>(current) >= log_size || h_parents[current] < -1 || (h_parents[current] != -1 && static_cast<size_t>(h_parents[current]) >= log_size)) {
                fprintf(stderr, "DeltaSteppingGPU Error: Invalid parent index (%d -> %d) during reconstruction.\n", current, (current >= 0 && static_cast<size_t>(current) < log_size) ? h_parents[current] : -2);
                fflush(stderr); current = -1; h_path_indices.clear(); break;
            }
            current = h_parents[current];
            safety_count++;
        }
        if (current == startIdx && safety_count < max_path_len) {
            h_path_indices.assign(path_reversed.rbegin(), path_reversed.rend());
        }
        else if (current != -1) {
            fprintf(stderr, "DeltaSteppingGPU Warning: Path reconstruction stopped due to cycle or excessive length (%zu iterations).\n", safety_count); fflush(stderr);
            h_path_indices.clear();
        }
        else {
            fprintf(stderr, "DeltaSteppingGPU Warning: Path reconstruction failed (start node not reached).\n"); fflush(stderr);
            h_path_indices.clear();
        }
    }
    else {
        // Target not reachable, path remains empty
    }


    // --- 8. Free GPU Memory ---
    cleanup();

    return h_path_indices;
}
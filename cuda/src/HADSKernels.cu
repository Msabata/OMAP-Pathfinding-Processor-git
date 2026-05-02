// File: kernels/HADSKernels.cu

// --- Assumed includes/definitions ---
#include "cuda/include/PathfindingUtilsGPU.cuh" // shared structs, atomicMinFloat, getElevationAtDevice,
                                                 // calculateToblerEdgeCost, D_DX_DEV/D_DY_DEV externs

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>

// --- End Assumed includes/definitions ---

// Bring shared device helpers into scope for unqualified use in kernels
using PathfindingUtilsGPU::toIndexGPU;
using PathfindingUtilsGPU::toCoordsGPU;
using PathfindingUtilsGPU::D_DX_DEV;
using PathfindingUtilsGPU::D_DY_DEV;

//===========================================================
// HADS Specific Device Functions
//===========================================================
__device__ inline float calculate_heuristic_gpu(int x, int y, int goal_x, int goal_y, float W) {
    float dx_h = static_cast<float>(x - goal_x);
    float dy_h = static_cast<float>(y - goal_y);
    // No epsilon jitter: pure Euclidean distance keeps the heuristic admissible.
    return W * sqrtf(dx_h * dx_h + dy_h * dy_h);
}

__global__ void precompute_local_heuristic_kernel(
    float* d_heuristic, int width, int height,
    int goal_x, int goal_y, int radius_cells, float W)
{ // This opening brace should now be fine after fixing typedefs
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int stride = blockDim.x * gridDim.x;
    int total_nodes = width * height;
    float radius_sq = static_cast<float>(radius_cells) * radius_cells;
    for (int i = idx; i < total_nodes; i += stride) {
        int x, y;
        toCoordsGPU(i, width, &x, &y);
        float dx_node = static_cast<float>(x - goal_x);
        float dy_node = static_cast<float>(y - goal_y);
        float dist_sq = dx_node * dx_node + dy_node * dy_node;
        if (dist_sq <= radius_sq) {
            d_heuristic[i] = W * sqrtf(dist_sq + 1e-9f);
        }
    }
}

//===========================================================
// HADS Relaxation Kernels
//===========================================================
// *** FIX: Removed extern "C" from kernel DEFINITIONS ***
__global__ void relaxLightEdgesHADS_Kernel(
    const LogicalGridInfoGPU* logInfoPtr, const ElevationGridInfoGPU* elevInfoPtr,
    float* d_distance, int* d_bucket, int* d_nextBucket, int currentBucket,
    int* d_changed, int* d_parents, float delta_arg, float threshold_arg,
    const float* d_heuristic, int goal_x, int goal_y, float W, float PRUNE_FACTOR)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int stride = blockDim.x * gridDim.x;
    int log_width = logInfoPtr->width;
    int size = log_width * logInfoPtr->height;
    for (int i = idx; i < size; i += stride) {
        if (d_bucket[i] == currentBucket && d_distance[i] < FLT_MAX) {
            int x, y; toCoordsGPU(i, log_width, &x, &y); float current_g = d_distance[i];
            float h_u = calculate_heuristic_gpu(x, y, goal_x, goal_y, W);
            float world_x_curr = ((float)x + 0.5f) * logInfoPtr->resolution; float world_y_curr = ((float)y + 0.5f) * logInfoPtr->resolution;
            float current_elevation = getElevationAtDevice(world_x_curr, world_y_curr, elevInfoPtr);
            for (int dir = 0; dir < 8; dir++) {
                int nx = x + D_DX_DEV[dir]; int ny = y + D_DY_DEV[dir];
                if (nx < 0 || nx >= log_width || ny < 0 || ny >= logInfoPtr->height) continue;
                int neighborIdx = toIndexGPU(nx, ny, log_width);
                float h_v = d_heuristic[neighborIdx];
                if (h_v < 0.0f) { h_v = calculate_heuristic_gpu(nx, ny, goal_x, goal_y, W); }
                if (h_v > h_u * PRUNE_FACTOR) { continue; } // Pruned!
                float weight = calculateToblerEdgeCost(x, y, nx, ny, logInfoPtr, elevInfoPtr, current_elevation);
                if (weight >= FLT_MAX || weight > threshold_arg) continue; // Check LIGHT edge
                float tentativeDistance = current_g + weight; float old_dist = atomicMinFloat(&d_distance[neighborIdx], tentativeDistance);
                if (old_dist > tentativeDistance) {
                    int bucket_index = static_cast<int>(floorf(tentativeDistance / delta_arg));
                    atomicExch(&d_nextBucket[neighborIdx], bucket_index);
                    // Use atomicExch to avoid data races when multiple threads update the same neighbor.
                    atomicExch(&d_parents[neighborIdx], i);
                    atomicExch(d_changed, 1);
                }
            }
        }
    }
}

// *** FIX: Removed extern "C" from kernel DEFINITIONS ***
__global__ void relaxHeavyEdgesHADS_Kernel(
    const LogicalGridInfoGPU* logInfoPtr, const ElevationGridInfoGPU* elevInfoPtr,
    float* d_distance, int* d_bucket, int* d_nextBucket, int currentBucket,
    int* d_changed, int* d_parents, float delta_arg, float threshold_arg,
    const float* d_heuristic, int goal_x, int goal_y, float W, float PRUNE_FACTOR)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int stride = blockDim.x * gridDim.x;
    int log_width = logInfoPtr->width;
    int size = log_width * logInfoPtr->height;
    for (int i = idx; i < size; i += stride) {
        if (d_bucket[i] == currentBucket && d_distance[i] < FLT_MAX) {
            int x, y; toCoordsGPU(i, log_width, &x, &y); float current_g = d_distance[i];
            float h_u = calculate_heuristic_gpu(x, y, goal_x, goal_y, W);
            float world_x_curr = ((float)x + 0.5f) * logInfoPtr->resolution; float world_y_curr = ((float)y + 0.5f) * logInfoPtr->resolution;
            float current_elevation = getElevationAtDevice(world_x_curr, world_y_curr, elevInfoPtr);
            for (int dir = 0; dir < 8; dir++) {
                int nx = x + D_DX_DEV[dir]; int ny = y + D_DY_DEV[dir];
                if (nx < 0 || nx >= log_width || ny < 0 || ny >= logInfoPtr->height) continue;
                int neighborIdx = toIndexGPU(nx, ny, log_width);
                float h_v = d_heuristic[neighborIdx];
                if (h_v < 0.0f) { h_v = calculate_heuristic_gpu(nx, ny, goal_x, goal_y, W); }
                if (h_v > h_u * PRUNE_FACTOR) { continue; } // Pruned!
                float weight = calculateToblerEdgeCost(x, y, nx, ny, logInfoPtr, elevInfoPtr, current_elevation);
                if (weight >= FLT_MAX || weight <= threshold_arg) continue; // Check HEAVY edge
                float tentativeDistance = current_g + weight; float old_dist = atomicMinFloat(&d_distance[neighborIdx], tentativeDistance);
                if (old_dist > tentativeDistance) {
                    int bucket_index = static_cast<int>(floorf(tentativeDistance / delta_arg));
                    atomicExch(&d_nextBucket[neighborIdx], bucket_index);
                    // Use atomicExch to avoid data races on heavy-edge parent writes.
                    atomicExch(&d_parents[neighborIdx], i);
                    atomicExch(d_changed, 1);
                }
            }
        }
    }
}

__global__ void findNextBucketKernel_HADS(int* bucket, int size, int currentBucket, int* nextNonEmptyBucket) {
    extern __shared__ int shared_minBucket[];
    int tid = threadIdx.x;
    shared_minBucket[tid] = INT_MAX;
    int idx = blockIdx.x * blockDim.x + tid;
    int stride = blockDim.x * gridDim.x;
    int thread_min = INT_MAX;
    for (int i = idx; i < size; i += stride) {
        if (bucket[i] > currentBucket && bucket[i] != -1 && bucket[i] < thread_min) {
            thread_min = bucket[i];
        }
    }
    shared_minBucket[tid] = thread_min;
    __syncthreads();
    for (unsigned int s = blockDim.x / 2; s > 0; s >>= 1) {
        if (tid < s) {
            if (shared_minBucket[tid + s] < shared_minBucket[tid]) {
                shared_minBucket[tid] = shared_minBucket[tid + s];
            }
        }
        __syncthreads();
    }
    if (tid == 0) {
        if (shared_minBucket[0] != INT_MAX) {
            atomicMin(nextNonEmptyBucket, shared_minBucket[0]);
        }
    }
}


__global__ void isBucketEmptyKernel_HADS(int* bucket, int size, int bucketNum, int* isEmptyFlag) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int stride = blockDim.x * gridDim.x;
    for (int i = idx; i < size; i += stride) {
        if (bucket[i] == bucketNum) {
            atomicExch(isEmptyFlag, 0);
            return;
        }
    }
}
__global__ void updateBucketsKernel_HADS(int* bucket, int* nextBucket, int size) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int stride = blockDim.x * gridDim.x;
    for (int i = idx; i < size; i += stride) {
        if (nextBucket[i] != -1) {
            bucket[i] = nextBucket[i];
            nextBucket[i] = -1;
        }
    }
}
//===========================================================
// Standard Delta-Stepping Utility Kernels (DEFINITIONS REMOVED)
//===========================================================
// Kernels updateBucketsKernel, findNextBucketKernel, isBucketEmptyKernel
// are assumed to be DEFINED in DeltaSteppingKernels.cu (or another linked .cu file)

//===========================================================
// HOST-CALLABLE WRAPPERS for Kernel Launches
// These MUST be callable from C++, so use extern "C"
//===========================================================

extern "C" cudaError_t launch_precompute_local_heuristic(
    dim3 gridDim, dim3 blockDim, cudaStream_t stream,
    float* d_heuristic, int width, int height,
    int goal_x, int goal_y, int radius_cells, float W)
{
    precompute_local_heuristic_kernel << <gridDim, blockDim, 0, stream >> > (
        d_heuristic, width, height, goal_x, goal_y, radius_cells, W);
    // Return potential launch error
    return cudaGetLastError();
}

extern "C" cudaError_t launch_relaxLightEdgesHADS(
    dim3 gridDim, dim3 blockDim, cudaStream_t stream,
    const LogicalGridInfoGPU* logInfoPtr, const ElevationGridInfoGPU* elevInfoPtr,
    float* d_distance, int* d_bucket, int* d_nextBucket, int currentBucket,
    int* d_changed, int* d_parents, float delta_arg, float threshold_arg,
    const float* d_heuristic, int goal_x, int goal_y, float W, float PRUNE_FACTOR)
{
    relaxLightEdgesHADS_Kernel << <gridDim, blockDim, 0, stream >> > (
        logInfoPtr, elevInfoPtr, d_distance, d_bucket, d_nextBucket, currentBucket,
        d_changed, d_parents, delta_arg, threshold_arg,
        d_heuristic, goal_x, goal_y, W, PRUNE_FACTOR);
    return cudaGetLastError();
}

extern "C" cudaError_t launch_relaxHeavyEdgesHADS(
    dim3 gridDim, dim3 blockDim, cudaStream_t stream,
    const LogicalGridInfoGPU* logInfoPtr, const ElevationGridInfoGPU* elevInfoPtr,
    float* d_distance, int* d_bucket, int* d_nextBucket, int currentBucket,
    int* d_changed, int* d_parents, float delta_arg, float threshold_arg,
    const float* d_heuristic, int goal_x, int goal_y, float W, float PRUNE_FACTOR)
{
    relaxHeavyEdgesHADS_Kernel << <gridDim, blockDim, 0, stream >> > (
        logInfoPtr, elevInfoPtr, d_distance, d_bucket, d_nextBucket, currentBucket,
        d_changed, d_parents, delta_arg, threshold_arg,
        d_heuristic, goal_x, goal_y, W, PRUNE_FACTOR);
    return cudaGetLastError();
}

// --- Wrappers for Common Kernels (if needed) ---
// Define similar wrappers here for updateBucketsKernel, findNextBucketKernel, isBucketEmptyKernel
// OR, if DeltaSteppingGPU.cpp already calls these from MSVC, it must ALSO use wrappers
// defined in DeltaSteppingKernels.cu

extern "C" cudaError_t launch_updateBuckets(
    dim3 gridDim, dim3 blockDim, cudaStream_t stream,
    int* bucket, int* nextBucket, int size)
{
    // Assuming updateBucketsKernel is defined elsewhere (.cu) but declared for linking
    // Need the actual kernel definition for this file to compile.
    // If defined in DeltaSteppingKernels.cu, this wrapper needs to be there.
    // For now, let's assume they *are* defined here for illustration
    updateBucketsKernel_HADS << <gridDim, blockDim, 0, stream >> > (bucket, nextBucket, size);
    return cudaGetLastError();
}

extern "C" cudaError_t launch_findNextBucket(
    dim3 gridDim, dim3 blockDim, size_t sharedMemBytes, cudaStream_t stream,
    int* bucket, int size, int currentBucket, int* nextNonEmptyBucket)
{
    findNextBucketKernel_HADS << <gridDim, blockDim, sharedMemBytes, stream >> > (
        bucket, size, currentBucket, nextNonEmptyBucket);
    return cudaGetLastError();
}

extern "C" cudaError_t launch_isBucketEmpty(
    dim3 gridDim, dim3 blockDim, cudaStream_t stream,
    int* bucket, int size, int bucketNum, int* isEmptyFlag)
{
    isBucketEmptyKernel_HADS << <gridDim, blockDim, 0, stream >> > (
        bucket, size, bucketNum, isEmptyFlag);
    return cudaGetLastError();
}

// --- IMPORTANT ---
// If the common kernels (updateBuckets, findNext, isBucketEmpty) are DEFINED
// in DeltaSteppingKernels.cu, then these launch wrappers MUST ALSO be defined
// in DeltaSteppingKernels.cu and declared in DeltaSteppingGPU.cpp using extern "C".
// You cannot define the wrappers here if the kernels themselves are defined elsewhere.
// For this example, I'm assuming the kernels are available here for the wrappers.
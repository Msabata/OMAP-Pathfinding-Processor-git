// File: kernels/HADSKernels.cu

// --- Assumed includes/definitions ---
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>

//#include "DeltaSteppingGPU.cpp"
// #include "PathfindingUtilsGPU.cuh" // Optional: If using shared header
// --- End Assumed includes/definitions ---

//===========================================================
// Utility Device Functions (Defined as inline or included)
//===========================================================
#define EPSILON_GPU 1e-6f
#define FLAG_IMPASSABLE_GPU (1 << 2)

__device__ inline float atomicMinFloat(float* addr, float value) { /* ... definition ... */
    float old;
    old = *addr;
    while (old > value) {
        int* addr_as_int = (int*)addr;
        int old_int = *addr_as_int;
        if (old <= value) break;
        int assumed = atomicCAS(addr_as_int, old_int, __float_as_int(value));
        if (assumed == old_int) {
            break;
        }
        else {
            old = *addr;
        }
    }
    return old;
}
__device__ inline int toIndexGPU(int x, int y, int width) { return y * width + x; }
__device__ inline void toCoordsGPU(int index, int width, int* x, int* y) { if (width <= 0) { *x = -1; *y = -1; return; } *y = index / width; *x = index - (*y * width); }
__device__ int D_DX_DEV[8] = { 1, 0, -1, 0, 1, -1, -1, 1 };
__device__ int D_DY_DEV[8] = { 0, 1, 0, -1, 1, 1, -1, -1 };

// --- Device Data Structures ---
typedef struct {
    int width; int height; float resolution;
    float* values; uint8_t* flags;
} LogicalGridInfoGPU; // *** FIX: Added semicolon ***

typedef struct {
    int width; int height; float resolution; float inv_resolution;
    float origin_offset_x; float origin_offset_y;
    float* values;
} ElevationGridInfoGPU; // *** FIX: Added semicolon ***


// --- Device Helper: Elevation Sampling ---
__device__ inline float getElevationAtDevice(float world_x, float world_y, const ElevationGridInfoGPU* elevInfo) { /* ... definition ... */
    if (!elevInfo || !elevInfo->values || elevInfo->width <= 0 || elevInfo->height <= 0) return 0.0f;
    float rel_x = world_x - elevInfo->origin_offset_x;
    float rel_y = world_y - elevInfo->origin_offset_y;
    if (fabsf(elevInfo->resolution) < EPSILON_GPU) return 0.0f;
    float grid_x_f = rel_x * elevInfo->inv_resolution;
    float grid_y_f = rel_y * elevInfo->inv_resolution;
    int x0 = floorf(grid_x_f);
    int y0 = floorf(grid_y_f);
    int ix0 = max(0, min(x0, elevInfo->width - 1));
    int iy0 = max(0, min(y0, elevInfo->height - 1));
    int ix1 = max(0, min(x0 + 1, elevInfo->width - 1));
    int iy1 = max(0, min(y0 + 1, elevInfo->height - 1));
    unsigned long long idx00 = (unsigned long long)iy0 * elevInfo->width + ix0;
    unsigned long long idx10 = (unsigned long long)iy0 * elevInfo->width + ix1;
    unsigned long long idx01 = (unsigned long long)iy1 * elevInfo->width + ix0;
    unsigned long long idx11 = (unsigned long long)iy1 * elevInfo->width + ix1;
    float Q00 = elevInfo->values[idx00]; float Q10 = elevInfo->values[idx10];
    float Q01 = elevInfo->values[idx01]; float Q11 = elevInfo->values[idx11];
    float tx = fmaxf(0.0f, fminf(grid_x_f - (float)x0, 1.0f));
    float ty = fmaxf(0.0f, fminf(grid_y_f - (float)y0, 1.0f));
    float val_0 = Q00 * (1.0f - tx) + Q10 * tx;
    float val_1 = Q01 * (1.0f - tx) + Q11 * tx;
    return val_0 * (1.0f - ty) + val_1 * ty;
}

// --- Device Helper: Edge Cost Calculation (Tobler) ---
__device__ inline float calculateToblerEdgeCost(int x, int y, int nx, int ny, const LogicalGridInfoGPU* logInfo, const ElevationGridInfoGPU* elevInfo, float current_elevation) { /* ... definition ... */
    if (!logInfo || !logInfo->values || !logInfo->flags || !elevInfo) return FLT_MAX;
    int log_width = logInfo->width; int log_height = logInfo->height;
    if (nx < 0 || nx >= log_width || ny < 0 || ny >= log_height) return FLT_MAX;
    if (logInfo->resolution <= EPSILON_GPU) return FLT_MAX;
    int neighborIdx = toIndexGPU(nx, ny, log_width);
    float neighbor_base_cost = logInfo->values[neighborIdx];
    uint8_t neighbor_flags = logInfo->flags[neighborIdx];
    if (neighbor_base_cost <= 0.0f || (neighbor_flags & FLAG_IMPASSABLE_GPU)) return FLT_MAX;
    float base_geometric_cost = (x != nx && y != ny) ? 1.41421356f : 1.0f;
    float delta_dist_world = base_geometric_cost * logInfo->resolution;
    if (delta_dist_world <= EPSILON_GPU) return FLT_MAX;
    float world_x_neigh = ((float)nx + 0.5f) * logInfo->resolution;
    float world_y_neigh = ((float)ny + 0.5f) * logInfo->resolution;
    float neighbor_elevation = getElevationAtDevice(world_x_neigh, world_y_neigh, elevInfo);
    float delta_h = neighbor_elevation - current_elevation;
    float S = delta_h / delta_dist_world;
    float SlopeFactor = expf(-3.5f * fabsf(S + 0.05f));
    if (SlopeFactor <= EPSILON_GPU) return FLT_MAX;
    float time_penalty = 1.0f / SlopeFactor;
    float final_move_cost = base_geometric_cost * neighbor_base_cost * time_penalty;
    return fmaxf(0.0f, final_move_cost);
}

//===========================================================
// HADS Specific Device Functions
//===========================================================
__device__ inline float calculate_heuristic_gpu(int x, int y, int goal_x, int goal_y, float W) { /* ... definition ... */
    float dx_h = static_cast<float>(x - goal_x);
    float dy_h = static_cast<float>(y - goal_y);
    return W * sqrtf(dx_h * dx_h + dy_h * dy_h + 1e-9f);
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
                    d_parents[neighborIdx] = i;
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
                    d_parents[neighborIdx] = i;
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
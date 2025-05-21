// File: DeltaSteppingKernels.cu

#include "cuda/include/PathfindingUtilsGPU.cuh" // Include the GPU header (uses extern __device__)

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <float.h>  // For FLT_MAX
#include <limits.h> // For INT_MAX used as sentinel
#include <math.h>   // Includes device math functions like fmaxf, fminf, floorf, expf, fabsf
#include <stdint.h> // For uint8_t
// #include <stdio.h> // No longer needed

//===========================================================
// Define Global __device__ Arrays Declared in PathfindingUtilsGPU.cuh
//===========================================================
__device__ int D_DX_DEV[8] = { 1, 0, -1, 0, 1, -1, -1, 1 };
__device__ int D_DY_DEV[8] = { 0, 1, 0, -1, 1, 1, -1, -1 };

//===========================================================
// Utility Device Functions
//===========================================================

#define EPSILON_GPU 1e-6f
#define FLAG_IMPASSABLE_GPU (1 << 2)

// --- atomicMin for float using atomicCAS ---
__device__ inline float atomicMinFloat(float* addr, float value) {
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

//===========================================================
// Device Data Structures
//===========================================================

typedef struct {
    int width; int height; float resolution;
    float* values; uint8_t* flags;
} LogicalGridInfoGPU;

typedef struct {
    int width; int height; float resolution; float inv_resolution;
    float origin_offset_x; float origin_offset_y;
    float* values;
} ElevationGridInfoGPU;

//===========================================================
// Device Helper: Elevation Sampling (Unchanged)
//===========================================================
__device__ float getElevationAtDevice(
    float world_x, float world_y,
    const ElevationGridInfoGPU* elevInfo)
{
    // ... implementation from previous versions ...
    float rel_x = world_x - elevInfo->origin_offset_x;
    float rel_y = world_y - elevInfo->origin_offset_y;
    float grid_x_f = rel_x * elevInfo->inv_resolution;
    float grid_y_f = rel_y * elevInfo->inv_resolution;
    int x0 = floorf(grid_x_f);
    int y0 = floorf(grid_y_f);
    float tx = grid_x_f - (float)x0;
    float ty = grid_y_f - (float)y0;
    int ix0 = max(0, min(x0, elevInfo->width - 1));
    int iy0 = max(0, min(y0, elevInfo->height - 1));
    int ix1 = max(0, min(x0 + 1, elevInfo->width - 1));
    int iy1 = max(0, min(y0 + 1, elevInfo->height - 1));
    unsigned long long idx00 = (unsigned long long)iy0 * elevInfo->width + ix0;
    unsigned long long idx10 = (unsigned long long)iy0 * elevInfo->width + ix1;
    unsigned long long idx01 = (unsigned long long)iy1 * elevInfo->width + ix0;
    unsigned long long idx11 = (unsigned long long)iy1 * elevInfo->width + ix1;
    unsigned long long total_elev_cells = (unsigned long long)elevInfo->width * elevInfo->height;
    if (idx00 >= total_elev_cells || idx10 >= total_elev_cells || idx01 >= total_elev_cells || idx11 >= total_elev_cells) {
        unsigned long long clamped_idx = (unsigned long long)iy0 * elevInfo->width + ix0;
        return (clamped_idx < total_elev_cells) ? elevInfo->values[clamped_idx] : 0.0f;
    }
    float Q00 = elevInfo->values[idx00];
    float Q10 = elevInfo->values[idx10];
    float Q01 = elevInfo->values[idx01];
    float Q11 = elevInfo->values[idx11];
    tx = fmaxf(0.0f, fminf(tx, 1.0f));
    ty = fmaxf(0.0f, fminf(ty, 1.0f));
    float top_interp = Q00 * (1.0f - tx) + Q10 * tx;
    float bottom_interp = Q01 * (1.0f - tx) + Q11 * tx;
    return top_interp * (1.0f - ty) + bottom_interp * ty;
}

//===========================================================
// Device Helper: Edge Cost Calculation (**Simplified Debug Version Active**)
//===========================================================
__device__ float calculateToblerEdgeCost(
    int x, int y, int nx, int ny,
    const LogicalGridInfoGPU* logInfo,
    const ElevationGridInfoGPU* elevInfo,
    float current_elevation) // current_elevation is unused in simplified version
{
    /*
    // --- TEMPORARY DEBUGGING COST FUNCTION ---
    int log_width = logInfo->width;
    if (nx < 0 || nx >= log_width || ny < 0 || ny >= logInfo->height) return FLT_MAX;
    int neighborIdx = PathfindingUtilsGPU::toIndexGPU(nx, ny, log_width);
    float neighbor_base_cost = logInfo->values[neighborIdx];
    uint8_t neighbor_flags = logInfo->flags[neighborIdx];
    if (neighbor_base_cost <= 0.0f || (neighbor_flags & FLAG_IMPASSABLE_GPU)) return FLT_MAX;
    float base_geometric_cost = 1.0f;
    if (x != nx && y != ny) base_geometric_cost = 1.41421356f;
    float simple_cost = base_geometric_cost * fmaxf(0.01f, neighbor_base_cost);
    return simple_cost;*/

    // --- ORIGINAL FULL TOBLER CODE (Commented Out) ---
    int log_width = logInfo->width;
    if (nx < 0 || nx >= log_width || ny < 0 || ny >= logInfo->height) return FLT_MAX;
    int neighborIdx = PathfindingUtilsGPU::toIndexGPU(nx, ny, log_width);
    float neighbor_base_cost = logInfo->values[neighborIdx];
    uint8_t neighbor_flags = logInfo->flags[neighborIdx];
    if (neighbor_base_cost <= 0.0f || (neighbor_flags & FLAG_IMPASSABLE_GPU)) return FLT_MAX;
    float base_geometric_cost = 1.0f;
    if (x != nx && y != ny) base_geometric_cost = 1.41421356f;
    float delta_dist_world = base_geometric_cost * logInfo->resolution;
    if (delta_dist_world <= EPSILON_GPU) return FLT_MAX; // Avoid division by zero
    float world_x_neigh = ((float)nx + 0.5f) * logInfo->resolution;
    float world_y_neigh = ((float)ny + 0.5f) * logInfo->resolution;
    float neighbor_elevation = getElevationAtDevice(world_x_neigh, world_y_neigh, elevInfo);
    float delta_h = neighbor_elevation - current_elevation;
    float S = delta_h / delta_dist_world;
    float SlopeFactor = expf(-3.5f * fabsf(S + 0.05f));
    float time_penalty = (SlopeFactor > EPSILON_GPU) ? (1.0f / SlopeFactor) : FLT_MAX;
    if (time_penalty >= FLT_MAX) return FLT_MAX;
    float final_move_cost = base_geometric_cost * neighbor_base_cost * time_penalty;
    return fmaxf(0.0f, final_move_cost);

}

//===========================================================
// Kernel: relax light edges
//===========================================================
extern "C" __global__ void relaxLightEdgesKernel(
    const LogicalGridInfoGPU* logInfoPtr, const ElevationGridInfoGPU* elevInfoPtr,
    float* d_distance, int* d_bucket, int* d_nextBucket, int currentBucket,
    int* d_changed, int* d_parents, float delta_arg, float threshold_arg)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int stride = blockDim.x * gridDim.x;
    int log_width = logInfoPtr->width;
    int size = log_width * logInfoPtr->height;

    for (int i = idx; i < size; i += stride) {
        if (d_bucket[i] == currentBucket && d_distance[i] < FLT_MAX) {
            int x, y;
            PathfindingUtilsGPU::toCoordsGPU(i, log_width, &x, &y);
            float current_g = d_distance[i];

            // Calculate current elevation (needed for full cost function)
            float world_x_curr = ((float)x + 0.5f) * logInfoPtr->resolution;
            float world_y_curr = ((float)y + 0.5f) * logInfoPtr->resolution;
            float current_elevation = getElevationAtDevice(world_x_curr, world_y_curr, elevInfoPtr);

            for (int dir = 0; dir < 8; dir++) {
                int dx_val = D_DX_DEV[dir];
                int dy_val = D_DY_DEV[dir];
                int nx = x + dx_val; // No volatile needed
                int ny = y + dy_val;

                if (nx < 0 || nx >= log_width || ny < 0 || ny >= logInfoPtr->height) continue;

                float weight = calculateToblerEdgeCost(x, y, nx, ny, logInfoPtr, elevInfoPtr, current_elevation);

                if (weight >= FLT_MAX || weight > threshold_arg) continue;

                float tentativeDistance = current_g + weight;
                int neighborIdx = PathfindingUtilsGPU::toIndexGPU(nx, ny, log_width);

                float old_dist = atomicMinFloat(&d_distance[neighborIdx], tentativeDistance);

                if (old_dist > tentativeDistance) {
                    int bucket_index = static_cast<int>(floorf(tentativeDistance / delta_arg));
                    d_nextBucket[neighborIdx] = bucket_index;
                    d_parents[neighborIdx] = i;
                    atomicExch(d_changed, 1);
                }
            }
        }
    }
}

//===========================================================
// Kernel: relax heavy edges
//===========================================================
extern "C" __global__ void relaxHeavyEdgesKernel(
    const LogicalGridInfoGPU* logInfoPtr, const ElevationGridInfoGPU* elevInfoPtr,
    float* d_distance, int* d_bucket, int* d_nextBucket, int currentBucket,
    int* d_changed, int* d_parents, float delta_arg, float threshold_arg)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int stride = blockDim.x * gridDim.x;
    int log_width = logInfoPtr->width;
    int size = log_width * logInfoPtr->height;

    for (int i = idx; i < size; i += stride) {
        if (d_bucket[i] == currentBucket && d_distance[i] < FLT_MAX) {
            int x, y;
            PathfindingUtilsGPU::toCoordsGPU(i, log_width, &x, &y);
            float current_g = d_distance[i];

            float world_x_curr = ((float)x + 0.5f) * logInfoPtr->resolution;
            float world_y_curr = ((float)y + 0.5f) * logInfoPtr->resolution;
            float current_elevation = getElevationAtDevice(world_x_curr, world_y_curr, elevInfoPtr);

            for (int dir = 0; dir < 8; dir++) {
                int dx_val = D_DX_DEV[dir];
                int dy_val = D_DY_DEV[dir];
                int nx = x + dx_val;
                int ny = y + dy_val;

                if (nx < 0 || nx >= log_width || ny < 0 || ny >= logInfoPtr->height) continue;

                float weight = calculateToblerEdgeCost(x, y, nx, ny, logInfoPtr, elevInfoPtr, current_elevation);

                // Check if HEAVY edge
                if (weight >= FLT_MAX || weight <= threshold_arg) continue;

                float tentativeDistance = current_g + weight;
                int neighborIdx = PathfindingUtilsGPU::toIndexGPU(nx, ny, log_width);

                float old_dist = atomicMinFloat(&d_distance[neighborIdx], tentativeDistance);

                if (old_dist > tentativeDistance) {
                    int bucket_index = static_cast<int>(floorf(tentativeDistance / delta_arg));
                    d_nextBucket[neighborIdx] = bucket_index;
                    d_parents[neighborIdx] = i;
                    atomicExch(d_changed, 1);
                }
            }
        }
    }
}


//===========================================================
// Kernel: update buckets (Unchanged)
//===========================================================
extern "C" __global__ void updateBucketsKernel(int* bucket, int* nextBucket, int size) {
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
// Kernel: find next non-empty bucket (Unchanged)
//===========================================================
extern "C" __global__ void findNextBucketKernel(int* bucket, int size, int currentBucket, int* nextNonEmptyBucket) {
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

//===========================================================
// Kernel: check if a specific bucket is empty (Unchanged)
//===========================================================
extern "C" __global__ void isBucketEmptyKernel(int* bucket, int size, int bucketNum, int* isEmptyFlag) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int stride = blockDim.x * gridDim.x;
    for (int i = idx; i < size; i += stride) {
        if (bucket[i] == bucketNum) {
            atomicExch(isEmptyFlag, 0);
            return;
        }
    }
}
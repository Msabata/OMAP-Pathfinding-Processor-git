// File: DeltaSteppingKernels.cu

#include "cuda/include/PathfindingUtilsGPU.cuh" // shared structs, helpers, and D_DX_DEV/D_DY_DEV externs

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <float.h>  // For FLT_MAX
#include <limits.h> // For INT_MAX used as sentinel
#include <math.h>   // Includes device math functions like fmaxf, fminf, floorf, expf, fabsf
#include <stdint.h> // For uint8_t

//===========================================================
// Define Global __device__ Arrays Declared in PathfindingUtilsGPU.cuh
// (Definitions live here once; all other .cu files use extern declarations.)
//===========================================================
namespace PathfindingUtilsGPU {
    __device__ int D_DX_DEV[8] = { 1, 0, -1, 0, 1, -1, -1, 1 };
    __device__ int D_DY_DEV[8] = { 0, 1, 0, -1, 1, 1, -1, -1 };
}

// Bring shared device helpers into scope for unqualified use in kernels
using PathfindingUtilsGPU::toIndexGPU;
using PathfindingUtilsGPU::toCoordsGPU;
using PathfindingUtilsGPU::D_DX_DEV;
using PathfindingUtilsGPU::D_DY_DEV;

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
                    // Use atomicExch to avoid data races: multiple threads may update the same neighbor.
                    // d_distance is the authoritative sync point; these writes may race but converge
                    // correctly over subsequent relaxation passes.
                    atomicExch(&d_nextBucket[neighborIdx], bucket_index);
                    atomicExch(&d_parents[neighborIdx], i);
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
                    // Use atomicExch to avoid data races on the heavy-edge path as well.
                    atomicExch(&d_nextBucket[neighborIdx], bucket_index);
                    atomicExch(&d_parents[neighborIdx], i);
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
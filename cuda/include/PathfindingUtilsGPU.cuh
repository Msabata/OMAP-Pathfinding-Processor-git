// File: PathfindingUtilsGPU.cuh
#ifndef PATHFINDING_UTILS_GPU_CUH
#define PATHFINDING_UTILS_GPU_CUH

#include <cuda_runtime.h> // For __device__, __constant__
#include <device_launch_parameters.h>
#include <float.h>
#include <math.h>
#include <stdint.h>

//===========================================================
// Shared GPU Constants
//===========================================================
#ifndef EPSILON_GPU
#define EPSILON_GPU 1e-6f
#endif
#ifndef FLAG_IMPASSABLE_GPU
#define FLAG_IMPASSABLE_GPU (1 << 2)
#endif
// Maximum Tobler time penalty: caps extreme slopes as costly but passable.
#ifndef MAX_TOBLER_PENALTY_GPU
#define MAX_TOBLER_PENALTY_GPU 1000.0f
#endif

// Declare constant arrays — defined once in DeltaSteppingKernels.cu
// Using D_ prefix to emphasise Device scope and avoid potential name clashes
namespace PathfindingUtilsGPU {

    extern __device__ int D_DX_DEV[8];
    extern __device__ int D_DY_DEV[8];

    // --- Coordinate/Index Helpers ---
    __device__ inline int toIndexGPU(int x, int y, int width) {
        return y * width + x;
    }

    __device__ inline void toCoordsGPU(int index, int width, int* x, int* y) {
        *y = index / width;
        *x = index - (*y * width);
    }

} // namespace PathfindingUtilsGPU

//===========================================================
// Shared GPU Device Data Structures
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
// Shared GPU Device Helper Functions
//===========================================================

// --- atomicMin for float using atomicCAS ---
__device__ inline float atomicMinFloat(float* addr, float value) {
    float old = *addr;
    while (old > value) {
        int* addr_as_int = (int*)addr;
        int old_int = *addr_as_int;
        if (old <= value) break;
        int assumed = atomicCAS(addr_as_int, old_int, __float_as_int(value));
        if (assumed == old_int) break;
        old = *addr;
    }
    return old;
}

// --- Bilinear elevation sampling ---
__device__ inline float getElevationAtDevice(
    float world_x, float world_y,
    const ElevationGridInfoGPU* elevInfo)
{
    if (!elevInfo || !elevInfo->values || elevInfo->width <= 0 || elevInfo->height <= 0) return 0.0f;
    float rel_x = world_x - elevInfo->origin_offset_x;
    float rel_y = world_y - elevInfo->origin_offset_y;
    if (fabsf(elevInfo->resolution) < EPSILON_GPU) return 0.0f;
    float grid_x_f = rel_x * elevInfo->inv_resolution;
    float grid_y_f = rel_y * elevInfo->inv_resolution;
    int x0 = static_cast<int>(floorf(grid_x_f));
    int y0 = static_cast<int>(floorf(grid_y_f));
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

// --- Tobler edge cost (canonical shared implementation) ---
// Caps extreme slope penalties at MAX_TOBLER_PENALTY_GPU rather than hard-blocking.
__device__ inline float calculateToblerEdgeCost(
    int x, int y, int nx, int ny,
    const LogicalGridInfoGPU* logInfo,
    const ElevationGridInfoGPU* elevInfo,
    float current_elevation)
{
    if (!logInfo || !logInfo->values || !logInfo->flags || !elevInfo) return FLT_MAX;
    int log_width = logInfo->width;
    int log_height = logInfo->height;
    if (nx < 0 || nx >= log_width || ny < 0 || ny >= log_height) return FLT_MAX;
    if (logInfo->resolution <= EPSILON_GPU) return FLT_MAX;
    int neighborIdx = PathfindingUtilsGPU::toIndexGPU(nx, ny, log_width);
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
    float time_penalty = (SlopeFactor > EPSILON_GPU)
        ? fminf(1.0f / SlopeFactor, MAX_TOBLER_PENALTY_GPU)
        : FLT_MAX;
    if (time_penalty >= FLT_MAX) return FLT_MAX;
    float final_move_cost = base_geometric_cost * neighbor_base_cost * time_penalty;
    return fmaxf(0.0f, final_move_cost);
}

#endif // PATHFINDING_UTILS_GPU_CUH
// File: PathfindingUtilsGPU.cuh
#ifndef PATHFINDING_UTILS_GPU_CUH
#define PATHFINDING_UTILS_GPU_CUH

#include <cuda_runtime.h> // For __device__, __constant__
#include <device_launch_parameters.h>

// Declare constants needed by the Delta-Stepping kernels
namespace PathfindingUtilsGPU {

    // Declare constant arrays - they will be defined in DeltaSteppingKernels.cu
    // Using D_ prefix to emphasize Device scope and avoid potential name clashes
    extern __device__ int D_DX_DEV[8];
    extern __device__ int D_DY_DEV[8];
    // Note: D_COSTS is not needed as Tobler cost is dynamic

    // --- Coordinate/Index Helpers (Device versions) ---
    // Defined inline here as they are simple and used within device code
    __device__ inline int toIndexGPU(int x, int y, int width) {
        // Assumes x, y are already validated or boundary checks are done elsewhere
        return y * width + x;
    }

    __device__ inline void toCoordsGPU(int index, int width, int* x, int* y) {
        // Integer division and modulo
        *y = index / width;
        *x = index - (*y * width); // Slightly faster than % often
    }

} // namespace PathfindingUtilsGPU

#endif // PATHFINDING_UTILS_GPU_CUH
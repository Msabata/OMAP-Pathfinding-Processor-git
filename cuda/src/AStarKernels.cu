// File: kernels/AStarKernels.cu

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>
// Include or define common utilities (inline device functions are okay)
// #include "PathfindingUtilsGPU.cuh"


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
__device__ inline float calculate_heuristic_gpu(int x, int y, int goal_x, int goal_y, float W) { /* Euclidean definition */
    float dx_h = static_cast<float>(x - goal_x); float dy_h = static_cast<float>(y - goal_y);
    return W * sqrtf(dx_h * dx_h + dy_h * dy_h + 1e-9f);
}


//===========================================================
// A* Specific Data Structures & Kernels
//===========================================================

// Optimized node structure
typedef struct {
    float g;           // Cost from start (float)
    float f;           // Total estimated cost (g + h) (float)
    int parent_idx;    // Index of the node this came from
    uint8_t visited;   // 0: unvisited, 1: open (in fringe), 2: closed (visited)
} PathNodeGPU; // Renamed slightly to avoid conflict if PathNode exists elsewhere


// Kernel to initialize all nodes
__global__ void initAStarNodesKernel(PathNodeGPU* nodes, int width, int height, int start_idx, int goal_x, int goal_y, float heuristic_weight) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int stride = blockDim.x * gridDim.x;
    int total_nodes = width * height;

    for (int i = idx; i < total_nodes; i += stride) {
        nodes[i].g = FLT_MAX;
        nodes[i].f = FLT_MAX;
        nodes[i].parent_idx = -1;
        nodes[i].visited = 0; // Mark as unvisited

        // Special handling for the start node
        if (i == start_idx) {
            int start_x, start_y;
            toCoordsGPU(i, width, &start_x, &start_y);
            float h_start = calculate_heuristic_gpu(start_x, start_y, goal_x, goal_y, heuristic_weight);
            nodes[i].g = 0.0f;
            nodes[i].f = h_start; // f = g + h = 0 + h
            nodes[i].visited = 1; // Mark start node as open
        }
    }
}

// Kernel to find the best node candidate within each block (lowest f-score)
__global__ void findLocalBestAStarNodes(PathNodeGPU* nodes, int node_count,
    int* block_bests_idx, float* block_bests_f, // Use float for f
    int* d_any_open_flag)
{
    // Shared memory for reduction within the block (idx and f)
    extern __shared__ char shared_mem_bytes[]; // Use char array for flexible types
    int* shared_idx = (int*)shared_mem_bytes;
    float* shared_f = (float*)(shared_mem_bytes + blockDim.x * sizeof(int));

    int tid = threadIdx.x;
    int bid = blockIdx.x;
    int block_dim = blockDim.x;
    int grid_dim = gridDim.x;

    int local_best_idx = -1;
    float local_best_f = FLT_MAX; // Use FLT_MAX

    // Grid-stride loop
    for (int i = bid * block_dim + tid; i < node_count; i += grid_dim * block_dim) {
        // Consider only nodes in the open set (visited == 1)
        // Breaking ties by lower g-score can sometimes help, but adds complexity. Sticking to f-score only.
        if (nodes[i].visited == 1 && nodes[i].f < local_best_f) {
            local_best_f = nodes[i].f;
            local_best_idx = i;
        }
    }

    // Store thread's best in shared memory
    shared_idx[tid] = local_best_idx;
    shared_f[tid] = local_best_f;
    __syncthreads();

    // Reduction to find block's best node
    for (int s = block_dim / 2; s > 0; s >>= 1) {
        if (tid < s) {
            int other_idx = shared_idx[tid + s];
            float other_f = shared_f[tid + s];
            if (other_idx != -1 && (other_f < shared_f[tid] || shared_idx[tid] == -1)) {
                shared_idx[tid] = other_idx;
                shared_f[tid] = other_f;
            }
        }
        __syncthreads();
    }

    // Block leader writes result and updates global flag
    if (tid == 0) {
        block_bests_idx[bid] = shared_idx[0];
        block_bests_f[bid] = shared_f[0];
        if (shared_idx[0] != -1) {
            atomicExch(d_any_open_flag, 1); // Signal that *some* open node exists globally
        }
    }
}

// Kernel to find the global best node AND expand its neighbors
__global__ void findGlobalAndExpandAStar(
    PathNodeGPU* nodes,
    int* block_bests_idx, float* block_bests_f, // Use float for f
    int num_reduce_blocks, // Number of blocks from the previous kernel
    int* d_global_best_idx, // Output: index of the globally best node
    int* d_path_found_flag,
    int* d_no_open_nodes_flag, // Added flag to signal if no best node found
    // Grid and Cost Function Params
    const LogicalGridInfoGPU* logInfo, const ElevationGridInfoGPU* elevInfo,
    int width, int height, int goal_idx, int goal_x, int goal_y, float heuristic_weight)
{
    // --- Phase 1: Find Global Best (Executed by a single block) ---
    // Shared memory for reduction across blocks (idx and f)
    extern __shared__ char shared_mem_bytes[];
    int* shared_idx = (int*)shared_mem_bytes;
    float* shared_f = (float*)(shared_mem_bytes + blockDim.x * sizeof(int));

    int tid = threadIdx.x; // Thread ID within this single block (blockDim.x == num_reduce_blocks)

    // Load block results into shared memory
    if (tid < num_reduce_blocks) {
        shared_idx[tid] = block_bests_idx[tid];
        shared_f[tid] = block_bests_f[tid];
    }
    else {
        shared_idx[tid] = -1;
        shared_f[tid] = FLT_MAX;
    }
    __syncthreads();

    // Reduction across block results
    for (int s = blockDim.x / 2; s > 0; s >>= 1) {
        if (tid < s) {
            int other_idx = shared_idx[tid + s];
            float other_f = shared_f[tid + s];
            if (other_idx != -1 && (other_f < shared_f[tid] || shared_idx[tid] == -1)) {
                shared_idx[tid] = other_idx;
                shared_f[tid] = other_f;
            }
        }
        __syncthreads();
    }

    // Thread 0 determines and stores the global best node index
    __shared__ int current_best_idx_shared;
    __shared__ bool is_goal_shared; // Flag if the best node is the goal
    is_goal_shared = false; // Initialize

    if (tid == 0) {
        current_best_idx_shared = shared_idx[0];
        *d_global_best_idx = current_best_idx_shared; // Write to global output

        if (current_best_idx_shared == -1) {
            atomicExch(d_no_open_nodes_flag, 1); // Signal that no open nodes remain
        }
        else {
            // Mark the chosen node as 'closed' (visited = 2)
            // This is potentially racy if multiple iterations run without sync,
            // but standard A* assumes one expansion per iteration.
            nodes[current_best_idx_shared].visited = 2;
            // Check if the goal was selected
            if (current_best_idx_shared == goal_idx) {
                atomicExch(d_path_found_flag, 1);
                is_goal_shared = true;
            }
        }
    }
    __syncthreads(); // Ensure all threads see the global best index and goal status

    // --- Phase 2: Expand Neighbors (Only if a valid node was found and it's not the goal) ---
    int current_idx = current_best_idx_shared;

    if (current_idx == -1 || is_goal_shared) {
        return; // Exit if no node to expand or goal was found
    }

    // Get current node's details (read by all threads, calculated by thread 0 implicitly)
    // Read g-score carefully - ensure thread 0's write is visible
    float current_g = nodes[current_idx].g;
    int current_x, current_y;
    toCoordsGPU(current_idx, width, &current_x, &current_y);

    // Get current elevation (needed once per expansion)
    float world_x_curr = ((float)current_x + 0.5f) * logInfo->resolution;
    float world_y_curr = ((float)current_y + 0.5f) * logInfo->resolution;
    float current_elevation = getElevationAtDevice(world_x_curr, world_y_curr, elevInfo);


    // Expand neighbors: Use first 8 threads (can potentially parallelize more, but keeps simple)
    if (tid < 8) {
        int dir = tid;
        int nx = current_x + D_DX_DEV[dir];
        int ny = current_y + D_DY_DEV[dir];

        // Check bounds
        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
            int neighbor_idx = toIndexGPU(nx, ny, width);

            // Check if neighbor is not already closed
            // Reading visited state might be slightly stale, but acceptable for A* heuristics
            if (nodes[neighbor_idx].visited != 2) {

                // Calculate dynamic edge cost
                float weight = calculateToblerEdgeCost(current_x, current_y, nx, ny, logInfo, elevInfo, current_elevation);

                // If valid cost (not impassable)
                if (weight < FLT_MAX) {
                    float new_g = current_g + weight;

                    // --- Atomically update neighbor if path is better ---
                    // This requires careful handling if multiple threads could update the same neighbor
                    // In this 8-thread expansion, each thread handles a unique neighbor, so direct access *should* be safe.
                    // However, using atomicMinFloat on g is safer against potential future parallelism changes.

                    // Read current g_neighbor for comparison (can be slightly racy without atomic)
                    // float current_g_neighbor = nodes[neighbor_idx].g;
                    // if (new_g < current_g_neighbor) { ... }

                    // Safer approach: Use atomicMinFloat for g, then update others if successful
                    float old_g_neighbor = atomicMinFloat(&nodes[neighbor_idx].g, new_g);

                    if (new_g < old_g_neighbor) {
                        // Our path was better, update parent, f-score, and visited status
                        nodes[neighbor_idx].parent_idx = current_idx;
                        float h_neighbor = calculate_heuristic_gpu(nx, ny, goal_x, goal_y, heuristic_weight);
                        nodes[neighbor_idx].f = new_g + h_neighbor;
                        nodes[neighbor_idx].visited = 1; // Mark as open

                        // Optional: Check if this neighbor is the goal (early exit)
                        // if (neighbor_idx == goal_idx) {
                        //     atomicExch(d_path_found_flag, 1);
                        // }
                    }
                }
            }
        }
    }
}


//===========================================================
// HOST-CALLABLE WRAPPERS for Kernel Launches
//===========================================================
extern "C" cudaError_t launch_initAStarNodes(
    dim3 gridDim, dim3 blockDim, cudaStream_t stream,
    PathNodeGPU* nodes, int width, int height, int start_idx, int goal_x, int goal_y, float heuristic_weight)
{
    initAStarNodesKernel << <gridDim, blockDim, 0, stream >> > (
        nodes, width, height, start_idx, goal_x, goal_y, heuristic_weight);
    return cudaGetLastError();
}

extern "C" cudaError_t launch_findLocalBestAStarNodes(
    dim3 gridDim, dim3 blockDim, size_t sharedMemBytes, cudaStream_t stream,
    PathNodeGPU* nodes, int node_count,
    int* block_bests_idx, float* block_bests_f, int* d_any_open_flag)
{
    findLocalBestAStarNodes << <gridDim, blockDim, sharedMemBytes, stream >> > (
        nodes, node_count, block_bests_idx, block_bests_f, d_any_open_flag);
    return cudaGetLastError();
}

extern "C" cudaError_t launch_findGlobalAndExpandAStar(
    dim3 gridDim, dim3 blockDim, size_t sharedMemBytes, cudaStream_t stream,
    PathNodeGPU* nodes,
    int* block_bests_idx, float* block_bests_f, int num_reduce_blocks,
    int* d_global_best_idx, int* d_path_found_flag, int* d_no_open_nodes_flag,
    const LogicalGridInfoGPU* logInfo, const ElevationGridInfoGPU* elevInfo,
    int width, int height, int goal_idx, int goal_x, int goal_y, float heuristic_weight)
{
    findGlobalAndExpandAStar << <gridDim, blockDim, sharedMemBytes, stream >> > (
        nodes, block_bests_idx, block_bests_f, num_reduce_blocks,
        d_global_best_idx, d_path_found_flag, d_no_open_nodes_flag,
        logInfo, elevInfo, width, height, goal_idx, goal_x, goal_y, heuristic_weight);
    return cudaGetLastError();
}
// File: algoritms heads/HADS_GPU.hpp
#ifndef HADS_GPU_HPP
#define HADS_GPU_HPP

#include "map/MapProcessingCommon.h" // For Grid_V3
#include "map/PathfindingUtils.hpp"  // For GridPoint
#include <vector>
#include <string>

/**
 * @brief Finds the shortest path using a Heuristic-Accelerated Delta-Stepping
 *        algorithm on the GPU.
 *
 * Calculates path cost based on the Tobler function using logical grid data and
 * separate elevation data. Uses a heuristic to prune search space.
 *
 * @param logical_grid The input logical grid (base costs and flags).
 * @param elevation_values Vector containing elevation data (row-major).
 * @param elevation_width Width of the elevation grid.
 * @param elevation_height Height of the elevation grid.
 * @param log_cell_resolution Real-world size of a logical grid cell edge.
 * @param elev_cell_resolution Real-world size of an elevation grid cell edge.
 * @param origin_offset_x Real-world X offset (elevation_origin_x - logical_origin_x).
 * @param origin_offset_y Real-world Y offset (elevation_origin_y - logical_origin_y).
 * @param start Start point in logical grid coordinates.
 * @param end End point in logical grid coordinates.
 * @param delta_param The DELTA value for bucket width (tuning parameter).
 * @param light_edge_threshold_param Threshold separating light/heavy edges (tuning parameter).
 * @param heuristic_radius_cells Radius (in grid cells) around goal for heuristic precomputation.
 * @param prune_factor Heuristic pruning aggressiveness (e.g., 1.1 = prune if h_v > h_u * 1.1).
 * @param heuristic_weight Weight 'W' applied to heuristic calculation (e.g., 1.0).
 * @return std::vector<int> A vector of logical grid indices representing the path,
 *                          or an empty vector if no path is found or an error occurs.
 */
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
    float heuristic_weight
);

#endif // HADS_GPU_HPP
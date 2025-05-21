// File: algoritms heads/AStarGPU.hpp
#ifndef ASTAR_GPU_HPP
#define ASTAR_GPU_HPP

#include "map/MapProcessingCommon.h" // For Grid_V3
#include "map/PathfindingUtils.hpp"  // For GridPoint
#include <vector>
#include <string>

/**
 * @brief Finds the shortest path using a GPU-accelerated A* algorithm.
 *
 * Calculates path cost based on the Tobler function using logical grid data and
 * separate elevation data. Expands one node per iteration.
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
 * @param heuristic_weight Weight 'W' applied to heuristic calculation (e.g., 1.0).
 * @param max_iterations Safety limit for iterations.
 * @return std::vector<int> A vector of logical grid indices representing the path,
 *                          or an empty vector if no path is found or an error occurs.
 */
std::vector<int> findPathGPU_AStar(
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
    float heuristic_weight,
    int max_iterations = -1 // Default: calculate based on grid size
);

#endif // ASTAR_GPU_HPP
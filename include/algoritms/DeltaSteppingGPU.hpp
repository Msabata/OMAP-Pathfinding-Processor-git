#ifndef DELTA_STEPPING_GPU_HPP
#define DELTA_STEPPING_GPU_HPP

#include "map/MapProcessingCommon.h" // For Grid_V3, GridPoint
#include "map/PathfindingUtils.hpp"
#include <vector>
#include <string> // For error messages if needed

// Forward declare device structures defined in .cu file if needed by host,
// but typically host only interacts via pointers passed to kernels.

/**
 * @brief Finds the shortest path using the Delta-Stepping algorithm on the GPU.
 *
 * Calculates path cost based on the Tobler function using logical grid data and
 * separate elevation data.
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
 * @param light_edge_threshold_param The threshold separating light/heavy edges (tuning parameter).
 * @return std::vector<int> A vector of logical grid indices representing the path,
 *                          or an empty vector if no path is found or an error occurs.
 */
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
    float delta_param, // Use float for tuning parameters
    float light_edge_threshold_param
);


#endif // DELTA_STEPPING_GPU_HPP
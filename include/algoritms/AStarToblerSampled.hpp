// File: AStarToblerSampled.hpp
#ifndef ASTAR_TOBLER_SAMPLED_HPP
#define ASTAR_TOBLER_SAMPLED_HPP

#include "map/MapProcessingCommon.h" // For Grid_V3
#include "map/PathfindingUtils.hpp"  // For GridPoint
#include <vector>

namespace Pathfinding {

    // Declaration of the A* function
    std::vector<int> findAStarPath_Tobler_Sampled(
        const mapgeo::Grid_V3& logical_grid, // Use full namespace if needed
        const std::vector<float>& elevation_values,
        int elevation_width,
        int elevation_height,
        float log_cell_resolution,
        float elev_cell_resolution,
        float origin_offset_x,
        float origin_offset_y,
        const GridPoint& start,
        const GridPoint& end,
        int heuristic_type
    );

} // namespace Pathfinding

#endif // ASTAR_TOBLER_SAMPLED_HPP

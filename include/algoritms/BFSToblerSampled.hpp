#pragma once
#ifndef BFS_TOBLER_SAMPLED_HPP
#define BFS_TOBLER_SAMPLED_HPP

#include "map/MapProcessingCommon.h" // For Grid_V3
#include "map/PathfindingUtils.hpp"  // For GridPoint
#include <vector>

namespace Pathfinding {

    // Declaration of the BFS function
    // Note: Elevation/cost related parameters are kept for API consistency with A*,
    // but they are NOT used in the BFS pathfinding logic itself.
    std::vector<int> findBFSPath_Tobler_Sampled(
        const mapgeo::Grid_V3& logical_grid,
        const std::vector<float>& elevation_values, // Unused by BFS logic
        int elevation_width,                      // Unused by BFS logic
        int elevation_height,                     // Unused by BFS logic
        float log_cell_resolution,                // Potentially needed for obstacle checks if they depend on it indirectly, or just for context. Not used for cost.
        float elev_cell_resolution,               // Unused by BFS logic
        float origin_offset_x,                    // Unused by BFS logic
        float origin_offset_y,                    // Unused by BFS logic
        const GridPoint& start,
        const GridPoint& end
    );

} // namespace Pathfinding

#endif // BFS_TOBLER_SAMPLED_HPP
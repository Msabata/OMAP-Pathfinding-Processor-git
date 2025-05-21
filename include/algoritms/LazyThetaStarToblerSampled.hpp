#ifndef LAZY_THETA_STAR_TOBLER_SAMPLED_HPP
#define LAZY_THETA_STAR_TOBLER_SAMPLED_HPP

#include "map/MapProcessingCommon.h" // For Grid_V3
#include "map/PathfindingUtils.hpp"  // For GridPoint
#include <vector>

namespace Pathfinding {

    // Declaration of the Lazy Theta* function
    std::vector<int> findLazyThetaStarPath_Tobler_Sampled(
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
        int heuristic_type // Kept for API consistency
    );

} // namespace Pathfinding

#endif // LAZY_THETA_STAR_TOBLER_SAMPLED_HPP
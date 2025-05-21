// File: DebugUtils.hpp
#ifndef DEBUG_UTILS_HPP
#define DEBUG_UTILS_HPP

// --- Only compile this file's contents if DEBUG or _DEBUG is defined ---
// --- Adjust the condition based on your build system's debug macro ---
#if defined(_DEBUG) || defined(DEBUG)

#include "map/MapProcessingCommon.h" // Provides Grid_V3, GridPoint
#include "map/PathfindingUtils.hpp" // Provides approx_equal_float, toIndex
#include <vector>               // Provides std::vector
#include <cstddef>              // Provides size_t

namespace debugutils { // Namespace for debugging utilities

    /**
     * @brief (DEBUG) Prints a text-based visualization of the grid with background colors
     *        representing terrain and highlighting the path, start, and end points.
     * @param grid The logical grid containing terrain data (costs, flags).
     * @param path_indices Vector of logical grid indices representing the found path.
     * @param start_pt The starting grid point coordinates.
     * @param end_pt The ending grid point coordinates.
     * @param max_rows Max rows of the grid to print to the console.
     * @param max_cols Max columns of the grid to print to the console.
     */
    void debugVisualizeGridWithPath(
        const mapgeo::Grid_V3& grid,
        const std::vector<int>& path_indices,
        const GridPoint& start_pt,
        const GridPoint& end_pt,
        size_t max_rows = 100, // Default value for max rows
        size_t max_cols = 100 // Default value for max columns
    );

    void saveGridDataBinary(
        const std::string& filename_prefix, // e.g., "grid_save"
        const mapgeo::Grid_V3& grid,
        const std::vector<int>& path_indices,
        const GridPoint& start_pt,
        const GridPoint& end_pt);

} // namespace debugutils

#endif // defined(_DEBUG) || defined(DEBUG)

#endif // DEBUG_UTILS_HPP
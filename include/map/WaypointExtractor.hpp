#ifndef WAYPOINT_EXTRACTOR_HPP
#define WAYPOINT_EXTRACTOR_HPP

#include <vector>
#include <string>
#include <optional>
#include "MapProcessingCommon.h" // For mapgeo::GridPoint definition
#include "PathfindingUtils.hpp"

namespace waypoint {

    /**
     * @brief Extracts Start (701), Control (702), and Finish (706) points from an XML file.
     *
     * Parses the specified XML file to find symbol definitions and objects corresponding
     * to ISOM codes 701 (Start), 702 (Control), and 706 (Finish). It validates that
     * exactly one Start and one Finish point exist, and that all waypoint objects
     * represent single coordinate points. Controls (702) are optional but must also be
     * single points if present.
     *
     * The function uses the provided coordinate bounds (typically obtained from a pre-scan
     * of the main map file) and target grid dimensions to normalize the real-world
     * coordinates found in the XML into integer grid coordinates.
     *
     * @param xmlFilePath Path to the XML file containing the waypoint definitions.
     * @param x_min_um Minimum X coordinate bound (micrometers) from the main map scan.
     * @param x_max_um Maximum X coordinate bound (micrometers) from the main map scan.
     * @param y_min_um Minimum Y coordinate bound (micrometers) from the main map scan.
     * @param y_max_um Maximum Y coordinate bound (micrometers) from the main map scan.
     * @param grid_width The target width of the logical grid (for normalization).
     * @param grid_height The target height of the logical grid (for normalization).
     * @return An optional containing a vector of GridPoints {start, control1, ..., controlN, end}
     *         in logical grid coordinates, ordered by their appearance in the source file.
     *         Returns std::nullopt if requirements are not met or critical errors occur
     *         (e.g., file not found, required symbols missing, validation failure).
     */
    std::optional<std::vector<GridPoint>> extractWaypointsFromFile(
        const std::string& xmlFilePath,
        double x_min_um,
        double x_max_um,
        double y_min_um,
        double y_max_um,
        int grid_width,
        int grid_height
    );

} // namespace waypoint

#endif // WAYPOINT_EXTRACTOR_HPP
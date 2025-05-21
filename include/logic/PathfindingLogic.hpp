#pragma once
#ifndef APP_PATHFINDING_LOGIC_HPP
#define APP_PATHFINDING_LOGIC_HPP

#include <string>
#include <vector>
#include <optional>
#include <map>

// --- Forward declare or include necessary types ---
// Option 1: Include common headers (might increase compile time slightly)
#include "map/MapProcessingCommon.h" // Includes GridPoint, ObstacleConfigMap, NormalizationResult
#include "map/MapProcessor.hpp"      // Includes Grid_V3
#include "map/ElevationFetchingCommon.hpp" // Includes ElevationData
#include "map/ElevationFetcherPy.hpp"
#include "logic/BackendInterface.hpp"


// --- The Logic Class ---

namespace app {

    class PathfindingLogic {
    public:
        PathfindingLogic();
        ~PathfindingLogic();

        // The main processing function, taking parameters and returning results
        BackendResult processAndFindPath(const BackendInputParams& params);

    private:
        // Optional: Add private helper methods if needed for clarity
        // e.g., mapscan::ScanResult scanMap(const std::string& mapFile);
        //      std::optional<std::vector<GridPoint>> extractWaypoints(...);
    };

} // namespace app

#endif // APP_PATHFINDING_LOGIC_HPP
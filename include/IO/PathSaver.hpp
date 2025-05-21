// File: PathSaver.hpp
#ifndef PATH_SAVER_HPP
#define PATH_SAVER_HPP

#include "map/MapProcessingCommon.h" // For Grid_V3, NormalizationResult
#include "map/PathfindingUtils.hpp" // For GridPoint (though not directly used, good include context)
#include <string>
#include <vector>
#include <optional>

namespace pathsaver {

    /**
     * @brief Saves a calculated path into a copy of an OCAD-like XML file.
     *
     * This function takes a path represented by logical grid indices, denormalizes
     * the coordinates back to the original internal units (e.g., micrometers) using
     * the provided normalization parameters, finds a specified symbol (by ISOM code)
     * within the XML file, creates a new line object (<object type="2">) using that
     * symbol, populates its <coords> with the denormalized path points, and saves
     * the modified XML structure to the specified output file path.
     *
     * It assumes the output file already exists (e.g., created by copying the source).
     * It attempts to find the first <part>/<objects> structure within the first layer
     * specified in the processor config or the first layer found.
     *
     * @param outputFilePath The full path to the XML file to modify and save.
     *                       This file should ideally be a copy of the original course file.
     * @param pathIndices A vector of logical grid indices representing the path.
     * @param logicalGrid The Grid_V3 object used for pathfinding (needed for width).
     * @param normInfo The NormalizationResult containing min/max/scale used for processing.
     * @param symbolCodeToFind The ISOM code (e.g., "704") of the symbol to use for the path object.
     * @param targetLayerTag Optional: Specify the layer tag (e.g., "course") where the object should be added.
     *                       If empty, it tries the first layer found in the file.
     * @return True if the path was successfully added and the file saved, false otherwise.
     */
    bool savePathToOmap(
        const std::string& outputFilePath,
        const std::vector<int>& pathIndices,
        const mapgeo::Grid_V3& logicalGrid,
        const mapgeo::NormalizationResult& normInfo,
        const std::string& symbolCodeToFind = "704", // Default to "Route" symbol
        const std::string& targetLayerTag = "" // Optional: Target layer name
    );

} // namespace pathsaver

#endif // PATH_SAVER_HPP
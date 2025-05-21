/**
 * @file MapProcessor.hpp
 * @brief Defines the main MapProcessor class responsible for orchestrating
 *        the map data loading, processing, and grid generation pipeline.
 */
#ifndef MAP_PROCESSOR_H
#define MAP_PROCESSOR_H

#include "MapProcessingCommon.h" // Include common types
#include "ParallelProcessorFlags.hpp" // Include needed definitions
#include <string>
#include <vector>
#include <map>
#include <optional> // For potentially returning grid or error state
#include <cmath> // For std::abs in getter

 // Forward declaration for tinyxml2 to avoid including its header here
namespace tinyxml2 { class XMLElement; }

namespace mapgeo {

    /**
     * @brief Configuration struct specifically for the MapProcessor class.
     */
    struct MapProcessorConfig {
        int grid_width = 100;
        int grid_height = 100;
        std::vector<std::string> layers_to_process = { "barrier" };
    };

    struct PointXY { double x = 0.0, y = 0.0; };
    struct BoundsXY {
        double min_x = 0.0, max_x = 0.0, min_y = 0.0, max_y = 0.0;
        bool initialized = false; // Flag to check if any points were processed
    };


    /**
     * @brief Type definition for the map that holds obstacle configuration.
     *        Maps base ISOM symbol codes to their base terrain cost multipliers.
     */
    using ObstacleConfigMap = std::map<std::string, float>;

    /**
     * @class MapProcessor
     * @brief Orchestrates the loading of XML map data, processing of features,
     *        and generation of the final base terrain cost grid.
     */
    class MapProcessor {
    public:
        
        /**
         * @brief Gets the bounds of the raw X/Y coordinates (in micrometers)
         *        read directly from <coords> tags, before normalization or Y-inversion.
         *        Returns std::nullopt if no coordinates were found or loaded.
         */
        std::optional<BoundsXY> getRawFileCoordinateBoundsUM() const {
            return (rawFileBoundsUM_.initialized) ? std::make_optional(rawFileBoundsUM_) : std::nullopt;
        }
        
        /// --- Constructor ---
        explicit MapProcessor(const MapProcessorConfig& config);
         /**
         * @brief Loads map data from the specified XML file.
         *        Parses symbols, objects, coordinates, and georeferencing info.
         * @param xmlFilePath Path to the XML file.
         * @return True if loading was successful, false otherwise.
         */
        bool loadMap(const std::string& xmlFilePath);


        /** @brief Checks if georeferencing tags were found and parsed during loadMap. */
        bool isGeoreferenced() const { return georeferencingFound_; }


        /** @brief Gets the parsed UTM reference point coordinates (x=Easting, y=Northing), if found. */
        std::optional<PointXY> getParsedRefUTM() const { return parsedRefUTM_; }

        /** @brief Gets the parsed Lat/Lon reference point coordinates (x=Lon, y=Lat), if found. */
        std::optional<PointXY> getParsedRefLatLon() const { return parsedRefLatLon_; }
        /**
        * @brief Generates the logical grid based on loaded data and obstacle config.
        *        Should be called *after* a successful loadMap.
        * @param obstacleConfig Map of ISOM codes to terrain cost values.
        * @return An optional containing the generated Grid_V3, or std::nullopt on failure.
        */
        std::optional<Grid_V3> generateGrid(const ObstacleConfigMap& obstacleConfig);

        // --- Getters for Post-Processing Info ---
        /** @brief Gets the full normalization result including resolution. */
        NormalizationResult getNormalizationResult() const { return normParams_; }

        /** @brief Gets the average logical cell resolution, assuming square cells. */
        double getAverageLogicalResolution() const {
            if (!normParams_.valid) return 0.0;
            // Basic check for squareness (adjust tolerance if needed)
            constexpr double tolerance = 1e-5;
            if (std::abs(normParams_.resolution_x - normParams_.resolution_y) < tolerance) {
                return normParams_.resolution_x;
            }
            else {
                // Log warning or just return average
                // std::cerr << "Warning: Logical grid cells are not square ("
                //           << normParams_.resolution_x << " x " << normParams_.resolution_y
                //           << "). Using average resolution for A*.\n";
                return (normParams_.resolution_x + normParams_.resolution_y) * 0.5;
            }
        }


    private:
        // --- Member Variables ---
        MapProcessorConfig config_;
        std::map<std::string, SymbolDefinition> symbolDefinitions_;
        std::vector<IntermediateObjectData> intermediateObjects_;
        NormalizationResult normParams_; // Use the updated struct
        bool mapLoaded_ = false;

        bool georeferencingFound_ = false;
        std::optional<PointXY> parsedRefUTM_;       // Stores {Easting, Northing}
        std::optional<PointXY> parsedRefLatLon_;    // Stores {Longitude, Latitude}
        BoundsXY rawFileBoundsUM_; // Min/Max of raw X/Y read from file (in micrometers)

        // --- Internal Helper Methods ---
        std::vector<Point> parseCoordinates(const char* coord_string) const;
        void parseSymbolDefinitionsInternal(const tinyxml2::XMLElement* mapElement);
        void parseObjectGeometryInternal(const tinyxml2::XMLElement* mapElement);
        bool calculateNormalizationParamsInternal(); // Updates normParams_
        std::vector<FinalFeatureData> prepareFeatureDataInternal(const ObstacleConfigMap& obstacleConfig) const;
        std::vector<PolygonInputData> preparePass1InputInternal(const std::vector<FinalFeatureData>& preparedFeatures) const;
        void applyFeatureSpecificRulesInternal(Grid_V3& grid, const std::vector<FinalFeatureData>& features) const;

        /** @brief Parses the <georeferencing> section of the map element. */
        void parseGeoreferencingInternal(const tinyxml2::XMLElement* mapElement);

        /**
        *@brief Updates the raw coordinate bounds(rawFileBoundsUM_) with a new point.
        * Expected input units are micrometers as read from the file.
        */
        void updateRawBoundsInternal(double x_um, double y_um);
    };

} // namespace mapgeo
#endif // MAP_PROCESSOR_H
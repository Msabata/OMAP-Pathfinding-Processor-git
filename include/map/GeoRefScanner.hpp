// File: GeoRefScanner.hpp
#ifndef GEO_REF_SCANNER_HPP
#define GEO_REF_SCANNER_HPP

#include <string>
#include <vector>
#include <optional>

// Forward declaration for tinyxml2 (avoids including its header here)
namespace tinyxml2 { class XMLElement; }

namespace mapscan { // New namespace for this specific task

    // Simple struct for point data (used for ref points)
    struct PointXY {
        double x = 0.0;
        double y = 0.0;
    };

    // Simple struct for coordinate bounds (raw micrometers)
    struct BoundsXY {
        double min_x = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double min_y = std::numeric_limits<double>::max();
        double max_y = std::numeric_limits<double>::lowest();
        bool initialized = false; // Flag to check if any points were processed
    };

    // Structure to hold the results of the scan
    struct ScanResult {
        bool georeferencingFound = false;
        std::optional<PointXY> refUTM;       // {Easting, Northing}
        std::optional<PointXY> refLatLon;    // {Longitude, Latitude}
        std::optional<BoundsXY> rawBoundsUM; // Micrometer bounds if coordinates found
        std::optional<double> mapScale;
    };

    /**
     * @brief Scans an XML map file to extract georeferencing information and
     *        the raw coordinate bounds (in micrometers).
     *        This performs a quick pre-scan without fully parsing objects.
     * @param xmlFilePath Path to the map XML file.
     * @param layers_to_process List of layer tags (e.g., {"barrier", "vegetation"})
     *                          within which to search for coordinate data for bounds calculation.
     * @return A ScanResult struct containing the found information. Check optional fields.
     */
    ScanResult scanXmlForGeoRefAndBounds(
        const std::string& xmlFilePath,
        const std::vector<std::string>& layers_to_process
    );

} // namespace mapscan

#endif // GEO_REF_SCANNER_HPP
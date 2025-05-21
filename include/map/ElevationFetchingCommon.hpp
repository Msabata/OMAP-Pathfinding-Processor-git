// File: ElevationFetchingCommon.hpp
#ifndef ELEVATION_FETCHING_COMMON_HPP // <<< CHECK THIS NAME IS UNIQUE AND MATCHES
#define ELEVATION_FETCHING_COMMON_HPP // <<< CHECK THIS NAME IS UNIQUE AND MATCHES

#include <vector>
#include <string>
#include <optional>
#include <limits>

// --- Define mapscan structs locally if GeoRefScanner.hpp isn't included ---
// --- It's better if this header has NO dependency on GeoRefScanner.hpp ---
namespace mapscan {
    struct PointXY_Internal { double x = 0.0, y = 0.0; }; // Use internal name
    struct BoundsXY_Internal {
        double min_x = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double min_y = std::numeric_limits<double>::max();
        double max_y = std::numeric_limits<double>::lowest();
        bool initialized = false;
    };
} // namespace mapscan


namespace ElevationFetching {

    // --- Basic Point Types ---
    struct PointUTM { double x = 0.0, y = 0.0; };
    struct PointLatLon { double lat = 0.0, lon = 0.0; };

    // --- Input Data ---
    struct MapGeoRefRawData {
        std::optional<mapscan::PointXY_Internal> refUTM_raw; // Use internal struct
        std::optional<mapscan::PointXY_Internal> refLatLon_raw;
        std::optional<mapscan::BoundsXY_Internal> rawBoundsUM_raw;
    };

    // --- Calculated Properties ---
    struct MapGeoRefCalculated {
        PointUTM refUTM;
        PointLatLon refLatLon;
        struct { double minX = 0.0, maxX = 0.0, minY = 0.0, maxY = 0.0; } offsetBoundsM;
        struct { double minX = 0.0, maxX = 0.0, minY = 0.0, maxY = 0.0; } utmBoundsM;
        PointUTM logicalOriginUTM;
    };

    // --- API Configuration ---
    enum class ApiType { OPEN_METEO, OPEN_ELEVATION, GOOGLE };
    struct ApiConfig {
        ApiType type = ApiType::OPEN_METEO;
        std::string openElevationApiKey;
        std::string googleApiKey;
        std::string cacheDirectory = "./elevation_cache/";
        int batchSize = 100;
    };

    // --- Result Structure ---
    struct ElevationDataResult {
        bool success = false;
        std::string errorMessage;
        std::vector<float> values;
        int width = 0; int height = 0;
        double resolution_m = 0.0;
        double origin_utm_x = 0.0; double origin_utm_y = 0.0;
        ApiType sourceApi = ApiType::OPEN_METEO;
    };

} // namespace ElevationFetching

#endif // ELEVATION_FETCHING_COMMON_HPP // <<< CHECK THIS NAME MATCHES #ifndef
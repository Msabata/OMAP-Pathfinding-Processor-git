// File: ElevationFetcherPy.hpp
#ifndef ELEVATION_FETCHER_PY_HPP
#define ELEVATION_FETCHER_PY_HPP

#include <vector>
#include <string>
#include <optional> // Only needed if ElevationData struct were complex, not strictly needed now
#include <limits>   // Only needed if ElevationData struct were complex, not strictly needed now

namespace ElevationFetcher {

    // --- Result Structure (Matching Python Dict) ---
    // (Assuming this struct is defined correctly here or included from elsewhere)
    struct ElevationData {
        bool success = false;
        std::string errorMessage = "";
        int width = 0;
        int height = 0;
        // Store origin in the projected system (e.g., Krovak)
        double origin_proj_x = 0.0;
        double origin_proj_y = 0.0;
        double resolution_meters = 0.0;
        std::vector<float> values; // Flattened elevation values (row-major)

        // Helper to check if data seems valid after fetch
        bool hasData() const {
            return success && width > 0 && height > 0 && !values.empty() && static_cast<size_t>(width * height) == values.size();
        }
    };

    // --- Helper Struct for Single Projected Point Result ---
    struct ProjectedPointResult {
        bool success = false;
        double x = 0.0; // Projected X
        double y = 0.0; // Projected Y
        std::string error = ""; // Error message if success is false
    };


    // --- Interface Functions ---

    /**
     * @brief Initializes the embedded Python interpreter. MUST be called once before other functions.
     * @return True if initialization was successful, false otherwise.
     */
    bool initializePython();

    /**
     * @brief Finalizes the embedded Python interpreter. MUST be called once before application exit.
     */
    void finalizePython();

    /**
     * @brief Fetches elevation data by calling the embedded Python function.
     * @param pythonModuleName Name of the Python file (without .py), e.g., "elevation_logic".
     * @param pythonFunctionName Name of the function in the module, e.g., "get_elevation_grid".
     * @param known_lat Latitude (WGS84) of the known anchor point.
     * @param known_lon Longitude (WGS84) of the known anchor point.
     * @param known_internal_x Internal X micrometers coordinate of the anchor point (e.g., 0.0).
     * @param known_internal_y Internal Y micrometers coordinate of the anchor point (e.g., 0.0).
     * @param raw_min_x_um Minimum internal X bound micrometers.
     * @param raw_min_y_um Minimum internal Y bound micrometers.
     * @param raw_max_x_um Maximum internal X bound micrometers.
     * @param raw_max_y_um Maximum internal Y bound micrometers.
     * @param map_scale Map scale denominator (e.g., 10000).
     * @param desired_resolution_meters Desired grid resolution in meters.
     * @return ElevationData struct containing results or error state. Check result.success.
     */
    ElevationData fetchElevationDataEmbedded(
        const std::string& pythonModuleName,
        const std::string& pythonFunctionName,
        // Anchor point
        double known_lat, double known_lon,
        double known_internal_x, double known_internal_y,
        // Map definition (Pass bounds as micrometers µm)
        double raw_min_x_um, double raw_min_y_um,
        double raw_max_x_um, double raw_max_y_um,
        double map_scale,
        // Query parameters
        double desired_resolution_meters
    );

    /**
     * @brief Calls Python helper to convert a single Lat/Lon point to the projected CRS (e.g., Krovak).
     * @param pythonModuleName Name of the Python file (without .py).
     * @param pythonFunctionName Name of the Python helper function, e.g., "convert_latlon_to_projected".
     * @param lon Longitude (WGS84).
     * @param lat Latitude (WGS84).
     * @return ProjectedPointResult containing success status and projected x, y coordinates or error.
     */
    ProjectedPointResult convertLatLonToProjectedViaPython(
        const std::string& pythonModuleName,
        const std::string& pythonFunctionName,
        double lon,
        double lat
    );


} // namespace ElevationFetcher

#endif // ELEVATION_FETCHER_PY_HPP
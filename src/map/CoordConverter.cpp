// File: CoordConverter.cpp

#include "map/CoordConverter.hpp"
#include <cmath> // For M_PI, std::cos

#ifndef M_PI // Define M_PI if not defined (e.g., not included by <cmath> by default)
#define M_PI 3.14159265358979323846
#endif

namespace ElevationFetching {

    PointLatLon utmToLatLonApprox(
        PointUTM point_utm,
        PointUTM ref_utm,
        PointLatLon ref_latlon
    ) {
        // Constants (use doubles for precision)
        const double r_earth_km = 6371.0; // Mean radius, adjust if specific ellipsoid needed (GRS80 radius ~6378.137)
        const double pi = M_PI;
        const double deg_to_rad = pi / 180.0;
        const double rad_to_deg = 180.0 / pi;

        // Calculate meter offsets from the reference point
        double dx_meters = point_utm.x - ref_utm.x; // Easting difference
        double dy_meters = point_utm.y - ref_utm.y; // Northing difference

        // Convert meter offsets to approximate degree offsets
        // Latitude offset is relatively constant
        double lat_offset_deg = (dy_meters / 1000.0 / r_earth_km) * rad_to_deg;

        // Longitude offset depends on latitude
        double ref_lat_rad = ref_latlon.lat * deg_to_rad;
        double cos_ref_lat = std::cos(ref_lat_rad);

        // Avoid division by zero near poles (cos(90) or cos(-90) is 0)
        // Also handle cases where cos is very small, leading to large longitude changes
        double lon_offset_deg = 0.0;
        if (std::abs(cos_ref_lat) > 1e-9) { // Check against small epsilon
            lon_offset_deg = (dx_meters / 1000.0 / r_earth_km) * rad_to_deg / cos_ref_lat;
        }
        else {
            // Handle pole case: Longitude change is ill-defined or very large.
            // Depending on requirement, could return error, clamp, or use a different projection near poles.
            // For simplicity here, we assume longitude offset is negligible right at the pole.
            lon_offset_deg = 0.0; // Or potentially set an error flag / throw
            // std::cerr << "Warning: Calculating longitude offset near pole (Lat=" << ref_latlon.lat << "). Result may be inaccurate.\n";
        }


        // Calculate the new approximate Lat/Lon
        PointLatLon result;
        result.lat = ref_latlon.lat + lat_offset_deg;
        result.lon = ref_latlon.lon + lon_offset_deg;

        // Optional: Clamp latitude to valid range [-90, 90]
        result.lat = std::max(-90.0, std::min(90.0, result.lat));

        // Optional: Normalize longitude to [-180, 180] or [0, 360] if needed
        // result.lon = std::fmod(result.lon + 180.0, 360.0) - 180.0; // Example wrap to -180..180

        return result;
    }

} // namespace ElevationFetching
// File: GeoProcessor.cpp

#include "map/GeoProcessor.hpp"
#include <stdexcept> // For potential errors if needed
#include <iostream>  // For error logging

namespace ElevationFetching {

    std::optional<MapGeoRefCalculated> processRawGeoData(
        const MapGeoRefRawData& raw_data
    ) {
        // Validate necessary inputs
        if (!raw_data.refUTM_raw || !raw_data.refLatLon_raw || !raw_data.rawBoundsUM_raw) {
            std::cerr << "Error (GeoProcessor): Missing essential raw georeferencing data (ref points or bounds).\n";
            return std::nullopt;
        }
        if (!raw_data.rawBoundsUM_raw.value().initialized) {
            std::cerr << "Error (GeoProcessor): Raw coordinate bounds were not initialized (no points found?).\n";
            return std::nullopt;
        }


        MapGeoRefCalculated calculated;

        // Store confirmed reference points
        calculated.refUTM.x = raw_data.refUTM_raw.value().x;
        calculated.refUTM.y = raw_data.refUTM_raw.value().y;
        calculated.refLatLon.lon = raw_data.refLatLon_raw.value().x; // Raw stores Lon in x
        calculated.refLatLon.lat = raw_data.refLatLon_raw.value().y; // Raw stores Lat in y

        // Calculate meter offsets from raw bounds (in micrometers)
        // Assuming file Y increases downwards, map Y increases Northwards
        const auto& raw_bounds = raw_data.rawBoundsUM_raw.value();
        calculated.offsetBoundsM.minX = raw_bounds.min_x / 1000000.0;
        calculated.offsetBoundsM.maxX = raw_bounds.max_x / 1000000.0;
        // Apply Y-inversion: File MinY -> Max Offset, File MaxY -> Min Offset
        calculated.offsetBoundsM.minY = raw_bounds.min_y / 1000000.0 * -1.0; // Should be negative if map origin Y > file Y
        calculated.offsetBoundsM.maxY = raw_bounds.max_y / 1000000.0 * -1.0; // Should be positive if map origin Y < file Y
        // Swap if inversion made min > max (depends on coordinate system interpretation)
        if (calculated.offsetBoundsM.minY > calculated.offsetBoundsM.maxY) {
            std::swap(calculated.offsetBoundsM.minY, calculated.offsetBoundsM.maxY);
            // Note: This swap assumes the raw min/max correctly captured the range,
            // and the inversion flipped their roles relative to the ref point.
            // If file Y increases upwards, remove the * -1.0 and the swap. Needs verification!
            std::cout << "Warning (GeoProcessor): Y-offset bounds swapped after inversion. Verify coordinate system interpretation.\n";
        }


        // Calculate absolute UTM bounds
        calculated.utmBoundsM.minX = calculated.refUTM.x + calculated.offsetBoundsM.minX;
        calculated.utmBoundsM.maxX = calculated.refUTM.x + calculated.offsetBoundsM.maxX;
        calculated.utmBoundsM.minY = calculated.refUTM.y + calculated.offsetBoundsM.minY;
        calculated.utmBoundsM.maxY = calculated.refUTM.y + calculated.offsetBoundsM.maxY;

        // Calculate the absolute UTM coordinate of the logical grid's origin (0,0)
        // This corresponds to the minimum coordinate value found in the raw data,
        // converted to a meter offset and added to the reference UTM point.
        // Uses the same offset calculation logic as for utmBoundsM.minX/Y.
        double origin_offset_x_m = raw_bounds.min_x / 1000000.0;
        // Use raw_bounds.min_y for the Northing offset IF Y-coords in file increase *upwards*
        // Use raw_bounds.max_y for the Northing offset IF Y-coords in file increase *downwards* (apply inversion)
        // Let's assume Y increases downwards in file, consistent with above:
        double origin_offset_y_m = raw_bounds.min_y / 1000000.0 * -1.0; // If Y increases downwards

        calculated.logicalOriginUTM.x = calculated.refUTM.x + origin_offset_x_m;
        calculated.logicalOriginUTM.y = calculated.refUTM.y + origin_offset_y_m;


        return calculated;
    }

} // namespace ElevationFetching
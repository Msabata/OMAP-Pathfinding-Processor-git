// File: GeoProcessor.hpp
#ifndef GEO_PROCESSOR_HPP
#define GEO_PROCESSOR_HPP

#include "ElevationFetchingCommon.hpp"
#include <optional>

namespace ElevationFetching {

    /**
     * @brief Processes raw georeferencing data extracted from the map file.
     *        Calculates meter offsets, absolute UTM bounds, and the UTM coordinate
     *        corresponding to the logical grid's origin (0,0).
     * @param raw_data Struct containing the raw reference points and coordinate bounds (µm).
     * @return An optional containing the calculated geospatial properties,
     *         or std::nullopt if essential raw data (refs or bounds) is missing.
     */
    std::optional<MapGeoRefCalculated> processRawGeoData(
        const MapGeoRefRawData& raw_data
    );

} // namespace ElevationFetching

#endif // GEO_PROCESSOR_HPP
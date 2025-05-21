// File: CoordConverter.hpp
#ifndef COORD_CONVERTER_HPP
#define COORD_CONVERTER_HPP

#include "ElevationFetchingCommon.hpp"

namespace ElevationFetching {

    /**
     * @brief Converts UTM coordinates to approximate Lat/Lon using a spherical Earth model.
     *        This is an approximation suitable for small areas away from the poles.
     * @param point_utm The UTM coordinate (Easting, Northing in meters) to convert.
     * @param ref_utm The UTM coordinate of the reference point.
     * @param ref_latlon The Geographic coordinate (Lat, Lon in degrees) of the reference point.
     * @return The approximate geographic coordinate (Lat, Lon in degrees).
     */
    PointLatLon utmToLatLonApprox(
        PointUTM point_utm,
        PointUTM ref_utm,
        PointLatLon ref_latlon
    );

} // namespace ElevationFetching

#endif // COORD_CONVERTER_HPP
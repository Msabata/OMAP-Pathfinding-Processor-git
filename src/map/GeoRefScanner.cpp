// File: GeoRefScanner.cpp

#include "map/GeoRefScanner.hpp"
#include "IO/tinyxml2.h"       // XML parsing library implementation detail

#include <iostream>         // For error logging
#include <sstream>
#include <limits>
#include <vector>
#include <string>
#include <charconv>         // For std::from_chars
#include <string_view>
#include <cmath>            // For std::fabs, std::round

namespace mapscan {

    // --- Internal Helper Functions ---

    // Updates raw bounds structure
    void updateRawBoundsInternal(BoundsXY& bounds, double x_um, double y_um) {
        if (!bounds.initialized) {
            bounds.min_x = x_um; bounds.max_x = x_um;
            bounds.min_y = y_um; bounds.max_y = y_um;
            bounds.initialized = true;
        }
        else {
            bounds.min_x = std::min(bounds.min_x, x_um);
            bounds.max_x = std::max(bounds.max_x, x_um);
            bounds.min_y = std::min(bounds.min_y, y_um);
            bounds.max_y = std::max(bounds.max_y, y_um);
        }
    }

    // Lightweight coordinate parser for bounds calculation
    void parseCoordinatesForBounds(const char* coord_string, BoundsXY& bounds) {
        if (!coord_string) return;
        std::stringstream ss(coord_string);
        std::string segment;
        while (std::getline(ss, segment, ';')) {
            segment.erase(0, segment.find_first_not_of(" \t\n\r\f\v"));
            segment.erase(segment.find_last_not_of(" \t\n\r\f\v") + 1);
            if (segment.empty()) continue;

            std::stringstream segment_ss(segment);
            std::string value_str;
            double x_val, y_val;
            int value_count = 0;

            while (segment_ss >> value_str) {
                double num_val;
                std::string_view sv(value_str);
                auto res = std::from_chars(sv.data(), sv.data() + sv.size(), num_val);
                if (res.ec != std::errc()) {
                    // Optional: Log warning about skipping non-numeric value
                    value_count = 0;
                    break; // Skip rest of segment part
                }

                value_count++;
                if (value_count == 1) {
                    x_val = num_val;
                }
                else if (value_count == 2) {
                    y_val = num_val;
                    updateRawBoundsInternal(bounds, x_val, y_val); // Update bounds
                    // Check for optional flag but don't use it, just consume if present
                    double potential_flag;
                    if (segment_ss >> value_str) {
                        std::string_view sv_flag(value_str);
                        auto res_flag = std::from_chars(sv_flag.data(), sv_flag.data() + sv_flag.size(), potential_flag);
                        if (res_flag.ec != std::errc() || std::fabs(potential_flag - std::round(potential_flag)) >= 1e-9) {
                            // Wasn't a flag, treat as next X
                            value_count = 1;
                            x_val = potential_flag;
                        }
                        else { // Was a flag, reset count
                            value_count = 0;
                        }
                    }
                    else { value_count = 0; } // No third value
                }
            }
        }
    }

    // Parses only the <georeferencing> section
    void parseGeoreferencing(const tinyxml2::XMLElement* mapElement, ScanResult& result) {
        result.georeferencingFound = false; // Reset
        result.refUTM.reset();
        result.refLatLon.reset();

        if (!mapElement) return;
        const tinyxml2::XMLElement* georefElement = mapElement->FirstChildElement("georeferencing");
        if (!georefElement) return; // No tag found

        result.georeferencingFound = true;

        const char* scale_attr_cstr = georefElement->Attribute("scale");
        if (scale_attr_cstr) {
            std::string_view scale_sv(scale_attr_cstr);
            double scale_val;
            auto scale_res = std::from_chars(scale_sv.data(), scale_sv.data() + scale_sv.size(), scale_val);
            if (scale_res.ec == std::errc()) {
                result.mapScale = scale_val; // Store the parsed scale
            }
            else {
                std::cerr << "Warning (GeoRefScanner): Could not parse 'scale' attribute value '" << scale_attr_cstr << "'." << std::endl;
                result.mapScale.reset(); // Ensure it's reset on error
            }
        }
        else {
            std::cerr << "Warning (GeoRefScanner): Missing 'scale' attribute in <georeferencing> tag." << std::endl;
            result.mapScale.reset(); // Ensure it's reset if attribute missing
        }

        // Parse Projected CRS Ref Point
        const tinyxml2::XMLElement* projCrsElement = georefElement->FirstChildElement("projected_crs");
        if (projCrsElement) {
            const tinyxml2::XMLElement* refPointElement = projCrsElement->FirstChildElement("ref_point");
            if (refPointElement) {
                const char* x_attr = refPointElement->Attribute("x");
                const char* y_attr = refPointElement->Attribute("y");
                if (x_attr && y_attr) {
                    PointXY utm_point;
                    std::string_view x_sv(x_attr); std::string_view y_sv(y_attr);
                    auto x_res = std::from_chars(x_sv.data(), x_sv.data() + x_sv.size(), utm_point.x);
                    auto y_res = std::from_chars(y_sv.data(), y_sv.data() + y_sv.size(), utm_point.y);
                    if (x_res.ec == std::errc() && y_res.ec == std::errc()) {
                        result.refUTM = utm_point;
                    }
                    else { /* Log Warning */ }
                }
                else { /* Log Warning */ }
            }
        }

        // Parse Geographic CRS Ref Point
        const tinyxml2::XMLElement* geoCrsElement = georefElement->FirstChildElement("geographic_crs");
        if (geoCrsElement) {
            const tinyxml2::XMLElement* refPointDegElement = geoCrsElement->FirstChildElement("ref_point_deg");
            if (refPointDegElement) {
                const char* lat_attr = refPointDegElement->Attribute("lat");
                const char* lon_attr = refPointDegElement->Attribute("lon");
                if (lat_attr && lon_attr) {
                    PointXY latlon_point;
                    std::string_view lat_sv(lat_attr); std::string_view lon_sv(lon_attr);
                    auto lon_res = std::from_chars(lon_sv.data(), lon_sv.data() + lon_sv.size(), latlon_point.x); // Lon -> x
                    auto lat_res = std::from_chars(lat_sv.data(), lat_sv.data() + lat_sv.size(), latlon_point.y); // Lat -> y
                    if (lon_res.ec == std::errc() && lat_res.ec == std::errc()) {
                        result.refLatLon = latlon_point;
                    }
                    else { /* Log Warning */ }
                }
                else { /* Log Warning */ }
            }
        }
    }

    // Iterates XML structure only to find coordinates for bounds
    void calculateRawBounds(const tinyxml2::XMLElement* mapElement, const std::vector<std::string>& layers_to_process, BoundsXY& bounds) {
        if (!mapElement) return;

        for (const std::string& layer_tag : layers_to_process) {
            const tinyxml2::XMLElement* layer = mapElement->FirstChildElement(layer_tag.c_str()); if (!layer) continue;
            const tinyxml2::XMLElement* objects = nullptr; // Robust finding
            const tinyxml2::XMLElement* direct_objects = layer->FirstChildElement("objects");
            if (direct_objects) {
                objects = direct_objects;
            }
            else {
                const tinyxml2::XMLElement* parts = layer->FirstChildElement("parts");
                if (parts) {
                    const tinyxml2::XMLElement* part = parts->FirstChildElement("part");
                    if (part) { objects = part->FirstChildElement("objects"); }
                }
            }
            if (!objects) continue;

            for (const tinyxml2::XMLElement* obj = objects->FirstChildElement("object"); obj; obj = obj->NextSiblingElement("object")) {
                const tinyxml2::XMLElement* coordsEl = obj->FirstChildElement("coords");
                const char* coords_str = coordsEl ? coordsEl->GetText() : nullptr;
                if (coords_str) {
                    // Call the lightweight parser just for bounds
                    parseCoordinatesForBounds(coords_str, bounds);
                }
            }
        }
    }


    // --- Public Scan Function ---

    ScanResult scanXmlForGeoRefAndBounds(
        const std::string& xmlFilePath,
        const std::vector<std::string>& layers_to_process
    ) {
        ScanResult result; // Initialize result struct

        tinyxml2::XMLDocument doc;
        if (doc.LoadFile(xmlFilePath.c_str()) != tinyxml2::XML_SUCCESS) {
            std::cerr << "Error loading XML for scan: " << xmlFilePath << " - " << doc.ErrorStr() << std::endl;
            return result; // Return default (empty) result on load failure
        }

        tinyxml2::XMLElement* mapElement = doc.FirstChildElement("map");
        if (!mapElement) {
            std::cerr << "Error: No <map> element in XML for scan.\n";
            return result; // Return default result
        }

        // Parse georeferencing info into the result struct
        parseGeoreferencing(mapElement, result);

        // Calculate raw bounds info into a local BoundsXY struct
        BoundsXY localBounds; // Initialized internally with max/lowest values
        calculateRawBounds(mapElement, layers_to_process, localBounds);

        // If bounds were initialized (points found), add to result
        if (localBounds.initialized) {
            result.rawBoundsUM = localBounds;
        }
        else {
            result.rawBoundsUM.reset(); // Ensure optional is empty if no points found
            if (!result.georeferencingFound) {
                // Optionally log if neither georef nor bounds data found
                std::cerr << "Warning: Scan found no georeferencing or coordinate data in specified layers.\n";
            }
            else {
                std::cerr << "Warning: Scan found georeferencing but no coordinate data in specified layers.\n";
            }
        }

        return result;
    }

} // namespace mapscan
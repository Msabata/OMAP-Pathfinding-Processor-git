#include "map/WaypointExtractor.hpp"
#include "IO/tinyxml2.h"             // XML parsing library
#include "map/MapProcessingCommon.h"  // Includes Point, GridPoint, CoordFlags
#include "map/PathfindingUtils.hpp"  // For GridPoint definition
#include <vector>
#include <string>
#include <optional>
#include <map>
#include <sstream>
#include <cmath>     // For std::floor, std::abs
#include <limits>    // For std::numeric_limits
#include <iostream>  // For std::cerr, std::cout, std::endl
#include <algorithm> // For std::max, std::min, std::sort
#include <functional>// For std::function (used in recursive lambda)
#include <stdexcept> // For std::runtime_error
#include <cstring>   // Needed for strcmp
#include <cstdio>    // Needed for printf (for stdout debug)

// --- Optional Debug Flag ---
#define WAYPOINT_EXTRACTOR_DEBUG // Keep enabled for debugging
// #undef WAYPOINT_EXTRACTOR_DEBUG // Comment out or undefine to disable debug prints

#ifdef WAYPOINT_EXTRACTOR_DEBUG
// --- Use printf for stdout debug messages ---
#define DEBUG_PRINT(...) printf("DEBUG (WaypointExtractor): " __VA_ARGS__)
#else
#define DEBUG_PRINT(...) // Define as empty when not debugging
#endif

namespace waypoint {

    // --- Local Helper to Parse Coordinates (Adapted from MapProcessor) ---
    // Parses coordinates from a string, handling space and semicolon separators.
    // Returns a vector of Points (real-world coords + flags).
    std::vector<mapgeo::Point> parseCoordsLocal(const char* coord_string) {
        std::vector<mapgeo::Point> points;
        if (!coord_string) return points;
        std::stringstream ss(coord_string);
        std::string segment;

        // Handle potential mix of semicolon and space separators
        while (std::getline(ss, segment, ';')) {
            // Trim leading/trailing whitespace from segment
            segment.erase(0, segment.find_first_not_of(" \t\n\r\f\v"));
            segment.erase(segment.find_last_not_of(" \t\n\r\f\v") + 1);
            if (segment.empty()) continue;

            std::stringstream segment_ss(segment);
            std::vector<double> values;
            double val;
            // Read all numbers from the segment
            while (segment_ss >> val) {
                values.push_back(val);
            }

            // Process values in chunks of 2 or 3 (X Y or X Y Flag)
            for (size_t i = 0; i < values.size(); ) {
                if (i + 1 < values.size()) { // Have at least X and Y
                    mapgeo::Point p;
                    p.x = values[i];
                    p.y = values[i + 1];
                    p.flag = mapgeo::CoordFlags::NoFlag; // Default flag

                    if (i + 2 < values.size()) { // Check for potential Flag
                        double potential_flag = values[i + 2];
                        // Check if it looks like an integer flag (non-negative)
                        if (std::floor(potential_flag) == potential_flag && potential_flag >= 0 && potential_flag <= std::numeric_limits<int>::max()) {
                            p.flag = static_cast<int>(potential_flag);
                            i += 3; // Consumed X, Y, Flag
                        }
                        else {
                            i += 2; // Consumed X, Y only
                        }
                    }
                    else {
                        i += 2; // Consumed X, Y only
                    }
                    points.push_back(p);
                }
                else {
                    // Lone number at the end? Ignore it.
                    i++;
                }
            }
        }
        return points;
    }


    // --- Main Extraction Function ---
    std::optional<std::vector<GridPoint>> extractWaypointsFromFile(
        const std::string& xmlFilePath,
        double x_min_um, double x_max_um, double y_min_um, double y_max_um,
        int grid_width, int grid_height)
    {
        DEBUG_PRINT("Starting extraction for file: %s\n", xmlFilePath.c_str());
        DEBUG_PRINT("  Bounds: X[%.2f, %.2f] Y[%.2f, %.2f]\n", x_min_um, x_max_um, y_min_um, y_max_um);
        DEBUG_PRINT("  Grid Target: %d x %d\n", grid_width, grid_height);

        // 1. --- Initial Checks ---
        if (grid_width <= 0 || grid_height <= 0) {
            std::cerr << "Error (extractWaypoints): grid_width (" << grid_width
                << ") and grid_height (" << grid_height << ") must be positive." << std::endl;
            return std::nullopt;
        }
        // Check range validity - allow zero range only if grid is 1x1
        if ((x_max_um < x_min_um || y_max_um < y_min_um) ||
            ((x_max_um == x_min_um || y_max_um == y_min_um) && (grid_width > 1 || grid_height > 1))) {
            std::cerr << "Error (extractWaypoints): Invalid coordinate range provided (X: "
                << x_min_um << "-" << x_max_um << ", Y: " << y_min_um << "-" << y_max_um
                << ") for grid size " << grid_width << "x" << grid_height << "." << std::endl;
            if (x_max_um < x_min_um || y_max_um < y_min_um) return std::nullopt; // Definitely an error
            if (grid_width > 1 || grid_height > 1) return std::nullopt; // Zero range only okay for 1x1 grid
        }

        // 2. --- XML Parsing ---
        tinyxml2::XMLDocument doc;
        DEBUG_PRINT("  Loading XML file...\n");
        if (doc.LoadFile(xmlFilePath.c_str()) != tinyxml2::XML_SUCCESS) {
            std::cerr << "Error (extractWaypoints): Failed to load XML file '" << xmlFilePath << "'. Error: " << doc.ErrorStr() << std::endl;
            return std::nullopt;
        }
        // Find <map> element IGNORING namespace (check only name)
        tinyxml2::XMLElement* mapElement = nullptr;
        for (tinyxml2::XMLNode* node = doc.FirstChild(); node; node = node->NextSibling()) {
            if (node->ToElement() && strcmp(node->Value(), "map") == 0) {
                mapElement = node->ToElement();
                break;
            }
        }
        if (!mapElement) {
            std::cerr << "Error (extractWaypoints): No <map> element found in XML '" << xmlFilePath << "'." << std::endl;
            return std::nullopt;
        }
        DEBUG_PRINT("  <map> element found.\n");

        // 3. --- Symbol Definition Parsing ---
        std::string symbol_id_start = "";
        std::string symbol_id_control = "";
        std::string symbol_id_finish = "";
        const tinyxml2::XMLElement* symbolsParent = nullptr;

        DEBUG_PRINT("  Searching for <symbols> element (namespace-agnostic)...\n");

        // Recursive lambda to find first element named "symbols"
        std::function<const tinyxml2::XMLElement* (const tinyxml2::XMLElement*)> findFirstSymbolsElement =
            [&](const tinyxml2::XMLElement* currentElement) -> const tinyxml2::XMLElement* {
            if (!currentElement) return nullptr;
            DEBUG_PRINT("    Checking element: <%s>\n", currentElement->Value());
            if (strcmp(currentElement->Value(), "symbols") == 0) {
                DEBUG_PRINT("      Found <symbols>!\n");
                return currentElement;
            }
            // Recurse into children
            for (const tinyxml2::XMLElement* child = currentElement->FirstChildElement(); child; child = child->NextSiblingElement()) {
                const tinyxml2::XMLElement* found = findFirstSymbolsElement(child);
                if (found) return found; // Return immediately if found in subtree
            }
            return nullptr; // Not found in this branch
            };

        symbolsParent = findFirstSymbolsElement(mapElement); // Start search from <map>

        if (!symbolsParent) {
            std::cerr << "Error (extractWaypoints): Could not find the <symbols> element anywhere within the structure of '" << xmlFilePath << "'. Cannot identify waypoint symbols." << std::endl;
            return std::nullopt; // Cannot proceed
        }
        DEBUG_PRINT("  <symbols> element located.\n");

        DEBUG_PRINT("  Parsing symbols...\n");
        int symbol_count = 0;
        // Iterate through children named "symbol"
        for (const tinyxml2::XMLElement* sym = symbolsParent->FirstChildElement(); sym; sym = sym->NextSiblingElement()) {
            if (strcmp(sym->Value(), "symbol") != 0) continue; // Skip non-symbol elements

            symbol_count++;
            const char* id_attr = sym->Attribute("id");
            const char* code_attr = sym->Attribute("code");
            if (id_attr && code_attr) {
                std::string code_str = code_attr;
                DEBUG_PRINT("    Symbol ID: %s, Code: %s\n", id_attr, code_str.c_str());
                if (code_str == "701") { // Look for "701.0"
                    symbol_id_start = id_attr;
                    DEBUG_PRINT("      -> Identified as START symbol.\n");
                }
                else if (code_str == "701.0") { // Look for "701"
                    symbol_id_start = id_attr;
                    DEBUG_PRINT("      -> Identified as START symbol.\n");
                }
                else if (code_str == "702.0") { // Look for "702.0"
                    symbol_id_control = id_attr;
                    DEBUG_PRINT("      -> Identified as CONTROL symbol.\n");
                }
                else if (code_str == "702") { // Look for "702.0"
                    symbol_id_control = id_attr;
                    DEBUG_PRINT("      -> Identified as CONTROL symbol.\n");
                }
                else if (code_str == "706.0") { // Look for "706.0"
                    symbol_id_finish = id_attr;
                    DEBUG_PRINT("      -> Identified as FINISH symbol.\n");
                }
                else if (code_str == "706") { // Look for "706.0"
                    symbol_id_finish = id_attr;
                    DEBUG_PRINT("      -> Identified as FINISH symbol.\n");
                }

            }
            else {
                DEBUG_PRINT("    Symbol element missing 'id' or 'code' attribute, skipping.\n");
            }
        }
        DEBUG_PRINT("  Parsed %d potential symbol definitions.\n", symbol_count);

        if (symbol_id_start.empty()) {
            std::cerr << "Error (extractWaypoints): Symbol definition for Start (701) not found in XML '" << xmlFilePath << "'." << std::endl;
            return std::nullopt;
        }
        if (symbol_id_finish.empty()) {
            std::cerr << "Error (extractWaypoints): Symbol definition for Finish (706) not found in XML '" << xmlFilePath << "'." << std::endl;
            return std::nullopt;
        }
        if (symbol_id_control.empty()) {
            std::cout << "Warning (extractWaypoints): Symbol definition for Control (702) not found. Controls will be ignored if present." << std::endl;
            DEBUG_PRINT("  Control symbol (702) definition missing.\n");
        }
        else {
            DEBUG_PRINT("  Start Symbol ID: %s, Control Symbol ID: %s, Finish Symbol ID: %s\n", symbol_id_start.c_str(), symbol_id_control.c_str(), symbol_id_finish.c_str());
        }


        // 4. --- Object Parsing & Waypoint Identification ---
        std::optional<mapgeo::Point> start_real;
        std::vector<mapgeo::Point> controls_real;
        std::optional<mapgeo::Point> finish_real;
        int object_counter = 0; // To preserve original order
        int waypoints_found = 0; // To count how many waypoints are identified
        int start_found_count = 0; // Specific counter for validation
        int finish_found_count = 0; // Specific counter for validation

        // Store errors found during parsing instead of returning immediately
        std::vector<std::string> parsing_errors;

        DEBUG_PRINT("  Recursively searching for <object> elements (namespace-agnostic)...\n");
        // Recursive lambda modified to use strcmp for element name matching
        std::function<void(const tinyxml2::XMLElement*)> findObjectsRecursive =
            [&](const tinyxml2::XMLElement* element) {
            if (!element) return;

            // Process current element if it's named "object"
            if (strcmp(element->Value(), "object") == 0) {
                object_counter++;
                const char* sym_id_attr = element->Attribute("symbol");
                if (sym_id_attr) {
                    std::string current_sym_id = sym_id_attr;
                    DEBUG_PRINT("    Found object #%d with symbol ID '%s'. Checking coords...\n", object_counter, current_sym_id.c_str());

                    // Find coords element named "coords"
                    const tinyxml2::XMLElement* coordsEl = nullptr;
                    for (const tinyxml2::XMLElement* child = element->FirstChildElement(); child; child = child->NextSiblingElement()) {
                        if (strcmp(child->Value(), "coords") == 0) {
                            coordsEl = child;
                            break;
                        }
                    }

                    const char* coords_str = coordsEl ? coordsEl->GetText() : nullptr;

                    if (coords_str) {
                        std::vector<mapgeo::Point> points = parseCoordsLocal(coords_str);
                        DEBUG_PRINT("      Parsed %zu coordinate points.\n", points.size());

                        // Check based on found symbol ID
                        if (current_sym_id == symbol_id_start) {
                            DEBUG_PRINT("        Matches START symbol ID.\n");
                            start_found_count++; // Increment counter
                            if (points.size() != 1) {
                                parsing_errors.push_back("Start (701) object #" + std::to_string(object_counter) + " is not a single point (found " + std::to_string(points.size()) + ").");
                                DEBUG_PRINT("        ERROR: Not a single point!\n");
                            }
                            else if (start_real.has_value()) {
                                parsing_errors.push_back("Multiple Start (701) objects found (object #" + std::to_string(object_counter) + " is another).");
                                DEBUG_PRINT("        ERROR: Already found a start point!\n");
                            }
                            else {
                                start_real = points[0]; start_real->flag = object_counter; waypoints_found++;
                                DEBUG_PRINT("        -> Assigned as Start Point (X:%.2f, Y:%.2f).\n", start_real->x, start_real->y);
                            }
                        }
                        else if (!symbol_id_control.empty() && current_sym_id == symbol_id_control) {
                            DEBUG_PRINT("        Matches CONTROL symbol ID.\n");
                            if (points.size() != 1) {
                                parsing_errors.push_back("Control (702) object #" + std::to_string(object_counter) + " is not a single point (found " + std::to_string(points.size()) + ").");
                                DEBUG_PRINT("        ERROR: Not a single point!\n");
                            }
                            else {
                                mapgeo::Point control_pt = points[0]; control_pt.flag = object_counter;
                                controls_real.push_back(control_pt); waypoints_found++;
                                DEBUG_PRINT("        -> Added as Control Point #%zu (X:%.2f, Y:%.2f).\n", controls_real.size(), control_pt.x, control_pt.y);
                            }
                        }
                        else if (current_sym_id == symbol_id_finish) {
                            DEBUG_PRINT("        Matches FINISH symbol ID.\n");
                            finish_found_count++; // Increment counter
                            if (points.size() != 1) {
                                parsing_errors.push_back("Finish (706) object #" + std::to_string(object_counter) + " is not a single point (found " + std::to_string(points.size()) + ").");
                                DEBUG_PRINT("        ERROR: Not a single point!\n");
                            }
                            else if (finish_real.has_value()) {
                                parsing_errors.push_back("Multiple Finish (706) objects found (object #" + std::to_string(object_counter) + " is another).");
                                DEBUG_PRINT("        ERROR: Already found a finish point!\n");
                            }
                            else {
                                finish_real = points[0]; finish_real->flag = object_counter; waypoints_found++;
                                DEBUG_PRINT("        -> Assigned as Finish Point (X:%.2f, Y:%.2f).\n", finish_real->x, finish_real->y);
                            }
                        }
                        else {
                            DEBUG_PRINT("        Symbol ID does not match Start/Control/Finish.\n");
                        }
                    }
                    else {
                        DEBUG_PRINT("      Object has no <coords> element, skipping coordinate check.\n");
                        // Only warn if it was a potential waypoint symbol
                        if (current_sym_id == symbol_id_start || current_sym_id == symbol_id_finish || (!symbol_id_control.empty() && current_sym_id == symbol_id_control)) {
                            // Using std::cerr for actual warnings, std::cout for info
                            std::cerr << "Warning (extractWaypoints): Object #" << object_counter << " with matching waypoint symbol ID '" << current_sym_id << "' has no <coords> element." << std::endl;
                        }
                    }
                }
                else {
                    DEBUG_PRINT("    Found object #%d without symbol ID, ignoring.\n", object_counter);
                }
            }

            // Recurse
            for (const tinyxml2::XMLElement* child = element->FirstChildElement(); child; child = child->NextSiblingElement()) {
                findObjectsRecursive(child);
            }
            };

        // Execute recursive search
        findObjectsRecursive(mapElement);

        DEBUG_PRINT("  Finished object search. Found %d relevant waypoint objects (Start: %d, Finish: %d, Controls: %zu).\n",
            waypoints_found, start_found_count, finish_found_count, controls_real.size());

        // Report collected parsing errors
        if (!parsing_errors.empty()) {
            std::cerr << "Error (extractWaypoints): Validation errors encountered during object parsing:" << std::endl; // Use cerr for errors
            for (const auto& err : parsing_errors) {
                std::cerr << "  - " << err << std::endl; // Use cerr for errors
            }
            return std::nullopt; // Fail if any validation errors occurred
        }

        // 5. --- Final Validation (with specific error messages to stderr) ---
        DEBUG_PRINT("  Performing final validation...\n");
        bool validation_ok = true;
        if (start_found_count == 0) {
            std::cerr << "Error (extractWaypoints): Start object (Symbol 701, ID " << symbol_id_start << ") was not found." << std::endl; // Use cerr for errors
            validation_ok = false;
        }
        else if (start_found_count > 1) {
            std::cerr << "Error (extractWaypoints): Multiple Start objects (Symbol 701, ID " << symbol_id_start << ") were found (" << start_found_count << ")." << std::endl; // Use cerr for errors
            validation_ok = false;
        }

        if (finish_found_count == 0) {
            std::cerr << "Error (extractWaypoints): Finish object (Symbol 706, ID " << symbol_id_finish << ") was not found." << std::endl; // Use cerr for errors
            validation_ok = false;
        }
        else if (finish_found_count > 1) {
            std::cerr << "Error (extractWaypoints): Multiple Finish objects (Symbol 706, ID " << symbol_id_finish << ") were found (" << finish_found_count << ")." << std::endl; // Use cerr for errors
            validation_ok = false;
        }

        if (!validation_ok) {
            return std::nullopt; // Return nullopt if any core validation failed
        }
        DEBUG_PRINT("  Final validation passed.\n");


        // 6. --- Sort Controls by Original Order ---
        if (!controls_real.empty()) {
            DEBUG_PRINT("  Sorting %zu control points by original order...\n", controls_real.size());
            std::sort(controls_real.begin(), controls_real.end(), [](const mapgeo::Point& a, const mapgeo::Point& b) {
                return a.flag < b.flag; // Sort using stored object_counter
                });
        }

        // 7. --- Normalization Parameter Calculation ---
        DEBUG_PRINT("  Calculating normalization parameters...\n");
        const double norm_offset_x = x_min_um;
        const double norm_offset_y = y_min_um;
        const double range_x = x_max_um - x_min_um;
        const double range_y = y_max_um - y_min_um;
        const double epsilon = 1e-9;

        const double scale_x = (std::abs(range_x) < epsilon || grid_width <= 1)
            ? 1.0
            : (static_cast<double>(grid_width - 1) / range_x);
        const double scale_y = (std::abs(range_y) < epsilon || grid_height <= 1)
            ? 1.0
            : (static_cast<double>(grid_height - 1) / range_y);
        DEBUG_PRINT("    Offset: X=%.2f, Y=%.2f\n", norm_offset_x, norm_offset_y);
        DEBUG_PRINT("    Scale: X=%.6f, Y=%.6f\n", scale_x, scale_y);


        // 8. --- Coordinate Conversion ---
        DEBUG_PRINT("  Converting real coordinates to grid coordinates...\n");
        std::vector<GridPoint> waypoints_gp;
        waypoints_gp.reserve(2 + controls_real.size());

        auto convert_to_grid = [&](const mapgeo::Point& real_point, const std::string& label) -> GridPoint {
            float nx = static_cast<float>((real_point.x - norm_offset_x) * scale_x);
            float ny = static_cast<float>((real_point.y - norm_offset_y) * scale_y);
            int ix = static_cast<int>(std::floor(nx));
            int iy = static_cast<int>(std::floor(ny));
            int max_x_idx = grid_width - 1;
            int max_y_idx = grid_height - 1;
            int clamped_ix = std::max(0, std::min(ix, max_x_idx));
            int clamped_iy = std::max(0, std::min(iy, max_y_idx));
            DEBUG_PRINT("    %s (%.2f, %.2f) -> Norm (%.3f, %.3f) -> Grid (%d, %d) -> Clamped (%d, %d)\n",
                label.c_str(), real_point.x, real_point.y, nx, ny, ix, iy, clamped_ix, clamped_iy);
            return { clamped_ix, clamped_iy };
            };

        waypoints_gp.push_back(convert_to_grid(start_real.value(), "Start"));
        int control_idx = 1;
        for (const auto& control_pt : controls_real) {
            waypoints_gp.push_back(convert_to_grid(control_pt, "Control " + std::to_string(control_idx++)));
        }
        waypoints_gp.push_back(convert_to_grid(finish_real.value(), "Finish"));

        // 9. --- Return ---
        // Use std::cout for final informational message
        std::cout << "Info (extractWaypoints): Successfully extracted " << waypoints_gp.size() << " waypoints from '" << xmlFilePath << "'." << std::endl;
        DEBUG_PRINT("Extraction successful.\n");
        return waypoints_gp;
    }

} // namespace waypoint
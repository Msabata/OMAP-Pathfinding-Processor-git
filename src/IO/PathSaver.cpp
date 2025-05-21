// File: PathSaver.cpp (Corrected Includes and Scope)
#include "IO/PathSaver.hpp"
#include "IO/tinyxml2.h"
#include "map/MapProcessingCommon.h"
#include "map/PathfindingUtils.hpp"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <limits>
#include <iostream>
#include <vector>
#include <set>       // <<< NEEDED INCLUDE FOR std::set
#include <map>       // <<< NEEDED INCLUDE FOR std::map
#include <string>    // <<< NEEDED INCLUDE FOR std::string operations

namespace pathsaver {

    // Helper: Check Collinearity (unchanged)
    inline bool areGridPointsCollinear(const mapgeo::IntPoint& p1, const mapgeo::IntPoint& p2, const mapgeo::IntPoint& p3) {
        long long cross_product = static_cast<long long>(p2.x - p1.x) * (p3.y - p1.y) -
            static_cast<long long>(p2.y - p1.y) * (p3.x - p1.x);
        return cross_product == 0;
    }

    // Helper: Find First Element (unchanged)
    tinyxml2::XMLElement* findFirstElement(tinyxml2::XMLNode* parent, const char* name) {
        if (!parent) return nullptr;
        tinyxml2::XMLElement* elem = parent->FirstChildElement(name);
        if (!elem) {
            for (tinyxml2::XMLNode* node = parent->FirstChild(); node; node = node->NextSibling()) {
                elem = findFirstElement(node, name);
                if (elem) break;
            }
        }
        return elem;
    }

    bool savePathToOmap(
        const std::string& outputFilePath,
        const std::vector<int>& pathIndices,
        const mapgeo::Grid_V3& logicalGrid,
        const mapgeo::NormalizationResult& normInfo,
        const std::string& pathSymbolCode,  // e.g., "704.0"
        const std::string& targetLayerTag   // Effectively ignored, insertion point found via waypoints
    ) {
        // --- Initial Checks ---
        if (pathIndices.empty()) {
            std::cerr << "Error (savePathToOmap): Cannot save an empty path." << std::endl;
            return false;
        }
        if (pathIndices.size() == 1) {
            std::cerr << "Warning (savePathToOmap): Path has only one point. Saving it." << std::endl;
            // Allow saving single point path
        }
        if (!logicalGrid.isValid()) {
            std::cerr << "Error (savePathToOmap): Logical grid is invalid." << std::endl;
            return false;
        }
        if (!normInfo.valid) {
            std::cerr << "Error (savePathToOmap): Normalization parameters are invalid." << std::endl;
            return false;
        }
        const double scale_x = normInfo.scale_x;
        const double scale_y = normInfo.scale_y;
        const double scale_epsilon = 1e-9;
        if (std::abs(scale_x) < scale_epsilon || std::abs(scale_y) < scale_epsilon) {
            std::cerr << "Error (savePathToOmap): Invalid scale factors (near zero) in normalization info." << std::endl;
            return false;
        }
        // --- End Initial Checks ---

        // 1. Load the XML document
        tinyxml2::XMLDocument doc;
        if (doc.LoadFile(outputFilePath.c_str()) != tinyxml2::XML_SUCCESS) {
            std::cerr << "Error (savePathToOmap): Failed to load XML file: " << outputFilePath << " - " << doc.ErrorStr() << std::endl;
            return false;
        }

        tinyxml2::XMLElement* mapElement = doc.FirstChildElement("map");
        if (!mapElement) {
            std::cerr << "Error (savePathToOmap): No <map> element found in " << outputFilePath << std::endl;
            return false;
        }

        // --- Intermediate Step: Build map from Symbol Code to Symbol ID ---
        std::map<std::string, std::string> codeToIdMap;
        tinyxml2::XMLElement* symbolsContainer = findFirstElement(mapElement, "symbols"); // Search broadly first
        if (!symbolsContainer) { // Try searching within layers if not found at top level
            for (tinyxml2::XMLElement* layer = mapElement->FirstChildElement(); layer; layer = layer->NextSiblingElement()) {
                symbolsContainer = layer->FirstChildElement("symbols");
                if (symbolsContainer) break;
            }
        }

        if (!symbolsContainer) {
            std::cerr << "Error (savePathToOmap): Could not find <symbols> container element." << std::endl;
            return false;
        }

        for (tinyxml2::XMLElement* sym = symbolsContainer->FirstChildElement("symbol"); sym; sym = sym->NextSiblingElement("symbol")) {
            const char* codeAttr = sym->Attribute("code");
            const char* idAttr = sym->Attribute("id");
            if (codeAttr && idAttr) {
                codeToIdMap[codeAttr] = idAttr; // Map code (e.g., "704.0") to ID (e.g., "31")
            }
        }
        // --- End Code-to-ID Map Build ---

        // 2. Find the Symbol ID for the PATH symbol code
        std::string pathSymbolId;
        auto pathSymIt = codeToIdMap.find(pathSymbolCode);
        if (pathSymIt != codeToIdMap.end()) {
            pathSymbolId = pathSymIt->second;
        }

        if (pathSymbolId.empty()) {
            std::cerr << "Error (savePathToOmap): Path symbol with code '" << pathSymbolCode << "' not found in symbol definitions." << std::endl;
            return false; // Cannot proceed without the path symbol
        }
        std::cout << "  Found path symbol ID '" << pathSymbolId << "' for code '" << pathSymbolCode << "'." << std::endl;

        // 3. Find the insertion point (<objects>) by finding a known waypoint object
        tinyxml2::XMLElement* objectsElement = nullptr;
        const std::set<std::string> waypointCodes = { "701.0", "702.0", "706.0" }; // Codes for Start, Control, Finish
        std::set<std::string> waypointSymbolIds;
        for (const auto& code : waypointCodes) {
            auto it = codeToIdMap.find(code);
            if (it != codeToIdMap.end()) {
                waypointSymbolIds.insert(it->second);
            }
        }

        if (waypointSymbolIds.empty()) {
            std::cerr << "Error (savePathToOmap): Could not find IDs for any waypoint symbols (701.0, 702.0, 706.0)." << std::endl;
            return false; // Cannot determine where to insert
        }

        // Iterate through all objects in the map to find one using a waypoint symbol ID
        bool insertionPointFound = false;
        for (tinyxml2::XMLElement* layer = mapElement->FirstChildElement(); layer && !insertionPointFound; layer = layer->NextSiblingElement()) {
            tinyxml2::XMLElement* parts = layer->FirstChildElement("parts");
            if (!parts) continue;
            for (tinyxml2::XMLElement* part = parts->FirstChildElement("part"); part && !insertionPointFound; part = part->NextSiblingElement("part")) {
                tinyxml2::XMLElement* objects = part->FirstChildElement("objects");
                if (!objects) continue;
                for (tinyxml2::XMLElement* obj = objects->FirstChildElement("object"); obj && !insertionPointFound; obj = obj->NextSiblingElement("object")) {
                    const char* symRef = obj->Attribute("symbol");
                    if (symRef && waypointSymbolIds.count(std::string(symRef))) { // Convert char* to string for set::count
                        // Found an object using a waypoint symbol. Use its parent <objects> element.
                        objectsElement = objects; // The parent <objects> of the found waypoint object
                        if (layer->Name()) {
                            std::cout << "  Found waypoint object (Symbol ID: " << symRef << ") in layer '" << layer->Name() << "'. Using its <objects> container." << std::endl;
                        }
                        else {
                            std::cout << "  Found waypoint object (Symbol ID: " << symRef << ") in unnamed layer. Using its <objects> container." << std::endl;
                        }
                        insertionPointFound = true;
                        // break; // No need to break inner loops, outer loops check !insertionPointFound
                    }
                }
            }
        }

        if (!objectsElement) {
            std::cerr << "Error (savePathToOmap): Could not find any existing waypoint objects (using symbols "
                << [&]() -> std::string { // Lambda to format set contents
                std::string s = "{";
                bool first = true;
                for (const auto& id : waypointSymbolIds) {
                    if (!first) s += ",";
                    s += id;
                    first = false;
                }
                return s + "}";
                }() // Immediately call the lambda
                    << ") to determine insertion point." << std::endl;
                return false;
        }
        // --- End Finding Insertion Point ---

        // --- 4. Denormalize Coordinates with Path Simplification & Semicolon Formatting & Count ---
        std::stringstream coordStream;
        coordStream << std::fixed << std::setprecision(0); // Integer precision

        const double min_x_norm = normInfo.min_x; // Use distinct names to avoid shadowing
        const double min_y_norm = normInfo.min_y;
        // scale_x, scale_y already defined earlier
        const int gridWidth = static_cast<int>(logicalGrid.width());
        int last_added_path_index = -1; // Index within pathIndices

        bool first_point_added = false; // Track if we've added the first point
        int points_added_count = 0;     // Counter for points added
        size_t last_simplified_point_original_index = std::string::npos;
        for (size_t i = 0; i < pathIndices.size(); ++i) {
            bool add_current_point = false;
            bool is_last_original_point = (i == pathIndices.size() - 1); // Check if it's the very last point in the full path

            // Simplification Logic: Add first, last, and corner points
            if (i == 0 || is_last_original_point) { // Always add first and last original points
                add_current_point = true;
            }
            else {
                if (last_added_path_index >= 0) {
                    int p_last_idx = pathIndices[last_added_path_index];
                    int p_curr_idx = pathIndices[i];
                    int p_next_idx = pathIndices[i + 1];
                    mapgeo::IntPoint p_last_grid, p_curr_grid, p_next_grid;
                    PathfindingUtils::toCoords(p_last_idx, gridWidth, p_last_grid.x, p_last_grid.y);
                    PathfindingUtils::toCoords(p_curr_idx, gridWidth, p_curr_grid.x, p_curr_grid.y);
                    PathfindingUtils::toCoords(p_next_idx, gridWidth, p_next_grid.x, p_next_grid.y);
                    if (!areGridPointsCollinear(p_last_grid, p_curr_grid, p_next_grid)) {
                        add_current_point = true;
                    }
                }
                else {
                    add_current_point = true; // Should be the second point if first was added
                }
            }
            // --- End Simplification Logic ---

            if (add_current_point) {
                int index_to_add = pathIndices[i];
                int gx, gy;
                PathfindingUtils::toCoords(index_to_add, gridWidth, gx, gy);

                double norm_x = static_cast<double>(gx) + 0.5;
                double norm_y = static_cast<double>(gy) + 0.5;
                double real_x = (norm_x / scale_x) + min_x_norm;
                double real_y = (norm_y / scale_y) + min_y_norm;

                long long final_x = static_cast<long long>(std::round(real_x));
                long long final_y = static_cast<long long>(std::round(real_y));

                if (first_point_added) {
                    coordStream << ";"; // Semicolon separator
                }
                coordStream << final_x << " " << final_y; // Add "X Y" pair

                // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
                // ADD THE FLAG '16' IF THIS IS THE VERY LAST POINT BEING ADDED
                // We know it's the last point if it corresponds to the last index
                // of the *original* pathIndices vector.
                if (is_last_original_point) {
                    coordStream << " 16;"; // Append the flag with a preceding space
                    std::cout << "  (Appended flag 16 to the final coordinate)" << std::endl;
                }
                // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

                first_point_added = true;
                last_added_path_index = static_cast<int>(i);
                points_added_count++;
                last_simplified_point_original_index = i; // Keep track of the original index
            }
        } // End loop through pathIndices

        std::string coordsString = coordStream.str();
        // --- End Coordinate Denormalization and Simplification ---


        // --- 5. Create New Object/Coords/Pattern Elements (Corrected Order and Type) ---
        tinyxml2::XMLElement* newObject = doc.NewElement("object");
        if (!newObject) {
            std::cerr << "Error (savePathToOmap): Failed to create new <object> element." << std::endl;
            return false;
        }
        newObject->SetAttribute("type", "1"); // Set TYPE TO 1 (Area) as requested
        newObject->SetAttribute("symbol", pathSymbolId.c_str());

        // ---- Create and add <coords> FIRST ----
        tinyxml2::XMLElement* newCoords = doc.NewElement("coords");
        if (!newCoords) {
            std::cerr << "Error (savePathToOmap): Failed to create new <coords> element." << std::endl;
            return false;
        }
        newCoords->SetAttribute("count", points_added_count); // Set COUNT attribute
        newCoords->SetText(coordsString.c_str());             // Set coordinate string
        newObject->InsertEndChild(newCoords);                 // Add <coords> to <object>

        // ---- Create and add <pattern> SECOND ----
        tinyxml2::XMLElement* newPattern = doc.NewElement("pattern");
        if (!newPattern) {
            std::cerr << "Error (savePathToOmap): Failed to create new <pattern> element." << std::endl;
            return false;
        }
        newPattern->SetAttribute("rotation", "0");

        tinyxml2::XMLElement* patternCoord = doc.NewElement("coord");
        if (!patternCoord) {
            std::cerr << "Error (savePathToOmap): Failed to create new <coord> element for pattern." << std::endl;
            return false;
        }
        patternCoord->SetAttribute("x", "0");
        patternCoord->SetAttribute("y", "0");

        newPattern->InsertEndChild(patternCoord);     // Add <coord> to <pattern>
        newObject->InsertEndChild(newPattern);        // Add <pattern> to <object> AFTER coords
        // --- End Revised Object Creation ---


        // 6. Insert New Object into the found <objects> element
        objectsElement->InsertEndChild(newObject);
        std::cout << "  New simplified path object (type 1, with count and pattern) created and added." << std::endl;


        // 7. Save Modified Document
        if (doc.SaveFile(outputFilePath.c_str()) != tinyxml2::XML_SUCCESS) {
            std::cerr << "Error (savePathToOmap): Failed to save modified XML file: " << outputFilePath << " - " << doc.ErrorStr() << std::endl;
            return false;
        }

        std::cout << "  Successfully saved simplified path (semicolon separated, type 1) to " << outputFilePath << std::endl;
        return true;
    }

} // namespace pathsaver
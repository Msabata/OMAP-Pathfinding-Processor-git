// File: DebugUtils.cpp
// --- IMPORTANT: Configure build system for DEBUG ONLY ---

#if defined(_DEBUG) || defined(DEBUG)
#define _CRT_SECURE_NO_WARNINGS

#include "debug/DebugUtils.hpp"
#include "map/MapProcessingCommon.h"
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <unordered_set>
#include <cstdio>

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <unordered_set>
#include <cstdio>
#include <fstream>  // For file output
#include <chrono>   // For timestamp
#include <iomanip>  // For formatting timestamp
#include <sstream>  // For building filename
#include <cstdint>  // For fixed-width integers like int32_t
#include "IO/json.hpp"
namespace debugutils {

    // --- ANSI color codes and Box Drawing characters ---
    namespace {
        // Colors (same as before)
        const std::string BG_RESET = "\033[0m"; /* ... FG_WHITE, FG_BLACK, etc. ... */
        const std::string FG_WHITE = "\033[97m";
        const std::string FG_BLACK = "\033[30m";
        const std::string BG_BLACK = "\033[40m";
        const std::string BG_DARK_GREY = "\033[100m";
        const std::string BG_WHITE = "\033[107m";
        const std::string BG_BLUE = "\033[44m";
        const std::string BG_CYAN = "\033[46m";
        const std::string BG_MAGENTA = "\033[45m";
        const std::string BG_BRIGHT_YELLOW = "\033[103m";
        const std::string BG_YELLOW = "\033[43m";
        const std::string BG_BRIGHT_GREEN = "\033[102m";
        const std::string BG_GREEN = "\033[42m";
        const std::string BG_RED = "\033[41m";
        const std::string BG_PATH = "\033[105m";
        const std::string BG_START_END = "\033[101m";

        // Box Drawing Characters (same as before)
        const std::string BOX_HL = "\u2500"; const std::string BOX_VL = "\u2502";
        const std::string BOX_TL = "\u250C"; const std::string BOX_TR = "\u2510";
        const std::string BOX_BL = "\u2514"; const std::string BOX_BR = "\u2518";
    }

    // --- Visualization Function DEFINITION ---
    void debugVisualizeGridWithPath(
        const mapgeo::Grid_V3& grid,
        const std::vector<int>& path_indices,
        const GridPoint& start_pt,
        const GridPoint& end_pt,
        size_t max_rows,
        size_t max_cols)
    {
        if (!grid.isValid()) { /* ... */ return; }

        size_t print_height = std::min(grid.height(), max_rows);
        // Use max_cols for the number of *grid cells* horizontally,
        // the actual character width will be double this.
        size_t print_width_cells = std::min(grid.width(), max_cols);
        int grid_width_int = static_cast<int>(grid.width());

        if (print_height == 0 || print_width_cells == 0) { /* ... */ return; }

        std::unordered_set<int> path_set(path_indices.begin(), path_indices.end());
        int start_index = -1, end_index = -1;
        if (grid.inBounds(start_pt.x, start_pt.y)) start_index = PathfindingUtils::toIndex(start_pt.x, start_pt.y, grid_width_int);
        if (grid.inBounds(end_pt.x, end_pt.y)) end_index = PathfindingUtils::toIndex(end_pt.x, end_pt.y, grid_width_int);

        // --- Print Header and Legend ---
        std::cout << "\n--- (DEBUG) Grid Visualization ---" << std::endl;
        // (Legend print statement - maybe adjust wording slightly if needed)
        std::cout << "Legend: S/E=" << BG_START_END << " " << BG_RESET /* ... etc ... */ << std::endl;


        if (grid.width() > max_cols || grid.height() > max_rows) {
            // Warning refers to cell limits, not character limits
            std::cout << "Warning: Grid larger than display limit (" << max_cols << "x" << max_rows << " cells). Truncating output.\n";
        }

        // --- Print Top Border (Double Width) ---
        std::cout << BOX_TL;
        // Loop print_width_cells * 2 times for the horizontal lines
        for (size_t i = 0; i < print_width_cells * 2; ++i) {
            std::cout << BOX_HL;
        }
        std::cout << BOX_TR << std::endl;

        // --- Print Grid Rows ---
        for (size_t y = 0; y < print_height; ++y) {
            std::cout << BOX_VL; // Left border

            // Loop through the number of cells horizontally
            for (size_t x = 0; x < print_width_cells; ++x) {
                // --- Cell coloring logic (identical to previous version) ---
                int current_index = PathfindingUtils::toIndex(static_cast<int>(x), static_cast<int>(y), grid_width_int);
                if (x >= grid.width() || y >= grid.height()) {
                    std::cout << "  "; // Print two spaces if out of bounds
                    continue;
                }
                const auto& cell = grid.at(x, y);
                std::string bgColor = BG_WHITE; std::string fgColor = FG_BLACK; std::string displayChar = " ";

                if (current_index != -1 && current_index == start_index) {
                    bgColor = BG_START_END; displayChar = "S"; fgColor = FG_WHITE;
                }
                else if (current_index != -1 && current_index == end_index) {
                    bgColor = BG_START_END; displayChar = "E"; fgColor = FG_WHITE;
                }
                else if (path_set.count(current_index)) {
                    bgColor = BG_PATH; displayChar = "*"; fgColor = FG_WHITE;
                }
                else {
                    // Terrain coloring logic (same as before) ...
                    bool is_road = cell.hasFlag(mapgeo::GridFlags::FLAG_ROAD_PATH); if (is_road) displayChar = "#";
                    if (cell.hasFlag(mapgeo::GridFlags::FLAG_IMPASSABLE) || cell.value < 0.0f) { bgColor = BG_DARK_GREY; displayChar = "X"; fgColor = FG_WHITE; }
                    else if (is_road) { bgColor = BG_BLACK; fgColor = FG_WHITE; }
                    else if (cell.hasFlag(mapgeo::GridFlags::FLAG_WATER_MARSH)) { bgColor = BG_CYAN; fgColor = FG_BLACK; }
                    else if (cell.hasFlag(mapgeo::GridFlags::FLAG_UNDERGROWTH)) { bgColor = BG_GREEN; fgColor = FG_BLACK; }
                    else if (!mapgeo::approx_equal_float(cell.value, 1.0f)) {
                        if (cell.value >= 10.0f)
                            bgColor = BG_RED;
                        else if (cell.value >= 5.0f)
                            bgColor = BG_GREEN;
                        else if (cell.value >= 2.0f)
                            bgColor = BG_BLUE;
                        else if (cell.value >= 1.67f) bgColor = BG_BRIGHT_GREEN;
                        else if (cell.value >= 1.43f) bgColor = BG_BRIGHT_GREEN; else if (cell.value >= 1.25f) bgColor = BG_YELLOW; else if (cell.value >= 1.11f) bgColor = BG_BRIGHT_YELLOW; else if (cell.value > 1.0f) bgColor = BG_WHITE; else bgColor = BG_RED; // Faster RED
                        if (bgColor == BG_WHITE || bgColor == BG_BRIGHT_YELLOW || bgColor == BG_YELLOW || bgColor == BG_BRIGHT_GREEN || bgColor == BG_CYAN) fgColor = FG_BLACK; else fgColor = FG_WHITE;
                    }
                    else { bgColor = BG_WHITE; fgColor = FG_BLACK; }
                }

                // *** Print TWO characters for better aspect ratio ***
                // Option 1: Print two spaces (emphasizes background color)
                std::cout << bgColor << fgColor << "  " << BG_RESET;

                // Option 2: Print the display character twice (if you want S, E, *, # visible)
                // std::string doubleChar = displayChar + displayChar; // Create a two-char string
                // std::cout << bgColor << fgColor << doubleChar << BG_RESET;

                // Option 3: Print character then space (compromise)
                // std::cout << bgColor << fgColor << displayChar << " " << BG_RESET;

                // --- End cell coloring logic ---
            }

            std::cout << BOX_VL << std::endl; // Right border
        }

        // --- Print Bottom Border (Double Width) ---
        std::cout << BOX_BL;
        // Loop print_width_cells * 2 times
        for (size_t i = 0; i < print_width_cells * 2; ++i) {
            std::cout << BOX_HL;
        }
        std::cout << BOX_BR << std::endl;

        // --- Print Footer ---
        if (print_height < grid.height() || print_width_cells < grid.width()) {
            // Report truncation based on cell dimensions
            std::cout << "(Output truncated to content area " << print_height << "x" << print_width_cells << " cells)" << std::endl;
        }
        std::cout << "--- End (DEBUG) Grid Visualization ---" << std::endl;
        //saveGridDataBinary("grid_debug_data", grid, path_indices, start_pt, end_pt);
    }
    void saveGridDataBinary(
        const std::string& filename_prefix,
        const mapgeo::Grid_V3& grid,
        const std::vector<int>& path_indices,
        const GridPoint& start_pt,
        const GridPoint& end_pt)
    {
        if (!grid.isValid()) {
            std::cerr << "Error: Cannot save invalid grid." << std::endl;
            return;
        }

        // 1. Generate timestamp and base filename (same as before)
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        std::tm now_tm = *std::localtime(&now_c);
        std::stringstream ss_timestamp;
        ss_timestamp << std::put_time(&now_tm, "%Y%m%d_%H%M%S");
        std::string timestamp_str = ss_timestamp.str();
        std::string base_filename = filename_prefix + "_" + timestamp_str;

        std::string meta_filename = base_filename + "_meta.json";
        std::string data_filename = base_filename + "_data.bin";

        // 2. Save Metadata to JSON file (using json::JSON)
        json::JSON metadata = json::Object(); // Create an empty JSON object

        // Create the nested "metadata" object
        metadata["metadata"] = json::Object(); // Assign an empty object first
        metadata["metadata"]["timestamp_str"] = timestamp_str; // Assign string

        // Format ISO timestamp string
        std::stringstream ss_iso_time;
        ss_iso_time << std::put_time(&now_tm, "%Y-%m-%dT%H:%M:%SZ");
        metadata["metadata"]["timestamp_iso"] = ss_iso_time.str(); // Assign string

        metadata["metadata"]["grid_width"] = static_cast<long>(grid.width()); // Use long for Integral type
        metadata["metadata"]["grid_height"] = static_cast<long>(grid.height()); // Use long for Integral type
        metadata["metadata"]["binary_format"] = "float32_value_int32_flags_row_major"; // Assign string

        // Handle Start Point
        if (grid.inBounds(start_pt.x, start_pt.y)) {
            json::JSON start_point_obj = json::Object();
            start_point_obj["x"] = static_cast<long>(start_pt.x);
            start_point_obj["y"] = static_cast<long>(start_pt.y);
            metadata["metadata"]["start_point"] = start_point_obj; // Assign the object
        }
        else {
            metadata["metadata"]["start_point"] = nullptr; // Assign null
        }

        // Handle End Point
        if (grid.inBounds(end_pt.x, end_pt.y)) {
            json::JSON end_point_obj = json::Object();
            end_point_obj["x"] = static_cast<long>(end_pt.x);
            end_point_obj["y"] = static_cast<long>(end_pt.y);
            metadata["metadata"]["end_point"] = end_point_obj; // Assign the object
        }
        else {
            metadata["metadata"]["end_point"] = nullptr; // Assign null
        }

        // --- Store Path Indices in Metadata ---
        // Need to manually create a JSON array
        json::JSON path_array = json::Array();
        for (int index : path_indices) {
            // The library uses long for integrals, safer to cast if 'int' isn't exactly 'long'
            path_array.append(static_cast<long>(index));
        }
        metadata["path_indices"] = path_array; // Assign the created array object

        // --- Store Flag Definitions in Metadata (CRUCIAL) ---
        // Create the nested "flag_definitions" object
        metadata["flag_definitions"] = json::Object();
        // *** Replace with ACTUAL integer values from mapgeo::GridFlags ***
        // Use long for the Integral type in this JSON library
        metadata["flag_definitions"]["FLAG_IMPASSABLE"] = static_cast<long>(mapgeo::GridFlags::FLAG_IMPASSABLE);
        metadata["flag_definitions"]["FLAG_ROAD_PATH"] = static_cast<long>(mapgeo::GridFlags::FLAG_ROAD_PATH);
        metadata["flag_definitions"]["FLAG_WATER_MARSH"] = static_cast<long>(mapgeo::GridFlags::FLAG_WATER_MARSH);
        metadata["flag_definitions"]["FLAG_UNDERGROWTH"] = static_cast<long>(mapgeo::GridFlags::FLAG_UNDERGROWTH);
        // Add ALL other flags used in your logic

        // --- Write the JSON object to the file ---
        try {
            std::ofstream meta_outfile(meta_filename);
            if (!meta_outfile.is_open()) {
                std::cerr << "Error: Could not open metadata file for writing: " << meta_filename << std::endl;
                return; // Don't proceed if metadata can't be saved
            }
            // Use the stream operator provided by the library.
            // It calls dump() internally, which should provide formatting.
            meta_outfile << metadata << std::endl;

            meta_outfile.close(); // Good practice to close explicitly
            if (!meta_outfile) { // Check stream state after close/flush
                std::cerr << "Error: Failed to write all data to metadata file: " << meta_filename << std::endl;
                // Consider deleting the potentially corrupt file?
                return;
            }
            std::cout << "Debug grid metadata saved to: " << meta_filename << std::endl;
        }
        // Note: Your simple JSON library doesn't seem to throw standard exceptions
        // on invalid operations during construction, it might print errors to cerr.
        // File stream errors are handled via stream state checks.
        catch (const std::exception& e) { // Catch potential file system errors etc.
            std::cerr << "Error during metadata file write operation: " << e.what() << std::endl;
            return; // Don't proceed
        }
        catch (...) {
            std::cerr << "Unknown error during metadata file write operation." << std::endl;
            return; // Don't proceed
        }


        // 3. Save Grid Data to Binary file (This part is UNCHANGED)
        std::ofstream data_outfile(data_filename, std::ios::binary | std::ios::out);
        if (!data_outfile.is_open()) {
            std::cerr << "Error: Could not open binary data file for writing: " << data_filename << std::endl;
            // Clean up the already created metadata file? Optional.
            // std::remove(meta_filename.c_str());
            return;
        }

        try {
            size_t width = grid.width();
            size_t height = grid.height();

            for (size_t y = 0; y < height; ++y) {
                for (size_t x = 0; x < width; ++x) {
                    const auto& cell = grid.at(x, y);
                    float value_f32 = cell.value;
                    int32_t flags_i32 = static_cast<int32_t>(cell.flags);

                    data_outfile.write(reinterpret_cast<const char*>(&value_f32), sizeof(value_f32));
                    data_outfile.write(reinterpret_cast<const char*>(&flags_i32), sizeof(flags_i32));

                    if (!data_outfile) {
                        std::cerr << "Error: Failed to write binary data at cell ("
                            << x << "," << y << ") to " << data_filename << std::endl;
                        data_outfile.close();
                        // Clean up metadata file? Optional.
                        // std::remove(meta_filename.c_str());
                        return;
                    }
                }
            }
            data_outfile.close();
            if (!data_outfile) {
                std::cerr << "Error: Failed to write all data to binary file: " << data_filename << std::endl;
                // Clean up metadata file? Optional.
                // std::remove(meta_filename.c_str());
                return;
            }
            std::cout << "Debug grid binary data saved to: " << data_filename << std::endl;

        }
        catch (const std::exception& e) {
            std::cerr << "Error writing binary data file: " << e.what() << std::endl;
            // Ensure file is closed even if exception happens mid-write
        }
        catch (...) {
            std::cerr << "Unknown error writing binary data file." << std::endl;
        }
    }

} // namespace debugutils

#endif // defined(_DEBUG) || defined(DEBUG)
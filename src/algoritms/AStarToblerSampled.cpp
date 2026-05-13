
// [ Code for AStarToblerSampled.cpp as provided previously ]
// File: AStarToblerSampled.cpp

#include "algoritms/AStarToblerSampled.hpp"// Include the header declaring the function
#include "map/MapProcessingCommon.h"  // For Grid_V3, GridCellData, FLAG_IMPASSABLE
#include "map/PathfindingUtils.hpp"   // For constants, heuristic, GridPoint, etc.
#include "map/ElevationSampler.hpp"   // For the ElevationSampler class

#include <vector>
#include <queue>
#include <limits>
#include <cmath>
#include <stdexcept>
#include <iostream> // For optional debug/error output
#include <string>   // For error messages

// Use namespaces if desired
using namespace mapgeo;
using namespace PathfindingUtils;

namespace Pathfinding {

    std::vector<int> findAStarPath_Tobler_Sampled(
        const Grid_V3& logical_grid,
        const std::vector<float>& elevation_values,
        int elevation_width,
        int elevation_height,
        float log_cell_resolution,
        float elev_cell_resolution,
        float origin_offset_x,
        float origin_offset_y,
        const GridPoint& start,
        const GridPoint& end,
        int heuristic_type
    ) {
        const int log_width = static_cast<int>(logical_grid.width());
        const int log_height = static_cast<int>(logical_grid.height());
        const int log_size = log_width * log_height;
        std::vector<int> resultPath;

        // --- Input Validation ---
        if (!logical_grid.isValid() || log_cell_resolution <= EPSILON) {
            // std::cerr << "A* Error: Invalid logical grid or resolution.\n";
            return resultPath;
        }

        // Validate and create elevation sampler (constructor throws on error)
         // Wrap in try-catch if constructor exceptions need graceful handling here
        ElevationSampler elevation_sampler(
            elevation_values,
            elevation_width,
            elevation_height,
            elev_cell_resolution,
            origin_offset_x,
            origin_offset_y
        );


        if (!logical_grid.inBounds(start.x, start.y) || !logical_grid.inBounds(end.x, end.y)) {
            // std::cerr << "A* Error: Start/End out of logical bounds.\n";
            return resultPath;
        }

        const int startIdx = toIndex(start.x, start.y, log_width);
        const int endIdx = toIndex(end.x, end.y, log_width);

        // Obstacle Check Start/End
        const GridCellData& startCell = logical_grid.at(start.x, start.y);
        if (startCell.value <= 0.0f || startCell.hasFlag(GridFlags::FLAG_IMPASSABLE)) { return resultPath; }
        const GridCellData& endCell = logical_grid.at(end.x, end.y);
        if (endCell.value <= 0.0f || endCell.hasFlag(GridFlags::FLAG_IMPASSABLE)) { return resultPath; }

        if (startIdx == endIdx) { resultPath.push_back(startIdx); return resultPath; }

        // --- A* Data Structures ---
        static thread_local std::vector<float> g_scores;
        static thread_local std::vector<float> f_scores;
        static thread_local std::vector<bool> closed;
        static thread_local std::vector<int> parents;
        static thread_local std::vector<float> cell_elevation;
        try {
            g_scores.assign(log_size, std::numeric_limits<float>::max());
            f_scores.assign(log_size, std::numeric_limits<float>::max());
            closed.assign(log_size, false);
            parents.assign(log_size, -1);
            cell_elevation.resize(static_cast<size_t>(log_size));
        }
        catch (const std::bad_alloc&) { return resultPath; }

        // Precompute elevation for every cell centre once to avoid repeated sampling.
        for (int cy = 0; cy < log_height; ++cy) {
            for (int cx = 0; cx < log_width; ++cx) {
                float wx = (static_cast<float>(cx) + 0.5f) * log_cell_resolution;
                float wy = (static_cast<float>(cy) + 0.5f) * log_cell_resolution;
                cell_elevation[toIndex(cx, cy, log_width)] = elevation_sampler.getElevationAt(wx, wy);
            }
        }

        // --- Priority Queue: store (f_score, node_index) pairs to avoid stale comparator captures ---
        using PQEntry = std::pair<float, int>;
        std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> openQueue;

        // --- Initialization ---
        g_scores[startIdx] = 0.0f;
        float h_start = calculate_heuristic(start.x, start.y, end.x, end.y, heuristic_type);
        f_scores[startIdx] = h_start;
        openQueue.push({h_start, startIdx});

        // --- A* Main Loop ---
        while (!openQueue.empty()) {
            auto [current_f, currentIdx] = openQueue.top();
            openQueue.pop();

            if (currentIdx == endIdx) { break; }
            if (closed[currentIdx]) { continue; } // stale entry
            closed[currentIdx] = true;

            int x, y;
            toCoords(currentIdx, log_width, x, y);
            const float current_g = g_scores[currentIdx];

            // Lookup precomputed elevation for current cell.
            float current_elevation = cell_elevation[currentIdx];

            // --- Explore Neighbors ---
            for (int dir = 0; dir < NUM_DIRECTIONS; ++dir) {
                const int nx = x + dx[dir];
                const int ny = y + dy[dir];

                if (nx < 0 || nx >= log_width || ny < 0 || ny >= log_height) { continue; } // Logical bounds

                const int neighborIdx = toIndex(nx, ny, log_width);
                const GridCellData& neighborCell = logical_grid.at(nx, ny);

                // Obstacle Check
                if (neighborCell.value <= 0.0f || neighborCell.hasFlag(GridFlags::FLAG_IMPASSABLE)) { continue; }

                // --- Cost Calculation via shared Tobler function ---
                float neighbor_elevation = cell_elevation[neighborIdx];
                float delta_h = neighbor_elevation - current_elevation;
                float final_move_cost = toblerEdgeCost(dir, log_cell_resolution, delta_h, neighborCell.value);
                if (final_move_cost >= std::numeric_limits<float>::max()) { continue; }

                // --- Update Neighbor ---
                float tentative_g = current_g + final_move_cost;

                if (tentative_g < g_scores[neighborIdx]) {
                    parents[neighborIdx] = currentIdx;
                    g_scores[neighborIdx] = tentative_g;
                    float new_f = tentative_g + calculate_heuristic(nx, ny, end.x, end.y, heuristic_type);
                    f_scores[neighborIdx] = new_f;
                    openQueue.push({new_f, neighborIdx});
                }
            } // End neighbor loop
        } // End while openQueue not empty

        // --- Path Reconstruction ---
        if (parents[endIdx] == -1 && startIdx != endIdx) { return resultPath; }
        std::vector<int> path_reversed;
        int current = endIdx;
        size_t safety_count = 0;
        const size_t max_path_len = static_cast<size_t>(log_size) + 1;
        while (current != -1 && safety_count < max_path_len) {
            path_reversed.push_back(current);
            if (current == startIdx) break;
            current = parents[current];
            safety_count++;
        }
        if (current != startIdx && startIdx != endIdx) return std::vector<int>(); // Failed
        if (safety_count >= max_path_len) return std::vector<int>();           // Failed (cycle?)

        resultPath.assign(path_reversed.rbegin(), path_reversed.rend());
        return resultPath;
    }

}// namespace Pathfinding
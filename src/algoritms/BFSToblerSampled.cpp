#pragma once
#include "algoritms/BFSToblerSampled.hpp"   // Include the header declaring the function
#include "map/MapProcessingCommon.h"
#include "map/PathfindingUtils.hpp"
// #include "map/ElevationSampler.hpp" // Not needed for BFS logic

#include <vector>
#include <queue>    // Use std::queue instead of std::priority_queue
#include <limits>
#include <cmath>
#include <stdexcept>
#include <iostream>
#include <string>

using namespace mapgeo;
using namespace PathfindingUtils;

namespace Pathfinding {

    std::vector<int> findBFSPath_Tobler_Sampled(
        const Grid_V3& logical_grid,
        const std::vector<float>& elevation_values, // Unused parameter
        int elevation_width,                      // Unused parameter
        int elevation_height,                     // Unused parameter
        float log_cell_resolution,                // Unused parameter (unless needed for complex obstacle logic)
        float elev_cell_resolution,               // Unused parameter
        float origin_offset_x,                    // Unused parameter
        float origin_offset_y,                    // Unused parameter
        const GridPoint& start,
        const GridPoint& end
    ) {
        const int log_width = static_cast<int>(logical_grid.width());
        const int log_height = static_cast<int>(logical_grid.height());
        const int log_size = log_width * log_height;
        std::vector<int> resultPath;

        // --- Input Validation (Same as A*) ---
        if (!logical_grid.isValid()) { // Resolution not strictly needed for BFS core logic
            return resultPath;
        }

        // Elevation Sampler is NOT created or used for BFS pathfinding logic
        // ElevationSampler elevation_sampler(...); // Removed

        if (!logical_grid.inBounds(start.x, start.y) || !logical_grid.inBounds(end.x, end.y)) {
            return resultPath;
        }

        const int startIdx = toIndex(start.x, start.y, log_width);
        const int endIdx = toIndex(end.x, end.y, log_width);

        const GridCellData& startCell = logical_grid.at(start.x, start.y);
        if (startCell.value <= 0.0f || startCell.hasFlag(GridFlags::FLAG_IMPASSABLE)) { return resultPath; }
        const GridCellData& endCell = logical_grid.at(end.x, end.y);
        if (endCell.value <= 0.0f || endCell.hasFlag(GridFlags::FLAG_IMPASSABLE)) { return resultPath; }


        if (startIdx == endIdx) { resultPath.push_back(startIdx); return resultPath; }

        // --- BFS Data Structures ---
        // No costs (g_scores, f_scores) needed
        static thread_local std::vector<bool> visited; // Replaces 'closed' set
        static thread_local std::vector<int> parents;  // Path reconstruction
        try {
            visited.assign(log_size, false);
            parents.assign(log_size, -1);
        }
        catch (const std::bad_alloc&) { return resultPath; }

        // --- Queue (FIFO) ---
        std::queue<int> openQueue; // Standard queue for BFS

        // --- Initialization ---
        visited[startIdx] = true; // Mark start as visited
        openQueue.push(startIdx); // Enqueue start node

        // --- BFS Main Loop ---
        bool found = false;
        while (!openQueue.empty()) {
            const int currentIdx = openQueue.front(); // Get front element
            openQueue.pop();                         // Remove from front

            if (currentIdx == endIdx) {
                found = true;
                break; // Goal found
            }

            int x, y;
            toCoords(currentIdx, log_width, x, y);

            // --- Explore Neighbors ---
            for (int dir = 0; dir < NUM_DIRECTIONS; ++dir) {
                const int nx = x + dx[dir];
                const int ny = y + dy[dir];

                if (nx < 0 || nx >= log_width || ny < 0 || ny >= log_height) { continue; } // Bounds check

                const int neighborIdx = toIndex(nx, ny, log_width);

                // Check if visited *before* checking grid obstacles for efficiency
                if (visited[neighborIdx]) { continue; }

                const GridCellData& neighborCell = logical_grid.at(nx, ny);

                // Obstacle Check (only based on logical grid)
                if (neighborCell.value <= 0.0f || neighborCell.hasFlag(GridFlags::FLAG_IMPASSABLE)) {
                    continue;
                }

                // --- Process Unvisited, Valid Neighbor ---
                // No cost calculation needed for BFS

                visited[neighborIdx] = true;      // Mark as visited
                parents[neighborIdx] = currentIdx; // Set parent for path reconstruction
                openQueue.push(neighborIdx);      // Enqueue the neighbor
            } // End neighbor loop
        } // End while openQueue not empty

        // --- Path Reconstruction (Same as A*) ---
        if (!found) { return resultPath; } // If loop finished without finding endIdx

        // Path reconstruction logic remains identical using the parents array
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
        // Check if reconstruction succeeded
        if (current != startIdx && startIdx != endIdx) return std::vector<int>(); // Should not happen if found=true, but good practice
        if (safety_count >= max_path_len) return std::vector<int>();           // Cycle?

        resultPath.assign(path_reversed.rbegin(), path_reversed.rend());
        return resultPath;
    }

}// namespace Pathfinding
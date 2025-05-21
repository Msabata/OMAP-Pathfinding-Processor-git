#include "algoritms/ThetaStarToblerSampled.hpp" // Include the header
#include "map/MapProcessingCommon.h"
#include "map/PathfindingUtils.hpp"
#include "map/ElevationSampler.hpp"

#include <vector>
#include <queue>
#include <limits>
#include <cmath>
#include <stdexcept>
#include <iostream> // For optional debug/error output
#include <string>   // For error messages
#include <functional> // For std::function if needed, or lambda captures
#include <cstdlib>    // For abs(int)

// Use namespaces if desired
using namespace mapgeo;
using namespace PathfindingUtils;

namespace Pathfinding {

    // Forward declarations for helper functions within this file
    namespace { // Use an anonymous namespace for internal linkage

        // Calculates Euclidean distance heuristic, scaled for admissibility
        float calculate_theta_heuristic(
            int x1, int y1, int x2, int y2,
            float resolution, float min_terrain_cost_factor)
        {
            float dx = static_cast<float>(x2 - x1);
            float dy = static_cast<float>(y2 - y1);
            // World distance
            float dist = std::sqrt(dx * dx + dy * dy) * resolution;
            return dist * min_terrain_cost_factor; // Scale by min cost
        }

        // Gets world coordinates for the center of a grid cell
        inline Point_float<float> getWorldCoords(
            int x, int y, float resolution, float offset_x, float offset_y)
        {
            return {
                (static_cast<float>(x) + 0.5f) * resolution + offset_x,
                (static_cast<float>(y) + 0.5f) * resolution + offset_y
            };
        }

        // Calculates Euclidean distance between two world points
        inline float worldDistance(const Point_float<float>& p1, const Point_float<float>& p2) {
            float dx = p2.x - p1.x;
            float dy = p2.y - p1.y;
            return std::sqrt(dx * dx + dy * dy);
        }

        // Checks Line of Sight using Bresenham's algorithm
        // Returns true if LOS is clear, false otherwise.
        // Checks obstacles based on flags and value.



        // Calculates the cost of traversing a straight line segment using Tobler/Terrain cost
        // Sums costs over intersected sub-segments (cell center to cell center).


        std::vector<GridPoint> getLineSegmentCells(int x0, int y0, int x1, int y1) {
            std::vector<GridPoint> line;
            int dx = std::abs(x1 - x0);
            int dy = -std::abs(y1 - y0);
            int sx = (x0 < x1) ? 1 : -1;
            int sy = (y0 < y1) ? 1 : -1;
            int err = dx + dy; // error value e_xy

            int cx = x0;
            int cy = y0;

            while (true) {
                line.push_back({ cx, cy }); // Add current point

                if (cx == x1 && cy == y1) break; // Reached destination

                int e2 = 2 * err;
                bool moved_x = false;
                if (e2 >= dy) { // e_xy+e_x >= 0
                    if (cx == x1) break;
                    err += dy;
                    cx += sx;
                    moved_x = true;
                }
                bool moved_y = false;
                if (e2 <= dx) { // e_xy+e_y <= 0
                    if (cy == y1) break;
                    err += dx;
                    cy += sy;
                    moved_y = true;
                }
                // --- Handling Grid Corner Cases (Optional but can be more robust) ---
                // If moved diagonally and the error allows passing through a shared corner
                // without hitting the diagonal cell, Bresenham might skip cells needed
                // for strict grid connectivity. This check adds adjacent cells if needed.
                // However, for pure interpolation, the standard steps are often sufficient.
                // If path connectivity seems broken around corners, add checks here to
                // potentially add (cx-sx, cy) or (cx, cy-sy) if needed based on error term.
                // For now, we use the standard Bresenham steps.
            }
            return line;
        }


        // --- Line of Sight Check (remains the same) ---
        // Checks Line of Sight using Bresenham's algorithm
        // Returns true if LOS is clear, false otherwise.
        bool hasLineOfSight(
            int x0, int y0, int x1, int y1,
            const Grid_V3& grid, int grid_width, int grid_height)
        {
            // Check endpoints first (caller should ideally ensure they are valid)
            if (!grid.inBounds(x0, y0) || !grid.inBounds(x1, y1)) return false;
            const auto& cell0 = grid.at(x0, y0);
            const auto& cell1 = grid.at(x1, y1);

            int dx = std::abs(x1 - x0);
            int dy = -std::abs(y1 - y0); // Use negative dy for standard Bresenham iteration
            int sx = (x0 < x1) ? 1 : -1;
            int sy = (y0 < y1) ? 1 : -1;
            int err = dx + dy; // error value e_xy

            int cx = x0; // current x
            int cy = y0; // current y

            while (true) {
                // Check the *current* cell (cx, cy) for obstacle, *excluding the start cell* (x0, y0)
                if (cx != x0 || cy != y0) { // Don't check the start cell itself
                    if (!grid.inBounds(cx, cy)) return false; // Hit edge of map during check
                    const auto& cell = grid.at(cx, cy);
                    if (cell.hasFlag(GridFlags::FLAG_IMPASSABLE) || cell.value <= numeric_traits<float>::epsilon) {
                        return false; // Obstacle hit
                    }
                }

                if (cx == x1 && cy == y1) break; // Reached destination

                int e2 = 2 * err;
                if (e2 >= dy) { // e_xy+e_x >= 0
                    if (cx == x1) break; // Should not happen given endpoint check?
                    err += dy;
                    cx += sx;
                }
                if (e2 <= dx) { // e_xy+e_y <= 0
                    if (cy == y1) break;
                    err += dx;
                    cy += sy;
                }
            }
            // If loop completes, LOS is clear
            return true;
        }


        // --- Segment Cost Calculation (remains the same) ---
        float calculateSegmentCost(
            int x_start, int y_start, int x_end, int y_end,
            const Grid_V3& grid, int grid_width, int grid_height,
            const ElevationSampler& elevation_sampler,
            float log_resolution, float origin_x, float origin_y,
            float& infinite_penalty_ref // Use reference for efficiency
        )
        {
            float total_cost = 0.0f;

            // Use the line interpolation function to get the exact cells
            std::vector<GridPoint> segment_cells = getLineSegmentCells(x_start, y_start, x_end, y_end);

            if (segment_cells.size() < 2) { // Need at least start and end
                return 0.0f; // Or handle as error? Cost of staying put is 0.
            }

            // Get elevation for the first point
            Point_float<float> prev_world = getWorldCoords(segment_cells[0].x, segment_cells[0].y, log_resolution, origin_x, origin_y);
            float prev_elev = elevation_sampler.getElevationAt(prev_world.x, prev_world.y);

            // Iterate through the segments defined by the interpolated cells
            for (size_t i = 1; i < segment_cells.size(); ++i) {
                const GridPoint& current_gp = segment_cells[i];
                const GridPoint& prev_gp = segment_cells[i - 1]; // For reference, though coords are in prev_world

                if (!grid.inBounds(current_gp.x, current_gp.y)) return infinite_penalty_ref; // Path goes out of bounds

                const auto& current_cell_data = grid.at(current_gp.x, current_gp.y);
                // Check if the cell we are *entering* is impassable (redundant w/ LOS, but safe)
                if (current_cell_data.hasFlag(GridFlags::FLAG_IMPASSABLE) || current_cell_data.value <= numeric_traits<float>::epsilon) {
                    return infinite_penalty_ref; // Cannot traverse into this cell
                }

                Point_float<float> curr_world = getWorldCoords(current_gp.x, current_gp.y, log_resolution, origin_x, origin_y);
                float current_elev = elevation_sampler.getElevationAt(curr_world.x, curr_world.y);

                float delta_h = current_elev - prev_elev;
                // Calculate distance between the *centers* of the cells for the sub-segment
                float delta_dist_world = worldDistance(prev_world, curr_world);

                // Avoid division by zero if points are coincident
                if (delta_dist_world <= numeric_traits<float>::epsilon) {
                    // No distance moved, cost is 0 for this tiny step
                }
                else {
                    float S = delta_h / delta_dist_world;
                    float SlopeFactor = expf(-3.5f * fabsf(S + 0.05f));
                    float time_penalty = (SlopeFactor > numeric_traits<float>::epsilon) ? (1.0f / SlopeFactor) : infinite_penalty_ref;

                    if (time_penalty >= infinite_penalty_ref) {
                        return infinite_penalty_ref; // Unpassable slope encountered
                    }

                    float terrain_cost = current_cell_data.value; // Cost of the cell being entered
                    float sub_segment_cost = delta_dist_world * time_penalty * terrain_cost;
                    total_cost += sub_segment_cost;
                }

                // Update previous state for the next iteration
                prev_world = curr_world;
                prev_elev = current_elev;
            }

            return total_cost;
        }
    } // end anonymous namespace


    // --- Theta* Implementation ---
    std::vector<int> findThetaStarPath_Tobler_Sampled(
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
        int heuristic_type // Parameter kept for API consistency
    ) {
        const int log_width = static_cast<int>(logical_grid.width());
        const int log_height = static_cast<int>(logical_grid.height());
        const int log_size = log_width * log_height;
        std::vector<int> resultPath;

        // --- Input Validation ---
        if (!logical_grid.isValid() || log_cell_resolution <= EPSILON) {
            return resultPath;
        }
        // Create elevation sampler (constructor throws on error)
        ElevationSampler elevation_sampler(
            elevation_values,
            elevation_width,
            elevation_height,
            elev_cell_resolution,
            origin_offset_x,
            origin_offset_y
        );

        if (!logical_grid.inBounds(start.x, start.y) || !logical_grid.inBounds(end.x, end.y)) {
            return resultPath;
        }

        const int startIdx = toIndex(start.x, start.y, log_width);
        const int endIdx = toIndex(end.x, end.y, log_width);

        const GridCellData& startCell = logical_grid.at(start.x, start.y);
        if (startCell.value <= numeric_traits<float>::epsilon || startCell.hasFlag(GridFlags::FLAG_IMPASSABLE)) { return resultPath; }
        const GridCellData& endCell = logical_grid.at(end.x, end.y);
        if (endCell.value <= numeric_traits<float>::epsilon || endCell.hasFlag(GridFlags::FLAG_IMPASSABLE)) { return resultPath; }

        if (startIdx == endIdx) { resultPath.push_back(startIdx); return resultPath; }

        // --- Theta* Data Structures ---
        static thread_local std::vector<float> g_scores; // Cost from start
        static thread_local std::vector<float> f_scores; // Estimated total cost (g + h)
        static thread_local std::vector<bool> closed;    // Visited/closed set
        static thread_local std::vector<int> parents;   // Path reconstruction
        try {
            g_scores.assign(log_size, std::numeric_limits<float>::max());
            f_scores.assign(log_size, std::numeric_limits<float>::max());
            closed.assign(log_size, false);
            parents.assign(log_size, -1);
        }
        catch (const std::bad_alloc&) { return resultPath; }

        // --- Priority Queue ---
        // Compares f_scores, tie-breaks with g_scores (standard A*)
        auto cmp = [&](int l, int r) {
            if (std::fabs(f_scores[l] - f_scores[r]) > numeric_traits<float>::epsilon) return f_scores[l] > f_scores[r];
            return g_scores[l] > g_scores[r];
            };
        std::priority_queue<int, std::vector<int>, decltype(cmp)> openQueue(cmp);

        // --- Constants ---
        const float infinite_penalty = std::numeric_limits<float>::max();
        const float MIN_TERRAIN_COST_FACTOR = 0.5f; // As confirmed

        // --- Initialization ---
        g_scores[startIdx] = 0.0f;
        // Use the scaled Euclidean heuristic for Theta*
        f_scores[startIdx] = calculate_theta_heuristic(start.x, start.y, end.x, end.y, log_cell_resolution, MIN_TERRAIN_COST_FACTOR);
        openQueue.push(startIdx);

        // --- Theta* Main Loop ---
        while (!openQueue.empty()) {
            const int currentIdx = openQueue.top();
            openQueue.pop();

            if (currentIdx == endIdx) { break; } // Goal reached
            if (closed[currentIdx]) { continue; } // Already processed
            closed[currentIdx] = true;

            int x, y;
            toCoords(currentIdx, log_width, x, y);
            const float current_g = g_scores[currentIdx];
            const int parent_of_currentIdx = parents[currentIdx]; // Get parent for LOS check

            // --- Explore Neighbors ---
            for (int dir = 0; dir < NUM_DIRECTIONS; ++dir) {
                const int nx = x + dx[dir];
                const int ny = y + dy[dir];

                if (nx < 0 || nx >= log_width || ny < 0 || ny >= log_height) { continue; } // Check bounds

                const int neighborIdx = toIndex(nx, ny, log_width);
                if (closed[neighborIdx]) { continue; } // Skip already processed neighbors

                const GridCellData& neighborCell = logical_grid.at(nx, ny);
                // Check if neighbor is traversable (basic check)
                if (neighborCell.value <= numeric_traits<float>::epsilon || neighborCell.hasFlag(GridFlags::FLAG_IMPASSABLE)) {
                    continue;
                }

                float tentative_g;
                int chosen_parentIdx;

                // --- Theta* Logic: Check Line of Sight ---
                if (parent_of_currentIdx != -1) { // If current is not the start node
                    int px, py;
                    toCoords(parent_of_currentIdx, log_width, px, py);

                    // Check LOS from parent_of_current to neighbor
                    if (hasLineOfSight(px, py, nx, ny, logical_grid, log_width, log_height))
                    {
                        // LOS exists: Calculate cost along the straight segment
                        float segment_cost = calculateSegmentCost(
                            px, py, nx, ny,
                            logical_grid, log_width, log_height,
                            elevation_sampler,
                            log_cell_resolution, origin_offset_x, origin_offset_y,
                            const_cast<float&>(infinite_penalty) // Pass ref efficiently
                        );

                        // If segment is passable
                        if (segment_cost < infinite_penalty) {
                            tentative_g = g_scores[parent_of_currentIdx] + segment_cost;
                            chosen_parentIdx = parent_of_currentIdx;

                            // --- Compare with standard A* path cost ---
                            // Calculate cost from current to neighbor (standard A* step)
                            float current_to_neighbor_cost = calculateSegmentCost(
                                x, y, nx, ny, // Just one step
                                logical_grid, log_width, log_height,
                                elevation_sampler,
                                log_cell_resolution, origin_offset_x, origin_offset_y,
                                const_cast<float&>(infinite_penalty)
                            );

                            if (current_to_neighbor_cost < infinite_penalty) {
                                float g_via_current = current_g + current_to_neighbor_cost;
                                // If path via current node is cheaper, switch back to A* behavior for this step
                                if (g_via_current < tentative_g) {
                                    tentative_g = g_via_current;
                                    chosen_parentIdx = currentIdx;
                                }
                            }
                            // else: Path via current is infinitely costly, stick with LOS path if possible.

                        }
                        else {
                            // LOS exists but is impassable (e.g., extreme slope), fall back to standard A* step
                            chosen_parentIdx = currentIdx; // Assume path via current node initially
                            float current_to_neighbor_cost = calculateSegmentCost(
                                x, y, nx, ny, // Just one step cost
                                logical_grid, log_width, log_height,
                                elevation_sampler,
                                log_cell_resolution, origin_offset_x, origin_offset_y,
                                const_cast<float&>(infinite_penalty)
                            );
                            if (current_to_neighbor_cost >= infinite_penalty) continue; // Cannot reach neighbor at all
                            tentative_g = current_g + current_to_neighbor_cost;
                        }
                    }
                    else {
                        // No LOS: Use standard A* step cost calculation
                        chosen_parentIdx = currentIdx; // Path must go through current
                        float current_to_neighbor_cost = calculateSegmentCost(
                            x, y, nx, ny, // Just one step cost
                            logical_grid, log_width, log_height,
                            elevation_sampler,
                            log_cell_resolution, origin_offset_x, origin_offset_y,
                            const_cast<float&>(infinite_penalty)
                        );
                        if (current_to_neighbor_cost >= infinite_penalty) continue; // Cannot reach neighbor
                        tentative_g = current_g + current_to_neighbor_cost;
                    }
                }
                else {
                    // Current node is the start node, must use standard step
                    chosen_parentIdx = currentIdx; // Parent is the start node itself (current)
                    float current_to_neighbor_cost = calculateSegmentCost(
                        x, y, nx, ny, // Just one step cost
                        logical_grid, log_width, log_height,
                        elevation_sampler,
                        log_cell_resolution, origin_offset_x, origin_offset_y,
                        const_cast<float&>(infinite_penalty)
                    );
                    if (current_to_neighbor_cost >= infinite_penalty) continue; // Cannot reach neighbor
                    tentative_g = current_g + current_to_neighbor_cost; // current_g is 0 here
                }


                // --- Update Neighbor if path is better ---
                if (tentative_g < g_scores[neighborIdx]) {
                    parents[neighborIdx] = chosen_parentIdx; // Set parent (could be current or parent_of_current)
                    g_scores[neighborIdx] = tentative_g;
                    // Update f_score using the scaled Euclidean heuristic
                    f_scores[neighborIdx] = tentative_g + calculate_theta_heuristic(
                        nx, ny, end.x, end.y, log_cell_resolution, MIN_TERRAIN_COST_FACTOR);
                    openQueue.push(neighborIdx); // Add/update neighbor in priority queue
                }
            } // End neighbor loop
        } // End while openQueue not empty

        // --- Path Reconstruction (MODIFIED) ---

        if (parents[endIdx] == -1 && startIdx != endIdx) {
            return resultPath; // End not reached
        }

        // 1. Get the sequence of jump-points (waypoints) from end to start
        std::vector<int> waypoints_reversed;
        int current_wp = endIdx;
        size_t safety_count = 0;
        const size_t max_path_len = static_cast<size_t>(log_size) * 2 + 2; // Allow for longer interpolated paths
        while (current_wp != -1 && safety_count < max_path_len) {
            waypoints_reversed.push_back(current_wp);
            if (current_wp == startIdx) break;
            current_wp = parents[current_wp];
            safety_count++;
        }

        // Handle failures during waypoint reconstruction
        if (current_wp != startIdx && startIdx != endIdx) return std::vector<int>(); // Failed
        if (safety_count >= max_path_len) return std::vector<int>();           // Failed (cycle?)

        // 2. Reverse the waypoints to get the sequence from start to end
        std::reverse(waypoints_reversed.begin(), waypoints_reversed.end()); // Now it's [start, wp1, wp2, ..., end]

        // 3. Interpolate between waypoints
        if (!waypoints_reversed.empty()) {
            resultPath.push_back(waypoints_reversed[0]); // Add the start node

            // Iterate through the segments between waypoints
            for (size_t i = 0; i < waypoints_reversed.size() - 1; ++i) {
                int p1_idx = waypoints_reversed[i];
                int p2_idx = waypoints_reversed[i + 1];

                int x1, y1, x2, y2;
                toCoords(p1_idx, log_width, x1, y1);
                toCoords(p2_idx, log_width, x2, y2);

                // Get the cells along the line segment connecting the waypoints
                std::vector<GridPoint> segment_cells = getLineSegmentCells(x1, y1, x2, y2);

                // Add the cells from the segment to the result path, skipping the first cell
                // (which is p1 and already in the path from the previous iteration or initialization)
                for (size_t j = 1; j < segment_cells.size(); ++j) {
                    // Ensure the cell isn't the same as the last one added (can happen with short segments)
                    int cell_idx = toIndex(segment_cells[j].x, segment_cells[j].y, log_width);
                    if (resultPath.empty() || cell_idx != resultPath.back()) {
                        resultPath.push_back(cell_idx);
                    }
                }
            }
        }
        else if (startIdx == endIdx) {
            resultPath.push_back(startIdx); // Handle trivial case
        }


        return resultPath;
    }

}// namespace Pathfinding
#include "algoritms/LazyThetaStarToblerSampled.hpp" // Include the header
#include "map/MapProcessingCommon.h"
#include "map/PathfindingUtils.hpp"
#include "map/ElevationSampler.hpp"

#include <vector>
#include <queue>
#include <limits>
#include <cmath>
#include <stdexcept>
#include <iostream>
#include <string>
#include <functional>
#include <cstdlib>
#include <algorithm> // For std::reverse

// Use namespaces if desired
using namespace mapgeo;
using namespace PathfindingUtils;

namespace Pathfinding {

    // --- Anonymous namespace for internal helper functions ---
    // --- These helpers are identical to the standard Theta* version ---
    namespace {

        // Heuristic calculation (Scaled Euclidean)
        float calculate_theta_heuristic(
            int x1, int y1, int x2, int y2,
            float resolution, float min_terrain_cost_factor)
        {
            float dx = static_cast<float>(x2 - x1);
            float dy = static_cast<float>(y2 - y1);
            float dist = std::sqrt(dx * dx + dy * dy) * resolution;
            return dist * min_terrain_cost_factor;
        }

        // World coordinates from grid coordinates
        inline Point_float<float> getWorldCoords(
            int x, int y, float resolution, float offset_x, float offset_y) {
            return { (static_cast<float>(x) + 0.5f) * resolution + offset_x,
                    (static_cast<float>(y) + 0.5f) * resolution + offset_y };
        }

        // World distance between points
        inline float worldDistance(const Point_float<float>& p1, const Point_float<float>& p2) {
            float dx = p2.x - p1.x; float dy = p2.y - p1.y;
            return std::sqrt(dx * dx + dy * dy);
        }

        // Bresenham for path interpolation
        std::vector<GridPoint> getLineSegmentCells(int x0, int y0, int x1, int y1) {
            std::vector<GridPoint> line;
            int dx = std::abs(x1 - x0); int dy = -std::abs(y1 - y0);
            int sx = (x0 < x1) ? 1 : -1; int sy = (y0 < y1) ? 1 : -1;
            int err = dx + dy; int cx = x0; int cy = y0;
            while (true) {
                line.push_back({ cx, cy });
                if (cx == x1 && cy == y1) break;
                int e2 = 2 * err;
                if (e2 >= dy) { if (cx == x1) break; err += dy; cx += sx; }
                if (e2 <= dx) { if (cy == y1) break; err += dx; cy += sy; }
            }
            return line;
        }

        // Line of Sight check using Bresenham
        bool hasLineOfSight(
            int x0, int y0, int x1, int y1,
            const Grid_V3& grid, int grid_width, int grid_height)
        {
            if (!grid.inBounds(x0, y0) || !grid.inBounds(x1, y1)) return false;
            int dx = std::abs(x1 - x0); int dy = -std::abs(y1 - y0);
            int sx = (x0 < x1) ? 1 : -1; int sy = (y0 < y1) ? 1 : -1;
            int err = dx + dy; int cx = x0; int cy = y0;
            while (true) {
                if (cx != x0 || cy != y0) {
                    if (!grid.inBounds(cx, cy)) return false;
                    const auto& cell = grid.at(cx, cy);
                    if (cell.hasFlag(GridFlags::FLAG_IMPASSABLE) || cell.value <= numeric_traits<float>::epsilon) {
                        return false;
                    }
                }
                if (cx == x1 && cy == y1) break;
                int e2 = 2 * err;
                if (e2 >= dy) { if (cx == x1) break; err += dy; cx += sx; }
                if (e2 <= dx) { if (cy == y1) break; err += dx; cy += sy; }
            }
            return true;
        }

        // Segment cost calculation (Tobler + Terrain along interpolated path)
        float calculateSegmentCost(
            int x_start, int y_start, int x_end, int y_end,
            const Grid_V3& grid, int grid_width, int grid_height,
            const ElevationSampler& elevation_sampler,
            float log_resolution, float origin_x, float origin_y,
            float& infinite_penalty_ref)
        {
            float total_cost = 0.0f;
            std::vector<GridPoint> segment_cells = getLineSegmentCells(x_start, y_start, x_end, y_end);
            if (segment_cells.size() < 2) { return 0.0f; }

            Point_float<float> prev_world = getWorldCoords(segment_cells[0].x, segment_cells[0].y, log_resolution, origin_x, origin_y);
            float prev_elev = elevation_sampler.getElevationAt(prev_world.x, prev_world.y);

            for (size_t i = 1; i < segment_cells.size(); ++i) {
                const GridPoint& current_gp = segment_cells[i];
                if (!grid.inBounds(current_gp.x, current_gp.y)) return infinite_penalty_ref;
                const auto& current_cell_data = grid.at(current_gp.x, current_gp.y);
                if (current_cell_data.hasFlag(GridFlags::FLAG_IMPASSABLE) || current_cell_data.value <= numeric_traits<float>::epsilon) {
                    return infinite_penalty_ref;
                }
                Point_float<float> curr_world = getWorldCoords(current_gp.x, current_gp.y, log_resolution, origin_x, origin_y);
                float current_elev = elevation_sampler.getElevationAt(curr_world.x, curr_world.y);
                float delta_h = current_elev - prev_elev;
                float delta_dist_world = worldDistance(prev_world, curr_world);

                if (delta_dist_world > numeric_traits<float>::epsilon) {
                    float S = delta_h / delta_dist_world;
                    float SlopeFactor = expf(-3.5f * fabsf(S + 0.05f));
                    float time_penalty = (SlopeFactor > numeric_traits<float>::epsilon) ? (1.0f / SlopeFactor) : infinite_penalty_ref;
                    if (time_penalty >= infinite_penalty_ref) return infinite_penalty_ref;
                    float terrain_cost = current_cell_data.value;
                    total_cost += delta_dist_world * time_penalty * terrain_cost;
                }
                prev_world = curr_world;
                prev_elev = current_elev;
            }
            return total_cost;
        }

    } // end anonymous namespace


    // --- Lazy Theta* Implementation ---
    std::vector<int> findLazyThetaStarPath_Tobler_Sampled(
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

        // --- Input Validation --- (Same as before)
        if (!logical_grid.isValid() || log_cell_resolution <= EPSILON) return resultPath;
        ElevationSampler elevation_sampler(elevation_values, elevation_width, elevation_height, elev_cell_resolution, origin_offset_x, origin_offset_y);
        if (!logical_grid.inBounds(start.x, start.y) || !logical_grid.inBounds(end.x, end.y)) return resultPath;
        const int startIdx = toIndex(start.x, start.y, log_width);
        const int endIdx = toIndex(end.x, end.y, log_width);
        const GridCellData& startCell = logical_grid.at(start.x, start.y);
        if (startCell.value <= numeric_traits<float>::epsilon || startCell.hasFlag(GridFlags::FLAG_IMPASSABLE)) return resultPath;
        const GridCellData& endCell = logical_grid.at(end.x, end.y);
        if (endCell.value <= numeric_traits<float>::epsilon || endCell.hasFlag(GridFlags::FLAG_IMPASSABLE)) return resultPath;
        if (startIdx == endIdx) { resultPath.push_back(startIdx); return resultPath; }

        // --- Lazy Theta* Data Structures --- (Same as Theta*)
        static thread_local std::vector<float> g_scores;
        static thread_local std::vector<float> f_scores;
        static thread_local std::vector<bool> closed;
        static thread_local std::vector<int> parents;
        try {
            g_scores.assign(log_size, std::numeric_limits<float>::max());
            f_scores.assign(log_size, std::numeric_limits<float>::max());
            closed.assign(log_size, false);
            parents.assign(log_size, -1);
        }
        catch (const std::bad_alloc&) { return resultPath; }

        // --- Priority Queue --- (Same as Theta*)
        auto cmp = [&](int l, int r) {
            if (std::fabs(f_scores[l] - f_scores[r]) > numeric_traits<float>::epsilon) return f_scores[l] > f_scores[r];
            return g_scores[l] > g_scores[r];
            };
        std::priority_queue<int, std::vector<int>, decltype(cmp)> openQueue(cmp);

        // --- Constants --- (Same as Theta*)
        const float infinite_penalty = std::numeric_limits<float>::max();
        const float MIN_TERRAIN_COST_FACTOR = 0.5f;

        // --- Initialization --- (Same as Theta*)
        g_scores[startIdx] = 0.0f;
        f_scores[startIdx] = calculate_theta_heuristic(start.x, start.y, end.x, end.y, log_cell_resolution, MIN_TERRAIN_COST_FACTOR);
        openQueue.push(startIdx);

        // --- Lazy Theta* Main Loop ---
        while (!openQueue.empty()) {
            const int currentIdx = openQueue.top();
            openQueue.pop();

            // Check if already processed (can happen if node added multiple times)
            if (closed[currentIdx]) { continue; }

            // --- Lazy Update Step ---
            // Check if the current node's path can be shortened by linking to its grandparent
            int parentIdx = parents[currentIdx];
            if (parentIdx != -1) { // Only if current is not the start node
                int grandParentIdx = parents[parentIdx];
                if (grandParentIdx != -1) { // Only if parent is not the start node
                    int x_curr, y_curr, x_gp, y_gp;
                    toCoords(currentIdx, log_width, x_curr, y_curr);
                    toCoords(grandParentIdx, log_width, x_gp, y_gp);

                    // Check LOS from grandparent to current
                    if (hasLineOfSight(x_gp, y_gp, x_curr, y_curr, logical_grid, log_width, log_height))
                    {
                        // Calculate cost along the straight segment
                        float segment_cost = calculateSegmentCost(
                            x_gp, y_gp, x_curr, y_curr,
                            logical_grid, log_width, log_height,
                            elevation_sampler,
                            log_cell_resolution, origin_offset_x, origin_offset_y,
                            const_cast<float&>(infinite_penalty)
                        );

                        if (segment_cost < infinite_penalty) {
                            float g_via_grandparent = g_scores[grandParentIdx] + segment_cost;
                            // If path via grandparent is shorter, update current node's parent and g_score
                            if (g_via_grandparent < g_scores[currentIdx]) {
                                g_scores[currentIdx] = g_via_grandparent;
                                parents[currentIdx] = grandParentIdx;
                                // F-score needs update too if g-score changed!
                                // Re-calculate f-score based on the new g-score
                                f_scores[currentIdx] = g_scores[currentIdx] + calculate_theta_heuristic(
                                    x_curr, y_curr, end.x, end.y, log_cell_resolution, MIN_TERRAIN_COST_FACTOR);
                                // Note: We don't re-insert into the queue here, as we are already processing `currentIdx`.
                            }
                        }
                    }
                }
            }
            // --- End of Lazy Update Step ---

            // Mark current node as closed *after* potential lazy update
            closed[currentIdx] = true;

            // Check for goal *after* potential lazy update and marking closed
            if (currentIdx == endIdx) { break; } // Goal reached

            // Get current coordinates after potential updates
            int x, y;
            toCoords(currentIdx, log_width, x, y);
            const float current_g = g_scores[currentIdx]; // Use potentially updated g-score

            // --- Explore Neighbors (Simplified A*-like relaxation) ---
            for (int dir = 0; dir < NUM_DIRECTIONS; ++dir) {
                const int nx = x + dx[dir];
                const int ny = y + dy[dir];

                if (nx < 0 || nx >= log_width || ny < 0 || ny >= log_height) { continue; }

                const int neighborIdx = toIndex(nx, ny, log_width);
                if (closed[neighborIdx]) { continue; }

                const GridCellData& neighborCell = logical_grid.at(nx, ny);
                if (neighborCell.value <= numeric_traits<float>::epsilon || neighborCell.hasFlag(GridFlags::FLAG_IMPASSABLE)) {
                    continue;
                }

                // Calculate cost for the single step from current to neighbor
                float step_cost = calculateSegmentCost(
                    x, y, nx, ny, // Single step segment
                    logical_grid, log_width, log_height,
                    elevation_sampler,
                    log_cell_resolution, origin_offset_x, origin_offset_y,
                    const_cast<float&>(infinite_penalty)
                );

                if (step_cost >= infinite_penalty) continue; // Cannot make this step

                float tentative_g = current_g + step_cost;

                // Update neighbor only if this path is better (standard A* relaxation)
                if (tentative_g < g_scores[neighborIdx]) {
                    parents[neighborIdx] = currentIdx; // Parent is always the current node here
                    g_scores[neighborIdx] = tentative_g;
                    f_scores[neighborIdx] = tentative_g + calculate_theta_heuristic(
                        nx, ny, end.x, end.y, log_cell_resolution, MIN_TERRAIN_COST_FACTOR);
                    openQueue.push(neighborIdx);
                }
            } // End neighbor loop
        } // End while openQueue not empty

        // --- Path Reconstruction (Identical to standard Theta* version) ---
        resultPath.clear(); // Ensure path is clear before reconstruction
        if (parents[endIdx] == -1 && startIdx != endIdx) { return resultPath; }

        std::vector<int> waypoints_reversed;
        int current_wp = endIdx;
        size_t safety_count = 0;
        const size_t max_path_len = static_cast<size_t>(log_size) * 2 + 2;
        while (current_wp != -1 && safety_count < max_path_len) {
            waypoints_reversed.push_back(current_wp);
            if (current_wp == startIdx) break;
            current_wp = parents[current_wp];
            safety_count++;
        }
        if (current_wp != startIdx && startIdx != endIdx) return std::vector<int>();
        if (safety_count >= max_path_len) return std::vector<int>();
        std::reverse(waypoints_reversed.begin(), waypoints_reversed.end());

        if (!waypoints_reversed.empty()) {
            resultPath.push_back(waypoints_reversed[0]);
            for (size_t i = 0; i < waypoints_reversed.size() - 1; ++i) {
                int p1_idx = waypoints_reversed[i]; int p2_idx = waypoints_reversed[i + 1];
                int x1, y1, x2, y2;
                toCoords(p1_idx, log_width, x1, y1); toCoords(p2_idx, log_width, x2, y2);
                std::vector<GridPoint> segment_cells = getLineSegmentCells(x1, y1, x2, y2);
                for (size_t j = 1; j < segment_cells.size(); ++j) {
                    int cell_idx = toIndex(segment_cells[j].x, segment_cells[j].y, log_width);
                    if (resultPath.empty() || cell_idx != resultPath.back()) {
                        resultPath.push_back(cell_idx);
                    }
                }
            }
        }
        else if (startIdx == endIdx) { resultPath.push_back(startIdx); }

        return resultPath;
    }

}// namespace Pathfinding
#include "map/MapProcessor.hpp"
#include "map/ParallelProcessorFlags.hpp" // Needed for Pass 1 processing
#include "IO/tinyxml2.h"               // XML parsing library implementation detail

#include <iostream>
#include <sstream>
#include <set>          // Used by MinimalRasterizer
#include <queue>        // Used by MinimalRasterizer (flood fill)
#include <stdexcept>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <cstdio>       // For fprintf/printf in rasterizer/debug output
#include <omp.h> 
#include <map>      // For Edge Table (alternative: vector if y-range known)
#include <vector>
#include <algorithm>// For std::sort, std::remove_if
#include <cmath>    // For std::ceil, std::floor, std::abs, std::isfinite
#include <limits>   // For std::numeric_limits


namespace mapgeo {
    namespace { // Anonymous namespace for internal helpers not part of the MapProcessor class interface

        // =================== Minimal Rasterizer (Internal Helper Class) ===================
        // (Code for MinimalRasterizer as provided previously - ensure height_ fix is applied)
        class MinimalRasterizer {
            using VertexData = FinalFeatureData::VertexData;
            using IntPointSet = std::set<IntPoint>;


            // --- Edge structure for Scanline ---
            struct EdgeInfo {
                double ymax;       // The y-coordinate where the edge ends (exclusive or inclusive depending on rule)
                double current_x;  // The current x-intersection with the scanline
                double inv_slope;  // Inverse slope (dx/dy)
                // No need for is_hole flag, parity handles it

                // Constructor for convenience
                EdgeInfo(double ym, double cx, double is) : ymax(ym), current_x(cx), inv_slope(is) {}
            };

            // --- Edge Table Type ---
            // Using std::map keyed by start y-scanline. Flexible but potentially slower than vector.
            using EdgeTable = std::map<int, std::vector<EdgeInfo>>;
            // Alternative: using std::vector<std::vector<EdgeInfo>> edgeTable(grid_height_); requires grid_height > 0

        public:
            MinimalRasterizer(size_t grid_width, size_t grid_height)
                : grid_width_(grid_width),
                grid_height_(grid_height),
                visited_(grid_width* grid_height, false),
                total_grid_cells_(grid_width* grid_height)
            {
                // Ensure width/height are non-zero before allocating visited_
                if (grid_width == 0 || grid_height == 0) {
                    // Handle error or default construction state
                    total_grid_cells_ = 0;
                    // Potentially throw or set an error flag
                }
                else {
                    visited_.assign(grid_width * grid_height, false);
                }
            }
            //OLD getCoveredCells for flood fill
            //IntPointSet getCoveredCells(const std::vector<VertexData>& outer_boundary_vd,
            //    const std::vector<std::vector<VertexData>>& hole_boundaries_vd,
            //    int effective_object_type,
            //    const std::string& feature_sym_id = "")
            //{
            //    // Reset state for this feature
            //    coveredCellsSet_.clear();
            //    if (total_grid_cells_ > 0) { // Avoid clearing/filling if grid is invalid
            //        std::fill(visited_.begin(), visited_.end(), false);
            //    }
            //    else if (!outer_boundary_vd.empty()) {
            //        fprintf(stderr, "Warning (Rasterizer): Grid has zero cells, cannot process feature %s\n", feature_sym_id.c_str());
            //        return coveredCellsSet_; // Return empty if grid is invalid
            //    }
            //    if (outer_boundary_vd.empty()) {
            //        return coveredCellsSet_;
            //    }
            //    // --- Helper Lambda: Convert VertexData loop to IntPoint loop ---
            //    auto convert_loop = [&](const std::vector<VertexData>& vd_loop) {
            //        std::vector<IntPoint> int_loop;
            //        if (vd_loop.empty() || grid_width_ == 0 || grid_height_ == 0) return int_loop; // Check grid dims
            //        int_loop.reserve(vd_loop.size() + 1);
            //        int max_x_idx = static_cast<int>(grid_width_ - 1);
            //        int max_y_idx = static_cast<int>(grid_height_ - 1);
            //        // No need for max_x/y_idx < 0 checks if we ensure grid_width/height > 0 earlier
            //        for (const auto& vd : vd_loop) {
            //            int ix = static_cast<int>(std::floor(vd.pos.x));
            //            int iy = static_cast<int>(std::floor(vd.pos.y));
            //            ix = std::max(0, std::min(ix, max_x_idx)); // Clamp X
            //            iy = std::max(0, std::min(iy, max_y_idx)); // Clamp Y
            //            if (int_loop.empty() || !(int_loop.back().x == ix && int_loop.back().y == iy)) {
            //                int_loop.push_back({ ix, iy });
            //            }
            //        }
            //        if (effective_object_type == 1 && int_loop.size() >= 3) {
            //            if (!(int_loop.back() == int_loop.front())) {
            //                int_loop.push_back(int_loop.front());
            //            }
            //        }
            //        return int_loop;
            //        }; // End lambda convert_loop
            //    auto outer_boundary_int = convert_loop(outer_boundary_vd);
            //    if (outer_boundary_int.empty()) {
            //        return coveredCellsSet_;
            //    }
            //    if (effective_object_type == 0 || outer_boundary_int.size() == 1) {
            //        if (inBounds(outer_boundary_int[0].x, outer_boundary_int[0].y)) {
            //            coveredCellsSet_.insert(outer_boundary_int[0]);
            //        }
            //        return coveredCellsSet_;
            //    }
            //    IntPointSet boundaryPoints;
            //    drawBoundary(outer_boundary_vd, boundaryPoints, effective_object_type);
            //    for (const auto& hole_vd : hole_boundaries_vd) {
            //        drawBoundary(hole_vd, boundaryPoints,1);
            //    }
            //    coveredCellsSet_ = boundaryPoints;
            //    bool needs_fill = (effective_object_type == 1 && outer_boundary_int.size() >= 3);
            //    if (needs_fill) {
            //        IntPoint start = findValidFillStart(outer_boundary_int, boundaryPoints);
            //        if (start.x != -1) {
            //            size_t size_before_fill = coveredCellsSet_.size();
            //            floodFill(start, boundaryPoints);
            //            size_t size_after_fill = coveredCellsSet_.size();
            //            const double fill_threshold = 0.95;
            //            if (total_grid_cells_ > 0 && (double)size_after_fill / total_grid_cells_ > fill_threshold) {
            //                //fprintf(stderr, "ERROR: Feature (SymID: %s, Type: %d) fill covered %zu cells (%.1f%% of grid), likely fill error. DISCARDING FILL, using boundary only (%zu cells).\n",
            //                //    feature_sym_id.c_str(), effective_object_type,
            //                //    size_after_fill, 100.0 * size_after_fill / total_grid_cells_,
            //                //    boundaryPoints.size());
            //                coveredCellsSet_ = boundaryPoints;
            //            }
            //        }
            //        else {
            //            if (!feature_sym_id.empty()) {
            //                // Consider logging levels, this might be verbose
            //                // printf("Debug Rasterizer: Fill skipped for area (SymID: %s) - no valid interior start point found.\n", feature_sym_id.c_str());
            //            }
            //        }
            //    }
            //    return coveredCellsSet_;
            //}
            // --- Main Public Method (Refactored Orchestration) ---
            // --- Main Public Method (Refactored Orchestration - Option 2: Fill First) ---
            IntPointSet getCoveredCells(const std::vector<VertexData>& outer_boundary_vd,
                const std::vector<std::vector<VertexData>>& hole_boundaries_vd,
                int effective_object_type,
                const std::string& feature_sym_id = "")
            {
                // Reset state for this feature - only the result set now
                coveredCellsSet_.clear();

                if (outer_boundary_vd.empty() || total_grid_cells_ == 0) {
                    if (!outer_boundary_vd.empty()) {
                        fprintf(stderr, "Warning (Rasterizer): Grid has zero cells, cannot process feature %s\n", feature_sym_id.c_str());
                    }
                    return coveredCellsSet_;
                }

                // Initialize sets
                IntPointSet boundaryPoints;
                IntPointSet interiorPoints;

                // --- Handle different object types ---
                if (effective_object_type != 1) { // 0=Point, 2=Line
                    // Only draw boundary (respecting open loops for lines)
                    drawBoundary(outer_boundary_vd, boundaryPoints, effective_object_type);
                    // For points/lines, the result *is* the boundary. No interior.
                    coveredCellsSet_ = boundaryPoints;
                }
                else { // 1=Area
                    // --- Step 1: Perform Scanline Fill for Area interior ---
                    interiorPoints = scanlineFillInternal(outer_boundary_vd, hole_boundaries_vd);

                    // --- Step 2: Draw boundaries *after* filling ---
                    // We still calculate boundaryPoints separately for potential use in checks or if scanline fails.
                    drawBoundary(outer_boundary_vd, boundaryPoints, 1); // Draw outer closed
                    for (const auto& hole_vd : hole_boundaries_vd) {
                        if (hole_vd.size() >= 3) {
                            drawBoundary(hole_vd, boundaryPoints, 1); // Draw holes closed
                        }
                    }

                    // --- Step 3: Combine Results (Prioritize Interior, then add Boundary) ---
                    coveredCellsSet_ = interiorPoints; // Start with interior pixels
                    // Add boundary points. If a pixel was both interior and boundary,
                    // it remains in the set, effectively prioritizing the boundary's presence
                    // in the final set, although it was added second. std::set handles uniqueness.
                    coveredCellsSet_.insert(boundaryPoints.begin(), boundaryPoints.end());

                    // --- Step 4: Excessive Fill Check (Optional but Recommended) ---
                    // Note: The check condition `coveredCellsSet_.size() > boundaryPoints.size()` might need adjustment
                    // if scanline fill *perfectly* fills up to but not including the boundary.
                    // A safer check might be if interiorPoints is not empty.
                    const double fill_threshold = 0.95; // Example threshold
                    if (total_grid_cells_ > 0 && !interiorPoints.empty() && // Check if filling actually happened
                        (double)coveredCellsSet_.size() / total_grid_cells_ > fill_threshold)
                    {
                        fprintf(stderr, "ERROR: Feature (SymID: %s, Type: %d) scanline fill covered %zu cells (%.1f%% of grid), likely error. DISCARDING FILL, using boundary only (%zu cells).\n",
                            feature_sym_id.c_str(), effective_object_type,
                            coveredCellsSet_.size(), 100.0 * coveredCellsSet_.size() / total_grid_cells_,
                            boundaryPoints.size());
                        coveredCellsSet_ = boundaryPoints; // Revert to boundary only if fill seems excessive
                    }
                }

                return coveredCellsSet_;
            }
        private:
            size_t grid_width_;
            size_t grid_height_;
            size_t total_grid_cells_;
            std::vector<bool> visited_; // Size matches grid_width_ * grid_height_
            IntPointSet coveredCellsSet_;
            //IntPointSet coveredCellsSet_; // Stores the combined result for the current feature
            const double FP_EPSILON = 1e-9; // Epsilon for floating point checks

            inline bool inBounds(int x, int y) const {
                // Check against member variables
                return static_cast<unsigned>(x) < grid_width_ && static_cast<unsigned>(y) < grid_height_;
            }


            void drawBoundary(const std::vector<VertexData>& polygon_vd, IntPointSet& boundarySet, int effective_object_type) {
                if (polygon_vd.size() < 2 || grid_width_ == 0 || grid_height_ == 0) return; // Check grid dims

                int max_x_idx = static_cast<int>(grid_width_ - 1);
                int max_y_idx = static_cast<int>(grid_height_ - 1);

                // Draw segments between consecutive points (0->1, 1->2, ..., n-2 -> n-1)
                for (size_t i = 0; i < polygon_vd.size() - 1; ++i) {
                    const auto& p1_vd = polygon_vd[i];
                    const auto& p2_vd = polygon_vd[i + 1]; // Use i+1

                    // Check gap/dash flag on the *end* point of the segment (p2_vd)
                    if ((p2_vd.original_flag & CoordFlags::GapPoint) || (p2_vd.original_flag & CoordFlags::DashPoint)) {
                        continue; // Skip drawing this segment
                    }

                    // Convert points (clamping done inside bresenham)
                    IntPoint p1_int = { static_cast<int>(std::floor(p1_vd.pos.x)), static_cast<int>(std::floor(p1_vd.pos.y)) };
                    IntPoint p2_int = { static_cast<int>(std::floor(p2_vd.pos.x)), static_cast<int>(std::floor(p2_vd.pos.y)) };
                    p1_int.x = std::max(0, std::min(p1_int.x, max_x_idx));
                    p1_int.y = std::max(0, std::min(p1_int.y, max_y_idx));
                    p2_int.x = std::max(0, std::min(p2_int.x, max_x_idx));
                    p2_int.y = std::max(0, std::min(p2_int.y, max_y_idx));

                    bresenhamLineToSet(p1_int, p2_int, boundarySet);
                }

                // Draw the closing segment ONLY if it's an Area type (1) and has enough points
                bool close_loop = (effective_object_type == 1 && polygon_vd.size() >= 3);
                if (close_loop) {
                    const auto& p_last_vd = polygon_vd.back();
                    const auto& p_first_vd = polygon_vd.front();

                    // Check gap/dash flag on the *end* point of the closing segment (p_first_vd)
                    if (!((p_first_vd.original_flag & CoordFlags::GapPoint) || (p_first_vd.original_flag & CoordFlags::DashPoint))) {
                        IntPoint p_last_int = { static_cast<int>(std::floor(p_last_vd.pos.x)), static_cast<int>(std::floor(p_last_vd.pos.y)) };
                        IntPoint p_first_int = { static_cast<int>(std::floor(p_first_vd.pos.x)), static_cast<int>(std::floor(p_first_vd.pos.y)) };
                        p_last_int.x = std::max(0, std::min(p_last_int.x, max_x_idx));
                        p_last_int.y = std::max(0, std::min(p_last_int.y, max_y_idx));
                        p_first_int.x = std::max(0, std::min(p_first_int.x, max_x_idx));
                        p_first_int.y = std::max(0, std::min(p_first_int.y, max_y_idx));


                        bresenhamLineToSet(p_last_int, p_first_int, boundarySet); // Connect last to first
                    }
                }
                
            }

            void bresenhamLineToSet(IntPoint p1, IntPoint p2, IntPointSet& pointSet) {
                int dx = std::abs(p2.x - p1.x), sx = (p1.x < p2.x) ? 1 : -1;
                int dy = -std::abs(p2.y - p1.y), sy = (p1.y < p2.y) ? 1 : -1;
                int err = dx + dy;

                while (true) {
                    // inBounds check inside the loop is necessary
                    if (inBounds(p1.x, p1.y)) {
                        pointSet.insert(p1);
                    }
                    if (p1.x == p2.x && p1.y == p2.y) break;

                    int e2 = 2 * err;
                    if (e2 >= dy) {
                        if (p1.x == p2.x && p1.y == p2.y) break;
                        err += dy;
                        p1.x += sx;
                    }
                    if (e2 <= dx) {
                        if (p1.x == p2.x && p1.y == p2.y) break;
                        err += dx;
                        p1.y += sy;
                    }
                }
            }

            // --- NEW Scanline Fill Implementation ---
    /**
     * @brief Fills the interior of a polygon (defined by outer and hole boundaries)
     *        using a scanline algorithm with an Edge Table and Active Edge Table.
     * @param outer_boundary_vd The vertices defining the outer boundary.
     * @param hole_boundaries_vd A list of vertex lists defining the holes.
     * @return An IntPointSet containing the integer coordinates of the filled interior pixels.
     *         Boundary pixels themselves are NOT included in this set.
     */

            /* // workin same what
            IntPointSet scanlineFillInternal(const std::vector<VertexData>& outer_boundary_vd,
                const std::vector<std::vector<VertexData>>& hole_boundaries_vd, const std::string& feature_sym_id) const
            {
                IntPointSet filledInteriorPoints;
                if (outer_boundary_vd.size() < 3 || grid_width_ == 0 || grid_height_ == 0) {
                    return filledInteriorPoints; // Cannot fill if outer loop is not at least a triangle or grid invalid
                }
                const std::string target_sym_id = "106"; // The ID of the problematic feature
                const bool enable_logging = (feature_sym_id == target_sym_id);
                EdgeTable edgeTable;
                double global_min_y = std::numeric_limits<double>::max();
                double global_max_y = std::numeric_limits<double>::lowest();

                // --- Lambda to process one loop (outer or hole) and add edges to ET ---
                auto buildEdgesForLoop = [&](const std::vector<VertexData>& loop_vd) {
                    if (loop_vd.size() < 2) return; // Need at least 2 points for an edge

                    for (size_t i = 0; i < loop_vd.size(); ++i) {
                        const VertexData& vd1 = loop_vd[i];
                        const VertexData& vd2 = loop_vd[(i + 1) % loop_vd.size()]; // Wrap around

                        const Point_float<float>& p1 = vd1.pos;
                        const Point_float<float>& p2 = vd2.pos;

                        // Skip segments marked as gaps/dashes (relevant for outer loop, less so for holes?)
                        // Check the *end* point flag of the segment (vd2)
                        if ((vd2.original_flag & CoordFlags::GapPoint) || (vd2.original_flag & CoordFlags::DashPoint)) {
                            continue;
                        }


                        // Determine ymin/ymax points and values using double
                        double y1 = static_cast<double>(p1.y);
                        double y2 = static_cast<double>(p2.y);
                        double x1 = static_cast<double>(p1.x);
                        double x2 = static_cast<double>(p2.x);

                        // Ignore Horizontal Edges for Edge Table
                        if (std::abs(y1 - y2) < FP_EPSILON) {
                            continue;
                        }

                        // Find edge endpoints and min/max y
                        double ymin, ymax, x_at_ymin;
                        if (y1 < y2) {
                            ymin = y1; ymax = y2; x_at_ymin = x1;
                        }
                        else {
                            ymin = y2; ymax = y1; x_at_ymin = x2;
                        }

                        // Update global y bounds (use vertex coordinates directly)
                        global_min_y = std::min({ global_min_y, ymin });
                        global_max_y = std::max({ global_max_y, ymax });

                        // Calculate inverse slope (dx/dy)
                        double inv_slope;
                        double dx = x2 - x1;
                        double dy = y2 - y1; // Already checked dy != 0

                        if (std::abs(dx) < FP_EPSILON) { // Vertical edge
                            inv_slope = std::numeric_limits<double>::infinity();
                        }
                        else {
                            inv_slope = dx / dy;
                        }

                        // Determine starting scanline index (use ceil for robustness)
                        int start_y_index = static_cast<int>(std::ceil(ymin));
                        // Clip start index to grid bounds just in case float precision pushed it slightly below 0
                        start_y_index = std::max(0, start_y_index);

                        // Add edge info to the map
                        // The ymax here defines when the edge should be REMOVED.
                        // Standard scanline removes when scanline y >= ymax.
                        edgeTable[start_y_index].emplace_back(ymax, x_at_ymin, inv_slope);

                    } // End loop through segments
                    }; // End lambda buildEdgesForLoop


                // --- Build Edge Table for outer and hole boundaries ---
                buildEdgesForLoop(outer_boundary_vd);
                for (const auto& hole_loop : hole_boundaries_vd) {
                    buildEdgesForLoop(hole_loop); // Treat hole edges exactly the same
                }

                // --- Scanline Processing ---
                if (edgeTable.empty()) {
                    // Likely only horizontal edges, no interior to fill
                    return filledInteriorPoints;
                }

                std::vector<EdgeInfo> aet; // Active Edge Table (store structs directly)

                int min_scanline_y = std::max(0, static_cast<int>(std::ceil(global_min_y)));
                int max_scanline_y = std::min(static_cast<int>(grid_height_ - 1), static_cast<int>(std::floor(global_max_y)));
                if (max_scanline_y < min_scanline_y) return filledInteriorPoints; // Handle cases where polygon is tiny or outside grid after clamping


                for (int y = min_scanline_y; y <= max_scanline_y; ++y) {

                    if (enable_logging) {
                        fprintf(stderr, "\n--- Scanline y=%d (Feature %s) ---\n", y, feature_sym_id.c_str());
                        // Log AET before/after sort, fill spans, updates etc.
                    }
                        

                    double current_scanline_y_double = static_cast<double>(y);

                    // i. Remove finished edges from AET
                    // Remove if edge ends *strictly before* this scanline? Or at/before?
                    // Standard: Remove if ymax <= current_scanline_y. Let's stick to that.
                    aet.erase(std::remove_if(aet.begin(), aet.end(),
                        [&](const EdgeInfo& edge) { return edge.ymax <= current_scanline_y_double; }),
                        aet.end());

                    // ii. Add new edges starting at this scanline from ET
                    auto et_it = edgeTable.find(y);
                    if (et_it != edgeTable.end()) {
                        aet.insert(aet.end(), et_it->second.begin(), et_it->second.end());
                    }

                    if (aet.empty()) continue; // Nothing to fill on this line if AET empty

                    // iii. Sort AET by current_x
                    std::sort(aet.begin(), aet.end(), [](const EdgeInfo& a, const EdgeInfo& b) {
                        // Handle potential exact ties carefully if needed, maybe add secondary sort key?
                        if (std::abs(a.current_x - b.current_x) < 1e-9) {
                            // Optional tie-breaker using slope? Can help with vertices touching scanline.
                            return a.inv_slope < b.inv_slope;
                        }
                        return a.current_x < b.current_x;
                        });

                    // iv. Fill spans using sorted AET (odd/even pairs)
                    for (size_t i = 0; i + 1 < aet.size(); i += 2) {
                        double x_start = aet[i].current_x;
                        double x_end = aet[i + 1].current_x;

                        // Convert float x range to integer pixel range
                        // Use ceil for start, floor for end to fill pixels whose centers are within the span
                        int fill_x_min = static_cast<int>(std::ceil(x_start));
                        int fill_x_max = static_cast<int>(std::floor(x_end));

                        // Ensure max isn't less than min after conversion (can happen for thin vertical slivers)
                        if (fill_x_min > fill_x_max) continue;

                        // Clip fill range to grid bounds
                        fill_x_min = std::max(0, fill_x_min);
                        fill_x_max = std::min(static_cast<int>(grid_width_ - 1), fill_x_max);


                        // Fill pixels within the span for the current scanline y
                        for (int x = fill_x_min; x <= fill_x_max; ++x) {
                            // No need to check bounds again if fill_x_* are clipped
                            // The check against boundaryPoints is done *outside* this function now
                            filledInteriorPoints.insert({ x, y });
                        }
                    }

                    // v. Update current_x for next scanline for all edges remaining in AET
                    for (EdgeInfo& edge : aet) {
                        // Only update if slope is finite (not a vertical edge)
                        if (std::isfinite(edge.inv_slope)) {
                            edge.current_x += edge.inv_slope;
                        }
                    }
                } // End scanline loop y

                return filledInteriorPoints;
            }*/

            // --- Scanline Fill Implementation with Refined AET Removal & Minimal Logging ---
            //IntPointSet scanlineFillInternal(const std::vector<VertexData>& outer_boundary_vd,
            //    const std::vector<std::vector<VertexData>>& hole_boundaries_vd,
            //    const std::string& feature_sym_id,
            //    bool enable_logging = false) const
            //{
            //    IntPointSet filledInteriorPoints;
            //    if (outer_boundary_vd.size() < 3 || grid_width_ == 0 || grid_height_ == 0) return filledInteriorPoints;
            //    EdgeTable edgeTable;
            //    double global_min_y = std::numeric_limits<double>::max();
            //    double global_max_y = std::numeric_limits<double>::lowest();
            //    // --- buildEdgesForLoop Lambda (No change needed from version where Gap/Dash check was removed) ---
            //    auto buildEdgesForLoop = [&](const std::vector<VertexData>& loop_vd) {
            //        if (loop_vd.size() < 2) return;
            //        for (size_t i = 0; i < loop_vd.size(); ++i) {
            //            const VertexData& vd1 = loop_vd[i];
            //            const VertexData& vd2 = loop_vd[(i + 1) % loop_vd.size()];
            //            const Point_float<float>& p1 = vd1.pos; const Point_float<float>& p2 = vd2.pos;
            //            double y1 = static_cast<double>(p1.y); double y2 = static_cast<double>(p2.y);
            //            double x1 = static_cast<double>(p1.x); double x2 = static_cast<double>(p2.x);
            //            if (std::abs(y1 - y2) < FP_EPSILON) continue; // Skip horizontal
            //            double ymin, ymax, x_at_ymin;
            //            if (y1 < y2) { ymin = y1; ymax = y2; x_at_ymin = x1; }
            //            else { ymin = y2; ymax = y1; x_at_ymin = x2; }
            //            global_min_y = std::min(global_min_y, ymin); global_max_y = std::max(global_max_y, ymax);
            //            double inv_slope; double dx = x2 - x1; double dy = y2 - y1;
            //            if (std::abs(dx) < FP_EPSILON) inv_slope = std::numeric_limits<double>::infinity();
            //            else inv_slope = dx / dy;
            //            int start_y_index = std::max(0, static_cast<int>(std::ceil(ymin)));
            //            edgeTable[start_y_index].emplace_back(ymax, x_at_ymin, inv_slope);
            //        }
            //        };
            //    buildEdgesForLoop(outer_boundary_vd);
            //    for (const auto& hole_loop : hole_boundaries_vd) buildEdgesForLoop(hole_loop);
            //    if (edgeTable.empty()) return filledInteriorPoints;
            //    std::vector<EdgeInfo> aet; // Active Edge Table
            //    int min_scanline_y = std::max(0, static_cast<int>(std::ceil(global_min_y)));
            //    int max_scanline_y = std::min(static_cast<int>(grid_height_ - 1), static_cast<int>(std::floor(global_max_y)));
            //    if (max_scanline_y < min_scanline_y) return filledInteriorPoints;
            //    if (enable_logging) fprintf(stderr, "[Feat %s] ScanlineFill: Processing Y range [%d, %d].\n", feature_sym_id.c_str(), min_scanline_y, max_scanline_y);
            //    // --- Main Scanline Loop ---
            //    for (int y = min_scanline_y; y <= max_scanline_y; ++y) {
            //        // 1. Add New Edges FIRST for scanline y
            //        auto et_it = edgeTable.find(y);
            //        if (et_it != edgeTable.end()) {
            //            aet.insert(aet.end(), et_it->second.begin(), et_it->second.end());
            //        }
            //        // 2. Remove edges that END strictly *before* scanline y (using previous logic was okay too, this is just another standard way)
            //        aet.erase(std::remove_if(aet.begin(), aet.end(),
            //            [&](const EdgeInfo& edge) { return static_cast<int>(std::floor(edge.ymax)) < y; }), // Keep edges ending ON this scanline y
            //            aet.end());
            //        if (aet.empty()) continue;
            //        // 3. Sort AET by current_x
            //        std::sort(aet.begin(), aet.end(), [&](const EdgeInfo& a, const EdgeInfo& b) {
            //            if (std::abs(a.current_x - b.current_x) < FP_EPSILON * 10) {
            //                bool a_inf = !std::isfinite(a.inv_slope); bool b_inf = !std::isfinite(b.inv_slope);
            //                if (a_inf && b_inf) return false; if (a_inf) return false; if (b_inf) return true;
            //                return std::abs(a.inv_slope) > std::abs(b.inv_slope); // Shallower first tie-break
            //            }
            //            return a.current_x < b.current_x;
            //            });
            //        // --- MINIMAL LOGGING ---
            //        if (enable_logging && (aet.size() % 2 != 0)) {
            //            fprintf(stderr, "[Feat %s] WARNING: Odd number of edges (%zu) in AET at Y=%d!\n",
            //                feature_sym_id.c_str(), aet.size(), y);
            //            fprintf(stderr, "  AET Sorted (%zu): ", aet.size());
            //            for (const auto& edge : aet) fprintf(stderr, "[x=%.3f, ymax=%.1f, invS=%.3f] ", edge.current_x, edge.ymax, edge.inv_slope);
            //            fprintf(stderr, "\n");
            //        }
            //        // --- End Minimal Logging ---
            //        // 4. Fill spans for scanline y using the AET sorted based on intersections *at* y
            //        for (size_t i = 0; i + 1 < aet.size(); i += 2) {
            //            double x_start = aet[i].current_x;
            //            double x_end = aet[i + 1].current_x;
            //            int fill_x_min = static_cast<int>(std::ceil(x_start));
            //            int fill_x_max = static_cast<int>(std::floor(x_end));
            //            if (fill_x_min > fill_x_max) continue;
            //            fill_x_min = std::max(0, fill_x_min);
            //            fill_x_max = std::min(static_cast<int>(grid_width_ - 1), fill_x_max);
            //            for (int x = fill_x_min; x <= fill_x_max; ++x) {
            //                if (inBounds(x, y)) filledInteriorPoints.insert({ x, y });
            //            }
            //        }
            //        // 5. Update X coordinates for the *next* scanline (y+1)
            //        for (EdgeInfo& edge : aet) {
            //            if (std::isfinite(edge.inv_slope)) {
            //                edge.current_x += edge.inv_slope;
            //            }
            //        }
            //    } // End scanline loop y
            //    if (enable_logging) fprintf(stderr, "[Feat %s] ScanlineFill: Finished. Found %zu interior points.\n", feature_sym_id.c_str(), filledInteriorPoints.size());
            //    return filledInteriorPoints;
            //} // End scanlineFillInternal
            
        IntPointSet scanlineFillInternal(const std::vector<VertexData>& outer_boundary_vd,
            const std::vector<std::vector<VertexData>>& hole_boundaries_vd) const
        {
            IntPointSet filledInteriorPoints;
            if (outer_boundary_vd.size() < 3 || grid_width_ == 0 || grid_height_ == 0) {
                return filledInteriorPoints;
            }

            EdgeTable edgeTable;
            double global_min_y = std::numeric_limits<double>::max();
            double global_max_y = std::numeric_limits<double>::lowest();

            // Lambda to process one loop (outer or hole) and add edges to ET
            auto buildEdgesForLoop = [&](const std::vector<VertexData>& loop_vd) {
                if (loop_vd.size() < 2) return;
                for (size_t i = 0; i < loop_vd.size(); ++i) {
                    const VertexData& vd1 = loop_vd[i];
                    const VertexData& vd2 = loop_vd[(i + 1) % loop_vd.size()];
                    const Point_float<float>& p1 = vd1.pos; const Point_float<float>& p2 = vd2.pos;
                    double y1 = static_cast<double>(p1.y); double y2 = static_cast<double>(p2.y);
                    double x1 = static_cast<double>(p1.x); double x2 = static_cast<double>(p2.x);

                    if (std::abs(y1 - y2) < FP_EPSILON) continue; // Skip horizontal

                    double ymin, edge_actual_ymax, x_at_ymin; // Use edge_actual_ymax locally
                    if (y1 < y2) { ymin = y1; edge_actual_ymax = y2; x_at_ymin = x1; }
                    else { ymin = y2; edge_actual_ymax = y1; x_at_ymin = x2; }

                    global_min_y = std::min(global_min_y, ymin);
                    global_max_y = std::max(global_max_y, edge_actual_ymax); // Use actual max for bounds

                    // Adjust the ymax used for AET removal slightly downwards
                    double ymax_for_aet = edge_actual_ymax - FP_EPSILON * 10;

                    double inv_slope; double dx = x2 - x1; double dy = y2 - y1;
                    if (std::abs(dx) < FP_EPSILON) inv_slope = std::numeric_limits<double>::infinity();
                    else inv_slope = dx / dy;

                    int start_y_index = std::max(0, static_cast<int>(std::ceil(ymin)));

                    // Store the ADJUSTED ymax
                    edgeTable[start_y_index].emplace_back(ymax_for_aet, x_at_ymin, inv_slope);
                }
                };

            // Build Edge Table for outer and hole boundaries
            buildEdgesForLoop(outer_boundary_vd);
            for (const auto& hole_loop : hole_boundaries_vd) buildEdgesForLoop(hole_loop);

            if (edgeTable.empty()) return filledInteriorPoints;

            // Scanline Processing
            std::vector<EdgeInfo> aet;
            int min_scanline_y = std::max(0, static_cast<int>(std::ceil(global_min_y)));
            int max_scanline_y = std::min(static_cast<int>(grid_height_ - 1), static_cast<int>(std::floor(global_max_y)));
            if (max_scanline_y < min_scanline_y) return filledInteriorPoints;

            for (int y = min_scanline_y; y <= max_scanline_y; ++y) {
                // Add new edges for scanline y
                auto et_it = edgeTable.find(y);
                if (et_it != edgeTable.end()) aet.insert(aet.end(), et_it->second.begin(), et_it->second.end());

                // Remove edges ending *before* scanline y (using adjusted ymax)
                aet.erase(std::remove_if(aet.begin(), aet.end(),
                    [&](const EdgeInfo& edge) { return edge.ymax < static_cast<double>(y); }), // Strict < comparison
                    aet.end());

                if (aet.empty()) continue;

                // Sort AET by current_x
                std::sort(aet.begin(), aet.end(), [&](const EdgeInfo& a, const EdgeInfo& b) {
                    if (std::abs(a.current_x - b.current_x) < FP_EPSILON * 10) {
                        bool a_inf = !std::isfinite(a.inv_slope); bool b_inf = !std::isfinite(b.inv_slope);
                        if (a_inf && b_inf) return false; if (a_inf) return false; if (b_inf) return true;
                        return std::abs(a.inv_slope) > std::abs(b.inv_slope);
                    }
                    return a.current_x < b.current_x;
                    });

                // Fill spans using sorted AET (odd/even pairs)
                 // The check for odd number of edges can be removed now, but left as assert if desired
                 // assert(aet.size() % 2 == 0); // Or keep the fprintf warning if you prefer
                for (size_t i = 0; i + 1 < aet.size(); i += 2) {
                    double x_start = aet[i].current_x;
                    double x_end = aet[i + 1].current_x;
                    int fill_x_min = static_cast<int>(std::ceil(x_start));
                    int fill_x_max = static_cast<int>(std::ceil(x_end)) - 1; // Use ceil/ceil-1 rule
                    if (fill_x_min > fill_x_max) continue;
                    fill_x_min = std::max(0, fill_x_min);
                    fill_x_max = std::min(static_cast<int>(grid_width_ - 1), fill_x_max);
                    for (int x = fill_x_min; x <= fill_x_max; ++x) {
                        if (inBounds(x, y)) filledInteriorPoints.insert({ x, y });
                    }
                }

                // Update X coordinates for the next scanline (y+1)
                for (EdgeInfo& edge : aet) {
                    if (std::isfinite(edge.inv_slope)) {
                        edge.current_x += edge.inv_slope;
                    }
                }
            } // End scanline loop y

            return filledInteriorPoints;
        } // End scanlineFillInternal

            IntPoint findInteriorPoint(const std::vector<IntPoint>& polygon) {
                if (polygon.empty()) return { -1, -1 };
                if (polygon.size() == 1) return polygon[0];

                long long sum_x = 0, sum_y = 0;
                size_t count = (polygon.size() > 1 && polygon.front() == polygon.back()) ? polygon.size() - 1 : polygon.size();

                if (count == 0) return { -1,-1 };

                for (size_t i = 0; i < count; ++i) {
                    sum_x += polygon[i].x;
                    sum_y += polygon[i].y;
                }
                return { static_cast<int>(sum_x / count), static_cast<int>(sum_y / count) };
            }

            IntPoint findValidFillStart(const std::vector<IntPoint>& polygon, const IntPointSet& boundarySet) {
                if (polygon.size() < 3 || grid_width_ == 0 || grid_height_ == 0) return { -1, -1 }; // Check grid dims

                IntPoint centroid = findInteriorPoint(polygon);
                if (centroid.x != -1) {
                    const int offsets[] = { 0, 0,  1, 0,  -1, 0,  0, 1,  0, -1,  1, 1,  -1, -1,  1, -1,  -1, 1 };
                    const int num_offsets = sizeof(offsets) / (sizeof(int) * 2);
                    for (int i = 0; i < num_offsets; ++i) {
                        IntPoint test_pt = { centroid.x + offsets[i * 2], centroid.y + offsets[i * 2 + 1] };
                        if (inBounds(test_pt.x, test_pt.y) && boundarySet.find(test_pt) == boundarySet.end()) {
                            return test_pt;
                        }
                    }
                }

                if (polygon.size() >= 2) { // Should always be true
                    IntPoint p1 = polygon[0];
                    IntPoint p2 = polygon[1];
                    if (p1.x == p2.x && p1.y == p2.y && polygon.size() >= 3) p2 = polygon[2];
                    int mid_x = p1.x + (p2.x - p1.x) / 2;
                    int mid_y = p1.y + (p2.y - p1.y) / 2;
                    const int fallback_offsets[] = { 0, 0,  1, 0,  -1, 0,  0, 1,  0, -1,  1, 1,  -1, -1,  1, -1,  -1, 1 };
                    const int num_fb_offsets = sizeof(fallback_offsets) / (sizeof(int) * 2);
                    for (int i = 0; i < num_fb_offsets; ++i) {
                        IntPoint test_pt = { mid_x + fallback_offsets[i * 2], mid_y + fallback_offsets[i * 2 + 1] };
                        if (inBounds(test_pt.x, test_pt.y) && boundarySet.find(test_pt) == boundarySet.end()) {
                            return test_pt;
                        }
                    }
                }
                return { -1,-1 };
            }
            // Old floodFill function (commented out)
            //void floodFill(IntPoint start, const IntPointSet& boundaryPoints) {
            //    if (!inBounds(start.x, start.y) || grid_width_ == 0 || grid_height_ == 0) return; // Check grid dims
            //    // Check index validity relative to visited_ size
            //    size_t start_idx = static_cast<size_t>(start.y) * grid_width_ + static_cast<size_t>(start.x);
            //    if (start_idx >= visited_.size()) {
            //        fprintf(stderr, "Warning (floodFill): Start index %zu out of bounds for visited array (size %zu).\n", start_idx, visited_.size());
            //        return;
            //    }
            //    if (visited_[start_idx] || boundaryPoints.count(start)) return;
            //    std::queue<IntPoint> q;
            //    visited_[start_idx] = true;
            //    q.push(start);
            //    coveredCellsSet_.insert(start);
            //    const int dx[] = { 1, -1, 0, 0 };
            //    const int dy[] = { 0, 0, 1, -1 };
            //    while (!q.empty()) {
            //        IntPoint curr = q.front();
            //        q.pop();
            //        for (int i = 0; i < 4; ++i) {
            //            int nx = curr.x + dx[i];
            //            int ny = curr.y + dy[i];
            //            if (inBounds(nx, ny)) {
            //                const size_t neighbor_idx = static_cast<size_t>(ny) * grid_width_ + static_cast<size_t>(nx);
            //                IntPoint neighbor_pt = { nx, ny };
            //                // Check index validity relative to visited_ size before accessing visited_
            //                if (neighbor_idx >= visited_.size()) {
            //                    fprintf(stderr, "Warning (floodFill): Neighbor index %zu out of bounds for visited array (size %zu).\n", neighbor_idx, visited_.size());
            //                    continue; // Skip this neighbor
            //                }
            //                if (!visited_[neighbor_idx] && boundaryPoints.find(neighbor_pt) == boundaryPoints.end()) {
            //                    visited_[neighbor_idx] = true;
            //                    q.push(neighbor_pt);
            //                    coveredCellsSet_.insert(neighbor_pt);
            //                }
            //            }
            //        }
            //    }
            //}

        }; // End class MinimalRasterizer

        // =================== Other Internal Helper Functions ===================
        std::string getBaseIsomCode(const std::string& full_code) {
            size_t dot_pos = full_code.find('.');
            if (dot_pos != std::string::npos) {
                return full_code.substr(0, dot_pos);
            }
            return full_code;
        }

        std::uint8_t getPathfindingFlags(const std::string& base_isom_code, float value) {
            std::uint8_t flags = GridFlags::FLAG_NONE;
            if (value <= 0.0f) { // Use <= 0 as impassable marker
                flags |= GridFlags::FLAG_IMPASSABLE;
            }
            // Specific feature flags
            if (base_isom_code >= "501" && base_isom_code <= "508") flags |= GridFlags::FLAG_ROAD_PATH;
            if (base_isom_code >= "301" && base_isom_code <= "311" &&
                base_isom_code != "304" && base_isom_code != "305" && base_isom_code != "306" && base_isom_code != "309") flags |= GridFlags::FLAG_WATER_MARSH;
            if (base_isom_code == "407" || base_isom_code == "409") flags |= GridFlags::FLAG_UNDERGROWTH;
            if (base_isom_code == "304" ||
                base_isom_code == "305" || 
                base_isom_code == "306" || 
                base_isom_code == "309" || 
                base_isom_code == "312")   
            {
                flags |= GridFlags::FLAG_ROAD_PATH; // Add this flag to trigger the override
            }
            return flags;
        }

    } // End anonymous namespace

    // =================== MapProcessor Method Implementations ===================
    MapProcessor::MapProcessor(const MapProcessorConfig& config) : config_(config) {}

    std::vector<Point> MapProcessor::parseCoordinates(const char* coord_string) const {
        std::vector<Point> points;
        if (!coord_string) return points;
        std::stringstream ss(coord_string);
        std::string segment;
        while (std::getline(ss, segment, ';')) {
            std::stringstream segment_ss(segment);
            std::string value_str;
            std::vector<double> values;
            while (segment_ss >> value_str) {
                try { values.push_back(std::stod(value_str)); }
                catch (...) { values.clear(); break; }
            }
            for (size_t i = 0; i + 1 < values.size(); ) {
                Point p; p.x = values[i]; p.y = values[i + 1];
                if (i + 2 < values.size()) {
                    double potential_flag = values[i + 2];
                    if (std::floor(potential_flag) == potential_flag) {
                        p.flag = static_cast<int>(potential_flag); i += 3;
                    }
                    else { p.flag = CoordFlags::NoFlag; i += 2; }
                }
                else { p.flag = CoordFlags::NoFlag; i += 2; }
                points.push_back(p);
            }
        } return points;
    }

    void MapProcessor::parseSymbolDefinitionsInternal(const tinyxml2::XMLElement* mapElement) {
        symbolDefinitions_.clear();
        if (!mapElement) return;
        const tinyxml2::XMLElement* symbolsParent = nullptr;
        if (!config_.layers_to_process.empty()) {
            const tinyxml2::XMLElement* layer = mapElement->FirstChildElement(config_.layers_to_process[0].c_str());
            if (layer) symbolsParent = layer->FirstChildElement("symbols");
        }
        if (!symbolsParent) symbolsParent = mapElement->FirstChildElement("symbols");
        if (!symbolsParent && mapElement->Value() && strcmp(mapElement->Value(), "symbols") == 0) symbolsParent = mapElement;
        if (!symbolsParent) { std::cerr << "Warning: Could not find <symbols> element.\n"; return; }
        // std::cout << "Info: Reading symbol definitions...\n";
        int count = 0;
        for (const tinyxml2::XMLElement* sym = symbolsParent->FirstChildElement("symbol"); sym; sym = sym->NextSiblingElement("symbol")) {
            const char* id_attr = sym->Attribute("id"); const char* type_attr = sym->Attribute("type"); const char* code_attr = sym->Attribute("code");
            if (!id_attr || !type_attr || !code_attr) continue;
            SymbolDefinition def; def.isom_code = code_attr;
            try { def.symbol_type = std::stoi(type_attr); }
            catch (...) { continue; }
            symbolDefinitions_[id_attr] = def; count++;
        }
        // std::cout << "Info: Read " << count << " symbol definitions.\n";
    }

    void MapProcessor::parseObjectGeometryInternal(const tinyxml2::XMLElement* mapElement) {
        intermediateObjects_.clear(); if (!mapElement) return;
        // std::cout << "Info: Starting object parsing...\n";
        int total_objects_parsed = 0;
        for (const std::string& layer_tag : config_.layers_to_process) {
            const tinyxml2::XMLElement* layer = mapElement->FirstChildElement(layer_tag.c_str()); if (!layer) continue;
            // std::cout << "Info: Processing layer '" << layer_tag << "'.\n";
            const tinyxml2::XMLElement* parts = layer->FirstChildElement("parts"); if (!parts) continue;
            const tinyxml2::XMLElement* part = parts->FirstChildElement("part"); if (!part) continue;
            const tinyxml2::XMLElement* objects = part->FirstChildElement("objects"); if (!objects) continue;
            int objects_in_layer = 0;
            for (const tinyxml2::XMLElement* obj = objects->FirstChildElement("object"); obj; obj = obj->NextSiblingElement("object")) {
                const char* sym_id_attr = obj->Attribute("symbol"); const char* obj_type_attr = obj->Attribute("type");
                const tinyxml2::XMLElement* coordsEl = obj->FirstChildElement("coords"); const char* coords_str = coordsEl ? coordsEl->GetText() : nullptr;
                if (!sym_id_attr || !obj_type_attr || !coords_str) continue;
                int obj_type = -1; try { obj_type = std::stoi(obj_type_attr); }
                catch (...) { continue; }
                if (obj_type < 0 || obj_type > 2) continue; // Only Point, Area, Line
                std::vector<Point> points = parseCoordinates(coords_str); if (points.empty()) continue;
                intermediateObjects_.push_back({ std::move(points), sym_id_attr, obj_type });
                objects_in_layer++; total_objects_parsed++;
            }
            // std::cout << "Info: Parsed " << objects_in_layer << " objects from layer '" << layer_tag << "'.\n";
        }
        // std::cout << "Info: Finished parsing. Total relevant objects parsed: " << total_objects_parsed << "\n";
    }

    // --- Updated calculateNormalizationParamsInternal ---
    bool MapProcessor::calculateNormalizationParamsInternal() {
        if (intermediateObjects_.empty()) {
            std::cerr << "Warning: Cannot calculate normalization, no intermediate objects loaded." << std::endl;
            normParams_.valid = false; // Ensure valid is false
            return false;
        }
        double min_x_g = std::numeric_limits<double>::max(), max_x_g = std::numeric_limits<double>::lowest();
        double min_y_g = std::numeric_limits<double>::max(), max_y_g = std::numeric_limits<double>::lowest();
        bool points_found = false;
        for (const auto& objData : intermediateObjects_) {
            if (objData.original_points.empty()) continue;
            points_found = true;
            for (const auto& pt : objData.original_points) {
                min_x_g = std::min(min_x_g, pt.x); max_x_g = std::max(max_x_g, pt.x);
                min_y_g = std::min(min_y_g, pt.y); max_y_g = std::max(max_y_g, pt.y);
            }
        }
        if (!points_found) {
            std::cerr << "Warning: Cannot calculate normalization, no valid points found." << std::endl;
            normParams_.valid = false;
            return false;
        }

        normParams_.min_x = min_x_g;
        normParams_.min_y = min_y_g;
        double range_x = max_x_g - min_x_g;
        double range_y = max_y_g - min_y_g;
        const double epsilon = 1e-9; // Use double epsilon

        // Handle grid width/height of 1 or 0
        bool single_col = config_.grid_width <= 1;
        bool single_row = config_.grid_height <= 1;
        if (config_.grid_width <= 0 || config_.grid_height <= 0) {
            std::cerr << "Error: Grid dimensions must be positive for normalization.\n";
            normParams_.valid = false;
            return false;
        }


        normParams_.scale_x = (single_col || std::fabs(range_x) < epsilon) ? 1.0 : (static_cast<double>(config_.grid_width - 1) / range_x);
        normParams_.scale_y = (single_row || std::fabs(range_y) < epsilon) ? 1.0 : (static_cast<double>(config_.grid_height - 1) / range_y);

        // Calculate resolutions, handle potential division by zero if scale is zero (e.g., range was huge or grid dim=1 resulted in scale=1?)
        // If scale is 1 (due to grid dim 1), resolution doesn't make much sense, default to 0? Or handle upstream? Let's default to 0.
        normParams_.resolution_x = (std::fabs(normParams_.scale_x) < epsilon || single_col) ? 0.0 : (1.0 / normParams_.scale_x);
        normParams_.resolution_y = (std::fabs(normParams_.scale_y) < epsilon || single_row) ? 0.0 : (1.0 / normParams_.scale_y);


        normParams_.valid = true;
        // std::cout << "Info: Normalization Params: Min(" << normParams_.min_x << "," << normParams_.min_y
        //           << ") Scale(" << normParams_.scale_x << "," << normParams_.scale_y
        //           << ") Res(" << normParams_.resolution_x << "," << normParams_.resolution_y << ")\n";
        return true;
    }


    //std::vector<FinalFeatureData> MapProcessor::prepareFeatureDataInternal(const ObstacleConfigMap& obstacleConfig) const {
    //    std::vector<FinalFeatureData> featuresForRasterization;
    //    if (!normParams_.valid) { std::cerr << "Error: Cannot prepare feature data, normalization invalid.\n"; return featuresForRasterization; }
    //    featuresForRasterization.reserve(intermediateObjects_.size());
    //    // std::cout << "Info: Preparing final feature data...\n";
    //    for (const auto& iData : intermediateObjects_) {
    //        const std::vector<Point>& original_points = iData.original_points; if (original_points.empty()) continue;
    //        FinalFeatureData fData; fData.original_symbol_id = iData.original_symbol_id; fData.object_type = iData.object_type;
    //        std::vector<FinalFeatureData::VertexData> current_loop; bool processing_outer_boundary = true;
    //        for (const auto& p : original_points) {
    //            bool is_hole_start_point = (p.flag & CoordFlags::HolePoint);
    //            float nx = static_cast<float>((p.x - normParams_.min_x) * normParams_.scale_x);
    //            float ny = static_cast<float>((p.y - normParams_.min_y) * normParams_.scale_y);
    //            // Clamp normalized coordinates carefully
    //            nx = std::min(static_cast<float>(config_.grid_width > 0 ? config_.grid_width - 1 : 0), std::max(0.0f, nx));
    //            ny = std::min(static_cast<float>(config_.grid_height > 0 ? config_.grid_height - 1 : 0), std::max(0.0f, ny));
    //            FinalFeatureData::VertexData vd = { {nx, ny}, p.flag };
    //            if (is_hole_start_point) {
    //                if (processing_outer_boundary) {
    //                    if (!current_loop.empty()) { fData.outer_boundary = std::move(current_loop); current_loop.clear(); }
    //                    processing_outer_boundary = false;
    //                }
    //                else {
    //                    if (current_loop.size() >= 3) fData.hole_boundaries.push_back(std::move(current_loop));
    //                    else if (!current_loop.empty()) fprintf(stderr, "Warning: Skipping hole loop < 3 points for SymID %s.\n", fData.original_symbol_id.c_str());
    //                    current_loop.clear();
    //                }
    //                current_loop.push_back(vd);
    //            }
    //            else {
    //                if (!processing_outer_boundary) {
    //                    if (current_loop.size() >= 3) fData.hole_boundaries.push_back(std::move(current_loop));
    //                    else if (!current_loop.empty()) fprintf(stderr, "Warning: Skipping hole loop < 3 points for SymID %s.\n", fData.original_symbol_id.c_str());
    //                    current_loop.clear(); processing_outer_boundary = true;
    //                }
    //                current_loop.push_back(vd);
    //            }
    //        }
    //        if (!current_loop.empty()) {
    //            if (processing_outer_boundary) fData.outer_boundary = std::move(current_loop);
    //            else {
    //                if (current_loop.size() >= 3) fData.hole_boundaries.push_back(std::move(current_loop));
    //                else fprintf(stderr, "Warning: Skipping final hole loop < 3 points for SymID %s.\n", fData.original_symbol_id.c_str());
    //            }
    //        }
    //        if (fData.outer_boundary.empty()) continue;
    //        int sym_def_type = -1; std::string isom_code_full = iData.original_symbol_id;
    //        auto def_it = symbolDefinitions_.find(iData.original_symbol_id);
    //        if (def_it != symbolDefinitions_.end()) { sym_def_type = def_it->second.symbol_type; isom_code_full = def_it->second.isom_code; }
    //        std::string base_isom_code = getBaseIsomCode(isom_code_full);
    //        auto obs_it = obstacleConfig.find(base_isom_code); fData.value = (obs_it != obstacleConfig.end()) ? obs_it->second : 1.0f;
    //        fData.specific_flags = getPathfindingFlags(base_isom_code, fData.value);
    //        featuresForRasterization.push_back(std::move(fData));
    //    }
    //    // std::cout << "Info: Prepared " << featuresForRasterization.size() << " features for Pass 2.\n";
    //    return featuresForRasterization;
    //}

    //New scanline rasterizer
    std::vector<FinalFeatureData> MapProcessor::prepareFeatureDataInternal(const ObstacleConfigMap& obstacleConfig) const {
        std::vector<FinalFeatureData> featuresForRasterization;
        if (!normParams_.valid) { std::cerr << "Error: Cannot prepare feature data, normalization invalid.\n"; return featuresForRasterization; }
        featuresForRasterization.reserve(intermediateObjects_.size());
        // std::cout << "Info: Preparing final feature data...\n";

        // --- Helper lambda for cleaning loops (remove consecutive duplicates) ---
        auto clean_loop = [](const std::vector<FinalFeatureData::VertexData>& input_loop) {
            if (input_loop.size() < 2) return input_loop; // Nothing to clean

            std::vector<FinalFeatureData::VertexData> cleaned_loop;
            cleaned_loop.reserve(input_loop.size());
            cleaned_loop.push_back(input_loop[0]); // Add first point

            for (size_t i = 1; i < input_loop.size(); ++i) {
                // Use approx_equal_float for position comparison
                if (!approx_equal_float(input_loop[i].pos.x, cleaned_loop.back().pos.x) ||
                    !approx_equal_float(input_loop[i].pos.y, cleaned_loop.back().pos.y))
                {
                    cleaned_loop.push_back(input_loop[i]);
                }
                // If positions are identical, keep the flags from the *last* vertex
                // (though flags might be ambiguous - this strategy is simple)
                else if (!cleaned_loop.empty()) {
                    cleaned_loop.back().original_flag = input_loop[i].original_flag;
                }

            }
            // Check if first and last point became identical after cleaning (for potential area closure issues)
            if (cleaned_loop.size() > 1 &&
                approx_equal_float(cleaned_loop.front().pos.x, cleaned_loop.back().pos.x) &&
                approx_equal_float(cleaned_loop.front().pos.y, cleaned_loop.back().pos.y)) {
                // Keep first point's flags, discard last identical point
                cleaned_loop.pop_back();
            }


            return cleaned_loop;
            };
        // --- End helper lambda ---


        for (const auto& iData : intermediateObjects_) {
            const std::vector<Point>& original_points = iData.original_points; if (original_points.empty()) continue;
            FinalFeatureData fData; fData.original_symbol_id = iData.original_symbol_id; fData.object_type = iData.object_type;
            std::vector<FinalFeatureData::VertexData> current_loop; bool processing_outer_boundary = true;

            for (const auto& p : original_points) {
                // ... (coordinate normalization and clamping - UNCHANGED) ...
                float nx = static_cast<float>((p.x - normParams_.min_x) * normParams_.scale_x);
                float ny = static_cast<float>((p.y - normParams_.min_y) * normParams_.scale_y);
                nx = std::min(static_cast<float>(config_.grid_width > 0 ? config_.grid_width - 1 : 0), std::max(0.0f, nx));
                ny = std::min(static_cast<float>(config_.grid_height > 0 ? config_.grid_height - 1 : 0), std::max(0.0f, ny));
                FinalFeatureData::VertexData vd = { {nx, ny}, p.flag };
                // ... (hole splitting logic - UNCHANGED) ...
                bool is_hole_start_point = (p.flag & CoordFlags::HolePoint);
                if (is_hole_start_point) {
                    if (processing_outer_boundary) {
                        if (!current_loop.empty()) { fData.outer_boundary = std::move(current_loop); current_loop.clear(); }
                        processing_outer_boundary = false;
                    }
                    else {
                        if (current_loop.size() >= 3) fData.hole_boundaries.push_back(std::move(current_loop));
                        else if (!current_loop.empty()) //fprintf(stderr, "Warning: Skipping hole loop < 3 points for SymID %s.\n", fData.original_symbol_id.c_str());
                        current_loop.clear();
                    }
                    current_loop.push_back(vd);
                }
                else {
                    if (!processing_outer_boundary) {
                        if (current_loop.size() >= 3) fData.hole_boundaries.push_back(std::move(current_loop));
                        else if (!current_loop.empty()) //fprintf(stderr, "Warning: Skipping hole loop < 3 points for SymID %s.\n", fData.original_symbol_id.c_str());
                        current_loop.clear(); processing_outer_boundary = true;
                    }
                    current_loop.push_back(vd);
                }
            }
            // Assign remaining loop
            if (!current_loop.empty()) {
                if (processing_outer_boundary) fData.outer_boundary = std::move(current_loop);
                else {
                    if (current_loop.size() >= 3) fData.hole_boundaries.push_back(std::move(current_loop));
                    else; //fprintf(stderr, "Warning: Skipping final hole loop < 3 points for SymID %s.\n", fData.original_symbol_id.c_str());
                }
            }

            // --- Apply Cleaning ---
            fData.outer_boundary = clean_loop(fData.outer_boundary);
            for (auto& hole_loop : fData.hole_boundaries) {
                hole_loop = clean_loop(hole_loop);
            }
            // Remove any hole loops that became too small after cleaning
            fData.hole_boundaries.erase(
                std::remove_if(fData.hole_boundaries.begin(), fData.hole_boundaries.end(),
                    [](const auto& loop) { return loop.size() < 3; }),
                fData.hole_boundaries.end());
            // --- End Cleaning ---


            if (fData.outer_boundary.empty()) continue; // Skip if cleaning removed everything

            // ... (Assign value and flags based on symbol - UNCHANGED) ...
            int sym_def_type = -1; std::string isom_code_full = iData.original_symbol_id;
            auto def_it = symbolDefinitions_.find(iData.original_symbol_id);
            if (def_it != symbolDefinitions_.end()) { sym_def_type = def_it->second.symbol_type; isom_code_full = def_it->second.isom_code; }
            std::string base_isom_code = getBaseIsomCode(isom_code_full);
            auto obs_it = obstacleConfig.find(base_isom_code); fData.value = (obs_it != obstacleConfig.end()) ? obs_it->second : 1.0f;
            fData.specific_flags = getPathfindingFlags(base_isom_code, fData.value);

            featuresForRasterization.push_back(std::move(fData));
        }
        // std::cout << "Info: Prepared " << featuresForRasterization.size() << " features for Pass 2.\n";
        return featuresForRasterization;
    }

    // --- Helper to determine effective type (can be reused) ---
    inline int determineEffectiveObjectType(int original_type, uint8_t flags) {
        int effective_type = original_type;
        // Override Area type to Line type if it's flagged as a road/path
        if ((flags & GridFlags::FLAG_ROAD_PATH) != 0 && effective_type == 1) {
            effective_type = 2; // Force line rasterization (no fill)
        }
        return effective_type;
    }

    std::vector<PolygonInputData> MapProcessor::preparePass1InputInternal(const std::vector<FinalFeatureData>& preparedFeatures) const {
        std::vector<PolygonInputData> pass1InputList; pass1InputList.reserve(preparedFeatures.size());
        for (const auto& fData : preparedFeatures) {
            if (fData.outer_boundary.size() >= 2) {
                std::vector<Point_float<float>> pass1_verts; pass1_verts.reserve(fData.outer_boundary.size());
                for (const auto& vd : fData.outer_boundary) pass1_verts.push_back(vd.pos);

                // *** START FIX ***
                // Determine the effective type based on original type and flags
                int effective_type = determineEffectiveObjectType(fData.object_type, fData.specific_flags);
                // Pass the effective type along with vertices and value
                pass1InputList.emplace_back(std::move(pass1_verts), fData.value, effective_type);
                // *** END FIX ***
            }
        }
        // std::cout << "Info: Prepared " << pass1InputList.size() << " geometries for Pass 1.\n";
        return pass1InputList;
    }


    //void MapProcessor::applyFeatureSpecificRulesInternal(Grid_V3& grid, const std::vector<FinalFeatureData>& features) const {
    //    // std::cout << "Info: Starting Pass 2: Applying specific feature rules...\n";
    //    if (!grid.isValid()) return;
    //    MinimalRasterizer rasterizer(grid.width(), grid.height());
    //    const float MIN_PASSABLE_VALUE = 0.01f; const float BACKGROUND_VALUE = 1.0f;
    //    for (size_t feature_idx = 0; feature_idx < features.size(); ++feature_idx) {
    //        const auto& feature = features[feature_idx];
    //        bool is_multiplicative = (feature.specific_flags & (GridFlags::FLAG_UNDERGROWTH | GridFlags::FLAG_WATER_MARSH)) != 0;
    //        float feature_value = feature.value; uint8_t feature_flags = feature.specific_flags;
    //        int effective_object_type = feature.object_type;
    //        if ((feature_flags & GridFlags::FLAG_ROAD_PATH) != 0 && effective_object_type == 1) effective_object_type = 2;
    //        auto coveredCells = rasterizer.getCoveredCells(feature.outer_boundary, feature.hole_boundaries, effective_object_type, feature.original_symbol_id);
    //        if (coveredCells.empty()) continue;
    //        // printf("--- Pass 2: Feat %zu (SymID: %s), OrigType=%d, EffType=%d, Val=%.2f, Flags=%d, Cells=%zu, Holes=%zu ---\n", feature_idx, feature.original_symbol_id.c_str(), feature.object_type, effective_object_type, feature_value, (int)feature_flags, coveredCells.size(), feature.hole_boundaries.size());
    //        for (const auto& p : coveredCells) {
    //            if (!grid.inBounds(p.x, p.y)) continue;
    //            GridCellData& gridCell = grid.at(p.x, p.y);
    //            // Use <= 0.0f for impassable check consistent with flag generation
    //            if (feature.value <= 0.0f || (feature_flags & GridFlags::FLAG_IMPASSABLE)) {
    //                gridCell.value = -1.0f; gridCell.flags |= (GridFlags::FLAG_IMPASSABLE | feature_flags); continue;
    //            }
    //            if (gridCell.value <= 0.0f || gridCell.hasFlag(GridFlags::FLAG_IMPASSABLE)) {
    //                gridCell.flags |= feature_flags; continue;
    //            }
    //            if (is_multiplicative) {
    //                if (approx_equal_float(gridCell.value, BACKGROUND_VALUE)) gridCell.value = feature_value;
    //                else { if (feature_value > 0) gridCell.value *= feature_value; else gridCell.value = MIN_PASSABLE_VALUE; }
    //                gridCell.value = std::max(MIN_PASSABLE_VALUE, gridCell.value);
    //            }
    //            else {
    //                if (approx_equal_float(gridCell.value, BACKGROUND_VALUE) && !approx_equal_float(feature_value, BACKGROUND_VALUE)) {
    //                    gridCell.value = feature_value;
    //                }
    //            }
    //            gridCell.flags |= feature_flags;
    //        }
    //    }
    //    // std::cout << "Info: Pass 2 complete.\n";
    //}
    ///////////////////////////////////////////////////moded///////////////////////////////////////////////////////////////////////
    void MapProcessor::applyFeatureSpecificRulesInternal(Grid_V3& grid, const std::vector<FinalFeatureData>& features) const {
        // std::cout << "Info: Starting Pass 2: Applying specific feature rules (Parallel Attempt)...\n";
        if (!grid.isValid() || features.empty()) {
            std::cerr << "Warning (applyFeatureSpecificRulesInternal): Grid is invalid or no features to process. Skipping Pass 2.\n";
            return;
        }

        const float MIN_PASSABLE_VALUE = 0.01f;
        const float BACKGROUND_VALUE = 1.0f;
        const size_t num_features = features.size();

#pragma omp parallel // Start parallel region
        {
            // --- Thread-Local Rasterizer ---
            // Each thread gets its own instance to avoid state conflicts
            MinimalRasterizer rasterizer(grid.width(), grid.height());

            // --- Parallel Loop over Features ---
            // Use dynamic scheduling as features might vary in complexity (vertex count, area size)
#pragma omp for schedule(dynamic)
            for (int feature_idx = 0; feature_idx < num_features; ++feature_idx) {
                const auto& feature = features[feature_idx];

                //// --- Rasterize (Thread-Local Operation) ---
                //int effective_object_type = feature.object_type;
                //// Adjust effective type for roads/paths that are areas (treat as lines for cost application maybe?)
                //if ((feature.specific_flags & GridFlags::FLAG_ROAD_PATH) != 0 && effective_object_type == 1) {
                //    effective_object_type = 2; // Example: Treat area roads like lines for rule application? Adjust if needed.
                //}
                bool is_multiplicative = (feature.specific_flags & (GridFlags::FLAG_UNDERGROWTH | GridFlags::FLAG_WATER_MARSH)) != 0;
                uint8_t feature_flags = feature.specific_flags;
                int effective_object_type = determineEffectiveObjectType(feature.object_type, feature_flags);

                // This call uses the thread-local rasterizer's state
                auto coveredCells = rasterizer.getCoveredCells(
                    feature.outer_boundary,
                    feature.hole_boundaries,
                    effective_object_type,
                    feature.original_symbol_id // Pass symbol ID for potential debug messages inside rasterizer
                );

                if (coveredCells.empty()) {
                    continue; // Skip grid update if this feature covers no cells
                }

                // --- Update Shared Grid (Synchronized Operation) ---
                // Only one thread can execute this block at a time to prevent race conditions
                // on the shared 'grid' object.
#pragma omp critical (GridUpdatePass2)
                {
                    const bool is_multiplicative = (feature.specific_flags & (GridFlags::FLAG_UNDERGROWTH | GridFlags::FLAG_WATER_MARSH)) != 0;
                    const float feature_value = feature.value;
                    const uint8_t feature_flags = feature.specific_flags;
                    const bool feature_is_impassable = (feature_value <= 0.0f || (feature_flags & GridFlags::FLAG_IMPASSABLE));

                    // Apply rules to each covered cell for this feature
                    for (const auto& p : coveredCells) {
                        // Bounds check (should be redundant if rasterizer is correct, but safe)
                        if (!grid.inBounds(p.x, p.y)) continue;

                        GridCellData& gridCell = grid.at(p.x, p.y);

                        // Rule 1: If the FEATURE is impassable, make the grid cell impassable.
                        if (feature_is_impassable) {
                            gridCell.value = -1.0f; // Standard impassable value
                            gridCell.flags |= (GridFlags::FLAG_IMPASSABLE | feature_flags); // OR flags
                            continue; // Skip other rules for this cell
                        }

                        // Rule 2: If the grid cell is ALREADY impassable, don't change its value, just OR flags.
                        if (gridCell.value <= 0.0f || gridCell.hasFlag(GridFlags::FLAG_IMPASSABLE)) {
                            gridCell.flags |= feature_flags; // Only OR flags
                            continue; // Skip other rules for this cell
                        }

                        // Rule 3 & 4: Apply feature value based on type (Multiplicative vs. Overwrite)
                        if (is_multiplicative) {
                            // Apply multiplicative cost only if grid cell is not impassable
                            if (approx_equal_float(gridCell.value, BACKGROUND_VALUE)) {
                                // If cell is background, set to feature value directly
                                gridCell.value = feature_value;
                            }
                            else {
                                // Otherwise, multiply existing cost by feature cost
                                // Avoid multiplying by zero or negative; ensure minimum passable value
                                if (feature_value > 0) {
                                    gridCell.value *= feature_value;
                                }
                                else {
                                    // If multiplicative feature has invalid value, maybe just set to min passable?
                                    gridCell.value = MIN_PASSABLE_VALUE;
                                }
                            }
                            // Clamp to minimum positive value if it became zero or negative through multiplication
                            gridCell.value = std::max(MIN_PASSABLE_VALUE, gridCell.value);
                        }
                        else {
                            // Apply overwrite cost (non-multiplicative)
                            // Only overwrite if the cell is currently background (1.0)
                            // This preserves values from potentially more important features applied earlier (like boundaries from Pass 1)
                            // Or values from other overlapping features (though order is less defined now)
                            if (approx_equal_float(gridCell.value, BACKGROUND_VALUE) && !approx_equal_float(feature_value, BACKGROUND_VALUE)) {
                                gridCell.value = feature_value;
                            }
                            // Ensure value is at least the minimum passable if it wasn't background
                            gridCell.value = std::max(MIN_PASSABLE_VALUE, gridCell.value);
                        }

                        // Rule 5: Always OR the specific flags from the feature onto the cell.
                        gridCell.flags |= feature_flags;

                    } // End loop through coveredCells
                } // End critical section for grid update
            } // End parallel for loop over features
        } // End parallel region
		std::cout << "Info: Pass 2 complete.\n";
    }

    bool MapProcessor::loadMap(const std::string& xmlFilePath) {
        mapLoaded_ = false; symbolDefinitions_.clear(); intermediateObjects_.clear(); normParams_ = NormalizationResult{};
        tinyxml2::XMLDocument doc; if (doc.LoadFile(xmlFilePath.c_str()) != tinyxml2::XML_SUCCESS) { std::cerr << "Error loading XML: " << xmlFilePath << " - " << doc.ErrorStr() << std::endl; return false; }
        tinyxml2::XMLElement* mapElement = doc.FirstChildElement("map"); if (!mapElement) { std::cerr << "Error: No <map> element in XML.\n"; return false; }
        parseSymbolDefinitionsInternal(mapElement); parseObjectGeometryInternal(mapElement);
        // if (intermediateObjects_.empty()) { std::cerr << "Warning: No relevant objects parsed.\n"; /* Continue? */ }
        mapLoaded_ = true; return true;
    }

    std::optional<Grid_V3> MapProcessor::generateGrid(const ObstacleConfigMap& obstacleConfig) {
        if (!mapLoaded_) { std::cerr << "Error: Map not loaded.\n"; return std::nullopt; }
        if (!calculateNormalizationParamsInternal()) { std::cerr << "Error: Failed normalization.\n"; return std::nullopt; }
        auto featuresForRasterization = prepareFeatureDataInternal(obstacleConfig);
        auto processorValueInputList = preparePass1InputInternal(featuresForRasterization);
        Grid_V3 pathfindingGrid(config_.grid_width, config_.grid_height);
        if (!pathfindingGrid.isValid()) { std::cerr << "Error: Failed to create valid grid.\n"; return std::nullopt; } // Check grid creation
        ParallelPolygonProcessorFlags pass1_processor;
        // std::cout << "\nInfo: Starting Pass 1...\n";
        pass1_processor.process(pathfindingGrid, processorValueInputList);
        // std::cout << "Info: Pass 1 complete.\n";
        // std::cout << "\nInfo: Starting Pass 2...\n";
        applyFeatureSpecificRulesInternal(pathfindingGrid, featuresForRasterization);
        // std::cout << "Info: Pass 2 complete.\n";
        return pathfindingGrid;
    }

} // namespace mapgeo
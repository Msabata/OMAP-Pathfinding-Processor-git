/**
 * @file ParallelProcessorFlags.cpp
 * @brief Implements the parallel boundary processing logic (Pass 1).
 */
#include "map/ParallelProcessorFlags.hpp" // Corresponding header
#include "map/MapProcessingCommon.h"    // For Grid_V3, Point_float etc.

#include <vector>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <queue>
#include <algorithm>
#include <utility>
#include <omp.h> // Crucial for parallel execution directives (#pragma omp)
#include <iostream>
#include <cassert>

namespace mapgeo {
    namespace { // Use anonymous namespace for internal linkage (helper classes/functions)

        /**
         * @brief Configuration constants used by the BoundaryOnlyRasterizer.
         */
        struct FillerConfig {
            float impassable_boundary_value = -1.0f;        // Value to set for impassable boundaries
            float background_value = 1.0f;                // Default background grid value
            GridFlags boundary_flags = FLAG_BOUNDARY;     // Flag to set on boundary cells
            GridCellData background_cell_template;        // Template cell for resetting temp grids

            FillerConfig() {
                background_cell_template.value = background_value;
                background_cell_template.flags = FLAG_NONE;
            }

            /** @brief Validates that configuration values are finite numbers. */
            bool validate() const {
                return std::isfinite(impassable_boundary_value) && std::isfinite(background_value);
            }
        };

        /**
         * @class BoundaryOnlyRasterizer
         * @brief Rasterizes only the boundary lines of polygons/lines onto a grid.
         *        Used internally by ParallelPolygonProcessorFlags during Pass 1.
         *        Does *not* handle flags like GapPoint/DashPoint or perform filling.
         * @tparam CoordType The floating-point type for input vertex coordinates (default float).
         */
        template<typename CoordType = float>
        class BoundaryOnlyRasterizer {
        public:
            /**
             * @brief Constructs the rasterizer.
             * @param config Configuration values (impassable value, background value, etc.).
             * @param grid_width Width of the target grid.
             * @param grid_height Height of the target grid.
             */
            BoundaryOnlyRasterizer(const FillerConfig& config, std::size_t grid_width, std::size_t grid_height)
                : config_(config), grid_width_(grid_width), grid_height_(grid_height)
            {
                if (!config_.validate()) {
                    throw std::invalid_argument("Invalid filler config for BoundaryOnlyRasterizer");
                }
                // Pre-allocate some space assuming boundary won't exceed perimeter roughly
                modifiedCells_.reserve(grid_width + grid_height);
            }

            /** @brief Sets the value to apply to boundary cells for the current feature. */
            void setFeatureValue(float value) {
                // Ensure the value is valid; default to 0.0f if not (though should be caught later).
                feature_value_ = std::isfinite(value) ? value : -1.0f; // Default to impassable if value is bad
            }


            /**
             * @brief Draws the boundary of the given polygon onto the temporary grid.
             * @param tempGrid The grid to draw onto (typically a thread-local temporary grid).
             * @param polygon_vertices The normalized vertices of the polygon/line boundary.
             * @return A vector of integer grid points that were modified by this operation.
             */
            const std::vector<IntPoint>& processBoundary(Grid_V3& tempGrid, const std::vector<Point_float<CoordType>>& polygon_vertices, int effective_object_type) {
                modifiedCells_.clear(); // Clear results from previous feature
                if (polygon_vertices.size() < 2) {
                    return modifiedCells_; // Need at least two points for a line
                }

                // Convert float vertices to integer grid points, removing duplicates
                auto grid_poly = convertToIntPoints(polygon_vertices);

                if (grid_poly.size() >= 2) {
                    //drawPolygonEdges(tempGrid, grid_poly);
                    drawPolygonEdges(tempGrid, grid_poly, effective_object_type);
                }
                return modifiedCells_;
            }

        private:
            const FillerConfig& config_;        // Rasterizer configuration
            std::size_t grid_width_;            // Target grid width
            std::size_t grid_height_;           // Target grid height
            std::vector<IntPoint> modifiedCells_; // Stores grid cells modified in the last call
            float feature_value_ = 1.0f;      // Value to apply for the current feature

            /** @brief Checks if integer coordinates (x, y) are within the grid bounds. */
            inline bool inBounds(int x, int y) const {
                return static_cast<unsigned>(x) < grid_width_ && static_cast<unsigned>(y) < grid_height_; // Use grid_height_
            }


            /**
             * @brief Converts floating-point vertices to integer grid coordinates, clamping to bounds
             *        and removing consecutive duplicate points.
             * @param poly The input vector of normalized floating-point vertices.
             * @return A vector of integer grid points.
             */
            std::vector<IntPoint> convertToIntPoints(const std::vector<Point_float<CoordType>>& poly) const {
                std::vector<IntPoint> result;
                if (poly.empty()) return result;

                result.reserve(poly.size());
                int max_x_idx = static_cast<int>(grid_width_ - 1);
                int max_y_idx = static_cast<int>(grid_height_ - 1); // Use grid_height_
                if (max_x_idx < 0) max_x_idx = 0; // Handle 1-cell wide grid case
                if (max_y_idx < 0) max_y_idx = 0; // Handle 1-cell high grid case

                for (const auto& p : poly) {
                    // Floor coordinates to get the integer cell index
                    int ix = static_cast<int>(std::floor(p.x));
                    int iy = static_cast<int>(std::floor(p.y));

                    // Clamp coordinates to be within grid boundaries
                    ix = std::max(0, std::min(ix, max_x_idx));
                    iy = std::max(0, std::min(iy, max_y_idx));

                    // Add the point only if it's different from the last one added
                    if (result.empty() || !(result.back().x == ix && result.back().y == iy)) {
                        result.push_back({ ix, iy });
                    }
                }
                return result;
            }

            /**
             * @brief Draws all edges of a polygon (represented by integer points) onto the grid.
             *        Connects the last point back to the first.
             * @param grid The grid to draw onto.
             * @param polygon The vertices of the polygon as integer points.
             */
            void drawPolygonEdges(Grid_V3& grid, const std::vector<IntPoint>& polygon, int effective_object_type) {
                if (polygon.size() < 2) return;
                for (std::size_t i = 0; i < polygon.size() - 1; ++i) {
                    bresenhamLine(grid, polygon[i], polygon[i + 1]);
                }

                // Draw the closing segment ONLY if it's an Area type (1) and has enough points
                bool close_loop = (effective_object_type == 1 && polygon.size() >= 3);
                if (close_loop) {
                    bresenhamLine(grid, polygon.back(), polygon.front()); // Connect last to first
                }
            }

            /**
             * @brief Draws a line segment between two integer points using Bresenham's line algorithm.
             *        Sets the grid cell value and boundary flag for each point on the line.
             * @param grid The grid to modify.
             * @param p1 Starting point of the line.
             * @param p2 Ending point of the line.
             */
            void bresenhamLine(Grid_V3& grid, IntPoint p1, IntPoint p2) {
                int dx = std::abs(p2.x - p1.x), sx = (p1.x < p2.x) ? 1 : -1;
                int dy = -std::abs(p2.y - p1.y), sy = (p1.y < p2.y) ? 1 : -1;
                int err = dx + dy; // error value e_xy

                while (true) {
                    if (inBounds(p1.x, p1.y)) {
                        GridCellData& cell = grid.at(p1.x, p1.y);
                        float value_to_set;

                        // Determine the value: impassable takes priority
                        if (feature_value_ <= 0.0f) { // Feature itself is impassable (or invalid)
                            value_to_set = config_.impassable_boundary_value; // Should be -1.0f
                        }
                        else {
                            value_to_set = feature_value_; // Use the feature's base terrain cost
                        }

                        // Update cell only if value or flag changes (optimization)
                        if (!approx_equal_float(cell.value, value_to_set) || !cell.hasFlag(config_.boundary_flags)) {
                            cell.value = value_to_set;
                            cell.flags = config_.boundary_flags; // ONLY set boundary flag in Pass 1
                            modifiedCells_.push_back({ p1.x, p1.y }); // Record modification
                        }
                    }

                    if (p1.x == p2.x && p1.y == p2.y) break; // Reached the end point

                    int e2 = 2 * err;
                    if (e2 >= dy) { // e_xy+e_x > 0
                        if (p1.x == p2.x && p1.y == p2.y) break; // Check again after potential increment
                        err += dy;
                        p1.x += sx;
                    }
                    if (e2 <= dx) { // e_xy+e_y < 0
                        if (p1.x == p2.x && p1.y == p2.y) break; // Check again after potential increment
                        err += dx;
                        p1.y += sy;
                    }
                }
            }
        }; // End class BoundaryOnlyRasterizer

    } // anonymous namespace

    // --- Public Method Implementation ---
    void ParallelPolygonProcessorFlags::process(Grid_V3& finalGrid, const std::vector<PolygonInputData>& polygonDataList) const {
        const std::size_t grid_width = finalGrid.width();
        const std::size_t grid_height = finalGrid.height();

        if (!finalGrid.isValid()) { // Check if grid dimensions are valid
            std::cerr << "Error (ParallelProcessorFlags): Process called with zero-dimension grid." << std::endl;
            return;
        }


        const FillerConfig config; // Use default config for Pass 1

        // Filter out invalid polygons (less than 2 vertices) beforehand
        std::vector<PolygonInputData> validPolygonData;
        validPolygonData.reserve(polygonDataList.size());
        for (const auto& data : polygonDataList) {
            if (data.polygon_vertices.size() >= 2) {
                validPolygonData.push_back(data);
            }
        }

        const std::size_t num_valid_polygons = validPolygonData.size();
        if (num_valid_polygons == 0) {
            std::cout << "Info (ParallelProcessorFlags): No valid lines/polygons (>= 2 vertices) found for Pass 1 processing." << std::endl;
            return;
        }

        // Start parallel region
#pragma omp parallel
        {
            // Thread-local resources: each thread gets its own temporary grid and rasterizer
            Grid_V3 tempGrid(grid_width, grid_height, config.background_cell_template);
            BoundaryOnlyRasterizer<float> boundary_rasterizer(config, grid_width, grid_height);

            // Distribute polygon processing among threads
            // dynamic schedule useful if polygons have vastly different vertex counts
#pragma omp for schedule(dynamic)
            for (long long i = 0; i < static_cast<long long>(num_valid_polygons); ++i) {
                const auto& polyData = validPolygonData[i];

                // Reset temp grid for the current polygon (optional, could reuse but reset is safer)
                tempGrid.reset(config.background_cell_template);

                // Set the value this feature will draw
                boundary_rasterizer.setFeatureValue(polyData.replacement_value);

                // Draw the boundary onto the thread-local temporary grid
                const std::vector<IntPoint>& modifiedBoundaryCoords = boundary_rasterizer.processBoundary(
                    tempGrid,
                    polyData.polygon_vertices,
                    polyData.effective_object_type // <-- Pass the type here
                );

                // --- Critical Section: Merge results onto the shared final grid ---
                // Only one thread can execute this block at a time to prevent race conditions
#pragma omp critical (FinalGridUpdate)
                {
                    for (const auto& p : modifiedBoundaryCoords) {
                        // Double-check bounds just in case (should be guaranteed by rasterizer)
                        if (finalGrid.inBounds(p.x, p.y)) {
                            const GridCellData& temp_cell = tempGrid.at(p.x, p.y); // Cell from thread's rasterization
                            GridCellData& final_cell = finalGrid.at(p.x, p.y);   // Corresponding cell in the shared grid

                            bool temp_is_impassable = temp_cell.value <= 0.0f; // Use <= 0 as impassable marker
                            bool final_is_impassable = final_cell.value <= 0.0f;

                            // Rule 1: Impassable boundaries from temp grid always overwrite final grid
                            if (temp_is_impassable) {
                                final_cell.value = config.impassable_boundary_value; // Standardize to -1.0f
                                final_cell.setFlag(config.boundary_flags); // Ensure flag is set
                                continue; // Skip other rules
                            }

                            // Rule 2: If final grid cell is already impassable, don't change value
                            if (final_is_impassable) {
                                final_cell.setFlag(config.boundary_flags); // Still set flag if temp wasn't impassable
                                continue; // Skip other rules
                            }

                            // Rule 3: Apply temp value if final is background OR temp value has larger magnitude
                            // Compare absolute values, ignore sign (since both are positive here)
                            if (approx_equal_float(final_cell.value, config.background_value) ||
                                std::fabs(temp_cell.value) > std::fabs(final_cell.value))
                            {
                                final_cell.value = temp_cell.value;
                            }

                            // Rule 4: Always set the boundary flag if we modified this cell in temp grid
                            final_cell.setFlag(config.boundary_flags);
                        }
                    } // End loop through modified coordinates
                } // End critical section
            } // End parallel for loop
        } // End parallel region

    } // End ParallelPolygonProcessorFlags::process

} // namespace mapgeo
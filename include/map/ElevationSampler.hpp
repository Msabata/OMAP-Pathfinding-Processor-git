// File: ElevationSampler.hpp
#ifndef ELEVATION_SAMPLER_HPP
#define ELEVATION_SAMPLER_HPP

#include <vector>
#include <cmath>
#include <stdexcept>
#include <string>
#include <algorithm> // for std::min/max

/**
 * @brief Handles sampling elevation data from a grid using bilinear interpolation.
 *        Designed to work with elevation grids that may have different dimensions,
 *        resolution, and origin offset compared to the main pathfinding grid.
 */
struct ElevationSampler {
    const std::vector<float>& elev_data;
    const int elev_width;
    const int elev_height;
    const float elev_resolution; // Real-world size of one elevation cell edge
    const float origin_x_offset; // Real-world offset: elevation_origin_x - logical_origin_x
    const float origin_y_offset; // Real-world offset: elevation_origin_y - logical_origin_y
    const float inv_elev_resolution; // Precomputed for efficiency

    /**
     * @brief Constructs the ElevationSampler.
     * @param data Reference to the vector containing elevation values (row-major).
     * @param width Width of the elevation grid.
     * @param height Height of the elevation grid.
     * @param resolution Real-world distance between elevation cell centers.
     * @param ox Real-world X offset of elevation grid origin relative to logical grid origin.
     * @param oy Real-world Y offset of elevation grid origin relative to logical grid origin.
     * @throws std::invalid_argument If dimensions, resolution, or data size are invalid.
     */
    ElevationSampler(const std::vector<float>& data, int width, int height, float resolution, float ox = 0.0f, float oy = 0.0f)
        : elev_data(data), elev_width(width), elev_height(height), elev_resolution(resolution),
        origin_x_offset(ox), origin_y_offset(oy), inv_elev_resolution(1.0f / resolution) // Precompute inverse
    {
        if (width <= 0 || height <= 0) {
            throw std::invalid_argument("ElevationSampler: Dimensions must be positive. Got " + std::to_string(width) + "x" + std::to_string(height));
        }
        if (resolution <= 1e-6f) { // Use epsilon for float comparison
            throw std::invalid_argument("ElevationSampler: Resolution must be positive. Got " + std::to_string(resolution));
        }
        if (static_cast<size_t>(width * height) != data.size()) {
            throw std::invalid_argument("ElevationSampler: Data size (" + std::to_string(data.size()) + ") does not match dimensions (" + std::to_string(width) + "x" + std::to_string(height) + "=" + std::to_string(width * height) + ")");
        }
    }

    /**
     * @brief Samples elevation at given real-world coordinates using bilinear interpolation.
     *        Handles boundary conditions by clamping coordinates to the edge of the elevation grid.
     * @param world_x Real-world X coordinate where elevation is needed.
     * @param world_y Real-world Y coordinate where elevation is needed.
     * @return Interpolated elevation value. Returns edge elevation if coords are outside bounds.
     */
    float getElevationAt(float world_x, float world_y) const {
        // Convert world coords relative to elevation grid origin
        float rel_x = world_x - origin_x_offset;
        float rel_y = world_y - origin_y_offset;

        // Convert relative world coords to fractional elevation grid indices
        // Multiplying by inverse is often faster than division
        float grid_x_f = rel_x * inv_elev_resolution;
        float grid_y_f = rel_y * inv_elev_resolution;

        // Get integer base indices (top-left corner of the sampling square)
        int x0 = static_cast<int>(std::floor(grid_x_f));
        int y0 = static_cast<int>(std::floor(grid_y_f));

        // Calculate interpolation weights (fractional parts)
        float tx = grid_x_f - static_cast<float>(x0);
        float ty = grid_y_f - static_cast<float>(y0);

        // --- Clamp Coordinates to Grid Edges ---
        // Find indices of the 4 surrounding grid points, clamping them within bounds
        int ix0 = std::max(0, std::min(x0, elev_width - 1));
        int iy0 = std::max(0, std::min(y0, elev_height - 1));
        int ix1 = std::max(0, std::min(x0 + 1, elev_width - 1));
        int iy1 = std::max(0, std::min(y0 + 1, elev_height - 1));

        // --- Get Elevations of the 4 corner points ---
        // Use size_t for indexing vectors
        size_t idx00 = static_cast<size_t>(iy0) * elev_width + ix0;
        size_t idx10 = static_cast<size_t>(iy0) * elev_width + ix1;
        size_t idx01 = static_cast<size_t>(iy1) * elev_width + ix0;
        size_t idx11 = static_cast<size_t>(iy1) * elev_width + ix1;

        // Basic safety check on indices (shouldn't be needed if clamping works, but defense-in-depth)
        if (idx00 >= elev_data.size() || idx10 >= elev_data.size() || idx01 >= elev_data.size() || idx11 >= elev_data.size()) {
            size_t clamped_idx = static_cast<size_t>(iy0) * elev_width + ix0;
            return (clamped_idx < elev_data.size()) ? elev_data[clamped_idx] : 0.0f;
        }

        float Q00 = elev_data[idx00]; // Top-left
        float Q10 = elev_data[idx10]; // Top-right
        float Q01 = elev_data[idx01]; // Bottom-left
        float Q11 = elev_data[idx11]; // Bottom-right

        // --- Perform Bilinear Interpolation ---
        tx = std::max(0.0f, std::min(tx, 1.0f));
        ty = std::max(0.0f, std::min(ty, 1.0f));

        float top_interp = Q00 * (1.0f - tx) + Q10 * tx;
        float bottom_interp = Q01 * (1.0f - tx) + Q11 * tx;
        float final_elev = top_interp * (1.0f - ty) + bottom_interp * ty;

        return final_elev;
    }
};

#endif // ELEVATION_SAMPLER_HPP
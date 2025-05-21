#pragma once
#ifndef PATHFINDING_UTILS_HPP
#define PATHFINDING_UTILS_HPP

#include <vector>
#include <cmath>    // For sqrtf, fabsf, expf
#include <cstdlib>  // For std::abs (int version)
#include <limits>   // For numeric_limits
#include <stdexcept> // For invalid_argument
#include <algorithm> // For std::min/max needed by diagonal heuristic

// Include the sampler definition - NOT needed here directly, but A* needs it.
// #include "ElevationSampler.hpp"

// Define GridPoint here or include the header where it's defined (e.g., MapProcessingCommon.h if IntPoint is used)
struct GridPoint {
    int x = 0;
    int y = 0;
    bool operator==(const GridPoint& other) const {
        return x == other.x && y == other.y;
    }
     // Add a hash function if needed for unordered sets/maps
     struct Hash {
        std::size_t operator()(const GridPoint& p) const {
            return static_cast<std::size_t>(p.x) * 73856093 ^ static_cast<std::size_t>(p.y) * 19349663;
        }
    };
};


namespace PathfindingUtils {

    // --- Constants ---
    inline constexpr int NUM_DIRECTIONS = 8;
    inline constexpr int dx[NUM_DIRECTIONS] = { 1, 0, -1, 0, 1, -1, -1, 1 };
    inline constexpr int dy[NUM_DIRECTIONS] = { 0, 1, 0, -1, 1, 1, -1, -1 };
    inline constexpr float costs[NUM_DIRECTIONS] = { // Base geometric costs (Distance factor)
        1.0f, 1.0f, 1.0f, 1.0f,
        1.41421356f, 1.41421356f, 1.41421356f, 1.41421356f
    };
    inline constexpr float EPSILON = 1e-6f; // Small value for float checks
    // Heuristic type constants
    inline constexpr int HEURISTIC_EUCLIDEAN = 0;
    inline constexpr int HEURISTIC_DIAGONAL = 1;
    inline constexpr int HEURISTIC_MANHATTAN = 2;
    inline constexpr int HEURISTIC_MIN_COST = 3; // Scaled Diagonal by min combined cost factor

    // --- Internal Heuristic Implementations ---
    namespace internal {
        inline float euclidean_distance(int x1, int y1, int x2, int y2) {
            float dx_h = static_cast<float>(x1 - x2);
            float dy_h = static_cast<float>(y1 - y2);
            return sqrtf(dx_h * dx_h + dy_h * dy_h);
        }

        inline float diagonal_distance(int x1, int y1, int x2, int y2) {
            float dx_h = std::fabs(static_cast<float>(x1 - x2));
            float dy_h = std::fabs(static_cast<float>(y1 - y2));
            // Use constants defined above for costs
            return costs[0] * (dx_h + dy_h) + (costs[4] - 2.0f * costs[0]) * std::min(dx_h, dy_h);
        }


        inline float manhattan_distance(int x1, int y1, int x2, int y2) {
            return static_cast<float>(std::abs(x1 - x2) + std::abs(y1 - y2));
        }
    } // namespace internal


    /**
     * @brief Calculates the heuristic estimate between two points based on type.
     *        Note: Heuristic operates on GRID coordinates.
     */
    inline float calculate_heuristic(int x1, int y1, int x2, int y2, int heuristic_type) {
        switch (heuristic_type) {
            case HEURISTIC_DIAGONAL:
                return internal::diagonal_distance(x1, y1, x2, y2);
            case HEURISTIC_MANHATTAN:
                return internal::manhattan_distance(x1, y1, x2, y2);
            case HEURISTIC_MIN_COST: // Scaled Diagonal * min cost factor
                // Minimum possible combined multiplier (terrain * slope_penalty) identified as 0.8
                return internal::diagonal_distance(x1, y1, x2, y2) * 0.8f;
            case HEURISTIC_EUCLIDEAN: // Euclidean (Default)
            default:
                return internal::euclidean_distance(x1, y1, x2, y2);
        }
    }

    // --- Coordinate/Index Helpers ---
    inline int toIndex(int x, int y, int width) {
        // No bounds check here for performance; caller ensures validity
        return y * width + x;
    }

    inline void toCoords(int index, int width, int& x, int& y) {
        if (width <= 0) {
            x = -1; y = -1; // Indicate error
        } else {
             y = index / width;
             x = index % width;
        }
    }

} // namespace PathfindingUtils

#endif // PATHFINDING_UTILS_HPP
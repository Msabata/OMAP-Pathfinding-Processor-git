
/**
 * @file ParallelProcessorFlags.hpp
 * @brief Defines structures and the class responsible for the parallel
 *        processing phase (Pass 1), which focuses on drawing feature boundaries.
 */
#ifndef PARALLEL_PROCESSOR_FLAGS_H
#define PARALLEL_PROCESSOR_FLAGS_H

#include "MapProcessingCommon.h" // Include common definitions
#include <vector>
#include <omp.h>                 // Include OpenMP header for parallel processing directives

namespace mapgeo {

    /**
     * @brief Simplified input data structure used specifically for Pass 1 processing.
     *        Contains only the outer boundary vertices, the base feature value,
     *        and the effective object type for rasterization.
     */
    struct PolygonInputData {
        std::vector<Point_float<float>> polygon_vertices; // Normalized outer boundary vertices ONLY
        float replacement_value = 1.0f;                   // Base terrain cost value for this polygon/line
        int effective_object_type = 2;                    // 0=Point, 1=Area(Closed), 2=Line(Open)

        PolygonInputData() = default;
        // Update constructor if you have one, or use direct initialization
        PolygonInputData(std::vector<Point_float<float>> vertices, float value, int type) // Added type
            : polygon_vertices(std::move(vertices)), replacement_value(value), effective_object_type(type) {}
    };

    /**
     * @class ParallelPolygonProcessorFlags
     * @brief Executes Pass 1 of the grid generation.
     *        Processes polygon/line boundaries in parallel using OpenMP to establish
     *        initial boundary values (impassable or feature base terrain cost) on the grid.
     *        This pass only sets the FLAG_BOUNDARY flag.
     */
    class ParallelPolygonProcessorFlags {
    public:
        ParallelPolygonProcessorFlags() = default;

        // Disable copy/move constructors and assignments (not needed, avoids accidental copies)
        ParallelPolygonProcessorFlags(const ParallelPolygonProcessorFlags&) = delete;
        ParallelPolygonProcessorFlags& operator=(const ParallelPolygonProcessorFlags&) = delete;
        ParallelPolygonProcessorFlags(ParallelPolygonProcessorFlags&&) = delete;
        ParallelPolygonProcessorFlags& operator=(ParallelPolygonProcessorFlags&&) = delete;

        /**
         * @brief Processes a list of polygon/line data in parallel to draw boundaries onto the final grid.
         * @param finalGrid The target Grid_V3 object to modify.
         * @param polygonDataList A list of PolygonInputData representing features to process.
         */
        void process(Grid_V3& finalGrid, const std::vector<PolygonInputData>& polygonDataList) const;
    };

} // namespace mapgeo
#endif // PARALLEL_PROCESSOR_FLAGS_H
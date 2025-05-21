#include "logic/PathfindingLogic.hpp"

// --- Standard Library Includes ---
#include <stdexcept>
#include <chrono>
#include <iostream> // For std::cerr/cout (optional, prefer qDebug)
#include <fstream>  // If doing file copy manually
#include <cmath>
#include <iterator> // For std::make_move_iterator

// --- Qt Includes ---
#include <QDebug>   // For logging
#include <QFileInfo> // Useful for path manipulation if needed

// --- Backend Includes ---
#include "map/MapProcessor.hpp"
#include "map/GeoRefScanner.hpp"
#include "map/WaypointExtractor.hpp"
#include "map/ElevationFetcherPy.hpp" // Includes Python interaction
#include "map/PathfindingUtils.hpp"   // Includes GridPoint definition, constants
// #include "debug/DebugUtils.hpp"    // Optional for backend debugging

// --- Algorithm Includes ---
#include "algoritms/AStarToblerSampled.hpp"
#include "algoritms/DijkstraToblerSampled.hpp"
#include "algoritms/BFSToblerSampled.hpp"
#include "algoritms/ThetaStarToblerSampled.hpp"
#include "algoritms/LazyThetaStarToblerSampled.hpp"

#ifdef USE_CUDA
//#include "algoritms/DeltaSteppingGPU.hpp"
//#include "algoritms/HADS_GPU.hpp"
//#include "algoritms/AStarGPU.hpp"
//
//// *** ADD extern "C" declarations HERE for the GPU functions ***
//// *** These match the headers but are wrapped for linkage ***
//extern "C" {
//    std::vector<int> findPathGPU_DeltaStepping(
//        const mapgeo::Grid_V3& logical_grid, const std::vector<float>& elevation_values,
//        int elevation_width, int elevation_height, float log_cell_resolution, float elev_cell_resolution,
//        float origin_offset_x, float origin_offset_y, const GridPoint& start, const GridPoint& end,
//        float delta, float threshold);
//
//    std::vector<int> findPathGPU_HADS(
//        const mapgeo::Grid_V3& logical_grid, const std::vector<float>& elevation_values,
//        int elevation_width, int elevation_height, float log_cell_resolution, float elev_cell_resolution,
//        float origin_offset_x, float origin_offset_y, const GridPoint& start, const GridPoint& end,
//        float delta_param, float light_edge_threshold_param, int heuristic_radius_cells,
//        float prune_factor, float heuristic_weight);
//
//    std::vector<int> findPathGPU_AStar(
//        const mapgeo::Grid_V3& logical_grid, const std::vector<float>& elevation_values,
//        int elevation_width, int elevation_height, float log_cell_resolution, float elev_cell_resolution,
//        float origin_offset_x, float origin_offset_y, const GridPoint& start, const GridPoint& end,
//        float heuristic_weight, int max_iterations);
//} // End extern "C" declarations for PathfindingLogic's use

#endif // USE_CUDA


#ifdef _OPENMP
#include <omp.h>
#endif

// --- Using namespaces for brevity (use with caution in headers) ---
using namespace mapgeo;
using namespace Pathfinding;
using namespace PathfindingUtils;
using namespace mapscan;
using namespace ElevationFetcher;
using namespace waypoint;

namespace app {

    PathfindingLogic::PathfindingLogic() = default;
    PathfindingLogic::~PathfindingLogic() = default;

    //-----------------------------------------------------------------------------
    // Main processing function
    //-----------------------------------------------------------------------------
    BackendResult PathfindingLogic::processAndFindPath(const BackendInputParams& params) {
        BackendResult result;
        result.success = false; // Assume failure initially
        result.usedMapFilePath = params.mapFilePath;
        result.usedGridWidth = params.desiredGridWidth;
        result.usedGridHeight = params.desiredGridHeight;
        result.currentControlsFilePath = params.controlsFilePath; // Store for GUI

        std::optional<Grid_V3> logical_grid_opt; // Use local optionals
        std::optional<NormalizationResult> normInfo_opt;

        try {
            qDebug() << "PathfindingLogic: Starting processing. Map:" << QString::fromStdString(params.mapFilePath)
                << "Controls:" << QString::fromStdString(params.controlsFilePath);

#ifdef _OPENMP
            omp_set_num_threads(params.numThreads);
            qDebug() << "PathfindingLogic: Set OpenMP threads to" << params.numThreads;
#endif

            auto start_full_proc = std::chrono::high_resolution_clock::now();

            //--------------------------------------------
            // 1. Map Scan & Processing
            //--------------------------------------------
            auto start_map_proc = std::chrono::high_resolution_clock::now();

            // Scan for georef - needed for elevation fetching decision
            // Layers needed might depend on where georef info is stored
            std::vector<std::string> layers_for_scan = { "barrier", "course" };
            mapscan::ScanResult scanResult = mapscan::scanXmlForGeoRefAndBounds(params.mapFilePath, layers_for_scan);
            bool canFetchElevation = false;
            bool useRealElevation = true; // Assume dummy unless successfully fetched
            double mapScaleFromXml = 10000.0; // Default scale
            if (scanResult.georeferencingFound && scanResult.refLatLon &&
                scanResult.rawBoundsUM && scanResult.rawBoundsUM->initialized && scanResult.mapScale)
            {
                canFetchElevation = true; // Potential to fetch
                mapScaleFromXml = scanResult.mapScale.value();
                qDebug() << "PathfindingLogic: GeoRef found. Scale:" << mapScaleFromXml;
            }
            else {
                qWarning() << "PathfindingLogic: GeoRef info incomplete in map file.";
                if (!scanResult.rawBoundsUM || !scanResult.rawBoundsUM->initialized) {
                    throw std::runtime_error("Could not determine coordinate bounds from map file.");
                }
            }

            // Process Grid (reuse or generate)
            if (params.reuseGridIfPossible && params.existingGrid.has_value() && params.existingNormInfo.has_value()) {
                qDebug() << "PathfindingLogic: Reusing existing grid.";
                logical_grid_opt = params.existingGrid; // Copy from input
                normInfo_opt = params.existingNormInfo; // Copy from input
            }
            else {
                qDebug() << "PathfindingLogic: Processing map and generating grid...";
                MapProcessorConfig procConfig;
                procConfig.grid_width = params.desiredGridWidth;
                procConfig.grid_height = params.desiredGridHeight;
                procConfig.layers_to_process = { "barrier", "course" }; // Layers for actual features
                MapProcessor processor(procConfig);
                if (!processor.loadMap(params.mapFilePath)) {
                    throw std::runtime_error("Map load failed: " + params.mapFilePath);
                }
                logical_grid_opt = processor.generateGrid(params.obstacleCosts);
                if (!logical_grid_opt) { throw std::runtime_error("Grid generation failed"); }
                normInfo_opt = processor.getNormalizationResult();
                if (!normInfo_opt || !normInfo_opt->valid) { throw std::runtime_error("Normalization results invalid after grid generation."); }
                qDebug() << "PathfindingLogic: New grid generated.";
            }
            auto end_map_proc = std::chrono::high_resolution_clock::now();
            result.mapProcessingDurationMs = std::chrono::duration<double, std::milli>(end_map_proc - start_map_proc).count();
            qDebug() << "PathfindingLogic: Map processing took" << result.mapProcessingDurationMs << "ms.";

            // Ensure we have grid and norm info to proceed
            if (!logical_grid_opt || !normInfo_opt) {
                throw std::runtime_error("Logical grid or normalization info is missing after processing step.");
            }
            // Store the definitive grid/norm info in the result
            result.processedGrid = logical_grid_opt;
            result.normalizationInfo = normInfo_opt;
            const auto& finalNormInfo = normInfo_opt.value(); // Use reference now

            //double real_world_min_x = finalNormInfo.min_x;
            //double real_world_min_y = finalNormInfo.min_y;
            double real_world_max_x = finalNormInfo.min_x + (params.desiredGridWidth * finalNormInfo.resolution_x);
            double real_world_max_y = finalNormInfo.min_y + (params.desiredGridHeight * finalNormInfo.resolution_y);

            //--------------------------------------------
            // 2. Waypoint Extraction
            //--------------------------------------------
            qDebug() << "PathfindingLogic: Extracting waypoints...";
            std::optional<std::vector<GridPoint>> waypointsOpt = waypoint::extractWaypointsFromFile(
                params.controlsFilePath,
                finalNormInfo.min_x, real_world_max_x, // Use bounds from actual processed grid
                finalNormInfo.min_y, real_world_max_y,
                params.desiredGridWidth, params.desiredGridHeight
            );

            if (!waypointsOpt || waypointsOpt.value().size() < 2) {
                throw std::runtime_error("Failed to extract valid Start/Control/End sequence from controls file: " + params.controlsFilePath);
            }
            const std::vector<GridPoint>& waypoints = waypointsOpt.value();
            result.waypointsFound = waypoints.size();
            qDebug() << "PathfindingLogic: Extracted" << result.waypointsFound << "waypoints.";

            //--------------------------------------------
            // 3. Elevation Fetching & Param Calculation
            //--------------------------------------------
            qDebug() << "PathfindingLogic: Preparing elevation data...";
            auto start_elev_fetch = std::chrono::high_resolution_clock::now();
            ElevationData elevationResult;
            

            if (canFetchElevation) { // Only attempt if scan was successful
                qDebug() << "PathfindingLogic: Attempting Python elevation fetch...";
                const auto& rawBounds = scanResult.rawBoundsUM.value(); // Safe now
                const auto& anchorLatLon = scanResult.refLatLon.value(); // Safe now
                double anchorInternalX = 0.0; // Relative anchor coords
                double anchorInternalY = 0.0;

                elevationResult = fetchElevationDataEmbedded(
                    params.pyModuleName, params.pyFetchFuncName,
                    anchorLatLon.y, anchorLatLon.x,
                    anchorInternalX, anchorInternalY,
                    rawBounds.min_x, rawBounds.min_y,
                    rawBounds.max_x, rawBounds.max_y,
                    mapScaleFromXml,
                    params.desiredElevationResolution
                );
                auto end_elev_fetch = std::chrono::high_resolution_clock::now();
                result.elevationFetchDurationMs = std::chrono::duration<double, std::milli>(end_elev_fetch - start_elev_fetch).count();

                if (elevationResult.success && elevationResult.hasData()) {
                    qDebug() << "PathfindingLogic: Python fetch successful (" << result.elevationFetchDurationMs << "ms).";
                    useRealElevation = true;
                }
                else {
                    qWarning() << "PathfindingLogic: Python elevation fetch failed. Reason:" << QString::fromStdString(elevationResult.errorMessage);
                    // Keep useRealElevation = false
                }
            }
            else {
                qDebug() << "PathfindingLogic: Skipping Python elevation fetch (GeoRef info incomplete).";
            }
            result.elevationDataUsed = elevationResult; // Store result even if failed/dummy parts

            // Calculate final parameters
            std::vector<float> elevation_values_final;
            int elevation_width_final = params.desiredGridWidth;
            int elevation_height_final = params.desiredGridHeight;
            double elevation_resolution_final_dbl = 1.0;
            double elevation_origin_final_x = 0.0;
            double elevation_origin_final_y = 0.0;
            float origin_offset_x = 0.0f;
            float origin_offset_y = 0.0f;
            float log_cell_resolution_meters = 1.0f;
            double logical_origin_internal_x_um = finalNormInfo.min_x;
            double logical_origin_internal_y_um = finalNormInfo.min_y;
            double logical_res_internal_units_um = (real_world_max_x - finalNormInfo.min_x) / params.desiredGridWidth; // Approx if needed

            double meters_per_internal_unit = mapScaleFromXml / 1000000.0;
            log_cell_resolution_meters = static_cast<float>(logical_res_internal_units_um * meters_per_internal_unit);
            if (log_cell_resolution_meters <= 1e-6f) {
                log_cell_resolution_meters = 1.0f; // Fallback
                qWarning() << "PathfindingLogic: Calculated logical resolution near zero, using fallback 1.0m.";
            }
            qDebug() << "PathfindingLogic: Final Logical Cell Res (m):" << log_cell_resolution_meters;

            if (useRealElevation) {
                elevation_values_final = std::move(elevationResult.values); // Move data
                elevation_width_final = elevationResult.width;
                elevation_height_final = elevationResult.height;
                elevation_resolution_final_dbl = elevationResult.resolution_meters;
                elevation_origin_final_x = elevationResult.origin_proj_x;
                elevation_origin_final_y = elevationResult.origin_proj_y;

                // Convert anchor to projected CRS for offset calculation
                const auto& anchorLatLon = scanResult.refLatLon.value();
                ProjectedPointResult anchorProj = convertLatLonToProjectedViaPython(
                    params.pyModuleName, params.pyConvertFuncName, anchorLatLon.x, anchorLatLon.y
                );

                if (anchorProj.success) {
                    double known_proj_x = anchorProj.x; double known_proj_y = anchorProj.y;
                    double origin_offset_x_m = (logical_origin_internal_x_um - 0.0) * meters_per_internal_unit; // Offset from anchor
                    double origin_offset_y_m = (logical_origin_internal_y_um - 0.0) * meters_per_internal_unit * -1.0; // Y-invert
                    double logicalOriginProjX = known_proj_x + origin_offset_x_m;
                    double logicalOriginProjY = known_proj_y + origin_offset_y_m;
                    origin_offset_x = static_cast<float>(elevation_origin_final_x - logicalOriginProjX);
                    origin_offset_y = static_cast<float>(elevation_origin_final_y - logicalOriginProjY);
                    qDebug() << "PathfindingLogic: Calculated origin offset (m): X=" << origin_offset_x << "Y=" << origin_offset_y;
                }
                else {
                    qWarning() << "PathfindingLogic: Could not convert anchor Lat/Lon:" << QString::fromStdString(anchorProj.error) << ". Using zero offset.";
                    useRealElevation = false; // Fallback to dummy if conversion fails
                    origin_offset_x = 0.0f;
                    origin_offset_y = 0.0f;
                }
            }


            // Setup dummy data if real failed or wasn't attempted
            result.usedDummyElevation = !useRealElevation;
            if (!useRealElevation) {
                qDebug() << "PathfindingLogic: Using dummy elevation grid.";
                elevation_values_final.assign(static_cast<size_t>(params.desiredGridWidth) * params.desiredGridHeight, 100.0f);
                elevation_width_final = params.desiredGridWidth;
                elevation_height_final = params.desiredGridHeight;
                elevation_resolution_final_dbl = log_cell_resolution_meters > 1e-6 ? log_cell_resolution_meters : 1.0;
                elevation_origin_final_x = 0.0;
                elevation_origin_final_y = 0.0;
                origin_offset_x = 0.0f;
                origin_offset_y = 0.0f;
            }
            float elevation_resolution_final = static_cast<float>(elevation_resolution_final_dbl);
            
            // Store final calculated parameters in result
            result.finalLogicalResolutionMeters = log_cell_resolution_meters;
            result.finalOriginOffsetX = origin_offset_x;
            result.finalOriginOffsetY = origin_offset_y;

            qDebug() << "PathfindingLogic: Elevation data prepared.";


            //--------------------------------------------
            // 4. Pathfinding Loop
            //--------------------------------------------
            qDebug() << "PathfindingLogic: Starting pathfinding loop for algorithm:" << QString::fromStdString(params.algorithmName);
            std::vector<int> full_path_indices;
            bool path_found_for_all_segments = true;
            std::string errorMsg; // Store error from segments
            double total_pathfinding_segment_duration_ms = 0.0;

            const Grid_V3& grid = logical_grid_opt.value(); // Use const ref to grid

            for (size_t i = 0; i < waypoints.size() - 1; ++i) {
                GridPoint segment_start_point = waypoints[i];
                GridPoint segment_end_point = waypoints[i + 1];

                qDebug() << "PathfindingLogic: Calculating segment" << (i + 1) << "/" << (waypoints.size() - 1) << "from"
                    << segment_start_point.x << "," << segment_start_point.y << "to"
                    << segment_end_point.x << "," << segment_end_point.y;

                // Bounds Check
                if (!grid.inBounds(segment_start_point.x, segment_start_point.y) || !grid.inBounds(segment_end_point.x, segment_end_point.y)) {
                    errorMsg = QString("Segment %1 start/end point (%2,%3 -> %4,%5) out of grid bounds (WxH: %6x%7).")
                        .arg(i + 1)
                        .arg(segment_start_point.x).arg(segment_start_point.y)
                        .arg(segment_end_point.x).arg(segment_end_point.y)
                        .arg(grid.width()).arg(grid.height())
                        .toStdString();
                    qCritical() << "PathfindingLogic Error:" << QString::fromStdString(errorMsg);
                    path_found_for_all_segments = false;
                    break;
                }
                // Identical Point Check
                if (segment_start_point == segment_end_point) {
                    qDebug() << "PathfindingLogic: Segment points identical, skipping calculation.";
                    int pointIndex = toIndex(segment_start_point.x, segment_start_point.y, grid.width());
                    if (full_path_indices.empty() || full_path_indices.back() != pointIndex) {
                        full_path_indices.push_back(pointIndex);
                    }
                    continue;
                }

                // --- Call Selected Pathfinding Function ---
                auto start_segment = std::chrono::high_resolution_clock::now();
                std::vector<int> segment_path_indices;
                bool isGpuAlgorithm = params.algorithmName.find("GPU") != std::string::npos;
                
#ifdef USE_CUDA
                //// --- GPU Algorithm Calls ---
                //if (params.algorithmName == "Delta Stepping - GPU") {
                //    segment_path_indices = findPathGPU_DeltaStepping(
                //        grid, elevation_values_final, elevation_width_final, elevation_height_final,
                //        log_cell_resolution_meters, elevation_resolution_final, origin_offset_x, origin_offset_y,
                //        segment_start_point, segment_end_point, params.gpuDelta, params.gpuThreshold);
                //}
                //else if (params.algorithmName == "HADS - GPU") {
                //    segment_path_indices = findPathGPU_HADS(
                //        grid, elevation_values_final, elevation_width_final, elevation_height_final,
                //        log_cell_resolution_meters, elevation_resolution_final, origin_offset_x, origin_offset_y,
                //        segment_start_point, segment_end_point,
                //        params.gpuDelta, params.gpuThreshold, params.hadsRadius, params.hadsPruneFactor, params.hadsHeuristicWeight);
                //}
                //else if (params.algorithmName == "A* - GPU") {
                //    segment_path_indices = findPathGPU_AStar(
                //        grid, elevation_values_final, elevation_width_final, elevation_height_final,
                //        log_cell_resolution_meters, elevation_resolution_final, origin_offset_x, origin_offset_y,
                //        segment_start_point, segment_end_point, params.heuristicType);
                //}
                //else
#endif // USE_CUDA
                    // --- CPU Algorithm Calls ---
                    if (params.algorithmName == "Optimized A*") {
                        segment_path_indices = findAStarPath_Tobler_Sampled(
                            grid, elevation_values_final, elevation_width_final, elevation_height_final,
                            log_cell_resolution_meters, elevation_resolution_final, origin_offset_x, origin_offset_y,
                            segment_start_point, segment_end_point, params.heuristicType);
                    }
                    else if (params.algorithmName == "Dijkstra") {
                        segment_path_indices = findDijkstraPath_Tobler_Sampled(
                            grid, elevation_values_final, elevation_width_final, elevation_height_final,
                            log_cell_resolution_meters, elevation_resolution_final, origin_offset_x, origin_offset_y,
                            segment_start_point, segment_end_point);
                    }
                    else if (params.algorithmName == "BFS") {
                        segment_path_indices = findBFSPath_Tobler_Sampled( // Ensure signature matches
                            grid, elevation_values_final, elevation_width_final, elevation_height_final,
                            log_cell_resolution_meters, elevation_resolution_final, origin_offset_x, origin_offset_y,
                            segment_start_point, segment_end_point);
                    }
                    else if (params.algorithmName == "Theta*") {
                        segment_path_indices = findThetaStarPath_Tobler_Sampled( // Ensure signature matches
                            grid, elevation_values_final, elevation_width_final, elevation_height_final,
                            log_cell_resolution_meters, elevation_resolution_final, origin_offset_x, origin_offset_y,
                            segment_start_point, segment_end_point, params.heuristicType);
                    }
                    else if (params.algorithmName == "Lazy Theta*") {
                        segment_path_indices = findLazyThetaStarPath_Tobler_Sampled( // Ensure signature matches
                            grid, elevation_values_final, elevation_width_final, elevation_height_final,
                            log_cell_resolution_meters, elevation_resolution_final, origin_offset_x, origin_offset_y,
                            segment_start_point, segment_end_point, params.heuristicType);
                    }
                // --- Error Handling for Unknown Algorithm ---
                    else {
                        // This logic handles the case where the name is unrecognized,
                        // considering whether CUDA is enabled or not.
#ifdef USE_CUDA
                        if (isGpuAlgorithm) { // Name contained GPU but didn't match known GPU types
                            errorMsg = "Selected GPU algorithm '" + params.algorithmName + "' is not implemented.";
                        }
                        else { // Name did not contain GPU and didn't match known CPU types
                            errorMsg = "Unsupported CPU algorithm selected: " + params.algorithmName;
                        }
#else // USE_CUDA not defined
                        if (isGpuAlgorithm) { // Name contained GPU, but CUDA is off
                            errorMsg = "GPU algorithm '" + params.algorithmName + "' selected, but CUDA is disabled in this build.";
                        }
                        else { // Name did not contain GPU and didn't match known CPU types
                            errorMsg = "Unsupported CPU algorithm selected: " + params.algorithmName;
                        }
#endif // USE_CUDA
                        throw std::runtime_error(errorMsg); // Throw exception for unsupported algorithm
                    }

                auto end_segment = std::chrono::high_resolution_clock::now();
                double segment_duration_ms = std::chrono::duration<double, std::milli>(end_segment - start_segment).count();
                total_pathfinding_segment_duration_ms += segment_duration_ms;
                qDebug() << "PathfindingLogic: Segment" << (i + 1) << "took" << segment_duration_ms << "ms.";

                // Check Segment Result & Concatenate
                if (segment_path_indices.empty()) {
                    errorMsg = QString("Path not found for segment %1 (Start: %2,%3 End: %4,%5).")
                        .arg(i + 1)
                        .arg(segment_start_point.x).arg(segment_start_point.y)
                        .arg(segment_end_point.x).arg(segment_end_point.y)
                        .toStdString();
                    qWarning() << "PathfindingLogic:" << QString::fromStdString(errorMsg);
                    path_found_for_all_segments = false;
                    break; // Stop processing segments if one fails
                }

                // Concatenate paths (using std::move_iterator for efficiency)
                if (full_path_indices.empty()) {
                    full_path_indices = std::move(segment_path_indices);
                }
                else {
                    if (!segment_path_indices.empty()) { // Should always be true here, but check
                        // Avoid duplication if segment starts where previous ended
                        if (segment_path_indices.size() > 1 && full_path_indices.back() == segment_path_indices.front()) {
                            full_path_indices.insert(full_path_indices.end(),
                                std::make_move_iterator(segment_path_indices.begin() + 1),
                                std::make_move_iterator(segment_path_indices.end()));
                        }
                        else if (segment_path_indices.size() == 1 && full_path_indices.back() == segment_path_indices.front()) {
                            // Segment is only one point, and it's a duplicate, do nothing
                        }
                        else {
                            // Append full segment if no overlap or first point differs
                            if (segment_path_indices.size() > 1 && full_path_indices.back() != segment_path_indices.front()) {
                                qWarning() << "PathfindingLogic: Segment" << (i + 1) << "start does not match previous end. Appending full segment.";
                            }
                            full_path_indices.insert(full_path_indices.end(),
                                std::make_move_iterator(segment_path_indices.begin()),
                                std::make_move_iterator(segment_path_indices.end()));
                        }
                    }
                }
            } // End waypoint loop

            result.pathfindingDurationMs = total_pathfinding_segment_duration_ms;
            qDebug() << "PathfindingLogic: Pathfinding loop finished. Total segment time:" << result.pathfindingDurationMs << "ms.";

            //--------------------------------------------
            // 5. Finalize Result
            //--------------------------------------------
            if (path_found_for_all_segments) {
                if (full_path_indices.empty() && waypoints.size() > 1) {
                    // This case might happen if all segments were identical points
                    qWarning() << "PathfindingLogic: All segments processed, but resulting path is empty (possible if all waypoints were identical).";
                    result.success = true; // Technically succeeded, just no path needed
                    result.errorMessage = "Path consists of identical points only.";
                }
                else if (full_path_indices.empty() && waypoints.size() <= 1) {
                    // Less than 2 waypoints, nothing to pathfind
                    result.success = true;
                    result.errorMessage = "Not enough waypoints for pathfinding.";
                }
                else {
                    result.success = true;
                    result.fullPathIndices = std::move(full_path_indices);
                    qDebug() << "PathfindingLogic: Full path found. Length:" << result.fullPathIndices.size();
                }
            }
            else {
                result.success = false;
                result.errorMessage = errorMsg; // Store last error message
                result.fullPathIndices.clear(); // Ensure path is empty on failure
                qDebug() << "PathfindingLogic: Path generation failed for one or more segments.";
            }

        }
        catch (const std::exception& e) {
            result.success = false;
            result.errorMessage = std::string("Backend Error: ") + e.what();
            qCritical() << "PathfindingLogic Error:" << QString::fromStdString(result.errorMessage);
        }
        catch (...) {
            result.success = false;
            result.errorMessage = "Backend Error: An unknown exception occurred.";
            qCritical() << "PathfindingLogic Error:" << QString::fromStdString(result.errorMessage);
        }

        auto end_full_proc = std::chrono::high_resolution_clock::now();
        //qDebug() << "PathfindingLogic: Total processing time:" << std::chrono::duration<double, std::milli>(end_full_proc - start_full_proc).count() << "ms.";

        return result;
    } // End processAndFindPath


} // namespace app
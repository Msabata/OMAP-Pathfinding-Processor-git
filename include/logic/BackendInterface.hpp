// include/logic/BackendInterface.hpp
#pragma once
#ifndef APP_BACKEND_INTERFACE_HPP
#define APP_BACKEND_INTERFACE_HPP

#include <string>
#include <vector>
#include <optional>
#include <map>

// Include necessary type definitions used within the structs
#include "map/MapProcessingCommon.h" // Includes GridPoint, ObstacleConfigMap, NormalizationResult
#include "map/MapProcessor.hpp"      // Includes Grid_V3
#include "map/ElevationFetchingCommon.hpp" // Includes ElevationData

// --- Define Interface Structs HERE ONLY ---

struct BackendInputParams {
    // File Paths
    std::string mapFilePath;
    std::string controlsFilePath;

    // Grid Configuration
    int desiredGridWidth = 1000;
    int desiredGridHeight = 1000;
    mapgeo::ObstacleConfigMap obstacleCosts;
    unsigned int numThreads = 1;

    // Elevation
    double desiredElevationResolution = 90.0;
    std::string pyModuleName = "elevation_logic";
    std::string pyFetchFuncName = "get_elevation_grid";
    std::string pyConvertFuncName = "convert_latlon_to_projected";

    // Pathfinding
    std::string algorithmName = "Optimized A*";
    int heuristicType = 3; // HEURISTIC_MIN_COST (Assuming PathfindingUtils.hpp defines this)

    // GPU Parameters
    float gpuDelta = 50.0f;
    float gpuThreshold = 50.0f;
    int hadsRadius = 1000;
    float hadsPruneFactor = 1.05f;
    float hadsHeuristicWeight = 0.95f;

    // Grid Reuse Data
    bool reuseGridIfPossible = false;
    std::optional<mapgeo::Grid_V3> existingGrid;
    std::optional<mapgeo::NormalizationResult> existingNormInfo;
};

struct BackendResult {
    bool success = false;
    std::string errorMessage;

    // Input Echoing
    std::string usedMapFilePath;
    std::string currentControlsFilePath;
    int usedGridWidth = 0;
    int usedGridHeight = 0;

    // Map Processing Outputs
    std::optional<mapgeo::Grid_V3> processedGrid;
    std::optional<mapgeo::NormalizationResult> normalizationInfo;

    // Elevation Outputs
    std::optional<ElevationFetcher::ElevationData> elevationDataUsed; // Use correct namespace
    bool usedDummyElevation = true;
    float finalLogicalResolutionMeters = 1.0f;
    float finalOriginOffsetX = 0.0f;
    float finalOriginOffsetY = 0.0f;

    // Pathfinding Outputs
    std::vector<int> fullPathIndices;
    double pathfindingDurationMs = 0.0;
    double mapProcessingDurationMs = 0.0;
    double elevationFetchDurationMs = 0.0;

    // Debug/Info
    size_t waypointsFound = 0;
};

#endif // APP_BACKEND_INTERFACE_HPP
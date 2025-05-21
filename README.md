# OMAP Pathfinding Processor (CMake Project)

This project implements a sophisticated pathfinding processor for orienteering maps (.omap format or compatible XML). It calculates time-optimal routes by considering terrain type, slope (using Tobler's hiking function), and various pathfinding algorithms, including CPU and GPU (CUDA) accelerated versions. The project is built using CMake.

## Features

*   **Map Processing:**
    *   Parses .omap/XML map files (ISOM 2017-2 symbol interpretation).
    *   Handles complex geometries including points, lines, and areas with holes, gaps, and dashes.
    *   Generates a 2D logical grid representing base terrain traversal costs.
    *   Utilizes a two-pass rasterization process:
        *   Pass 1: Parallel boundary drawing (OpenMP, if enabled).
        *   Pass 2: Feature-parallel area filling using a robust Scanline algorithm and rule application.
    *   Automatic calculation of logical grid resolution based on map coordinates.
*   **Elevation Data Integration:**
    *   Fetches Digital Elevation Model (DEM) data via an external Python script (`python/elevation_logic.py`) using `pybind11` for C++/Python interop.
    *   Handles potentially different resolutions and origins between the map's logical grid and the elevation grid.
    *   Uses `ElevationSampler` for bilinear interpolation of elevation values.
*   **Pathfinding Algorithms:**
    *   **CPU Implementations:**
        *   A* (Optimized)
        *   Dijkstra's Algorithm
        *   Breadth-First Search (BFS)
        *   Theta* (Any-Angle)
        *   Lazy Theta* (Optimized Any-Angle)
    *   **GPU (CUDA) Implementations (Conditional - if `USE_CUDA=ON`):**
        *   Delta-Stepping
        *   HADS (Heuristic-Accelerated Delta-Stepping)
        *   A* (Experimental, one-node expansion per iteration)
    *   Dynamic "on-the-fly" edge cost calculation considering:
        *   Geometric distance (axial/diagonal).
        *   Base terrain cost from the processed map grid.
        *   Slope penalty derived from Tobler's hiking function: `exp(-3.5 * abs(Slope + 0.05))`.
*   **Waypoint Processing:**
    *   Extracts Start (701), Finish (706), and Control (703) points from a separate .omap "controls" file.
    *   Calculates the path sequentially between waypoints.
*   **Path Export:**
    *   Saves the calculated path back into a new .omap file, overlaying the route on a copy of the controls file.
*   **GUI (Built with Qt):**
    *   User-friendly interface for file loading, parameter configuration, and initiating calculations.
    *   Asynchronous backend processing to keep UI responsive.
    *   Light/Dark theme support.
*   **Build System:** Cross-platform CMake for project configuration and compilation.
*   **Optimizations:** Supports optional OpenMP for CPU parallelism and AVX2 for SIMD instructions.

## Project Structure (Key Directories)

*   `CMakeLists.txt`: Main CMake configuration file.
*   `cuda/`: CUDA specific code.
    *   `include/`: CUDA headers (`.cuh`), e.g., `PathfindingUtilsGPU.cuh`.
    *   `src/`: CUDA kernels (`.cu`), e.g., `AStarKernels.cu`, `DeltaSteppingKernels.cu`.
*   `include/`: C++ public headers (`.hpp`, `.h`) organized by module:
    *   `algoritms/`, `debug/`, `IO/`, `logic/`, `map/`.
*   `python/`: Python scripts, e.g., `elevation_logic.py`.
*   `src/`: C++ source files (`.cpp`) organized by module:
    *   `main.cpp`: Entry point for the Qt application.
    *   `gui/`: GUI implementation, e.g., `main_window.cpp`.
    *   (Other modules mirroring `include/`).
*   `build/` (Typically created by user): Directory for out-of-source builds.

## Prerequisites

1.  **CMake:** Version 3.14 or higher.
2.  **C++ Compiler:** Supporting C++17 (e.g., GCC, Clang, MSVC).
3.  **Qt Framework:** Version 5 or 6 (as configured in CMakeLists.txt, typically Qt5::Widgets).
    *   Ensure Qt development libraries and tools are installed and discoverable by CMake.
4.  **Python:** Version 3.9 or higher (as configured in CMakeLists.txt).
    *   Interpreter and Development libraries.
    *   `pip` for installing packages.
    *   Required Python packages (installed via CMake or manually):
        ```bash
        pip install pybind11 requests pyproj
        ```
5.  **(Optional) NVIDIA CUDA Toolkit:** If building with GPU support (`USE_CUDA=ON`).
    *   Version compatible with project settings (e.g., 11.x, 12.x).
    *   NVIDIA GPU Driver compatible with the CUDA Toolkit.
6.  **(Optional) OpenMP:** If `USE_OPENMP=ON`, your compiler must support OpenMP.

## Building the Project with CMake

1.  **Clone the repository:**
    ```bash
    git clone <repository-url>
    cd <repository-directory>
    ```
2.  **Create a build directory:**
    ```bash
    mkdir build
    cd build
    ```
3.  **Configure the project using CMake:**
    *   **Basic Configuration (e.g., Release, no CUDA):**
        ```bash
        cmake .. -DCMAKE_BUILD_TYPE=Release -DUSE_CUDA=OFF
        ```
    *   **With CUDA Support (e.g., Release):**
        ```bash
        cmake .. -DCMAKE_BUILD_TYPE=Release -DUSE_CUDA=ON
        ```
    *   **Other options:**
        *   `-DUSE_OPENMP=ON` (default) or `OFF`
        *   `-DUSE_AVX2=ON` (default) or `OFF`
        *   Specify a generator if needed (e.g., `-G "Visual Studio 17 2022"` or `-G "Ninja"`).
        *   You might need to help CMake find Qt by setting `CMAKE_PREFIX_PATH`, e.g., `-DCMAKE_PREFIX_PATH=/path/to/Qt/version/compiler`.

    CMake will attempt to find all dependencies. Review the output for any errors or warnings.

4.  **Install Python Dependencies (CMake target):**
    After successful configuration, you can use the custom CMake target to install Python packages:
    ```bash
    cmake --build . --target install_python_deps
    ```
    Alternatively, install them manually using `pip install -r requirements.txt` (a `requirements.txt` is generated in the build directory).

5.  **Compile the project:**
    ```bash
    cmake --build . --config Release 
    ```
    (Replace `Release` with `Debug` if you configured for a debug build).

6.  **Run the Application:**
    *   The executable (e.g., `MinimalQtProject2.exe` on Windows, `MinimalQtProject2` on Linux/macOS) will be located in the build directory (e.g., `build/Release` or `build/`).
    *   Ensure the `python` subdirectory (containing `elevation_logic.py`) is correctly located relative to the executable, or that `PYTHON_MODULE_PATH` (set by CMake during compilation) points to it, so the C++ application can find and import the Python module. Typically, placing the `python` folder next to the executable or in a location Python's `sys.path` will check is sufficient.

## Usage

Run the compiled executable. The application provides a graphical user interface (GUI) to:
1.  Load `.omap` map and control files.
2.  Configure map processing parameters (grid size, obstacle costs, etc.).
3.  Select a pathfinding algorithm (CPU or GPU) and its specific parameters (heuristics, Delta, HADS settings).
4.  Initiate the path calculation.
5.  View status and (currently text-based) results.
6.  Export the calculated path to a new `.omap` file.

(Future development includes in-app map and path visualization).

## Key Code Components

*   **`src/gui/main_window.cpp/.hpp`**: Qt-based GUI implementation.
*   **`src/logic/PathfindingLogic.cpp/.hpp`**: Orchestrates the overall workflow.
*   **`src/map/MapProcessor.cpp/.hpp`**: Map parsing and rasterization.
*   **`src/map/ElevationFetcherPy.cpp/.hpp`**: Python interop for elevation.
*   **`src/algoritms/`**: CPU algorithm implementations.
*   **`cuda/src/` & `cuda/include/`**: GPU algorithm kernels and device utilities.
*   **`python/elevation_logic.py`**: Python script for DEM data.

## Known Limitations

*   Curve interpolation (`CoordFlags::CurveStart`) is not currently implemented in map processing.
*   A* pathfinding (and related heuristics) assumes logical grid cells are approximately square for diagonal distance calculations.
*   The GPU A* implementation expands only one node per iteration, which limits its parallelism and performance.

## Future Work

*   Implement GPU-accelerated map rasterization.
*   Integrate a more advanced parallel A* algorithm for GPU.
*   Develop an AI module for learning runner styles.
*   Implement Fuzzy A*.
*   Direct map and path visualization within the Qt application.

## Contributing

In future ask me:)

## License
GNU GENERAL PUBLIC LICENSE Version 3

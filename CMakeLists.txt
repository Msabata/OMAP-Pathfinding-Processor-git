cmake_minimum_required(VERSION 3.14)
project(OMAP Pathfinding Processor VERSION 1.0 LANGUAGES CXX)

# Core compilation parameters optimization
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Qt dependency resolution
find_package(Qt5 COMPONENTS Widgets REQUIRED)

# Python integration - optimized detection mechanism
# 1. Locate Python interpreter and development files
find_package(Python3 3.9 COMPONENTS Interpreter Development REQUIRED)
message(STATUS "Python3 detected: ${Python3_EXECUTABLE} (v${Python3_VERSION})")
message(STATUS "Python3 include paths: ${Python3_INCLUDE_DIRS}")
message(STATUS "Python3 libraries: ${Python3_LIBRARIES}")

# 2. Pybind11 dynamic discovery via Python introspection
execute_process(
    COMMAND "${Python3_EXECUTABLE}" -c "import pybind11; print(pybind11.get_include())"
    OUTPUT_VARIABLE PYBIND11_INCLUDE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE PYBIND11_IMPORT_RESULT
)

if(NOT PYBIND11_IMPORT_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to locate pybind11. Ensure it's installed via: pip install pybind11")
endif()

message(STATUS "Pybind11 include path: ${PYBIND11_INCLUDE_DIR}")

# CUDA support configuration (conditional)
option(USE_CUDA "Enable CUDA acceleration" OFF)
if(USE_CUDA)
    enable_language(CUDA)
    set(CMAKE_CUDA_STANDARD 14)
    set(CMAKE_CUDA_STANDARD_REQUIRED ON)
    
    find_package(CUDA REQUIRED)
    message(STATUS "CUDA detected: v${CUDA_VERSION}")
    
    add_compile_definitions(USE_CUDA=1)
else()
    add_compile_definitions(USE_CUDA=0)
endif()

# Source file collection with optimized patterns
# Using deterministic collection to ensure consistent builds
set(CPP_SOURCES "")
set(HPP_HEADERS "")
set(CUDA_SOURCES "")
set(CUDA_HEADERS "")

# Recursive source discovery with explicit pattern matching
file(GLOB_RECURSE CPP_SOURCES 
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

# Remove GPU implementation files if CUDA is not enabled
if(NOT USE_CUDA)
    list(REMOVE_ITEM CPP_SOURCES 
         "${CMAKE_CURRENT_SOURCE_DIR}/src/algoritms/AStarGPU.cpp"
         "${CMAKE_CURRENT_SOURCE_DIR}/src/algoritms/DeltaSteppingGPU.cpp"
         "${CMAKE_CURRENT_SOURCE_DIR}/src/algoritms/HADS_GPU.cpp"
    )
endif()

file(GLOB_RECURSE HPP_HEADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h"
)

# Conditional CUDA file collection
if(USE_CUDA)
    file(GLOB_RECURSE CUDA_SOURCES 
        "${CMAKE_CURRENT_SOURCE_DIR}/cuda/src/*.cu"
    )
    file(GLOB_RECURSE CUDA_HEADERS 
        "${CMAKE_CURRENT_SOURCE_DIR}/cuda/include/*.cuh"
    )
    
    # Add the PathfindingUtilsGPU.cuh manually to make sure it's included
    list(APPEND CUDA_HEADERS 
        "${CMAKE_CURRENT_SOURCE_DIR}/cuda/include/PathfindingUtilsGPU.cuh"
    )
endif()

# Source discovery diagnostics (verbose)
message(STATUS "C++ sources detected: ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
foreach(src ${CPP_SOURCES})
    message(STATUS "  ${src}")
endforeach()

message(STATUS "Header files detected:")
foreach(hdr ${HPP_HEADERS})
    message(STATUS "  ${hdr}")
endforeach()

if(USE_CUDA)
    message(STATUS "CUDA sources detected:")
    foreach(cu ${CUDA_SOURCES})
        message(STATUS "  ${cu}")
    endforeach()
    
    message(STATUS "CUDA headers detected:")
    foreach(cuh ${CUDA_HEADERS})
        message(STATUS "  ${cuh}")
    endforeach()
endif()

# Executable target definition with comprehensive source inclusion
add_executable(${PROJECT_NAME}
    ${CPP_SOURCES}
    ${HPP_HEADERS}
    ${CUDA_SOURCES}
    ${CUDA_HEADERS}
 "src/logic/PathfindingLogic.cpp")

# Include path optimization - prioritized for compiler search efficiency
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${Python3_INCLUDE_DIRS}
    ${PYBIND11_INCLUDE_DIR}
)

# CUDA include paths (conditional)
if(USE_CUDA)
    target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/cuda/include
        ${CUDA_INCLUDE_DIRS}
    )
endif()

# Python path resolution at compile time
target_compile_definitions(${PROJECT_NAME} PRIVATE
    PYTHON_EXECUTABLE="${Python3_EXECUTABLE}"
    PYTHON_MODULE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/python"
)

# Enable OpenMP parallelization with fallback mechanism
option(USE_OPENMP "Enable OpenMP parallelization" ON)
if(USE_OPENMP)
    find_package(OpenMP)
    if(OpenMP_CXX_FOUND)
        message(STATUS "OpenMP detected: parallelization enabled")
        target_compile_definitions(${PROJECT_NAME} PRIVATE USE_OPENMP=1)
        target_link_libraries(${PROJECT_NAME} PRIVATE OpenMP::OpenMP_CXX)
    else()
        message(WARNING "OpenMP requested but not detected. Falling back to sequential execution.")
        target_compile_definitions(${PROJECT_NAME} PRIVATE USE_OPENMP=0)
    endif()
else()
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_OPENMP=0)
endif()

# SIMD optimization via AVX2
option(USE_AVX2 "Enable AVX2 SIMD acceleration" ON)
if(USE_AVX2)
    message(STATUS "AVX2 SIMD acceleration: enabled")
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_AVX2=1)
    
    # Platform-specific compilation flags
    if(MSVC)
        target_compile_options(${PROJECT_NAME} PRIVATE /arch:AVX2)
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${PROJECT_NAME} PRIVATE -mavx2)
    else()
        message(WARNING "AVX2 enabled but compiler detection failed. Manual flag configuration required.")
    endif()
else()
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_AVX2=0)
endif()

# Core dependency linkage
target_link_libraries(${PROJECT_NAME} PRIVATE 
    Qt5::Widgets
    ${Python3_LIBRARIES}
)

# Conditional CUDA linkage
if(USE_CUDA)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${CUDA_LIBRARIES})
endif()

# Platform-specific optimizations
if(WIN32)
    target_compile_definitions(${PROJECT_NAME} PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
endif()

# CUDA architecture targeting
if(USE_CUDA)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        CUDA_ARCHITECTURES "52;60;70;75"  # Pascal, Volta, Turing GPU compatibility
    )
    
    # Floating-point optimization for CUDA
    target_compile_options(${PROJECT_NAME} PRIVATE 
        $<$<COMPILE_LANGUAGE:CUDA>:--use_fast_math>
    )
endif()

# MSVC-specific exception handling model
if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /EHsc)
endif()

# Python dependency management
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/requirements.txt"
    "requests\n"
    "pyproj\n"
)

# Utility target for dependency installation
add_custom_target(install_python_deps
    COMMAND ${Python3_EXECUTABLE} -m pip install -r ${CMAKE_CURRENT_BINARY_DIR}/requirements.txt
    COMMENT "Installing Python dependencies (requests, pyproj)..."
    VERBATIM
)
// File: ElevationFetcherPy.cpp
#include "map/ElevationFetcherPy.hpp"

// Make sure pybind11 headers are included correctly by your build system
#include <pybind11/embed.h> // Main header for embedding
#include <pybind11/stl.h>   // For automatic C++ <-> Python STL container conversions

#include <iostream>
#include <stdexcept>
#include <sstream> // For error messages
#include <filesystem> // Optional: For more robust path finding if needed (C++17)

namespace py = pybind11;
using namespace pybind11::literals; // Allows C++ string literals as Python args ("arg_name"_a)

namespace ElevationFetcher {

    // Global flag to track interpreter state
    bool g_pythonInitialized = false;

    // --- Interpreter Management ---
    bool initializePython() {
        if (g_pythonInitialized) {
            std::cout << "Info: Python interpreter already initialized." << std::endl;
            return true;
        }
        try {
            std::cout << "Initializing Python interpreter..." << std::endl;
            py::initialize_interpreter();
            g_pythonInitialized = true;

            // Optional: Add the script's directory to Python's path
            // This helps Python find the 'elevation_logic.py' module
            try {
                py::module_ sys = py::module_::import("sys");
                py::list path = sys.attr("path");

                // --- DETERMINE SCRIPT DIRECTORY (NEEDS ADJUSTMENT FOR YOUR SETUP) ---
                // Option 1: Hardcode absolute path (simplest for initial debugging)
                std::string script_dir = "D:/python"; // ADJUST IF NEEDED

                // Option 2: Assume script is copied to exe directory (requires Post-Build Event)
                // Requires code to get executable path (platform specific or C++17 filesystem)
                // Example using C++17 filesystem (might need linking adjustments):
                // std::filesystem::path exe_path = std::filesystem::current_path(); // Or more robust method
                // std::string script_dir = exe_path.string(); // If script is right next to exe

                // Option 3: Relative path (depends on where VS runs the executable from)
                // std::string script_dir = "./python"; // If run from project root
                // std::string script_dir = "../python"; // If run from build dir one level down
                // --- END DETERMINE SCRIPT DIRECTORY ---


                // Check if path already exists to avoid duplicates
                bool found = false;
                for (const auto& item : path) {
                    if (py::isinstance<py::str>(item) && item.cast<std::string>() == script_dir) {
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    path.attr("insert")(0, script_dir); // Add to front
                    std::cout << "Info: Added '" << script_dir << "' to Python sys.path" << std::endl;
                }
                else {
                    std::cout << "Info: Python sys.path already contains '" << script_dir << "'" << std::endl;
                }

                // Print the final path for verification
                std::cout << "Current Python sys.path:" << std::endl;
                py::print(path); // py::print uses Python's print

            }
            catch (py::error_already_set& e) {
                std::cerr << "Warning: Failed during Python sys.path modification: " << e.what() << std::endl;
                if (PyErr_Occurred()) PyErr_Print();
                e.restore();
                // Continue anyway, maybe it finds it another way...
            }
            catch (const std::exception& e) {
                std::cerr << "Warning: C++ exception during Python sys.path modification: " << e.what() << std::endl;
            }

            // --- Optional: Test importing critical modules ---
            std::cout << "Attempting to import required Python modules..." << std::endl;
            try {
                py::module_::import("requests");
                std::cout << "  Successfully imported 'requests'." << std::endl;
                py::module_::import("pyproj");
                std::cout << "  Successfully imported 'pyproj'." << std::endl;
            }
            catch (py::error_already_set& e) {
                std::cerr << "ERROR: Failed to import required Python module during initialization: " << e.what() << std::endl;
                if (PyErr_Occurred()) PyErr_Print(); // Print the detailed Python error
                e.restore();
                // return false; // Consider failing initialization if core libs missing
            }
            catch (const std::exception& e) {
                std::cerr << "ERROR: C++ exception during Python module import test: " << e.what() << std::endl;
                // return false;
            }
            std::cout << "Python module import check complete." << std::endl;
            // --- End Optional Import Test ---


            std::cout << "Python interpreter initialized successfully." << std::endl;
            return true;

        }
        catch (py::error_already_set& e) {
            std::cerr << "Error: Failed to initialize Python interpreter: " << e.what() << std::endl;
            if (PyErr_Occurred()) PyErr_Print();
            return false;
        }
        catch (const std::exception& e) {
            std::cerr << "Error: Non-pybind exception during Python initialization: " << e.what() << std::endl;
            return false;
        }
    }

    void finalizePython() {
        if (g_pythonInitialized) {
            std::cout << "Finalizing Python interpreter..." << std::endl;
            // Ensure GIL is released if held by this thread (usually not needed at exit, but safe)
            // if (PyGILState_Check()) { // Check if current thread holds GIL
            //    PyGILState_Release(PyGILState_Ensure()); // Release if held
            // }
            py::finalize_interpreter();
            g_pythonInitialized = false;
            std::cout << "Python interpreter finalized." << std::endl;
        }
        else {
            std::cout << "Info: Python interpreter was not initialized or already finalized." << std::endl;
        }
    }

    // --- Helper to Parse Python Dictionary Result for ElevationData ---
    ElevationData parsePythonElevationResult(const py::dict& pyResult) {
        ElevationData result;
        std::stringstream errorStream;
        try {
            // Use .attr("get") with default value for safer access
            result.success = pyResult.attr("get")("success", py::bool_(false)).cast<bool>();
            result.errorMessage = pyResult.attr("get")("error_message", py::str("")).cast<std::string>();

            if (result.success) {
                // Only parse other fields if success is true
                result.width = pyResult["width"].cast<int>(); // Use [] if key MUST exist on success
                result.height = pyResult["height"].cast<int>();
                result.origin_proj_x = pyResult["origin_proj_x"].cast<double>();
                result.origin_proj_y = pyResult["origin_proj_y"].cast<double>();
                result.resolution_meters = pyResult["resolution_meters"].cast<double>();
                result.values = pyResult["values"].cast<std::vector<float>>();

                // Validate data consistency
                if (!result.hasData()) {
                    errorStream << "Parsed Python result failed validation (size="
                        << result.values.size() << ", expected="
                        << static_cast<size_t>(result.width) * result.height << ").";
                    throw std::runtime_error(errorStream.str()); // Throw to be caught below
                }
            }
            else {
                // If success is false, errorMessage should already be set from Python
                if (result.errorMessage.empty()) { // Safety check
                    result.errorMessage = "Python function returned success=False with no error message.";
                }
                std::cerr << "Python script reported failure: " << result.errorMessage << std::endl;
            }
        }
        catch (const py::cast_error& e) {
            errorStream << "Failed to cast Python result dictionary field: " << e.what();
            result.success = false;
            result.errorMessage = errorStream.str();
            std::cerr << "Error parsing Python result: " << result.errorMessage << std::endl;
            result.values.clear();
        }
        catch (const std::exception& e) {
            errorStream << "Standard exception parsing Python result: " << e.what();
            result.success = false;
            result.errorMessage = errorStream.str();
            std::cerr << "Error parsing Python result: " << result.errorMessage << std::endl;
            result.values.clear();
        }
        return result;
    }


    // --- Main Fetch Function ---
    ElevationData fetchElevationDataEmbedded(
        const std::string& pythonModuleName,
        const std::string& pythonFunctionName,
        // Anchor point
        double known_lat, double known_lon,
        double known_internal_x, double known_internal_y,
        // Map definition (Pass bounds as micrometers µm)
        double raw_min_x_um, double raw_min_y_um,
        double raw_max_x_um, double raw_max_y_um,
        double map_scale,
        // Query parameters
        double desired_resolution_meters
    ) {
        ElevationData cppResult;
        if (!g_pythonInitialized) {
            cppResult.success = false;
            cppResult.errorMessage = "Python interpreter not initialized.";
            std::cerr << "Error (fetchElevationDataEmbedded): " << cppResult.errorMessage << std::endl;
            return cppResult;
        }

        // Acquire the Global Interpreter Lock for this scope
        // Ensures thread safety if C++ code uses multiple threads calling Python
        py::gil_scoped_acquire acquire;

        std::cout << "Info (C++): Attempting to import Python module '" << pythonModuleName << "'..." << std::endl;
        try {
            // Import the specified Python module
            py::module_ elev_module = py::module_::import(pythonModuleName.c_str());
            std::cout << "Info (C++): Module '" << pythonModuleName << "' imported successfully." << std::endl;

            // Get the function from the module
            std::cout << "Info (C++): Attempting to get function '" << pythonFunctionName << "'..." << std::endl;
            py::object py_func = elev_module.attr(pythonFunctionName.c_str());
            std::cout << "Info (C++): Got function '" << pythonFunctionName << "'." << std::endl;


            // Call the Python function with named arguments
            std::cout << "Info (C++): Calling Python function '" << pythonFunctionName << "'..." << std::endl;
            py::object result_obj = py_func(
                "known_lat"_a = known_lat,
                "known_lon"_a = known_lon,
                "known_internal_x"_a = known_internal_x,
                "known_internal_y"_a = known_internal_y,
                // --- Corrected argument names to pass ---
                "raw_min_x_um"_a = raw_min_x_um,
                "raw_min_y_um"_a = raw_min_y_um,
                "raw_max_x_um"_a = raw_max_x_um,
                "raw_max_y_um"_a = raw_max_y_um,
                // --- End Correction ---
                "map_scale"_a = map_scale,
                "desired_resolution_meters"_a = desired_resolution_meters
            );
            std::cout << "Info (C++): Python function call returned." << std::endl;


            // Check if the result is a Python dictionary and parse it
            if (py::isinstance<py::dict>(result_obj)) {
                std::cout << "Info (C++): Parsing Python dictionary result..." << std::endl;
                cppResult = parsePythonElevationResult(result_obj.cast<py::dict>());
                std::cout << "Info (C++): Parsing complete (Success=" << cppResult.success << ")." << std::endl;
            }
            else {
                cppResult.success = false;
                cppResult.errorMessage = "Python function [" + pythonFunctionName + "] did not return a dictionary.";
                std::cerr << "Error (fetchElevationDataEmbedded): " << cppResult.errorMessage << std::endl;
            }

        }
        catch (py::error_already_set& e) {
            // Catch Python exceptions during import or execution
            cppResult.success = false;
            cppResult.errorMessage = "Python Exception occurred: ";
            cppResult.errorMessage += e.what();
            std::cerr << "Error (fetchElevationDataEmbedded): " << cppResult.errorMessage << std::endl;
            // Print the Python traceback if possible
            if (PyErr_Occurred()) {
                std::cerr << "--- Python Traceback ---" << std::endl;
                PyErr_Print(); // This prints to stderr
                std::cerr << "--- End Traceback ---" << std::endl;
            }
            e.restore(); // Clear the Python error state is important
        }
        catch (const std::exception& e) {
            // Catch C++ exceptions (e.g., during pybind operations)
            cppResult.success = false;
            cppResult.errorMessage = "C++ Exception during Python call: ";
            cppResult.errorMessage += e.what();
            std::cerr << "Error (fetchElevationDataEmbedded): " << cppResult.errorMessage << std::endl;
        }

        // GIL is released automatically when 'acquire' goes out of scope
        return cppResult;
    }


    // --- Coordinate Conversion Helper Function ---
    ProjectedPointResult convertLatLonToProjectedViaPython(
        const std::string& pythonModuleName,
        const std::string& pythonFunctionName,
        double lon,
        double lat
    ) {
        ProjectedPointResult cppResult;
        if (!g_pythonInitialized) {
            cppResult.success = false;
            cppResult.error = "Python interpreter not initialized.";
            std::cerr << "Error (convertLatLonToProjectedViaPython): " << cppResult.error << std::endl;
            return cppResult;
        }

        py::gil_scoped_acquire acquire; // Acquire GIL

        try {
            py::module_ elev_module = py::module_::import(pythonModuleName.c_str());
            py::object py_func = elev_module.attr(pythonFunctionName.c_str());

            std::cout << "Info (C++): Calling Python function '" << pythonFunctionName << "' for coord conversion..." << std::endl;
            py::object result_obj = py_func("lon"_a = lon, "lat"_a = lat);
            std::cout << "Info (C++): Python conversion function call returned." << std::endl;


            if (py::isinstance<py::dict>(result_obj)) {
                py::dict pyResult = result_obj.cast<py::dict>();
                cppResult.success = pyResult.attr("get")("success", py::bool_(false)).cast<bool>();
                if (cppResult.success) {
                    cppResult.x = pyResult["x"].cast<double>();
                    cppResult.y = pyResult["y"].cast<double>();
                }
                else {
                    cppResult.error = pyResult.attr("get")("error", py::str("Unknown Python error during conversion.")).cast<std::string>();
                    std::cerr << "Python conversion function reported failure: " << cppResult.error << std::endl;
                }
            }
            else {
                cppResult.success = false;
                cppResult.error = "Python conversion function did not return a dictionary.";
                std::cerr << "Error (convertLatLonToProjectedViaPython): " << cppResult.error << std::endl;
            }

        }
        catch (py::error_already_set& e) {
            cppResult.success = false;
            cppResult.error = "Python Exception during conversion: ";
            cppResult.error += e.what();
            std::cerr << "Error (convertLatLonToProjectedViaPython): " << cppResult.error << std::endl;
            if (PyErr_Occurred()) {
                std::cerr << "--- Python Traceback ---" << std::endl;
                PyErr_Print();
                std::cerr << "--- End Traceback ---" << std::endl;
            }
            e.restore();
        }
        catch (const std::exception& e) {
            cppResult.success = false;
            cppResult.error = "C++ Exception during Python conversion call: ";
            cppResult.error += e.what();
            std::cerr << "Error (convertLatLonToProjectedViaPython): " << cppResult.error << std::endl;
        }

        return cppResult;
    }


} // namespace ElevationFetcher
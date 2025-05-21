#ifndef MAP_PROCESSING_COMMON_H
#define MAP_PROCESSING_COMMON_H

#include <vector>
#include <string>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <algorithm>
#include <map>
#include <cassert> // Added for potential use, good practice

namespace mapgeo { // Namespace for all map processing related code

    // =================== NUMERICAL PRIMITIVES & UTILS ===================

    /**
     * @brief Provides type-specific numerical properties, like epsilon for floats.
     * @tparam T The numeric type.
     */
    template<typename T>
    struct numeric_traits {
        static constexpr T epsilon = std::numeric_limits<T>::epsilon() * 100;
    };

    /**
     * @brief Compares two floating-point numbers for approximate equality.
     * @tparam T Floating-point type (float or double).
     * @param a First value.
     * @param b Second value.
     * @return True if values are approximately equal within a calculated tolerance, false otherwise.
     */
    template<typename T>
    inline bool approx_equal_float(T a, T b) {
        static_assert(std::is_floating_point<T>::value, "approx_equal_float requires a floating-point type");
        if (a == b) return true; // Handle exact equality efficiently
        // Handle comparison near zero
        if (std::fabs(a) < numeric_traits<T>::epsilon && std::fabs(b) < numeric_traits<T>::epsilon) return true;
        // Relative epsilon comparison
        return std::abs(a - b) <= numeric_traits<T>::epsilon * std::max({ static_cast<T>(1.0), std::abs(a), std::abs(b) });
    }

    // =================== GEOMETRIC PRIMITIVES ===================

    /**
     * @brief Represents a point from the original XML coordinate data.
     *        Includes the original X, Y values and the integer flag associated with it.
     */
    struct Point {
        double x = 0.0;
        double y = 0.0;
        int flag = 0; // See CoordFlags enum
    };

    /**
     * @brief Represents a 2D point with floating-point coordinates, typically
     *        used for normalized coordinates after initial parsing.
     * @tparam T The floating-point type (e.g., float, double). Default is float.
     */
    template<typename T = float>
    struct Point_float {
        static_assert(std::is_floating_point<T>::value, "Point_float requires a floating-point type");
        T x = 0;
        T y = 0;
    };

    /**
     * @brief Represents a point with integer coordinates, typically corresponding
     *        to grid cell indices. Includes comparison operators and a hash function
     *        for use in sets/maps.
     */
    struct IntPoint {
        int x = 0;
        int y = 0;

        bool operator==(const IntPoint& other) const {
            return x == other.x && y == other.y;
        }
        // Define less-than for ordering (e.g., in std::set)
        bool operator<(const IntPoint& other) const {
            return y < other.y || (y == other.y && x < other.x);
        }
        // Hash function for use in unordered containers (e.g., std::unordered_set)
        struct Hash {
            std::size_t operator()(const IntPoint& p) const {
                // Simple hash combining x and y using prime numbers
                return static_cast<std::size_t>(p.x) * 73856093 ^ static_cast<std::size_t>(p.y) * 19349663;
            }
        };
    };

    // =================== GRID CELL & GRID STRUCTURES ===================

    /**
     * @brief Flags associated with coordinate points read from the XML.
     *        These are used during parsing and geometry preparation.
     */
    enum CoordFlags : int {
        NoFlag = 0,      // No special meaning
        CurveStart = 1,      // Point starts a Bezier curve segment (NOT IMPLEMENTED)
        ClosePoint = 2,      // Suggests closing a shape (often implicit)
        GapPoint = 4,      // Line segment *ending* at this point is a gap (not drawn)
        HolePoint = 16,     // Point marks the start of a hole boundary within an area
        DashPoint = 32      // Line segment *ending* at this point is a dash (not drawn)
    };

    /**
     * @brief Flags applied to the final grid cells, used for pathfinding logic.
     *        These indicate properties of the cell relevant to traversal.
     */
    enum GridFlags : std::uint8_t {
        FLAG_NONE = 0,        // Default state, traversable
        FLAG_BOUNDARY = 1 << 0,   // Cell is part of a drawn feature boundary (VISUAL/DEBUG ONLY for pathfinding)
        FLAG_FILLED = 1 << 1,   // Cell is part of a filled area (mostly for debug/intermediate)
        FLAG_IMPASSABLE = 1 << 2,   // Cell cannot be traversed (Checked by A*)
        FLAG_ROAD_PATH = 1 << 3,   // Cell represents a road/path feature
        FLAG_WATER_MARSH = 1 << 4,   // Cell represents water or marshy ground
        FLAG_UNDERGROWTH = 1 << 5    // Cell represents dense undergrowth (affects traversal cost)
        // Add more flags as needed
    };

    /**
     * @brief Data stored in each cell of the final traversability grid.
     *        Contains the base terrain traversability multiplier value and pathfinding flags.
     */
    struct GridCellData {
        float value = 1.0f;             // Base Terrain Cost Multiplier (Cost ~ value, <= 0.0f = impassable type)
        std::uint8_t flags = FLAG_NONE; // Bitmask of GridFlags

        // --- Flag Manipulation Methods ---
        inline void setFlag(GridFlags f) { flags |= f; }
        inline void clearFlag(GridFlags f) { flags &= ~f; }
        inline bool hasFlag(GridFlags f) const { return (flags & f) != 0; }
        inline void clearAllFlags() { flags = FLAG_NONE; }
        inline void setFlagsValue(std::uint8_t nf) { flags = nf; }

        // --- Comparison Operators ---
        bool operator==(const GridCellData& o) const {
            return flags == o.flags && approx_equal_float(value, o.value);
        }
        bool operator!=(const GridCellData& o) const {
            return !(*this == o);
        }
    };

    /**
     * @class Grid_V3
     * @brief Represents the 2D logical grid containing base terrain costs and flags.
     *        Stores GridCellData for each cell and provides access methods.
     */
    class Grid_V3 {
    public:
        /**
         * @brief Constructs a grid of specified dimensions.
         * @param w Width of the grid.
         * @param h Height of the grid.
         * @param initialValue The initial GridCellData for all cells.
         */
        Grid_V3(std::size_t w, std::size_t h, GridCellData initialValue = GridCellData{})
            : width_(w), height_(h), data_(w* h, initialValue)
        {
            if (w == 0 || h == 0) {
                // Allow zero-size construction, check dimensions before use elsewhere
                // throw std::invalid_argument("Grid_V3 dimensions cannot be zero.");
            }
        }
        // Default constructor for potential delayed initialization
        Grid_V3() : width_(0), height_(0) {}


        // Default copy/move constructors/assignments are suitable
        Grid_V3(const Grid_V3&) = default;
        Grid_V3& operator=(const Grid_V3&) = default;
        Grid_V3(Grid_V3&&) noexcept = default;
        Grid_V3& operator=(Grid_V3&&) noexcept = default;
        ~Grid_V3() = default;

        /**
         * @brief Checks if integer coordinates (x, y) are within the grid bounds.
         * @param x X-coordinate (column index).
         * @param y Y-coordinate (row index).
         * @return True if (x, y) is within bounds, false otherwise.
         */
        inline bool inBounds(int x, int y) const {
            // Check unsigned comparison for efficiency after casting
            return static_cast<unsigned>(x) < width_ && static_cast<unsigned>(y) < height_;
        }

        /**
         * @brief Provides mutable access to the GridCellData at (x, y).
         * @param x X-coordinate (column index).
         * @param y Y-coordinate (row index).
         * @return Reference to the GridCellData at (x, y).
         * @throws std::out_of_range if coordinates are out of bounds (in debug builds).
         */
        inline GridCellData& at(std::size_t x, std::size_t y) {
#ifndef NDEBUG // Perform bounds check only in debug builds
            if (!inBounds(static_cast<int>(x), static_cast<int>(y))) {
                throw std::out_of_range("Grid_V3::at() access out of bounds");
            }
#endif
            return data_[y * width_ + x]; // Row-major storage
        }

        /**
         * @brief Provides const access to the GridCellData at (x, y).
         * @param x X-coordinate (column index).
         * @param y Y-coordinate (row index).
         * @return Const reference to the GridCellData at (x, y).
         * @throws std::out_of_range if coordinates are out of bounds (in debug builds).
         */
        inline const GridCellData& at(std::size_t x, std::size_t y) const {
#ifndef NDEBUG // Perform bounds check only in debug builds
            if (!inBounds(static_cast<int>(x), static_cast<int>(y))) {
                throw std::out_of_range("Grid_V3::at() const access out of bounds");
            }
#endif
            return data_[y * width_ + x]; // Row-major storage
        }

        /** @brief Gets the width of the grid. */
        std::size_t width() const { return width_; }
        /** @brief Gets the height of the grid. */
        std::size_t height() const { return height_; }

        /** @brief Gets a mutable reference to the underlying grid data vector. */
        std::vector<GridCellData>& data() { return data_; }
        /** @brief Gets a const reference to the underlying grid data vector. */
        const std::vector<GridCellData>& data() const { return data_; }

        /**
         * @brief Resets all cells in the grid to a specified value.
         * @param value The GridCellData value to set for all cells. Defaults to a default-constructed cell.
         */
        void reset(GridCellData value = GridCellData{}) {
            std::fill(data_.begin(), data_.end(), value);
        }

        /** @brief Checks if the grid has valid dimensions (non-zero width and height). */
        bool isValid() const { return width_ > 0 && height_ > 0; }


    private:
        std::vector<GridCellData> data_;    // Contiguous storage for grid cells (row-major)
        std::size_t width_;                 // Grid width
        std::size_t height_;                // Grid height
    };

    // =================== MAP DATA STRUCTURES ===================

    /**
     * @brief Parsed information about a map symbol definition from the XML.
     */
    struct SymbolDefinition {
        std::string isom_code; // ISOM code (e.g., "505.1", "401")
        int symbol_type = -1;  // Original type from XML (0=point, 1=area, 2=line)
    };

    /**
     * @brief Intermediate storage for map objects after initial XML parsing.
     *        Contains original coordinates and references before normalization and rule processing.
     */
    struct IntermediateObjectData {
        std::vector<Point> original_points; // Original points with flags
        std::string original_symbol_id;     // ID referencing <symbol> definition
        int object_type = -1;               // Original type from XML (0=point, 1=area, 2=line)
    };

    /**
     * @brief Processed feature data ready for rasterization (Pass 2).
     *        Contains normalized coordinates separated into outer and hole boundaries,
     *        along with calculated traversal value and pathfinding flags.
     */
    struct FinalFeatureData {
        /** @brief Combines normalized position with the original coordinate flag. */
        struct VertexData {
            Point_float<float> pos;           // Normalized vertex position
            int original_flag = CoordFlags::NoFlag; // Flag from the original Point
        };

        std::vector<VertexData> outer_boundary;               // Normalized vertices for the main shape boundary
        std::vector<std::vector<VertexData>> hole_boundaries; // List of normalized hole boundaries (each is a list of vertices)
        float value = 1.0f;                                   // Calculated base terrain cost multiplier
        std::uint8_t specific_flags = GridFlags::FLAG_NONE;   // Calculated pathfinding flags for this feature
        int object_type = -1;                                 // Original object type (0=point, 1=area, 2=line)
        std::string original_symbol_id;                       // Symbol ID for debugging/lookup
    };

    /**
     * @brief Parameters derived from coordinate normalization, including resolution.
     */
    struct NormalizationResult {
        double min_x = 0.0;     // Real-world X coord corresponding to grid cell (0,0) center
        double min_y = 0.0;     // Real-world Y coord corresponding to grid cell (0,0) center
        double scale_x = 1.0;   // Scale factor X (grid units per real-world unit)
        double scale_y = 1.0;   // Scale factor Y (grid units per real-world unit)
        double resolution_x = 0.0; // Real-world X distance per grid cell step
        double resolution_y = 0.0; // Real-world Y distance per grid cell step
        bool valid = false;     // Flag indicating if parameters were successfully calculated
    };

    /**
     * @brief Configuration settings for the overall map processing.
     */
    struct ProcessingConfig {
        int grid_width = 100;                   // Desired width of the output grid
        int grid_height = 100;                  // Desired height of the output grid
        std::string input_xml_path = "map.xml"; // Default input file path
        std::vector<std::string> layers_to_process = { "barrier" }; // XML layer tags to process
    };

    /**
     * @brief Configuration settings for displaying the grid output.
     */
    struct DisplayConfig {
        size_t max_print_rows = 50;      // Max rows to print in console visualization
        size_t max_print_cols = 100;     // Max columns to print in console visualization
        bool enable_color = true;        // Use ANSI color codes for console output
        bool dump_numeric = true;        // Print a numeric dump of the top-left grid section
        size_t numeric_dump_dim = 10;    // Dimension (NxN) of the numeric dump
    };

} // namespace mapgeo
#endif // MAP_PROCESSING_COMMON_H


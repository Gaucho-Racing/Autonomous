/**
 * @file delaunay_planner.hpp
 * @brief Delaunay Triangulation-based Path Planner for Formula SAE Autonomous Racing
 * 
 * This header defines the core path planning algorithm that computes a safe,
 * smooth centerline path through a cone-defined track.
 * 
 * ============================================================================
 * ALGORITHM OVERVIEW
 * ============================================================================
 * 
 * 1. CONE INPUT
 *    - Blue cones define the LEFT track boundary
 *    - Yellow cones define the RIGHT track boundary
 *    - Positions are in vehicle frame (X=forward, Y=left)
 * 
 * 2. DELAUNAY TRIANGULATION
 *    - Connects all cone positions into optimal triangles
 *    - Uses Bowyer-Watson algorithm (O(n log n) average case)
 *    - Automatically identifies neighboring cones
 * 
 * 3. EDGE FILTERING
 *    - Extract edges that connect BLUE to YELLOW cones
 *    - These represent valid "cross-track" connections
 *    - Filter by maximum edge length to reject outliers
 * 
 * 4. MIDPOINT EXTRACTION
 *    - Calculate midpoint of each valid blue-yellow edge
 *    - These midpoints approximate the track centerline
 * 
 * 5. PATH ORDERING
 *    - Order midpoints from closest to farthest
 *    - Uses nearest-neighbor traversal
 * 
 * 6. SPLINE SMOOTHING
 *    - Fit cubic spline through ordered midpoints
 *    - Ensures C2 continuity (smooth position, velocity, acceleration)
 * 
 * 7. VELOCITY PROFILING
 *    - Compute curvature at each point
 *    - Set speed limits based on lateral acceleration constraints
 * 
 * ============================================================================
 * HARDWARE CONSTRAINTS (Jetson Orin Nano + RC Car)
 * ============================================================================
 * 
 * - Target planning frequency: 20-50 Hz
 * - Maximum cone count: ~50 (limited by camera FOV)
 * - Lookahead distance: 10-15m (ZED 2i depth range)
 * - Memory budget: <10MB for planner
 * - Must handle partial/noisy cone detections
 * 
 * ============================================================================
 * COORDINATE FRAMES
 * ============================================================================
 * 
 * Vehicle Frame (base_link):
 *   - Origin: Center of rear axle
 *   - X-axis: Forward (positive ahead)
 *   - Y-axis: Left (positive to driver's left)
 *   - Z-axis: Up
 * 
 * All inputs and outputs use this frame unless otherwise specified.
 * 
 * @author Your FSAE Team
 * @date 2025
 */

#ifndef PATH_PLANNER__DELAUNAY_PLANNER_HPP_
#define PATH_PLANNER__DELAUNAY_PLANNER_HPP_

#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <limits>
#include <optional>
#include <memory>

namespace fsae {
namespace path_planner {

// ============================================================================
// DATA STRUCTURES
// ============================================================================

/**
 * @brief Cone color classification
 * 
 * In Formula SAE/Formula Student Driverless:
 * - BLUE marks the LEFT boundary (driver's left when facing forward)
 * - YELLOW marks the RIGHT boundary
 * - ORANGE marks start/finish lines and special zones
 */
enum class ConeColor : uint8_t {
    BLUE = 0,           ///< Left boundary
    YELLOW = 1,         ///< Right boundary  
    ORANGE_SMALL = 2,   ///< Small orange cone
    ORANGE_LARGE = 3,   ///< Large orange cone (start/finish)
    UNKNOWN = 4         ///< Unclassified
};

/**
 * @brief Represents a single detected cone
 * 
 * Position is in vehicle frame coordinates.
 */
struct Cone {
    double x;               ///< X position [m] (forward from vehicle)
    double y;               ///< Y position [m] (left of vehicle)
    ConeColor color;        ///< Cone color classification
    float confidence;       ///< Detection confidence [0.0, 1.0]
    
    Cone() : x(0), y(0), color(ConeColor::UNKNOWN), confidence(1.0f) {}
    Cone(double x_, double y_, ConeColor color_, float conf = 1.0f)
        : x(x_), y(y_), color(color_), confidence(conf) {}
};

/**
 * @brief Single point on the planned path
 * 
 * Contains position, orientation, and kinematic information
 * needed by the controller.
 */
struct PathPoint {
    double x;               ///< X position [m]
    double y;               ///< Y position [m]
    double heading;         ///< Heading angle [rad] (0 = +X, CCW positive)
    double curvature;       ///< Path curvature [1/m] (positive = left turn)
    double speed;           ///< Target speed [m/s]
    double distance;        ///< Cumulative distance from path start [m]
    
    PathPoint() : x(0), y(0), heading(0), curvature(0), speed(0), distance(0) {}
};

/**
 * @brief Complete planned path
 */
struct PlannedPath {
    std::vector<PathPoint> points;  ///< Ordered path points
    double total_length;            ///< Total path length [m]
    bool valid;                     ///< True if path is valid/drivable
    
    PlannedPath() : total_length(0), valid(false) {}
};

/**
 * @brief Configuration parameters for the path planner
 * 
 * These should be tuned for your specific vehicle and track.
 */
struct PlannerConfig {
    // Geometric constraints
    double max_edge_length = 8.0;       ///< Max triangulation edge [m]
    double min_edge_length = 0.5;       ///< Min triangulation edge [m]
    double lookahead_distance = 15.0;   ///< How far ahead to plan [m]
    double lookbehind_distance = 2.0;   ///< Include cones slightly behind [m]
    
    // Path generation
    double smoothing_resolution = 0.2;  ///< Distance between path points [m]
    int min_path_points = 5;            ///< Minimum valid path length
    
    // Speed constraints (tune for your vehicle!)
    double max_speed = 8.0;             ///< Maximum speed [m/s] (~29 km/h)
    double min_speed = 2.0;             ///< Minimum speed [m/s]
    double max_lateral_accel = 6.0;     ///< Lateral acceleration limit [m/s²]
    double max_longitudinal_accel = 4.0;///< Longitudinal accel limit [m/s²]
    double max_longitudinal_decel = 6.0;///< Braking decel limit [m/s²]
    
    // Robustness
    int min_cones_per_side = 2;         ///< Minimum blue/yellow cones needed
    double cone_confidence_threshold = 0.3; ///< Min confidence to use cone
    
    // Track width estimation (for validation)
    double expected_track_width = 3.0;  ///< Expected track width [m]
    double track_width_tolerance = 1.5; ///< Allowed deviation [m]
};

// ============================================================================
// 2D GEOMETRY UTILITIES
// ============================================================================

/**
 * @brief 2D point/vector for geometric computations
 */
struct Point2D {
    double x, y;
    
    Point2D() : x(0), y(0) {}
    Point2D(double x_, double y_) : x(x_), y(y_) {}
    
    Point2D operator+(const Point2D& other) const {
        return Point2D(x + other.x, y + other.y);
    }
    
    Point2D operator-(const Point2D& other) const {
        return Point2D(x - other.x, y - other.y);
    }
    
    Point2D operator*(double scalar) const {
        return Point2D(x * scalar, y * scalar);
    }
    
    double dot(const Point2D& other) const {
        return x * other.x + y * other.y;
    }
    
    double cross(const Point2D& other) const {
        return x * other.y - y * other.x;
    }
    
    double norm() const {
        return std::sqrt(x * x + y * y);
    }
    
    double norm_squared() const {
        return x * x + y * y;
    }
    
    Point2D normalized() const {
        double n = norm();
        if (n < 1e-10) return Point2D(0, 0);
        return Point2D(x / n, y / n);
    }
};

/**
 * @brief Triangle for Delaunay triangulation
 */
struct Triangle {
    std::array<size_t, 3> vertices;  ///< Indices into point array
    Point2D circumcenter;            ///< Center of circumscribed circle
    double circumradius_sq;          ///< Squared radius of circumcircle
    
    Triangle() : vertices{0, 0, 0}, circumradius_sq(0) {}
    Triangle(size_t v0, size_t v1, size_t v2) : vertices{v0, v1, v2}, circumradius_sq(0) {}
};

/**
 * @brief Edge in the triangulation
 */
struct Edge {
    size_t v0, v1;  ///< Vertex indices (v0 < v1 for uniqueness)
    
    Edge(size_t a, size_t b) {
        v0 = std::min(a, b);
        v1 = std::max(a, b);
    }
    
    bool operator==(const Edge& other) const {
        return v0 == other.v0 && v1 == other.v1;
    }
    
    bool operator<(const Edge& other) const {
        if (v0 != other.v0) return v0 < other.v0;
        return v1 < other.v1;
    }
};

// ============================================================================
// DELAUNAY PATH PLANNER CLASS
// ============================================================================

/**
 * @brief Main path planner class using Delaunay triangulation
 * 
 * This class implements the complete planning pipeline from cone
 * detections to a smooth, speed-profiled path.
 * 
 * Thread Safety: NOT thread-safe. Use external synchronization
 * or create separate instances per thread.
 * 
 * Example usage:
 * @code
 * fsae::path_planner::DelaunayPlanner planner;
 * planner.configure(config);
 * 
 * std::vector<Cone> cones = getConeDetections();
 * PlannedPath path = planner.plan(cones);
 * 
 * if (path.valid) {
 *     // Use path.points for control
 * }
 * @endcode
 */
class DelaunayPlanner {
public:
    /**
     * @brief Default constructor with default configuration
     */
    DelaunayPlanner();
    
    /**
     * @brief Constructor with custom configuration
     * @param config Planner configuration parameters
     */
    explicit DelaunayPlanner(const PlannerConfig& config);
    
    /**
     * @brief Update configuration at runtime
     * @param config New configuration parameters
     */
    void configure(const PlannerConfig& config);
    
    /**
     * @brief Get current configuration
     * @return Current planner configuration
     */
    const PlannerConfig& getConfig() const { return config_; }
    
    /**
     * @brief Main planning function
     * 
     * Takes detected cones and produces a planned path.
     * This is the main entry point called each planning cycle.
     * 
     * @param cones Vector of detected cones with positions and colors
     * @return PlannedPath containing path points and validity flag
     * 
     * Performance: O(n log n) for n cones, typically <1ms for 50 cones
     */
    PlannedPath plan(const std::vector<Cone>& cones);
    
    /**
     * @brief Get the raw midpoints before smoothing (for debugging)
     * @return Vector of midpoint coordinates
     */
    const std::vector<Point2D>& getRawMidpoints() const { return midpoints_; }
    
    /**
     * @brief Get triangulation edges (for visualization)
     * @return Vector of valid cross-track edges
     */
    const std::vector<Edge>& getValidEdges() const { return valid_edges_; }

private:
    // Configuration
    PlannerConfig config_;
    
    // Internal state (reused to avoid allocations)
    std::vector<Point2D> points_;           ///< Cone positions for triangulation
    std::vector<ConeColor> colors_;         ///< Color of each point
    std::vector<Triangle> triangles_;       ///< Delaunay triangles
    std::vector<Edge> valid_edges_;         ///< Blue-yellow edges
    std::vector<Point2D> midpoints_;        ///< Track centerline points
    std::vector<Point2D> ordered_midpoints_;///< Ordered by distance
    
    // Spline coefficients (for cubic spline)
    std::vector<double> spline_ax_, spline_bx_, spline_cx_, spline_dx_;
    std::vector<double> spline_ay_, spline_by_, spline_cy_, spline_dy_;
    std::vector<double> spline_t_;
    
    // ========================================================================
    // PIPELINE STAGES
    // ========================================================================
    
    /**
     * @brief Filter cones by distance and confidence
     * 
     * Removes cones that are:
     * - Beyond lookahead distance
     * - Behind the vehicle (beyond lookbehind)
     * - Below confidence threshold
     * 
     * @param cones Input cone detections
     * @return Filtered cones ready for processing
     */
    std::vector<Cone> filterCones(const std::vector<Cone>& cones);
    
    /**
     * @brief Perform Delaunay triangulation on cone positions
     * 
     * Implements the Bowyer-Watson incremental algorithm:
     * 1. Create super-triangle containing all points
     * 2. Add points one by one
     * 3. For each point, find triangles whose circumcircle contains it
     * 4. Remove those triangles and re-triangulate the cavity
     * 5. Remove super-triangle vertices
     * 
     * @return True if triangulation succeeded
     */
    bool computeDelaunay();
    
    /**
     * @brief Extract valid cross-track edges from triangulation
     * 
     * A valid edge connects a BLUE cone to a YELLOW cone
     * and has length within acceptable bounds.
     */
    void extractValidEdges();
    
    /**
     * @brief Calculate midpoints of valid edges
     * 
     * These midpoints form the raw (unsmoothed) centerline.
     */
    void calculateMidpoints();
    
    /**
     * @brief Order midpoints from closest to farthest
     * 
     * Uses greedy nearest-neighbor algorithm starting
     * from the point closest to the vehicle.
     */
    void orderMidpoints();
    
    /**
     * @brief Fit cubic spline through ordered midpoints
     * 
     * Uses natural cubic spline (second derivative = 0 at endpoints).
     * Stores spline coefficients for fast evaluation.
     * 
     * @return True if spline fitting succeeded
     */
    bool fitSpline();
    
    /**
     * @brief Sample the spline at regular intervals
     * 
     * @param path Output path to populate with sampled points
     */
    void sampleSpline(PlannedPath& path);
    
    /**
     * @brief Compute heading and curvature at each path point
     * 
     * Uses spline derivatives for accurate computation:
     * - heading = atan2(dy/dt, dx/dt)
     * - curvature = (dx/dt * d²y/dt² - dy/dt * d²x/dt²) / (dx/dt² + dy/dt²)^(3/2)
     */
    void computePathKinematics(PlannedPath& path);
    
    /**
     * @brief Compute speed profile based on curvature and acceleration limits
     * 
     * Three-pass algorithm:
     * 1. Forward pass: limit acceleration
     * 2. Backward pass: limit deceleration
     * 3. Curvature pass: limit lateral acceleration
     */
    void computeSpeedProfile(PlannedPath& path);
    
    /**
     * @brief Generate fallback path when normal planning fails
     * 
     * Creates a simple straight path at minimum speed.
     * Used when insufficient cones are detected.
     * 
     * @return Fallback path
     */
    PlannedPath generateFallbackPath();
    
    // ========================================================================
    // GEOMETRY UTILITIES
    // ========================================================================
    
    /**
     * @brief Compute circumcircle of a triangle
     * 
     * The circumcircle passes through all three vertices.
     * 
     * @param p0, p1, p2 Triangle vertices
     * @param center Output circumcenter
     * @param radius_sq Output squared circumradius
     * @return True if triangle is non-degenerate
     */
    bool computeCircumcircle(const Point2D& p0, const Point2D& p1, const Point2D& p2,
                             Point2D& center, double& radius_sq);
    
    /**
     * @brief Check if point is inside triangle's circumcircle
     * 
     * @param p Point to test
     * @param tri Triangle to test against
     * @return True if point is strictly inside circumcircle
     */
    bool isInsideCircumcircle(const Point2D& p, const Triangle& tri);
    
    /**
     * @brief Solve tridiagonal system for cubic spline
     * 
     * Uses Thomas algorithm (O(n) time complexity).
     * 
     * @param n System size
     * @param a Lower diagonal
     * @param b Main diagonal
     * @param c Upper diagonal
     * @param d Right-hand side (modified in place with solution)
     */
    void solveTridiagonal(int n, const std::vector<double>& a,
                          const std::vector<double>& b,
                          const std::vector<double>& c,
                          std::vector<double>& d);
};

}  // namespace path_planner
}  // namespace fsae

#endif  // PATH_PLANNER__DELAUNAY_PLANNER_HPP_

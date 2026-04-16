/**
 * @file delaunay_planner.cpp
 * @brief Implementation of Delaunay Triangulation Path Planner
 * 
 * This file contains the complete implementation of the path planning
 * algorithm. Each function is heavily documented to explain both
 * WHAT it does and WHY it does it that way.
 */

#include "path_planner/delaunay_planner.hpp"
#include <set>
#include <cstring>
#include <stdexcept>

namespace fsae {
namespace path_planner {

// ============================================================================
// CONSTRUCTOR / CONFIGURATION
// ============================================================================

DelaunayPlanner::DelaunayPlanner() : config_() {
    // Reserve memory to avoid allocations during planning
    // Typical FSAE track has 20-50 visible cones
    points_.reserve(64);
    colors_.reserve(64);
    triangles_.reserve(128);
    valid_edges_.reserve(64);
    midpoints_.reserve(32);
    ordered_midpoints_.reserve(32);
}

DelaunayPlanner::DelaunayPlanner(const PlannerConfig& config) : config_(config) {
    points_.reserve(64);
    colors_.reserve(64);
    triangles_.reserve(128);
    valid_edges_.reserve(64);
    midpoints_.reserve(32);
    ordered_midpoints_.reserve(32);
}

void DelaunayPlanner::configure(const PlannerConfig& config) {
    config_ = config;
}

// ============================================================================
// MAIN PLANNING FUNCTION
// ============================================================================

PlannedPath DelaunayPlanner::plan(const std::vector<Cone>& cones) {
    /*
     * PLANNING PIPELINE OVERVIEW:
     * 
     * Input: Raw cone detections from perception
     *    ↓
     * [1] Filter cones by distance, confidence
     *    ↓
     * [2] Delaunay triangulation
     *    ↓
     * [3] Extract blue-yellow edges
     *    ↓
     * [4] Calculate edge midpoints
     *    ↓
     * [5] Order midpoints by proximity
     *    ↓
     * [6] Fit cubic spline
     *    ↓
     * [7] Sample spline at fixed intervals
     *    ↓
     * [8] Compute heading & curvature
     *    ↓
     * [9] Compute speed profile
     *    ↓
     * Output: Smooth, drivable path with velocity targets
     */
    
    // Step 1: Filter and validate cones
    std::vector<Cone> filtered = filterCones(cones);
    
    // Count cones by color
    int blue_count = 0, yellow_count = 0;
    for (const auto& cone : filtered) {
        if (cone.color == ConeColor::BLUE) blue_count++;
        else if (cone.color == ConeColor::YELLOW) yellow_count++;
    }
    
    // Need minimum cones on each side for valid triangulation
    if (blue_count < config_.min_cones_per_side || 
        yellow_count < config_.min_cones_per_side) {
        return generateFallbackPath();
    }
    
    // Convert to internal format
    points_.clear();
    colors_.clear();
    for (const auto& cone : filtered) {
        points_.push_back(Point2D(cone.x, cone.y));
        colors_.push_back(cone.color);
    }
    
    // Step 2: Delaunay triangulation
    if (!computeDelaunay()) {
        return generateFallbackPath();
    }
    
    // Step 3: Extract valid edges
    extractValidEdges();
    
    if (valid_edges_.empty()) {
        return generateFallbackPath();
    }
    
    // Step 4: Calculate midpoints
    calculateMidpoints();
    
    if (midpoints_.size() < 2) {
        return generateFallbackPath();
    }
    
    // Step 5: Order midpoints
    orderMidpoints();
    
    // Step 6: Fit spline
    if (!fitSpline()) {
        return generateFallbackPath();
    }
    
    // Step 7-9: Sample and compute properties
    PlannedPath path;
    sampleSpline(path);
    computePathKinematics(path);
    computeSpeedProfile(path);
    
    // Validate path
    path.valid = (path.points.size() >= static_cast<size_t>(config_.min_path_points));
    
    return path;
}

// ============================================================================
// CONE FILTERING
// ============================================================================

std::vector<Cone> DelaunayPlanner::filterCones(const std::vector<Cone>& cones) {
    /*
     * WHY FILTER?
     * 
     * 1. Distance: Cones too far away have uncertain positions due to:
     *    - Depth sensor noise increases with distance
     *    - Object detection accuracy degrades
     *    - We only need to plan the immediate path
     * 
     * 2. Behind vehicle: Cones behind us aren't useful for planning
     *    (but we keep a small margin for robustness)
     * 
     * 3. Confidence: Low-confidence detections may be false positives
     *    (e.g., mistaking a person's leg for a cone)
     */
    
    std::vector<Cone> filtered;
    filtered.reserve(cones.size());
    
    for (const auto& cone : cones) {
        // Skip low-confidence detections
        if (cone.confidence < config_.cone_confidence_threshold) {
            continue;
        }
        
        // Skip unknown color cones (can't determine boundary)
        if (cone.color == ConeColor::UNKNOWN) {
            continue;
        }
        
        // Calculate distance from vehicle
        double distance = std::sqrt(cone.x * cone.x + cone.y * cone.y);
        
        // Skip cones beyond lookahead
        if (distance > config_.lookahead_distance) {
            continue;
        }
        
        // Skip cones too far behind
        if (cone.x < -config_.lookbehind_distance) {
            continue;
        }
        
        filtered.push_back(cone);
    }
    
    return filtered;
}

// ============================================================================
// DELAUNAY TRIANGULATION (Bowyer-Watson Algorithm)
// ============================================================================

bool DelaunayPlanner::computeDelaunay() {
    /*
     * BOWYER-WATSON ALGORITHM:
     * 
     * This is the standard incremental algorithm for Delaunay triangulation.
     * It builds the triangulation by adding points one at a time.
     * 
     * Time complexity: O(n log n) average, O(n²) worst case
     * Space complexity: O(n)
     * 
     * Steps:
     * 1. Create a "super-triangle" that contains all input points
     * 2. For each point p:
     *    a. Find all triangles whose circumcircle contains p
     *    b. These triangles form a "cavity"
     *    c. Delete these triangles
     *    d. Create new triangles connecting p to the cavity boundary
     * 3. Remove all triangles that share vertices with super-triangle
     * 
     * The key insight is that for a valid Delaunay triangulation,
     * no point should be inside any triangle's circumcircle.
     */
    
    if (points_.size() < 3) {
        return false;
    }
    
    triangles_.clear();
    
    // Find bounding box of all points
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();
    
    for (const auto& p : points_) {
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
    }
    
    // Add margin to bounding box
    double dx = max_x - min_x;
    double dy = max_y - min_y;
    double margin = std::max(dx, dy) * 2.0 + 10.0;
    
    // Create super-triangle vertices (stored at end of points array)
    // These will be removed after triangulation
    size_t super_v0 = points_.size();
    size_t super_v1 = points_.size() + 1;
    size_t super_v2 = points_.size() + 2;
    
    // Super-triangle must contain all points with comfortable margin
    points_.push_back(Point2D(min_x - margin, min_y - margin));
    points_.push_back(Point2D(max_x + margin, min_y - margin));
    points_.push_back(Point2D((min_x + max_x) / 2, max_y + margin));
    
    // Add placeholder colors for super-triangle vertices
    colors_.push_back(ConeColor::UNKNOWN);
    colors_.push_back(ConeColor::UNKNOWN);
    colors_.push_back(ConeColor::UNKNOWN);
    
    // Initialize with super-triangle
    Triangle super_tri(super_v0, super_v1, super_v2);
    computeCircumcircle(points_[super_v0], points_[super_v1], points_[super_v2],
                        super_tri.circumcenter, super_tri.circumradius_sq);
    triangles_.push_back(super_tri);
    
    // Add points one by one
    std::vector<Triangle> bad_triangles;
    std::vector<Edge> polygon;  // Boundary of the cavity
    
    for (size_t i = 0; i < super_v0; ++i) {  // Don't process super-triangle vertices
        const Point2D& p = points_[i];
        
        bad_triangles.clear();
        polygon.clear();
        
        // Find all triangles whose circumcircle contains point p
        for (const auto& tri : triangles_) {
            if (isInsideCircumcircle(p, tri)) {
                bad_triangles.push_back(tri);
            }
        }
        
        // Find the boundary of the cavity (edges that belong to only one bad triangle)
        // This is the "polygon" that will be retriangulated
        std::vector<Edge> all_edges;
        for (const auto& tri : bad_triangles) {
            all_edges.push_back(Edge(tri.vertices[0], tri.vertices[1]));
            all_edges.push_back(Edge(tri.vertices[1], tri.vertices[2]));
            all_edges.push_back(Edge(tri.vertices[2], tri.vertices[0]));
        }
        
        // Sort edges to find duplicates
        std::sort(all_edges.begin(), all_edges.end());
        
        // Edges that appear exactly once are on the boundary
        for (size_t j = 0; j < all_edges.size(); ) {
            size_t count = 1;
            while (j + count < all_edges.size() && all_edges[j] == all_edges[j + count]) {
                count++;
            }
            if (count == 1) {
                polygon.push_back(all_edges[j]);
            }
            j += count;
        }
        
        // Remove bad triangles
        triangles_.erase(
            std::remove_if(triangles_.begin(), triangles_.end(),
                [&bad_triangles](const Triangle& t) {
                    for (const auto& bad : bad_triangles) {
                        if (t.vertices == bad.vertices) return true;
                    }
                    return false;
                }),
            triangles_.end()
        );
        
        // Create new triangles from p to each edge of the polygon
        for (const auto& edge : polygon) {
            Triangle new_tri(i, edge.v0, edge.v1);
            if (computeCircumcircle(points_[i], points_[edge.v0], points_[edge.v1],
                                    new_tri.circumcenter, new_tri.circumradius_sq)) {
                triangles_.push_back(new_tri);
            }
        }
    }
    
    // Remove triangles that share a vertex with super-triangle
    triangles_.erase(
        std::remove_if(triangles_.begin(), triangles_.end(),
            [super_v0, super_v1, super_v2](const Triangle& t) {
                for (size_t v : t.vertices) {
                    if (v == super_v0 || v == super_v1 || v == super_v2) {
                        return true;
                    }
                }
                return false;
            }),
        triangles_.end()
    );
    
    // Remove super-triangle vertices
    points_.resize(super_v0);
    colors_.resize(super_v0);
    
    return !triangles_.empty();
}

// ============================================================================
// EDGE EXTRACTION AND MIDPOINT CALCULATION
// ============================================================================

void DelaunayPlanner::extractValidEdges() {
    /*
     * VALID EDGE DEFINITION:
     * 
     * An edge is "valid" for path planning if it connects a BLUE cone
     * to a YELLOW cone. This represents a "cross-track" connection,
     * meaning it goes from the left boundary to the right boundary.
     * 
     * We filter out:
     * - Blue-to-blue edges (along left boundary)
     * - Yellow-to-yellow edges (along right boundary)
     * - Orange edges (special markers)
     * - Edges that are too short (noise)
     * - Edges that are too long (incorrect connections)
     * 
     * The track width constraint helps reject spurious edges that
     * might connect cones across different track sections.
     */
    
    valid_edges_.clear();
    std::set<Edge> unique_edges;
    
    // Extract all edges from triangulation
    for (const auto& tri : triangles_) {
        unique_edges.insert(Edge(tri.vertices[0], tri.vertices[1]));
        unique_edges.insert(Edge(tri.vertices[1], tri.vertices[2]));
        unique_edges.insert(Edge(tri.vertices[2], tri.vertices[0]));
    }
    
    // Filter to valid cross-track edges
    for (const auto& edge : unique_edges) {
        ConeColor color0 = colors_[edge.v0];
        ConeColor color1 = colors_[edge.v1];
        
        // Check if this is a blue-yellow edge
        bool is_blue_yellow = 
            (color0 == ConeColor::BLUE && color1 == ConeColor::YELLOW) ||
            (color0 == ConeColor::YELLOW && color1 == ConeColor::BLUE);
        
        if (!is_blue_yellow) {
            continue;
        }
        
        // Check edge length
        const Point2D& p0 = points_[edge.v0];
        const Point2D& p1 = points_[edge.v1];
        double length = (p1 - p0).norm();
        
        if (length < config_.min_edge_length || length > config_.max_edge_length) {
            continue;
        }
        
        // Check against expected track width (soft constraint)
        double width_error = std::abs(length - config_.expected_track_width);
        if (width_error > config_.track_width_tolerance) {
            continue;
        }
        
        valid_edges_.push_back(edge);
    }
}

void DelaunayPlanner::calculateMidpoints() {
    /*
     * MIDPOINT CALCULATION:
     * 
     * For each valid edge, the midpoint represents the approximate
     * track centerline at that cross-section.
     * 
     * midpoint = (blue_cone + yellow_cone) / 2
     * 
     * In reality, the racing line isn't exactly at the midpoint -
     * you want to "apex" corners by going closer to the inside.
     * However, for safety and simplicity, the centerline is a good
     * starting point. Advanced implementations can offset the path
     * toward apexes.
     */
    
    midpoints_.clear();
    
    for (const auto& edge : valid_edges_) {
        const Point2D& p0 = points_[edge.v0];
        const Point2D& p1 = points_[edge.v1];
        
        Point2D midpoint((p0.x + p1.x) / 2.0, (p0.y + p1.y) / 2.0);
        midpoints_.push_back(midpoint);
    }
}

// ============================================================================
// MIDPOINT ORDERING
// ============================================================================

void DelaunayPlanner::orderMidpoints() {
    /*
     * NEAREST-NEIGHBOR PATH ORDERING:
     * 
     * The midpoints are unordered - we need to sequence them to form
     * a logical path. We use a greedy nearest-neighbor approach:
     * 
     * 1. Start with the midpoint closest to the vehicle (origin)
     * 2. Repeatedly add the nearest unvisited midpoint
     * 
     * This is O(n²) which is fine for <50 midpoints. For larger
     * point sets, you'd use a spatial data structure (k-d tree).
     * 
     * LIMITATIONS:
     * - May not find globally optimal ordering
     * - Can fail for complex track layouts (hairpin turns)
     * - Assumes points are roughly sequential along track
     * 
     * For FSAE tracks (which don't have extreme geometry), this
     * simple approach works well.
     */
    
    if (midpoints_.size() <= 1) {
        ordered_midpoints_ = midpoints_;
        return;
    }
    
    ordered_midpoints_.clear();
    ordered_midpoints_.reserve(midpoints_.size());
    
    std::vector<bool> visited(midpoints_.size(), false);
    
    // Find starting point (closest to origin)
    size_t current = 0;
    double min_dist = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < midpoints_.size(); ++i) {
        double dist = midpoints_[i].norm();
        if (dist < min_dist) {
            min_dist = dist;
            current = i;
        }
    }
    
    // Greedy nearest-neighbor traversal
    ordered_midpoints_.push_back(midpoints_[current]);
    visited[current] = true;
    
    for (size_t count = 1; count < midpoints_.size(); ++count) {
        const Point2D& current_point = midpoints_[current];
        
        size_t nearest = 0;
        double nearest_dist = std::numeric_limits<double>::max();
        
        for (size_t i = 0; i < midpoints_.size(); ++i) {
            if (visited[i]) continue;
            
            double dist = (midpoints_[i] - current_point).norm();
            if (dist < nearest_dist) {
                nearest_dist = dist;
                nearest = i;
            }
        }
        
        ordered_midpoints_.push_back(midpoints_[nearest]);
        visited[nearest] = true;
        current = nearest;
    }
}

// ============================================================================
// CUBIC SPLINE FITTING
// ============================================================================

bool DelaunayPlanner::fitSpline() {
    /*
     * CUBIC SPLINE INTERPOLATION:
     * 
     * We fit a parametric cubic spline through the ordered midpoints:
     *   x(t) = spline_x(t)
     *   y(t) = spline_y(t)
     * 
     * where t is the cumulative arc length (distance along path).
     * 
     * CUBIC SPLINE PROPERTIES:
     * - Passes through all control points
     * - C2 continuous (position, velocity, acceleration all smooth)
     * - "Natural" boundary condition: second derivative = 0 at endpoints
     * 
     * MATHEMATICAL FORMULATION:
     * 
     * For each segment i between t[i] and t[i+1]:
     *   S_i(t) = a_i + b_i*(t-t_i) + c_i*(t-t_i)² + d_i*(t-t_i)³
     * 
     * Constraints:
     * - S_i(t_i) = y_i                    (pass through points)
     * - S_i(t_{i+1}) = y_{i+1}            (continuity)
     * - S'_i(t_{i+1}) = S'_{i+1}(t_{i+1}) (velocity continuity)
     * - S''_i(t_{i+1}) = S''_{i+1}(t_{i+1}) (acceleration continuity)
     * - S''_0(t_0) = 0, S''_n(t_n) = 0   (natural boundary)
     * 
     * This leads to a tridiagonal system that can be solved in O(n) time.
     */
    
    const size_t n = ordered_midpoints_.size();
    
    if (n < 3) {
        return false;
    }
    
    // Compute parameter values (cumulative arc length)
    spline_t_.resize(n);
    spline_t_[0] = 0.0;
    
    for (size_t i = 1; i < n; ++i) {
        double dx = ordered_midpoints_[i].x - ordered_midpoints_[i-1].x;
        double dy = ordered_midpoints_[i].y - ordered_midpoints_[i-1].y;
        spline_t_[i] = spline_t_[i-1] + std::sqrt(dx*dx + dy*dy);
    }
    
    if (spline_t_.back() < 0.1) {
        return false;  // Path too short
    }
    
    // Extract x and y coordinates
    std::vector<double> x(n), y(n);
    for (size_t i = 0; i < n; ++i) {
        x[i] = ordered_midpoints_[i].x;
        y[i] = ordered_midpoints_[i].y;
    }
    
    // Compute spline coefficients for x(t) and y(t)
    // Both use the same parameter t
    
    const size_t nm1 = n - 1;
    
    // Compute h[i] = t[i+1] - t[i]
    std::vector<double> h(nm1);
    for (size_t i = 0; i < nm1; ++i) {
        h[i] = spline_t_[i+1] - spline_t_[i];
    }
    
    // Build tridiagonal system for second derivatives
    // System: a[i]*M[i-1] + b[i]*M[i] + c[i]*M[i+1] = d[i]
    // where M[i] = S''(t_i)
    
    std::vector<double> a(n), b(n), c(n), dx_vec(n), dy_vec(n);
    
    // Natural spline boundary conditions: M[0] = M[n-1] = 0
    a[0] = 0; b[0] = 1; c[0] = 0; dx_vec[0] = 0; dy_vec[0] = 0;
    a[nm1] = 0; b[nm1] = 1; c[nm1] = 0; dx_vec[nm1] = 0; dy_vec[nm1] = 0;
    
    // Interior points
    for (size_t i = 1; i < nm1; ++i) {
        a[i] = h[i-1];
        b[i] = 2.0 * (h[i-1] + h[i]);
        c[i] = h[i];
        
        dx_vec[i] = 6.0 * ((x[i+1] - x[i]) / h[i] - (x[i] - x[i-1]) / h[i-1]);
        dy_vec[i] = 6.0 * ((y[i+1] - y[i]) / h[i] - (y[i] - y[i-1]) / h[i-1]);
    }
    
    // Solve for M (second derivatives)
    solveTridiagonal(n, a, b, c, dx_vec);  // dx_vec now contains M_x
    solveTridiagonal(n, a, b, c, dy_vec);  // dy_vec now contains M_y
    
    // Compute spline coefficients for each segment
    // S_i(t) = a_i + b_i*(t-t_i) + c_i*(t-t_i)² + d_i*(t-t_i)³
    
    spline_ax_.resize(nm1);
    spline_bx_.resize(nm1);
    spline_cx_.resize(nm1);
    spline_dx_.resize(nm1);
    
    spline_ay_.resize(nm1);
    spline_by_.resize(nm1);
    spline_cy_.resize(nm1);
    spline_dy_.resize(nm1);
    
    for (size_t i = 0; i < nm1; ++i) {
        // X coefficients
        spline_ax_[i] = x[i];
        spline_bx_[i] = (x[i+1] - x[i]) / h[i] - h[i] * (2*dx_vec[i] + dx_vec[i+1]) / 6.0;
        spline_cx_[i] = dx_vec[i] / 2.0;
        spline_dx_[i] = (dx_vec[i+1] - dx_vec[i]) / (6.0 * h[i]);
        
        // Y coefficients
        spline_ay_[i] = y[i];
        spline_by_[i] = (y[i+1] - y[i]) / h[i] - h[i] * (2*dy_vec[i] + dy_vec[i+1]) / 6.0;
        spline_cy_[i] = dy_vec[i] / 2.0;
        spline_dy_[i] = (dy_vec[i+1] - dy_vec[i]) / (6.0 * h[i]);
    }
    
    return true;
}

void DelaunayPlanner::solveTridiagonal(int n, const std::vector<double>& a,
                                        const std::vector<double>& b,
                                        const std::vector<double>& c,
                                        std::vector<double>& d) {
    /*
     * THOMAS ALGORITHM (Tridiagonal Matrix Algorithm):
     * 
     * Solves the system:
     *   b[0]*x[0] + c[0]*x[1] = d[0]
     *   a[i]*x[i-1] + b[i]*x[i] + c[i]*x[i+1] = d[i]  for i = 1..n-2
     *   a[n-1]*x[n-2] + b[n-1]*x[n-1] = d[n-1]
     * 
     * Time complexity: O(n)
     * Space complexity: O(n)
     * 
     * The algorithm modifies d in place to contain the solution.
     */
    
    if (n <= 0) return;
    
    std::vector<double> c_prime(n);
    std::vector<double> d_prime(n);
    
    // Forward sweep
    c_prime[0] = c[0] / b[0];
    d_prime[0] = d[0] / b[0];
    
    for (int i = 1; i < n; ++i) {
        double denom = b[i] - a[i] * c_prime[i-1];
        if (std::abs(denom) < 1e-12) {
            denom = 1e-12;  // Prevent division by zero
        }
        c_prime[i] = c[i] / denom;
        d_prime[i] = (d[i] - a[i] * d_prime[i-1]) / denom;
    }
    
    // Back substitution
    d[n-1] = d_prime[n-1];
    for (int i = n - 2; i >= 0; --i) {
        d[i] = d_prime[i] - c_prime[i] * d[i+1];
    }
}

// ============================================================================
// SPLINE SAMPLING AND PATH PROPERTY COMPUTATION
// ============================================================================

void DelaunayPlanner::sampleSpline(PlannedPath& path) {
    /*
     * SPLINE SAMPLING:
     * 
     * We evaluate the spline at regular intervals along the
     * parameter t (arc length). This gives evenly-spaced points
     * along the path, which is important for the controller.
     * 
     * The sampling resolution (config_.smoothing_resolution) determines
     * how many path points we generate. Finer resolution means smoother
     * control but more computation.
     */
    
    path.points.clear();
    
    if (spline_t_.empty() || spline_ax_.empty()) {
        return;
    }
    
    double total_length = spline_t_.back();
    int num_samples = static_cast<int>(total_length / config_.smoothing_resolution) + 1;
    num_samples = std::max(num_samples, config_.min_path_points);
    
    path.points.reserve(num_samples);
    
    size_t segment = 0;  // Current spline segment
    
    for (int i = 0; i < num_samples; ++i) {
        double t = (i * total_length) / (num_samples - 1);
        
        // Find the correct segment for this t value
        while (segment < spline_t_.size() - 2 && t > spline_t_[segment + 1]) {
            segment++;
        }
        
        // Local parameter within segment
        double dt = t - spline_t_[segment];
        
        // Evaluate spline
        PathPoint point;
        point.x = spline_ax_[segment] + spline_bx_[segment] * dt + 
                  spline_cx_[segment] * dt * dt + spline_dx_[segment] * dt * dt * dt;
        point.y = spline_ay_[segment] + spline_by_[segment] * dt + 
                  spline_cy_[segment] * dt * dt + spline_dy_[segment] * dt * dt * dt;
        point.distance = t;
        
        path.points.push_back(point);
    }
    
    path.total_length = total_length;
}

void DelaunayPlanner::computePathKinematics(PlannedPath& path) {
    /*
     * KINEMATIC PROPERTY COMPUTATION:
     * 
     * For each path point, we compute:
     * 
     * 1. HEADING: Direction of travel
     *    heading = atan2(dy/dt, dx/dt)
     * 
     * 2. CURVATURE: How sharply the path turns
     *    κ = (dx/dt * d²y/dt² - dy/dt * d²x/dt²) / (dx/dt² + dy/dt²)^(3/2)
     * 
     * The curvature is critical for:
     * - Computing safe cornering speeds
     * - Feedforward steering commands
     * - Path tracking controllers
     * 
     * We compute derivatives analytically from the spline coefficients:
     *   dx/dt = b + 2*c*dt + 3*d*dt²
     *   d²x/dt² = 2*c + 6*d*dt
     */
    
    if (path.points.size() < 2 || spline_ax_.empty()) {
        return;
    }
    
    size_t segment = 0;
    
    for (auto& point : path.points) {
        // Find segment for this point
        double t = point.distance;
        while (segment < spline_t_.size() - 2 && t > spline_t_[segment + 1]) {
            segment++;
        }
        
        double dt = t - spline_t_[segment];
        
        // First derivatives
        double dx_dt = spline_bx_[segment] + 2.0 * spline_cx_[segment] * dt + 
                       3.0 * spline_dx_[segment] * dt * dt;
        double dy_dt = spline_by_[segment] + 2.0 * spline_cy_[segment] * dt + 
                       3.0 * spline_dy_[segment] * dt * dt;
        
        // Second derivatives
        double d2x_dt2 = 2.0 * spline_cx_[segment] + 6.0 * spline_dx_[segment] * dt;
        double d2y_dt2 = 2.0 * spline_cy_[segment] + 6.0 * spline_dy_[segment] * dt;
        
        // Heading
        point.heading = std::atan2(dy_dt, dx_dt);
        
        // Curvature
        double velocity_sq = dx_dt * dx_dt + dy_dt * dy_dt;
        double velocity_cubed = velocity_sq * std::sqrt(velocity_sq);
        
        if (velocity_cubed > 1e-6) {
            point.curvature = (dx_dt * d2y_dt2 - dy_dt * d2x_dt2) / velocity_cubed;
        } else {
            point.curvature = 0.0;
        }
    }
}

void DelaunayPlanner::computeSpeedProfile(PlannedPath& path) {
    /*
     * SPEED PROFILE COMPUTATION:
     * 
     * We use a three-pass algorithm to ensure physically realizable speeds:
     * 
     * PASS 1 (Curvature): Set maximum speed based on lateral acceleration
     *   v_max = sqrt(a_lat_max / |κ|)
     * 
     * PASS 2 (Forward): Limit acceleration
     *   v[i+1] <= sqrt(v[i]² + 2 * a_max * ds)
     * 
     * PASS 3 (Backward): Limit deceleration
     *   v[i] <= sqrt(v[i+1]² + 2 * a_brake_max * ds)
     * 
     * This ensures the car can actually achieve the speed profile
     * given its acceleration and braking capabilities.
     * 
     * For racing, you'd also want to consider:
     * - Engine power limits (speed-dependent max accel)
     * - Tire friction ellipse
     * - Downforce effects
     * 
     * But for FSAE at moderate speeds, this simplified model works well.
     */
    
    if (path.points.empty()) {
        return;
    }
    
    // PASS 1: Curvature-limited speed
    for (auto& point : path.points) {
        double abs_curvature = std::abs(point.curvature);
        
        if (abs_curvature > 1e-6) {
            double v_curve = std::sqrt(config_.max_lateral_accel / abs_curvature);
            point.speed = std::min(v_curve, config_.max_speed);
        } else {
            point.speed = config_.max_speed;
        }
        
        // Apply minimum speed
        point.speed = std::max(point.speed, config_.min_speed);
    }
    
    // PASS 2: Forward pass (acceleration limited)
    // v[i+1]² <= v[i]² + 2*a*ds
    for (size_t i = 0; i < path.points.size() - 1; ++i) {
        double ds = path.points[i+1].distance - path.points[i].distance;
        double v_current = path.points[i].speed;
        double v_max_next = std::sqrt(v_current * v_current + 
                                       2.0 * config_.max_longitudinal_accel * ds);
        path.points[i+1].speed = std::min(path.points[i+1].speed, v_max_next);
    }
    
    // PASS 3: Backward pass (deceleration limited)
    // v[i]² <= v[i+1]² + 2*a_brake*ds
    for (size_t i = path.points.size() - 1; i > 0; --i) {
        double ds = path.points[i].distance - path.points[i-1].distance;
        double v_current = path.points[i].speed;
        double v_max_prev = std::sqrt(v_current * v_current + 
                                       2.0 * config_.max_longitudinal_decel * ds);
        path.points[i-1].speed = std::min(path.points[i-1].speed, v_max_prev);
    }
}

// ============================================================================
// FALLBACK PATH
// ============================================================================

PlannedPath DelaunayPlanner::generateFallbackPath() {
    /*
     * FALLBACK PATH:
     * 
     * Generated when normal planning fails (not enough cones, etc.)
     * 
     * This is a simple straight path forward at minimum speed.
     * It's conservative but safe - the car will creep forward
     * until it can see enough cones to plan properly.
     * 
     * In competition, you'd want more sophisticated fallback
     * behavior, such as:
     * - Follow the last known path
     * - Estimate track direction from partial cone data
     * - Emergency stop if situation is too uncertain
     */
    
    PlannedPath path;
    
    double length = 5.0;  // 5 meter straight path
    int num_points = static_cast<int>(length / config_.smoothing_resolution);
    
    for (int i = 0; i < num_points; ++i) {
        PathPoint point;
        point.x = (i + 1) * config_.smoothing_resolution;
        point.y = 0.0;  // Straight ahead
        point.heading = 0.0;
        point.curvature = 0.0;
        point.speed = config_.min_speed;  // Slow when uncertain
        point.distance = point.x;
        path.points.push_back(point);
    }
    
    path.total_length = length;
    path.valid = true;  // It's valid, just conservative
    
    return path;
}

// ============================================================================
// GEOMETRY UTILITIES
// ============================================================================

bool DelaunayPlanner::computeCircumcircle(const Point2D& p0, const Point2D& p1, 
                                           const Point2D& p2,
                                           Point2D& center, double& radius_sq) {
    /*
     * CIRCUMCIRCLE COMPUTATION:
     * 
     * The circumcircle of a triangle is the unique circle that passes
     * through all three vertices. Its center (circumcenter) is
     * equidistant from all vertices.
     * 
     * We use the formula based on solving the system:
     *   |center - p0|² = |center - p1|² = |center - p2|²
     * 
     * This gives us a 2x2 linear system which we solve directly.
     */
    
    double ax = p0.x, ay = p0.y;
    double bx = p1.x, by = p1.y;
    double cx = p2.x, cy = p2.y;
    
    double d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
    
    if (std::abs(d) < 1e-12) {
        // Degenerate triangle (collinear points)
        return false;
    }
    
    double ax2_ay2 = ax * ax + ay * ay;
    double bx2_by2 = bx * bx + by * by;
    double cx2_cy2 = cx * cx + cy * cy;
    
    center.x = (ax2_ay2 * (by - cy) + bx2_by2 * (cy - ay) + cx2_cy2 * (ay - by)) / d;
    center.y = (ax2_ay2 * (cx - bx) + bx2_by2 * (ax - cx) + cx2_cy2 * (bx - ax)) / d;
    
    radius_sq = (center.x - ax) * (center.x - ax) + (center.y - ay) * (center.y - ay);
    
    return true;
}

bool DelaunayPlanner::isInsideCircumcircle(const Point2D& p, const Triangle& tri) {
    /*
     * CIRCUMCIRCLE TEST:
     * 
     * Check if point p is strictly inside the circumcircle of the triangle.
     * This is the key test for the Bowyer-Watson algorithm.
     * 
     * We use a small epsilon for numerical robustness.
     */
    
    double dx = p.x - tri.circumcenter.x;
    double dy = p.y - tri.circumcenter.y;
    double dist_sq = dx * dx + dy * dy;
    
    // Strictly inside (with small epsilon for numerical stability)
    return dist_sq < tri.circumradius_sq - 1e-10;
}

}  // namespace path_planner
}  // namespace fsae

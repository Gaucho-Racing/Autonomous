/**
 * @file pure_pursuit.hpp
 * @brief Pure Pursuit Path-Following Controller for FSAE Racing
 * 
 * This header defines the Pure Pursuit controller, a geometric path-tracking
 * algorithm that computes steering commands to follow a reference path.
 * 
 * ============================================================================
 * PURE PURSUIT ALGORITHM
 * ============================================================================
 * 
 * Pure Pursuit works by:
 * 1. Finding a "lookahead point" on the path ahead of the vehicle
 * 2. Computing the steering angle needed to reach that point via a circular arc
 * 
 * It's called "Pure Pursuit" because the vehicle purely pursues (chases)
 * a moving target point along the path.
 * 
 * ============================================================================
 * MATHEMATICAL DERIVATION
 * ============================================================================
 * 
 * Given:
 *   - Vehicle at origin of its local frame (rear axle)
 *   - Lookahead point at (x_L, y_L) in vehicle frame
 *   - Wheelbase L
 *   - Lookahead distance l_d = sqrt(x_L² + y_L²)
 * 
 * The vehicle follows a circular arc to reach the lookahead point.
 * 
 * From the geometry of circular arcs:
 *   sin(α) = y_L / l_d
 * 
 * where α is the angle from the vehicle's heading to the lookahead point.
 * 
 * For a circular arc, the curvature κ is:
 *   κ = 2 * sin(α) / l_d = 2 * y_L / l_d²
 * 
 * Using the bicycle model, steering angle δ relates to curvature:
 *   κ = tan(δ) / L
 * 
 * Therefore:
 *   δ = arctan(κ * L) = arctan(2 * L * y_L / l_d²)
 * 
 * ============================================================================
 * TUNING GUIDANCE
 * ============================================================================
 * 
 * LOOKAHEAD DISTANCE:
 *   - Too short: Oscillations, aggressive corrections
 *   - Too long: Cuts corners, slow to respond
 *   - Typical: l_d = k * v + l_min, where k ≈ 0.5-1.0 seconds
 * 
 * For FSAE at 5-10 m/s:
 *   - min_lookahead: 1.5-2.0 m
 *   - max_lookahead: 6-10 m
 *   - lookahead_gain: 0.5-0.8 s
 * 
 * WHEELBASE:
 *   - Measure your actual vehicle!
 *   - 1/10 RC cars: 0.25-0.35 m
 *   - Full FSAE cars: 1.5-1.6 m
 * 
 * ============================================================================
 * COORDINATE FRAME
 * ============================================================================
 * 
 * Vehicle frame (base_link):
 *   - Origin: Center of rear axle
 *   - X: Forward (positive ahead)
 *   - Y: Left (positive to driver's left)
 * 
 * Steering convention:
 *   - Positive steering angle = turn left
 *   - Negative steering angle = turn right
 * 
 * @author Your FSAE Team
 * @date 2025
 */

#ifndef PURE_PURSUIT__PURE_PURSUIT_HPP_
#define PURE_PURSUIT__PURE_PURSUIT_HPP_

#include <vector>
#include <cmath>
#include <algorithm>
#include <optional>

namespace fsae {
namespace control {

// ============================================================================
// DATA STRUCTURES
// ============================================================================

/**
 * @brief Vehicle state for control
 */
struct VehicleState {
    double x;               ///< X position in world frame [m]
    double y;               ///< Y position in world frame [m]
    double heading;         ///< Heading angle [rad] (0 = +X, CCW positive)
    double velocity;        ///< Longitudinal velocity [m/s]
    double steering_angle;  ///< Current steering angle [rad]
    double yaw_rate;        ///< Angular velocity [rad/s]
    
    VehicleState() : x(0), y(0), heading(0), velocity(0), steering_angle(0), yaw_rate(0) {}
};

/**
 * @brief Path point for controller input
 * 
 * This matches the output from the path planner.
 */
struct PathPoint {
    double x;               ///< X position [m]
    double y;               ///< Y position [m]
    double heading;         ///< Path heading [rad]
    double curvature;       ///< Path curvature [1/m]
    double speed;           ///< Target speed [m/s]
    
    PathPoint() : x(0), y(0), heading(0), curvature(0), speed(5.0) {}
    PathPoint(double x_, double y_, double s = 5.0) : x(x_), y(y_), heading(0), curvature(0), speed(s) {}
};

/**
 * @brief Control command output
 */
struct ControlCommand {
    double steering_angle;  ///< Steering angle [rad] (positive = left)
    double throttle;        ///< Throttle command [0.0, 1.0]
    double brake;           ///< Brake command [0.0, 1.0]
    double target_speed;    ///< Target speed for low-level controller [m/s]
    bool emergency_stop;    ///< Emergency stop flag
    
    ControlCommand() : steering_angle(0), throttle(0), brake(0), target_speed(0), emergency_stop(false) {}
};

/**
 * @brief Controller configuration
 */
struct PurePursuitConfig {
    // Vehicle parameters (MEASURE YOUR VEHICLE!)
    double wheelbase = 0.30;            ///< Distance between axles [m]
    double max_steering_angle = 0.40;   ///< Physical steering limit [rad] (~23°)
    
    // Lookahead parameters
    double min_lookahead = 1.5;         ///< Minimum lookahead distance [m]
    double max_lookahead = 6.0;         ///< Maximum lookahead distance [m]
    double lookahead_gain = 0.6;        ///< Velocity multiplier for lookahead [s]
    
    // Speed control parameters
    double speed_kp = 2.0;              ///< Speed controller proportional gain
    double speed_ki = 0.5;              ///< Speed controller integral gain
    double speed_kd = 0.1;              ///< Speed controller derivative gain
    double speed_integral_limit = 5.0;  ///< Anti-windup limit for integral term
    
    // Feedforward
    bool use_curvature_feedforward = true;  ///< Use path curvature for feedforward
    double curvature_ff_gain = 0.8;         ///< Feedforward gain (0-1)
    
    // Safety limits
    double max_speed = 8.0;             ///< Maximum allowed speed [m/s]
    double emergency_stop_distance = 0.5; ///< Stop if path ends within this [m]
};

// ============================================================================
// PURE PURSUIT CONTROLLER
// ============================================================================

/**
 * @brief Pure Pursuit path-following controller
 * 
 * Computes steering and speed commands to follow a reference path.
 * 
 * Key features:
 * - Adaptive lookahead distance based on velocity
 * - Optional curvature feedforward for improved tracking
 * - PID speed controller with anti-windup
 * - Emergency stop detection
 * 
 * Thread Safety: NOT thread-safe. Use one instance per thread.
 * 
 * Example:
 * @code
 * PurePursuitConfig config;
 * config.wheelbase = 0.30;  // RC car
 * 
 * PurePursuitController controller(config);
 * 
 * VehicleState state = getCurrentState();
 * std::vector<PathPoint> path = getPath();
 * 
 * ControlCommand cmd = controller.computeControl(state, path);
 * applyControl(cmd);
 * @endcode
 */
class PurePursuitController {
public:
    /**
     * @brief Default constructor
     */
    PurePursuitController();
    
    /**
     * @brief Constructor with configuration
     */
    explicit PurePursuitController(const PurePursuitConfig& config);
    
    /**
     * @brief Update configuration
     */
    void configure(const PurePursuitConfig& config);
    
    /**
     * @brief Get current configuration
     */
    const PurePursuitConfig& getConfig() const { return config_; }
    
    /**
     * @brief Main control function
     * 
     * Computes steering and speed commands to follow the path.
     * 
     * @param state Current vehicle state
     * @param path Reference path to follow
     * @param dt Time step since last call [s] (for integral/derivative)
     * @return Control command
     * 
     * Call this at your control rate (typically 50-100 Hz).
     */
    ControlCommand computeControl(const VehicleState& state,
                                   const std::vector<PathPoint>& path,
                                   double dt = 0.02);
    
    /**
     * @brief Reset controller state (integral terms, etc.)
     * 
     * Call this when:
     * - Starting autonomous mode
     * - After emergency stop
     * - When switching to a new path
     */
    void reset();
    
    /**
     * @brief Get the current lookahead point (for debugging/visualization)
     * @return Lookahead point if one was found
     */
    std::optional<PathPoint> getLookaheadPoint() const { return lookahead_point_; }
    
    /**
     * @brief Get cross-track error (for debugging)
     * @return Signed cross-track error [m] (positive = left of path)
     */
    double getCrossTrackError() const { return cross_track_error_; }
    
    /**
     * @brief Get heading error (for debugging)
     * @return Heading error [rad]
     */
    double getHeadingError() const { return heading_error_; }

private:
    PurePursuitConfig config_;
    
    // Controller state
    double speed_integral_;         ///< Integral term for speed PID
    double last_speed_error_;       ///< Previous speed error for derivative
    
    // Diagnostic state
    std::optional<PathPoint> lookahead_point_;
    double cross_track_error_;
    double heading_error_;
    
    /**
     * @brief Calculate adaptive lookahead distance
     * 
     * l_d = gain * |velocity| + min_lookahead
     * Clamped to [min_lookahead, max_lookahead]
     * 
     * @param velocity Current vehicle velocity [m/s]
     * @return Lookahead distance [m]
     */
    double calculateLookahead(double velocity) const;
    
    /**
     * @brief Find the lookahead point on the path
     * 
     * Searches the path for a point approximately lookahead_distance
     * away from the vehicle. If no such point exists, returns the
     * farthest point on the path.
     * 
     * @param state Vehicle state
     * @param path Reference path
     * @param lookahead_distance Target lookahead distance [m]
     * @return Lookahead point and its target speed
     */
    std::pair<PathPoint, double> findLookaheadPoint(
        const VehicleState& state,
        const std::vector<PathPoint>& path,
        double lookahead_distance);
    
    /**
     * @brief Compute steering angle using Pure Pursuit geometry
     * 
     * @param state Vehicle state
     * @param lookahead Lookahead point (in world frame)
     * @param lookahead_distance Actual distance to lookahead point
     * @return Steering angle [rad]
     */
    double computeSteering(const VehicleState& state,
                           const PathPoint& lookahead,
                           double lookahead_distance);
    
    /**
     * @brief Compute feedforward steering from path curvature
     * 
     * Uses the relationship: δ = arctan(κ * L)
     * 
     * @param curvature Path curvature at lookahead point [1/m]
     * @return Feedforward steering angle [rad]
     */
    double computeCurvatureFeedforward(double curvature) const;
    
    /**
     * @brief Compute throttle and brake using PID speed control
     * 
     * @param current_speed Current vehicle speed [m/s]
     * @param target_speed Desired speed [m/s]
     * @param dt Time step [s]
     * @return Pair of (throttle, brake), each in [0, 1]
     */
    std::pair<double, double> computeSpeedControl(double current_speed,
                                                   double target_speed,
                                                   double dt);
    
    /**
     * @brief Find closest point on path to vehicle
     * 
     * Used for computing cross-track error.
     * 
     * @param state Vehicle state
     * @param path Reference path
     * @return Index of closest point
     */
    size_t findClosestPoint(const VehicleState& state,
                            const std::vector<PathPoint>& path);
    
    /**
     * @brief Normalize angle to [-π, π]
     */
    static double normalizeAngle(double angle);
};

}  // namespace control
}  // namespace fsae

#endif  // PURE_PURSUIT__PURE_PURSUIT_HPP_

/**
 * @file pure_pursuit.cpp
 * @brief Implementation of Pure Pursuit Controller
 */

#include "pure_pursuit/pure_pursuit.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace fsae {
namespace control {

// ============================================================================
// CONSTRUCTOR / CONFIGURATION
// ============================================================================

PurePursuitController::PurePursuitController() 
    : config_()
    , speed_integral_(0.0)
    , last_speed_error_(0.0)
    , cross_track_error_(0.0)
    , heading_error_(0.0) {
}

PurePursuitController::PurePursuitController(const PurePursuitConfig& config)
    : config_(config)
    , speed_integral_(0.0)
    , last_speed_error_(0.0)
    , cross_track_error_(0.0)
    , heading_error_(0.0) {
}

void PurePursuitController::configure(const PurePursuitConfig& config) {
    config_ = config;
    reset();
}

void PurePursuitController::reset() {
    speed_integral_ = 0.0;
    last_speed_error_ = 0.0;
    cross_track_error_ = 0.0;
    heading_error_ = 0.0;
    lookahead_point_.reset();
}

// ============================================================================
// MAIN CONTROL FUNCTION
// ============================================================================

ControlCommand PurePursuitController::computeControl(
    const VehicleState& state,
    const std::vector<PathPoint>& path,
    double dt) {
    
    ControlCommand cmd;
    
    // Input validation
    if (path.empty()) {
        cmd.emergency_stop = true;
        cmd.steering_angle = 0.0;
        cmd.throttle = 0.0;
        cmd.brake = 1.0;
        cmd.target_speed = 0.0;
        return cmd;
    }
    
    // Calculate lookahead distance
    double lookahead_distance = calculateLookahead(state.velocity);
    
    // Find lookahead point
    auto [lookahead, target_speed] = findLookaheadPoint(state, path, lookahead_distance);
    lookahead_point_ = lookahead;
    
    // Check if path is ending soon
    double dx_end = path.back().x - state.x;
    double dy_end = path.back().y - state.y;
    double distance_to_end = std::sqrt(dx_end * dx_end + dy_end * dy_end);
    
    if (distance_to_end < config_.emergency_stop_distance) {
        cmd.emergency_stop = true;
        cmd.steering_angle = 0.0;
        cmd.throttle = 0.0;
        cmd.brake = 1.0;
        cmd.target_speed = 0.0;
        return cmd;
    }
    
    // Pure Pursuit steering
    double dx_la = lookahead.x - state.x;
    double dy_la = lookahead.y - state.y;
    double actual_lookahead = std::sqrt(dx_la * dx_la + dy_la * dy_la);
    
    double steering_pp = computeSteering(state, lookahead, actual_lookahead);
    
    // Curvature feedforward
    double steering_ff = 0.0;
    if (config_.use_curvature_feedforward) {
        steering_ff = computeCurvatureFeedforward(lookahead.curvature);
    }
    
    // Combine feedback and feedforward
    double steering_combined = (1.0 - config_.curvature_ff_gain) * steering_pp + 
                               config_.curvature_ff_gain * steering_ff;
    
    cmd.steering_angle = std::clamp(steering_combined, 
                                     -config_.max_steering_angle, 
                                     config_.max_steering_angle);
    
    // Speed control
    target_speed = std::min(target_speed, config_.max_speed);
    cmd.target_speed = target_speed;
    
    auto [throttle, brake] = computeSpeedControl(std::abs(state.velocity), target_speed, dt);
    cmd.throttle = throttle;
    cmd.brake = brake;
    
    // Compute diagnostics
    size_t closest_idx = findClosestPoint(state, path);
    const PathPoint& closest = path[closest_idx];
    
    double dx_cte = state.x - closest.x;
    double dy_cte = state.y - closest.y;
    double path_heading = closest.heading;
    cross_track_error_ = -dx_cte * std::sin(path_heading) + dy_cte * std::cos(path_heading);
    heading_error_ = normalizeAngle(state.heading - closest.heading);
    
    return cmd;
}

// ============================================================================
// LOOKAHEAD CALCULATION
// ============================================================================

double PurePursuitController::calculateLookahead(double velocity) const {
    double lookahead = config_.lookahead_gain * std::abs(velocity) + config_.min_lookahead;
    return std::clamp(lookahead, config_.min_lookahead, config_.max_lookahead);
}

std::pair<PathPoint, double> PurePursuitController::findLookaheadPoint(
    const VehicleState& state,
    const std::vector<PathPoint>& path,
    double lookahead_distance) {
    
    PathPoint best_point = path.back();
    double best_speed = path.back().speed;
    double best_distance_error = std::numeric_limits<double>::max();
    
    for (const auto& point : path) {
        double dx = point.x - state.x;
        double dy = point.y - state.y;
        
        double cos_h = std::cos(state.heading);
        double sin_h = std::sin(state.heading);
        double x_local = dx * cos_h + dy * sin_h;
        
        if (x_local < 0.1) {
            continue;
        }
        
        double distance = std::sqrt(dx * dx + dy * dy);
        double distance_error = std::abs(distance - lookahead_distance);
        
        if (distance_error < best_distance_error) {
            best_distance_error = distance_error;
            best_point = point;
            best_speed = point.speed;
        }
    }
    
    return {best_point, best_speed};
}

// ============================================================================
// STEERING COMPUTATION
// ============================================================================

double PurePursuitController::computeSteering(
    const VehicleState& state,
    const PathPoint& lookahead,
    double lookahead_distance) {
    
    double dx = lookahead.x - state.x;
    double dy = lookahead.y - state.y;
    
    double cos_h = std::cos(state.heading);
    double sin_h = std::sin(state.heading);
    
    double x_local = dx * cos_h + dy * sin_h;
    double y_local = -dx * sin_h + dy * cos_h;
    
    double lookahead_sq = x_local * x_local + y_local * y_local;
    
    if (lookahead_sq < 0.01) {
        return 0.0;
    }
    
    double curvature = 2.0 * y_local / lookahead_sq;
    double steering = std::atan(curvature * config_.wheelbase);
    
    return steering;
}

double PurePursuitController::computeCurvatureFeedforward(double curvature) const {
    return std::atan(curvature * config_.wheelbase);
}

// ============================================================================
// SPEED CONTROL
// ============================================================================

std::pair<double, double> PurePursuitController::computeSpeedControl(
    double current_speed,
    double target_speed,
    double dt) {
    
    double error = target_speed - current_speed;
    
    // Proportional term
    double p_term = config_.speed_kp * error;
    
    // Integral term with anti-windup
    speed_integral_ += error * dt;
    speed_integral_ = std::clamp(speed_integral_, 
                                  -config_.speed_integral_limit, 
                                  config_.speed_integral_limit);
    double i_term = config_.speed_ki * speed_integral_;
    
    // Derivative term
    double d_term = 0.0;
    if (dt > 1e-6) {
        double error_rate = (error - last_speed_error_) / dt;
        d_term = config_.speed_kd * error_rate;
    }
    last_speed_error_ = error;
    
    double output = p_term + i_term + d_term;
    
    double throttle = 0.0;
    double brake = 0.0;
    
    if (output > 0) {
        throttle = std::clamp(output, 0.0, 1.0);
    } else {
        brake = std::clamp(-output, 0.0, 1.0);
    }
    
    return {throttle, brake};
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

size_t PurePursuitController::findClosestPoint(
    const VehicleState& state,
    const std::vector<PathPoint>& path) {
    
    size_t closest_idx = 0;
    double min_dist_sq = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < path.size(); ++i) {
        double dx = path[i].x - state.x;
        double dy = path[i].y - state.y;
        double dist_sq = dx * dx + dy * dy;
        
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            closest_idx = i;
        }
    }
    
    return closest_idx;
}

double PurePursuitController::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

}  // namespace control
}  // namespace fsae

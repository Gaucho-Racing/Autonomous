/**
 * @file controller_node.cpp
 * @brief ROS 2 Node for Pure Pursuit Controller
 * 
 * This node implements path following using the Pure Pursuit algorithm.
 * It subscribes to planned paths and vehicle state, and publishes
 * control commands.
 * 
 * ============================================================================
 * SUBSCRIPTIONS
 * ============================================================================
 *   /planned_path (nav_msgs/msg/Path)
 *       Reference path to follow
 * 
 *   /path_speeds (std_msgs/msg/Float64MultiArray)
 *       Speed profile for the path
 * 
 *   /odom (nav_msgs/msg/Odometry)
 *       Vehicle state (position, velocity)
 * 
 * ============================================================================
 * PUBLICATIONS
 * ============================================================================
 *   /drive (ackermann_msgs/msg/AckermannDriveStamped)
 *       Ackermann steering commands for car-like vehicles
 * 
 *   /cmd_vel (geometry_msgs/msg/Twist)
 *       Alternative velocity commands
 * 
 *   /controller_markers (visualization_msgs/msg/MarkerArray)
 *       Visualization of lookahead point, errors
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Ackermann messages for car-like vehicles
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include "pure_pursuit/pure_pursuit.hpp"

#include <chrono>
#include <memory>
#include <mutex>

using namespace std::chrono_literals;

namespace fsae {
namespace ros2 {

class PurePursuitNode : public rclcpp::Node {
public:
    PurePursuitNode() : Node("pure_pursuit_controller") {
        // ====================================================================
        // DECLARE PARAMETERS
        // ====================================================================
        
        // Vehicle parameters - MEASURE YOUR VEHICLE!
        this->declare_parameter("wheelbase", 0.30);  // RC car default
        this->declare_parameter("max_steering_angle", 0.40);  // ~23 degrees
        
        // Lookahead parameters
        this->declare_parameter("min_lookahead", 1.5);
        this->declare_parameter("max_lookahead", 6.0);
        this->declare_parameter("lookahead_gain", 0.6);
        
        // Speed controller
        this->declare_parameter("speed_kp", 2.0);
        this->declare_parameter("speed_ki", 0.5);
        this->declare_parameter("speed_kd", 0.1);
        
        // Feedforward
        this->declare_parameter("use_curvature_feedforward", true);
        this->declare_parameter("curvature_ff_gain", 0.8);
        
        // Safety
        this->declare_parameter("max_speed", 8.0);
        this->declare_parameter("emergency_stop_distance", 0.5);
        
        // Control rate
        this->declare_parameter("control_rate_hz", 50.0);
        this->declare_parameter("publish_markers", true);
        
        // ====================================================================
        // CONFIGURE CONTROLLER
        // ====================================================================
        
        control::PurePursuitConfig config;
        config.wheelbase = this->get_parameter("wheelbase").as_double();
        config.max_steering_angle = this->get_parameter("max_steering_angle").as_double();
        config.min_lookahead = this->get_parameter("min_lookahead").as_double();
        config.max_lookahead = this->get_parameter("max_lookahead").as_double();
        config.lookahead_gain = this->get_parameter("lookahead_gain").as_double();
        config.speed_kp = this->get_parameter("speed_kp").as_double();
        config.speed_ki = this->get_parameter("speed_ki").as_double();
        config.speed_kd = this->get_parameter("speed_kd").as_double();
        config.use_curvature_feedforward = this->get_parameter("use_curvature_feedforward").as_bool();
        config.curvature_ff_gain = this->get_parameter("curvature_ff_gain").as_double();
        config.max_speed = this->get_parameter("max_speed").as_double();
        config.emergency_stop_distance = this->get_parameter("emergency_stop_distance").as_double();
        
        controller_ = std::make_unique<control::PurePursuitController>(config);
        publish_markers_ = this->get_parameter("publish_markers").as_bool();
        
        RCLCPP_INFO(this->get_logger(), "Pure Pursuit Controller Configuration:");
        RCLCPP_INFO(this->get_logger(), "  Wheelbase: %.3f m", config.wheelbase);
        RCLCPP_INFO(this->get_logger(), "  Max steering: %.2f rad (%.1f deg)", 
                    config.max_steering_angle, config.max_steering_angle * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "  Lookahead: %.1f - %.1f m", 
                    config.min_lookahead, config.max_lookahead);
        
        // ====================================================================
        // CREATE SUBSCRIBERS
        // ====================================================================
        
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planned_path", 10,
            std::bind(&PurePursuitNode::pathCallback, this, std::placeholders::_1));
        
        speed_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/path_speeds", 10,
            std::bind(&PurePursuitNode::speedCallback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&PurePursuitNode::odomCallback, this, std::placeholders::_1));
        
        // ====================================================================
        // CREATE PUBLISHERS
        // ====================================================================
        
        ackermann_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10);
        
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/controller_markers", 10);
        
        // ====================================================================
        // CREATE CONTROL TIMER
        // ====================================================================
        
        double rate = this->get_parameter("control_rate_hz").as_double();
        control_dt_ = 1.0 / rate;
        auto period = std::chrono::duration<double>(control_dt_);
        
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&PurePursuitNode::controlCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "Pure Pursuit Controller Node initialized at %.1f Hz", rate);
    }

private:
    // ========================================================================
    // CALLBACKS
    // ========================================================================
    
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        
        current_path_.clear();
        for (const auto& pose : msg->poses) {
            control::PathPoint point;
            point.x = pose.pose.position.x;
            point.y = pose.pose.position.y;
            
            // Extract yaw from quaternion
            double siny = 2.0 * (pose.pose.orientation.w * pose.pose.orientation.z);
            double cosy = 1.0 - 2.0 * (pose.pose.orientation.z * pose.pose.orientation.z);
            point.heading = std::atan2(siny, cosy);
            
            // Default speed (will be overwritten by speed callback)
            point.speed = 5.0;
            point.curvature = 0.0;
            
            current_path_.push_back(point);
        }
        
        path_received_ = true;
    }
    
    void speedCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        
        // Update path speeds
        for (size_t i = 0; i < msg->data.size() && i < current_path_.size(); ++i) {
            current_path_[i].speed = msg->data[i];
        }
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        current_state_.x = msg->pose.pose.position.x;
        current_state_.y = msg->pose.pose.position.y;
        
        // Extract yaw from quaternion
        double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z);
        double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
        current_state_.heading = std::atan2(siny, cosy);
        
        current_state_.velocity = msg->twist.twist.linear.x;
        current_state_.yaw_rate = msg->twist.twist.angular.z;
        
        state_received_ = true;
    }
    
    void controlCallback() {
        // Get local copies of data
        std::vector<control::PathPoint> path;
        control::VehicleState state;
        
        {
            std::lock_guard<std::mutex> lock(path_mutex_);
            path = current_path_;
        }
        
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            state = current_state_;
        }
        
        // Check if we have data
        if (path.empty()) {
            // No path - publish zero command
            publishEmergencyStop();
            return;
        }
        
        // For testing without odometry: assume vehicle at origin
        if (!state_received_) {
            state.x = 0.0;
            state.y = 0.0;
            state.heading = 0.0;
            state.velocity = 3.0;  // Assume some velocity for testing
        }
        
        // Compute control
        control::ControlCommand cmd = controller_->computeControl(state, path, control_dt_);
        
        // Publish commands
        publishAckermann(cmd);
        publishTwist(cmd, state);
        
        // Publish visualization
        if (publish_markers_) {
            publishMarkers(state, cmd);
        }
        
        // Log diagnostics periodically
        control_count_++;
        if (control_count_ % 250 == 0) {  // Every 5 seconds at 50 Hz
            RCLCPP_INFO(this->get_logger(), 
                "Control: steer=%.2f deg, speed=%.2f m/s, CTE=%.3f m",
                cmd.steering_angle * 180.0 / M_PI,
                cmd.target_speed,
                controller_->getCrossTrackError());
        }
    }
    
    // ========================================================================
    // PUBLISHING
    // ========================================================================
    
    void publishAckermann(const control::ControlCommand& cmd) {
        ackermann_msgs::msg::AckermannDriveStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "base_link";
        
        msg.drive.steering_angle = cmd.steering_angle;
        msg.drive.steering_angle_velocity = 0.0;  // Not controlled
        msg.drive.speed = cmd.target_speed;
        msg.drive.acceleration = 0.0;  // Not controlled directly
        msg.drive.jerk = 0.0;
        
        ackermann_pub_->publish(msg);
    }
    
    void publishTwist(const control::ControlCommand& cmd, const control::VehicleState& state) {
        geometry_msgs::msg::Twist msg;
        
        // Convert Ackermann to Twist
        // For car-like robots: angular.z = v * tan(steering) / wheelbase
        double wheelbase = controller_->getConfig().wheelbase;
        
        msg.linear.x = cmd.target_speed;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;
        
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = cmd.target_speed * std::tan(cmd.steering_angle) / wheelbase;
        
        twist_pub_->publish(msg);
    }
    
    void publishEmergencyStop() {
        ackermann_msgs::msg::AckermannDriveStamped ack_msg;
        ack_msg.header.stamp = this->now();
        ack_msg.drive.speed = 0.0;
        ack_msg.drive.steering_angle = 0.0;
        ackermann_pub_->publish(ack_msg);
        
        geometry_msgs::msg::Twist twist_msg;
        twist_pub_->publish(twist_msg);
    }
    
    void publishMarkers(const control::VehicleState& state, 
                        const control::ControlCommand& cmd) {
        visualization_msgs::msg::MarkerArray markers;
        auto stamp = this->now();
        int id = 0;
        
        // Lookahead point marker
        auto lookahead = controller_->getLookaheadPoint();
        if (lookahead.has_value()) {
            visualization_msgs::msg::Marker la_marker;
            la_marker.header.stamp = stamp;
            la_marker.header.frame_id = "base_link";
            la_marker.ns = "lookahead";
            la_marker.id = id++;
            la_marker.type = visualization_msgs::msg::Marker::SPHERE;
            la_marker.action = visualization_msgs::msg::Marker::ADD;
            
            la_marker.pose.position.x = lookahead->x;
            la_marker.pose.position.y = lookahead->y;
            la_marker.pose.position.z = 0.3;
            la_marker.pose.orientation.w = 1.0;
            
            la_marker.scale.x = 0.3;
            la_marker.scale.y = 0.3;
            la_marker.scale.z = 0.3;
            
            la_marker.color.r = 1.0;
            la_marker.color.g = 0.0;
            la_marker.color.b = 1.0;
            la_marker.color.a = 0.8;
            
            markers.markers.push_back(la_marker);
            
            // Line from vehicle to lookahead
            visualization_msgs::msg::Marker line_marker;
            line_marker.header.stamp = stamp;
            line_marker.header.frame_id = "base_link";
            line_marker.ns = "lookahead_line";
            line_marker.id = id++;
            line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::msg::Marker::ADD;
            
            line_marker.pose.orientation.w = 1.0;
            line_marker.scale.x = 0.05;
            
            line_marker.color.r = 1.0;
            line_marker.color.g = 0.0;
            line_marker.color.b = 1.0;
            line_marker.color.a = 0.5;
            
            geometry_msgs::msg::Point p1, p2;
            p1.x = 0.0; p1.y = 0.0; p1.z = 0.1;
            p2.x = lookahead->x; p2.y = lookahead->y; p2.z = 0.1;
            line_marker.points.push_back(p1);
            line_marker.points.push_back(p2);
            
            markers.markers.push_back(line_marker);
        }
        
        // Steering visualization (arc showing intended path)
        visualization_msgs::msg::Marker steer_marker;
        steer_marker.header.stamp = stamp;
        steer_marker.header.frame_id = "base_link";
        steer_marker.ns = "steering_arc";
        steer_marker.id = id++;
        steer_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        steer_marker.action = visualization_msgs::msg::Marker::ADD;
        
        steer_marker.pose.orientation.w = 1.0;
        steer_marker.scale.x = 0.05;
        
        steer_marker.color.r = 0.0;
        steer_marker.color.g = 1.0;
        steer_marker.color.b = 1.0;
        steer_marker.color.a = 0.8;
        
        // Draw predicted arc based on steering angle
        double wheelbase = controller_->getConfig().wheelbase;
        if (std::abs(cmd.steering_angle) > 0.01) {
            double radius = wheelbase / std::tan(cmd.steering_angle);
            
            for (int i = 0; i <= 20; ++i) {
                double arc_length = i * 0.2;  // 4m total
                double theta = arc_length / radius;
                
                geometry_msgs::msg::Point p;
                p.x = radius * std::sin(theta);
                p.y = radius * (1.0 - std::cos(theta));
                p.z = 0.05;
                steer_marker.points.push_back(p);
            }
        } else {
            // Straight line
            for (int i = 0; i <= 20; ++i) {
                geometry_msgs::msg::Point p;
                p.x = i * 0.2;
                p.y = 0.0;
                p.z = 0.05;
                steer_marker.points.push_back(p);
            }
        }
        
        markers.markers.push_back(steer_marker);
        
        marker_pub_->publish(markers);
    }
    
    // Controller instance
    std::unique_ptr<control::PurePursuitController> controller_;
    
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr speed_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // Publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    double control_dt_;
    
    // Data storage
    std::vector<control::PathPoint> current_path_;
    control::VehicleState current_state_;
    std::mutex path_mutex_;
    std::mutex state_mutex_;
    bool path_received_ = false;
    bool state_received_ = false;
    
    // Configuration
    bool publish_markers_;
    
    // Statistics
    size_t control_count_ = 0;
};

}  // namespace ros2
}  // namespace fsae

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<fsae::ros2::PurePursuitNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

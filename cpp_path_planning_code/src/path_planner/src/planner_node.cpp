/**
 * @file planner_node.cpp
 * @brief ROS 2 Node for Path Planning
 * 
 * This node wraps the DelaunayPlanner algorithm in a ROS 2 interface.
 * It subscribes to cone detections and publishes planned paths.
 * 
 * ============================================================================
 * SUBSCRIPTIONS
 * ============================================================================
 *   /detected_cones (fsae_msgs/msg/ConeArray)
 *       Array of detected cone positions and colors
 * 
 * ============================================================================
 * PUBLICATIONS
 * ============================================================================
 *   /planned_path (nav_msgs/msg/Path)
 *       Planned path as sequence of poses
 * 
 *   /path_markers (visualization_msgs/msg/MarkerArray)
 *       Visualization for RViz (cones, path, midpoints)
 * 
 *   /path_speeds (std_msgs/msg/Float64MultiArray)
 *       Speed profile for controller
 * 
 * ============================================================================
 * PARAMETERS
 * ============================================================================
 *   ~max_speed (double, default: 8.0)
 *   ~min_speed (double, default: 2.0)
 *   ~lookahead_distance (double, default: 15.0)
 *   ~max_edge_length (double, default: 8.0)
 *   ~smoothing_resolution (double, default: 0.2)
 *   ~max_lateral_accel (double, default: 6.0)
 *   ~publish_markers (bool, default: true)
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "path_planner/delaunay_planner.hpp"

// For now, we use a simplified cone message approach
// In production, you'd use custom fsae_msgs
#include <geometry_msgs/msg/point_stamped.hpp>

#include <chrono>
#include <memory>
#include <vector>

using namespace std::chrono_literals;

namespace fsae {
namespace ros2 {

class PathPlannerNode : public rclcpp::Node {
public:
    PathPlannerNode() : Node("path_planner") {
        // ====================================================================
        // DECLARE PARAMETERS
        // ====================================================================
        
        this->declare_parameter("max_speed", 8.0);
        this->declare_parameter("min_speed", 2.0);
        this->declare_parameter("lookahead_distance", 15.0);
        this->declare_parameter("max_edge_length", 8.0);
        this->declare_parameter("smoothing_resolution", 0.2);
        this->declare_parameter("max_lateral_accel", 6.0);
        this->declare_parameter("max_longitudinal_accel", 4.0);
        this->declare_parameter("max_longitudinal_decel", 6.0);
        this->declare_parameter("expected_track_width", 3.0);
        this->declare_parameter("publish_markers", true);
        this->declare_parameter("planning_rate_hz", 20.0);
        
        // ====================================================================
        // CONFIGURE PLANNER
        // ====================================================================
        
        path_planner::PlannerConfig config;
        config.max_speed = this->get_parameter("max_speed").as_double();
        config.min_speed = this->get_parameter("min_speed").as_double();
        config.lookahead_distance = this->get_parameter("lookahead_distance").as_double();
        config.max_edge_length = this->get_parameter("max_edge_length").as_double();
        config.smoothing_resolution = this->get_parameter("smoothing_resolution").as_double();
        config.max_lateral_accel = this->get_parameter("max_lateral_accel").as_double();
        config.max_longitudinal_accel = this->get_parameter("max_longitudinal_accel").as_double();
        config.max_longitudinal_decel = this->get_parameter("max_longitudinal_decel").as_double();
        config.expected_track_width = this->get_parameter("expected_track_width").as_double();
        
        planner_ = std::make_unique<path_planner::DelaunayPlanner>(config);
        publish_markers_ = this->get_parameter("publish_markers").as_bool();
        
        RCLCPP_INFO(this->get_logger(), "Path Planner Configuration:");
        RCLCPP_INFO(this->get_logger(), "  Max speed: %.2f m/s", config.max_speed);
        RCLCPP_INFO(this->get_logger(), "  Lookahead: %.2f m", config.lookahead_distance);
        RCLCPP_INFO(this->get_logger(), "  Max lateral accel: %.2f m/s^2", config.max_lateral_accel);
        
        // ====================================================================
        // CREATE PUBLISHERS
        // ====================================================================
        
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/planned_path", 10);
        
        speed_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/path_speeds", 10);
        
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/path_markers", 10);
        
        // ====================================================================
        // CREATE TIMER FOR PLANNING LOOP
        // ====================================================================
        
        double rate = this->get_parameter("planning_rate_hz").as_double();
        auto period = std::chrono::duration<double>(1.0 / rate);
        
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&PathPlannerNode::planningCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "Path Planner Node initialized at %.1f Hz", rate);
    }
    
    /**
     * @brief Set cone detections (called from external source or subscription)
     */
    void setCones(const std::vector<path_planner::Cone>& cones) {
        std::lock_guard<std::mutex> lock(cones_mutex_);
        latest_cones_ = cones;
        cones_updated_ = true;
    }

private:
    void planningCallback() {
        std::vector<path_planner::Cone> cones;
        
        {
            std::lock_guard<std::mutex> lock(cones_mutex_);
            if (latest_cones_.empty()) {
                // Generate test cones for development
                generateTestCones();
            }
            cones = latest_cones_;
        }
        
        // Run planner
        auto start = std::chrono::high_resolution_clock::now();
        
        path_planner::PlannedPath planned_path = planner_->plan(cones);
        
        auto end = std::chrono::high_resolution_clock::now();
        double planning_time_ms = std::chrono::duration<double, std::milli>(end - start).count();
        
        // Update statistics
        planning_count_++;
        total_planning_time_ms_ += planning_time_ms;
        
        if (planning_count_ % 100 == 0) {
            double avg_time = total_planning_time_ms_ / planning_count_;
            RCLCPP_INFO(this->get_logger(), 
                "Planning stats: %lu runs, avg time: %.2f ms", 
                planning_count_, avg_time);
        }
        
        // Publish path
        publishPath(planned_path);
        
        // Publish visualization
        if (publish_markers_) {
            publishMarkers(planned_path, cones);
        }
    }
    
    void publishPath(const path_planner::PlannedPath& planned_path) {
        nav_msgs::msg::Path msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "base_link";
        
        for (const auto& point : planned_path.points) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = msg.header;
            
            pose.pose.position.x = point.x;
            pose.pose.position.y = point.y;
            pose.pose.position.z = 0.0;
            
            // Convert heading to quaternion (yaw only)
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = std::sin(point.heading / 2.0);
            pose.pose.orientation.w = std::cos(point.heading / 2.0);
            
            msg.poses.push_back(pose);
        }
        
        path_pub_->publish(msg);
        
        // Publish speed profile separately
        std_msgs::msg::Float64MultiArray speed_msg;
        for (const auto& point : planned_path.points) {
            speed_msg.data.push_back(point.speed);
        }
        speed_pub_->publish(speed_msg);
    }
    
    void publishMarkers(const path_planner::PlannedPath& planned_path,
                        const std::vector<path_planner::Cone>& cones) {
        visualization_msgs::msg::MarkerArray markers;
        int id = 0;
        
        auto stamp = this->now();
        
        // Cone markers
        for (const auto& cone : cones) {
            visualization_msgs::msg::Marker marker;
            marker.header.stamp = stamp;
            marker.header.frame_id = "base_link";
            marker.ns = "cones";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.position.x = cone.x;
            marker.pose.position.y = cone.y;
            marker.pose.position.z = 0.15;
            marker.pose.orientation.w = 1.0;
            
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.3;
            
            marker.color.a = 1.0;
            if (cone.color == path_planner::ConeColor::BLUE) {
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
            } else if (cone.color == path_planner::ConeColor::YELLOW) {
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
            } else {
                marker.color.r = 1.0;
                marker.color.g = 0.5;
                marker.color.b = 0.0;
            }
            
            markers.markers.push_back(marker);
        }
        
        // Path line marker
        visualization_msgs::msg::Marker path_marker;
        path_marker.header.stamp = stamp;
        path_marker.header.frame_id = "base_link";
        path_marker.ns = "path";
        path_marker.id = id++;
        path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::msg::Marker::ADD;
        
        path_marker.pose.orientation.w = 1.0;
        path_marker.scale.x = 0.1;
        
        path_marker.color.r = 0.0;
        path_marker.color.g = 1.0;
        path_marker.color.b = 0.0;
        path_marker.color.a = 1.0;
        
        for (const auto& point : planned_path.points) {
            geometry_msgs::msg::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0.05;
            path_marker.points.push_back(p);
        }
        
        markers.markers.push_back(path_marker);
        
        // Speed visualization (spheres colored by speed)
        for (size_t i = 0; i < planned_path.points.size(); i += 5) {
            const auto& point = planned_path.points[i];
            
            visualization_msgs::msg::Marker speed_marker;
            speed_marker.header.stamp = stamp;
            speed_marker.header.frame_id = "base_link";
            speed_marker.ns = "speed";
            speed_marker.id = id++;
            speed_marker.type = visualization_msgs::msg::Marker::SPHERE;
            speed_marker.action = visualization_msgs::msg::Marker::ADD;
            
            speed_marker.pose.position.x = point.x;
            speed_marker.pose.position.y = point.y;
            speed_marker.pose.position.z = 0.1;
            speed_marker.pose.orientation.w = 1.0;
            
            speed_marker.scale.x = 0.15;
            speed_marker.scale.y = 0.15;
            speed_marker.scale.z = 0.15;
            
            // Color by speed (green = fast, red = slow)
            double speed_ratio = (point.speed - planner_->getConfig().min_speed) /
                                 (planner_->getConfig().max_speed - planner_->getConfig().min_speed);
            speed_ratio = std::clamp(speed_ratio, 0.0, 1.0);
            
            speed_marker.color.r = 1.0 - speed_ratio;
            speed_marker.color.g = speed_ratio;
            speed_marker.color.b = 0.0;
            speed_marker.color.a = 0.8;
            
            markers.markers.push_back(speed_marker);
        }
        
        marker_pub_->publish(markers);
    }
    
    void generateTestCones() {
        // Generate simple curved track for testing
        latest_cones_.clear();
        
        // Left boundary (blue)
        latest_cones_.push_back({3.0, 2.0, path_planner::ConeColor::BLUE, 0.95f});
        latest_cones_.push_back({6.0, 2.5, path_planner::ConeColor::BLUE, 0.92f});
        latest_cones_.push_back({9.0, 3.5, path_planner::ConeColor::BLUE, 0.90f});
        latest_cones_.push_back({12.0, 3.0, path_planner::ConeColor::BLUE, 0.88f});
        
        // Right boundary (yellow)
        latest_cones_.push_back({3.0, -2.0, path_planner::ConeColor::YELLOW, 0.94f});
        latest_cones_.push_back({6.0, -1.5, path_planner::ConeColor::YELLOW, 0.91f});
        latest_cones_.push_back({9.0, -0.5, path_planner::ConeColor::YELLOW, 0.89f});
        latest_cones_.push_back({12.0, -1.0, path_planner::ConeColor::YELLOW, 0.87f});
    }
    
    // Planner instance
    std::unique_ptr<path_planner::DelaunayPlanner> planner_;
    
    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr speed_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Cone data
    std::vector<path_planner::Cone> latest_cones_;
    std::mutex cones_mutex_;
    bool cones_updated_ = false;
    
    // Configuration
    bool publish_markers_;
    
    // Statistics
    size_t planning_count_ = 0;
    double total_planning_time_ms_ = 0.0;
};

}  // namespace ros2
}  // namespace fsae

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<fsae::ros2::PathPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

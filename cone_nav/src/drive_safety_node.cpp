#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class DriveSafetyNode : public rclcpp::Node
{
public:
  DriveSafetyNode()
  : Node("drive_safety_node")
  {
    declare_parameter("drive_enabled", true);
    declare_parameter("safety_enabled", true);
    declare_parameter("base_frame", "base_link");
    declare_parameter("wheelbase", 0.32);
    declare_parameter("safety_grid_timeout_sec", 0.20);
    declare_parameter("safety_command_timeout_sec", 0.20);
    declare_parameter("safety_braking_deceleration", 2.0);
    declare_parameter("safety_system_latency_sec", 0.10);
    declare_parameter("safety_stop_margin", 0.30);
    declare_parameter("safety_corridor_half_width", 0.08);
    declare_parameter("safety_occupied_threshold", 50);

    drive_enabled_ = get_parameter("drive_enabled").as_bool();
    safety_enabled_ = get_parameter("safety_enabled").as_bool();
    base_frame_ = get_parameter("base_frame").as_string();
    wheelbase_ = get_parameter("wheelbase").as_double();
    grid_timeout_ = get_parameter("safety_grid_timeout_sec").as_double();
    command_timeout_ = get_parameter("safety_command_timeout_sec").as_double();
    deceleration_ = std::max(0.1, get_parameter("safety_braking_deceleration").as_double());
    latency_ = std::max(0.0, get_parameter("safety_system_latency_sec").as_double());
    stop_margin_ = std::max(0.0, get_parameter("safety_stop_margin").as_double());
    corridor_half_width_ = std::max(
      0.0, get_parameter("safety_corridor_half_width").as_double());
    occupied_threshold_ = get_parameter("safety_occupied_threshold").as_int();

    grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/obstacles/local_grid", 1,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        latest_grid_ = std::move(msg);
        last_grid_received_ = now();
      });
    candidate_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      "/drive_candidate", 10,
      std::bind(&DriveSafetyNode::candidateCallback, this, std::placeholders::_1));
    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
    watchdog_ = create_wall_timer(50ms, std::bind(&DriveSafetyNode::watchdog, this));

    RCLCPP_INFO(
      get_logger(), "Drive safety %s; final command publisher is /drive",
      safety_enabled_ ? "enabled" : "bypassed");
  }

private:
  void candidateCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
  {
    last_command_received_ = now();
    if (!drive_enabled_) {
      publishStop();
      return;
    }
    if (!safety_enabled_) {
      publishCommand(*msg, std::max(0.0, msg->drive.speed));
      return;
    }
    if (!gridFresh()) {
      publishStop();
      return;
    }

    const double speed = std::max(0.0, msg->drive.speed);
    const double clearance = pathClearance(msg->drive.steering_angle);
    if (!std::isfinite(clearance)) {
      publishCommand(*msg, speed);
      return;
    }

    const double usable_distance = clearance - stop_margin_;
    const double stopping_distance = speed * latency_ + speed * speed / (2.0 * deceleration_);
    if (usable_distance <= 0.0 || stopping_distance >= usable_distance) {
      publishStop();
      return;
    }

    const double max_safe_speed = -deceleration_ * latency_ + std::sqrt(
      deceleration_ * deceleration_ * latency_ * latency_ +
      2.0 * deceleration_ * usable_distance);
    publishCommand(*msg, std::clamp(speed, 0.0, max_safe_speed));
  }

  bool gridFresh() const
  {
    return latest_grid_ && latest_grid_->header.frame_id == base_frame_ &&
      last_grid_received_.nanoseconds() != 0 &&
      (now() - last_grid_received_).seconds() <= grid_timeout_;
  }

  double pathClearance(double steering_angle) const
  {
    double closest = std::numeric_limits<double>::infinity();
    const double curvature = std::tan(steering_angle) / std::max(wheelbase_, 1e-3);
    const auto & grid = *latest_grid_;
    const int width = static_cast<int>(grid.info.width);
    const int height = static_cast<int>(grid.info.height);
    for (int gy = 0; gy < height; ++gy) {
      for (int gx = 0; gx < width; ++gx) {
        const int index = gy * width + gx;
        if (grid.data[static_cast<size_t>(index)] < occupied_threshold_) {
          continue;
        }
        const double x = grid.info.origin.position.x + (gx + 0.5) * grid.info.resolution;
        const double y = grid.info.origin.position.y + (gy + 0.5) * grid.info.resolution;
        if (x <= 0.0) {
          continue;
        }
        const double path_y = 0.5 * curvature * x * x;
        if (std::abs(y - path_y) <= corridor_half_width_) {
          closest = std::min(closest, std::hypot(x, y));
        }
      }
    }
    return closest;
  }

  void watchdog()
  {
    if (!drive_enabled_ || last_command_received_.nanoseconds() == 0 ||
      (now() - last_command_received_).seconds() > command_timeout_ ||
      (safety_enabled_ && !gridFresh()))
    {
      publishStop();
    }
  }

  void publishCommand(
    const ackermann_msgs::msg::AckermannDriveStamped & candidate, double speed)
  {
    auto output = candidate;
    output.header.stamp = now();
    output.header.frame_id = base_frame_;
    output.drive.speed = speed;
    drive_pub_->publish(output);
  }

  void publishStop()
  {
    ackermann_msgs::msg::AckermannDriveStamped stop;
    stop.header.stamp = now();
    stop.header.frame_id = base_frame_;
    stop.drive.speed = 0.0;
    stop.drive.steering_angle = 0.0;
    drive_pub_->publish(stop);
  }

  bool drive_enabled_{true};
  bool safety_enabled_{true};
  std::string base_frame_{"base_link"};
  double wheelbase_{0.32};
  double grid_timeout_{0.2};
  double command_timeout_{0.2};
  double deceleration_{2.0};
  double latency_{0.1};
  double stop_margin_{0.3};
  double corridor_half_width_{0.08};
  int64_t occupied_threshold_{50};
  rclcpp::Time last_grid_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_command_received_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_grid_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr candidate_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriveSafetyNode>());
  rclcpp::shutdown();
  return 0;
}

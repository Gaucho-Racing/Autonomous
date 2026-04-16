#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class PurePursuitNode : public rclcpp::Node
{
public:
  PurePursuitNode()
  : Node("pure_pursuit_node")
  {
    declare_parameter("lookahead_distance", 1.5);
    declare_parameter("lookahead_min", 0.8);
    declare_parameter("lookahead_max", 3.0);
    declare_parameter("speed_target", 1.5);
    declare_parameter("speed_max", 3.0);
    declare_parameter("wheelbase", 0.32);
    declare_parameter("steering_gain", 1.0);
    declare_parameter("max_steering_angle", 0.4);

    lookahead_distance_ = get_parameter("lookahead_distance").as_double();
    lookahead_min_ = get_parameter("lookahead_min").as_double();
    lookahead_max_ = get_parameter("lookahead_max").as_double();
    speed_target_ = get_parameter("speed_target").as_double();
    speed_max_ = get_parameter("speed_max").as_double();
    wheelbase_ = get_parameter("wheelbase").as_double();
    steering_gain_ = get_parameter("steering_gain").as_double();
    max_steering_angle_ = get_parameter("max_steering_angle").as_double();

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/path", 10, std::bind(&PurePursuitNode::pathCallback, this, std::placeholders::_1));
    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

    last_path_time_ = now();
    safety_timer_ = create_wall_timer(100ms, std::bind(&PurePursuitNode::safetyTimer, this));

    RCLCPP_INFO(get_logger(), "pure_pursuit_node publishing /drive");
  }

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    last_path_time_ = now();

    if (msg->poses.size() < 2) {
      publishStop();
      return;
    }

    const double lookahead = adaptiveLookahead();
    const auto * goal = findLookaheadPoint(*msg, lookahead);
    if (goal == nullptr) {
      publishStop();
      return;
    }

    const double goal_x = goal->pose.position.x;
    const double goal_y = goal->pose.position.y;
    if (std::hypot(goal_x, goal_y) < 1e-3) {
      publishStop();
      return;
    }

    const double alpha = std::atan2(goal_y, goal_x);
    double steering_angle = std::atan2(2.0 * wheelbase_ * std::sin(alpha), lookahead);
    steering_angle *= steering_gain_;
    steering_angle = std::clamp(steering_angle, -max_steering_angle_, max_steering_angle_);

    const double turn_scale = 1.0 - 0.5 * std::abs(steering_angle) / max_steering_angle_;
    double speed = speed_target_ * std::clamp(turn_scale, 0.0, 1.0);
    speed = std::clamp(speed, 0.3, speed_max_);
    current_speed_command_ = speed;

    ackermann_msgs::msg::AckermannDriveStamped drive;
    drive.header.stamp = now();
    drive.header.frame_id = "base_link";
    drive.drive.steering_angle = steering_angle;
    drive.drive.speed = speed;
    drive_pub_->publish(drive);
  }

  const geometry_msgs::msg::PoseStamped * findLookaheadPoint(
    const nav_msgs::msg::Path & path, double lookahead) const
  {
    const geometry_msgs::msg::PoseStamped * last_forward = nullptr;
    for (const auto & pose : path.poses) {
      const double x = pose.pose.position.x;
      const double y = pose.pose.position.y;
      if (x < -0.05) {
        continue;
      }
      last_forward = &pose;
      if (std::hypot(x, y) >= lookahead) {
        return &pose;
      }
    }
    return last_forward;
  }

  double adaptiveLookahead() const
  {
    if (speed_max_ <= 1e-6) {
      return std::clamp(lookahead_distance_, lookahead_min_, lookahead_max_);
    }
    const double speed_for_scale = current_speed_command_ > 1e-3 ? current_speed_command_ : speed_target_;
    const double ratio = std::clamp(speed_for_scale / speed_max_, 0.0, 1.0);
    const double adaptive = lookahead_min_ + ratio * (lookahead_max_ - lookahead_min_);
    return std::clamp(adaptive, lookahead_min_, lookahead_max_);
  }

  void safetyTimer()
  {
    if ((now() - last_path_time_).seconds() > 0.5) {
      publishStop();
    }
  }

  void publishStop()
  {
    current_speed_command_ = 0.0;
    ackermann_msgs::msg::AckermannDriveStamped drive;
    drive.header.stamp = now();
    drive.header.frame_id = "base_link";
    drive.drive.steering_angle = 0.0;
    drive.drive.speed = 0.0;
    drive_pub_->publish(drive);
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::TimerBase::SharedPtr safety_timer_;

  rclcpp::Time last_path_time_;
  double lookahead_distance_{1.5};
  double lookahead_min_{0.8};
  double lookahead_max_{3.0};
  double speed_target_{1.5};
  double speed_max_{3.0};
  double wheelbase_{0.32};
  double steering_gain_{1.0};
  double max_steering_angle_{0.4};
  double current_speed_command_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitNode>());
  rclcpp::shutdown();
  return 0;
}

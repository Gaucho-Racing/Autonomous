#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <memory>

class AckermannToTwistNode : public rclcpp::Node
{
public:
  AckermannToTwistNode()
  : Node("ackermann_to_twist_node")
  {
    declare_parameter("wheelbase", 0.32);
    wheelbase_ = std::max(1e-3, get_parameter("wheelbase").as_double());
    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    drive_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      "/drive", 10,
      [this](ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr msg) {
        geometry_msgs::msg::Twist twist;
        twist.linear.x = msg->drive.speed;
        twist.angular.z = msg->drive.speed * std::tan(msg->drive.steering_angle) / wheelbase_;
        twist_pub_->publish(twist);
      });
  }

private:
  double wheelbase_{0.32};
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AckermannToTwistNode>());
  rclcpp::shutdown();
  return 0;
}

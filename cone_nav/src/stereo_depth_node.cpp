#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>

class StereoDepthNode : public rclcpp::Node
{
public:
  StereoDepthNode()
  : Node("stereo_depth_node")
  {
    declare_parameter("sim_image_topic", "/sim/camera/image_raw");
    declare_parameter("sim_right_image_topic", "/sim/camera/right/image_raw");
    declare_parameter("sim_camera_info_topic", "/sim/camera/camera_info");
    declare_parameter("sim_right_camera_info_topic", "/sim/camera/right/camera_info");
    declare_parameter("stereo_depth_topic", "/sim/camera/stereo_depth");
    declare_parameter("stereo_baseline", 0.12);
    declare_parameter("stereo_min_disparity", 0);
    declare_parameter("stereo_num_disparities", 128);
    declare_parameter("stereo_block_size", 7);
    declare_parameter("stereo_max_depth", 15.0);

    baseline_ = get_parameter("stereo_baseline").as_double();
    max_depth_ = get_parameter("stereo_max_depth").as_double();
    int min_disparity = static_cast<int>(get_parameter("stereo_min_disparity").as_int());
    int num_disparities = static_cast<int>(get_parameter("stereo_num_disparities").as_int());
    num_disparities = std::max(16, ((num_disparities + 15) / 16) * 16);
    int block_size = static_cast<int>(get_parameter("stereo_block_size").as_int());
    block_size = std::max(3, block_size | 1);
    matcher_ = cv::StereoSGBM::create(
      min_disparity, num_disparities, block_size, 8 * 3 * block_size * block_size,
      32 * 3 * block_size * block_size, 1, 31, 10, 100, 2,
      cv::StereoSGBM::MODE_SGBM_3WAY);

    left_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      get_parameter("sim_camera_info_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        fx_ = msg->k[0];
      });
    right_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      get_parameter("sim_right_camera_info_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (std::abs(msg->p[0]) > 1e-6 && std::abs(msg->p[3]) > 1e-6) {
          baseline_ = std::abs(msg->p[3] / msg->p[0]);
        }
      });

    left_sub_.subscribe(
      this, get_parameter("sim_image_topic").as_string(), rmw_qos_profile_sensor_data);
    right_sub_.subscribe(
      this, get_parameter("sim_right_image_topic").as_string(), rmw_qos_profile_sensor_data);
    sync_ = std::make_shared<Synchronizer>(SyncPolicy(5), left_sub_, right_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.02));
    sync_->registerCallback(
      std::bind(&StereoDepthNode::imageCallback, this, std::placeholders::_1, std::placeholders::_2));
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>(
      get_parameter("stereo_depth_topic").as_string(), rclcpp::SensorDataQoS());
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  void imageCallback(
    sensor_msgs::msg::Image::ConstSharedPtr left_msg,
    sensor_msgs::msg::Image::ConstSharedPtr right_msg)
  {
    if (fx_ <= 0.0 || baseline_ <= 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for stereo calibration");
      return;
    }
    cv_bridge::CvImageConstPtr left;
    cv_bridge::CvImageConstPtr right;
    try {
      left = cv_bridge::toCvCopy(left_msg, sensor_msgs::image_encodings::MONO8);
      right = cv_bridge::toCvCopy(right_msg, sensor_msgs::image_encodings::MONO8);
    } catch (const cv_bridge::Exception & ex) {
      RCLCPP_WARN(get_logger(), "Stereo image conversion failed: %s", ex.what());
      return;
    }
    if (left->image.size() != right->image.size()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Stereo image sizes differ");
      return;
    }

    cv::Mat disparity_fixed;
    matcher_->compute(left->image, right->image, disparity_fixed);
    cv::Mat depth(left->image.size(), CV_32FC1, cv::Scalar(
      std::numeric_limits<float>::quiet_NaN()));
    for (int v = 0; v < disparity_fixed.rows; ++v) {
      for (int u = 0; u < disparity_fixed.cols; ++u) {
        const float disparity = static_cast<float>(disparity_fixed.at<int16_t>(v, u)) / 16.0F;
        if (disparity <= 0.0F) {
          continue;
        }
        const float range = static_cast<float>(fx_ * baseline_) / disparity;
        if (std::isfinite(range) && range > 0.0F && range <= max_depth_) {
          depth.at<float>(v, u) = range;
        }
      }
    }
    cv_bridge::CvImage output(left_msg->header, sensor_msgs::image_encodings::TYPE_32FC1, depth);
    depth_pub_->publish(*output.toImageMsg());
  }

  double fx_{0.0};
  double baseline_{0.12};
  double max_depth_{15.0};
  cv::Ptr<cv::StereoSGBM> matcher_;
  message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> right_sub_;
  std::shared_ptr<Synchronizer> sync_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StereoDepthNode>());
  rclcpp::shutdown();
  return 0;
}

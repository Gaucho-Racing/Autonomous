#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

struct ConePoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  int class_id{0};
};

class ConeLocalizerNode : public rclcpp::Node
{
public:
  ConeLocalizerNode()
  : Node("cone_localizer_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter("use_sim", false);
    declare_parameter("camera_frame", "zed2i_left_camera_frame");
    declare_parameter("base_frame", "base_link");
    declare_parameter("max_detection_depth", 15.0);
    declare_parameter("min_detection_depth", 0.3);
    declare_parameter("cone_cluster_radius", 0.4);
    declare_parameter("real_depth_topic", "/zed2i/zed_node/depth/depth_registered");
    declare_parameter("sim_depth_topic", "/sim/camera/depth");
    declare_parameter("real_camera_info_topic", "/zed2i/zed_node/left/camera_info");
    declare_parameter("sim_camera_info_topic", "/sim/camera/camera_info");

    use_sim_ = get_parameter("use_sim").as_bool();
    camera_frame_ = get_parameter("camera_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    max_detection_depth_ = get_parameter("max_detection_depth").as_double();
    min_detection_depth_ = get_parameter("min_detection_depth").as_double();
    cone_cluster_radius_ = get_parameter("cone_cluster_radius").as_double();

    const std::string depth_topic = use_sim_ ?
      get_parameter("sim_depth_topic").as_string() :
      get_parameter("real_depth_topic").as_string();
    const std::string camera_info_topic = use_sim_ ?
      get_parameter("sim_camera_info_topic").as_string() :
      get_parameter("real_camera_info_topic").as_string();

    left_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/cones/left", 10);
    right_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/cones/right", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/cones/all", 10);

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, 10,
      std::bind(&ConeLocalizerNode::cameraInfoCallback, this, std::placeholders::_1));
    depth_watch_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr) { last_depth_time_ = now(); });

    detections_sub_.subscribe(this, "/cone_detections", rmw_qos_profile_sensor_data);
    depth_sub_.subscribe(this, depth_topic, rmw_qos_profile_sensor_data);
    sync_ = std::make_shared<Synchronizer>(SyncPolicy(10), detections_sub_, depth_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.05));
    sync_->registerCallback(
      std::bind(&ConeLocalizerNode::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

    watchdog_timer_ = create_wall_timer(
      std::chrono::seconds(5), std::bind(&ConeLocalizerNode::depthWatchdog, this));

    RCLCPP_INFO(
      get_logger(), "cone_localizer_node using depth topic %s and camera info %s",
      depth_topic.c_str(), camera_info_topic.c_str());
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    vision_msgs::msg::Detection2DArray, sensor_msgs::msg::Image>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    fx_ = msg->k[0];
    fy_ = msg->k[4];
    cx_ = msg->k[2];
    cy_ = msg->k[5];
    have_camera_info_ = fx_ > 0.0 && fy_ > 0.0;
  }

  void syncCallback(
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr detections_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr depth_msg)
  {
    last_depth_time_ = now();

    if (!have_camera_info_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for camera_info intrinsics");
      publishEmpty(detections_msg->header.stamp);
      return;
    }

    if (!have_transform_ && !cacheTransform()) {
      publishEmpty(detections_msg->header.stamp);
      return;
    }

    cv_bridge::CvImageConstPtr depth_cv;
    try {
      depth_cv = cv_bridge::toCvShare(depth_msg);
    } catch (const cv_bridge::Exception & ex) {
      RCLCPP_WARN(get_logger(), "Depth conversion failed: %s", ex.what());
      publishEmpty(detections_msg->header.stamp);
      return;
    }

    std::vector<ConePoint> left;
    std::vector<ConePoint> right;
    std::vector<ConePoint> orange;

    for (const auto & detection : detections_msg->detections) {
      if (detection.results.empty()) {
        continue;
      }

      int class_id = -1;
      try {
        class_id = std::stoi(detection.results.front().hypothesis.class_id);
      } catch (const std::exception &) {
        continue;
      }

      const int u = static_cast<int>(std::lround(detection.bbox.center.position.x));
      const int v = static_cast<int>(std::lround(detection.bbox.center.position.y));
      const double depth = readDepthMeters(depth_cv->image, u, v);
      if (!std::isfinite(depth) || depth < min_detection_depth_ || depth > max_detection_depth_) {
        continue;
      }

      geometry_msgs::msg::PointStamped point_camera;
      point_camera.header = detections_msg->header;
      point_camera.header.frame_id = camera_frame_;
      point_camera.point.x = (static_cast<double>(u) - cx_) * depth / fx_;
      point_camera.point.y = (static_cast<double>(v) - cy_) * depth / fy_;
      point_camera.point.z = depth;

      geometry_msgs::msg::PointStamped point_base;
      tf2::doTransform(point_camera, point_base, camera_to_base_);

      ConePoint cone;
      cone.x = point_base.point.x;
      cone.y = point_base.point.y;
      cone.z = point_base.point.z;
      cone.class_id = class_id;

      if (class_id == 1) {
        mergeCone(left, cone);
      } else if (class_id == 0) {
        mergeCone(right, cone);
      } else if (class_id == 2) {
        mergeCone(orange, cone);
      }
    }

    publishCones(left, right, orange, detections_msg->header.stamp);
  }

  bool cacheTransform()
  {
    try {
      camera_to_base_ = tf_buffer_.lookupTransform(base_frame_, camera_frame_, tf2::TimePointZero);
      have_transform_ = true;
      RCLCPP_INFO(
        get_logger(), "Cached transform %s -> %s", camera_frame_.c_str(), base_frame_.c_str());
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for transform %s -> %s: %s",
        camera_frame_.c_str(), base_frame_.c_str(), ex.what());
      return false;
    }
  }

  double readDepthMeters(const cv::Mat & depth_image, int u, int v) const
  {
    if (u < 0 || v < 0 || u >= depth_image.cols || v >= depth_image.rows) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    if (depth_image.type() == CV_32FC1) {
      return static_cast<double>(depth_image.at<float>(v, u));
    }
    if (depth_image.type() == CV_16UC1) {
      return static_cast<double>(depth_image.at<uint16_t>(v, u)) * 0.001;
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  void mergeCone(std::vector<ConePoint> & cones, const ConePoint & candidate) const
  {
    for (auto & existing : cones) {
      const double dx = existing.x - candidate.x;
      const double dy = existing.y - candidate.y;
      const double dz = existing.z - candidate.z;
      if (std::sqrt(dx * dx + dy * dy + dz * dz) < cone_cluster_radius_) {
        existing.x = 0.5 * (existing.x + candidate.x);
        existing.y = 0.5 * (existing.y + candidate.y);
        existing.z = 0.5 * (existing.z + candidate.z);
        return;
      }
    }
    cones.push_back(candidate);
  }

  void publishEmpty(const builtin_interfaces::msg::Time & stamp)
  {
    publishCones({}, {}, {}, stamp);
  }

  void publishCones(
    const std::vector<ConePoint> & left,
    const std::vector<ConePoint> & right,
    const std::vector<ConePoint> & orange,
    const builtin_interfaces::msg::Time & stamp)
  {
    geometry_msgs::msg::PoseArray left_msg;
    geometry_msgs::msg::PoseArray right_msg;
    left_msg.header.stamp = stamp;
    left_msg.header.frame_id = base_frame_;
    right_msg.header = left_msg.header;

    for (const auto & cone : left) {
      left_msg.poses.push_back(toPose(cone));
    }
    for (const auto & cone : right) {
      right_msg.poses.push_back(toPose(cone));
    }

    left_pub_->publish(left_msg);
    right_pub_->publish(right_msg);
    marker_pub_->publish(buildMarkers(left, right, orange, stamp));
  }

  geometry_msgs::msg::Pose toPose(const ConePoint & cone) const
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = cone.x;
    pose.position.y = cone.y;
    pose.position.z = cone.z;
    pose.orientation.w = 1.0;
    return pose;
  }

  visualization_msgs::msg::MarkerArray buildMarkers(
    const std::vector<ConePoint> & left,
    const std::vector<ConePoint> & right,
    const std::vector<ConePoint> & orange,
    const builtin_interfaces::msg::Time & stamp) const
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    int id = 0;
    appendMarkers(markers, left, id, "yellow_cones", stamp, 1.0, 0.9, 0.0, 0.22);
    appendMarkers(markers, right, id, "blue_cones", stamp, 0.0, 0.25, 1.0, 0.22);
    appendMarkers(markers, orange, id, "orange_cones", stamp, 1.0, 0.35, 0.0, 0.32);
    return markers;
  }

  void appendMarkers(
    visualization_msgs::msg::MarkerArray & markers,
    const std::vector<ConePoint> & cones,
    int & id,
    const std::string & ns,
    const builtin_interfaces::msg::Time & stamp,
    double r,
    double g,
    double b,
    double scale) const
  {
    for (const auto & cone : cones) {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = stamp;
      marker.header.frame_id = base_frame_;
      marker.ns = ns;
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = toPose(cone);
      marker.scale.x = scale;
      marker.scale.y = scale;
      marker.scale.z = scale;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.color.a = 1.0;
      marker.lifetime = rclcpp::Duration::from_seconds(0.3);
      markers.markers.push_back(marker);
    }
  }

  void depthWatchdog()
  {
    if (last_depth_time_.nanoseconds() == 0 || (now() - last_depth_time_).seconds() > 5.0) {
      RCLCPP_WARN(get_logger(), "Depth topic has not published recently");
    }
  }

  bool use_sim_{false};
  std::string camera_frame_{"zed2i_left_camera_frame"};
  std::string base_frame_{"base_link"};
  double max_detection_depth_{15.0};
  double min_detection_depth_{0.3};
  double cone_cluster_radius_{0.4};

  double fx_{0.0};
  double fy_{0.0};
  double cx_{0.0};
  double cy_{0.0};
  bool have_camera_info_{false};
  bool have_transform_{false};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  geometry_msgs::msg::TransformStamped camera_to_base_;

  message_filters::Subscriber<vision_msgs::msg::Detection2DArray> detections_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  std::shared_ptr<Synchronizer> sync_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_watch_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr left_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr right_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::Time last_depth_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConeLocalizerNode>());
  rclcpp::shutdown();
  return 0;
}

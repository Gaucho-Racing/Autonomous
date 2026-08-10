#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

class DepthObstacleNode : public rclcpp::Node
{
public:
  DepthObstacleNode()
  : Node("depth_obstacle_node"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    declare_parameter("use_sim", false);
    declare_parameter("camera_frame", "zed2i_left_camera_optical_frame");
    declare_parameter("base_frame", "base_link");
    declare_parameter("real_depth_topic", "/zed2i/zed_node/depth/depth_registered");
    declare_parameter("sim_depth_topic", "/sim/camera/depth");
    declare_parameter("real_camera_info_topic", "/zed2i/zed_node/left/camera_info");
    declare_parameter("sim_camera_info_topic", "/sim/camera/camera_info");
    declare_parameter("obstacle_grid_topic", "/obstacles/local_grid");
    declare_parameter("obstacle_points_topic", "/obstacles/points");
    declare_parameter("obstacle_min_depth", 0.30);
    declare_parameter("obstacle_max_depth", 8.0);
    declare_parameter("obstacle_min_height", 0.05);
    declare_parameter("obstacle_max_height", 1.50);
    declare_parameter("obstacle_min_x", -0.30);
    declare_parameter("obstacle_max_x", 8.0);
    declare_parameter("obstacle_half_width", 3.0);
    declare_parameter("obstacle_pixel_stride", 4);
    declare_parameter("obstacle_grid_resolution", 0.10);
    declare_parameter("obstacle_inflation_radius", 0.22);
    declare_parameter("obstacle_persistence_sec", 0.25);
    declare_parameter("obstacle_hood_max_x", 0.45);
    declare_parameter("obstacle_hood_half_width", 0.20);
    declare_parameter("obstacle_hood_max_height", 0.30);

    use_sim_ = get_parameter("use_sim").as_bool();
    camera_frame_ = get_parameter("camera_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    min_depth_ = get_parameter("obstacle_min_depth").as_double();
    max_depth_ = get_parameter("obstacle_max_depth").as_double();
    min_height_ = get_parameter("obstacle_min_height").as_double();
    max_height_ = get_parameter("obstacle_max_height").as_double();
    min_x_ = get_parameter("obstacle_min_x").as_double();
    max_x_ = get_parameter("obstacle_max_x").as_double();
    half_width_ = get_parameter("obstacle_half_width").as_double();
    pixel_stride_ = std::max(
      1, static_cast<int>(get_parameter("obstacle_pixel_stride").as_int()));
    resolution_ = get_parameter("obstacle_grid_resolution").as_double();
    inflation_radius_ = get_parameter("obstacle_inflation_radius").as_double();
    persistence_sec_ = get_parameter("obstacle_persistence_sec").as_double();
    hood_max_x_ = get_parameter("obstacle_hood_max_x").as_double();
    hood_half_width_ = get_parameter("obstacle_hood_half_width").as_double();
    hood_max_height_ = get_parameter("obstacle_hood_max_height").as_double();

    grid_width_ = static_cast<int>(std::ceil((max_x_ - min_x_) / resolution_));
    grid_height_ = static_cast<int>(std::ceil((2.0 * half_width_) / resolution_));
    last_seen_.assign(static_cast<size_t>(grid_width_ * grid_height_), -1.0);

    const std::string depth_topic = use_sim_ ?
      get_parameter("sim_depth_topic").as_string() :
      get_parameter("real_depth_topic").as_string();
    const std::string camera_info_topic = use_sim_ ?
      get_parameter("sim_camera_info_topic").as_string() :
      get_parameter("real_camera_info_topic").as_string();

    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      get_parameter("obstacle_grid_topic").as_string(), 1);
    points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      get_parameter("obstacle_points_topic").as_string(), rclcpp::SensorDataQoS());
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, rclcpp::SensorDataQoS(),
      std::bind(&DepthObstacleNode::cameraInfoCallback, this, std::placeholders::_1));
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic, rclcpp::SensorDataQoS(),
      std::bind(&DepthObstacleNode::depthCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Depth obstacle extraction: %s -> /obstacles/local_grid (%dx%d at %.2fm)",
      depth_topic.c_str(), grid_width_, grid_height_, resolution_);
  }

private:
  struct Point3
  {
    float x;
    float y;
    float z;
  };

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    fx_ = msg->k[0];
    fy_ = msg->k[4];
    cx_ = msg->k[2];
    cy_ = msg->k[5];
    have_camera_info_ = fx_ > 0.0 && fy_ > 0.0;
  }

  void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    if (!have_camera_info_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for camera intrinsics");
      return;
    }
    if (!cacheTransform()) {
      return;
    }

    cv_bridge::CvImageConstPtr depth;
    try {
      depth = cv_bridge::toCvShare(msg);
    } catch (const cv_bridge::Exception & ex) {
      RCLCPP_WARN(get_logger(), "Depth conversion failed: %s", ex.what());
      return;
    }
    if (depth->image.type() != CV_32FC1 && depth->image.type() != CV_16UC1) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Depth must be 32FC1 metres or 16UC1 millimetres");
      return;
    }

    const double stamp_sec = rclcpp::Time(msg->header.stamp).seconds();
    std::vector<Point3> points;
    points.reserve(static_cast<size_t>(depth->image.rows * depth->image.cols) /
      static_cast<size_t>(pixel_stride_ * pixel_stride_));

    for (int v = 0; v < depth->image.rows; v += pixel_stride_) {
      for (int u = 0; u < depth->image.cols; u += pixel_stride_) {
        const double range = readDepth(depth->image, u, v);
        if (!std::isfinite(range) || range < min_depth_ || range > max_depth_) {
          continue;
        }

        geometry_msgs::msg::PointStamped camera_point;
        camera_point.header = msg->header;
        camera_point.header.frame_id = camera_frame_;
        camera_point.point.x = (static_cast<double>(u) - cx_) * range / fx_;
        camera_point.point.y = (static_cast<double>(v) - cy_) * range / fy_;
        camera_point.point.z = range;

        geometry_msgs::msg::PointStamped base_point;
        tf2::doTransform(camera_point, base_point, camera_to_base_);
        if (base_point.point.x < min_x_ || base_point.point.x >= max_x_ ||
          std::abs(base_point.point.y) >= half_width_ ||
          base_point.point.z < min_height_ || base_point.point.z > max_height_)
        {
          continue;
        }
        if (base_point.point.x <= hood_max_x_ &&
          std::abs(base_point.point.y) <= hood_half_width_ &&
          base_point.point.z <= hood_max_height_)
        {
          continue;
        }

        const int cell = cellIndex(base_point.point.x, base_point.point.y);
        if (cell >= 0) {
          last_seen_[static_cast<size_t>(cell)] = stamp_sec;
          points.push_back(Point3{
            static_cast<float>(base_point.point.x),
            static_cast<float>(base_point.point.y),
            static_cast<float>(base_point.point.z)});
        }
      }
    }

    publishGrid(msg->header.stamp, stamp_sec);
    publishPoints(msg->header.stamp, points);
  }

  bool cacheTransform()
  {
    try {
      camera_to_base_ = tf_buffer_.lookupTransform(base_frame_, camera_frame_, tf2::TimePointZero);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for transform %s -> %s: %s",
        camera_frame_.c_str(), base_frame_.c_str(), ex.what());
      return false;
    }
  }

  double readDepth(const cv::Mat & image, int u, int v) const
  {
    if (image.type() == CV_32FC1) {
      return static_cast<double>(image.at<float>(v, u));
    }
    return static_cast<double>(image.at<uint16_t>(v, u)) * 0.001;
  }

  int cellIndex(double x, double y) const
  {
    const int gx = static_cast<int>(std::floor((x - min_x_) / resolution_));
    const int gy = static_cast<int>(std::floor((y + half_width_) / resolution_));
    if (gx < 0 || gx >= grid_width_ || gy < 0 || gy >= grid_height_) {
      return -1;
    }
    return gy * grid_width_ + gx;
  }

  void publishGrid(const builtin_interfaces::msg::Time & stamp, double stamp_sec)
  {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = stamp;
    grid.header.frame_id = base_frame_;
    grid.info.resolution = static_cast<float>(resolution_);
    grid.info.width = static_cast<uint32_t>(grid_width_);
    grid.info.height = static_cast<uint32_t>(grid_height_);
    grid.info.origin.position.x = min_x_;
    grid.info.origin.position.y = -half_width_;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(static_cast<size_t>(grid_width_ * grid_height_), 0);

    const int inflation_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));
    for (int gy = 0; gy < grid_height_; ++gy) {
      for (int gx = 0; gx < grid_width_; ++gx) {
        const int source = gy * grid_width_ + gx;
        const double seen = last_seen_[static_cast<size_t>(source)];
        if (seen < 0.0 || stamp_sec - seen > persistence_sec_) {
          continue;
        }
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
          for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
            if (dx * dx + dy * dy > inflation_cells * inflation_cells) {
              continue;
            }
            const int ix = gx + dx;
            const int iy = gy + dy;
            if (ix >= 0 && ix < grid_width_ && iy >= 0 && iy < grid_height_) {
              grid.data[static_cast<size_t>(iy * grid_width_ + ix)] = 100;
            }
          }
        }
      }
    }
    grid_pub_->publish(grid);
  }

  void publishPoints(const builtin_interfaces::msg::Time & stamp, const std::vector<Point3> & points)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = base_frame_;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(points.size());
    cloud.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    for (const auto & point : points) {
      *x = point.x;
      *y = point.y;
      *z = point.z;
      ++x;
      ++y;
      ++z;
    }
    points_pub_->publish(cloud);
  }

  bool use_sim_{false};
  std::string camera_frame_;
  std::string base_frame_;
  double fx_{0.0};
  double fy_{0.0};
  double cx_{0.0};
  double cy_{0.0};
  bool have_camera_info_{false};
  double min_depth_{0.3};
  double max_depth_{8.0};
  double min_height_{0.05};
  double max_height_{1.5};
  double min_x_{-0.3};
  double max_x_{8.0};
  double half_width_{3.0};
  int pixel_stride_{4};
  double resolution_{0.1};
  double inflation_radius_{0.22};
  double persistence_sec_{0.25};
  double hood_max_x_{0.45};
  double hood_half_width_{0.20};
  double hood_max_height_{0.30};
  int grid_width_{0};
  int grid_height_{0};
  std::vector<double> last_seen_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  geometry_msgs::msg::TransformStamped camera_to_base_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthObstacleNode>());
  rclcpp::shutdown();
  return 0;
}

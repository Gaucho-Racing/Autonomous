#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cuda_runtime_api.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/cuda_stream_accessor.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

namespace
{

class TrtLogger final : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * msg) noexcept override
  {
    if (severity <= Severity::kWARNING) {
      std::cerr << "[TensorRT] " << msg << std::endl;
    }
  }
};

struct ParsedDetection
{
  float cx{0.0F};
  float cy{0.0F};
  float w{0.0F};
  float h{0.0F};
  float confidence{0.0F};
  int class_id{0};
};

size_t volumeOf(const nvinfer1::Dims & dims)
{
  if (dims.nbDims <= 0) {
    return 0;
  }
  size_t volume = 1;
  for (int i = 0; i < dims.nbDims; ++i) {
    if (dims.d[i] <= 0) {
      return 0;
    }
    volume *= static_cast<size_t>(dims.d[i]);
  }
  return volume;
}

float intersectionOverUnion(const ParsedDetection & a, const ParsedDetection & b)
{
  const float ax1 = a.cx - 0.5F * a.w;
  const float ay1 = a.cy - 0.5F * a.h;
  const float ax2 = a.cx + 0.5F * a.w;
  const float ay2 = a.cy + 0.5F * a.h;
  const float bx1 = b.cx - 0.5F * b.w;
  const float by1 = b.cy - 0.5F * b.h;
  const float bx2 = b.cx + 0.5F * b.w;
  const float by2 = b.cy + 0.5F * b.h;

  const float inter_x1 = std::max(ax1, bx1);
  const float inter_y1 = std::max(ay1, by1);
  const float inter_x2 = std::min(ax2, bx2);
  const float inter_y2 = std::min(ay2, by2);
  const float inter_w = std::max(0.0F, inter_x2 - inter_x1);
  const float inter_h = std::max(0.0F, inter_y2 - inter_y1);
  const float inter_area = inter_w * inter_h;
  const float union_area = a.w * a.h + b.w * b.h - inter_area;
  if (union_area <= std::numeric_limits<float>::epsilon()) {
    return 0.0F;
  }
  return inter_area / union_area;
}

std::vector<ParsedDetection> nonMaximumSuppression(
  std::vector<ParsedDetection> detections, float iou_threshold)
{
  std::sort(
    detections.begin(), detections.end(),
    [](const ParsedDetection & lhs, const ParsedDetection & rhs) {
      return lhs.confidence > rhs.confidence;
    });

  std::vector<ParsedDetection> kept;
  std::vector<bool> suppressed(detections.size(), false);
  for (size_t i = 0; i < detections.size(); ++i) {
    if (suppressed[i]) {
      continue;
    }
    kept.push_back(detections[i]);
    for (size_t j = i + 1; j < detections.size(); ++j) {
      if (suppressed[j] || detections[i].class_id != detections[j].class_id) {
        continue;
      }
      if (intersectionOverUnion(detections[i], detections[j]) > iou_threshold) {
        suppressed[j] = true;
      }
    }
  }
  return kept;
}

}  // namespace

class ConeDetectorNode : public rclcpp::Node
{
public:
  ConeDetectorNode()
  : Node("cone_detector_node")
  {
    declare_parameter("use_sim", false);
    declare_parameter("trt_engine_path", "models/cone_yolo.engine");
    declare_parameter("detection_confidence_threshold", 0.5);
    declare_parameter("nms_iou_threshold", 0.45);
    declare_parameter("real_image_topic", "/zed2i/zed_node/left/image_rect_color");
    declare_parameter("sim_image_topic", "/sim/camera/image_raw");
    declare_parameter("trt_input_width", 640);
    declare_parameter("trt_input_height", 640);
    declare_parameter("trt_output_num_detections", 25200);

    use_sim_ = get_parameter("use_sim").as_bool();
    confidence_threshold_ = static_cast<float>(
      get_parameter("detection_confidence_threshold").as_double());
    nms_iou_threshold_ = static_cast<float>(get_parameter("nms_iou_threshold").as_double());
    input_w_ = get_parameter("trt_input_width").as_int();
    input_h_ = get_parameter("trt_input_height").as_int();
    output_num_detections_fallback_ = get_parameter("trt_output_num_detections").as_int();

    const std::string image_topic = use_sim_ ?
      get_parameter("sim_image_topic").as_string() :
      get_parameter("real_image_topic").as_string();

    if (!queryCudaDevice()) {
      RCLCPP_WARN(
        get_logger(),
        "Could not confirm CUDA GPU availability. TensorRT initialization will still be attempted.");
    }

    const auto engine_path = resolveEnginePath(get_parameter("trt_engine_path").as_string());
    if (!loadEngine(engine_path)) {
      RCLCPP_FATAL(get_logger(), "Failed to load TensorRT engine: %s", engine_path.c_str());
      rclcpp::shutdown();
      return;
    }

    detections_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>("/cone_detections", 10);
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS(),
      std::bind(&ConeDetectorNode::imageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "cone_detector_node subscribed to %s, publishing /cone_detections",
      image_topic.c_str());
  }

  ~ConeDetectorNode() override
  {
    for (void * buffer : device_bindings_) {
      if (buffer != nullptr) {
        cudaFree(buffer);
      }
    }
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
    }
    if (context_ != nullptr) {
      context_->destroy();
    }
    if (engine_ != nullptr) {
      engine_->destroy();
    }
    if (runtime_ != nullptr) {
      runtime_->destroy();
    }
  }

private:
  bool queryCudaDevice()
  {
    int device = -1;
    if (cudaGetDevice(&device) != cudaSuccess) {
      return false;
    }

    cudaDeviceProp prop{};
    if (cudaGetDeviceProperties(&prop, device) != cudaSuccess) {
      return false;
    }

    RCLCPP_INFO(
      get_logger(), "Using CUDA device %d: %s, compute capability %d.%d",
      device, prop.name, prop.major, prop.minor);
    return true;
  }

  std::string resolveEnginePath(const std::string & configured_path) const
  {
    const std::filesystem::path path(configured_path);
    if (path.is_absolute() && std::filesystem::exists(path)) {
      return configured_path;
    }
    if (std::filesystem::exists(path)) {
      return std::filesystem::absolute(path).string();
    }

    try {
      const auto share_dir = ament_index_cpp::get_package_share_directory("cone_nav");
      const auto share_path = std::filesystem::path(share_dir) / configured_path;
      if (std::filesystem::exists(share_path)) {
        return share_path.string();
      }
    } catch (const std::exception &) {
    }

    return configured_path;
  }

  bool loadEngine(const std::string & engine_path)
  {
    std::ifstream engine_file(engine_path, std::ios::binary);
    if (!engine_file) {
      RCLCPP_FATAL(get_logger(), "TensorRT engine file does not exist or cannot be opened");
      return false;
    }

    engine_file.seekg(0, std::ios::end);
    const auto size = engine_file.tellg();
    engine_file.seekg(0, std::ios::beg);
    std::vector<char> engine_data(static_cast<size_t>(size));
    engine_file.read(engine_data.data(), static_cast<std::streamsize>(size));

    initLibNvInferPlugins(&trt_logger_, "");
    runtime_ = nvinfer1::createInferRuntime(trt_logger_);
    if (runtime_ == nullptr) {
      return false;
    }

    engine_ = runtime_->deserializeCudaEngine(engine_data.data(), engine_data.size());
    if (engine_ == nullptr) {
      return false;
    }

    context_ = engine_->createExecutionContext();
    if (context_ == nullptr) {
      return false;
    }

    if (!selectBindings() || !configureInputShape() || !allocateBuffers()) {
      return false;
    }

    if (cudaStreamCreate(&stream_) != cudaSuccess) {
      RCLCPP_FATAL(get_logger(), "Failed to create CUDA stream");
      return false;
    }

    ready_ = true;
    return true;
  }

  bool selectBindings()
  {
    const int nb_bindings = engine_->getNbBindings();
    input_binding_ = -1;
    output_binding_ = -1;
    for (int i = 0; i < nb_bindings; ++i) {
      if (engine_->bindingIsInput(i) && input_binding_ < 0) {
        input_binding_ = i;
      } else if (!engine_->bindingIsInput(i) && output_binding_ < 0) {
        output_binding_ = i;
      }
    }

    if (input_binding_ < 0 || output_binding_ < 0) {
      RCLCPP_FATAL(get_logger(), "TensorRT engine must expose one input and one output binding");
      return false;
    }

    if (engine_->getBindingDataType(input_binding_) != nvinfer1::DataType::kFLOAT ||
      engine_->getBindingDataType(output_binding_) != nvinfer1::DataType::kFLOAT)
    {
      RCLCPP_FATAL(get_logger(), "Only FP32 input/output bindings are supported by this node");
      return false;
    }

    device_bindings_.assign(static_cast<size_t>(nb_bindings), nullptr);
    return true;
  }

  bool configureInputShape()
  {
    nvinfer1::Dims dims = engine_->getBindingDimensions(input_binding_);
    if (dims.nbDims < 3) {
      RCLCPP_FATAL(get_logger(), "Expected TensorRT input dims [N,3,H,W] or [3,H,W]");
      return false;
    }

    const int c_index = dims.nbDims - 3;
    const int h_index = dims.nbDims - 2;
    const int w_index = dims.nbDims - 1;

    const int channels = dims.d[c_index] > 0 ? dims.d[c_index] : 3;
    if (channels != 3) {
      RCLCPP_FATAL(get_logger(), "Expected 3-channel TensorRT input, got %d channels", channels);
      return false;
    }
    if (dims.d[h_index] > 0) {
      input_h_ = dims.d[h_index];
    } else {
      dims.d[h_index] = input_h_;
    }
    if (dims.d[w_index] > 0) {
      input_w_ = dims.d[w_index];
    } else {
      dims.d[w_index] = input_w_;
    }
    dims.d[c_index] = 3;
    if (dims.nbDims == 4 && dims.d[0] <= 0) {
      dims.d[0] = 1;
    }

    if (!context_->setBindingDimensions(input_binding_, dims)) {
      RCLCPP_FATAL(get_logger(), "Failed to set TensorRT input binding dimensions");
      return false;
    }

    input_count_ = static_cast<size_t>(3 * input_h_ * input_w_);

    nvinfer1::Dims output_dims = context_->getBindingDimensions(output_binding_);
    output_count_ = volumeOf(output_dims);
    if (output_count_ == 0 || output_count_ % 6 != 0) {
      output_count_ = static_cast<size_t>(std::max(1, output_num_detections_fallback_)) * 6;
      RCLCPP_WARN(
        get_logger(),
        "Using fallback TensorRT output size %zu floats because output dims are dynamic/unknown",
        output_count_);
    }

    RCLCPP_INFO(
      get_logger(), "TensorRT input %dx%d, output detections %zu",
      input_w_, input_h_, output_count_ / 6);
    return true;
  }

  bool allocateBuffers()
  {
    const size_t input_bytes = input_count_ * sizeof(float);
    output_bytes_ = output_count_ * sizeof(float);
    if (cudaMalloc(&device_bindings_[input_binding_], input_bytes) != cudaSuccess ||
      cudaMalloc(&device_bindings_[output_binding_], output_bytes_) != cudaSuccess)
    {
      RCLCPP_FATAL(get_logger(), "Failed to allocate TensorRT CUDA buffers");
      return false;
    }
    host_output_.resize(output_count_);
    return true;
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    if (!ready_) {
      return;
    }

    int original_w = 0;
    int original_h = 0;
    cv::cuda::Stream cv_stream = cv::cuda::StreamAccessor::wrapStream(stream_);
    if (!preprocess(msg, cv_stream, original_w, original_h)) {
      return;
    }
    cv_stream.waitForCompletion();

    if (!context_->enqueueV2(device_bindings_.data(), stream_, nullptr)) {
      RCLCPP_WARN(get_logger(), "TensorRT enqueueV2 failed");
      return;
    }

    if (cudaMemcpyAsync(
        host_output_.data(), device_bindings_[output_binding_], output_bytes_,
        cudaMemcpyDeviceToHost, stream_) != cudaSuccess)
    {
      RCLCPP_WARN(get_logger(), "Failed to copy TensorRT output to host");
      return;
    }
    cudaStreamSynchronize(stream_);

    const auto detections = parseDetections(original_w, original_h);
    publishDetections(*msg, detections);
  }

  bool preprocess(
    const sensor_msgs::msg::Image::ConstSharedPtr & msg,
    cv::cuda::Stream & cv_stream,
    int & original_w,
    int & original_h)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
    } catch (const cv_bridge::Exception & ex) {
      RCLCPP_WARN(get_logger(), "Image conversion failed: %s", ex.what());
      return false;
    }

    const cv::Mat & image = cv_ptr->image;
    if (image.empty()) {
      return false;
    }
    original_w = image.cols;
    original_h = image.rows;

    cv::cuda::GpuMat gpu_image;
    gpu_image.upload(image, cv_stream);

    cv::cuda::GpuMat rgb_image;
    const auto & encoding = msg->encoding;
    if (image.channels() == 4) {
      if (encoding == sensor_msgs::image_encodings::RGBA8) {
        cv::cuda::cvtColor(gpu_image, rgb_image, cv::COLOR_RGBA2RGB, 0, cv_stream);
      } else {
        cv::cuda::cvtColor(gpu_image, rgb_image, cv::COLOR_BGRA2RGB, 0, cv_stream);
      }
    } else if (image.channels() == 3) {
      if (encoding == sensor_msgs::image_encodings::RGB8) {
        rgb_image = gpu_image;
      } else {
        cv::cuda::cvtColor(gpu_image, rgb_image, cv::COLOR_BGR2RGB, 0, cv_stream);
      }
    } else if (image.channels() == 1) {
      cv::cuda::cvtColor(gpu_image, rgb_image, cv::COLOR_GRAY2RGB, 0, cv_stream);
    } else {
      RCLCPP_WARN(get_logger(), "Unsupported image channel count: %d", image.channels());
      return false;
    }

    cv::cuda::GpuMat resized;
    cv::cuda::resize(rgb_image, resized, cv::Size(input_w_, input_h_), 0.0, 0.0, cv::INTER_LINEAR, cv_stream);

    cv::cuda::GpuMat normalized;
    resized.convertTo(normalized, CV_32FC3, 1.0 / 255.0, 0.0, cv_stream);

    auto * input = static_cast<float *>(device_bindings_[input_binding_]);
    std::vector<cv::cuda::GpuMat> channels;
    channels.reserve(3);
    for (int c = 0; c < 3; ++c) {
      channels.emplace_back(
        input_h_, input_w_, CV_32FC1,
        input + static_cast<size_t>(c) * input_h_ * input_w_);
    }
    cv::cuda::split(normalized, channels, cv_stream);
    return true;
  }

  std::vector<ParsedDetection> parseDetections(int original_w, int original_h) const
  {
    std::vector<ParsedDetection> detections;
    const size_t count = output_count_ / 6;
    detections.reserve(count);

    for (size_t i = 0; i < count; ++i) {
      const float * row = host_output_.data() + i * 6;
      const float confidence = row[4];
      if (confidence < confidence_threshold_) {
        continue;
      }

      const int class_id = static_cast<int>(std::lround(row[5]));
      if (class_id < 0 || class_id > 2) {
        continue;
      }

      float cx = row[0];
      float cy = row[1];
      float w = row[2];
      float h = row[3];
      if (w <= 0.0F || h <= 0.0F) {
        continue;
      }

      const bool normalized =
        std::abs(cx) <= 2.0F && std::abs(cy) <= 2.0F && w <= 2.0F && h <= 2.0F;
      if (normalized) {
        cx *= static_cast<float>(original_w);
        w *= static_cast<float>(original_w);
        cy *= static_cast<float>(original_h);
        h *= static_cast<float>(original_h);
      } else {
        const float scale_x = static_cast<float>(original_w) / static_cast<float>(input_w_);
        const float scale_y = static_cast<float>(original_h) / static_cast<float>(input_h_);
        cx *= scale_x;
        w *= scale_x;
        cy *= scale_y;
        h *= scale_y;
      }

      cx = std::clamp(cx, 0.0F, static_cast<float>(std::max(0, original_w - 1)));
      cy = std::clamp(cy, 0.0F, static_cast<float>(std::max(0, original_h - 1)));
      w = std::clamp(w, 1.0F, static_cast<float>(original_w));
      h = std::clamp(h, 1.0F, static_cast<float>(original_h));

      detections.push_back({cx, cy, w, h, confidence, class_id});
    }

    return nonMaximumSuppression(std::move(detections), nms_iou_threshold_);
  }

  void publishDetections(
    const sensor_msgs::msg::Image & image_msg,
    const std::vector<ParsedDetection> & detections)
  {
    vision_msgs::msg::Detection2DArray msg;
    msg.header = image_msg.header;

    for (const auto & parsed : detections) {
      vision_msgs::msg::Detection2D detection;
      detection.header = image_msg.header;
      detection.bbox.center.position.x = parsed.cx;
      detection.bbox.center.position.y = parsed.cy;
      detection.bbox.center.theta = 0.0;
      detection.bbox.size_x = parsed.w;
      detection.bbox.size_y = parsed.h;

      vision_msgs::msg::ObjectHypothesisWithPose result;
      result.hypothesis.class_id = std::to_string(parsed.class_id);
      result.hypothesis.score = parsed.confidence;
      detection.results.push_back(result);
      msg.detections.push_back(detection);
    }

    detections_pub_->publish(msg);
  }

  TrtLogger trt_logger_;
  nvinfer1::IRuntime * runtime_{nullptr};
  nvinfer1::ICudaEngine * engine_{nullptr};
  nvinfer1::IExecutionContext * context_{nullptr};
  cudaStream_t stream_{nullptr};

  bool use_sim_{false};
  bool ready_{false};
  float confidence_threshold_{0.5F};
  float nms_iou_threshold_{0.45F};
  int input_w_{640};
  int input_h_{640};
  int output_num_detections_fallback_{25200};
  int input_binding_{-1};
  int output_binding_{-1};
  size_t input_count_{0};
  size_t output_count_{0};
  size_t output_bytes_{0};
  std::vector<void *> device_bindings_;
  std::vector<float> host_output_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConeDetectorNode>());
  rclcpp::shutdown();
  return 0;
}

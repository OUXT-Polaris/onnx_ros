#include "onnx_ros/detic_component.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace onnx_ros
{

DeticComponent::DeticComponent(const rclcpp::NodeOptions & options)
: Node("detic_node", options),
  env_(ORT_LOGGING_LEVEL_WARNING, ""),
  session_options_([]() {
    auto session_options = Ort::SessionOptions();
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    return session_options;
  }()),
  session_(
    env_,
    std::string(
      ament_index_cpp::get_package_share_directory("onnx_ros") +
      "/models/Detic_C2_SwinB_896_4x_IN-21K+COCO_lvis_op16.onnx")
      .c_str(),
    session_options_),
  run_options_(Ort::RunOptions())
{
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "image_raw", 1, [this](const sensor_msgs::msg::Image::SharedPtr image) { callback(image); });
}

DeticComponent::~DeticComponent() {}

void DeticComponent::callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  const cv::Mat image = cv_bridge::toCvShare(msg)->image;
  auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeCPU);
  std::vector<float> vec;
  image.reshape(1, 1).convertTo(vec, CV_32FC1, 1. / 255);
  constexpr int64_t channels = 3;
  constexpr int64_t width = 800;
  constexpr int64_t height = 800;
  // HWC -> CHW
  std::vector<float> input_image_tensor;
  for (size_t ch = 0; ch < 3; ++ch) {
    for (size_t i = ch; i < vec.size(); i += 3) {
      input_image_tensor.emplace_back(vec[i]);
    }
  }
  const std::array<int64_t, 4> input_shape = {1, channels, height, width};
  std::array<float, channels * width * height> input;
  std::copy(input_image_tensor.begin(), input_image_tensor.end(), input.begin());
  const std::array<const char *, 2> input_names = {"img", "im_hw"};
  const std::array<const char *, 4> output_names = {"boxes", "scores", "classes", "masks"};
  std::array<int64_t, 2> input_hw = {800, 800};
  const std::array<int64_t, 2> input_hw_shape = {1, 2};
  // Ort::Value::CreateTensor<float>(
  //   memory_info, input_hw.data(), input_hw.size(), input_hw_shape.data(), input_hw_shape.size());

  // try {
  //   session.Run(
  //     runOptions, input_names.data(), &inputTensor, 1, output_names.data(), &outputTensor, 1);
  // } catch (Ort::Exception & e) {
  //   RCLCPP_ERROR_STREAM(get_logger(), e.what());
  //   return;
  // }
}

}  // namespace onnx_ros

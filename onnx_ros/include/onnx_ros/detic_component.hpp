#ifndef ONNX_ROS__DETIC_COMPONENT_HPP_
#define ONNX_ROS__DETIC_COMPONENT_HPP_

#include <cv_bridge/cv_bridge.h>
#include <onnxruntime_cxx_api.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "onnx_ros/visibility_control.h"

namespace onnx_ros
{

class DeticComponent : public rclcpp::Node
{
public:
  explicit DeticComponent(const rclcpp::NodeOptions & options);

  virtual ~DeticComponent();

private:
  Ort::Env env_;
  const Ort::SessionOptions session_options_;
  const Ort::Session session_;
  const Ort::RunOptions run_options_;
  void callback(const sensor_msgs::msg::Image::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

}  // namespace onnx_ros

#endif  // ONNX_ROS__DETIC_COMPONENT_HPP_

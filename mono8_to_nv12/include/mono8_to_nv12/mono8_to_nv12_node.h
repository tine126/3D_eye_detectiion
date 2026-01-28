// Copyright (c) 2024, TCL.
// Licensed under the Apache License, Version 2.0.

#ifndef MONO8_TO_NV12__MONO8_TO_NV12_NODE_H_
#define MONO8_TO_NV12__MONO8_TO_NV12_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <memory>
#include <chrono>
#include <atomic>

class Mono8ToNv12Node : public rclcpp::Node {
 public:
  explicit Mono8ToNv12Node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~Mono8ToNv12Node() override = default;

 private:
  void LeftImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void RightImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  sensor_msgs::msg::Image::SharedPtr ConvertMono8ToNv12(
      const sensor_msgs::msg::Image::ConstSharedPtr& mono8_msg,
      double& convert_time_ms);

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_sub_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub_;

  // Parameters
  std::string left_input_topic_;
  std::string right_input_topic_;
  std::string left_output_topic_;
  std::string right_output_topic_;

  // Timing parameters
  int timing_log_interval_ = 30;
  std::atomic<int> left_frame_count_{0};
  std::atomic<int> right_frame_count_{0};
};

#endif  // MONO8_TO_NV12__MONO8_TO_NV12_NODE_H_

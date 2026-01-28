// Copyright (c) 2024, TCL.
// Licensed under the Apache License, Version 2.0.

#include "mono8_to_nv12/mono8_to_nv12_node.h"
#include <cstring>

Mono8ToNv12Node::Mono8ToNv12Node(const rclcpp::NodeOptions& options)
    : Node("mono8_to_nv12_node", options) {
  // Declare parameters
  this->declare_parameter<std::string>("left_input_topic", "/left_ir/image_raw");
  this->declare_parameter<std::string>("right_input_topic", "/right_ir/image_raw");
  this->declare_parameter<std::string>("left_output_topic", "/image_left");
  this->declare_parameter<std::string>("right_output_topic", "/image_right");
  this->declare_parameter<int>("timing_log_interval", 30);

  // Get parameters
  this->get_parameter("left_input_topic", left_input_topic_);
  this->get_parameter("right_input_topic", right_input_topic_);
  this->get_parameter("left_output_topic", left_output_topic_);
  this->get_parameter("right_output_topic", right_output_topic_);
  this->get_parameter("timing_log_interval", timing_log_interval_);

  // Create publishers
  left_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      left_output_topic_, 10);
  right_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      right_output_topic_, 10);

  // Create subscribers
  left_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      left_input_topic_, 10,
      std::bind(&Mono8ToNv12Node::LeftImageCallback, this, std::placeholders::_1));
  right_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      right_input_topic_, 10,
      std::bind(&Mono8ToNv12Node::RightImageCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Mono8ToNv12Node initialized");
  RCLCPP_INFO(this->get_logger(), "  Left: %s -> %s",
      left_input_topic_.c_str(), left_output_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Right: %s -> %s",
      right_input_topic_.c_str(), right_output_topic_.c_str());
}

void Mono8ToNv12Node::LeftImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  auto callback_start = std::chrono::steady_clock::now();

  // Calculate receive delay
  auto msg_time = rclcpp::Time(msg->header.stamp);
  auto now_time = this->now();
  double recv_delay_ms = (now_time - msg_time).seconds() * 1000.0;

  // Convert image
  double convert_time_ms = 0;
  auto nv12_msg = ConvertMono8ToNv12(msg, convert_time_ms);

  if (nv12_msg) {
    left_pub_->publish(*nv12_msg);
  }

  // Calculate total time
  auto callback_end = std::chrono::steady_clock::now();
  double total_time_ms = std::chrono::duration_cast<std::chrono::microseconds>(
      callback_end - callback_start).count() / 1000.0;

  // Log timing
  int count = ++left_frame_count_;
  if (timing_log_interval_ > 0 && count % timing_log_interval_ == 0) {
    RCLCPP_INFO(this->get_logger(),
        "[mono8_to_nv12] LEFT  recv=%.2fms convert=%.2fms total=%.2fms",
        recv_delay_ms, convert_time_ms, total_time_ms);
  }
}

void Mono8ToNv12Node::RightImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  auto callback_start = std::chrono::steady_clock::now();

  // Calculate receive delay
  auto msg_time = rclcpp::Time(msg->header.stamp);
  auto now_time = this->now();
  double recv_delay_ms = (now_time - msg_time).seconds() * 1000.0;

  // Convert image
  double convert_time_ms = 0;
  auto nv12_msg = ConvertMono8ToNv12(msg, convert_time_ms);

  if (nv12_msg) {
    right_pub_->publish(*nv12_msg);
  }

  // Calculate total time
  auto callback_end = std::chrono::steady_clock::now();
  double total_time_ms = std::chrono::duration_cast<std::chrono::microseconds>(
      callback_end - callback_start).count() / 1000.0;

  // Log timing
  int count = ++right_frame_count_;
  if (timing_log_interval_ > 0 && count % timing_log_interval_ == 0) {
    RCLCPP_INFO(this->get_logger(),
        "[mono8_to_nv12] RIGHT recv=%.2fms convert=%.2fms total=%.2fms",
        recv_delay_ms, convert_time_ms, total_time_ms);
  }
}

sensor_msgs::msg::Image::SharedPtr Mono8ToNv12Node::ConvertMono8ToNv12(
    const sensor_msgs::msg::Image::ConstSharedPtr& mono8_msg,
    double& convert_time_ms) {
  auto convert_start = std::chrono::steady_clock::now();

  if (mono8_msg->encoding != "mono8") {
    RCLCPP_WARN(this->get_logger(), "Expected mono8 encoding, got %s",
        mono8_msg->encoding.c_str());
    convert_time_ms = 0;
    return nullptr;
  }

  int width = mono8_msg->width;
  int height = mono8_msg->height;

  auto nv12_msg = std::make_shared<sensor_msgs::msg::Image>();
  nv12_msg->header = mono8_msg->header;
  nv12_msg->width = width;
  nv12_msg->height = height;
  nv12_msg->encoding = "nv12";
  nv12_msg->is_bigendian = mono8_msg->is_bigendian;
  nv12_msg->step = width;

  // NV12 format: Y plane (width * height) + UV plane (width * height / 2)
  size_t y_size = width * height;
  size_t uv_size = y_size / 2;
  nv12_msg->data.resize(y_size + uv_size);

  // Y plane: copy mono8 data directly
  std::memcpy(nv12_msg->data.data(), mono8_msg->data.data(), y_size);

  // UV plane: fill with 128 (neutral chrominance)
  std::memset(nv12_msg->data.data() + y_size, 128, uv_size);

  auto convert_end = std::chrono::steady_clock::now();
  convert_time_ms = std::chrono::duration_cast<std::chrono::microseconds>(
      convert_end - convert_start).count() / 1000.0;

  return nv12_msg;
}

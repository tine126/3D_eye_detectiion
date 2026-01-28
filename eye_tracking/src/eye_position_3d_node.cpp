// Copyright (c) 2024, TCL.
// Licensed under the Apache License, Version 2.0.

#include "eye_tracking/eye_position_3d_node.h"
#include <fstream>
#include <sstream>
#include <regex>

EyePosition3DNode::EyePosition3DNode(const rclcpp::NodeOptions& options)
    : Node("eye_position_3d_node", options),
      left_sub_(this, "/eye_positions_left"),
      right_sub_(this, "/eye_positions_right") {
  // Declare parameters
  this->declare_parameter<std::string>("left_input_topic", "/eye_positions_left");
  this->declare_parameter<std::string>("right_input_topic", "/eye_positions_right");
  this->declare_parameter<std::string>("output_topic", "/eye_positions_3d");
  this->declare_parameter<std::string>("calibration_file", "");

  // Get parameters
  this->get_parameter("left_input_topic", left_input_topic_);
  this->get_parameter("right_input_topic", right_input_topic_);
  this->get_parameter("output_topic", output_topic_);
  this->get_parameter("calibration_file", calibration_file_);

  // Load calibration if provided
  if (!calibration_file_.empty()) {
    LoadCalibration(calibration_file_);
  }

  // Create publisher
  pub_ = this->create_publisher<eye_tracking_msgs::msg::EyePositions3D>(
      output_topic_, 10);

  // Setup synchronizer
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), left_sub_, right_sub_);
  sync_->registerCallback(
      std::bind(&EyePosition3DNode::SyncCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "EyePosition3DNode initialized");
  RCLCPP_INFO(this->get_logger(), "  fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f, baseline=%.3f",
      fx_, fy_, cx_, cy_, baseline_);
}

void EyePosition3DNode::LoadCalibration(const std::string& config_path) {
  try {
    std::ifstream file(config_path);
    if (!file.is_open()) {
      RCLCPP_WARN(this->get_logger(), "Cannot open calibration file: %s", config_path.c_str());
      return;
    }

    // Read entire file content
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string content = buffer.str();

    // Simple regex-based JSON parsing
    auto parse_double = [&content](const std::string& key, double default_val) -> double {
      std::regex pattern("\"" + key + "\"\\s*:\\s*([0-9.\\-]+)");
      std::smatch match;
      if (std::regex_search(content, match, pattern) && match.size() > 1) {
        return std::stod(match[1].str());
      }
      return default_val;
    };

    fx_ = parse_double("fx", fx_);
    fy_ = parse_double("fy", fy_);
    cx_ = parse_double("cx", cx_);
    cy_ = parse_double("cy", cy_);
    baseline_ = parse_double("baseline", baseline_);

    RCLCPP_INFO(this->get_logger(), "Loaded calibration from %s", config_path.c_str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load calibration: %s", e.what());
  }
}

void EyePosition3DNode::SyncCallback(
    const eye_tracking_msgs::msg::EyePositions2D::ConstSharedPtr& left_msg,
    const eye_tracking_msgs::msg::EyePositions2D::ConstSharedPtr& right_msg) {
  eye_tracking_msgs::msg::EyePositions3D result;
  result.header = left_msg->header;
  result.track_id = left_msg->track_id;
  result.left_eye.header = left_msg->header;
  result.right_eye.header = left_msg->header;
  result.left_eye.valid = false;
  result.right_eye.valid = false;

  // Calculate 3D position for left eye
  if (left_msg->left_eye.valid && right_msg->left_eye.valid) {
    float disparity = left_msg->left_eye.x - right_msg->left_eye.x;
    result.disparity_left = disparity;
    if (disparity > 0.1) {
      double Z = fx_ * baseline_ / disparity;
      double X = (left_msg->left_eye.x - cx_) * Z / fx_;
      double Y = (left_msg->left_eye.y - cy_) * Z / fy_;
      result.left_eye.position.x = X;
      result.left_eye.position.y = Y;
      result.left_eye.position.z = Z;
      result.left_eye.confidence = (left_msg->left_eye.confidence +
                                    right_msg->left_eye.confidence) / 2.0;
      result.left_eye.valid = true;
    }
  }

  // Calculate 3D position for right eye
  if (left_msg->right_eye.valid && right_msg->right_eye.valid) {
    float disparity = left_msg->right_eye.x - right_msg->right_eye.x;
    result.disparity_right = disparity;
    if (disparity > 0.1) {
      double Z = fx_ * baseline_ / disparity;
      double X = (left_msg->right_eye.x - cx_) * Z / fx_;
      double Y = (left_msg->right_eye.y - cy_) * Z / fy_;
      result.right_eye.position.x = X;
      result.right_eye.position.y = Y;
      result.right_eye.position.z = Z;
      result.right_eye.confidence = (left_msg->right_eye.confidence +
                                     right_msg->right_eye.confidence) / 2.0;
      result.right_eye.valid = true;
    }
  }

  pub_->publish(result);
}

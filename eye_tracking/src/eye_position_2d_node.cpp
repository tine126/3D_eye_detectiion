// Copyright (c) 2024, TCL.
// Licensed under the Apache License, Version 2.0.

#include "eye_tracking/eye_position_2d_node.h"

EyePosition2DNode::EyePosition2DNode(const rclcpp::NodeOptions& options)
    : Node("eye_position_2d_node", options) {
  // Declare parameters
  this->declare_parameter<std::string>("left_input_topic", "/face_landmarks_left");
  this->declare_parameter<std::string>("right_input_topic", "/face_landmarks_right");
  this->declare_parameter<std::string>("left_output_topic", "/eye_positions_left");
  this->declare_parameter<std::string>("right_output_topic", "/eye_positions_right");

  // Get parameters
  this->get_parameter("left_input_topic", left_input_topic_);
  this->get_parameter("right_input_topic", right_input_topic_);
  this->get_parameter("left_output_topic", left_output_topic_);
  this->get_parameter("right_output_topic", right_output_topic_);

  // Create publishers
  left_pub_ = this->create_publisher<eye_tracking_msgs::msg::EyePositions2D>(
      left_output_topic_, 10);
  right_pub_ = this->create_publisher<eye_tracking_msgs::msg::EyePositions2D>(
      right_output_topic_, 10);

  // Create subscribers
  left_sub_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
      left_input_topic_, 10,
      std::bind(&EyePosition2DNode::LeftLandmarksCallback, this, std::placeholders::_1));
  right_sub_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
      right_input_topic_, 10,
      std::bind(&EyePosition2DNode::RightLandmarksCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "EyePosition2DNode initialized");
}

void EyePosition2DNode::LeftLandmarksCallback(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg) {
  auto eye_msg = ExtractEyePositions(msg);
  left_pub_->publish(eye_msg);
}

void EyePosition2DNode::RightLandmarksCallback(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg) {
  auto eye_msg = ExtractEyePositions(msg);
  right_pub_->publish(eye_msg);
}

eye_tracking_msgs::msg::EyePositions2D EyePosition2DNode::ExtractEyePositions(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr& msg) {
  eye_tracking_msgs::msg::EyePositions2D result;
  result.header = msg->header;
  result.left_eye.header = msg->header;
  result.right_eye.header = msg->header;
  result.left_eye.valid = false;
  result.right_eye.valid = false;
  result.track_id = -1;

  if (msg->targets.empty()) {
    return result;
  }

  // Use the first detected face
  const auto& target = msg->targets[0];
  result.track_id = target.track_id;

  // Find landmarks attribute
  for (const auto& attr : target.attributes) {
    if (attr.type != "landmarks_106") {
      continue;
    }

    const auto& points = attr.points;
    if (points.size() < 106) {
      RCLCPP_WARN(this->get_logger(), "Insufficient landmarks: %zu", points.size());
      return result;
    }

    // Calculate left eye center (points 33-41)
    float left_x = 0, left_y = 0, left_conf = 0;
    int left_count = kLeftEyeEndIdx - kLeftEyeStartIdx + 1;
    for (int i = kLeftEyeStartIdx; i <= kLeftEyeEndIdx; ++i) {
      left_x += points[i].point.x;
      left_y += points[i].point.y;
      left_conf += points[i].confidence;
    }
    result.left_eye.x = left_x / left_count;
    result.left_eye.y = left_y / left_count;
    result.left_eye.confidence = left_conf / left_count;
    result.left_eye.valid = true;

    // Calculate right eye center (points 42-50)
    float right_x = 0, right_y = 0, right_conf = 0;
    int right_count = kRightEyeEndIdx - kRightEyeStartIdx + 1;
    for (int i = kRightEyeStartIdx; i <= kRightEyeEndIdx; ++i) {
      right_x += points[i].point.x;
      right_y += points[i].point.y;
      right_conf += points[i].confidence;
    }
    result.right_eye.x = right_x / right_count;
    result.right_eye.y = right_y / right_count;
    result.right_eye.confidence = right_conf / right_count;
    result.right_eye.valid = true;

    break;
  }

  return result;
}

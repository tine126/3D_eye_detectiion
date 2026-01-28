// Copyright (c) 2024, TCL.
// Licensed under the Apache License, Version 2.0.

#include "eye_tracking/eye_visualization_node.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

EyeVisualizationNode::EyeVisualizationNode(const rclcpp::NodeOptions& options)
    : Node("eye_visualization_node", options) {
  // Declare parameters
  this->declare_parameter<std::string>("image_topic", "/image_left");
  this->declare_parameter<std::string>("landmarks_topic", "/face_landmarks_left");
  this->declare_parameter<std::string>("eye3d_topic", "/eye_positions_3d");
  this->declare_parameter<std::string>("output_topic", "/visualization");

  // Get parameters
  this->get_parameter("image_topic", image_topic_);
  this->get_parameter("landmarks_topic", landmarks_topic_);
  this->get_parameter("eye3d_topic", eye3d_topic_);
  this->get_parameter("output_topic", output_topic_);

  // Create publisher
  vis_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);

  // Create subscribers
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, 10,
      std::bind(&EyeVisualizationNode::ImageCallback, this, std::placeholders::_1));
  landmarks_sub_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
      landmarks_topic_, 10,
      std::bind(&EyeVisualizationNode::LandmarksCallback, this, std::placeholders::_1));
  eye3d_sub_ = this->create_subscription<eye_tracking_msgs::msg::EyePositions3D>(
      eye3d_topic_, 10,
      std::bind(&EyeVisualizationNode::Eye3DCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "EyeVisualizationNode initialized");
}

void EyeVisualizationNode::ImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_image_ = msg;
  PublishVisualization();
}

void EyeVisualizationNode::LandmarksCallback(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_landmarks_ = msg;
}

void EyeVisualizationNode::Eye3DCallback(
    const eye_tracking_msgs::msg::EyePositions3D::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_eye3d_ = msg;
}

void EyeVisualizationNode::PublishVisualization() {
  if (!last_image_) return;

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(last_image_, "bgr8");
  } catch (const cv_bridge::Exception& e) {
    // Try mono8 conversion
    try {
      cv_ptr = cv_bridge::toCvCopy(last_image_, "mono8");
      cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_GRAY2BGR);
    } catch (const cv_bridge::Exception& e2) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e2.what());
      return;
    }
  }

  cv::Mat& img = cv_ptr->image;

  // Draw face landmarks
  if (last_landmarks_ && !last_landmarks_->targets.empty()) {
    for (const auto& target : last_landmarks_->targets) {
      // Draw face bounding box
      for (const auto& roi : target.rois) {
        if (roi.type == "face") {
          cv::rectangle(img,
              cv::Point(roi.rect.x_offset, roi.rect.y_offset),
              cv::Point(roi.rect.x_offset + roi.rect.width,
                        roi.rect.y_offset + roi.rect.height),
              cv::Scalar(0, 255, 0), 2);
        }
      }

      // Draw landmarks
      for (const auto& pt : target.points) {
        if (pt.type == "face_kps") {
          for (size_t i = 0; i < pt.point.size(); ++i) {
            cv::circle(img,
                cv::Point(pt.point[i].x, pt.point[i].y),
                1, cv::Scalar(255, 0, 0), -1);
          }
        }
      }
    }
  }

  // Draw 3D eye positions
  if (last_eye3d_) {
    char text[128];
    if (last_eye3d_->left_eye.valid) {
      snprintf(text, sizeof(text), "L: (%.2f, %.2f, %.2f)m",
          last_eye3d_->left_eye.position.x,
          last_eye3d_->left_eye.position.y,
          last_eye3d_->left_eye.position.z);
      cv::putText(img, text, cv::Point(10, 30),
          cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
    }
    if (last_eye3d_->right_eye.valid) {
      snprintf(text, sizeof(text), "R: (%.2f, %.2f, %.2f)m",
          last_eye3d_->right_eye.position.x,
          last_eye3d_->right_eye.position.y,
          last_eye3d_->right_eye.position.z);
      cv::putText(img, text, cv::Point(10, 60),
          cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
    }
  }

  cv_ptr->encoding = "bgr8";
  vis_pub_->publish(*cv_ptr->toImageMsg());
}

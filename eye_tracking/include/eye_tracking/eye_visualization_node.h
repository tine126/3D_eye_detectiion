// Copyright (c) 2024, TCL.
// Licensed under the Apache License, Version 2.0.

#ifndef EYE_TRACKING__EYE_VISUALIZATION_NODE_H_
#define EYE_TRACKING__EYE_VISUALIZATION_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ai_msgs/msg/perception_targets.hpp>
#include <eye_tracking_msgs/msg/eye_positions3_d.hpp>
#include <string>
#include <memory>
#include <mutex>

class EyeVisualizationNode : public rclcpp::Node {
 public:
  explicit EyeVisualizationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~EyeVisualizationNode() override = default;

 private:
  void ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void LandmarksCallback(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg);
  void Eye3DCallback(const eye_tracking_msgs::msg::EyePositions3D::ConstSharedPtr msg);

  void PublishVisualization();

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr landmarks_sub_;
  rclcpp::Subscription<eye_tracking_msgs::msg::EyePositions3D>::SharedPtr eye3d_sub_;

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr vis_pub_;

  // Cached data
  sensor_msgs::msg::Image::ConstSharedPtr last_image_;
  ai_msgs::msg::PerceptionTargets::ConstSharedPtr last_landmarks_;
  eye_tracking_msgs::msg::EyePositions3D::ConstSharedPtr last_eye3d_;
  std::mutex data_mutex_;

  // Parameters
  std::string image_topic_;
  std::string landmarks_topic_;
  std::string eye3d_topic_;
  std::string output_topic_;
};

#endif  // EYE_TRACKING__EYE_VISUALIZATION_NODE_H_

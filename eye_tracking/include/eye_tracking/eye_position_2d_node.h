// Copyright (c) 2024, TCL.
// Licensed under the Apache License, Version 2.0.

#ifndef EYE_TRACKING__EYE_POSITION_2D_NODE_H_
#define EYE_TRACKING__EYE_POSITION_2D_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <ai_msgs/msg/perception_targets.hpp>
#include <eye_tracking_msgs/msg/eye_positions2_d.hpp>
#include <string>
#include <memory>

class EyePosition2DNode : public rclcpp::Node {
 public:
  explicit EyePosition2DNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~EyePosition2DNode() override = default;

 private:
  void LeftLandmarksCallback(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg);
  void RightLandmarksCallback(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg);

  eye_tracking_msgs::msg::EyePositions2D ExtractEyePositions(
      const ai_msgs::msg::PerceptionTargets::ConstSharedPtr& msg);

  // Eye landmark indices (106-point model)
  static constexpr int kLeftEyeStartIdx = 33;
  static constexpr int kLeftEyeEndIdx = 41;
  static constexpr int kRightEyeStartIdx = 42;
  static constexpr int kRightEyeEndIdx = 50;

  // Subscribers
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr left_sub_;
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr right_sub_;

  // Publishers
  rclcpp::Publisher<eye_tracking_msgs::msg::EyePositions2D>::SharedPtr left_pub_;
  rclcpp::Publisher<eye_tracking_msgs::msg::EyePositions2D>::SharedPtr right_pub_;

  // Parameters
  std::string left_input_topic_;
  std::string right_input_topic_;
  std::string left_output_topic_;
  std::string right_output_topic_;
};

#endif  // EYE_TRACKING__EYE_POSITION_2D_NODE_H_

// Copyright (c) 2024, TCL.
// Licensed under the Apache License, Version 2.0.

#ifndef EYE_TRACKING__EYE_POSITION_3D_NODE_H_
#define EYE_TRACKING__EYE_POSITION_3D_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <eye_tracking_msgs/msg/eye_positions2_d.hpp>
#include <eye_tracking_msgs/msg/eye_positions3_d.hpp>
#include <string>
#include <memory>

class EyePosition3DNode : public rclcpp::Node {
 public:
  explicit EyePosition3DNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~EyePosition3DNode() override = default;

 private:
  void SyncCallback(
      const eye_tracking_msgs::msg::EyePositions2D::ConstSharedPtr& left_msg,
      const eye_tracking_msgs::msg::EyePositions2D::ConstSharedPtr& right_msg);

  void LoadCalibration(const std::string& config_path);

  // Stereo calibration parameters
  double fx_ = 600.0;
  double fy_ = 600.0;
  double cx_ = 640.0;
  double cy_ = 400.0;
  double baseline_ = 0.05;

  // Sync policy
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      eye_tracking_msgs::msg::EyePositions2D,
      eye_tracking_msgs::msg::EyePositions2D>;

  // Subscribers
  message_filters::Subscriber<eye_tracking_msgs::msg::EyePositions2D> left_sub_;
  message_filters::Subscriber<eye_tracking_msgs::msg::EyePositions2D> right_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // Publisher
  rclcpp::Publisher<eye_tracking_msgs::msg::EyePositions3D>::SharedPtr pub_;

  // Parameters
  std::string left_input_topic_;
  std::string right_input_topic_;
  std::string output_topic_;
  std::string calibration_file_;
};

#endif  // EYE_TRACKING__EYE_POSITION_3D_NODE_H_

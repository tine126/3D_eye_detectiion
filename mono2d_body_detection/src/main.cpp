// Copyright (c) 2024，D-Robotics.
// 精简版：只保留人脸检测功能

#include <memory>
#include "include/mono2d_body_det_node.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  RCLCPP_WARN(rclcpp::get_logger("mono2d_body_det"),
              "Face Detection Node (Lite) - Only face detection with SharedMem+NV12");
  rclcpp::spin(std::make_shared<Mono2dBodyDetNode>());
  rclcpp::shutdown();
  return 0;
}

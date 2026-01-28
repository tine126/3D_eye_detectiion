// Copyright (c) 2024, TCL.
// Licensed under the Apache License, Version 2.0.

#include <rclcpp/rclcpp.hpp>
#include "eye_tracking/eye_position_2d_node.h"
#include "eye_tracking/eye_position_3d_node.h"
#include "eye_tracking/eye_visualization_node.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;

  auto eye_2d_node = std::make_shared<EyePosition2DNode>();
  auto eye_3d_node = std::make_shared<EyePosition3DNode>();
  auto vis_node = std::make_shared<EyeVisualizationNode>();

  executor.add_node(eye_2d_node);
  executor.add_node(eye_3d_node);
  executor.add_node(vis_node);

  executor.spin();
  rclcpp::shutdown();
  return 0;
}

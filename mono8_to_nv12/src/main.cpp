// Copyright (c) 2024, TCL.
// Licensed under the Apache License, Version 2.0.

#include "mono8_to_nv12/mono8_to_nv12_node.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Mono8ToNv12Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

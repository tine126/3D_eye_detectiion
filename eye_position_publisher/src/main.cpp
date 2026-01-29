// Copyright (c) 2024，D-Robotics.
// eye_position_publisher: 从人脸关键点提取眼睛中心坐标

#include <memory>
#include "eye_position_publisher_node.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_WARN(rclcpp::get_logger("eye_position_publisher"),
                "Eye Position Publisher - extract eye center from face landmarks");
    rclcpp::spin(std::make_shared<EyePositionPublisherNode>());
    rclcpp::shutdown();
    return 0;
}

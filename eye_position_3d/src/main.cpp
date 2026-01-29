// Copyright (c) 2024，D-Robotics.
// eye_position_3d: 双目视觉三角测量计算三维眼睛坐标

#include <memory>
#include "eye_position_3d_node.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_WARN(rclcpp::get_logger("eye_position_3d"),
                "Eye Position 3D - stereo vision triangulation");
    rclcpp::spin(std::make_shared<EyePosition3DNode>());
    rclcpp::shutdown();
    return 0;
}

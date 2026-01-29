// Copyright (c) 2024，D-Robotics.
// 精简版：仅保留在线模式，SharedMem+NV12输入

#include <memory>
#include "face_landmarks_det_node.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_WARN(rclcpp::get_logger("face_landmarks_det"),
                "Face Landmarks Detection (Lite) - Online mode only, SharedMem+NV12");
    rclcpp::spin(std::make_shared<FaceLandmarksDetNode>());
    rclcpp::shutdown();
    return 0;
}

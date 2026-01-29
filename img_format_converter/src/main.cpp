// Copyright (c) 2024，D-Robotics.
// img_format_converter: mono8 -> nv12 格式转换节点

#include <memory>
#include "img_format_converter_node.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_WARN(rclcpp::get_logger("img_format_converter"),
                "Image Format Converter - mono8 to nv12 via SharedMem");
    rclcpp::spin(std::make_shared<ImgFormatConverterNode>());
    rclcpp::shutdown();
    return 0;
}

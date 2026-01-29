# Copyright (c) 2024，D-Robotics.
# img_format_converter: 双路IR mono8 -> nv12 格式转换节点

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    # 参数定义
    params = [
        # 左IR
        {"name": "left_sub_topic", "default": "/camera/left_ir/image_raw", "desc": "左IR输入topic"},
        {"name": "left_pub_topic", "default": "/hbmem_img_left", "desc": "左IR输出topic"},
        # 右IR
        {"name": "right_sub_topic", "default": "/camera/right_ir/image_raw", "desc": "右IR输入topic"},
        {"name": "right_pub_topic", "default": "/hbmem_img_right", "desc": "右IR输出topic"},
        # 图像配置
        {"name": "image_width", "default": "1280", "desc": "图像宽度"},
        {"name": "image_height", "default": "800", "desc": "图像高度"},
        {"name": "log_level", "default": "warn", "desc": "日志级别"},
    ]

    # 声明参数
    launch_args = [
        DeclareLaunchArgument(p["name"], default_value=p["default"], description=p["desc"])
        for p in params
    ]

    # 节点配置
    node_params = {
        p["name"]: LaunchConfiguration(p["name"])
        for p in params if p["name"] != "log_level"
    }

    node = Node(
        package="img_format_converter",
        executable="img_format_converter",
        output="screen",
        parameters=[node_params],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    return LaunchDescription(launch_args + [node])

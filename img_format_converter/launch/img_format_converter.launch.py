# Copyright (c) 2024，D-Robotics.
# img_format_converter: mono8 -> nv12 格式转换节点

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    # 参数定义
    params = [
        {"name": "sub_topic_name", "default": "/camera/ir/image_raw", "desc": "Input topic (mono8)"},
        {"name": "pub_topic_name", "default": "/hbmem_img", "desc": "Output topic (nv12)"},
        {"name": "image_width", "default": "1280", "desc": "Image width"},
        {"name": "image_height", "default": "800", "desc": "Image height"},
        {"name": "log_level", "default": "warn", "desc": "Log level"},
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

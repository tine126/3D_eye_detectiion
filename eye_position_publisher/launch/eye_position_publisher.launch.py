# Copyright (c) 2024，D-Robotics.
# eye_position_publisher: 从人脸关键点提取眼睛中心坐标

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    # 参数定义
    params = [
        # 左IR
        {"name": "eye_left_sub_topic", "default": "/face_landmarks_detection_left", "desc": "左IR关键点输入"},
        {"name": "eye_left_pub_topic", "default": "/eye_positions_left", "desc": "左IR眼睛位置输出"},
        # 右IR
        {"name": "eye_right_sub_topic", "default": "/face_landmarks_detection_right", "desc": "右IR关键点输入"},
        {"name": "eye_right_pub_topic", "default": "/eye_positions_right", "desc": "右IR眼睛位置输出"},
        # 日志
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
        package="eye_position_publisher",
        executable="eye_position_publisher_node",
        output="screen",
        parameters=[node_params],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    return LaunchDescription(launch_args + [node])

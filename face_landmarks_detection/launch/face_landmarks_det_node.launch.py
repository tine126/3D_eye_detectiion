# Copyright (c) 2024，D-Robotics.
# 精简版：仅保留在线模式，SharedMem+NV12输入

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    model_file_name = os.path.join(
        get_package_share_directory("face_landmarks_detection"),
        "config",
        "faceLandmark106pts.hbm",
    )

    # 参数定义
    params = [
        {"name": "is_sync_mode", "default": "0", "desc": "0=async, 1=sync"},
        {"name": "model_file_name", "default": model_file_name, "desc": "model path"},
        {"name": "score_threshold", "default": "0.5", "desc": "upstream face det threshold"},
        {"name": "expand_scale", "default": "1.25", "desc": "ROI expand scale"},
        {"name": "roi_size_min", "default": "16", "desc": "min ROI size"},
        {"name": "roi_size_max", "default": "255", "desc": "max ROI size"},
        {"name": "cache_len_limit", "default": "8", "desc": "image cache limit"},
        {"name": "ai_msg_timeout_ms", "default": "200", "desc": "AI msg match timeout"},
        {"name": "sharedmem_img_topic_name", "default": "/hbmem_img", "desc": "image topic"},
        {"name": "ai_msg_sub_topic_name", "default": "/hobot_mono2d_body_detection", "desc": "AI msg sub topic"},
        {"name": "ai_msg_pub_topic_name", "default": "/face_landmarks_detection", "desc": "AI msg pub topic"},
        {"name": "log_level", "default": "warn", "desc": "log level"},
    ]

    # 声明参数
    launch_args = [
        DeclareLaunchArgument(p["name"], default_value=p["default"], description=p["desc"])
        for p in params
    ]

    # 节点配置
    node_params = {p["name"]: LaunchConfiguration(p["name"]) for p in params if p["name"] != "log_level"}

    node = Node(
        package="face_landmarks_detection",
        executable="face_landmarks_detection",
        output="screen",
        parameters=[node_params],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    return LaunchDescription(launch_args + [node])

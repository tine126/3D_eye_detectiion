# Copyright (c) 2024，D-Robotics.
# 人脸关键点检测：双路SharedMem+NV12输入

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    landmarks_model_file_name = os.path.join(
        get_package_share_directory("face_landmarks_detection"),
        "config",
        "faceLandmark106pts.hbm",
    )

    # 参数定义
    params = [
        {"name": "is_sync_mode", "default": "1", "desc": "0=异步, 1=同步"},
        {"name": "landmarks_model_file_name", "default": landmarks_model_file_name, "desc": "模型路径"},
        {"name": "score_threshold", "default": "0.5", "desc": "人脸检测置信度阈值"},
        {"name": "expand_scale", "default": "1.1", "desc": "ROI扩展比例"},
        {"name": "roi_size_min", "default": "16", "desc": "最小ROI尺寸"},
        {"name": "roi_size_max", "default": "255", "desc": "最大ROI尺寸"},
        {"name": "cache_len_limit", "default": "8", "desc": "图像缓存上限"},
        {"name": "ai_msg_timeout_ms", "default": "200", "desc": "AI消息匹配超时"},
        # 双路topic配置
        {"name": "left_img_topic", "default": "/hbmem_img_left", "desc": "左IR图像topic"},
        {"name": "left_ai_sub_topic", "default": "/hobot_mono2d_body_detection_left", "desc": "左IR AI订阅topic"},
        {"name": "left_ai_pub_topic", "default": "/face_landmarks_detection_left", "desc": "左IR AI发布topic"},
        {"name": "right_img_topic", "default": "/hbmem_img_right", "desc": "右IR图像topic"},
        {"name": "right_ai_sub_topic", "default": "/hobot_mono2d_body_detection_right", "desc": "右IR AI订阅topic"},
        {"name": "right_ai_pub_topic", "default": "/face_landmarks_detection_right", "desc": "右IR AI发布topic"},
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
        package="face_landmarks_detection",
        executable="face_landmarks_detection",
        output="screen",
        parameters=[node_params],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    return LaunchDescription(launch_args + [node])

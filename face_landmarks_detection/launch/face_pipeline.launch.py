# Copyright (c) 2024，D-Robotics.
# 人脸检测+关键点检测 统一启动文件
# 一处配置，两处生效

import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取模型路径
    body_model = os.path.join(
        get_package_share_directory("mono2d_body_detection"),
        "config", "multitask_body_head_face_hand_kps_960x544.hbm"
    )
    landmarks_model = os.path.join(
        get_package_share_directory("face_landmarks_detection"),
        "config", "faceLandmark106pts.hbm"
    )

    # ========== 共享参数 (两节点必须一致) ==========
    shared_params = [
        {"name": "sharedmem_img_topic_name", "default": "/hbmem_img",
         "desc": "SharedMem image topic (shared)"},
        {"name": "is_sync_mode", "default": "0",
         "desc": "Inference mode: 0=async, 1=sync (shared)"},
        {"name": "score_threshold", "default": "0.5",
         "desc": "Face detection confidence threshold (shared)"},
    ]

    # ========== Pipeline链路参数 ==========
    pipeline_params = [
        {"name": "face_det_topic", "default": "/hobot_mono2d_body_detection",
         "desc": "Face detection output topic (body pub = face sub)"},
        {"name": "landmarks_topic", "default": "/face_landmarks_detection",
         "desc": "Landmarks output topic"},
    ]

    # ========== face_landmarks专用参数 ==========
    landmarks_params = [
        {"name": "expand_scale", "default": "1.25", "desc": "ROI expand scale"},
        {"name": "roi_size_min", "default": "16", "desc": "Min ROI size"},
        {"name": "roi_size_max", "default": "255", "desc": "Max ROI size"},
        {"name": "cache_len_limit", "default": "8", "desc": "Image cache limit"},
        {"name": "ai_msg_timeout_ms", "default": "200", "desc": "AI msg match timeout"},
    ]

    # ========== 其他参数 ==========
    other_params = [
        {"name": "log_level", "default": "warn", "desc": "Log level"},
    ]

    all_params = shared_params + pipeline_params + landmarks_params + other_params

    # 声明所有参数
    launch_args = [
        DeclareLaunchArgument(p["name"], default_value=p["default"], description=p["desc"])
        for p in all_params
    ]

    # ========== mono2d_body_detection 节点 ==========
    body_det_node = Node(
        package="mono2d_body_detection",
        executable="mono2d_body_detection",
        name="mono2d_body_det",
        output="screen",
        parameters=[{
            "model_file_name": body_model,
            "is_sync_mode": LaunchConfiguration("is_sync_mode"),
            "score_threshold": LaunchConfiguration("score_threshold"),
            "sharedmem_img_topic_name": LaunchConfiguration("sharedmem_img_topic_name"),
            "ai_msg_pub_topic_name": LaunchConfiguration("face_det_topic"),
        }],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    # ========== face_landmarks_detection 节点 ==========
    landmarks_det_node = Node(
        package="face_landmarks_detection",
        executable="face_landmarks_detection",
        name="face_landmarks_det",
        output="screen",
        parameters=[{
            "model_file_name": landmarks_model,
            "is_sync_mode": LaunchConfiguration("is_sync_mode"),
            "score_threshold": LaunchConfiguration("score_threshold"),
            "sharedmem_img_topic_name": LaunchConfiguration("sharedmem_img_topic_name"),
            "ai_msg_sub_topic_name": LaunchConfiguration("face_det_topic"),
            "ai_msg_pub_topic_name": LaunchConfiguration("landmarks_topic"),
            "expand_scale": LaunchConfiguration("expand_scale"),
            "roi_size_min": LaunchConfiguration("roi_size_min"),
            "roi_size_max": LaunchConfiguration("roi_size_max"),
            "cache_len_limit": LaunchConfiguration("cache_len_limit"),
            "ai_msg_timeout_ms": LaunchConfiguration("ai_msg_timeout_ms"),
        }],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    return LaunchDescription(launch_args + [body_det_node, landmarks_det_node])

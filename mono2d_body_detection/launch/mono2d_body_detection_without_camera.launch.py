# Copyright (c) 2024，D-Robotics.
# 精简版：只保留人脸检测功能，支持双路SharedMem+NV12输入

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_prefix


def generate_launch_description():
    # 获取模型文件绝对路径
    model_file_path = os.path.join(
        get_package_prefix("mono2d_body_detection"),
        "lib", "mono2d_body_detection", "config",
        "multitask_body_head_face_hand_kps_960x544.hbm"
    )

    # 模型文件参数
    model_file_name_arg = DeclareLaunchArgument(
        "model_file_name",
        default_value=model_file_path
    )
    # 同步/异步推理模式
    is_sync_mode_arg = DeclareLaunchArgument(
        "is_sync_mode", default_value="0"
    )
    # 人脸检测置信度阈值
    score_threshold_arg = DeclareLaunchArgument(
        "score_threshold", default_value="0.5"
    )
    # 双路topic配置
    left_img_topic_arg = DeclareLaunchArgument(
        "left_img_topic", default_value="/hbmem_img_left"
    )
    left_pub_topic_arg = DeclareLaunchArgument(
        "left_pub_topic", default_value="/hobot_mono2d_body_detection_left"
    )
    right_img_topic_arg = DeclareLaunchArgument(
        "right_img_topic", default_value="/hbmem_img_right"
    )
    right_pub_topic_arg = DeclareLaunchArgument(
        "right_pub_topic", default_value="/hobot_mono2d_body_detection_right"
    )
    # 日志级别
    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value="warn"
    )

    # 人脸检测节点 (双路)
    mono2d_body_det_node = Node(
        package='mono2d_body_detection',
        executable='mono2d_body_detection',
        output='screen',
        parameters=[
            {"model_file_name": LaunchConfiguration('model_file_name')},
            {"is_sync_mode": LaunchConfiguration('is_sync_mode')},
            {"score_threshold": LaunchConfiguration('score_threshold')},
            {"left_img_topic": LaunchConfiguration('left_img_topic')},
            {"left_pub_topic": LaunchConfiguration('left_pub_topic')},
            {"right_img_topic": LaunchConfiguration('right_img_topic')},
            {"right_pub_topic": LaunchConfiguration('right_pub_topic')},
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )

    return LaunchDescription([
        model_file_name_arg,
        is_sync_mode_arg,
        score_threshold_arg,
        left_img_topic_arg,
        left_pub_topic_arg,
        right_img_topic_arg,
        right_pub_topic_arg,
        log_level_arg,
        mono2d_body_det_node,
    ])

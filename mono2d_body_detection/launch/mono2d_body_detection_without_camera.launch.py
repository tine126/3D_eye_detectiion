# Copyright (c) 2024，D-Robotics.
# 精简版：只保留人脸检测功能，仅支持SharedMem+NV12输入
# 无相机版本：需要外部提供SharedMem图像

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 模型文件参数
    model_file_name_launch_arg = DeclareLaunchArgument(
        "model_file_name",
        default_value=TextSubstitution(text="config/multitask_body_head_face_hand_kps_960x544.hbm")
    )
    # 同步/异步推理模式: 0=异步(默认), 1=同步
    is_sync_mode_launch_arg = DeclareLaunchArgument(
        "is_sync_mode", default_value=TextSubstitution(text="0")
    )
    # 人脸检测置信度阈值
    score_threshold_launch_arg = DeclareLaunchArgument(
        "score_threshold", default_value=TextSubstitution(text="0.5")
    )

    # nv12->jpeg (用于web显示)
    jpeg_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_encode.launch.py')),
        launch_arguments={
            'codec_in_mode': 'shared_mem',
            'codec_out_mode': 'ros',
            'codec_sub_topic': '/hbmem_img',
            'codec_pub_topic': '/image'
        }.items()
    )

    # web显示
    web_smart_topic_arg = DeclareLaunchArgument(
        'smart_topic',
        default_value='/hobot_mono2d_body_detection',
        description='websocket smart topic')
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/image',
            'websocket_smart_topic': LaunchConfiguration('smart_topic')
        }.items()
    )

    # 人脸检测节点 (精简版)
    mono2d_body_pub_topic_arg = DeclareLaunchArgument(
        'mono2d_body_pub_topic',
        default_value='/hobot_mono2d_body_detection',
        description='face detection ai message publish topic')
    mono2d_body_det_node = Node(
        package='mono2d_body_detection',
        executable='mono2d_body_detection',
        output='screen',
        parameters=[
            {"model_file_name": LaunchConfiguration('model_file_name')},
            {"is_sync_mode": LaunchConfiguration('is_sync_mode')},
            {"score_threshold": LaunchConfiguration('score_threshold')},
            {"ai_msg_pub_topic_name": LaunchConfiguration('mono2d_body_pub_topic')}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    return LaunchDescription([
        model_file_name_launch_arg,
        is_sync_mode_launch_arg,
        score_threshold_launch_arg,
        # image codec (for web display)
        jpeg_codec_node,
        # face detection
        mono2d_body_pub_topic_arg,
        mono2d_body_det_node,
        # web display
        web_smart_topic_arg,
        web_node
    ])

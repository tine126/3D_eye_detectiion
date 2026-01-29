# Copyright (c) 2024，D-Robotics.
# 精简版：只保留人脸检测功能，仅支持SharedMem+NV12输入

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

    camera_type = os.getenv('CAM_TYPE')
    print("camera_type is ", camera_type)

    camera_node = None
    camera_type_mipi = None
    camera_device_arg = None

    if camera_type == "fb":
        print("using feedback - converting to SharedMem")
        # local image publish (需要转换为SharedMem)
        fb_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('hobot_image_publisher'),
                    'launch/hobot_image_publisher.launch.py')),
            launch_arguments={
                'publish_message_topic_name': '/image',
                'publish_is_shared_mem': 'False',
                'publish_is_compressed_img_pub': 'True'
            }.items()
        )
        camera_node = fb_node
        camera_type_mipi = False
    elif camera_type == "usb":
        print("using usb camera - converting to SharedMem")
        # using usb cam publish image (需要转换为SharedMem)
        usb_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('hobot_usb_cam'),
                    'launch/hobot_usb_cam.launch.py')),
            launch_arguments={
                'usb_image_width': '640',
                'usb_image_height': '480'
            }.items()
        )
        camera_node = usb_node
        camera_type_mipi = False
    else:
        print("using mipi cam (SharedMem native)")
        # using mipi cam publish image (原生SharedMem支持)
        mipi_cam_device_arg = DeclareLaunchArgument(
            'device',
            default_value='F37',
            description='mipi camera device')

        mipi_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('mipi_cam'),
                    'launch/mipi_cam.launch.py')),
            launch_arguments={
                'mipi_image_width': '960',
                'mipi_image_height': '544',
                'mipi_io_method': 'shared_mem',
                'mipi_frame_ts_type': 'realtime',
                'mipi_video_device': LaunchConfiguration('device')
            }.items()
        )

        camera_node = mipi_node
        camera_type_mipi = True
        camera_device_arg = mipi_cam_device_arg

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

    # jpeg->nv12 (非MIPI相机需要转换)
    nv12_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_decode.launch.py')),
        launch_arguments={
            'codec_in_mode': 'ros',
            'codec_out_mode': 'shared_mem',
            'codec_sub_topic': '/image',
            'codec_pub_topic': '/hbmem_img'
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

    # 零拷贝环境配置
    shared_mem_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_shm'),
                'launch/hobot_shm.launch.py'))
    )

    if camera_type_mipi:
        return LaunchDescription([
            model_file_name_launch_arg,
            is_sync_mode_launch_arg,
            score_threshold_launch_arg,
            camera_device_arg,
            # 启动零拷贝环境配置node
            shared_mem_node,
            # image publish
            camera_node,
            # image codec (for web display)
            jpeg_codec_node,
            # face detection
            mono2d_body_pub_topic_arg,
            mono2d_body_det_node,
            # web display
            web_smart_topic_arg,
            web_node
        ])
    else:
        return LaunchDescription([
            model_file_name_launch_arg,
            is_sync_mode_launch_arg,
            score_threshold_launch_arg,
            # 启动零拷贝环境配置node
            shared_mem_node,
            # image publish
            camera_node,
            # image codec (convert to SharedMem NV12)
            nv12_codec_node,
            # face detection
            mono2d_body_pub_topic_arg,
            mono2d_body_det_node,
            # web display
            web_smart_topic_arg,
            web_node
        ])

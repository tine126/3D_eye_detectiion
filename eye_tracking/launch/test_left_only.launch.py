# Copyright (c) 2024, TCL.
# Test launch file for left camera only

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get workspace install path
    workspace_install = os.path.expanduser('~/eye_tracking_ws/install')

    # Model paths
    body_detection_model = os.path.join(
        workspace_install,
        'mono2d_body_detection/lib/mono2d_body_detection/config/x5',
        'multitask_body_head_face_hand_kps_960x544.hbm'
    )

    face_landmarks_model = os.path.join(
        workspace_install,
        'face_landmarks_detection/share/face_landmarks_detection/config',
        'faceLandmark106pts.hbm'
    )

    # 1. Camera node (Gemini 335L - left IR only)
    camera_node = Node(
        package='orbbec_camera',
        executable='orbbec_camera_node',
        name='camera',
        parameters=[{
            'enable_color': False,
            'enable_depth': False,
            'enable_left_ir': True,
            'enable_right_ir': False,  # Disable right IR
            'left_ir_width': 1280,
            'left_ir_height': 800,
            'left_ir_fps': 30,
            'left_ir_format': 'Y8',
        }],
        output='screen'
    )

    # 2. Format conversion (mono8 to nv12) - left only
    mono2nv12_left = Node(
        package='mono8_to_nv12',
        executable='mono8_to_nv12_node',
        name='mono2nv12_left',
        parameters=[{
            'sub_topic': '/camera/left_ir/image_raw',
            'pub_topic': '/image_left',
        }],
        output='screen'
    )

    # 3. Body detection - left only
    body_detection_left = Node(
        package='mono2d_body_detection',
        executable='mono2d_body_detection',
        name='body_detection_left',
        parameters=[{
            'is_shared_mem_sub': 0,
            'model_file_name': body_detection_model,
            'ros_img_topic_name': '/image_left',
            'ai_msg_pub_topic_name': '/body_detection_left',
        }],
        output='screen'
    )

    # 4. Face landmarks detection - left only
    face_landmarks_left = Node(
        package='face_landmarks_detection',
        executable='face_landmarks_detection',
        name='face_landmarks_left',
        parameters=[{
            'is_shared_mem_sub': 0,
            'model_file_name': face_landmarks_model,
            'ai_msg_pub_topic_name': '/face_landmarks_left',
            'ai_msg_sub_topic_name': '/body_detection_left',
            'ros_img_topic_name': '/image_left',
        }],
        output='screen'
    )

    return LaunchDescription([
        # Start camera first
        camera_node,

        # Start format conversion after 2 seconds
        TimerAction(
            period=2.0,
            actions=[mono2nv12_left]
        ),

        # Start body detection after 3 seconds
        TimerAction(
            period=3.0,
            actions=[body_detection_left]
        ),

        # Start face landmarks after 4 seconds
        TimerAction(
            period=4.0,
            actions=[face_landmarks_left]
        ),
    ])

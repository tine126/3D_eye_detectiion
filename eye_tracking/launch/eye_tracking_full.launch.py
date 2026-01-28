"""
TCL Eye Tracking System - Full Launch File
Launches all nodes for the complete eye tracking pipeline:
1. Gemini 335L camera (dual IR)
2. Format conversion (mono8 -> nv12)
3. Body detection (left/right)
4. Face landmarks detection (left/right)
5. Eye tracking (2D, 3D, visualization)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    eye_tracking_pkg = get_package_share_directory('eye_tracking')
    mono8_to_nv12_pkg = get_package_share_directory('mono8_to_nv12')
    orbbec_camera_pkg = get_package_share_directory('orbbec_camera')

    # Model file paths (hardcoded absolute paths for workspace)
    # Note: get_package_share_directory returns system path /opt/tros/humble/share/...
    # We need to use workspace paths where models are actually installed
    workspace_install = os.path.expanduser('~/eye_tracking_ws/install')
    body_detection_model = os.path.join(
        workspace_install, 'mono2d_body_detection/lib/mono2d_body_detection/config',
        'multitask_body_head_face_hand_kps_960x544.hbm')
    face_landmarks_model = os.path.join(
        workspace_install, 'face_landmarks_detection/share/face_landmarks_detection/config',
        'faceLandmark106pts.hbm')

    # Default calibration file
    default_calib = os.path.join(eye_tracking_pkg, 'config', 'stereo_calibration.json')

    # Launch arguments
    declare_calib_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value=default_calib,
        description='Stereo calibration file path'
    )

    declare_trigger_interval_arg = DeclareLaunchArgument(
        'trigger_interval',
        default_value='5',
        description='Body detection trigger interval (frames)'
    )

    # 1. Camera launch (Gemini 335L with dual IR enabled)
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orbbec_camera_pkg, 'launch', 'gemini_330_series.launch.py')
        ),
        launch_arguments={
            'enable_left_ir': 'true',
            'enable_right_ir': 'true',
            'left_ir_width': '1280',
            'left_ir_height': '800',
            'left_ir_fps': '30',
            'right_ir_width': '1280',
            'right_ir_height': '800',
            'right_ir_fps': '30',
            'enable_color': 'false',
            'enable_depth': 'false',
            'enable_point_cloud': 'false',
        }.items()
    )

    # 2. Format conversion launch (mono8 -> nv12)
    format_convert_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mono8_to_nv12_pkg, 'launch', 'mono8_to_nv12.launch.py')
        ),
        launch_arguments={
            'left_input_topic': '/camera/left_ir/image_raw',
            'right_input_topic': '/camera/right_ir/image_raw',
        }.items()
    )

    # 3. Body detection nodes (left and right)
    body_detection_left = Node(
        package='mono2d_body_detection',
        executable='mono2d_body_detection',
        name='body_detection_left',
        parameters=[{
            'is_shared_mem_sub': 0,
            'model_file_name': body_detection_model,
            'ros_img_topic_name': '/image_left',
            'ai_msg_pub_topic_name': '/body_detection_left',
            'trigger_interval': LaunchConfiguration('trigger_interval'),
        }],
        output='screen'
    )

    body_detection_right = Node(
        package='mono2d_body_detection',
        executable='mono2d_body_detection',
        name='body_detection_right',
        parameters=[{
            'is_shared_mem_sub': 0,
            'model_file_name': body_detection_model,
            'ros_img_topic_name': '/image_right',
            'ai_msg_pub_topic_name': '/body_detection_right',
            'trigger_interval': LaunchConfiguration('trigger_interval'),
        }],
        output='screen'
    )

    # 4. Face landmarks detection nodes (left and right)
    face_landmarks_left = Node(
        package='face_landmarks_detection',
        executable='face_landmarks_detection',
        name='face_landmarks_left',
        parameters=[{
            'is_shared_mem_sub': 0,
            'model_file_name': face_landmarks_model,
            'ros_img_topic_name': '/image_left',
            'ai_msg_sub_topic_name': '/body_detection_left',
            'ai_msg_pub_topic_name': '/face_landmarks_left',
        }],
        output='screen'
    )

    face_landmarks_right = Node(
        package='face_landmarks_detection',
        executable='face_landmarks_detection',
        name='face_landmarks_right',
        parameters=[{
            'is_shared_mem_sub': 0,
            'model_file_name': face_landmarks_model,
            'ros_img_topic_name': '/image_right',
            'ai_msg_sub_topic_name': '/body_detection_right',
            'ai_msg_pub_topic_name': '/face_landmarks_right',
        }],
        output='screen'
    )

    # 5. Eye tracking node (combined: 2D, 3D, visualization)
    eye_tracking_node = Node(
        package='eye_tracking',
        executable='eye_tracking_node',
        name='eye_tracking_node',
        parameters=[{
            'calibration_file': LaunchConfiguration('calibration_file'),
        }],
        output='screen'
    )

    # Use TimerAction to delay node startup for proper initialization order
    delayed_format_convert = TimerAction(
        period=2.0,
        actions=[format_convert_launch]
    )

    delayed_body_detection = TimerAction(
        period=4.0,
        actions=[body_detection_left, body_detection_right]
    )

    delayed_face_landmarks = TimerAction(
        period=6.0,
        actions=[face_landmarks_left, face_landmarks_right]
    )

    delayed_eye_tracking = TimerAction(
        period=8.0,
        actions=[eye_tracking_node]
    )

    return LaunchDescription([
        declare_calib_arg,
        declare_trigger_interval_arg,
        camera_launch,
        delayed_format_convert,
        delayed_body_detection,
        delayed_face_landmarks,
        delayed_eye_tracking,
    ])

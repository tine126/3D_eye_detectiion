"""
Gemini 335L Camera Launch File for Eye Tracking System
Enables dual IR cameras at 1280x800@30fps
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    orbbec_camera_pkg = get_package_share_directory('orbbec_camera')

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orbbec_camera_pkg, 'launch', 'gemini_330_series.launch.py')
        ),
        launch_arguments={
            'camera_name': 'camera',
            'enable_left_ir': 'true',
            'enable_right_ir': 'true',
            'left_ir_width': '1280',
            'left_ir_height': '800',
            'left_ir_fps': '30',
            'left_ir_format': 'Y8',
            'right_ir_width': '1280',
            'right_ir_height': '800',
            'right_ir_fps': '30',
            'right_ir_format': 'Y8',
            'enable_color': 'false',
            'enable_depth': 'false',
            'enable_point_cloud': 'false',
            'enable_colored_point_cloud': 'false',
        }.items()
    )

    return LaunchDescription([
        camera_launch,
    ])

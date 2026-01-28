from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('eye_tracking')
    default_calib = os.path.join(pkg_dir, 'config', 'stereo_calibration.json')

    return LaunchDescription([
        DeclareLaunchArgument(
            'calibration_file',
            default_value=default_calib,
            description='Stereo calibration file path'
        ),
        Node(
            package='eye_tracking',
            executable='eye_position_2d_node',
            name='eye_position_2d_node',
            output='screen'
        ),
        Node(
            package='eye_tracking',
            executable='eye_position_3d_node',
            name='eye_position_3d_node',
            parameters=[{
                'calibration_file': LaunchConfiguration('calibration_file'),
            }],
            output='screen'
        ),
        Node(
            package='eye_tracking',
            executable='eye_visualization_node',
            name='eye_visualization_node',
            output='screen'
        ),
    ])

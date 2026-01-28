from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'left_input_topic',
            default_value='/left_ir/image_raw',
            description='Left IR image input topic'
        ),
        DeclareLaunchArgument(
            'right_input_topic',
            default_value='/right_ir/image_raw',
            description='Right IR image input topic'
        ),
        DeclareLaunchArgument(
            'left_output_topic',
            default_value='/image_left',
            description='Left NV12 image output topic'
        ),
        DeclareLaunchArgument(
            'right_output_topic',
            default_value='/image_right',
            description='Right NV12 image output topic'
        ),
        Node(
            package='mono8_to_nv12',
            executable='mono8_to_nv12_node',
            name='mono8_to_nv12_node',
            parameters=[{
                'left_input_topic': LaunchConfiguration('left_input_topic'),
                'right_input_topic': LaunchConfiguration('right_input_topic'),
                'left_output_topic': LaunchConfiguration('left_output_topic'),
                'right_output_topic': LaunchConfiguration('right_output_topic'),
            }],
            output='screen'
        )
    ])

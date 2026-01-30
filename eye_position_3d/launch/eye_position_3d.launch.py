# Copyright (c) 2024，D-Robotics.
# eye_position_3d: 双目视觉三角测量计算三维眼睛坐标

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    # 参数定义
    params = [
        # 输入话题
        {"name": "left_eye_topic", "default": "/eye_positions_left", "desc": "左IR眼睛2D坐标"},
        {"name": "right_eye_topic", "default": "/eye_positions_right", "desc": "右IR眼睛2D坐标"},
        # 相机信息话题
        {"name": "left_camera_info_topic", "default": "/camera/left_ir/camera_info", "desc": "左IR相机信息"},
        {"name": "right_camera_info_topic", "default": "/camera/right_ir/camera_info", "desc": "右IR相机信息"},
        # 输出话题
        {"name": "output_topic", "default": "/eye_positions_3d", "desc": "3D坐标输出"},
        # 算法参数
        {"name": "min_disparity", "default": "5.0", "desc": "最小视差(像素)"},
        {"name": "max_disparity", "default": "200.0", "desc": "最大视差(像素)"},
        {"name": "min_depth_mm", "default": "200.0", "desc": "最小深度(mm)"},
        {"name": "max_depth_mm", "default": "2000.0", "desc": "最大深度(mm)"},
        {"name": "max_y_diff", "default": "10.0", "desc": "最大Y坐标差异(像素)"},
        # 手动相机参数配置
        {"name": "use_manual_camera_params", "default": "true", "desc": "是否使用手动配置"},
        {"name": "manual_fx", "default": "619.688049", "desc": "手动焦距x"},
        {"name": "manual_fy", "default": "619.688049", "desc": "手动焦距y"},
        {"name": "manual_cx", "default": "638.0", "desc": "手动主点x"},
        {"name": "manual_cy", "default": "396.0", "desc": "手动主点y"},
        {"name": "manual_baseline_mm", "default": "95.0", "desc": "手动基线距离(mm)"},
        # 日志
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
        package="eye_position_3d",
        executable="eye_position_3d_node",
        output="screen",
        parameters=[node_params],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    return LaunchDescription(launch_args + [node])

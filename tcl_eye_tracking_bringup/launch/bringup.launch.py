# Copyright (c) 2024，D-Robotics.
# TCL人眼定位算法项目 - 统一启动文件

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ========== FastDDS 配置：禁用共享内存 ==========
    # HbmMsg1080P 使用 6MB 固定数组，需要禁用 data_sharing 避免分配器冲突
    fastdds_config = os.path.join(
        get_package_share_directory('tcl_eye_tracking_bringup'),
        'config', 'fastdds_no_shm.xml'
    )

    set_fastdds_env = SetEnvironmentVariable(
        name='FASTRTPS_DEFAULT_PROFILES_FILE',
        value=fastdds_config
    )
    set_fastdds_env2 = SetEnvironmentVariable(
        name='FASTDDS_DEFAULT_PROFILES_FILE',
        value=fastdds_config
    )
    set_rmw_env = SetEnvironmentVariable(
        name='RMW_FASTRTPS_USE_QOS_FROM_XML',
        value='1'
    )
    set_rmw_shm_env = SetEnvironmentVariable(
        name='RMW_FASTRTPS_USE_SHM',
        value='0'
    )

    # ========== 全局参数 ==========
    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='warn',
        description='日志级别 (debug, info, warn, error)'
    )

    # ========== 相机参数 ==========
    camera_args = [
        DeclareLaunchArgument('enable_depth', default_value='false'),
        DeclareLaunchArgument('enable_color', default_value='false'),
        DeclareLaunchArgument('enable_left_ir', default_value='true'),
        DeclareLaunchArgument('enable_right_ir', default_value='true'),
        DeclareLaunchArgument('left_ir_format', default_value='Y8'),
        DeclareLaunchArgument('right_ir_format', default_value='Y8'),
        DeclareLaunchArgument('left_ir_width', default_value='1280'),
        DeclareLaunchArgument('left_ir_height', default_value='800'),
        DeclareLaunchArgument('left_ir_fps', default_value='30'),
        DeclareLaunchArgument('right_ir_width', default_value='1280'),
        DeclareLaunchArgument('right_ir_height', default_value='800'),
        DeclareLaunchArgument('right_ir_fps', default_value='30'),
    ]

    # ========== 算法参数 ==========
    algo_args = [
        DeclareLaunchArgument('score_threshold', default_value='0.5',
                              description='人脸检测置信度阈值'),
        # 3D计算参数
        DeclareLaunchArgument('use_manual_camera_params', default_value='true'),
        DeclareLaunchArgument('manual_fx', default_value='619.688049'),
        DeclareLaunchArgument('manual_fy', default_value='619.688049'),
        DeclareLaunchArgument('manual_cx', default_value='638.0'),
        DeclareLaunchArgument('manual_cy', default_value='396.0'),
        DeclareLaunchArgument('manual_baseline_mm', default_value='95.0'),
    ]

    # ========== 1. 相机节点 ==========
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('orbbec_camera'),
            '/launch/gemini_330_series.launch.py'
        ]),
        launch_arguments={
            'enable_depth': LaunchConfiguration('enable_depth'),
            'enable_color': LaunchConfiguration('enable_color'),
            'enable_left_ir': LaunchConfiguration('enable_left_ir'),
            'enable_right_ir': LaunchConfiguration('enable_right_ir'),
            'left_ir_format': LaunchConfiguration('left_ir_format'),
            'right_ir_format': LaunchConfiguration('right_ir_format'),
            'left_ir_width': LaunchConfiguration('left_ir_width'),
            'left_ir_height': LaunchConfiguration('left_ir_height'),
            'left_ir_fps': LaunchConfiguration('left_ir_fps'),
            'right_ir_width': LaunchConfiguration('right_ir_width'),
            'right_ir_height': LaunchConfiguration('right_ir_height'),
            'right_ir_fps': LaunchConfiguration('right_ir_fps'),
        }.items()
    )

    # ========== 2. 格式转换节点 (延迟1秒启动) ==========
    img_converter_launch = TimerAction(
        period=1.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_package_share_directory('img_format_converter'),
                    '/launch/img_format_converter.launch.py'
                ]),
                launch_arguments={
                    'log_level': LaunchConfiguration('log_level'),
                }.items()
            )
        ]
    )

    # ========== 3. 人脸检测节点 (延迟2秒启动) ==========
    body_det_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_package_share_directory('mono2d_body_detection'),
                    '/launch/mono2d_body_detection_without_camera.launch.py'
                ]),
                launch_arguments={
                    'score_threshold': LaunchConfiguration('score_threshold'),
                    'log_level': LaunchConfiguration('log_level'),
                }.items()
            )
        ]
    )

    # ========== 4. 关键点检测节点 (延迟3秒启动) ==========
    landmarks_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_package_share_directory('face_landmarks_detection'),
                    '/launch/face_landmarks_det_node.launch.py'
                ]),
                launch_arguments={
                    'score_threshold': LaunchConfiguration('score_threshold'),
                    'log_level': LaunchConfiguration('log_level'),
                }.items()
            )
        ]
    )

    # ========== 5. 眼睛2D坐标提取节点 (延迟4秒启动) ==========
    eye_pub_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_package_share_directory('eye_position_publisher'),
                    '/launch/eye_position_publisher.launch.py'
                ]),
                launch_arguments={
                    'log_level': LaunchConfiguration('log_level'),
                }.items()
            )
        ]
    )

    # ========== 6. 三维坐标计算节点 (延迟5秒启动) ==========
    eye_3d_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_package_share_directory('eye_position_3d'),
                    '/launch/eye_position_3d.launch.py'
                ]),
                launch_arguments={
                    'use_manual_camera_params': LaunchConfiguration('use_manual_camera_params'),
                    'manual_fx': LaunchConfiguration('manual_fx'),
                    'manual_fy': LaunchConfiguration('manual_fy'),
                    'manual_cx': LaunchConfiguration('manual_cx'),
                    'manual_cy': LaunchConfiguration('manual_cy'),
                    'manual_baseline_mm': LaunchConfiguration('manual_baseline_mm'),
                    'log_level': LaunchConfiguration('log_level'),
                }.items()
            )
        ]
    )

    return LaunchDescription([
        # FastDDS 配置 (禁用共享内存)
        set_fastdds_env,
        set_fastdds_env2,
        set_rmw_env,
        set_rmw_shm_env,
        # 参数声明
        log_level_arg,
        *camera_args,
        *algo_args,
        # 节点启动 (按顺序延迟启动)
        camera_launch,
        img_converter_launch,
        body_det_launch,
        landmarks_launch,
        eye_pub_launch,
        eye_3d_launch,
    ])

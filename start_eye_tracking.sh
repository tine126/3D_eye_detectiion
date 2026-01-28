#!/bin/bash
# TCL人眼定位系统 - 一键启动脚本

source ~/eye_tracking_ws/install/setup.bash

# 启动所有节点
ros2 launch eye_tracking eye_tracking_full.launch.py

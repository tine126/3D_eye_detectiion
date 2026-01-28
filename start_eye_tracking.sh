#!/bin/bash
# TCL人眼定位系统 - 一键启动脚本

# Source ROS2 基础环境
source /opt/tros/humble/setup.bash

# Source 工作空间
source ~/eye_tracking_ws/install/setup.bash

# 创建日志目录
LOG_DIR=~/eye_tracking_ws/logs
mkdir -p "$LOG_DIR"

# 生成带时间戳的日志文件名
LOG_FILE="$LOG_DIR/run_$(date +%Y%m%d_%H%M%S).log"

echo "日志将保存到: $LOG_FILE"

# 启动所有节点，同时输出到终端和日志文件
ros2 launch eye_tracking eye_tracking_full.launch.py 2>&1 | tee "$LOG_FILE"

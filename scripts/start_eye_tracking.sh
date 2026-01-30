#!/bin/bash
# TCL人眼定位算法项目 - 启动脚本
# 功能: 启动所有节点并保存日志
#启动命令：~/ros2_ws/src/tcl_eye_tracking/scripts/start_eye_tracking.sh
# ========== 配置 ==========
LOG_DIR="$HOME/eye_tracking_logs"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="${LOG_DIR}/eye_tracking_${TIMESTAMP}.log"

# ========== 创建日志目录 ==========
mkdir -p "$LOG_DIR"

# ========== 清理残留进程 ==========
echo "清理残留进程..."
pkill -f mono2d_body_detection 2>/dev/null || true
pkill -f face_landmarks_detection 2>/dev/null || true
pkill -f img_format_converter 2>/dev/null || true
pkill -f eye_position 2>/dev/null || true
pkill -f orbbec_camera 2>/dev/null || true
sleep 1

# ========== 清理 FastDDS 共享内存残留 ==========
echo "清理 FastDDS 共享内存..."
rm -f /dev/shm/*fastdds* 2>/dev/null || true
rm -f /dev/shm/*fastrtps* 2>/dev/null || true

# ========== 环境设置 ==========
source /opt/tros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# FastDDS 配置：彻底禁用共享内存传输
# 注意: HbmMsg1080P 消息类型与 FastDDS SHM/data_sharing 不兼容
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export RMW_FASTRTPS_USE_SHM=0
export FASTRTPS_DEFAULT_PROFILES_FILE=~/ros2_ws/install/tcl_eye_tracking_bringup/share/tcl_eye_tracking_bringup/config/fastdds_no_shm.xml
export FASTDDS_DEFAULT_PROFILES_FILE=~/ros2_ws/install/tcl_eye_tracking_bringup/share/tcl_eye_tracking_bringup/config/fastdds_no_shm.xml

# ========== 显示启动信息 ==========
echo "=============================================="
echo "  TCL 人眼定位算法系统启动"
echo "=============================================="
echo "启动时间: $(date)"
echo "日志文件: $LOG_FILE"
echo "----------------------------------------------"

# ========== 启动节点 ==========
echo "正在启动所有节点..."
echo ""

# 使用 tee 同时输出到控制台和日志文件
ros2 launch tcl_eye_tracking_bringup bringup.launch.py \
    log_level:=info \
    2>&1 | tee "$LOG_FILE"

# ========== 退出处理 ==========
echo ""
echo "----------------------------------------------"
echo "系统已停止，日志已保存到: $LOG_FILE"

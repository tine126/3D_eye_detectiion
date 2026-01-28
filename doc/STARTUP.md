# TCL人眼定位系统 - 启动指南

## 一、系统架构

```
OrbbecSDK_ROS2 (Gemini 335L)
    │
    ├── /left_ir/image_raw (mono8)
    └── /right_ir/image_raw (mono8)
            ↓
[格式转换节点] mono8_to_nv12_node
    │
    ├── /image_left (nv12)
    └── /image_right (nv12)
            ↓
[人体检测节点] mono2d_body_detection
    │
    └── /hobot_mono2d_body_detection
            ↓
[人脸关键点节点] face_landmarks_detection
    │
    ├── /face_landmarks_left
    └── /face_landmarks_right
            ↓
[人眼2D坐标节点] eye_position_2d_node
    │
    ├── /eye_positions_left
    └── /eye_positions_right
            ↓
[人眼3D坐标节点] eye_position_3d_node
    │
    └── /eye_positions_3d
            ↓
[可视化节点] eye_visualization_node
    │
    └── /visualization
```

## 二、快速启动

### 2.1 一键启动脚本
```bash
# 创建启动脚本
cat > ~/start_eye_tracking.sh << 'EOF'
#!/bin/bash
source ~/eye_tracking_ws/install/setup.bash

# 启动所有节点
ros2 launch eye_tracking eye_tracking_full.launch.py
EOF

chmod +x ~/start_eye_tracking.sh
```

### 2.2 执行启动
```bash
~/start_eye_tracking.sh
```

## 三、分步启动

### 3.1 终端1: 启动相机
```bash
source ~/eye_tracking_ws/install/setup.bash
ros2 launch orbbec_camera gemini_335L.launch.py
```

### 3.2 终端2: 启动格式转换
```bash
source ~/eye_tracking_ws/install/setup.bash
ros2 launch mono8_to_nv12 mono8_to_nv12.launch.py
```

### 3.3 终端3: 启动人体检测 (左路)
```bash
source ~/eye_tracking_ws/install/setup.bash
ros2 run mono2d_body_detection mono2d_body_detection \
    --ros-args \
    -p is_shared_mem_sub:=0 \
    -p ros_img_topic_name:=/image_left \
    -p ai_msg_pub_topic_name:=/body_detection_left \
    -p trigger_interval:=5
```

### 3.4 终端4: 启动人体检测 (右路)
```bash
source ~/eye_tracking_ws/install/setup.bash
ros2 run mono2d_body_detection mono2d_body_detection \
    --ros-args \
    -p is_shared_mem_sub:=0 \
    -p ros_img_topic_name:=/image_right \
    -p ai_msg_pub_topic_name:=/body_detection_right \
    -p trigger_interval:=5
```

### 3.5 终端5: 启动人脸关键点 (左路)
```bash
source ~/eye_tracking_ws/install/setup.bash
ros2 run face_landmarks_detection face_landmarks_detection \
    --ros-args \
    -p is_shared_mem_sub:=0 \
    -p ros_img_topic_name:=/image_left \
    -p ai_msg_sub_topic_name:=/body_detection_left \
    -p ai_msg_pub_topic_name:=/face_landmarks_left
```

### 3.6 终端6: 启动人脸关键点 (右路)
```bash
source ~/eye_tracking_ws/install/setup.bash
ros2 run face_landmarks_detection face_landmarks_detection \
    --ros-args \
    -p is_shared_mem_sub:=0 \
    -p ros_img_topic_name:=/image_right \
    -p ai_msg_sub_topic_name:=/body_detection_right \
    -p ai_msg_pub_topic_name:=/face_landmarks_right
```

### 3.7 终端7: 启动人眼定位
```bash
source ~/eye_tracking_ws/install/setup.bash
ros2 launch eye_tracking eye_tracking.launch.py
```

## 四、调试命令

### 4.1 查看话题列表
```bash
ros2 topic list
```

### 4.2 查看话题帧率
```bash
ros2 topic hz /eye_positions_3d
```

### 4.3 查看3D坐标输出
```bash
ros2 topic echo /eye_positions_3d
```

### 4.4 查看节点图
```bash
ros2 run rqt_graph rqt_graph
```

### 4.5 查看节点列表
```bash
ros2 node list
```

## 五、常见问题

### 5.1 相机无法打开
```bash
# 检查USB权限
sudo chmod 666 /dev/ttyUSB*
# 或添加udev规则
```

### 5.2 帧率不足30fps
- 检查trigger_interval参数
- 减少dump_render_img输出

### 5.3 3D坐标不准确
- 检查stereo_calibration.json标定参数
- 确认baseline值正确
```

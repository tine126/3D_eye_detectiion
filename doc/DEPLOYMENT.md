# TCL人眼定位系统 - 部署指南

## 一、环境准备

### 1.1 系统要求
- 开发板: 地平线 RDK X5
- 相机: 奥比中光 Gemini 335L
- 系统: Ubuntu 22.04 + ROS2 Humble
- 帧率要求: 30fps

### 1.2 依赖安装

```bash
# 更新系统
sudo apt update && sudo apt upgrade -y

# 安装ROS2 Humble基础依赖
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-message-filters \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs

# 安装OpenCV
sudo apt install -y libopencv-dev

# 安装nlohmann-json (用于配置文件解析)
sudo apt install -y nlohmann-json3-dev
```

### 1.3 地平线BPU依赖
```bash
# 确保已安装地平线TogetheROS
# 参考: https://developer.horizon.ai/

# 验证dnn_node是否可用
ros2 pkg list | grep dnn_node
```

## 二、编译部署

### 2.1 创建工作空间
```bash
mkdir -p ~/eye_tracking_ws/src
cd ~/eye_tracking_ws/src
```

### 2.2 克隆代码
```bash
git clone https://github.com/tine126/3D_eye_detectiion.git
```

### 2.3 项目结构
克隆完成后，工作空间结构如下：

```
~/eye_tracking_ws/                      # ROS2工作空间根目录
├── build/                              # 编译生成（colcon build后自动创建）
├── install/                            # 安装目录（colcon build后自动创建）
├── log/                                # 日志目录（colcon build后自动创建）
└── src/                                # 源码目录
    └── 3D_eye_detectiion/              # 克隆的仓库
        ├── doc/                        # 文档
        ├── eye_tracking/               # 人眼3D定位主包
        ├── eye_tracking_msgs/          # 自定义消息定义
        ├── face_landmarks_detection/   # 人脸关键点检测
        ├── mono2d_body_detection/      # 人体检测
        ├── mono8_to_nv12/              # 图像格式转换
        └── OrbbecSDK_ROS2/             # 奥比中光相机驱动
```

> **注意**: 编译命令需要在 `~/eye_tracking_ws/` 目录下执行，colcon会递归扫描src目录下的所有ROS2包。

### 2.4 编译顺序
由于包之间存在依赖关系，需要按顺序编译：

```bash
cd ~/eye_tracking_ws

# 1. 首先编译消息包
colcon build --packages-select eye_tracking_msgs

# 2. 刷新环境
source install/setup.bash

# 3. 逐个编译其他包 (避免内存不足)
colcon build --packages-select mono8_to_nv12

colcon build --packages-select mono2d_body_detection \
    --allow-overriding mono2d_body_detection

colcon build --packages-select face_landmarks_detection \
    --allow-overriding face_landmarks_detection

colcon build --packages-select eye_tracking
```

### 2.5 交叉编译 (可选)
如果在x86主机上交叉编译：
```bash
# 设置交叉编译工具链
export CROSS_COMPILE=/path/to/aarch64-linux-gnu-

# 编译
colcon build --cmake-args \
    -DPLATFORM_X5=ON \
    -DCMAKE_TOOLCHAIN_FILE=/path/to/toolchain.cmake
```

## 三、模型部署

### 3.1 模型文件
将以下模型文件放置到对应位置：

```
~/eye_tracking_ws/install/mono2d_body_detection/lib/mono2d_body_detection/config/
├── multitask_body_head_face_hand_kps_960x544.hbm  # 人体检测模型

~/eye_tracking_ws/install/face_landmarks_detection/share/face_landmarks_detection/config/
├── faceLandmark106pts.hbm  # 人脸关键点模型
```

### 3.2 标定文件
编辑双目标定参数：
```bash
vim ~/eye_tracking_ws/install/eye_tracking/share/eye_tracking/config/stereo_calibration.json
```

内容示例：
```json
{
    "left_camera": {
        "fx": 619.688049,
        "fy": 619.688049,
        "cx": 638.0,
        "cy": 396.0
    },
    "right_camera": {
        "fx": 619.688049,
        "fy": 619.688049,
        "cx": 638.0,
        "cy": 396.0
    },
    "stereo": {
        "baseline": 0.095
    }
}
```

## 四、相机配置

### 4.1 OrbbecSDK_ROS2配置
OrbbecSDK_ROS2已包含在项目仓库中，无需单独克隆。

```bash
# 编译相机驱动
cd ~/eye_tracking_ws
colcon build --packages-select orbbec_camera --allow-overriding orbbec_camera
```

### 4.2 相机参数配置
创建相机启动配置文件：
```yaml
# camera_params.yaml
left_ir:
  enable: true
  width: 1280
  height: 800
  fps: 30
  format: mono8

right_ir:
  enable: true
  width: 1280
  height: 800
  fps: 30
  format: mono8
```

## 五、环境变量配置

### 5.1 添加到bashrc
```bash
echo "source ~/eye_tracking_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 5.2 共享内存配置 (可选)
```bash
# 启用零拷贝传输
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
```

## 六、验证部署

### 6.1 检查包是否安装成功
```bash
ros2 pkg list | grep -E "eye_tracking|mono8_to_nv12|mono2d_body|face_landmarks"
```

预期输出：
```
eye_tracking
eye_tracking_msgs
face_landmarks_detection
mono2d_body_detection
mono8_to_nv12
```

### 6.2 检查话题
```bash
# 启动系统后检查
ros2 topic list
```

预期话题：
```
/left_ir/image_raw
/right_ir/image_raw
/image_left
/image_right
/hobot_mono2d_body_detection
/face_landmarks_left
/face_landmarks_right
/eye_positions_left
/eye_positions_right
/eye_positions_3d
/visualization
```

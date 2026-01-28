# mono2d_body_detection 从零到跑通操作手册

> 本文档提供完整的部署指南，每条命令后附有解释说明。

---

## 目录

1. [环境要求](#一环境要求)
2. [安装与构建](#二安装与构建)
3. [运行命令](#三运行命令)
4. [验证结果](#四验证结果)
5. [常见报错 Top 10](#五常见报错-top-10)
6. [关键文件位置速查](#六关键文件位置速查)
7. [快速启动脚本](#七快速启动脚本)

---

## 一、环境要求

### 1.1 硬件平台

| 平台 | 支持的模型 | 说明 |
|------|-----------|------|
| RDK X3 | fasterRcnn | 入门级开发板，BPU推理 |
| RDK Ultra | fasterRcnn | 高性能开发板 |
| RDK X5 | fasterRcnn | 中端开发板 |
| RDK S100 | yolo-pose | 新一代平台 |
| RDK S600 | yolo-pose | 新一代高性能平台 |

**为何需要特定硬件**：该项目依赖地平线 BPU (Brain Processing Unit) 进行神经网络推理加速，模型文件 `.hbm` 是 BPU 专用格式，无法在普通 x86 PC 上直接运行推理。

### 1.2 系统镜像

```bash
# 确认系统版本
cat /etc/os-release
```

**支持的系统**：Ubuntu 20.04 / 22.04 / 24.04（需烧录 D-Robotics 官方镜像）

**为何需要官方镜像**：官方镜像预装了 BPU 驱动、hobot 库等底层依赖。

### 1.3 ROS2 版本对应关系

| tros版本 | ROS2 版本 | Ubuntu 版本 |
|----------|-----------|-------------|
| tros foxy | Foxy | 20.04 |
| tros humble | Humble | 22.04 |
| tros jazzy | Jazzy | 24.04 |

### 1.4 核心依赖包

| 包名 | 用途 |
|------|------|
| `rclcpp` | ROS2 C++ 客户端库 |
| `dnn_node` | 地平线 DNN 推理封装 |
| `hobot_cv` | 地平线图像处理库 |
| `hobot_mot` | 多目标跟踪库（非x86平台） |
| `ai_msgs` | AI 消息定义 |
| `hbm_img_msgs` | 共享内存图像消息 |
| `cv_bridge` | OpenCV-ROS 桥接 |
| `sensor_msgs` | 标准传感器消息 |

---

## 二、安装与构建

### 2.1 方式一：apt 安装（推荐）

```bash
# 更新软件源
sudo apt update
# 作用：同步最新的软件包索引，确保能找到最新版本

# tros humble 版本安装
sudo apt install -y tros-humble-mono2d-body-detection
# 作用：安装预编译的二进制包及所有依赖，无需手动编译
```

**其他版本**：

```bash
# tros foxy
sudo apt install -y tros-mono2d-body-detection

# tros jazzy
sudo apt install -y tros-jazzy-mono2d-body-detection
```

### 2.2 方式二：源码编译

```bash
# 1. 创建工作空间
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
# 作用：ROS2 标准工作空间结构，src 存放源码

# 2. 克隆代码
git clone https://github.com/D-Robotics/mono2d_body_detection.git
# 作用：获取源代码

# 3. 安装依赖
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
# 作用：自动解析 package.xml 并安装缺失的依赖包

# 4. 配置环境
source /opt/tros/humble/setup.bash
# 作用：加载 tros 环境变量，使 colcon 能找到 dnn_node 等依赖

# 5. 编译（根据平台选择）
```

**平台编译命令**：

```bash
# RDK X3:
colcon build --packages-select mono2d_body_detection --cmake-args -DPLATFORM_X3=ON

# RDK Ultra:
colcon build --packages-select mono2d_body_detection --cmake-args -DPLATFORM_Rdkultra=ON

# RDK X5:
colcon build --packages-select mono2d_body_detection --cmake-args -DPLATFORM_X5=ON

# RDK S100:
colcon build --packages-select mono2d_body_detection --cmake-args -DPLATFORM_S100=ON

# RDK S600:
colcon build --packages-select mono2d_body_detection --cmake-args -DPLATFORM_S600=ON
```

> **说明**：`-DPLATFORM_XXX` 决定链接哪个平台的 BPU 库和模型文件

```bash
# 6. 加载编译结果
source install/setup.bash
# 作用：使新编译的包可被 ros2 命令发现
```

---

## 三、运行命令

### 3.1 准备工作

```bash
# 1. 配置 tros 环境
source /opt/tros/humble/setup.bash
# 作用：加载 ROS2 和 tros 的环境变量

# 2. 拷贝配置文件到当前目录
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .
# 作用：模型文件(.hbm)和跟踪参数(.json)需要在当前目录下
# 注意：${TROS_DISTRO} 会自动替换为 foxy/humble/jazzy
```

### 3.2 使用 MIPI 摄像头

```bash
# 设置相机类型
export CAM_TYPE=mipi
# 作用：launch 文件根据此变量选择启动 mipi_cam 节点

# 启动
ros2 launch mono2d_body_detection mono2d_body_detection.launch.py
# 作用：启动完整 pipeline：相机→编码→检测→Web可视化
```

### 3.3 使用 USB 摄像头

```bash
export CAM_TYPE=usb
# 作用：切换为 USB 相机输入，启动 hobot_usb_cam 节点

ros2 launch mono2d_body_detection mono2d_body_detection.launch.py
```

### 3.4 本地图片回放（调试用）

```bash
export CAM_TYPE=fb
# 作用：fb = feedback，使用 hobot_image_publisher 发布本地图片

ros2 launch mono2d_body_detection mono2d_body_detection.launch.py \
  publish_image_source:=config/person_body.jpg \
  publish_image_format:=jpg \
  publish_output_image_w:=960 \
  publish_output_image_h:=544
```

**参数说明**：
- `publish_image_source`: 输入图片路径
- `publish_image_format`: 图片格式
- `publish_output_image_w/h`: 输出分辨率，需匹配模型输入

### 3.5 使用 yolo-pose 模型（S100/S600）

```bash
source /opt/tros/humble/setup.bash
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .
export CAM_TYPE=mipi

ros2 launch mono2d_body_detection mono2d_body_detection.launch.py \
  kps_model_type:=1 \
  kps_image_width:=640 \
  kps_image_height:=640 \
  kps_model_file_name:=config/yolo11x_pose_nashe_640x640_nv12.hbm
```

**参数说明**：
- `kps_model_type:=1` 表示使用 yolo-pose 模型（0=fasterRcnn）
- `kps_image_width/height`: yolo-pose 需要 640x640 输入
- `kps_model_file_name`: 指定 yolo-pose 模型文件

### 3.6 不带相机启动（仅检测节点）

```bash
ros2 launch mono2d_body_detection mono2d_body_detection_without_camera.launch.py
# 作用：仅启动检测节点，需要外部提供 /hbmem_img topic
# 适用场景：与其他图像源节点组合使用
```

---

## 四、验证结果

### 4.1 查看 Topic 列表

```bash
ros2 topic list
```

**预期输出**：
```
/hobot_mono2d_body_detection  # AI 检测结果
/hbmem_img                     # 共享内存图像（输入）
/image                         # JPEG 编码图像（Web显示用）
```

### 4.2 查看检测结果

```bash
ros2 topic echo /hobot_mono2d_body_detection
# 作用：实时打印检测结果消息
```

**消息结构解读** (`ai_msgs/msg/PerceptionTargets`)：

```yaml
header:
  stamp:
    sec: 1234567890      # 时间戳（秒）
    nanosec: 123456789   # 时间戳（纳秒）
  frame_id: "default"    # 帧ID
fps: 30                  # 处理帧率（<0表示无效）
perfs: []                # 性能统计（各阶段耗时）
targets:                 # 检测到的目标数组
  - type: "person"       # 目标类型
    track_id: 1          # 跟踪ID（track_mode=1时有效）
    rois:                # 检测框数组
      - type: "body"     # 框类型：body/head/face/hand
        rect:
          x_offset: 100  # 左上角 x
          y_offset: 50   # 左上角 y
          width: 200     # 宽度
          height: 400    # 高度
        confidence: 0.95 # 置信度
    points:              # 关键点数组
      - type: "body_kps" # 关键点类型
        point:           # 17个人体关键点
          - x: 150.0
            y: 80.0
            z: 0.0       # 2D检测z=0
disappeared_targets: []  # 消失的目标（用于跟踪）
```

### 4.3 Web 可视化

```bash
# 在同一网络的电脑浏览器中访问
http://<RDK_IP>:8000
```

> **作用**：websocket 节点提供实时视频流和检测框叠加显示
> **示例**：`http://192.168.1.100:8000`

### 4.4 查看节点状态

```bash
ros2 node list
```

**预期输出**：
```
/mono2d_body_det
/mipi_cam (或 /hobot_usb_cam)
/hobot_codec_encoder
/websocket
```

```bash
ros2 node info /mono2d_body_det
# 作用：查看节点的订阅/发布 topic 和参数
```

### 4.5 保存渲染图片（调试）

```bash
ros2 launch mono2d_body_detection mono2d_body_detection.launch.py \
  dump_render_img:=1
# 作用：将检测结果渲染到图片并保存到当前目录
# 输出文件：render_<frame_id>_<sec>_<nanosec>.jpeg
```

---

## 五、常见报错 Top 10

### 1. 模型文件找不到

**现象**：
```
[ERROR] [mono2d_body_det]: Init failed!
```

**原因**：未拷贝 config 目录或路径错误

**排查**：
```bash
ls -la config/
# 检查是否存在 .hbm 模型文件
```

**修复**：
```bash
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .
```

---

### 2. 共享内存环境变量未设置

**现象**：
```
[ERROR] Launching with zero-copy, but env of `RMW_FASTRTPS_USE_QOS_FROM_XML` is not set
```

**原因**：共享内存通信需要特定环境变量

**排查**：
```bash
echo $RMW_FASTRTPS_USE_QOS_FROM_XML
```

**修复**：
```bash
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/tros/humble/lib/hobot_shm/config/shm_fastdds.xml
# 或使用 launch 文件自动配置（hobot_shm 节点会设置）
```

---

### 3. 相机设备未找到

**现象**：
```
[ERROR] [mipi_cam]: Open camera failed!
```

**原因**：MIPI 相机未正确连接或设备号错误

**排查**：
```bash
ls /dev/video*
# 检查相机设备是否存在
```

**修复**：
- 检查相机排线连接
- 指定正确的设备：
```bash
ros2 launch mono2d_body_detection mono2d_body_detection.launch.py device:=GC4663
```

---

### 4. BPU 初始化失败

**现象**：
```
[ERROR] [dnn_node]: hbDNNInitializeFromFiles failed
```

**原因**：BPU 驱动未加载或模型与平台不匹配

**排查**：
```bash
# 检查 BPU 驱动
lsmod | grep hobot

# 检查 BPU 状态
cat /sys/class/bpu/bpu0/status
```

**修复**：
- 确认使用正确平台的模型文件
- 重启开发板
- 检查是否使用官方系统镜像

---

### 5. Topic 无数据

**现象**：
```bash
ros2 topic echo /hobot_mono2d_body_detection
# 无输出
```

**原因**：上游节点未发布图像或 topic 名称不匹配

**排查**：
```bash
ros2 topic hz /hbmem_img
# 检查图像 topic 频率

ros2 topic info /hbmem_img
# 检查发布者数量
```

**修复**：
- 确认相机节点正常运行
- 检查 `is_shared_mem_sub` 参数与实际输入是否匹配

---

### 6. 图像分辨率不匹配

**现象**：
```
[WARN] Input image size mismatch
```

**原因**：输入图像分辨率与模型期望不一致

**排查**：
```bash
ros2 param get /mono2d_body_det model_input_width
ros2 param get /mono2d_body_det model_input_height
```

**修复**：
- fasterRcnn 模型需要 **960x544**
- yolo-pose 模型需要 **640x640**
- 调整相机或 codec 输出分辨率

---

### 7. 依赖包缺失

**现象**：
```
Package 'dnn_node' not found
```

**原因**：未正确 source tros 环境

**排查**：
```bash
echo $AMENT_PREFIX_PATH
# 检查是否包含 /opt/tros/humble
```

**修复**：
```bash
source /opt/tros/humble/setup.bash
```

---

### 8. Web 页面无法访问

**现象**：浏览器访问 `http://IP:8000` 无响应

**原因**：websocket 节点未启动或网络不通

**排查**：
```bash
ros2 node list | grep websocket
netstat -tlnp | grep 8000
ping <RDK_IP>
```

**修复**：
- 确认 websocket 节点在运行
- 检查防火墙设置
- 确认 PC 和 RDK 在同一网段

---

### 9. 跟踪 ID 不稳定

**现象**：同一人的 track_id 频繁变化

**原因**：跟踪参数不适合当前场景

**排查**：查看 `config/iou2_method_param.json`

**修复**：调整跟踪参数
```json
{
  "iou_thres": 0.3,
  "missing_time_thres": 3,
  "vanish_frame_count": 80
}
```

---

### 10. 编译错误：平台未定义

**现象**：
```
invalid platform, build platform X3 default
```

**原因**：未指定 `-DPLATFORM_XXX` 编译选项

**排查**：检查 colcon build 命令

**修复**：
```bash
# 根据实际平台添加正确的 cmake 参数
colcon build --cmake-args -DPLATFORM_X5=ON
```

---

## 六、关键文件位置速查

| 文件/目录 | 路径 | 用途 |
|----------|------|------|
| 模型文件 | `config/*.hbm` | BPU 推理模型 |
| 跟踪配置 | `config/iou2_method_param.json` | MOT 参数 |
| Launch 文件 | `launch/mono2d_body_detection.launch.py` | 启动配置 |
| 主节点源码 | `src/mono2d_body_det_node.cpp` | 核心逻辑 |
| 日志位置 | `~/.ros/log/` | ROS2 日志 |
| 安装位置 | `/opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/` | apt 安装路径 |

---

## 七、快速启动脚本

创建 `run_body_det.sh`：

```bash
#!/bin/bash
# 一键启动脚本

# 配置环境
source /opt/tros/humble/setup.bash

# 拷贝配置（如果不存在）
if [ ! -d "config" ]; then
    cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .
fi

# 设置相机类型（可选：mipi/usb/fb）
export CAM_TYPE=${1:-mipi}

echo "Starting mono2d_body_detection with CAM_TYPE=$CAM_TYPE"

# 启动
ros2 launch mono2d_body_detection mono2d_body_detection.launch.py
```

**使用方法**：
```bash
chmod +x run_body_det.sh
./run_body_det.sh mipi  # MIPI 相机
./run_body_det.sh usb   # USB 相机
./run_body_det.sh fb    # 本地图片
```

---

## 八、参数速查表

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `is_sync_mode` | int | 0 | 0=异步推理，1=同步推理 |
| `model_file_name` | string | config/multitask...hbm | 模型文件路径 |
| `is_shared_mem_sub` | int | 1 | 1=共享内存订阅，0=ROS订阅 |
| `ai_msg_pub_topic_name` | string | /hobot_mono2d_body_detection | 输出topic名 |
| `ros_img_topic_name` | string | /image_raw | ROS图像topic名 |
| `image_gap` | int | 1 | 抽帧间隔，1=每帧处理 |
| `dump_render_img` | int | 0 | 1=保存渲染图 |
| `model_type` | int | 0 | 0=fasterRcnn，1=yolo-pose |
| `track_mode` | int | 1 | 0=不跟踪，1=跟踪 |

---

*文档生成时间：2026-01-27*

# OrbbecSDK_ROS2 代码走读 + 架构复盘

> 本文档基于仓库源码分析，帮助开发者在不看源码的情况下理解并改造 OrbbecSDK_ROS2。

---

## 目录

1. [仓库地图](#1-仓库地图)
2. [Package 清单](#2-package-清单)
3. [ROS2 通信接口表](#3-ros2-通信接口表)
4. [启动链路](#4-启动链路)
5. [相机SDK封装层](#5-相机sdk封装层)
6. [图像/深度/点云处理](#6-图像深度点云处理)
7. [参数体系](#7-参数体系)
8. [Build & Run 指南](#8-build--run-指南)
9. [人眼定位算法定制改造方案](#9-人眼定位算法定制改造方案)

---

## 1. 仓库地图

```
OrbbecSDK_ROS2/
├── .github/                          # GitHub CI/Issue模板
├── orbbec_camera/                    # 【核心驱动包】相机驱动主包
│   ├── cmake/                        # CMake查找脚本 (FindORBBEC_SDK.cmake)
│   ├── config/                       # 参数配置文件
│   │   ├── common.yaml               # 通用参数模板
│   │   ├── gemini330_series.yaml     # 各型号相机专用配置
│   │   ├── depthfilter/              # 深度滤波配置JSON
│   │   └── tools/                    # 工具配置
│   ├── examples/                     # 示例launch和代码
│   │   ├── benchmark/                # 性能测试
│   │   ├── gmsl_camera/              # GMSL相机示例
│   │   ├── multi_camera_synced_verification_tool/  # 多相机同步验证
│   │   └── multi_camera_time_sync/   # 时间同步示例
│   ├── include/orbbec_camera/        # C++头文件
│   ├── launch/                       # ROS2 launch文件
│   ├── SDK/                          # Orbbec SDK库文件
│   │   ├── include/libobsensor/      # SDK头文件 (C/C++ API)
│   │   └── lib/{arm64,x64}/          # 预编译库 (.so)
│   ├── scripts/                      # 辅助脚本 (udev规则、benchmark等)
│   ├── src/                          # C++源文件
│   └── tools/                        # 工具节点源码
├── orbbec_camera_msgs/               # 【消息包】自定义消息/服务定义
│   ├── msg/                          # 消息定义
│   └── srv/                          # 服务定义
└── orbbec_description/               # 【描述包】URDF模型和可视化
    ├── meshes/                       # 各型号相机3D模型 (.STL)
    ├── urdf/                         # URDF/xacro文件
    └── launch/                       # 模型可视化launch
```

### 目录职责说明

| 目录 | 职责 |
|------|------|
| `orbbec_camera/src/` | 核心驱动实现，包括设备管理、流控制、图像发布 |
| `orbbec_camera/include/` | C++头文件，定义类接口 |
| `orbbec_camera/config/` | YAML参数配置，支持不同相机型号 |
| `orbbec_camera/launch/` | ROS2启动文件，支持单/多相机场景 |
| `orbbec_camera/SDK/` | Orbbec官方SDK，包含头文件和预编译库 |
| `orbbec_camera/tools/` | 辅助工具节点（设备列表、性能测试等） |
| `orbbec_camera_msgs/` | 自定义ROS2消息和服务类型 |
| `orbbec_description/` | 相机URDF模型，用于可视化和TF |

---

## 2. Package 清单

### 2.1 orbbec_camera (核心驱动包)

| 项目 | 内容 |
|------|------|
| **用途** | Orbbec深度相机ROS2驱动，负责设备管理、取流、发布图像/点云/IMU |
| **语言** | C++ (C++17) |
| **依赖** | rclcpp, sensor_msgs, cv_bridge, image_transport, tf2_ros, OpenCV, Eigen3, yaml-cpp, diagnostic_updater |
| **可执行文件** | `orbbec_camera_node` (主节点), `list_devices_node`, `list_depth_work_mode_node`, `topic_statistics_node`, `ob_benchmark_node` 等 |
| **库** | `liborbbec_camera.so` (组件库), `libframe_latency.so`, `libstart_benchmark.so` |
| **Launch** | `orbbec_camera.launch.py` (通用), `gemini_330_series.launch.py`, `multi_camera.launch.py` 等 |
| **参数文件** | `config/common.yaml`, `config/gemini330_series.yaml` 等 |

#### 源文件职责

| 文件 | 职责 |
|------|------|
| `ob_camera_node_driver.cpp` | 设备发现、连接、生命周期管理 |
| `ob_camera_node.cpp` | 流配置、帧回调、图像/点云发布 |
| `ros_service.cpp` | ROS服务接口实现 |
| `image_publisher.cpp` | 图像发布封装 |
| `utils.cpp` | 工具函数 (格式转换、点云保存等) |
| `synced_imu_publisher.cpp` | IMU数据同步发布 |
| `d2c_viewer.cpp` | 深度-彩色对齐可视化 |
| `jpeg_decoder.cpp` | MJPEG软解码 |
| `rk_mpp_decoder.cpp` | RK硬件解码 (可选) |
| `jetson_nv_decoder.cpp` | Jetson硬件解码 (可选) |

### 2.2 orbbec_camera_msgs (消息包)

| 项目 | 内容 |
|------|------|
| **用途** | 自定义消息和服务类型定义 |
| **语言** | ROS2 IDL |
| **依赖** | std_msgs, sensor_msgs, rosidl_default_generators |

#### 消息定义

| 消息 | 字段 |
|------|------|
| `RGBD.msg` | header, rgb_camera_info, depth_camera_info, rgb, depth |
| `DeviceInfo.msg` | name, serial_number, firmware_version, hardware_version |
| `Metadata.msg` | header, json_data |
| `Extrinsics.msg` | 外参矩阵 |
| `IMUInfo.msg` | IMU信息 |

#### 服务定义

| 服务 | 用途 |
|------|------|
| `GetInt32.srv` / `SetInt32.srv` | 整数参数读写 |
| `GetBool.srv` / `SetBool.srv` | 布尔参数读写 |
| `SetFilter.srv` | 滤波器配置 |
| `GetDeviceInfo.srv` | 获取设备信息 |
| `GetCameraInfo.srv` | 获取相机内参 |

### 2.3 orbbec_description (描述包)

| 项目 | 内容 |
|------|------|
| **用途** | 相机URDF模型，用于RViz可视化和TF树 |
| **语言** | URDF/xacro |
| **支持型号** | Gemini335/336/335L/336L/335Lg, Gemini2/2L, Astra2, Femto Bolt |

---

## 3. ROS2 通信接口表

### 3.1 Topics

| Topic名称 | 类型 | 频率 | frame_id | QoS | 发布节点 |
|-----------|------|------|----------|-----|----------|
| `/<camera_name>/color/image_raw` | sensor_msgs/Image | 30Hz | `<camera_name>_color_optical_frame` | 可配置(default) | OBCameraNode |
| `/<camera_name>/color/camera_info` | sensor_msgs/CameraInfo | 30Hz | 同上 | 可配置 | OBCameraNode |
| `/<camera_name>/depth/image_raw` | sensor_msgs/Image | 30Hz | `<camera_name>_depth_optical_frame` | 可配置 | OBCameraNode |
| `/<camera_name>/depth/camera_info` | sensor_msgs/CameraInfo | 30Hz | 同上 | 可配置 | OBCameraNode |
| `/<camera_name>/left_ir/image_raw` | sensor_msgs/Image | 30Hz | `<camera_name>_left_ir_optical_frame` | 可配置 | OBCameraNode |
| `/<camera_name>/right_ir/image_raw` | sensor_msgs/Image | 30Hz | `<camera_name>_right_ir_optical_frame` | 可配置 | OBCameraNode |
| `/<camera_name>/depth/points` | sensor_msgs/PointCloud2 | 30Hz | `<camera_name>_depth_optical_frame` | 可配置 | OBCameraNode |
| `/<camera_name>/depth_registered/points` | sensor_msgs/PointCloud2 | 30Hz | `<camera_name>_color_optical_frame` | 可配置 | OBCameraNode |
| `/<camera_name>/imu/sample` | sensor_msgs/Imu | 200Hz | `<camera_name>_imu_frame` | 可配置 | OBCameraNode |
| `/<camera_name>/accel/sample` | sensor_msgs/Imu | 可配置 | `<camera_name>_accel_frame` | 可配置 | OBCameraNode |
| `/<camera_name>/gyro/sample` | sensor_msgs/Imu | 可配置 | `<camera_name>_gyro_frame` | 可配置 | OBCameraNode |
| `/<camera_name>/color/metadata` | orbbec_camera_msgs/Metadata | 30Hz | - | 可配置 | OBCameraNode |
| `/<camera_name>/device_status` | orbbec_camera_msgs/DeviceStatus | 1Hz | - | transient_local | OBCameraNodeDriver |

### 3.2 Services

| 服务名称 | 类型 | 请求/响应要点 |
|----------|------|---------------|
| `get_color_exposure` | GetInt32 | 响应: data(曝光值), success, message |
| `set_color_exposure` | SetInt32 | 请求: data(曝光值) |
| `get_color_gain` | GetInt32 | 响应: data(增益值) |
| `set_color_gain` | SetInt32 | 请求: data(增益值) |
| `set_color_auto_exposure` | std_srvs/SetBool | 请求: data(true/false) |
| `set_color_mirror` | std_srvs/SetBool | 镜像开关 |
| `set_color_flip` | std_srvs/SetBool | 翻转开关 |
| `set_laser_enable` | std_srvs/SetBool | 激光开关 |
| `set_ldp_enable` | std_srvs/SetBool | LDP开关 |
| `get_device_info` | GetDeviceInfo | 获取设备序列号、固件版本等 |
| `toggle_color` | std_srvs/SetBool | 动态开关color流 |
| `toggle_depth` | std_srvs/SetBool | 动态开关depth流 |
| `reboot_device` | std_srvs/Empty | 重启设备 |
| `save_images` | std_srvs/Empty | 保存当前帧图像 |
| `save_point_cloud` | std_srvs/Empty | 保存点云到PLY |

### 3.3 TF坐标系

**坐标系层级** (以camera_name=camera为例):

```
camera_link (base)
├── camera_depth_frame
│   └── camera_depth_optical_frame
├── camera_color_frame
│   └── camera_color_optical_frame
├── camera_left_ir_frame
│   └── camera_left_ir_optical_frame
├── camera_right_ir_frame
│   └── camera_right_ir_optical_frame
└── camera_imu_frame
```

**TF发布机制**:
- **位置**: `ob_camera_node.cpp:4205` `publishStaticTransforms()`
- **外参来源**: SDK `getExtrinsicTo()` 获取各传感器间外参
- **更新频率**: 默认静态TF (tf_publish_rate=0)，可配置动态发布

---

## 4. 启动链路

### 4.1 Launch → 可执行文件

```
orbbec_camera.launch.py
    │
    ├─ 加载参数: common.yaml + {camera_model}.yaml + 命令行参数
    │
    └─ 创建节点:
        ├─ [Foxy] Node(executable="orbbec_camera_node")
        └─ [Humble+] LoadComposableNodes → ComposableNode(plugin="orbbec_camera::OBCameraNodeDriver")
```

**关键代码** (`orbbec_camera.launch.py:114-179`):

```python
def create_node_action(context, args):
    params = get_params(context, args)  # 合并yaml和命令行参数
    camera_name = find_camera_name(params)

    if ros_distro == "foxy":
        return [Node(package='orbbec_camera', executable='orbbec_camera_node', ...)]
    else:
        composable_node = ComposableNode(plugin='orbbec_camera::OBCameraNodeDriver', ...)
        return [LoadComposableNodes(composable_node_descriptions=[composable_node])]
```

### 4.2 可执行文件入口 → 驱动初始化

```
main() [由rclcpp_components自动生成]
    │
    └─ OBCameraNodeDriver::OBCameraNodeDriver() [ob_camera_node_driver.cpp:99]
        │
        ├─ init() [ob_camera_node_driver.cpp:196]
        │   ├─ 设置信号处理器
        │   ├─ ob::Context::setExtensionsDirectory()
        │   ├─ ctx_ = std::make_unique<ob::Context>(config_path_)
        │   ├─ ctx_->registerDeviceChangedCallback() → onDeviceConnected/Disconnected
        │   ├─ check_connect_timer_ (1Hz轮询设备)
        │   └─ query_thread_ → queryDevice()
        │
        └─ onDeviceConnected() → startDevice()
            │
            └─ initializeDevice() → OBCameraNode构造
```

### 4.3 主循环 (帧回调)

```
OBCameraNode::startStreams() [ob_camera_node.cpp:1620]
    │
    ├─ pipeline_->start(pipeline_config_, callback)
    │
    └─ callback = [](std::shared_ptr<ob::FrameSet> frame_set) {
           onNewFrameSetCallback(frame_set);  // [ob_camera_node.cpp:3138]
       }
           │
           ├─ publishStaticTransforms() (首帧时)
           ├─ 获取各类型帧: depth_frame, color_frame, ir_frame...
           ├─ processDepthFrameFilter() / processColorFrameFilter()
           ├─ onNewFrameCallback() → 发布图像
           └─ publishPointCloud() → 发布点云
```

### 4.4 关键函数调用链

| 阶段 | 函数 | 文件:行号 |
|------|------|-----------|
| 节点构造 | `OBCameraNodeDriver()` | `ob_camera_node_driver.cpp:99` |
| 初始化 | `init()` | `ob_camera_node_driver.cpp:196` |
| 设备连接 | `onDeviceConnected()` | `ob_camera_node_driver.cpp:319` |
| 启动设备 | `startDevice()` | `ob_camera_node_driver.cpp` |
| 相机节点构造 | `OBCameraNode()` | `ob_camera_node.cpp:40` |
| 配置Topic | `setupTopics()` | `ob_camera_node.cpp` |
| 启动流 | `startStreams()` | `ob_camera_node.cpp:1620` |
| 帧回调 | `onNewFrameSetCallback()` | `ob_camera_node.cpp:3138` |
| 图像发布 | `onNewFrameCallback()` | `ob_camera_node.cpp` |
| 点云发布 | `publishPointCloud()` | `ob_camera_node.cpp:2653` |

---

## 5. 相机SDK封装层

### 5.1 SDK调用集中文件

| 文件 | SDK调用内容 |
|------|-------------|
| `ob_camera_node_driver.cpp` | Context创建、设备枚举、连接/断开回调 |
| `ob_camera_node.cpp` | Pipeline配置、流启停、帧获取、滤波器、点云生成 |
| `utils.cpp` | 格式转换、内参/外参获取 |

### 5.2 关键操作实现

#### 设备发现

**位置**: `ob_camera_node_driver.cpp:229-233`

```cpp
ctx_ = std::make_unique<ob::Context>(config_path_.c_str());
ctx_->enableNetDeviceEnumeration(enumerate_net_device_);
device_changed_callback_id_ = ctx_->registerDeviceChangedCallback(
    [this](const std::shared_ptr<ob::DeviceList> &removed_list,
           const std::shared_ptr<ob::DeviceList> &added_list) {
        onDeviceDisconnected(removed_list);
        onDeviceConnected(added_list);
    });
```

#### 设备打开

**位置**: `ob_camera_node_driver.cpp:319-347`

```cpp
void OBCameraNodeDriver::onDeviceConnected(const std::shared_ptr<ob::DeviceList> &device_list) {
    if (!device_) {
        startDevice(device_list);  // 选择设备并初始化
    }
}
```

#### 设备关闭

**位置**: `ob_camera_node.cpp:159-260`

```cpp
void OBCameraNode::clean() noexcept {
    is_running_.store(false);
    stopStreams();
    stopIMU();
    // 清理缓冲区和解码器
    delete[] rgb_buffer_;
    jpeg_decoder_.reset();
}
```

#### 取流

**位置**: `ob_camera_node.cpp:1620-1640`

```cpp
void OBCameraNode::startStreams() {
    pipeline_->start(pipeline_config_, [this](std::shared_ptr<ob::FrameSet> frame_set) {
        onNewFrameSetCallback(frame_set);
    });
}
```

#### 深度对齐

**位置**: `ob_camera_node.cpp` 滤波器配置

```cpp
// 硬件对齐
align_filter_.setAlignMode(ALIGN_D2C_HW_MODE);
// 软件对齐
align_filter_.setAlignMode(ALIGN_D2C_SW_MODE);
```

#### 深度转点云

**位置**: `ob_camera_node.cpp:2674-2732`

```cpp
void OBCameraNode::publishDepthPointCloud(const std::shared_ptr<ob::FrameSet> &frame_set) {
    auto camera_params = pipeline_->getCameraParam();
    depth_point_cloud_filter_.setCameraParam(camera_params);
    depth_point_cloud_filter_.setPositionDataScaled(depth_scale);
    depth_point_cloud_filter_.setCreatePointFormat(OB_FORMAT_POINT);
    auto result_frame = depth_point_cloud_filter_.process(depth_frame);
    // 转换为PointCloud2消息并发布
}
```

---

## 6. 图像/深度/点云处理

### 6.1 格式转换

**位置**: `ob_camera_node.cpp` 中的 `setupFormatConvertType()` 和帧回调

| 源格式 | 目标格式 | 实现位置 |
|--------|----------|----------|
| MJPG → RGB | cv::imdecode 或硬件解码 | `softwareDecodeColorFrame()` |
| YUYV → RGB | OpenCV cvtColor | `onNewColorFrameCallback()` |
| Y16 → mono16 | 直接映射 | `onNewFrameCallback()` |
| Depth → PointCloud2 | SDK PointCloudFilter | `publishDepthPointCloud()` |

**MJPEG解码** (`ob_camera_node.cpp:3050-3100`):

```cpp
void OBCameraNode::softwareDecodeColorFrame(const std::shared_ptr<ob::Frame> &frame) {
#if defined(USE_RK_HW_DECODER)
    jpeg_decoder_->decode(frame->getData(), frame->getDataSize(), rgb_buffer_);
#elif defined(USE_NV_HW_DECODER)
    jpeg_decoder_->decode(frame->getData(), frame->getDataSize(), rgb_buffer_);
#else
    cv::Mat raw_mat(1, frame->getDataSize(), CV_8UC1, frame->getData());
    cv::Mat rgb_mat = cv::imdecode(raw_mat, cv::IMREAD_COLOR);
#endif
}
```

### 6.2 时间戳同步

**时间域选择** (`common.yaml:29`):

```yaml
time_domain: "device"  # 可选: device, global, system
```

**实现** (`ob_camera_node.cpp` `getFrameTimestampUs()`):
- `device`: 使用设备硬件时间戳
- `global`: 使用全局同步时间戳
- `system`: 使用系统时间戳

**主机时间同步** (`ob_camera_node_driver.cpp:238`):

```cpp
enable_sync_host_time_ = declare_parameter<bool>("enable_sync_host_time", true);
```

### 6.3 深度滤波器

**位置**: `ob_camera_node.cpp` `setupDepthPostProcessFilter()` 和 `processDepthFrameFilter()`

| 滤波器 | 参数 | 作用 |
|--------|------|------|
| DecimationFilter | scale | 降采样 |
| NoiseRemovalFilter | min_diff, max_size | 去噪 |
| ThresholdFilter | min, max | 深度范围过滤 |
| SpatialFilter | alpha, diff_threshold | 空间平滑 |
| TemporalFilter | diff_threshold, weight | 时间平滑 |
| HoleFillingFilter | mode | 空洞填充 |
| HDRMergeFilter | exposure_1/2, gain_1/2 | HDR合并 |

### 6.4 性能瓶颈分析

| 瓶颈点 | 位置 | 原因 | 优化建议 |
|--------|------|------|----------|
| **MJPEG软解码** | `softwareDecodeColorFrame()` | CPU密集 | 使用硬件解码 |
| **点云生成** | `publishDepthPointCloud()` | 逐点计算 | 使用decimation降低分辨率 |
| **多滤波器串联** | `processDepthFrameFilter()` | 多次内存拷贝 | 减少滤波器数量 |
| **TF动态发布** | `publishDynamicTransforms()` | 高频率CPU负载 | 使用静态TF |

---

## 7. 参数体系

### 7.1 参数声明位置

| 文件 | 参数类型 |
|------|----------|
| `ob_camera_node_driver.cpp:196-317` | 设备连接参数 (serial_number, usb_port, net_device_ip) |
| `ob_camera_node.cpp` `getParameters()` | 流参数 (enable_*, width, height, fps, format) |
| `config/common.yaml` | 默认参数模板 |
| `config/{camera_model}.yaml` | 型号专用参数 |

### 7.2 参数加载优先级

```
命令行参数 > yaml配置 > 代码默认值
```

**实现** (`orbbec_camera.launch.py:62-104`):

```python
yaml_common_params = load_common_yaml_params()
yaml_params = load_config_yaml_params(config_file_path, camera_model)
yaml_params = update_params(yaml_common_params, yaml_params)
cmd_params = dict([a for a in [a.split(':=') for a in sys.argv] if len(a) == 2])
default_params = update_params(default_params, cmd_params)
```

### 7.3 动态参数更新

**支持动态更新的参数** (通过ROS2 parameter callback):
- 曝光/增益: 通过服务接口 `set_color_exposure` 等
- 滤波器参数: 通过 `set_filter` 服务

**触发流重启的参数**:
- `enable_color`, `enable_depth`, `enable_left_ir`, `enable_right_ir`
- `color_width`, `color_height`, `color_fps`, `color_format`
- `depth_width`, `depth_height`, `depth_fps`, `depth_format`
- `depth_registration`, `align_mode`
- `sync_mode`

### 7.4 常用参数速查表

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `camera_name` | "camera" | 相机命名空间 |
| `serial_number` | "" | 指定设备序列号 |
| `usb_port` | "" | 指定USB端口 |
| `enable_color` | true | 启用彩色流 |
| `enable_depth` | true | 启用深度流 |
| `color_width/height/fps` | 0 | 0表示自动选择 |
| `depth_registration` | true | 深度对齐到彩色 |
| `align_mode` | "SW" | 对齐模式: HW/SW |
| `enable_point_cloud` | true | 启用点云 |
| `enable_colored_point_cloud` | false | 启用彩色点云 |
| `publish_tf` | true | 发布TF |
| `time_domain` | "device" | 时间戳来源 |

---

## 8. Build & Run 指南

### 8.1 依赖安装

```bash
# ROS2基础依赖
sudo apt install ros-humble-cv-bridge ros-humble-image-transport \
    ros-humble-tf2-ros ros-humble-diagnostic-updater \
    ros-humble-camera-info-manager ros-humble-image-publisher

# OpenCV
sudo apt install libopencv-dev

# Eigen3
sudo apt install libeigen3-dev

# yaml-cpp
sudo apt install libyaml-cpp-dev

# udev规则 (Linux)
cd OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 8.2 构建命令

```bash
# 进入工作空间
cd ~/ros2_ws
ln -s /path/to/OrbbecSDK_ROS2 src/OrbbecSDK_ROS2

# 分步构建 (推荐)
colcon build --packages-select orbbec_camera_msgs
colcon build --packages-select orbbec_camera orbbec_description

# 或一次性构建
colcon build --symlink-install

# source环境
source install/setup.bash
```

### 8.3 运行命令

```bash
# 单相机启动 (Gemini 330系列)
ros2 launch orbbec_camera gemini_330_series.launch.py

# 通用启动 (指定型号)
ros2 launch orbbec_camera orbbec_camera.launch.py camera_model:=gemini330_series

# 指定序列号
ros2 launch orbbec_camera orbbec_camera.launch.py serial_number:=CP1234567890

# 多相机启动
ros2 launch orbbec_camera multi_camera.launch.py

# 启用点云
ros2 launch orbbec_camera gemini_330_series.launch.py enable_point_cloud:=true

# 启用深度对齐
ros2 launch orbbec_camera gemini_330_series.launch.py depth_registration:=true

# 调试模式
ros2 launch orbbec_camera gemini_330_series.launch.py log_level:=debug
```

### 8.4 环境变量

```bash
# SDK日志路径
export OB_LOG_PATH=~/orbbec_logs

# 扩展库路径 (自动设置)
# ${install_prefix}/lib/extensions
```

### 8.5 常见5类问题排查

#### 问题1: 设备未识别

| 排查步骤 | 命令 |
|----------|------|
| 检查udev规则 | `ls /etc/udev/rules.d/99-obsensor*` |
| 检查USB连接 | `lsusb \| grep 2bc5` |
| 查看详细日志 | `ros2 launch ... log_level:=debug` |
| 列出设备 | `ros2 run orbbec_camera list_devices_node` |

#### 问题2: 图像不出

| 排查步骤 | 说明 |
|----------|------|
| 检查enable参数 | `enable_color:=true` |
| 检查分辨率支持 | `ros2 run orbbec_camera list_camera_profile_mode_node` |
| 检查topic | `ros2 topic list` |
| 检查订阅数 | `ros2 topic info /camera/color/image_raw` |

#### 问题3: 点云异常

| 排查步骤 | 说明 |
|----------|------|
| 确认depth_registration | 彩色点云需要开启 |
| 检查相机内参 | `ros2 topic echo /camera/depth/camera_info` |
| 检查深度范围 | `threshold_filter_min/max` |
| 检查点云topic | `ros2 topic echo /camera/depth/points --no-arr` |

#### 问题4: 多相机冲突

| 排查步骤 | 说明 |
|----------|------|
| 指定usb_port | 每个相机指定不同端口 |
| 指定serial_number | 使用序列号区分 |
| 检查sync_mode | 主从模式配置 |
| 增加connection_delay | 避免同时初始化 |

#### 问题5: 性能问题

| 排查步骤 | 说明 |
|----------|------|
| 降低分辨率/帧率 | 减少数据量 |
| 启用硬件解码 | `USE_RK_HW_DECODER=ON` |
| 减少滤波器 | 关闭不必要的滤波 |
| 使用intra_process | `use_intra_process_comms:=true` |

---

## 9. 人眼定位算法定制改造方案

### 方案A: 订阅image topic做算法节点 (解耦)

#### 架构图

```
[orbbec_camera_node] --/camera/color/image_raw--> [eye_localization_node]
                     --/camera/depth/image_raw-->       |
                     --/camera/color/camera_info-->     v
                                                  /eye_position (自定义消息)
```

#### 改动文件

**新建包**: `eye_localization/`

```
eye_localization/
├── CMakeLists.txt
├── package.xml
├── include/eye_localization/
│   └── eye_localization_node.h
├── src/
│   └── eye_localization_node.cpp
├── launch/
│   └── eye_localization.launch.py
└── config/
    └── eye_localization_params.yaml
```

#### 新增消息 (在orbbec_camera_msgs中)

```
# msg/EyePosition.msg
std_msgs/Header header
geometry_msgs/Point left_eye    # 左眼3D坐标
geometry_msgs/Point right_eye   # 右眼3D坐标
float32 confidence              # 置信度
bool detected                   # 是否检测到
```

#### 节点设计

```cpp
class EyeLocalizationNode : public rclcpp::Node {
public:
    EyeLocalizationNode();

private:
    // 订阅
    image_transport::Subscriber color_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    // 发布
    rclcpp::Publisher<eye_localization_msgs::msg::EyePosition>::SharedPtr eye_pub_;

    // 回调
    void colorCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

    // 算法
    void detectEyes(const cv::Mat &color_image, const cv::Mat &depth_image);
};
```

#### 参数设计

```yaml
# config/eye_localization_params.yaml
eye_localization:
  ros__parameters:
    # 输入topic
    color_topic: "/camera/color/image_raw"
    depth_topic: "/camera/depth/image_raw"
    camera_info_topic: "/camera/color/camera_info"

    # 算法参数
    detection_model_path: "/path/to/model"
    confidence_threshold: 0.5
    max_detection_distance: 2.0  # 米

    # 输出
    output_topic: "/eye_position"
    publish_debug_image: true
```

#### 延迟与风险

| 项目 | 说明 |
|------|------|
| **延迟** | +5-15ms (topic传输 + 消息序列化) |
| **优点** | 完全解耦，不修改驱动代码，易于调试和替换 |
| **风险** | 图像拷贝开销，时间同步需额外处理 |
| **适用场景** | 算法迭代频繁，对延迟要求不苛刻 |

---

### 方案B: 驱动内挂载 (低延迟)

#### 架构图

```
[orbbec_camera_node]
    │
    └─ onNewFrameSetCallback()
           │
           ├─ 原有处理流程
           │
           └─ [新增] EyeLocalizationProcessor::process()
                      │
                      └─ 发布 /eye_position
```

#### 改动文件

| 文件 | 改动内容 |
|------|----------|
| `orbbec_camera/include/orbbec_camera/eye_localization_processor.h` | **新增** 算法处理器头文件 |
| `orbbec_camera/src/eye_localization_processor.cpp` | **新增** 算法处理器实现 |
| `orbbec_camera/include/orbbec_camera/ob_camera_node.h` | **修改** 添加处理器成员变量 |
| `orbbec_camera/src/ob_camera_node.cpp` | **修改** 在帧回调中调用处理器 |
| `orbbec_camera/CMakeLists.txt` | **修改** 添加新源文件 |
| `orbbec_camera_msgs/msg/EyePosition.msg` | **新增** 消息定义 |

#### 代码示例

**新增头文件** `eye_localization_processor.h`:

```cpp
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "orbbec_camera_msgs/msg/eye_position.hpp"

namespace orbbec_camera {

class EyeLocalizationProcessor {
public:
    EyeLocalizationProcessor(rclcpp::Node* node);
    void process(const std::shared_ptr<ob::Frame>& color_frame,
                 const std::shared_ptr<ob::Frame>& depth_frame);

private:
    rclcpp::Node* node_;
    rclcpp::Publisher<orbbec_camera_msgs::msg::EyePosition>::SharedPtr eye_pub_;

    // 算法参数
    bool enable_eye_detection_;
    float confidence_threshold_;
};

}  // namespace orbbec_camera
```

**修改帧回调** `ob_camera_node.cpp:3138`:

```cpp
void OBCameraNode::onNewFrameSetCallback(std::shared_ptr<ob::FrameSet> frame_set) {
    // ... 原有代码 ...

    // 新增: 人眼定位处理
    if (eye_localization_processor_ && enable_eye_detection_) {
        auto color_frame = frame_set->getFrame(OB_FRAME_COLOR);
        auto depth_frame = frame_set->getFrame(OB_FRAME_DEPTH);
        if (color_frame && depth_frame) {
            eye_localization_processor_->process(color_frame, depth_frame);
        }
    }

    // ... 原有代码继续 ...
}
```

#### 参数设计

```yaml
# 在 common.yaml 中添加
eye_localization:
  enable_eye_detection: true
  model_path: "/path/to/model"
  confidence_threshold: 0.5
  max_distance: 2.0
```

#### 延迟与风险

| 项目 | 说明 |
|------|------|
| **延迟** | +1-3ms (无topic传输开销) |
| **优点** | 最低延迟，直接访问原始帧数据 |
| **风险** | 与驱动代码耦合，升级维护困难 |
| **适用场景** | 对延迟要求极高，算法稳定 |

---

### 方案对比总结

| 对比项 | 方案A (解耦) | 方案B (内嵌) |
|--------|-------------|-------------|
| **延迟** | +5-15ms | +1-3ms |
| **代码侵入性** | 无 | 高 |
| **维护难度** | 低 | 高 |
| **调试便利性** | 高 | 低 |
| **升级兼容性** | 好 | 差 |
| **推荐场景** | 算法开发期 | 产品部署期 |

### 推荐策略

1. **开发阶段**: 使用方案A，快速迭代算法
2. **优化阶段**: 如延迟不满足要求，迁移到方案B
3. **混合方案**: 方案A用于调试，方案B用于生产

---

## 附录

### A. 支持的相机型号

| 型号 | PID | 特性 |
|------|-----|------|
| Gemini 335/336 | 0x0800/0x0803 | 双目IR |
| Gemini 335L/336L | 0x0804/0x0807 | 双目IR + 激光 |
| Gemini 335Le | 0x080E | 嵌入式版本 |
| Gemini 2/2L | 0x0670/0x0673 | ToF |
| Femto Bolt | 0x066b | Azure Kinect兼容 |
| Femto Mega | 0x0669 | 网络相机 |

### B. 参考文档

- Orbbec SDK文档: `SDK/include/libobsensor/`
- ROS2官方文档: https://docs.ros.org/
- image_transport: http://wiki.ros.org/image_transport

---

*文档生成时间: 2026-01-27*
*基于 OrbbecSDK_ROS2 v2.7.2*


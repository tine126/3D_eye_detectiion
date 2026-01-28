# OrbbecSDK_ROS2 项目工程分析文档

> 文档版本: 1.0
> 分析日期: 2026-01-27
> 目标: TCL人眼定位算法定制项目

---

## 目录

1. [项目概览](#一项目概览)
2. [整体架构](#二整体架构)
3. [Package/模块详解](#三package模块详解)
4. [关键数据流](#四关键数据流)
5. [核心代码入口与流程](#五核心代码入口与流程)
6. [配置与参数](#六配置与参数)
7. [编译与运行](#七编译与运行)
8. [二次开发指南](#八二次开发指南-tcl人眼定位算法定制)

---

## 一、项目概览

### 1.1 项目目标

**解决的问题：** 将 Orbbec（奥比中光）系列深度相机接入 ROS2 生态系统，提供标准化的图像/深度/点云/IMU 数据发布能力。

**在ROS2中提供的能力：**

| 能力 | 说明 |
|------|------|
| RGB彩色图像流 | 支持多种分辨率和格式 |
| Depth深度图像流 | 16位深度数据，单位mm |
| IR红外图像流 | 左/右双目红外 |
| 点云数据 | 普通点云/彩色点云 |
| IMU数据 | 加速度计/陀螺仪 |
| 深度-彩色对齐(D2C) | 硬件/软件对齐模式 |
| 多相机同步 | 主从同步、硬件触发 |
| TF坐标变换 | 自动发布相机坐标系 |

### 1.2 支持的相机型号

- Gemini 330/335/336 系列
- Gemini 2/2L/2XL
- Femto Bolt/Mega
- Astra 2
- Dabai 系列
- 其他 Orbbec 相机

### 1.3 项目版本

```
OB_ROS_MAJOR_VERSION: 2
OB_ROS_MINOR_VERSION: 7
OB_ROS_PATCH_VERSION: 2
```

---

## 二、整体架构

### 2.1 目录结构

```
OrbbecSDK_ROS2/
├── orbbec_camera/          # 核心功能包 - 相机驱动节点
│   ├── src/                # 源代码
│   ├── include/            # 头文件
│   ├── launch/             # 启动文件
│   ├── config/             # 配置文件
│   ├── SDK/                # OrbbecSDK库
│   ├── tools/              # 工具节点
│   └── examples/           # 示例代码
├── orbbec_camera_msgs/     # 自定义消息/服务定义
│   ├── msg/                # 消息定义
│   └── srv/                # 服务定义
└── orbbec_description/     # URDF模型描述
    ├── urdf/               # URDF文件
    └── meshes/             # 3D模型
```

### 2.2 架构图

```
┌─────────────────────────────────────────────────────────────────┐
│                        ROS2 Ecosystem                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │              OBCameraNodeDriver (主节点)                  │  │
│  │  ┌────────────────────────────────────────────────────┐  │  │
│  │  │                OBCameraNode                        │  │  │
│  │  │  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌──────────┐ │  │  │
│  │  │  │ Color   │ │ Depth   │ │ IR      │ │ IMU      │ │  │  │
│  │  │  │ Stream  │ │ Stream  │ │ Stream  │ │ Stream   │ │  │  │
│  │  │  └────┬────┘ └────┬────┘ └────┬────┘ └────┬─────┘ │  │  │
│  │  │       │           │           │           │        │  │  │
│  │  │       ▼           ▼           ▼           ▼        │  │  │
│  │  │  ┌─────────────────────────────────────────────┐   │  │  │
│  │  │  │           Filter Pipeline                   │   │  │  │
│  │  │  │ (Decimation/Spatial/Temporal/NoiseRemoval)  │   │  │  │
│  │  │  └─────────────────────────────────────────────┘   │  │  │
│  │  └────────────────────────────────────────────────────┘  │  │
│  └──────────────────────────────────────────────────────────┘  │
│                              │                                  │
│                              ▼                                  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                ROS2 Topics/Services                       │  │
│  │  /camera/color/image_raw    /camera/depth/image_raw       │  │
│  │  /camera/depth/points       /camera/imu/sample            │  │
│  └──────────────────────────────────────────────────────────┘  │
└──────────────────────────────┬──────────────────────────────────┘
                               │
                               ▼
┌──────────────────────────────────────────────────────────────────┐
│                      OrbbecSDK (C++ Library)                     │
│  ┌────────────┐  ┌────────────┐  ┌────────────┐                  │
│  │ ob::Context│  │ ob::Device │  │ ob::Pipeline│                 │
│  └────────────┘  └────────────┘  └────────────┘                  │
└──────────────────────────────────────────────────────────────────┘
                               │
                               ▼
┌──────────────────────────────────────────────────────────────────┐
│                    Orbbec Camera Hardware                        │
│  Gemini 330/335/336 | Femto Bolt/Mega | Astra 2 | etc.          │
└──────────────────────────────────────────────────────────────────┘
```

### 2.3 核心类关系

```
OBCameraNodeDriver (rclcpp::Node)
    │
    ├── ob::Context          # SDK上下文
    ├── ob::Device           # 设备句柄
    │
    └── OBCameraNode         # 相机功能实现
            │
            ├── ob::Pipeline         # 数据流管道
            ├── Publishers           # ROS2发布者
            ├── Services             # ROS2服务
            └── Filters              # 滤波器链
```

---

## 三、Package/模块详解

### 3.1 orbbec_camera（核心包）

#### 源代码文件

| 文件路径 | 作用 | 关键类/函数 |
|---------|------|------------|
| `src/ob_camera_node_driver.cpp` | **主入口** - 设备枚举、连接管理、生命周期 | `OBCameraNodeDriver` |
| `src/ob_camera_node.cpp` | **核心逻辑** - 流配置、数据发布、滤波处理 | `OBCameraNode` |
| `src/ros_service.cpp` | ROS2服务实现 | `setupCameraCtrlServices()` |
| `src/image_publisher.cpp` | 图像发布封装 | `ImagePublisher` |
| `src/d2c_viewer.cpp` | 深度-彩色对齐可视化 | `D2CViewer` |
| `src/synced_imu_publisher.cpp` | IMU数据同步发布 | `SyncedImuPublisher` |
| `src/dynamic_params.cpp` | 动态参数处理 | `Parameters` |
| `src/utils.cpp` | 工具函数 | 格式转换、QoS配置 |
| `src/jpeg_decoder.cpp` | MJPEG软解码 | `JpegDecoder` |
| `src/rk_mpp_decoder.cpp` | RK3588硬解码（可选） | `RKJPEGDecoder` |
| `src/jetson_nv_decoder.cpp` | Jetson硬解码（可选） | `JetsonNvJPEGDecoder` |

#### 头文件

| 文件路径 | 作用 |
|---------|------|
| `include/orbbec_camera/ob_camera_node_driver.h` | 驱动节点类声明 |
| `include/orbbec_camera/ob_camera_node.h` | 相机节点类声明 |
| `include/orbbec_camera/constants.h` | 常量定义 |
| `include/orbbec_camera/utils.h` | 工具函数声明 |
| `include/orbbec_camera/dynamic_params.h` | 动态参数类 |
| `include/orbbec_camera/image_publisher.h` | 图像发布器接口 |

#### 启动文件

| 文件路径 | 用途 |
|---------|------|
| `launch/orbbec_camera.launch.py` | 通用启动文件 |
| `launch/gemini_330_series.launch.py` | Gemini 330系列专用 |
| `launch/femto_bolt.launch.py` | Femto Bolt专用 |
| `launch/multi_camera.launch.py` | 多相机启动 |
| `launch/multi_camera_synced.launch.py` | 多相机同步启动 |

### 3.2 orbbec_camera_msgs（消息包）

#### 消息定义

| 文件 | 内容 | 用途 |
|------|------|------|
| `msg/RGBD.msg` | header + rgb + depth + camera_info | RGB-D组合消息 |
| `msg/DeviceInfo.msg` | 设备名称、序列号、固件版本等 | 设备信息查询 |
| `msg/DeviceStatus.msg` | 连接状态、温度等 | 设备状态监控 |
| `msg/Extrinsics.msg` | 旋转矩阵 + 平移向量 | 外参信息 |
| `msg/IMUInfo.msg` | IMU校准参数 | IMU标定信息 |
| `msg/Metadata.msg` | 帧时间戳、曝光、增益等 | 帧元数据 |

#### 服务定义

| 文件 | 请求/响应 | 用途 |
|------|----------|------|
| `srv/GetInt32.srv` | - / int32 data | 获取整型参数 |
| `srv/SetInt32.srv` | int32 data / bool success | 设置整型参数 |
| `srv/GetBool.srv` | - / bool data | 获取布尔参数 |
| `srv/GetString.srv` | - / string data | 获取字符串参数 |
| `srv/SetString.srv` | string data / bool success | 设置字符串参数 |
| `srv/SetFilter.srv` | 滤波器参数 / bool success | 配置滤波器 |
| `srv/GetDeviceInfo.srv` | - / DeviceInfo | 获取设备信息 |
| `srv/GetCameraInfo.srv` | - / CameraInfo | 获取相机内参 |
| `srv/SetArrays.srv` | int32[] data / bool success | 设置数组参数(ROI等) |

### 3.3 orbbec_description（描述包）

提供各型号相机的URDF模型，用于RViz可视化和TF树构建。

| 目录 | 内容 |
|------|------|
| `urdf/` | 各型号URDF/xacro文件 |
| `meshes/` | 3D模型STL文件 |
| `launch/` | 模型可视化启动文件 |

---

## 四、关键数据流

### 4.1 发布的Topics

```
/<camera_name>/
├── color/
│   ├── image_raw              # sensor_msgs/Image - RGB图像
│   ├── camera_info            # sensor_msgs/CameraInfo - 相机内参
│   └── metadata               # orbbec_camera_msgs/Metadata
├── depth/
│   ├── image_raw              # sensor_msgs/Image - 深度图(16UC1)
│   ├── camera_info            # sensor_msgs/CameraInfo
│   ├── points                 # sensor_msgs/PointCloud2 - 点云
│   └── metadata
├── ir/
│   ├── image_raw              # sensor_msgs/Image - IR图像
│   └── camera_info
├── left_ir/
│   ├── image_raw              # sensor_msgs/Image - 左IR图像
│   └── camera_info
├── right_ir/
│   ├── image_raw              # sensor_msgs/Image - 右IR图像
│   └── camera_info
├── gyro/
│   ├── sample                 # sensor_msgs/Imu - 陀螺仪数据
│   └── imu_info               # orbbec_camera_msgs/IMUInfo
├── accel/
│   ├── sample                 # sensor_msgs/Imu - 加速度计数据
│   └── imu_info               # orbbec_camera_msgs/IMUInfo
├── depth_registered/points    # sensor_msgs/PointCloud2 - 彩色点云
├── extrinsics/depth_to_color  # orbbec_camera_msgs/Extrinsics
└── device_status              # orbbec_camera_msgs/DeviceStatus
```

### 4.2 提供的Services

```
/<camera_name>/
├── 曝光控制
│   ├── get_color_exposure         # GetInt32
│   ├── set_color_exposure         # SetInt32
│   ├── get_depth_exposure         # GetInt32
│   ├── set_depth_exposure         # SetInt32
│   ├── get_ir_exposure            # GetInt32
│   └── set_ir_exposure            # SetInt32
├── 增益控制
│   ├── get_color_gain             # GetInt32
│   ├── set_color_gain             # SetInt32
│   ├── get_ir_gain                # GetInt32
│   └── set_ir_gain                # SetInt32
├── 自动曝光
│   ├── set_color_auto_exposure    # SetBool
│   ├── set_depth_auto_exposure    # SetBool
│   └── set_ir_auto_exposure       # SetBool
├── 图像变换
│   ├── set_color_mirror           # SetBool
│   ├── set_color_flip             # SetBool
│   ├── set_depth_mirror           # SetBool
│   └── set_depth_flip             # SetBool
├── 流控制
│   ├── toggle_color               # SetBool
│   ├── toggle_depth               # SetBool
│   ├── toggle_ir                  # SetBool
│   └── toggle_left_ir             # SetBool
├── 设备控制
│   ├── set_laser_enable           # SetBool
│   ├── set_ldp_enable             # SetBool
│   ├── set_fan_work_mode          # SetInt32
│   ├── reboot_device              # Empty
│   └── get_device_info            # GetDeviceInfo
├── 滤波器
│   └── set_filter                 # SetFilter
└── 其他
    ├── save_images                # Empty
    ├── save_point_cloud           # Empty
    └── get_sdk_version            # GetString
```

### 4.3 数据流向图

```
┌─────────────────────────────────────────────────────────────────┐
│                    Orbbec Camera Hardware                        │
└───────────────────────────┬─────────────────────────────────────┘
                            │ USB 3.0 / Network
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                      OrbbecSDK (ob::Pipeline)                    │
│  ob::Pipeline::start() → FrameSet Callback                       │
└───────────────────────────┬─────────────────────────────────────┘
                            │ std::shared_ptr<ob::FrameSet>
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│              OBCameraNode::onNewFrameSetCallback()               │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │ 1. 提取各类型帧: depth_frame, color_frame, ir_frame     │    │
│  │ 2. 应用滤波器: processDepthFrameFilter()                │    │
│  │ 3. 格式转换: MJPEG→RGB, Y16→depth                       │    │
│  │ 4. 构建ROS消息: sensor_msgs::Image                      │    │
│  │ 5. 发布到Topic: image_publishers_[stream]->publish()   │    │
│  └─────────────────────────────────────────────────────────┘    │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                      ROS2 Topic System                           │
│  /camera/color/image_raw  →  订阅者节点（如算法节点）            │
│  /camera/depth/image_raw  →  订阅者节点                          │
│  /camera/depth/points     →  订阅者节点                          │
└─────────────────────────────────────────────────────────────────┘
```

### 4.4 帧处理流程

```
ob::FrameSet (SDK回调)
       │
       ├──► getFrame(OB_FRAME_COLOR) ──► decodeColorFrame() ──► publishColorImage()
       │                                      │
       │                                      └──► MJPEG解码 (软件/硬件)
       │
       ├──► getFrame(OB_FRAME_DEPTH) ──► processDepthFrameFilter() ──► publishDepthImage()
       │                                      │
       │                                      ├──► DecimationFilter
       │                                      ├──► NoiseRemovalFilter
       │                                      ├──► SpatialFilter
       │                                      ├──► TemporalFilter
       │                                      └──► HoleFillingFilter
       │
       ├──► getFrame(OB_FRAME_IR_LEFT) ──► publishIRImage()
       │
       └──► getFrame(OB_FRAME_IR_RIGHT) ──► publishIRImage()
```

---

## 五、核心代码入口与流程

### 5.1 主节点入口

**文件位置：** `orbbec_camera/src/ob_camera_node_driver.cpp`

**节点注册（CMakeLists.txt 第183-185行）：**
```cmake
rclcpp_components_register_node(
    ${PROJECT_NAME}
    PLUGIN "orbbec_camera::OBCameraNodeDriver"
    EXECUTABLE orbbec_camera_node
)
```

### 5.2 初始化流程

```cpp
// 文件: ob_camera_node_driver.cpp:99-317

// 构造函数
OBCameraNodeDriver::OBCameraNodeDriver(const rclcpp::NodeOptions &node_options)
    : Node("orbbec_camera_node", "/", node_options) {
    init();
}

void OBCameraNodeDriver::init() {
    // 1. 信号处理注册
    signal(SIGSEGV, signalHandler);  // 段错误
    signal(SIGABRT, signalHandler);  // 异常终止
    signal(SIGINT, signalHandler);   // Ctrl+C
    signal(SIGTERM, signalHandler);  // 终止信号

    // 2. SDK初始化
    ob::Context::setExtensionsDirectory(extension_path_);
    ctx_ = std::make_unique<ob::Context>(config_path_);

    // 3. 参数声明与加载
    camera_name_ = declare_parameter<std::string>("camera_name", "camera");
    serial_number_ = declare_parameter<std::string>("serial_number", "");
    usb_port_ = declare_parameter<std::string>("usb_port", "");
    device_num_ = declare_parameter<int>("device_num", 1);
    // ... 更多参数

    // 4. 设备变化回调注册
    ctx_->registerDeviceChangedCallback([this](removed, added) {
        onDeviceDisconnected(removed);
        onDeviceConnected(added);
    });

    // 5. 定时器启动
    check_connect_timer_ = create_wall_timer(1s, checkConnectTimer);
    device_status_timer_ = create_wall_timer(100ms, deviceStatusTimer);

    // 6. 设备查询线程
    query_thread_ = std::make_shared<std::thread>(queryDevice);
    reset_device_thread_ = std::make_shared<std::thread>(resetDevice);
}
```

### 5.3 设备枚举与打开

```cpp
// 文件: ob_camera_node_driver.cpp:319-500

void OBCameraNodeDriver::onDeviceConnected(device_list) {
    if (!device_) {
        startDevice(device_list);
    }
}

void OBCameraNodeDriver::startDevice(device_list) {
    // 1. 选择设备（按序列号/USB端口/网络IP）
    device_ = selectDevice(device_list);

    // 2. 初始化设备
    initializeDevice(device_);

    // 3. 创建 OBCameraNode
    ob_camera_node_ = std::make_unique<OBCameraNode>(
        this, device_, parameters_, use_intra_process_
    );
}

std::shared_ptr<ob::Device> OBCameraNodeDriver::selectDevice(device_list) {
    // 优先级: serial_number > usb_port > net_device_ip > 第一个设备
    if (!serial_number_.empty()) {
        return selectDeviceBySerialNumber(device_list, serial_number_);
    }
    if (!usb_port_.empty()) {
        return selectDeviceByUSBPort(device_list, usb_port_);
    }
    if (!net_device_ip_.empty()) {
        return connectNetDevice(net_device_ip_, net_device_port_);
    }
    return device_list->getDevice(0);
}
```

### 5.4 流配置与发布循环

```cpp
// 文件: ob_camera_node.cpp:40-100

OBCameraNode::OBCameraNode(node, device, parameters, use_intra_process) {
    // 1. 初始化流名称映射
    stream_name_[COLOR] = "color";
    stream_name_[DEPTH] = "depth";
    stream_name_[INFRA1] = "left_ir";
    stream_name_[INFRA2] = "right_ir";

    // 2. 设置默认图像格式
    setupDefaultImageFormat();

    // 3. 配置Topics和Services
    setupTopics();  // 内部调用 setupPublishers() + setupCameraCtrlServices()
}

// 文件: ob_camera_node.cpp:1620-1640
void OBCameraNode::startStreams() {
    pipeline_ = std::make_unique<ob::Pipeline>(device_);

    // 配置流参数
    setupPipelineConfig();

    // 启动Pipeline，注册帧回调
    pipeline_->start(pipeline_config_, [this](frame_set) {
        onNewFrameSetCallback(frame_set);
    });
}
```

### 5.5 帧回调处理

```cpp
// 文件: ob_camera_node.cpp:3138-3320

void OBCameraNode::onNewFrameSetCallback(std::shared_ptr<ob::FrameSet> frame_set) {
    if (!is_running_.load() || !is_camera_node_initialized_.load()) {
        return;
    }

    try {
        // 1. 发布TF（首次）
        if (!tf_published_) {
            publishStaticTransforms();
            tf_published_ = true;
        }

        // 2. 提取各类型帧
        auto depth_frame = frame_set->getFrame(OB_FRAME_DEPTH);
        auto color_frame = frame_set->getFrame(OB_FRAME_COLOR);
        auto left_ir_frame = frame_set->getFrame(OB_FRAME_IR_LEFT);
        auto right_ir_frame = frame_set->getFrame(OB_FRAME_IR_RIGHT);

        // 3. 深度帧处理
        if (depth_frame) {
            depth_frame = processDepthFrameFilter(depth_frame);
            onNewFrameCallback(depth_frame, DEPTH);
            fps_counter_depth_->tick();
        }

        // 4. 彩色帧处理
        if (color_frame) {
            color_frame = processColorFrameFilter(color_frame);
            onNewColorFrameCallback(color_frame);
            fps_counter_color_->tick();
        }

        // 5. IR帧处理
        if (left_ir_frame) {
            onNewFrameCallback(left_ir_frame, INFRA1);
        }
        if (right_ir_frame) {
            onNewFrameCallback(right_ir_frame, INFRA2);
        }

        // 6. 点云发布
        if (enable_point_cloud_) {
            publishDepthPointCloud(frame_set);
        }
        if (enable_colored_point_cloud_) {
            publishColoredPointCloud(frame_set);
        }

    } catch (const ob::Error &e) {
        RCLCPP_ERROR_STREAM(logger_, "onNewFrameSetCallback error: " << e.getMessage());
    } catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(logger_, "onNewFrameSetCallback error: " << e.what());
    }
}
```

### 5.6 异常处理机制

```cpp
// 文件: ob_camera_node_driver.cpp:38-95

void signalHandler(int sig) {
    static std::atomic<bool> in_signal_handler{false};
    if (in_signal_handler.exchange(true)) {
        _exit(sig);  // 防止递归
    }

    if (sig == SIGINT || sig == SIGTERM) {
        // 正常退出信号
        rclcpp::shutdown();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    } else {
        // 崩溃信号：记录堆栈到日志
        std::string log_file_path = "Log/" + g_camera_name + "_crash_stack_trace.log";
        std::ofstream log_file(log_file_path, std::ios::app);

        backward::StackTrace st;
        st.load_here(32);
        backward::Printer p;
        p.print(st, log_file);

        _exit(sig);
    }
}
```

---

## 六、配置与参数

### 6.1 参数定义位置

| 位置 | 说明 | 优先级 |
|------|------|--------|
| 命令行参数 | `ros2 launch ... param:=value` | 最高 |
| Launch文件参数 | `launch/*.launch.py` | 高 |
| YAML配置文件 | `config/*.yaml` | 中 |
| 代码默认值 | `constants.h` / `declare_parameter()` | 最低 |

### 6.2 配置文件说明

| 文件 | 用途 |
|------|------|
| `config/common.yaml` | 通用默认参数（所有相机共用） |
| `config/gemini330_series.yaml` | Gemini 330系列专用参数 |
| `config/femto_bolt.yaml` | Femto Bolt专用参数 |
| `config/camera_params.yaml` | 简化参数模板 |
| `config/OrbbecSDKConfig_v2.0.xml` | SDK底层配置 |

### 6.3 核心参数列表

#### 设备参数

```yaml
# 设备标识
camera_name: "camera"           # 相机命名空间
serial_number: ""               # 序列号（多相机时指定）
usb_port: ""                    # USB端口（多相机时指定）
device_num: 1                   # 设备数量

# 连接设置
connection_delay: 10            # 重连延迟(ms)
uvc_backend: "libuvc"           # UVC后端: libuvc/v4l2
retry_on_usb3_detection_failure: false  # USB3检测失败重试
```

#### 图像流参数

```yaml
# 彩色流
enable_color: true
color_width: 0                  # 0=自动选择
color_height: 0
color_fps: 0
color_format: "ANY"             # MJPG/RGB/YUYV/ANY
color_qos: "default"
color_flip: false
color_mirror: false

# 深度流
enable_depth: true
depth_width: 0
depth_height: 0
depth_fps: 0
depth_format: "ANY"             # Y16/Y8/ANY
depth_qos: "default"
enable_depth_scale: true
depth_precision: ""             # 深度精度: 1mm/0.1mm

# 红外流
enable_left_ir: false
enable_right_ir: false
left_ir_width: 0
left_ir_height: 0
left_ir_fps: 0
left_ir_format: "ANY"
```

#### 曝光/增益参数

```yaml
# 彩色相机
enable_color_auto_exposure: true
color_exposure: -1              # -1=自动
color_gain: -1
color_ae_max_exposure: -1
color_ae_roi_left: -1           # ROI区域
color_ae_roi_top: -1
color_ae_roi_right: -1
color_ae_roi_bottom: -1
enable_color_auto_white_balance: true
color_white_balance: -1

# 红外相机
enable_ir_auto_exposure: true
ir_exposure: -1
ir_gain: -1
ir_ae_max_exposure: -1
ir_brightness: -1
```

#### 深度处理参数

```yaml
# D2C对齐
depth_registration: true        # 开启深度对齐
align_mode: "SW"                # SW软件/HW硬件

# 深度工作模式
depth_work_mode: ""             # Unbinned Dense Default等
device_preset: "Default"        # Default/Hand/High Accuracy等

# 激光控制
enable_laser: true
laser_energy_level: -1
enable_ldp: true
```

#### 滤波器参数

```yaml
# 抽取滤波器
enable_decimation_filter: false
decimation_filter_scale: -1

# 噪声去除滤波器
enable_noise_removal_filter: true
noise_removal_filter_min_diff: 256
noise_removal_filter_max_size: 80

# 阈值滤波器
enable_threshold_filter: false
threshold_filter_max: -1
threshold_filter_min: -1

# 空间滤波器
enable_spatial_filter: false
spatial_filter_alpha: -1.0
spatial_filter_diff_threshold: -1
spatial_filter_magnitude: -1
spatial_filter_radius: -1

# 时间滤波器
enable_temporal_filter: false
temporal_filter_diff_threshold: -1.0
temporal_filter_weight: -1.0

# 空洞填充滤波器
enable_hole_filling_filter: false
hole_filling_filter_mode: ""

# HDR合并
enable_hdr_merge: false
hdr_merge_exposure_1: -1
hdr_merge_gain_1: -1
hdr_merge_exposure_2: -1
hdr_merge_gain_2: -1
```

#### 点云参数

```yaml
enable_point_cloud: true
enable_colored_point_cloud: false
point_cloud_qos: "default"
cloud_frame_id: ""              # 自定义frame_id
ordered_pc: false               # 有序点云
```

#### IMU参数

```yaml
enable_accel: false
enable_gyro: false
accel_rate: "200hz"
accel_range: "4g"
gyro_rate: "200hz"
gyro_range: "1000dps"
enable_sync_output_accel_gyro: false
linear_accel_cov: 0.01
angular_vel_cov: 0.01
```

#### 多相机同步参数

```yaml
sync_mode: "standalone"         # free_run/standalone/primary/secondary
depth_delay_us: 0
color_delay_us: 0
trigger2image_delay_us: 0
trigger_out_enabled: true
trigger_out_delay_us: 0
frames_per_trigger: 2
software_trigger_period: 33
```

#### TF与高级参数

```yaml
# TF发布
publish_tf: true
tf_publish_rate: 0.0            # 0=静态TF

# 时间同步
time_domain: "device"           # device/global/system
enable_sync_host_time: true

# 帧同步
enable_frame_sync: true

# 诊断
diagnostic_period: 1.0
enable_heartbeat: false
```

### 6.4 参数生效流程

```
┌─────────────────┐
│ 命令行参数       │ ros2 launch ... param:=value
└────────┬────────┘
         │ 最高优先级
         ▼
┌─────────────────┐
│ Launch参数      │ DeclareLaunchArgument()
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ YAML配置文件    │ config/common.yaml + config/<model>.yaml
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ 代码默认值      │ constants.h / declare_parameter()
└────────┬────────┘
         │
         ▼
┌─────────────────────────────────────────────┐
│ orbbec_camera.launch.py::load_parameters() │
│   1. 加载 common.yaml                       │
│   2. 加载 <camera_model>.yaml 覆盖          │
│   3. 合并 launch 参数                       │
│   4. 合并命令行参数                         │
└────────┬────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────┐
│ OBCameraNodeDriver::init()                  │
│   declare_parameter() 声明并获取参数        │
└────────┬────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────┐
│ OBCameraNode::getParameters()               │
│   读取参数并应用到相机配置                  │
└─────────────────────────────────────────────┘
```

---

## 七、编译与运行

### 7.1 系统要求

- **操作系统：** Ubuntu 20.04/22.04 (推荐)
- **ROS2版本：** Foxy/Humble/Iron
- **编译器：** GCC 9+ / Clang 10+
- **CMake：** 3.8+

### 7.2 依赖安装

```bash
# ROS2核心依赖
sudo apt install ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-diagnostic-updater \
    ros-humble-tf2-ros \
    ros-humble-tf2-sensor-msgs \
    ros-humble-backward-ros \
    ros-humble-image-publisher

# 系统依赖
sudo apt install libeigen3-dev \
    libopencv-dev \
    libyaml-cpp-dev \
    libssl-dev

# USB权限配置
cd OrbbecSDK_ROS2/orbbec_camera/scripts
sudo cp 99-obsensor-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 7.3 编译命令

```bash
# 进入工作空间
cd ~/ros2_ws

# 编译所有包
colcon build --packages-select orbbec_camera_msgs orbbec_camera orbbec_description

# 仅编译相机包（Release模式）
colcon build --packages-select orbbec_camera \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# 编译选项
colcon build --packages-select orbbec_camera \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DUSE_RK_HW_DECODER=OFF \
    -DUSE_NV_HW_DECODER=OFF

# 清理后重新编译
rm -rf build/ install/ log/
colcon build
```

### 7.4 运行命令

```bash
# 加载环境
source ~/ros2_ws/install/setup.bash

# 基础启动（自动检测相机型号）
ros2 launch orbbec_camera orbbec_camera.launch.py

# 指定相机型号
ros2 launch orbbec_camera gemini_330_series.launch.py
ros2 launch orbbec_camera femto_bolt.launch.py
ros2 launch orbbec_camera astra2.launch.py

# 自定义参数启动
ros2 launch orbbec_camera orbbec_camera.launch.py \
    camera_name:=my_camera \
    enable_color:=true \
    enable_depth:=true \
    color_width:=1280 \
    color_height:=720 \
    color_fps:=30

# 多相机启动
ros2 launch orbbec_camera multi_camera.launch.py

# 多相机同步启动
ros2 launch orbbec_camera multi_camera_synced.launch.py
```

### 7.5 验证与调试

```bash
# 查看Topic列表
ros2 topic list | grep camera

# 查看图像数据
ros2 topic echo /camera/color/image_raw --no-arr

# 查看帧率
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw

# 查看相机信息
ros2 topic echo /camera/color/camera_info

# 调用服务
ros2 service call /camera/get_device_info orbbec_camera_msgs/srv/GetDeviceInfo
ros2 service call /camera/set_color_exposure orbbec_camera_msgs/srv/SetInt32 "{data: 100}"

# RViz可视化
rviz2 -d orbbec_camera/rviz/pointcloud.rviz
```

### 7.6 常见报错排查

| 错误信息 | 可能原因 | 解决方案 |
|---------|---------|----------|
| `No device found` | USB权限不足 | 安装udev规则并重启 |
| `Failed to open device` | 设备被占用 | `lsof /dev/video*` 检查占用进程 |
| `USB 2.0 detected` | USB线/端口问题 | 更换USB3.0线或端口 |
| `MJPEG decode failed` | 解码器问题 | 检查OpenCV安装 |
| `TF tree broken` | frame_id配置错误 | 检查camera_name参数 |
| `Pipeline start failed` | 流配置不支持 | 检查分辨率/帧率组合 |
| `Timeout waiting for frame` | 相机无响应 | 重插USB或重启相机 |
| `Permission denied` | 权限不足 | `sudo chmod 666 /dev/video*` |

### 7.7 性能优化建议

```bash
# 1. 降低CPU占用
ros2 launch orbbec_camera gemini_330_series_low_cpu.launch.py

# 2. 使用硬件解码（Jetson平台）
colcon build --cmake-args -DUSE_NV_HW_DECODER=ON

# 3. 使用硬件解码（RK3588平台）
colcon build --cmake-args -DUSE_RK_HW_DECODER=ON

# 4. 调整QoS策略
# 在launch参数中设置
color_qos:=SENSOR_DATA
depth_qos:=SENSOR_DATA

# 5. 禁用不需要的流
enable_left_ir:=false
enable_right_ir:=false
enable_point_cloud:=false
```

---

## 八、二次开发指南 - TCL人眼定位算法定制

### 8.1 开发方案选择

#### 方案A：独立订阅者节点（推荐）

```
┌─────────────────┐     ┌─────────────────────────────────┐
│ orbbec_camera   │────▶│  eye_localization_node          │
│ (原始驱动)      │     │  - 订阅 /camera/color/image_raw │
│                 │     │  - 订阅 /camera/depth/image_raw │
│                 │     │  - 人眼检测算法                 │
│                 │     │  - 发布检测结果                 │
└─────────────────┘     └─────────────────────────────────┘
```

**优点：**
- 解耦设计，不影响原驱动稳定性
- 易于独立开发、测试、部署
- 可热插拔，便于算法迭代
- 支持多算法并行运行

**缺点：**
- 额外的数据拷贝开销
- 略高的延迟（通常<5ms）

#### 方案B：深度集成到帧回调

```cpp
// 在 onNewFrameSetCallback() 中直接调用算法
void OBCameraNode::onNewFrameSetCallback(frame_set) {
    // ... 原有处理 ...

    // 插入人眼定位算法
    if (enable_eye_localization_) {
        auto result = eye_localizer_->detect(color_mat, depth_mat);
        publishEyeResult(result);
    }
}
```

**优点：**
- 最低延迟，直接访问原始帧
- 无额外数据拷贝
- 可与滤波器协同工作

**缺点：**
- 需要修改原驱动代码
- 耦合度高，升级困难
- 算法崩溃会影响整个驱动

### 8.2 方案A实现指南（推荐）

#### 8.2.1 创建算法包

```bash
cd ~/ros2_ws/src
ros2 pkg create eye_localization \
    --build-type ament_cmake \
    --dependencies rclcpp sensor_msgs cv_bridge image_transport
```

#### 8.2.2 节点代码框架

```cpp
// eye_localization/src/eye_localization_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

class EyeLocalizationNode : public rclcpp::Node {
public:
    EyeLocalizationNode() : Node("eye_localization_node") {
        // 声明参数
        camera_name_ = declare_parameter("camera_name", "camera");

        // 订阅图像
        color_sub_.subscribe(this, "/" + camera_name_ + "/color/image_raw");
        depth_sub_.subscribe(this, "/" + camera_name_ + "/depth/image_raw");

        // 同步订阅
        sync_ = std::make_shared<Sync>(SyncPolicy(10), color_sub_, depth_sub_);
        sync_->registerCallback(&EyeLocalizationNode::imageCallback, this);

        // 订阅相机内参
        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/" + camera_name_ + "/color/camera_info", 10,
            [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                camera_info_ = *msg;
            });

        // 发布检测结果
        result_pub_ = create_publisher<YourResultMsg>("eye_position", 10);

        // 初始化检测器
        initDetector();
    }

private:
    void imageCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr& color_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) {

        // 转换为OpenCV格式
        cv::Mat color = cv_bridge::toCvShare(color_msg, "bgr8")->image;
        cv::Mat depth = cv_bridge::toCvShare(depth_msg, "16UC1")->image;

        // 人眼检测
        auto result = detectEyes(color, depth);

        // 发布结果
        publishResult(result);
    }

    EyeResult detectEyes(const cv::Mat& color, const cv::Mat& depth) {
        EyeResult result;

        // 1. 人脸检测
        std::vector<cv::Rect> faces = face_detector_.detect(color);

        // 2. 人眼关键点检测
        for (const auto& face : faces) {
            auto landmarks = landmark_detector_.detect(color, face);

            // 3. 提取眼睛位置 (2D)
            cv::Point2f left_eye_2d = landmarks[36];  // 左眼
            cv::Point2f right_eye_2d = landmarks[45]; // 右眼

            // 4. 获取深度值
            float left_z = getDepth(depth, left_eye_2d);
            float right_z = getDepth(depth, right_eye_2d);

            // 5. 2D→3D转换
            result.left_eye_3d = pixel2Point(left_eye_2d, left_z);
            result.right_eye_3d = pixel2Point(right_eye_2d, right_z);
            result.detected = true;
        }

        return result;
    }

    cv::Point3f pixel2Point(cv::Point2f pixel, float depth_mm) {
        float z = depth_mm / 1000.0f;  // mm → m
        float x = (pixel.x - camera_info_.k[2]) * z / camera_info_.k[0];
        float y = (pixel.y - camera_info_.k[5]) * z / camera_info_.k[4];
        return cv::Point3f(x, y, z);
    }

    float getDepth(const cv::Mat& depth, cv::Point2f pt) {
        int x = static_cast<int>(pt.x);
        int y = static_cast<int>(pt.y);
        if (x >= 0 && x < depth.cols && y >= 0 && y < depth.rows) {
            return depth.at<uint16_t>(y, x);
        }
        return 0;
    }
};
```

#### 8.2.3 Launch文件

```python
# eye_localization/launch/eye_localization.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='eye_localization',
            executable='eye_localization_node',
            name='eye_localization',
            parameters=[{
                'camera_name': 'camera',
            }],
            output='screen'
        )
    ])
```

### 8.3 方案B实现指南（深度集成）

#### 8.3.1 需要修改的文件

| 文件 | 修改内容 |
|------|----------|
| `include/orbbec_camera/ob_camera_node.h` | 添加算法成员变量和方法 |
| `src/ob_camera_node.cpp` | 在回调中调用算法 |
| `config/common.yaml` | 添加算法参数 |
| `CMakeLists.txt` | 添加算法依赖 |

#### 8.3.2 头文件修改

```cpp
// include/orbbec_camera/ob_camera_node.h
// 在类声明中添加：

class OBCameraNode {
    // ... 原有成员 ...

    // ===== 人眼定位相关 =====
    bool enable_eye_localization_;
    std::unique_ptr<EyeLocalizer> eye_localizer_;
    rclcpp::Publisher<YourResultMsg>::SharedPtr eye_result_pub_;

    void initEyeLocalizer();
    void processEyeLocalization(const cv::Mat& color, const cv::Mat& depth);
};
```

#### 8.3.3 源文件修改

```cpp
// src/ob_camera_node.cpp

// 在构造函数中添加初始化
OBCameraNode::OBCameraNode(...) {
    // ... 原有初始化 ...

    // 人眼定位初始化
    enable_eye_localization_ = parameters_->get<bool>("enable_eye_localization", false);
    if (enable_eye_localization_) {
        initEyeLocalizer();
    }
}

// 在 onNewFrameSetCallback 中添加调用
void OBCameraNode::onNewFrameSetCallback(frame_set) {
    // ... 原有处理 ...

    // 人眼定位处理
    if (enable_eye_localization_ && color_frame && depth_frame) {
        cv::Mat color_mat = frameToMat(color_frame);
        cv::Mat depth_mat = frameToMat(depth_frame);
        processEyeLocalization(color_mat, depth_mat);
    }
}
```

### 8.4 关键数据获取方法

#### 8.4.1 获取图像数据

```cpp
// 从 ob::Frame 获取 cv::Mat
cv::Mat frameToMat(std::shared_ptr<ob::Frame> frame) {
    auto video_frame = frame->as<ob::VideoFrame>();
    int width = video_frame->getWidth();
    int height = video_frame->getHeight();
    auto format = video_frame->getFormat();

    if (format == OB_FORMAT_RGB) {
        return cv::Mat(height, width, CV_8UC3, video_frame->getData());
    } else if (format == OB_FORMAT_Y16) {
        return cv::Mat(height, width, CV_16UC1, video_frame->getData());
    } else if (format == OB_FORMAT_Y8) {
        return cv::Mat(height, width, CV_8UC1, video_frame->getData());
    }
    return cv::Mat();
}
```

#### 8.4.2 获取相机内参

```cpp
// 从 stream_intrinsics_ 获取内参
auto intrinsics = stream_intrinsics_[COLOR];
float fx = intrinsics.fx;  // 焦距x
float fy = intrinsics.fy;  // 焦距y
float cx = intrinsics.cx;  // 主点x
float cy = intrinsics.cy;  // 主点y

// 畸变系数
float k1 = intrinsics.k1;
float k2 = intrinsics.k2;
float k3 = intrinsics.k3;
float p1 = intrinsics.p1;
float p2 = intrinsics.p2;
```

#### 8.4.3 2D像素到3D点转换

```cpp
cv::Point3f pixel2Point(int u, int v, float depth_mm,
                        const OBCameraIntrinsic& intrinsics) {
    float z = depth_mm / 1000.0f;  // mm → m
    float x = (u - intrinsics.cx) * z / intrinsics.fx;
    float y = (v - intrinsics.cy) * z / intrinsics.fy;
    return cv::Point3f(x, y, z);
}
```

### 8.5 推荐的开发步骤

```
1. 环境搭建
   └── 编译运行原始驱动，验证相机正常工作

2. 算法原型验证（方案A）
   ├── 创建独立节点订阅图像
   ├── 集成人脸/人眼检测算法
   ├── 验证检测精度
   └── 测试性能（帧率、延迟）

3. 深度信息融合
   ├── 同步RGB和Depth数据
   ├── 实现2D→3D坐标转换
   └── 验证3D定位精度

4. 性能优化
   ├── 算法加速（GPU/NPU）
   ├── 降低延迟
   └── 稳定性测试

5. 生产部署（可选方案B）
   ├── 如需更低延迟，考虑深度集成
   └── 充分测试后合并到驱动
```

### 8.6 关键参数调优建议

```yaml
# 针对人眼定位场景的推荐配置

# 分辨率：根据检测距离选择
# 近距离(<1m): 640x480 足够
# 中距离(1-3m): 1280x720 推荐
# 远距离(>3m): 1920x1080
color_width: 1280
color_height: 720
color_fps: 30

# 深度配置
depth_width: 640
depth_height: 480
depth_registration: true    # 必须开启D2C对齐
align_mode: "SW"            # 软件对齐更精确

# 帧同步
enable_frame_sync: true     # 确保RGB-D同步

# 滤波器（根据场景调整）
enable_noise_removal_filter: true
enable_temporal_filter: true  # 减少深度抖动
```

---

## 九、附录

### 9.1 关键文件路径速查

| 类别 | 文件路径 |
|------|----------|
| 主入口 | `orbbec_camera/src/ob_camera_node_driver.cpp` |
| 核心逻辑 | `orbbec_camera/src/ob_camera_node.cpp` |
| 服务实现 | `orbbec_camera/src/ros_service.cpp` |
| 头文件 | `orbbec_camera/include/orbbec_camera/ob_camera_node.h` |
| 常量定义 | `orbbec_camera/include/orbbec_camera/constants.h` |
| 通用配置 | `orbbec_camera/config/common.yaml` |
| 启动文件 | `orbbec_camera/launch/orbbec_camera.launch.py` |
| 消息定义 | `orbbec_camera_msgs/msg/*.msg` |
| 服务定义 | `orbbec_camera_msgs/srv/*.srv` |
| CMake | `orbbec_camera/CMakeLists.txt` |

### 9.2 常用命令速查

```bash
# 编译
colcon build --packages-select orbbec_camera

# 启动
ros2 launch orbbec_camera orbbec_camera.launch.py

# 查看Topic
ros2 topic list | grep camera
ros2 topic hz /camera/color/image_raw

# 查看服务
ros2 service list | grep camera

# 调用服务
ros2 service call /camera/get_device_info orbbec_camera_msgs/srv/GetDeviceInfo

# 参数查看
ros2 param list /camera/camera
ros2 param get /camera/camera color_width
```

### 9.3 参考资源

- [Orbbec SDK 官方文档](https://developer.orbbec.com.cn/)
- [OrbbecSDK_ROS2 GitHub](https://github.com/orbbec/OrbbecSDK_ROS2)
- [ROS2 官方文档](https://docs.ros.org/)

---

**文档结束**

> 如有问题，请联系项目负责人。


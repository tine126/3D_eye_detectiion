# TCL 眼动追踪系统 - 详细技术报告

> 生成日期: 2026-01-30
> 平台: Horizon Robotics X5 (RDK X5)
> ROS版本: ROS2 Humble

---

## 目录

1. [系统概述](#一系统概述)
2. [系统架构](#二系统架构)
3. [节点详细说明](#三节点详细说明)
4. [消息定义](#四消息定义)
5. [启动配置](#五启动配置)
6. [数据流与性能](#六数据流与性能)
7. [关键技术点](#七关键技术点)

---

## 一、系统概述

TCL眼动追踪系统是一个基于ROS2的双目视觉眼动追踪解决方案，运行在Horizon Robotics X5平台上，利用BPU (Brain Processing Unit) 进行AI推理加速。

### 1.1 功能特性

- **双目立体视觉**: 使用Orbbec Gemini 335L的左右IR相机
- **实时人脸检测**: 基于FasterRCNN多任务模型
- **106点人脸关键点**: 精确定位眼睛区域
- **3D眼睛坐标计算**: 双目三角测量获取空间坐标
- **BPU硬件加速**: 利用X5平台的NPU进行推理

### 1.2 硬件要求

| 组件 | 规格 |
|------|------|
| 计算平台 | Horizon Robotics RDK X5 |
| 相机 | Orbbec Gemini 335L |
| IR分辨率 | 1280 x 800 @ 30fps |
| 基线距离 | 95mm |

### 1.3 软件依赖

- ROS2 Humble (TogetherROS)
- Horizon DNN Node
- Orbbec SDK ROS2
- FastDDS

---

## 二、系统架构

### 2.1 整体架构图

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        TCL Eye Tracking Pipeline                            │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌──────────────┐    ┌──────────────┐                                       │
│  │   Orbbec     │    │   Orbbec     │       ┌──────────────┐                │
│  │   Left IR    │    │   Right IR   │       │   Gemini     │                │
│  │   Camera     │    │   Camera     │       │   335L       │                │
│  └──────┬───────┘    └──────┬───────┘       └──────────────┘                │
│         │                   │                                               │
│         ▼                   ▼                                               │
│  /camera/left_ir/     /camera/right_ir/                                     │
│  image_raw (mono8)    image_raw (mono8)                                     │
│         │                   │                                               │
│         └─────────┬─────────┘                                               │
│                   ▼                                                         │
│         ┌─────────────────────┐                                             │
│         │ img_format_converter│  mono8 → NV12                               │
│         │     (Node 1)        │                                             │
│         └─────────┬───────────┘                                             │
│                   │                                                         │
│         ┌─────────┴─────────┐                                               │
│         ▼                   ▼                                               │
│  /hbmem_img_left      /hbmem_img_right                                      │
│  (HbmMsg1080P)        (HbmMsg1080P)                                         │
│         │                   │                                               │
│         └─────────┬─────────┘                                               │
│                   ▼                                                         │
│         ┌─────────────────────┐                                             │
│         │mono2d_body_detection│  FasterRCNN (BPU)                           │
│         │     (Node 2)        │  人脸检测                                   │
│         └─────────┬───────────┘                                             │
│                   │                                                         │
│         ┌─────────┴─────────┐                                               │
│         ▼                   ▼                                               │
│  /hobot_mono2d_body_  /hobot_mono2d_body_                                   │
│  detection_left       detection_right                                       │
│  (PerceptionTargets)  (PerceptionTargets)                                   │
│         │                   │                                               │
│         └─────────┬─────────┘                                               │
│                   ▼                                                         │
│         ┌─────────────────────┐                                             │
│         │face_landmarks_det   │  106点关键点 (BPU)                          │
│         │     (Node 3)        │                                             │
│         └─────────┬───────────┘                                             │
│                   │                                                         │
│         ┌─────────┴─────────┐                                               │
│         ▼                   ▼                                               │
│  /face_landmarks_     /face_landmarks_                                      │
│  detection_left       detection_right                                       │
│         │                   │                                               │
│         └─────────┬─────────┘                                               │
│                   ▼                                                         │
│         ┌─────────────────────┐                                             │
│         │eye_position_publisher│  提取眼睛中心                              │
│         │     (Node 4)        │                                             │
│         └─────────┬───────────┘                                             │
│                   │                                                         │
│         ┌─────────┴─────────┐                                               │
│         ▼                   ▼                                               │
│  /eye_positions_left  /eye_positions_right                                  │
│  (EyePositions)       (EyePositions)                                        │
│         │                   │                                               │
│         └─────────┬─────────┘                                               │
│                   ▼                                                         │
│         ┌─────────────────────┐                                             │
│         │   eye_position_3d   │  双目三角测量                               │
│         │     (Node 5)        │                                             │
│         └─────────┬───────────┘                                             │
│                   ▼                                                         │
│         /eye_positions_3d                                                   │
│         (EyePositions3D)                                                    │
│         X, Y, Z 坐标 (mm)                                                   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2.2 话题列表

| 话题名称 | 消息类型 | 发布者 | 订阅者 |
|----------|----------|--------|--------|
| /camera/left_ir/image_raw | sensor_msgs/Image | orbbec_camera | img_format_converter |
| /camera/right_ir/image_raw | sensor_msgs/Image | orbbec_camera | img_format_converter |
| /hbmem_img_left | hbm_img_msgs/HbmMsg1080P | img_format_converter | mono2d_body_detection, face_landmarks_detection |
| /hbmem_img_right | hbm_img_msgs/HbmMsg1080P | img_format_converter | mono2d_body_detection, face_landmarks_detection |
| /hobot_mono2d_body_detection_left | ai_msgs/PerceptionTargets | mono2d_body_detection | face_landmarks_detection |
| /hobot_mono2d_body_detection_right | ai_msgs/PerceptionTargets | mono2d_body_detection | face_landmarks_detection |
| /face_landmarks_detection_left | ai_msgs/PerceptionTargets | face_landmarks_detection | eye_position_publisher |
| /face_landmarks_detection_right | ai_msgs/PerceptionTargets | face_landmarks_detection | eye_position_publisher |
| /eye_positions_left | EyePositions | eye_position_publisher | eye_position_3d |
| /eye_positions_right | EyePositions | eye_position_publisher | eye_position_3d |
| /eye_positions_3d | EyePositions3D | eye_position_3d | (用户应用) |

---

## 三、节点详细说明

### 3.1 节点1: img_format_converter

**包名**: `img_format_converter`

**源文件**:
- `include/img_format_converter_node.h`
- `src/img_format_converter_node.cpp`

**功能描述**:
将Orbbec相机输出的mono8灰度图像转换为NV12格式，供Horizon BPU推理使用。

**工作流程**:
```
1. 订阅左/右IR相机的 sensor_msgs/Image (mono8格式)
2. 创建 HbmMsg1080P 消息 (Horizon共享内存消息)
3. 格式转换:
   - Y平面: 直接复制mono8数据 (灰度值 = 亮度值)
   - UV平面: 填充128 (中性灰，无色度)
4. 发布到 /hbmem_img_left 和 /hbmem_img_right
```

**核心代码逻辑**:
```cpp
void ImgFormatConverterNode::ConvertMono8ToNV12(
    const uint8_t* mono8_data, uint8_t* nv12_data,
    int width, int height)
{
    // Y平面：直接复制mono8数据（灰度值=亮度值）
    int y_size = width * height;
    std::memcpy(nv12_data, mono8_data, y_size);

    // UV平面：填充128（中性灰，无色度）
    int uv_size = y_size / 2;
    std::memset(nv12_data + y_size, 128, uv_size);
}
```

**参数配置**:

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| left_sub_topic | string | /camera/left_ir/image_raw | 左IR输入话题 |
| left_pub_topic | string | /hbmem_img_left | 左IR输出话题 |
| right_sub_topic | string | /camera/right_ir/image_raw | 右IR输入话题 |
| right_pub_topic | string | /hbmem_img_right | 右IR输出话题 |
| image_width | int | 1280 | 图像宽度 |
| image_height | int | 800 | 图像高度 |

**性能指标**:
- 转换耗时: 平均 3-4ms
- 端到端延迟: 平均 30ms
- 统计周期: 每5秒输出一次

---

### 3.2 节点2: mono2d_body_detection

**包名**: `mono2d_body_detection`

**源文件**:
- `include/mono2d_body_det_node.h`
- `src/mono2d_body_det_node.cpp`

**功能描述**:
使用FasterRCNN多任务模型在BPU上进行人脸检测，输出人脸边界框。

**工作流程**:
```
1. 订阅 /hbmem_img_left 和 /hbmem_img_right (NV12格式)
2. 图像预处理:
   - 使用 hobotcv_resize 缩放到模型输入尺寸 (960x544)
   - 构建 NV12 金字塔输入
3. BPU推理:
   - 模型: multitask_body_head_face_hand_kps_960x544.hbm
   - 支持异步/同步模式
4. 后处理:
   - 解析 face 检测结果 (输出索引5)
   - 置信度过滤 (默认阈值0.5)
   - 坐标缩放回原图尺寸
5. 发布 ai_msgs/PerceptionTargets 到对应通道
```

**参数配置**:

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| model_file_name | string | config/multitask...hbm | 模型文件路径 |
| score_threshold | float | 0.5 | 人脸检测置信度阈值 |
| is_sync_mode | int | 0 | 0=异步, 1=同步 |
| left_img_topic | string | /hbmem_img_left | 左IR输入话题 |
| mono2d_left_pub_topic | string | /hobot_mono2d_body_detection_left | 左IR输出话题 |

**模型信息**:
```
模型名称: multitask_body_head_face_hand_kps_960x544
输入: [1, 3, 544, 960], NV12_SEPARATE
输出:
  - (0-7) 检测框输出 (body, head, face, hand)
  - (8) 关键点输出
Face输出索引: 5
```

**性能指标**:
- 输入帧率: ~25 fps
- 推理耗时: ~38ms
- Pipeline延迟: ~68ms

---

### 3.3 节点3: face_landmarks_detection

**包名**: `face_landmarks_detection`

**源文件**:
- `include/face_landmarks_det_node.h`
- `src/face_landmarks_det_node.cpp`

**功能描述**:
对检测到的人脸进行106点关键点检测，精确定位面部特征。

**工作流程**:
```
1. 同时订阅:
   - 图像话题: /hbmem_img_left, /hbmem_img_right
   - AI消息话题: /hobot_mono2d_body_detection_left/right
2. 消息匹配:
   - 使用 AiMsgManage 进行时间戳匹配
   - 超时时间: 200ms
3. ROI处理:
   - 从 PerceptionTargets 提取人脸ROI
   - 扩展ROI (expand_scale=1.25)
   - 尺寸过滤 (16-255像素)
4. BPU推理:
   - 模型: faceLandmark106pts.hbm
   - 输入: 128x128 ROI区域
   - 输出: 106个关键点坐标
5. 发布带关键点的 PerceptionTargets
```

**参数配置**:

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| landmarks_model_file_name | string | config/faceLandmark106pts.hbm | 模型文件 |
| expand_scale | float | 1.25 | ROI扩展比例 |
| roi_size_min | int | 16 | ROI最小尺寸 |
| roi_size_max | int | 255 | ROI最大尺寸 |
| cache_len_limit | int | 8 | 图像缓存上限 |

**106点关键点分布**:
```
索引范围    |  区域
-----------|--------
0-32       |  脸部轮廓
33-41      |  左眼 (9个点)
42-50      |  右眼 (9个点)
51-71      |  鼻子
72-105     |  嘴巴
```

**性能指标**:
- 推理耗时: ~50-100ms
- 匹配率: ~80%

---

### 3.4 节点4: eye_position_publisher

**包名**: `eye_position_publisher`

**源文件**:
- `include/eye_position_publisher_node.h`
- `src/eye_position_publisher_node.cpp`

**功能描述**:
从106点关键点中提取左右眼中心坐标。

**工作流程**:
```
1. 订阅 /face_landmarks_detection_left 和 right
2. 遍历每个检测到的人脸:
   - 提取 landmarks 点集
   - 计算左眼中心: 平均(点33-41)
   - 计算右眼中心: 平均(点42-50)
3. 构建 EyePositions 消息:
   - track_id: 人脸追踪ID
   - left_eye: 左眼2D坐标
   - right_eye: 右眼2D坐标
   - valid_flag: 有效性标志
4. 发布到 /eye_positions_left 和 right
```

**眼睛中心计算算法**:
```cpp
Point32 CalculateEyeCenter(points, start_idx, end_idx) {
    float sum_x = 0, sum_y = 0;
    for (int i = start_idx; i <= end_idx; i++) {
        sum_x += points[i].x;
        sum_y += points[i].y;
    }
    int count = end_idx - start_idx + 1;
    return Point32(sum_x/count, sum_y/count);
}

// 左眼: CalculateEyeCenter(points, 33, 41)
// 右眼: CalculateEyeCenter(points, 42, 50)
```

**参数配置**:

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| eye_left_sub_topic | string | /face_landmarks_detection_left | 左IR输入 |
| eye_left_pub_topic | string | /eye_positions_left | 左IR输出 |
| eye_right_sub_topic | string | /face_landmarks_detection_right | 右IR输入 |
| eye_right_pub_topic | string | /eye_positions_right | 右IR输出 |

---

### 3.5 节点5: eye_position_3d

**包名**: `eye_position_3d`

**源文件**:
- `include/eye_position_3d_node.h`
- `src/eye_position_3d_node.cpp`

**功能描述**:
使用双目三角测量计算眼睛的3D空间坐标。

**工作流程**:
```
1. 使用 message_filters 同步订阅:
   - /eye_positions_left
   - /eye_positions_right
   - 同步策略: ApproximateTime, 容差100ms
2. 匹配同一人脸 (通过 track_id)
3. 三角测量计算:
   - 计算视差: disparity = x_left - x_right
   - 计算深度: Z = fx * baseline / disparity
   - 计算X: X = (x_left - cx) * Z / fx
   - 计算Y: Y = (y_left - cy) * Z / fy
4. 有效性检查:
   - 视差范围: [5, 200] 像素
   - 深度范围: [200, 2000] mm
   - Y坐标差异: < max_y_diff
5. 发布 EyePositions3D 消息
```

**三角测量原理图**:
```
    左相机                          右相机
       |                              |
       |                              |
       ▼                              ▼
   (x_l, y_l)                    (x_r, y_r)
       |                              |
       |      disparity = x_l - x_r   |
       |◄────────────────────────────►|
       |                              |
       └──────────────┬───────────────┘
                      |
                      ▼
               3D点 (X, Y, Z)
```

**计算公式**:
```
Z = fx × baseline / disparity
X = (x_left - cx) × Z / fx
Y = (y_left - cy) × Z / fy
```

**参数配置**:

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| use_manual_camera_params | bool | true | 使用手动相机参数 |
| manual_fx | float | 455.0 | 焦距x (像素) |
| manual_fy | float | 455.0 | 焦距y (像素) |
| manual_cx | float | 640.0 | 主点x |
| manual_cy | float | 400.0 | 主点y |
| manual_baseline_mm | float | 95.0 | 基线距离 (mm) |
| min_disparity | float | 5.0 | 最小视差 (像素) |
| max_disparity | float | 200.0 | 最大视差 (像素) |
| min_depth_mm | float | 200.0 | 最小深度 (mm) |
| max_depth_mm | float | 2000.0 | 最大深度 (mm) |

**坐标系定义**:
- 参考系: 左IR相机光心
- X轴: 水平向右为正
- Y轴: 垂直向下为正
- Z轴: 远离相机为正
- 单位: 毫米 (mm)

---

## 四、消息定义

### 4.1 EyePositions.msg

**位置**: `eye_position_publisher/msg/EyePositions.msg`

```
# 2D眼睛位置消息
std_msgs/Header header
uint32 face_count                    # 检测到的人脸数量
uint64[] track_ids                   # 追踪ID
geometry_msgs/Point32[] left_eyes    # 左眼2D坐标 (像素)
geometry_msgs/Point32[] right_eyes   # 右眼2D坐标 (像素)
bool[] valid_flags                   # 有效性标志
uint8 channel_id                     # 通道ID (0=左IR, 1=右IR)
```

### 4.2 EyePositions3D.msg

**位置**: `eye_position_3d/msg/EyePositions3D.msg`

```
# 三维眼睛位置消息
# 坐标系：以左IR相机为参考系，单位：毫米(mm)

std_msgs/Header header
uint32 face_count                    # 成功计算3D坐标的人脸数量
uint64[] track_ids                   # 追踪ID
geometry_msgs/Point[] left_eyes_3d   # 左眼三维坐标 (mm)
geometry_msgs/Point[] right_eyes_3d  # 右眼三维坐标 (mm)
float32[] left_eye_disparities       # 左眼视差 (像素)
float32[] right_eye_disparities      # 右眼视差 (像素)
bool[] valid_flags                   # 有效性标志
float32[] confidence                 # 置信度 (0.0-1.0)
```

---

## 五、启动配置

### 5.1 主启动文件

**文件**: `tcl_eye_tracking_bringup/launch/bringup.launch.py`

**启动顺序** (延迟启动避免依赖问题):

| 延迟 | 节点 | 说明 |
|------|------|------|
| 0s | orbbec_camera | Gemini 335L相机驱动 |
| 1s | img_format_converter | 格式转换 |
| 2s | mono2d_body_detection | 人脸检测 |
| 3s | face_landmarks_detection | 关键点检测 |
| 4s | eye_position_publisher | 眼睛坐标提取 |
| 5s | eye_position_3d | 3D坐标计算 |

### 5.2 FastDDS配置

**问题**: HbmMsg1080P使用6MB固定数组，FastDDS共享内存会导致分配器冲突。

**解决方案**: 禁用FastDDS共享内存

```xml
<!-- fastdds_no_shm.xml -->
<participant profile_name="participant_profile" is_default_profile="true">
    <rtps>
        <useBuiltinTransports>true</useBuiltinTransports>
        <builtinTransports max_msg_size="65500">UDPv4</builtinTransports>
    </rtps>
</participant>
```

**环境变量**:
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/fastdds_no_shm.xml
export FASTDDS_DEFAULT_PROFILES_FILE=/path/to/fastdds_no_shm.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export RMW_FASTRTPS_USE_SHM=0
```

---

## 六、数据流与性能

### 6.1 数据流图

```
Orbbec Camera (30fps, 1280x800, mono8)
         │
         ▼
img_format_converter (mono8→NV12, ~4ms)
         │
         ▼
mono2d_body_detection (BPU推理, ~38ms)
         │
         ▼
face_landmarks_detection (BPU推理, ~50-100ms)
         │
         ▼
eye_position_publisher (CPU计算, <1ms)
         │
         ▼
eye_position_3d (三角测量, <1ms)
         │
         ▼
/eye_positions_3d (X, Y, Z in mm)
```

### 6.2 性能指标

| 指标 | 数值 |
|------|------|
| 相机帧率 | 30 fps |
| 端到端延迟 | 100-150 ms |
| 3D输出帧率 | 2-3 fps |
| 有效率 | 30-60% |

### 6.3 延迟分解

| 阶段 | 耗时 |
|------|------|
| 格式转换 | ~4ms |
| 人脸检测推理 | ~38ms |
| 关键点检测推理 | ~50-100ms |
| 眼睛坐标提取 | <1ms |
| 3D三角测量 | <1ms |

---

## 七、关键技术点

### 7.1 双路并行处理

系统采用双路并行架构，左右IR相机数据独立处理：
- 每个节点同时处理左右两路数据
- 使用 channel_id 区分数据来源
- 最终在 eye_position_3d 节点进行同步融合

### 7.2 BPU硬件加速

利用Horizon X5平台的BPU进行AI推理：
- 人脸检测: FasterRCNN多任务模型
- 关键点检测: 106点人脸关键点模型
- 支持异步推理提高吞吐量

### 7.3 消息同步机制

- **图像-AI消息匹配**: AiMsgManage 基于时间戳匹配
- **左右相机同步**: message_filters ApproximateTime策略
- **输出排序**: NodeOutputManage 解决异步推理乱序问题

### 7.4 参数隔离

为避免ROS2 launch参数冲突，各节点参数使用前缀：
- mono2d_body_detection: `mono2d_` 前缀
- face_landmarks_detection: `landmarks_` 前缀
- eye_position_publisher: `eye_` 前缀

### 7.5 QoS配置

- 图像传输: SensorDataQoS (BEST_EFFORT)
- AI消息: 默认QoS (RELIABLE)
- 3D输出: SensorDataQoS (BEST_EFFORT)

---

## 附录

### A. 快速启动

```bash
# 1. 设置环境
source /opt/tros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 2. 启动系统
ros2 launch tcl_eye_tracking_bringup bringup.launch.py

# 3. 查看3D输出
ros2 topic echo /eye_positions_3d
```

### B. 常见问题

**Q1: 节点崩溃，报"incompatible type"错误**
- 原因: 参数名冲突导致话题配置错误
- 解决: 检查各节点参数是否使用了正确的前缀

**Q2: 3D有效率为0%**
- 原因: QoS不兼容或相机参数错误
- 解决: 检查QoS配置，验证相机内参

**Q3: 帧率过低**
- 原因: BPU推理耗时
- 解决: 降低输入分辨率或使用更轻量模型

---

*报告结束*



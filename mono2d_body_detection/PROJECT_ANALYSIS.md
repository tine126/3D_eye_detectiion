# mono2d_body_detection 项目工程分析报告

> 分析日期：2026-01-27
> 分析目的：为"人眼定位算法定制"项目提供技术基础

---

## 一、一句话定位

**mono2d_body_detection** 是一个基于 ROS2 + hobot_dnn 的**单目 RGB 人体检测推理节点**，在地平线 RDK 系列开发板（X3/Ultra/X5/S100/S600）上利用 BPU 进行模型推理，支持 **FasterRCNN**（人体/人头/人脸/人手框 + 17点人体关键点）和 **YOLO-Pose**（人体框 + 17点关键点）两种模型，典型场景包括人体检测、姿态估计、人员跟踪等智能视觉应用。

---

## 二、仓库地图

### 2.1 目录树（3层）

```
mono2d_body_detection/
├── config/                          # 配置文件与模型目录
│   ├── Rdkultra/                    # RDK Ultra 平台模型
│   │   └── multitask_body_head_face_hand_kps_960x544.hbm
│   ├── s100/                        # RDK S100 平台模型
│   │   └── yolo11x_pose_nashe_640x640_nv12.hbm
│   ├── s600/                        # RDK S600 平台模型
│   │   └── yolo11n_pose_nashp_640x640_nv12.hbm
│   ├── x3/                          # RDK X3 平台模型
│   │   └── multitask_body_head_face_hand_kps_960x544.hbm
│   ├── x5/                          # RDK X5 平台模型
│   │   └── multitask_body_head_face_hand_kps_960x544.hbm
│   ├── iou2_method_param.json       # MOT跟踪参数(body/face/head)
│   ├── iou2_euclid_method_param.json # MOT跟踪参数(hand)
│   ├── person_body.jpg              # 测试图片
│   └── 960x544.nv12                 # 测试NV12图片
├── include/                         # 头文件
│   ├── mono2d_body_det_node.h       # 主节点类声明
│   └── post_process/
│       └── yolo_pose_parser.h       # YOLO-Pose后处理解析器
├── src/                             # 源文件
│   ├── main.cpp                     # 可执行文件入口
│   ├── mono2d_body_det_node.cpp     # 主节点实现
│   └── post_process/
│       └── yolo_pose_parser.cpp     # YOLO-Pose后处理实现
├── launch/                          # Launch文件
│   ├── mono2d_body_detection.launch.py           # 完整启动(含相机)
│   └── mono2d_body_detection_without_camera.launch.py  # 无相机启动
├── imgs/                            # 文档图片
│   └── mono2d_body_detecion_render.jpg
├── CMakeLists.txt                   # 构建配置
├── package.xml                      # ROS2包描述
├── README.md / README_cn.md         # 文档
└── CHANGELOG.md                     # 变更日志
```

### 2.2 各目录作用说明

| 目录 | 作用 |
|------|------|
| `config/` | 存放各平台的 `.hbm` 模型文件、MOT跟踪配置JSON、测试图片 |
| `include/` | C++ 头文件，定义 `Mono2dBodyDetNode` 类和后处理解析器接口 |
| `src/` | C++ 源文件，包含节点实现和 YOLO-Pose 后处理逻辑 |
| `launch/` | ROS2 launch 文件，编排相机、编解码、检测节点、Web展示的启动流程 |
| `imgs/` | README 文档中使用的效果展示图片 |

---

## 三、运行链路

### 3.1 启动流程

```
ros2 launch mono2d_body_detection mono2d_body_detection.launch.py
                    │
                    ▼
┌─────────────────────────────────────────────────────────────┐
│  launch/mono2d_body_detection.launch.py                     │
│  根据 CAM_TYPE 环境变量选择相机节点:                         │
│  - mipi → mipi_cam (shared_mem)                             │
│  - usb  → hobot_usb_cam                                     │
│  - fb   → hobot_image_publisher (本地图片回灌)               │
│  启动: shared_mem_node → camera_node → codec_node →         │
│        mono2d_body_det_node → web_node                      │
└─────────────────────────────────────────────────────────────┘
                    │
                    ▼
┌─────────────────────────────────────────────────────────────┐
│  可执行文件入口: src/main.cpp:21-30                          │
│  rclcpp::init() → rclcpp::spin(Mono2dBodyDetNode) → shutdown│
└─────────────────────────────────────────────────────────────┘
```

### 3.2 节点初始化流程

```cpp
// src/mono2d_body_det_node.cpp:296-458
Mono2dBodyDetNode::Mono2dBodyDetNode()
    │
    ├─► 声明并获取参数 (is_sync_mode, model_file_name, is_shared_mem_sub, ...)
    │
    ├─► Init() → 调用父类 DnnNode::Init() 加载模型
    │
    ├─► GetModel() → 获取模型管理器，查询模型名称
    │
    ├─► 创建 Publisher: msg_publisher_ (ai_msgs::msg::PerceptionTargets)
    │
    ├─► GetModelInputSize() → 获取模型输入尺寸
    │
    ├─► 根据 is_shared_mem_sub_ 创建 Subscriber:
    │   ├─ =1 → sharedmem_img_subscription_ (/hbmem_img)
    │   └─ =0 → ros_img_subscription_ (/image_raw)
    │
    └─► 初始化 MOT 跟踪器 (hobot_mots_)
```

### 3.3 主循环（推理链路）

```
┌──────────────────────────────────────────────────────────────────┐
│ 图像回调 (SharedMemImgProcess / RosImgProcess)                   │
│ src/mono2d_body_det_node.cpp:839-963 / 966-1093                  │
└──────────────────────────────────────────────────────────────────┘
                    │
                    ▼
┌──────────────────────────────────────────────────────────────────┐
│ 1. 抽帧控制: if (++gap_cnt < image_gap_) return;                 │
│    src/mono2d_body_det_node.cpp:844-848 / 971-975                │
└──────────────────────────────────────────────────────────────────┘
                    │
                    ▼
┌──────────────────────────────────────────────────────────────────┐
│ 2. 图像预处理: 转换为 NV12PyramidInput                           │
│    - rgb8/bgr8 → GetNV12PyramidFromBGRImg()                      │
│    - nv12 → hobotcv_resize() + GetNV12PyramidFromNV12Img()       │
│    src/mono2d_body_det_node.cpp:869-913 / 1007-1037              │
└──────────────────────────────────────────────────────────────────┘
                    │
                    ▼
┌──────────────────────────────────────────────────────────────────┐
│ 3. 推理: Predict() → DnnNode::Run()                              │
│    src/mono2d_body_det_node.cpp:829-837                          │
└──────────────────────────────────────────────────────────────────┘
                    │
                    ▼
┌──────────────────────────────────────────────────────────────────┐
│ 4. 后处理回调: PostProcess()                                     │
│    src/mono2d_body_det_node.cpp:474-827                          │
│    ├─ model_type_==0 → FasterRCNN解析 (hobot::dnn_node::parser)  │
│    └─ model_type_==1 → YoloPoseParse() (自定义解析器)             │
└──────────────────────────────────────────────────────────────────┘
                    │
                    ▼
┌──────────────────────────────────────────────────────────────────┐
│ 5. MOT跟踪: DoMot() (非X86平台)                                   │
│    src/mono2d_body_det_node.cpp:1096-1137                        │
└──────────────────────────────────────────────────────────────────┘
                    │
                    ▼
┌──────────────────────────────────────────────────────────────────┐
│ 6. 发布结果: msg_publisher_->publish(ai_msgs::PerceptionTargets) │
│    src/mono2d_body_det_node.cpp:824                              │
└──────────────────────────────────────────────────────────────────┘
```

---

## 四、ROS2 接口

### 4.1 订阅的图像 Topic

| 通信方式 | Topic名称 | 消息类型 | 切换方式 |
|----------|-----------|----------|----------|
| **共享内存** (默认) | `/hbmem_img` | `hbm_img_msgs::msg::HbmMsg1080P` | `is_shared_mem_sub=1` |
| **普通ROS** | `/image_raw` | `sensor_msgs::msg::Image` | `is_shared_mem_sub=0` |

**切换方式代码证据**：
- 参数声明：`src/mono2d_body_det_node.cpp:300`
- 订阅创建：`src/mono2d_body_det_node.cpp:413-452`

```cpp
// 共享内存订阅
if (is_shared_mem_sub_) {
  sharedmem_img_subscription_ = this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
      sharedmem_img_topic_name_, ...);  // 默认 "/hbmem_img"
} else {
  ros_img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      ros_img_topic_name_, ...);  // 默认 "/image_raw"
}
```

### 4.2 发布的 AI 结果 Topic

| Topic 名称 | 消息类型 | 默认值 |
|-----------|----------|--------|
| `/hobot_mono2d_body_detection` | `ai_msgs::msg::PerceptionTargets` | 可通过 `ai_msg_pub_topic_name` 参数修改 |

### 4.3 消息结构详解

#### PerceptionTargets 结构

```
std_msgs/Header header          # 时间戳、frame_id
int16 fps                       # 输出帧率
Perf[] perfs                    # 性能统计数组
Target[] targets                # 检测目标数组
Target[] disappeared_targets    # 消失目标数组（用于跟踪）
```

#### Target 结构

```
string type                     # 目标类型，固定为 "person"
uint64 track_id                 # 跟踪ID（track_mode=1时有效）
Roi[] rois                      # 检测框数组
  - string type                 # "body"/"head"/"face"/"hand"
  - sensor_msgs/RegionOfInterest rect  # x_offset, y_offset, width, height
Point[] points                  # 关键点数组
  - string type                 # "body_kps"
  - geometry_msgs/Point32[] point      # 17个关键点坐标
  - float32[] confidence        # 每个关键点置信度
```

#### 性能统计字段（Perf）

| 字段名 | 说明 |
|--------|------|
| `{model_name}_preprocess` | 预处理耗时 |
| `{model_name}_predict_infer` | 推理耗时 |
| `{model_name}_predict_parse` | 解析耗时 |
| `{model_name}_postprocess` | 后处理耗时 |
| `{model_name}_pipeline` | 端到端延迟 |

代码证据：`src/mono2d_body_det_node.cpp:716-763`

---

## 五、模型与输出

### 5.1 支持的模型类型

| model_type | 模型名称 | 支持平台 | 输入尺寸 | 输出内容 |
|------------|---------|----------|----------|----------|
| **0** | FasterRCNN (multitask) | X3/Ultra/X5 | 960×544 | body/head/face/hand框 + 17点关键点 |
| **1** | YOLO11-Pose | S100/S600 | 640×640 | body框 + 17点关键点 |

### 5.2 FasterRCNN 模型输出结构

代码证据：`include/mono2d_body_det_node.h:116-130`

```cpp
const int32_t model_output_count_ = 9;
const int32_t body_box_output_index_ = 1;   // 人体框
const int32_t head_box_output_index_ = 3;   // 人头框
const int32_t face_box_output_index_ = 5;   // 人脸框
const int32_t hand_box_output_index_ = 7;   // 人手框
const int32_t kps_output_index_ = 8;        // 17点关键点
```

### 5.3 YOLO-Pose 模型输出结构

代码证据：`src/post_process/yolo_pose_parser.cpp:148-154`

```cpp
// 3个尺度的输出 (stride=8,16,32)
std::vector<std::vector<int32_t>> orders = {{1, 0, 2}, {4, 3, 5}, {7, 6, 8}};
// 每个尺度包含: boxes_tensor, cls_tensor, kpts_tensor
// 关键点数量: KPT_NUM = 17 (include/post_process/yolo_pose_parser.h:47)
```

### 5.4 模型文件配置

通过 `model_file_name` 参数指定，默认值：
- 声明位置：`src/mono2d_body_det_node.cpp:299`
- 默认值：`config/multitask_body_head_face_hand_kps_960x544.hbm`

Launch 文件中可覆盖：
```python
# launch/mono2d_body_detection.launch.py:29-31
model_file_name_launch_arg = DeclareLaunchArgument(
    "kps_model_file_name",
    default_value="config/multitask_body_head_face_hand_kps_960x544.hbm"
)
```

---

## 六、参数总表

| 参数名 | 类型 | 默认值 | 作用 | 影响范围 | 声明位置 |
|--------|------|--------|------|----------|----------|
| `is_sync_mode` | int | 0 | 推理模式：0=异步，1=同步 | 推理调度 | `mono2d_body_det_node.cpp:298` |
| `model_file_name` | string | `config/multitask_...hbm` | 模型文件路径 | 模型加载 | `mono2d_body_det_node.cpp:299` |
| `is_shared_mem_sub` | int | 1 | 图像订阅方式：1=共享内存，0=普通ROS | **订阅Topic** | `mono2d_body_det_node.cpp:300` |
| `ai_msg_pub_topic_name` | string | `/hobot_mono2d_body_detection` | AI结果发布Topic名 | **发布Topic** | `mono2d_body_det_node.cpp:301-302` |
| `ros_img_topic_name` | string | `/image_raw` | 普通ROS图像Topic名 | 订阅Topic | `mono2d_body_det_node.cpp:303-304` |
| `sharedmem_img_topic_name` | string | `/hbmem_img` | 共享内存图像Topic名 | 订阅Topic | `mono2d_body_det_node.cpp:305-306` |
| `image_gap` | int | 1 | 抽帧间隔：1=每帧处理，N=每N帧处理1帧 | **帧跳/性能** | `mono2d_body_det_node.cpp:307` |
| `dump_render_img` | int | 0 | 是否保存渲染图：0=否，1=是 | 调试 | `mono2d_body_det_node.cpp:308` |
| `track_mode` | int | 1 | 跟踪模式：0=不跟踪，1=跟踪 | MOT功能 | `mono2d_body_det_node.cpp:309` |
| `model_type` | int | 1 | 模型类型：0=FasterRCNN，1=YOLO-Pose | **推理模式** | `mono2d_body_det_node.cpp:310` |

### 参数生效位置

- 参数声明：`src/mono2d_body_det_node.cpp:298-310`
- 参数获取：`src/mono2d_body_det_node.cpp:312-324`
- Launch覆盖：`launch/mono2d_body_detection.launch.py:156-161`

---

## 七、人眼定位算法定制建议

### 7.1 推荐方案：新建独立算法节点（解耦方式）

**理由**：
1. **最小侵入性**：不修改现有检测节点，保持原有功能稳定
2. **复用现有输出**：订阅 `/hobot_mono2d_body_detection` 获取人体关键点
3. **灵活部署**：可独立启停、参数调优、模型替换
4. **易于调试**：单独节点便于性能分析和问题定位

**架构设计**：
```
[mono2d_body_detection] ──► /hobot_mono2d_body_detection ──► [eye_localization_node]
                                                                       │
                                                                       ▼
                                                               /eye_localization_result
```

**实现要点**：
1. 订阅 `ai_msgs::msg::PerceptionTargets`
2. 从 `target.points` 中提取人脸关键点（如果使用FasterRCNN的face框）
3. 或从 `target.rois` 中提取 `type="face"` 的框，再做人眼定位

### 7.2 备选方案：修改现有节点

如果需要在 BPU 上运行人眼定位模型，可以考虑：

**插入点1：PostProcess() 函数**
- 位置：`src/mono2d_body_det_node.cpp:474-827`
- 在解析完人体关键点后，添加人眼定位逻辑
- 优点：可复用已有的图像金字塔数据

**插入点2：新增后处理解析器**
- 参考：`src/post_process/yolo_pose_parser.cpp`
- 新建 `eye_localization_parser.cpp`
- 在 `PostProcess()` 中根据 `model_type` 调用

**插入点3：扩展模型输出**
- 如果训练了包含人眼关键点的模型
- 修改 `box_outputs_index_type_` 添加新的输出类型
- 位置：`include/mono2d_body_det_node.h:126-130`

### 7.3 关键点索引参考

YOLO-Pose 17点关键点顺序（COCO格式）：

| 索引 | 名称 | 索引 | 名称 | 索引 | 名称 |
|------|------|------|------|------|------|
| 0 | nose | 6 | right_shoulder | 12 | right_hip |
| 1 | **left_eye** | 7 | left_elbow | 13 | left_knee |
| 2 | **right_eye** | 8 | right_elbow | 14 | right_knee |
| 3 | left_ear | 9 | left_wrist | 15 | left_ankle |
| 4 | right_ear | 10 | right_wrist | 16 | right_ankle |
| 5 | left_shoulder | 11 | left_hip | | |

**人眼定位可直接使用索引 1 和 2**（left_eye, right_eye）

代码位置：`src/post_process/yolo_pose_parser.cpp:101-114`

### 7.4 更精细人眼定位建议

如果需要更精细的人眼定位（如瞳孔中心），建议：

1. 基于 face 框裁剪人脸区域
2. 使用专门的人眼/瞳孔检测模型
3. 输出眼睛边界框或瞳孔坐标

---

## 八、总结

建议采用**解耦方式**新建独立节点，订阅现有检测结果进行二次处理。如果需要在 BPU 上运行人眼模型，则在 `PostProcess()` 函数中添加调用逻辑，参考 `YoloPoseParse()` 的实现模式。

---

## 附录：核心文件速查表

| 文件 | 作用 |
|------|------|
| `src/main.cpp` | 可执行文件入口 |
| `src/mono2d_body_det_node.cpp` | 主节点实现（参数、订阅、推理、发布） |
| `include/mono2d_body_det_node.h` | 节点类声明、模型输出索引定义 |
| `src/post_process/yolo_pose_parser.cpp` | YOLO-Pose 后处理解析 |
| `launch/mono2d_body_detection.launch.py` | 完整启动文件 |
| `config/iou2_method_param.json` | MOT 跟踪参数配置 |

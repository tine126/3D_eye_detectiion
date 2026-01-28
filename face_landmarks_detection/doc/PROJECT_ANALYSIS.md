# face_landmarks_detection 项目分析报告

## 一句话定位

**人脸106关键点检测ROS2功能包**：输入为NV12格式图像 + 人脸检测框（ROI），输出为106个2D关键点的像素坐标 `(x, y, score)`。

- 关键点数量由模型输出张量的 `shape[3]` 决定（`src/face_landmarks_det_output_parser.cpp:67`）
- 格式定义在 `include/face_landmarks_det_output_parser.h:27-41` 的 `Point_` 和 `Landmarks` 结构体中

---

## 仓库地图

```
face_landmarks_detection/
├── CMakeLists.txt              # 构建配置
├── package.xml                 # ROS2包描述，依赖 dnn_node/ai_msgs/hbm_img_msgs
├── README.md / README_CN.md    # 使用文档
├── LICENSE                     # Apache-2.0
│
├── config/                     # 配置与模型
│   ├── image.png               # 离线测试用图
│   ├── x3/faceLandmark106pts.hbm   # RDK X3 平台模型
│   └── x5/faceLandmark106pts.hbm   # RDK X5 平台模型
│
├── doc/
│   └── face_landmarks_det_render.png  # 效果示例图
│
├── include/                    # 头文件
│   ├── face_landmarks_det_node.h          # 主节点类定义
│   ├── face_landmarks_det_output_parser.h # 输出解析器
│   ├── ai_msg_manage.h                    # AI消息缓存与时间戳匹配
│   └── img_convert_utils.h                # BGR↔NV12 转换工具
│
├── launch/                     # 启动文件
│   ├── face_landmarks_det_node.launch.py       # 单节点启动
│   ├── body_det_face_landmarks_det.launch.py   # 联合启动
│   └── body_det_face_landmarks_det_fb.launch.py
│
└── src/                        # 源文件
    ├── main.cpp                           # 入口
    ├── face_landmarks_det_node.cpp        # 核心逻辑
    ├── face_landmarks_det_output_parser.cpp # 模型输出解码
    ├── ai_msg_manage.cpp                  # AI消息管理
    └── img_convert_utils.cpp              # 颜色空间转换
```

---

## 运行链路

### 启动阶段

```
ros2 launch face_landmarks_detection face_landmarks_det_node.launch.py
    ↓
main.cpp:18-24
    rclcpp::spin(std::make_shared<FaceLandmarksDetNode>())
    ↓
FaceLandmarksDetNode 构造函数 (face_landmarks_det_node.cpp:32-130)
    1. 声明参数 (feed_type, model_file_name, is_shared_mem_sub 等)
    2. Init() 加载模型
    3. GetModelInputSize() 获取模型输入尺寸
    4. 根据 feed_type 选择运行模式
```

### 离线模式 (feed_type=1)

```
Feedback() (face_landmarks_det_node.cpp:387-464)
    ↓
cv::imread() 读取本地图像
    ↓
utils::bgr_to_nv12_mat() 转换为 NV12
    ↓
ImageProc::GetNV12PyramidFromNV12Img() 构建金字塔输入
    ↓
解析 roi_xyxy 参数构建 hbDNNRoi
    ↓
Predict() → Run() 执行推理
    ↓
PostProcess() 后处理
    ↓
Render() 渲染结果到 render.png
SaveLandmarksToTxt() 保存坐标到 face_landmarks.txt
```

### 在线模式 (feed_type=0)

```
创建订阅者和发布者:
  - ai_msg_subscription_ 订阅 /hobot_mono2d_body_detection
  - sharedmem_img_subscription_ 或 ros_img_subscription_ 订阅图像
  - ai_msg_publisher_ 发布 /face_landmarks_detection
  - predict_task_ 推理线程

图像回调 SharedMemImgProcess/RosImgProcess (cpp:488-706)
    ↓
转换为 NV12PyramidInput，缓存到 cache_img_ 队列
    ↓
AI消息回调 AiMsgProcess (cpp:474-486)
    ↓
ai_msg_manage_->Feed() 缓存AI消息

推理线程 RunPredict() (cpp:555-637)
    ↓
从 cache_img_ 取图像，通过时间戳匹配获取对应的 AI消息
    ↓
ai_msg_manage_->GetTargetRois() 提取 type="face" 的 ROI
    ↓
NormalizeRoi() 扩展ROI (expand_scale_=1.25) 并校验尺寸
    ↓
Predict() → Run() 执行 ROI 推理
    ↓
PostProcess() (cpp:161-384)
    ↓
FaceLandmarksDetOutputParser::Parse() 解码输出张量
    ↓
构建 ai_msgs::msg::PerceptionTargets 消息
    ↓
ai_msg_publisher_->publish() 发布结果
```

---

## ROS2 接口

### 订阅 Topics

| Topic名称 | 消息类型 | 参数控制 | 说明 |
|-----------|---------|---------|------|
| `/hbmem_img` | `hbm_img_msgs::msg::HbmMsg1080P` | `sharedmem_img_topic_name` | 共享内存图像（`is_shared_mem_sub=1`） |
| `/image_raw` | `sensor_msgs::msg::Image` | `ros_img_topic_name` | ROS标准图像（`is_shared_mem_sub=0`） |
| `/hobot_mono2d_body_detection` | `ai_msgs::msg::PerceptionTargets` | `ai_msg_sub_topic_name`（硬编码） | 上游人体/人脸检测结果 |

### 发布 Topics

| Topic名称 | 消息类型 | 参数控制 | 说明 |
|-----------|---------|---------|------|
| `/face_landmarks_detection` | `ai_msgs::msg::PerceptionTargets` | `ai_msg_pub_topic_name` | 关键点检测结果 |

### 消息字段说明

```cpp
// 发布的消息结构 (face_landmarks_det_node.cpp:260-375)
ai_msgs::msg::PerceptionTargets
├── header
│   ├── frame_id              # 图像帧ID
│   └── stamp                 # 时间戳（与输入图像一致）
├── fps                       # 输出帧率
├── targets[]                 # 检测目标数组
│   ├── type                  # 目标类型（继承自上游）
│   ├── track_id              # 跟踪ID（继承自上游）
│   ├── rois[]                # ROI数组
│   │   └── type="face"       # 人脸框
│   └── points[]              # 关键点数组
│       └── ai_msgs::msg::Point
│           ├── type="face_kps"           # 关键点类型标识
│           └── point[106]                # 106个点
│               ├── x (float32)           # 像素X坐标
│               └── y (float32)           # 像素Y坐标
├── perfs[]                   # 性能统计
│   ├── type                  # 阶段名称
│   ├── stamp_start/stamp_end # 起止时间戳
│   └── time_ms_duration      # 耗时(ms)
└── disappeared_targets       # 消失目标（透传）
```

### frame_id / 时间戳策略

- **frame_id**: 直接继承输入图像消息的 `header.frame_id`（`face_landmarks_det_node.cpp:523`）
- **时间戳**: 使用输入图像的时间戳进行 AI 消息匹配（`ai_msg_manage.cpp:41-42`）

---

## 模型与输入输出细节

### 模型文件位置

| 平台 | 路径 |
|-----|------|
| RDK X3 | `config/x3/faceLandmark106pts.hbm` |
| RDK X5 | `config/x5/faceLandmark106pts.hbm` |

默认加载路径由参数 `model_file_name` 控制。

### 模型输入

| 属性 | 值 | 证据 |
|-----|-----|------|
| 输入类型 | ROI Infer（裁剪推理） | `model_task_type_ = ModelTaskType::ModelRoiInferType` (cpp:173) |
| 颜色空间 | NV12 | `ImageProc::GetNV12PyramidFromNV12Img()` |
| ROI尺寸限制 | 宽高 ∈ [16, 255) | `roi_size_min_=16`, `roi_size_max_=255` (cpp:169-170) |
| ROI扩展比例 | 1.25 | `expand_scale_=1.25` (cpp:166) |
| ROI对齐要求 | left/top偶数，right/bottom奇数 | `NormalizeRoi()` (cpp:810-814) |

### 模型输出

| 属性 | 值 | 证据 |
|-----|-----|------|
| 输出张量数量 | 2 | `output_tensors[0]` 和 `output_tensors[1]` |
| 张量类型 | `HB_DNN_TENSOR_TYPE_S32` | parser.cpp:46-47 |
| 张量布局 | `HB_DNN_LAYOUT_NHWC` | parser.cpp:46-47 |
| 张量含义 | axis=0 → X方向热力图，axis=1 → Y方向热力图 | parser.cpp:49-50 |
| 输出形状 | `[batch, H, W, 106]` | parser.cpp:62-63, 67 |
| 量化方式 | SHIFT / SCALE / NONE | parser.cpp:128-146 |

### 解码算法

```
对于每个 ROI (batch):
  对于每个关键点 (0-105):
    1. 在热力图向量中找 argmax 位置 (max_index)
    2. 亚像素插值: 根据左右邻居值调整 ±0.25
    3. 坐标映射:
       x = max_index * roi_width / vector_size + roi.left
       y = max_index * roi_height / vector_size + roi.top
    4. 置信度 = min(score, max_value/2.0)，上限1.0
```

证据：`src/face_landmarks_det_output_parser.cpp:99-251`

---

## 参数总表

| 参数名 | 默认值 | 作用 | 重启/重载 |
|-------|-------|------|----------|
| `feed_type` | `0` | 0=在线推理，1=离线推理 | 需重启 |
| `feed_image_path` | `./config/image.png` | 离线模式输入图像路径 | 需重启 |
| `roi_xyxy` | `"0,0,0,0"` | 离线模式人脸ROI | 需重启 |
| `is_sync_mode` | `0` | 0=异步推理，1=同步推理 | 需重启 |
| `model_file_name` | `./config/faceLandmark106pts.hbm` | 模型文件路径 | 需重启 |
| `is_shared_mem_sub` | `1` | 0=ROS Image，1=共享内存 | 需重启 |
| `dump_render_img` | `0` | 0=不保存，1=每30帧保存 | 运行时生效 |
| `ai_msg_pub_topic_name` | `/face_landmarks_detection` | 发布topic名称 | 需重启 |
| `ros_img_topic_name` | `/image_raw` | ROS图像订阅topic | 需重启 |
| `sharedmem_img_topic_name` | `/hbmem_img` | 共享内存图像订阅topic | 需重启 |
| `log_level` | `info` | 日志级别 | 需重启 |

**注意**：所有参数在节点构造函数中通过 `declare_parameter` 声明，运行时不支持动态修改。

---

## 对"人眼定位定制"的价值

### 106点关键点中的眼睛相关点

**仓库未提供具体索引定义**。根据业界通用的106点人脸关键点标准：

| 区域 | 点索引（通用标准） | 说明 |
|-----|------------------|------|
| 左眼轮廓 | 33-41 (9点) | 左眼上下眼睑 |
| 右眼轮廓 | 42-50 (9点) | 右眼上下眼睑 |
| 左眼瞳孔 | 104 | 左眼中心 |
| 右眼瞳孔 | 105 | 右眼中心 |
| 左眼角 | 33(外眼角), 35(内眼角) | |
| 右眼角 | 42(内眼角), 46(外眼角) | |

### 获取眼睛中心/眼角坐标的方法

**方法1：直接使用瞳孔点（如果模型支持）**

```cpp
auto left_eye_center = face_landmarks_det_result->values[roi_idx][104];
auto right_eye_center = face_landmarks_det_result->values[roi_idx][105];
```

**方法2：从眼睛轮廓点计算中心**

```cpp
float left_eye_x = 0, left_eye_y = 0;
for (int i = 33; i <= 41; i++) {
    left_eye_x += points[i].x;
    left_eye_y += points[i].y;
}
left_eye_x /= 9;
left_eye_y /= 9;
```

**方法3：从眼角点计算中心**

```cpp
float left_eye_x = (points[33].x + points[35].x) / 2;
float left_eye_y = (points[33].y + points[35].y) / 2;
```

### 改造建议

1. **确认点索引**：运行离线推理，输出 `face_landmarks.txt`，对照渲染图 `render.png` 确认眼睛相关点的实际索引

2. **添加眼睛中心输出**：在 `PostProcess()` 中计算眼睛中心，添加到消息中

3. **修改位置**：
   - 计算逻辑：`src/face_landmarks_det_node.cpp:313-323` 附近
   - 消息定义：复用 `ai_msgs::msg::Point`

4. **验证命令**：

```bash
ros2 launch face_landmarks_detection face_landmarks_det_node.launch.py \
    feed_type:=1 \
    feed_image_path:=your_face.png \
    roi_xyxy:=x1,y1,x2,y2
# 查看输出：face_landmarks.txt 和 render.png
```

---

## 附录：关键代码位置索引

| 功能 | 文件 | 行号 |
|-----|------|-----|
| 节点入口 | `src/main.cpp` | 18-24 |
| 参数声明 | `src/face_landmarks_det_node.cpp` | 36-47 |
| 模型加载 | `src/face_landmarks_det_node.cpp` | 83, 147-159 |
| 离线推理 | `src/face_landmarks_det_node.cpp` | 387-464 |
| 在线推理 | `src/face_landmarks_det_node.cpp` | 555-637 |
| 后处理 | `src/face_landmarks_det_node.cpp` | 161-384 |
| 输出解码 | `src/face_landmarks_det_output_parser.cpp` | 21-258 |
| ROI归一化 | `src/face_landmarks_det_node.cpp` | 784-859 |
| AI消息匹配 | `src/ai_msg_manage.cpp` | 35-98 |
| 结果渲染 | `src/face_landmarks_det_node.cpp` | 710-755 |

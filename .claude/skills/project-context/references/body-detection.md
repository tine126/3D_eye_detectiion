# 人体检测算法流程

基于mono2d_body_detection的人体检测处理流程。

## 数据流概览

```
IR相机 (mono8, 1280×800)
    ↓
格式转换节点 (mono8 → nv12)
    ↓
/image_raw 订阅
    ↓
RosImgProcess() → GetNV12Pyramid() → BPU推理 → FasterRCNN解析 → MOT跟踪
    ↓
/hobot_mono2d_body_detection 发布
```

## 关键处理节点

| 阶段 | 函数 | 文件位置 |
|------|------|----------|
| 图像订阅 | `RosImgProcess()` | mono2d_body_det_node.cpp:839 |
| 图像缩放 | `hobotcv_resize()` | mono2d_body_det_node.cpp:893 |
| 金字塔生成 | `GetNV12PyramidFromNV12Img()` | mono2d_body_det_node.cpp:898 |
| 推理 | `Predict()` → `Run()` | mono2d_body_det_node.cpp:946 |
| 后处理 | `PostProcess()` | mono2d_body_det_node.cpp:474 |
| 结果解析 | `FasterRCNN::Parse()` | mono2d_body_det_node.cpp:532 |
| 目标跟踪 | `DoMot()` | mono2d_body_det_node.cpp:640 | 暂不需要 |
| 结果发布 | `msg_publisher_->publish()` | mono2d_body_det_node.cpp:824 |

## 图像缩放

采集图像与模型输入尺寸不一致时，使用`hobotcv_resize()`进行**非等比例缩放**：

| 项目 | 宽度 | 高度 |
|------|------|------|
| 采集尺寸 | 1280 | 800 |
| 模型输入 | 960 | 544 |
| 缩放比例 | 0.75 | 0.68 |

**注意**: 非等比例缩放会导致图像轻微变形。

## 配置参数

| 参数 | 值 | 说明 |
|------|-----|------|
| model_type | 0 | FasterRCNN模型 |
| model_file | x5/multitask_body_head_face_hand_kps_960x544.hbm | X5专用模型 |
| 模型输入 | 960×544 (nv12) | 需要缩放 |
| 订阅话题 | /image_raw | ROS标准图像 |
| 发布话题 | hobot_mono2d_body_detection | 检测结果 |

## 输出消息

**话题**: `hobot_mono2d_body_detection`
**类型**: `ai_msgs::msg::PerceptionTargets`

包含内容:
- `targets[]` - 检测目标
  - `track_id` - 跟踪ID
  - `rois[]` - 检测框 (body/head/face/hand)
  - `points[]` - 关键点 (body_kps)
- `disappeared_targets[]` - 消失目标
- `perfs[]` - 性能统计

## 注意事项

1. **图像格式**: 支持rgb8、bgr8、nv12，不支持mono8
2. **格式转换**: 需要mono8→nv12转换节点
3. **模型类型**: RDK X5使用model_type=0 (FasterRCNN)

## 触发策略 (待开发)

**触发条件**:
- 初始帧：必须执行
- 第N帧：周期性执行（N待定，可配置）
- 无ROI：人脸关键点检测丢失ROI时立即触发

**执行逻辑**:
```
if (初始帧 || 帧计数 % N == 0 || 收到无ROI信号) {
    执行人体检测
}
```

**双路处理**: 左右路独立维护触发状态

# 人脸关键点检测算法流程

基于face_landmarks_detection的人脸关键点检测处理流程。

## 核心特点

- **级联模型**: 依赖人体检测的face roi作为输入
- **双订阅**: 同时订阅图像和AI消息
- **ROI推理**: ModelRoiInferType，在人脸框区域内推理
- **输出**: 106个人脸关键点

## 数据流概览

```
/image_raw (图像)
    ↓
RosImgProcess() → NV12金字塔 → 缓存队列
                                    ↓
/hobot_mono2d_body_detection ──→ 提取face roi
                                    ↓
                            RunPredict() [匹配图像+roi]
                                    ↓
                            Predict() → BPU推理
                                    ↓
                            PostProcess() → 解析106点
                                    ↓
                    /face_landmarks_detection 发布
```

## 关键处理节点

| 阶段 | 函数 | 文件位置 |
|------|------|----------|
| 图像订阅 | `RosImgProcess()` | face_landmarks_det_node.cpp:488 |
| AI消息订阅 | `AiMsgProcess()` | face_landmarks_det_node.cpp:474 |
| ROI提取 | `ai_msg_manage_->GetTargetRois()` | face_landmarks_det_node.cpp:584 |
| 推理 | `Predict()` | face_landmarks_det_node.cpp:631 |
| 关键点解析 | `parser->Parse()` | face_landmarks_det_node.cpp:211 |
| 结果发布 | `ai_msg_publisher_->publish()` | face_landmarks_det_node.cpp:375 |

## 配置参数

| 参数 | 值 | 说明 |
|------|-----|------|
| model_file | x5/faceLandmark106pts.hbm | X5专用模型 |
| 关键点数量 | 106点 | 人脸关键点 |
| expand_scale | 1.25 | ROI扩展比例 |
| 订阅图像话题 | /image_raw | 原始图像 |
| 订阅AI话题 | /hobot_mono2d_body_detection | 人体检测结果 |
| 发布话题 | /face_landmarks_detection | 关键点结果 |

## 输出消息

**类型**: `ai_msgs::msg::PerceptionTargets`

关键点存储在 `target.points[]` 中：
- `type`: "face_kps"
- `point[]`: 106个(x,y)坐标点

## ROI更新策略 (待开发)

**ROI来源优先级**:
1. 上一帧的人脸检测框（按比例扩大）
2. 人体检测最新得到的人脸区域
3. 若都无，触发人体检测重新获取

**更新逻辑**:
```
if (上一帧有人脸检测框) {
    ROI = 扩大(上一帧人脸框, expand_scale)
} else if (人体检测有人脸区域) {
    ROI = 人体检测的人脸区域
} else {
    触发人体检测
    ROI = 人体检测的人脸区域
}
```

**双路处理**: 左右路独立维护ROI状态

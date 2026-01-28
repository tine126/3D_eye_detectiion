# IR流处理路径

基于OrbbecSDK_ROS2的IR流（mono8灰度图）处理流程。

## 数据流概览

```
Pipeline.start() → onNewFrameSetCallback() → processLeftIrFrameFilter()
    → onNewFrameCallback() → cv_bridge转换 → 发布ROS2话题
```

## 关键处理节点

| 阶段 | 函数 | 文件位置 |
|------|------|----------|
| 取流 | `frame_set->getFrame(OB_FRAME_IR_LEFT)` | ob_camera_node.cpp:3157 |
| 过滤 | `processLeftIrFrameFilter()` | ob_camera_node.cpp:2933 |
| 回调 | `onNewFrameCallback()` | ob_camera_node.cpp:3552 |
| 转换 | `frame->as<ob::IRFrame>()` | ob_camera_node.cpp:3575 |
| 拷贝 | `memcpy(image.data, video_frame->getData())` | ob_camera_node.cpp:3715 |
| 发布 | `image_publishers_->publish()` | ob_camera_node.cpp:3738 |

## mono8格式参数

| 参数 | 值 |
|------|-----|
| SDK格式 | OB_FORMAT_Y8 |
| OpenCV类型 | CV_8UC1 |
| ROS编码 | MONO8 |
| 每像素字节 | 1 byte |
| 帧大小 | 1280 × 800 = 1,024,000 bytes |

## 特点

- mono8格式无需解码，直接memcpy拷贝原始数据
- 延迟最低，适合实时处理场景

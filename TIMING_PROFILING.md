# TCL人眼定位系统 - 耗时统计设计文档

## 一、概述

本文档定义了人眼定位系统各节点的耗时统计点，用于性能分析和优化。

### 1.1 系统数据流

```
┌─────────────┐    ┌─────────────┐    ┌──────────────────┐    ┌───────────────────────┐
│ orbbec_camera│───▶│mono8_to_nv12│───▶│mono2d_body_detect│───▶│face_landmarks_detect  │
│  (相机取流)  │    │ (格式转换)   │    │   (人体检测)      │    │    (人脸关键点)        │
└─────────────┘    └─────────────┘    └──────────────────┘    └───────────────────────┘
                                                                          │
                   ┌─────────────────┐    ┌──────────────────┐            │
                   │eye_position_3d  │◀───│ eye_position_2d  │◀───────────┘
                   │  (3D坐标计算)   │    │  (眼位置提取)     │
                   └─────────────────┘    └──────────────────┘
```

### 1.2 时间戳传递

所有节点通过 `header.stamp` 传递原始图像的采集时间戳，用于计算端到端延迟。

---

## 二、统计点定义

### 2.0 orbbec_camera (相机取流节点)

| 编号 | 统计点 | 变量名 | 单位 | 说明 |
|------|--------|--------|------|------|
| 0.1 | 帧采集时间戳 | `frame_timestamp` | ms | 相机硬件采集时间 |
| 0.2 | SDK回调延迟 | `sdk_callback_delay` | ms | 从硬件采集到SDK回调 |
| 0.3 | ROS发布延迟 | `ros_publish_delay` | ms | 从SDK回调到publish |
| 0.4 | 帧率统计 | `actual_fps` | fps | 实际发布帧率 |

**日志格式:**
```
[orbbec_camera] frame_ts=123456789 sdk_delay=1.2ms pub_delay=0.3ms fps=30.1
```

---

### 2.1 mono8_to_nv12 (格式转换节点)

| 编号 | 统计点 | 变量名 | 单位 | 说明 |
|------|--------|--------|------|------|
| 1.1 | 接收延迟 | `recv_delay` | ms | header.stamp到回调触发 |
| 1.2 | 转换耗时 | `convert_time` | ms | mono8→nv12转换 |
| 1.3 | 总处理时间 | `total_time` | ms | 回调开始到publish |

**日志格式:**
```
[mono8_to_nv12] LEFT  recv=2.1ms convert=0.8ms total=1.0ms
[mono8_to_nv12] RIGHT recv=2.0ms convert=0.8ms total=0.9ms
```

---

### 2.2 mono2d_body_detection (人体检测节点)

| 编号 | 统计点 | 变量名 | 单位 | 说明 |
|------|--------|--------|------|------|
| 2.1 | 接收延迟 | `recv_delay` | ms | header.stamp到回调触发 |
| 2.2 | 预处理耗时 | `preprocess_time` | ms | 图像resize/padding |
| 2.3 | BPU推理耗时 | `infer_time` | ms | 神经网络推理 |
| 2.4 | 后处理耗时 | `postprocess_time` | ms | NMS/解码 |
| 2.5 | 总处理时间 | `total_time` | ms | 回调开始到publish |

**日志格式:**
```
[body_det] LEFT  recv=3.5ms pre=2.1ms infer=15.3ms post=1.2ms total=18.8ms
[body_det] RIGHT recv=3.4ms pre=2.0ms infer=15.1ms post=1.1ms total=18.5ms
```

---

### 2.3 face_landmarks_detection (人脸关键点节点)

| 编号 | 统计点 | 变量名 | 单位 | 说明 |
|------|--------|--------|------|------|
| 3.1 | 接收延迟 | `recv_delay` | ms | header.stamp到回调触发 |
| 3.2 | ROI处理耗时 | `roi_time` | ms | ROI缓存查找/扩展 |
| 3.3 | 预处理耗时 | `preprocess_time` | ms | 图像裁剪/resize |
| 3.4 | BPU推理耗时 | `infer_time` | ms | 关键点模型推理 |
| 3.5 | 后处理耗时 | `postprocess_time` | ms | 关键点解码 |
| 3.6 | 总处理时间 | `total_time` | ms | 回调开始到publish |

**日志格式:**
```
[face_lmk] LEFT  recv=25.0ms roi=0.1ms pre=1.5ms infer=8.2ms post=0.5ms total=10.5ms
[face_lmk] RIGHT recv=24.8ms roi=0.1ms pre=1.4ms infer=8.1ms post=0.4ms total=10.3ms
```

---

### 2.4 eye_position_2d_node (眼位置2D节点)

| 编号 | 统计点 | 变量名 | 单位 | 说明 |
|------|--------|--------|------|------|
| 4.1 | 接收延迟 | `recv_delay` | ms | header.stamp到回调触发 |
| 4.2 | 计算耗时 | `calc_time` | ms | 眼中心点计算 |
| 4.3 | 总处理时间 | `total_time` | ms | 回调开始到publish |

**日志格式:**
```
[eye_2d] LEFT  recv=35.5ms calc=0.02ms total=0.05ms
[eye_2d] RIGHT recv=35.3ms calc=0.02ms total=0.04ms
```

---

### 2.5 eye_position_3d_node (眼位置3D节点)

| 编号 | 统计点 | 变量名 | 单位 | 说明 |
|------|--------|--------|------|------|
| 5.1 | 左路接收时间 | `left_recv_time` | ms | 左路消息到达时间 |
| 5.2 | 右路接收时间 | `right_recv_time` | ms | 右路消息到达时间 |
| 5.3 | 同步等待时间 | `sync_wait_time` | ms | 等待消息同步 |
| 5.4 | 3D计算耗时 | `calc_time` | ms | 视差+三角测量 |
| 5.5 | 总处理时间 | `total_time` | ms | 同步回调到publish |

**日志格式:**
```
[eye_3d] left_recv=35.6ms right_recv=35.4ms sync_wait=0.2ms calc=0.01ms total=0.03ms
```

---

### 2.6 端到端延迟

| 编号 | 统计点 | 变量名 | 单位 | 说明 |
|------|--------|--------|------|------|
| 6.1 | 相机→3D总延迟 | `e2e_total` | ms | 图像采集到3D输出 |
| 6.2 | 左路链路延迟 | `e2e_left` | ms | 左路完整链路 |
| 6.3 | 右路链路延迟 | `e2e_right` | ms | 右路完整链路 |

**日志格式:**
```
[eye_3d] E2E: total=45.2ms left_chain=44.8ms right_chain=44.6ms
```

---

## 三、统计点汇总

| 节点 | 统计点数量 | 日志标签 |
|------|-----------|----------|
| orbbec_camera | 4 | `[orbbec_camera]` |
| mono8_to_nv12 | 3 | `[mono8_to_nv12]` |
| mono2d_body_detection | 5 | `[body_det]` |
| face_landmarks_detection | 6 | `[face_lmk]` |
| eye_position_2d_node | 3 | `[eye_2d]` |
| eye_position_3d_node | 5 | `[eye_3d]` |
| 端到端 | 3 | `[eye_3d] E2E` |
| **总计** | **29** | - |

---

## 四、实现方式

### 4.1 时间测量工具

使用 `std::chrono` 进行高精度时间测量：

```cpp
#include <chrono>

// 获取当前时间
auto now = std::chrono::steady_clock::now();

// 计算时间差 (毫秒)
auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
double ms = duration.count() / 1000.0;
```

### 4.2 接收延迟计算

```cpp
// 从ROS消息header计算接收延迟
auto msg_time = rclcpp::Time(msg->header.stamp);
auto now_time = this->now();
double recv_delay_ms = (now_time - msg_time).seconds() * 1000.0;
```

### 4.3 日志输出控制

通过参数控制日志输出频率：

```cpp
// 参数声明
this->declare_parameter<int>("timing_log_interval", 30);  // 每30帧输出一次

// 日志输出
if (frame_count_ % timing_log_interval_ == 0) {
    RCLCPP_INFO(this->get_logger(), "[node_name] ...");
}
```

---

## 五、预期性能指标

### 5.1 单节点耗时预期

| 节点 | 预期耗时 | 备注 |
|------|----------|------|
| mono8_to_nv12 | < 2ms | 纯内存操作 |
| mono2d_body_detection | 15-25ms | BPU推理 |
| face_landmarks_detection | 8-15ms | BPU推理 |
| eye_position_2d | < 0.1ms | 纯计算 |
| eye_position_3d | < 0.1ms | 纯计算 |

### 5.2 端到端延迟预期

| 指标 | 预期值 | 说明 |
|------|--------|------|
| 总延迟 | 40-60ms | 从采集到3D输出 |
| 帧率 | 25-30fps | 受BPU推理限制 |

---

## 六、使用方法

### 6.1 启用耗时统计

```bash
ros2 launch eye_tracking eye_tracking_full.launch.py timing_log_interval:=30
```

### 6.2 查看统计日志

```bash
# 实时查看
ros2 launch eye_tracking eye_tracking_full.launch.py 2>&1 | grep -E "\[.*\].*ms"

# 保存到文件
ros2 launch eye_tracking eye_tracking_full.launch.py 2>&1 | tee timing.log
```

### 6.3 分析统计数据

```bash
# 提取各节点耗时
grep "\[body_det\]" timing.log
grep "\[face_lmk\]" timing.log
grep "\[eye_3d\] E2E" timing.log
```

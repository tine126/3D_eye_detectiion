---
name: project-context
description: |
  TCL人眼定位算法定制项目的背景信息。当需要了解项目硬件配置、算法目标、开发环境时使用此skill。
  触发场景：(1) 询问项目使用的相机或开发板 (2) 询问算法目标和要求 (3) 需要了解项目整体背景
---

# 项目背景

## 硬件平台

### 相机
- **型号**: 奥比中光 Gemini 335L
- **类型**: RGB-D深度相机
- **分辨率**: 1280×800
- **图像格式**: mono8 (灰度图)
- **增益**: 100
- **曝光时间**: 4ms

### 开发板
- **型号**: 地平线 RDK X5
- **芯片**: 地平线征程5芯片
- **特点**: 支持BPU加速推理

## 算法目标

- **任务**: 人眼定位算法
- **目的**: 在图像中精确定位人眼位置

## 开发说明

开发时需注意：
1. 算法需适配RDK X5的BPU推理
2. 相机SDK为OrbbecSDK
3. 需考虑实时性要求

## 参考文档

- **IR流处理路径**: 详见 [references/ir-stream.md](references/ir-stream.md)
- **人体检测流程**: 详见 [references/body-detection.md](references/body-detection.md)
- **人脸关键点检测流程**: 详见 [references/face-landmarks.md](references/face-landmarks.md)
- **人眼坐标输出流程**: 详见 [references/eye-positions.md](references/eye-positions.md) (待开发)
- **人眼三维坐标计算**: 详见 [references/eye-3d-positions.md](references/eye-3d-positions.md) (待开发)

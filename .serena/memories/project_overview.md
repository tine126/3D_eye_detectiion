# TCL人眼定位算法定制项目

## 项目目的
基于双目IR相机的人眼三维定位系统，通过人体检测、人脸关键点检测、双目视差计算，实现人眼的三维坐标定位。

## 硬件平台
- 相机: 奥比中光 Gemini 335L (双目IR, 1280×800, mono8)
- 开发板: 地平线 RDK X5 (ROS2 Humble)
- 帧率要求: 30fps

## 技术栈
- ROS2 Humble
- C++14
- 地平线BPU DNN推理
- OpenCV
- CMake构建系统

## 代码结构
- mono2d_body_detection/ - 人体检测节点
- face_landmarks_detection/ - 人脸关键点检测节点
- OrbbecSDK_ROS2/ - 奥比中光相机驱动

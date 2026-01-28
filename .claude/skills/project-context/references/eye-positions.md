# 人眼二维坐标输出流程

基于人脸关键点检测结果，提取人眼二维坐标的处理流程。

## 系统架构（双目）

```
Gemini 335L双目IR相机
    ├── 左路IR (1280×800)
    └── 右路IR (1280×800)
            ↓
    格式转换 (mono8→nv12)
            ↓
    人体检测 (FasterRCNN) → face roi
            ↓
    人脸关键点检测 (106点)
            ↓
    人眼坐标提取节点
            ↓
    输出: 左路/右路各自的左眼、右眼坐标
```

## 眼睛关键点索引

| 部位 | 点索引 | 说明 |
|------|--------|------|
| 左眼轮廓 | 33-41 | 9点 |
| 右眼轮廓 | 42-50 | 9点 |
| 左眼瞳孔 | 104 | 备用 |
| 右眼瞳孔 | 105 | 备用 |

## 眼睛中心计算方法

使用轮廓点计算中心：

```cpp
// 左眼中心 (点33-41的平均值)
float left_eye_x = 0, left_eye_y = 0;
for (int i = 33; i <= 41; i++) {
    left_eye_x += points[i].x;
    left_eye_y += points[i].y;
}
left_eye_x /= 9;
left_eye_y /= 9;

// 右眼中心 (点42-50的平均值)
float right_eye_x = 0, right_eye_y = 0;
for (int i = 42; i <= 50; i++) {
    right_eye_x += points[i].x;
    right_eye_y += points[i].y;
}
right_eye_x /= 9;
right_eye_y /= 9;
```

## 人眼坐标提取节点设计

**输入**:
- 订阅: `/face_landmarks_detection_left` (左路)
- 订阅: `/face_landmarks_detection_right` (右路)

**输出**:
- 发布: `/eye_positions_left`
- 发布: `/eye_positions_right`

## 注意事项

1. **坐标缩放**: 关键点是模型尺寸(960×544)，需转换回原图(1280×800)
2. **双路同步**: 左右路图像需时间戳同步
3. **点索引确认**: 建议先离线推理确认实际眼睛点索引
4. **状态**: 待开发

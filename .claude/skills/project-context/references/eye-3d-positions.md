# 人眼三维坐标计算流程

基于双目视差的人眼三维坐标计算流程。

## 输入数据

```
左路相机                          右路相机
    ↓                               ↓
左眼2D: (x_L_left, y_L_left)      左眼2D: (x_R_left, y_R_left)
右眼2D: (x_L_right, y_L_right)    右眼2D: (x_R_right, y_R_right)
```

## 三角测量公式

```
视差:  d = x_left - x_right

深度:  Z = f * B / d

三维坐标:
  X = (x_left - cx) * Z / f
  Y = (y_left - cy) * Z / f
```

参数说明：
- `f`: 焦距 (像素)
- `B`: 基线距离 (mm)
- `cx, cy`: 主点坐标
- `d`: 视差 (像素)

## 数据流

```
/eye_positions_left ──┐
                      ├──→ 三维坐标计算节点
/eye_positions_right ─┘
            ↓
      时间戳同步匹配
            ↓
      计算视差 d = x_L - x_R
            ↓
      三角测量 (X, Y, Z)
            ↓
      /eye_positions_3d
```

## 节点设计

**输入**:
- 订阅: `/eye_positions_left`
- 订阅: `/eye_positions_right`
- 相机标定参数 (内参、基线)

**输出**:
- 发布: `/eye_positions_3d`

## 注意事项

1. **图像校正**: 计算前需确保左右图像已校正对齐
2. **时间同步**: 左右路数据需严格时间戳匹配
3. **坐标系**: 输出坐标系以左相机为原点
4. **有效性检查**: 视差过小时深度计算不可靠
5. **状态**: 待开发

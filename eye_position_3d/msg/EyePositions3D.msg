# 三维眼睛位置消息
# 坐标系：以左IR相机为参考系，单位：毫米(mm)

std_msgs/Header header
uint32 face_count                    # 成功计算3D坐标的人脸数量

# 每个人脸的数据
uint64[] track_ids                   # 追踪ID
geometry_msgs/Point[] left_eyes_3d   # 左眼三维坐标 (mm)
geometry_msgs/Point[] right_eyes_3d  # 右眼三维坐标 (mm)
float32[] left_eye_disparities       # 左眼视差 (像素)
float32[] right_eye_disparities      # 右眼视差 (像素)
bool[] valid_flags                   # 有效性标志
float32[] confidence                 # 置信度 (0.0-1.0)

# 眼睛位置消息
std_msgs/Header header
uint8 channel_id              # 通道标识 (0=左IR, 1=右IR)
uint32 face_count             # 检测到的人脸数量

# 每个人脸的眼睛数据（扁平化存储）
uint64[] track_ids            # 追踪ID
geometry_msgs/Point32[] left_eyes   # 左眼中心坐标
geometry_msgs/Point32[] right_eyes  # 右眼中心坐标
bool[] valid_flags            # 有效标志

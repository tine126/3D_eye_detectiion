// Copyright (c) 2024，D-Robotics.
// eye_position_publisher: 从人脸关键点提取眼睛中心坐标

#include "eye_position_publisher_node.h"

EyePositionPublisherNode::EyePositionPublisherNode(
    const std::string &node_name,
    const rclcpp::NodeOptions &options)
    : Node(node_name, options)
{
    // 声明并获取参数
    this->declare_parameter("eye_left_sub_topic", eye_left_sub_topic_);
    this->declare_parameter("eye_left_pub_topic", eye_left_pub_topic_);
    this->declare_parameter("eye_right_sub_topic", eye_right_sub_topic_);
    this->declare_parameter("eye_right_pub_topic", eye_right_pub_topic_);

    this->get_parameter("eye_left_sub_topic", eye_left_sub_topic_);
    this->get_parameter("eye_left_pub_topic", eye_left_pub_topic_);
    this->get_parameter("eye_right_sub_topic", eye_right_sub_topic_);
    this->get_parameter("eye_right_pub_topic", eye_right_pub_topic_);

    RCLCPP_INFO(this->get_logger(), "参数配置:");
    RCLCPP_INFO(this->get_logger(), "  左IR: %s -> %s", eye_left_sub_topic_.c_str(), eye_left_pub_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  右IR: %s -> %s", eye_right_sub_topic_.c_str(), eye_right_pub_topic_.c_str());

    // 创建左IR发布者和订阅者
    left_publisher_ = this->create_publisher<eye_position_publisher::msg::EyePositions>(
        eye_left_pub_topic_, rclcpp::SensorDataQoS());
    left_subscription_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
        eye_left_sub_topic_, rclcpp::SensorDataQoS(),
        std::bind(&EyePositionPublisherNode::LeftCallback, this, std::placeholders::_1));

    // 创建右IR发布者和订阅者
    right_publisher_ = this->create_publisher<eye_position_publisher::msg::EyePositions>(
        eye_right_pub_topic_, rclcpp::SensorDataQoS());
    right_subscription_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
        eye_right_sub_topic_, rclcpp::SensorDataQoS(),
        std::bind(&EyePositionPublisherNode::RightCallback, this, std::placeholders::_1));

    // 初始化统计
    stat_start_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(this->get_logger(), "节点初始化完成");
}

void EyePositionPublisherNode::LeftCallback(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg)
{
    ProcessMessage(msg, left_publisher_, 0);
}

void EyePositionPublisherNode::RightCallback(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg)
{
    ProcessMessage(msg, right_publisher_, 1);
}

void EyePositionPublisherNode::ProcessMessage(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg,
    rclcpp::Publisher<eye_position_publisher::msg::EyePositions>::SharedPtr publisher,
    uint8_t channel_id)
{
    if (!msg || !rclcpp::ok()) return;

    // 记录处理开始时间
    struct timespec time_start = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_start);

    // 计算从消息时间戳到回调入口的传输延迟
    int64_t msg_ts_ns = static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
                        static_cast<int64_t>(msg->header.stamp.nanosec);
    int64_t callback_enter_ns = static_cast<int64_t>(time_start.tv_sec) * 1000000000LL +
                                static_cast<int64_t>(time_start.tv_nsec);
    double transport_delay_ms = static_cast<double>(callback_enter_ns - msg_ts_ns) / 1000000.0;

    // 创建输出消息
    auto eye_msg = std::make_unique<eye_position_publisher::msg::EyePositions>();
    eye_msg->header = msg->header;
    eye_msg->channel_id = channel_id;

    uint32_t face_count = 0;

    // 遍历所有targets
    for (const auto &target : msg->targets)
    {
        // 查找face_kps类型的关键点
        for (const auto &point_set : target.points)
        {
            if (point_set.type != "face_kps") continue;

            const auto &points = point_set.point;

            // 检查关键点数量
            if (static_cast<int>(points.size()) < TOTAL_LANDMARKS)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "关键点数量不足: %zu < %d", points.size(), TOTAL_LANDMARKS);
                continue;
            }

            // 计算左眼中心
            auto left_eye = CalculateEyeCenter(points, LEFT_EYE_START_IDX, LEFT_EYE_END_IDX);
            // 计算右眼中心
            auto right_eye = CalculateEyeCenter(points, RIGHT_EYE_START_IDX, RIGHT_EYE_END_IDX);

            // 添加到消息
            eye_msg->track_ids.push_back(target.track_id);
            eye_msg->left_eyes.push_back(left_eye);
            eye_msg->right_eyes.push_back(right_eye);
            eye_msg->valid_flags.push_back(true);

            // 输出每帧二维坐标日志
            RCLCPP_INFO(this->get_logger(),
                "[通道%d] 人脸%d: 左眼=(%.2f, %.2f), 右眼=(%.2f, %.2f)",
                channel_id, face_count,
                left_eye.x, left_eye.y,
                right_eye.x, right_eye.y);

            face_count++;
        }
    }

    eye_msg->face_count = face_count;

    // 发布消息
    publisher->publish(std::move(eye_msg));

    // 记录处理结束时间
    struct timespec time_end = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_end);
    double process_ms = (time_end.tv_sec - time_start.tv_sec) * 1000.0 +
                        (time_end.tv_nsec - time_start.tv_nsec) / 1000000.0;

    // 输出耗时日志
    RCLCPP_INFO(this->get_logger(),
        "[通道%d] 帧处理完成: 人脸数=%d, 传输延迟=%.2fms, 处理耗时=%.2fms",
        channel_id, face_count, transport_delay_ms, process_ms);

    // 更新统计
    stat_msg_count_++;
    stat_face_count_ += face_count;

    // 定期打印统计
    PrintStatistics();
}

geometry_msgs::msg::Point32 EyePositionPublisherNode::CalculateEyeCenter(
    const std::vector<geometry_msgs::msg::Point32>& points,
    int start_idx, int end_idx)
{
    geometry_msgs::msg::Point32 center;
    center.x = 0.0f;
    center.y = 0.0f;
    center.z = 0.0f;

    int count = end_idx - start_idx + 1;
    for (int i = start_idx; i <= end_idx; ++i)
    {
        center.x += points[i].x;
        center.y += points[i].y;
    }

    center.x /= count;
    center.y /= count;

    return center;
}

void EyePositionPublisherNode::PrintStatistics()
{
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        now - stat_start_time_).count();

    if (elapsed >= STAT_INTERVAL_SEC)
    {
        uint64_t msg_count = stat_msg_count_.load();
        uint64_t face_count = stat_face_count_.load();

        if (msg_count > 0)
        {
            double msg_fps = static_cast<double>(msg_count) / elapsed;
            double avg_faces = static_cast<double>(face_count) / msg_count;

            RCLCPP_INFO(this->get_logger(),
                "统计 [%lds]: 消息数=%lu, 帧率=%.1f, 人脸总数=%lu, 平均人脸=%.2f",
                elapsed, msg_count, msg_fps, face_count, avg_faces);
        }

        // 重置统计
        stat_msg_count_ = 0;
        stat_face_count_ = 0;
        stat_start_time_ = now;
    }
}

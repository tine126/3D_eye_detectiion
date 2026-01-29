// Copyright (c) 2024，D-Robotics.
// eye_position_publisher: 从人脸关键点提取眼睛中心坐标

#ifndef EYE_POSITION_PUBLISHER_NODE_H_
#define EYE_POSITION_PUBLISHER_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <ai_msgs/msg/perception_targets.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include "eye_position_publisher/msg/eye_positions.hpp"
#include <atomic>
#include <chrono>

class EyePositionPublisherNode : public rclcpp::Node
{
public:
    explicit EyePositionPublisherNode(
        const std::string &node_name = "eye_position_publisher_node",
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~EyePositionPublisherNode() override = default;

private:
    // ========== 眼睛关键点索引常量 ==========
    static constexpr int LEFT_EYE_START_IDX = 33;   // 左眼起始索引
    static constexpr int LEFT_EYE_END_IDX = 41;     // 左眼结束索引
    static constexpr int RIGHT_EYE_START_IDX = 42;  // 右眼起始索引
    static constexpr int RIGHT_EYE_END_IDX = 50;    // 右眼结束索引
    static constexpr int TOTAL_LANDMARKS = 106;     // 总关键点数

    // ========== 话题配置 ==========
    // 左IR
    std::string eye_left_sub_topic_ = "/face_landmarks_detection_left";
    std::string eye_left_pub_topic_ = "/eye_positions_left";
    // 右IR
    std::string eye_right_sub_topic_ = "/face_landmarks_detection_right";
    std::string eye_right_pub_topic_ = "/eye_positions_right";

    // ========== 订阅/发布 ==========
    // 左IR
    rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr left_subscription_ = nullptr;
    rclcpp::Publisher<eye_position_publisher::msg::EyePositions>::SharedPtr left_publisher_ = nullptr;
    // 右IR
    rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr right_subscription_ = nullptr;
    rclcpp::Publisher<eye_position_publisher::msg::EyePositions>::SharedPtr right_publisher_ = nullptr;

    // ========== 性能统计 ==========
    std::atomic<uint64_t> stat_msg_count_{0};
    std::atomic<uint64_t> stat_face_count_{0};
    std::chrono::steady_clock::time_point stat_start_time_;
    static constexpr int STAT_INTERVAL_SEC = 5;

    // ========== 私有方法 ==========
    void LeftCallback(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg);
    void RightCallback(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg);
    void ProcessMessage(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg,
                        rclcpp::Publisher<eye_position_publisher::msg::EyePositions>::SharedPtr publisher,
                        uint8_t channel_id);
    geometry_msgs::msg::Point32 CalculateEyeCenter(
        const std::vector<geometry_msgs::msg::Point32>& points,
        int start_idx, int end_idx);
    void PrintStatistics();
};

#endif  // EYE_POSITION_PUBLISHER_NODE_H_

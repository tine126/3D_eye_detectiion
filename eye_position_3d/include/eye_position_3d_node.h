// Copyright (c) 2024，D-Robotics.
// eye_position_3d: 双目视觉三角测量计算三维眼睛坐标

#ifndef EYE_POSITION_3D_NODE_H_
#define EYE_POSITION_3D_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include "eye_position_publisher/msg/eye_positions.hpp"
#include "eye_position_3d/msg/eye_positions3_d.hpp"

#include <atomic>
#include <chrono>
#include <unordered_map>
#include <mutex>

// 相机内参结构
struct CameraParams {
    double fx = 0.0;  // 焦距x
    double fy = 0.0;  // 焦距y
    double cx = 0.0;  // 主点x
    double cy = 0.0;  // 主点y
    bool valid = false;
};

// 单眼3D计算结果
struct Eye3DResult {
    geometry_msgs::msg::Point position;  // 3D坐标 (mm)
    float disparity = 0.0f;              // 视差 (像素)
    float confidence = 0.0f;             // 置信度
    bool valid = false;                  // 有效性
};

class EyePosition3DNode : public rclcpp::Node
{
public:
    explicit EyePosition3DNode(
        const std::string &node_name = "eye_position_3d_node",
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~EyePosition3DNode() override;

private:
    // ========== 消息类型别名 ==========
    using EyePositionsMsg = eye_position_publisher::msg::EyePositions;
    using EyePositions3DMsg = eye_position_3d::msg::EyePositions3D;
    using CameraInfoMsg = sensor_msgs::msg::CameraInfo;

    // ========== 同步策略 ==========
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        EyePositionsMsg, EyePositionsMsg>;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

    // ========== 话题配置 ==========
    std::string left_eye_topic_ = "/eye_positions_left";
    std::string right_eye_topic_ = "/eye_positions_right";
    std::string left_camera_info_topic_ = "/camera/left_ir/camera_info";
    std::string right_camera_info_topic_ = "/camera/right_ir/camera_info";
    std::string output_topic_ = "/eye_positions_3d";

    // ========== 算法参数 ==========
    double min_disparity_ = 5.0;      // 最小视差(像素)
    double max_disparity_ = 200.0;    // 最大视差(像素)
    double min_depth_mm_ = 200.0;     // 最小深度(mm)
    double max_depth_mm_ = 2000.0;    // 最大深度(mm)
    double max_y_diff_ = 10.0;        // 最大Y坐标差异(像素)

    // ========== 手动相机参数配置 ==========
    bool use_manual_camera_params_ = true;  // 是否使用手动配置
    double manual_fx_ = 619.688049;   // 手动焦距x
    double manual_fy_ = 619.688049;   // 手动焦距y
    double manual_cx_ = 638.0;        // 手动主点x
    double manual_cy_ = 396.0;        // 手动主点y
    double manual_baseline_mm_ = 95.0; // 手动基线距离(mm)

    // ========== 相机参数 ==========
    CameraParams left_camera_params_;
    CameraParams right_camera_params_;
    double baseline_mm_ = 0.0;  // 基线距离(mm)
    std::mutex camera_params_mutex_;

    // ========== 订阅/发布 ==========
    std::shared_ptr<message_filters::Subscriber<EyePositionsMsg>> left_eye_sub_;
    std::shared_ptr<message_filters::Subscriber<EyePositionsMsg>> right_eye_sub_;
    std::shared_ptr<Synchronizer> sync_;

    rclcpp::Subscription<CameraInfoMsg>::SharedPtr left_camera_info_sub_;
    rclcpp::Subscription<CameraInfoMsg>::SharedPtr right_camera_info_sub_;
    rclcpp::Publisher<EyePositions3DMsg>::SharedPtr publisher_;

    // ========== 性能统计 ==========
    std::atomic<uint64_t> stat_msg_count_{0};
    std::atomic<uint64_t> stat_face_count_{0};
    std::atomic<uint64_t> stat_valid_count_{0};
    std::chrono::steady_clock::time_point stat_start_time_;
    static constexpr int STAT_INTERVAL_SEC = 5;

    // ========== 线程安全 ==========
    std::mutex callback_mutex_;
    std::atomic<bool> is_active_{true};

    // ========== 私有方法 ==========
    void LeftCameraInfoCallback(const CameraInfoMsg::ConstSharedPtr msg);
    void RightCameraInfoCallback(const CameraInfoMsg::ConstSharedPtr msg);

    void SyncCallback(const EyePositionsMsg::ConstSharedPtr left_msg,
                      const EyePositionsMsg::ConstSharedPtr right_msg);

    Eye3DResult Calculate3DCoordinate(
        float x_left, float y_left,
        float x_right, float y_right);

    void PrintStatistics();
};

#endif  // EYE_POSITION_3D_NODE_H_

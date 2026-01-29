// Copyright (c) 2024，D-Robotics.
// eye_position_3d: 双目视觉三角测量计算三维眼睛坐标

#include "eye_position_3d_node.h"
#include <cmath>

EyePosition3DNode::EyePosition3DNode(
    const std::string &node_name,
    const rclcpp::NodeOptions &options)
    : Node(node_name, options)
{
    // ========== 声明参数 ==========
    this->declare_parameter("left_eye_topic", left_eye_topic_);
    this->declare_parameter("right_eye_topic", right_eye_topic_);
    this->declare_parameter("left_camera_info_topic", left_camera_info_topic_);
    this->declare_parameter("right_camera_info_topic", right_camera_info_topic_);
    this->declare_parameter("output_topic", output_topic_);
    this->declare_parameter("min_disparity", min_disparity_);
    this->declare_parameter("max_disparity", max_disparity_);
    this->declare_parameter("min_depth_mm", min_depth_mm_);
    this->declare_parameter("max_depth_mm", max_depth_mm_);
    this->declare_parameter("max_y_diff", max_y_diff_);

    // 手动相机参数
    this->declare_parameter("use_manual_camera_params", use_manual_camera_params_);
    this->declare_parameter("manual_fx", manual_fx_);
    this->declare_parameter("manual_fy", manual_fy_);
    this->declare_parameter("manual_cx", manual_cx_);
    this->declare_parameter("manual_cy", manual_cy_);
    this->declare_parameter("manual_baseline_mm", manual_baseline_mm_);

    // ========== 获取参数 ==========
    left_eye_topic_ = this->get_parameter("left_eye_topic").as_string();
    right_eye_topic_ = this->get_parameter("right_eye_topic").as_string();
    left_camera_info_topic_ = this->get_parameter("left_camera_info_topic").as_string();
    right_camera_info_topic_ = this->get_parameter("right_camera_info_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    min_disparity_ = this->get_parameter("min_disparity").as_double();
    max_disparity_ = this->get_parameter("max_disparity").as_double();
    min_depth_mm_ = this->get_parameter("min_depth_mm").as_double();
    max_depth_mm_ = this->get_parameter("max_depth_mm").as_double();
    max_y_diff_ = this->get_parameter("max_y_diff").as_double();

    // 获取手动相机参数
    use_manual_camera_params_ = this->get_parameter("use_manual_camera_params").as_bool();
    manual_fx_ = this->get_parameter("manual_fx").as_double();
    manual_fy_ = this->get_parameter("manual_fy").as_double();
    manual_cx_ = this->get_parameter("manual_cx").as_double();
    manual_cy_ = this->get_parameter("manual_cy").as_double();
    manual_baseline_mm_ = this->get_parameter("manual_baseline_mm").as_double();

    RCLCPP_INFO(this->get_logger(), "Parameters:");
    RCLCPP_INFO(this->get_logger(), "  left_eye_topic: %s", left_eye_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  right_eye_topic: %s", right_eye_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  output_topic: %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  disparity range: [%.1f, %.1f] pixels",
                min_disparity_, max_disparity_);
    RCLCPP_INFO(this->get_logger(), "  depth range: [%.1f, %.1f] mm",
                min_depth_mm_, max_depth_mm_);

    // ========== 初始化相机参数 ==========
    if (use_manual_camera_params_) {
        // 使用手动配置的相机参数
        left_camera_params_.fx = manual_fx_;
        left_camera_params_.fy = manual_fy_;
        left_camera_params_.cx = manual_cx_;
        left_camera_params_.cy = manual_cy_;
        left_camera_params_.valid = true;

        right_camera_params_.fx = manual_fx_;
        right_camera_params_.fy = manual_fy_;
        right_camera_params_.cx = manual_cx_;
        right_camera_params_.cy = manual_cy_;
        right_camera_params_.valid = true;

        baseline_mm_ = manual_baseline_mm_;

        RCLCPP_WARN(this->get_logger(), "Using manual camera params:");
        RCLCPP_WARN(this->get_logger(), "  fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                    manual_fx_, manual_fy_, manual_cx_, manual_cy_);
        RCLCPP_WARN(this->get_logger(), "  baseline=%.2f mm", baseline_mm_);
    } else {
        // 创建CameraInfo订阅
        left_camera_info_sub_ = this->create_subscription<CameraInfoMsg>(
            left_camera_info_topic_, 10,
            std::bind(&EyePosition3DNode::LeftCameraInfoCallback, this, std::placeholders::_1));

        right_camera_info_sub_ = this->create_subscription<CameraInfoMsg>(
            right_camera_info_topic_, 10,
            std::bind(&EyePosition3DNode::RightCameraInfoCallback, this, std::placeholders::_1));
    }

    // ========== 创建message_filters订阅和同步器 ==========
    left_eye_sub_ = std::make_shared<message_filters::Subscriber<EyePositionsMsg>>(
        this, left_eye_topic_, rmw_qos_profile_default);
    right_eye_sub_ = std::make_shared<message_filters::Subscriber<EyePositionsMsg>>(
        this, right_eye_topic_, rmw_qos_profile_default);

    sync_ = std::make_shared<Synchronizer>(SyncPolicy(10), *left_eye_sub_, *right_eye_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.1));  // 100ms
    sync_->registerCallback(std::bind(&EyePosition3DNode::SyncCallback, this,
                                      std::placeholders::_1, std::placeholders::_2));

    // ========== 创建发布者 ==========
    publisher_ = this->create_publisher<EyePositions3DMsg>(output_topic_, 10);

    // ========== 初始化统计 ==========
    stat_start_time_ = std::chrono::steady_clock::now();

    RCLCPP_WARN(this->get_logger(), "Eye Position 3D node initialized, waiting for camera info...");
}

EyePosition3DNode::~EyePosition3DNode()
{
    is_active_.store(false);
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (sync_) {
        sync_.reset();
    }
    if (left_eye_sub_) {
        left_eye_sub_.reset();
    }
    if (right_eye_sub_) {
        right_eye_sub_.reset();
    }
}

void EyePosition3DNode::LeftCameraInfoCallback(const CameraInfoMsg::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> lock(camera_params_mutex_);
    if (left_camera_params_.valid) {
        return;  // 已获取，不再更新
    }

    // 从K矩阵提取内参: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    left_camera_params_.fx = msg->k[0];
    left_camera_params_.fy = msg->k[4];
    left_camera_params_.cx = msg->k[2];
    left_camera_params_.cy = msg->k[5];
    left_camera_params_.valid = true;

    RCLCPP_INFO(this->get_logger(), "Left camera params: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                left_camera_params_.fx, left_camera_params_.fy,
                left_camera_params_.cx, left_camera_params_.cy);
}

void EyePosition3DNode::RightCameraInfoCallback(const CameraInfoMsg::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> lock(camera_params_mutex_);
    if (right_camera_params_.valid && baseline_mm_ > 0) {
        return;  // 已获取，不再更新
    }

    // 从K矩阵提取内参
    right_camera_params_.fx = msg->k[0];
    right_camera_params_.fy = msg->k[4];
    right_camera_params_.cx = msg->k[2];
    right_camera_params_.cy = msg->k[5];
    right_camera_params_.valid = true;

    // 从P矩阵提取基线: P = [fx, 0, cx, Tx, 0, fy, cy, 0, 0, 0, 1, 0]
    // Tx = -fx * baseline, 所以 baseline = -Tx / fx
    if (msg->p[0] != 0 && msg->p[3] != 0) {
        baseline_mm_ = -msg->p[3] / msg->p[0] * 1000.0;  // 转换为mm
    }

    RCLCPP_INFO(this->get_logger(), "Right camera params: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                right_camera_params_.fx, right_camera_params_.fy,
                right_camera_params_.cx, right_camera_params_.cy);
    RCLCPP_INFO(this->get_logger(), "Baseline: %.2f mm", baseline_mm_);
}

void EyePosition3DNode::SyncCallback(
    const EyePositionsMsg::ConstSharedPtr left_msg,
    const EyePositionsMsg::ConstSharedPtr right_msg)
{
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (!is_active_.load()) {
        return;
    }

    // 检查相机参数是否就绪
    {
        std::lock_guard<std::mutex> params_lock(camera_params_mutex_);
        if (!left_camera_params_.valid || !right_camera_params_.valid || baseline_mm_ <= 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Camera params not ready yet");
            return;
        }
    }

    // 构建右眼数据的track_id索引 (O(1)查找)
    std::unordered_map<uint64_t, size_t> right_index_map;
    for (size_t i = 0; i < right_msg->track_ids.size(); ++i) {
        right_index_map[right_msg->track_ids[i]] = i;
    }

    // 准备输出消息
    EyePositions3DMsg output_msg;
    output_msg.header = left_msg->header;
    output_msg.face_count = 0;

    // 遍历左眼数据，匹配右眼
    for (size_t i = 0; i < left_msg->track_ids.size(); ++i) {
        uint64_t track_id = left_msg->track_ids[i];

        // 查找匹配的右眼数据
        auto it = right_index_map.find(track_id);
        if (it == right_index_map.end()) {
            continue;  // 未找到匹配
        }
        size_t right_idx = it->second;

        // 检查有效性标志
        if (!left_msg->valid_flags[i] || !right_msg->valid_flags[right_idx]) {
            continue;
        }

        // 获取2D坐标
        const auto& left_eye_left = left_msg->left_eyes[i];
        const auto& left_eye_right = right_msg->left_eyes[right_idx];
        const auto& right_eye_left = left_msg->right_eyes[i];
        const auto& right_eye_right = right_msg->right_eyes[right_idx];

        // 计算左眼3D坐标
        Eye3DResult left_eye_3d = Calculate3DCoordinate(
            left_eye_left.x, left_eye_left.y,
            left_eye_right.x, left_eye_right.y);

        // 计算右眼3D坐标
        Eye3DResult right_eye_3d = Calculate3DCoordinate(
            right_eye_left.x, right_eye_left.y,
            right_eye_right.x, right_eye_right.y);

        // 判断整体有效性
        bool valid = left_eye_3d.valid && right_eye_3d.valid;
        float confidence = (left_eye_3d.confidence + right_eye_3d.confidence) / 2.0f;

        // 添加到输出
        output_msg.track_ids.push_back(track_id);
        output_msg.left_eyes_3d.push_back(left_eye_3d.position);
        output_msg.right_eyes_3d.push_back(right_eye_3d.position);
        output_msg.left_eye_disparities.push_back(left_eye_3d.disparity);
        output_msg.right_eye_disparities.push_back(right_eye_3d.disparity);
        output_msg.valid_flags.push_back(valid);
        output_msg.confidence.push_back(confidence);
        output_msg.face_count++;

        if (valid) {
            stat_valid_count_++;
        }
    }

    // 发布消息
    publisher_->publish(output_msg);

    // 更新统计
    stat_msg_count_++;
    stat_face_count_ += output_msg.face_count;
    PrintStatistics();
}

Eye3DResult EyePosition3DNode::Calculate3DCoordinate(
    float x_left, float y_left,
    float x_right, float y_right)
{
    Eye3DResult result;
    result.valid = false;
    result.confidence = 0.0f;

    std::lock_guard<std::mutex> lock(camera_params_mutex_);

    // 计算视差
    float disparity = x_left - x_right;
    result.disparity = disparity;

    // 检查视差范围
    if (disparity < min_disparity_ || disparity > max_disparity_) {
        return result;
    }

    // 检查Y坐标差异
    float y_diff = std::abs(y_left - y_right);
    if (y_diff > max_y_diff_) {
        return result;
    }

    // 计算深度 Z = fx * baseline / disparity
    double Z = left_camera_params_.fx * baseline_mm_ / disparity;

    // 检查深度范围
    if (Z < min_depth_mm_ || Z > max_depth_mm_) {
        return result;
    }

    // 计算X, Y坐标 (以左相机为参考系)
    double X = (x_left - left_camera_params_.cx) * Z / left_camera_params_.fx;
    double Y = (y_left - left_camera_params_.cy) * Z / left_camera_params_.fy;

    // 填充结果
    result.position.x = X;
    result.position.y = Y;
    result.position.z = Z;
    result.valid = true;

    // 计算置信度 (基于视差和Y差异)
    float disparity_score = 1.0f - (disparity - min_disparity_) / (max_disparity_ - min_disparity_);
    float y_diff_score = 1.0f - y_diff / max_y_diff_;
    result.confidence = (disparity_score + y_diff_score) / 2.0f;

    return result;
}

void EyePosition3DNode::PrintStatistics()
{
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        now - stat_start_time_).count();

    if (elapsed >= STAT_INTERVAL_SEC) {
        uint64_t msg_count = stat_msg_count_.load();
        uint64_t face_count = stat_face_count_.load();
        uint64_t valid_count = stat_valid_count_.load();

        double fps = static_cast<double>(msg_count) / elapsed;
        double valid_rate = face_count > 0 ?
            static_cast<double>(valid_count) / face_count * 100.0 : 0.0;

        RCLCPP_INFO(this->get_logger(),
                    "Stats: %.1f fps, %lu msgs, %lu faces, %lu valid (%.1f%%)",
                    fps, msg_count, face_count, valid_count, valid_rate);

        // 重置统计
        stat_msg_count_ = 0;
        stat_face_count_ = 0;
        stat_valid_count_ = 0;
        stat_start_time_ = now;
    }
}

// Copyright (c) 2024，D-Robotics.
// 自适应ROI版：移除人脸检测依赖，使用自适应ROI追踪

#include "face_landmarks_det_node.h"
#include <algorithm>

// ==================== 工具函数 ====================

builtin_interfaces::msg::Time ConvertToRosTime(const struct timespec &time_spec)
{
    builtin_interfaces::msg::Time stamp;
    stamp.set__sec(time_spec.tv_sec);
    stamp.set__nanosec(time_spec.tv_nsec);
    return stamp;
}

int CalTimeMsDuration(const builtin_interfaces::msg::Time &start, const builtin_interfaces::msg::Time &end)
{
    return (end.sec - start.sec) * 1000 + end.nanosec / 1000 / 1000 - start.nanosec / 1000 / 1000;
}

// ==================== 构造函数 ====================

FaceLandmarksDetNode::FaceLandmarksDetNode(const std::string &node_name, const NodeOptions &options)
    : DnnNode(node_name, options)
{
    // 声明并获取参数
    this->declare_parameter<int>("is_sync_mode", is_sync_mode_);
    this->declare_parameter<std::string>("landmarks_model_file_name", landmarks_model_file_name_);
    this->declare_parameter<std::string>("left_img_topic", left_img_topic_);
    this->declare_parameter<std::string>("left_ai_pub_topic", left_ai_pub_topic_);
    this->declare_parameter<std::string>("right_img_topic", right_img_topic_);
    this->declare_parameter<std::string>("right_ai_pub_topic", right_ai_pub_topic_);
    this->declare_parameter<double>("expand_scale", expand_scale_);
    this->declare_parameter<int>("roi_size_min", roi_size_min_);
    this->declare_parameter<int>("roi_size_max", roi_size_max_);
    this->declare_parameter<int>("image_width", image_width_);
    this->declare_parameter<int>("image_height", image_height_);
    this->declare_parameter<int>("max_lost_frames", max_lost_frames_);

    this->get_parameter<int>("is_sync_mode", is_sync_mode_);
    this->get_parameter<std::string>("landmarks_model_file_name", landmarks_model_file_name_);
    this->get_parameter<std::string>("left_img_topic", left_img_topic_);
    this->get_parameter<std::string>("left_ai_pub_topic", left_ai_pub_topic_);
    this->get_parameter<std::string>("right_img_topic", right_img_topic_);
    this->get_parameter<std::string>("right_ai_pub_topic", right_ai_pub_topic_);
    this->get_parameter<float>("expand_scale", expand_scale_);
    this->get_parameter<int>("roi_size_min", roi_size_min_);
    this->get_parameter<int>("roi_size_max", roi_size_max_);
    this->get_parameter<int>("image_width", image_width_);
    this->get_parameter<int>("image_height", image_height_);
    this->get_parameter<int>("max_lost_frames", max_lost_frames_);

    // 打印配置信息
    RCLCPP_WARN(this->get_logger(),
        "\n========== 人脸关键点检测 (自适应ROI) ==========\n"
        " landmarks_model_file_name: %s\n"
        " is_sync_mode: %d (%s)\n"
        " expand_scale: %.2f\n"
        " roi_size: [%d, %d]\n"
        " image_size: %dx%d\n"
        " max_lost_frames: %d\n"
        " 左IR: %s -> %s\n"
        " 右IR: %s -> %s\n"
        "================================================",
        landmarks_model_file_name_.c_str(),
        is_sync_mode_, is_sync_mode_ == 0 ? "异步" : "同步",
        expand_scale_,
        roi_size_min_, roi_size_max_,
        image_width_, image_height_,
        max_lost_frames_,
        left_img_topic_.c_str(), left_ai_pub_topic_.c_str(),
        right_img_topic_.c_str(), right_ai_pub_topic_.c_str());

    // 初始化模型
    if (Init() != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Init failed!");
        rclcpp::shutdown();
        return;
    }

    // 获取模型输入尺寸
    if (GetModelInputSize(0, model_input_width_, model_input_height_) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Get model input size fail!");
        rclcpp::shutdown();
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Model input: %dx%d", model_input_width_, model_input_height_);

    // 创建双路发布者
    left_ai_pub_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(
        left_ai_pub_topic_, 10);
    right_ai_pub_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(
        right_ai_pub_topic_, 10);

    // 创建双路SharedMem图像订阅
    left_img_sub_ = this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
        left_img_topic_, rclcpp::SensorDataQoS(),
        std::bind(&FaceLandmarksDetNode::LeftImgCallback, this, std::placeholders::_1));
    right_img_sub_ = this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
        right_img_topic_, rclcpp::SensorDataQoS(),
        std::bind(&FaceLandmarksDetNode::RightImgCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Node initialized successfully (Adaptive ROI mode)");
}

FaceLandmarksDetNode::~FaceLandmarksDetNode()
{
    if (predict_task_ && predict_task_->joinable())
    {
        predict_task_->join();
        predict_task_.reset();
    }
}

// ==================== DnnNode接口实现 ====================

int FaceLandmarksDetNode::SetNodePara()
{
    if (!dnn_node_para_ptr_) return -1;
    dnn_node_para_ptr_->model_file = landmarks_model_file_name_;
    dnn_node_para_ptr_->model_name = model_name_;
    dnn_node_para_ptr_->model_task_type = model_task_type_;
    dnn_node_para_ptr_->task_num = 4;
    return 0;
}

int FaceLandmarksDetNode::Predict(
    std::vector<std::shared_ptr<DNNInput>> &inputs,
    const std::shared_ptr<std::vector<hbDNNRoi>> rois,
    std::shared_ptr<DnnNodeOutput> dnn_output)
{
    return Run(inputs, dnn_output, rois, is_sync_mode_ == 1);
}

// ==================== ROI管理函数 ====================

hbDNNRoi FaceLandmarksDetNode::GetDefaultROI()
{
    // 返回画面中间1/3区域
    hbDNNRoi roi;
    roi.left = image_width_ / 3;
    roi.top = image_height_ / 3;
    roi.right = image_width_ * 2 / 3;
    roi.bottom = image_height_ * 2 / 3;

    // ROI边界对齐：left/top必须为偶数，right/bottom必须为奇数
    roi.left += (roi.left % 2 == 0 ? 0 : 1);
    roi.top += (roi.top % 2 == 0 ? 0 : 1);
    roi.right -= (roi.right % 2 == 1 ? 0 : 1);
    roi.bottom -= (roi.bottom % 2 == 1 ? 0 : 1);

    return roi;
}

std::vector<hbDNNRoi> FaceLandmarksDetNode::GetCurrentROIs(
    std::vector<AdaptiveRoi>& rois, std::mutex& roi_mtx)
{
    std::lock_guard<std::mutex> lock(roi_mtx);

    std::vector<hbDNNRoi> result;

    if (rois.empty())
    {
        // 没有追踪目标，使用默认ROI
        result.push_back(GetDefaultROI());
    }
    else
    {
        // 使用已有的追踪ROI
        for (const auto& adaptive_roi : rois)
        {
            result.push_back(adaptive_roi.roi);
        }
    }

    return result;
}

hbDNNRoi FaceLandmarksDetNode::ComputeRoiFromLandmarks(
    const std::vector<geometry_msgs::msg::Point32>& landmarks)
{
    if (landmarks.empty())
    {
        return GetDefaultROI();
    }

    // 计算关键点的边界框
    float min_x = landmarks[0].x;
    float max_x = landmarks[0].x;
    float min_y = landmarks[0].y;
    float max_y = landmarks[0].y;

    for (const auto& pt : landmarks)
    {
        min_x = std::min(min_x, pt.x);
        max_x = std::max(max_x, pt.x);
        min_y = std::min(min_y, pt.y);
        max_y = std::max(max_y, pt.y);
    }

    // 计算中心和尺寸
    float center_x = (min_x + max_x) / 2.0f;
    float center_y = (min_y + max_y) / 2.0f;
    float box_w = max_x - min_x;
    float box_h = max_y - min_y;

    // 扩展ROI
    float w_new = box_w * expand_scale_;
    float h_new = box_h * expand_scale_;

    hbDNNRoi roi;
    roi.left = static_cast<int32_t>(center_x - w_new / 2);
    roi.top = static_cast<int32_t>(center_y - h_new / 2);
    roi.right = static_cast<int32_t>(center_x + w_new / 2);
    roi.bottom = static_cast<int32_t>(center_y + h_new / 2);

    // 裁剪到图像边界
    roi.left = std::max(0, roi.left);
    roi.top = std::max(0, roi.top);
    roi.right = std::min(image_width_, roi.right);
    roi.bottom = std::min(image_height_, roi.bottom);

    // ROI边界对齐
    roi.left += (roi.left % 2 == 0 ? 0 : 1);
    roi.top += (roi.top % 2 == 0 ? 0 : 1);
    roi.right -= (roi.right % 2 == 1 ? 0 : 1);
    roi.bottom -= (roi.bottom % 2 == 1 ? 0 : 1);

    return roi;
}

void FaceLandmarksDetNode::UpdateROIs(
    std::vector<AdaptiveRoi>& rois, std::mutex& roi_mtx,
    const ai_msgs::msg::PerceptionTargets& results)
{
    std::lock_guard<std::mutex> lock(roi_mtx);

    // 收集本次检测到的人脸
    std::vector<AdaptiveRoi> new_rois;

    for (const auto& target : results.targets)
    {
        for (const auto& point : target.points)
        {
            if (point.type == "face_kps" && !point.point.empty())
            {
                AdaptiveRoi new_roi;
                new_roi.roi = ComputeRoiFromLandmarks(point.point);
                new_roi.track_id = next_track_id_++;
                new_roi.lost_count = 0;
                new_rois.push_back(new_roi);
            }
        }
    }

    if (new_rois.empty())
    {
        // 没有检测到人脸，增加所有现有ROI的丢失计数
        for (auto& roi : rois)
        {
            roi.lost_count++;
        }

        // 移除丢失次数过多的ROI
        rois.erase(
            std::remove_if(rois.begin(), rois.end(),
                [this](const AdaptiveRoi& r) { return r.lost_count > max_lost_frames_; }),
            rois.end());
    }
    else
    {
        // 用新检测到的ROI替换旧的
        rois = std::move(new_rois);
    }
}

// ==================== 双路图像回调 ====================

void FaceLandmarksDetNode::LeftImgCallback(const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg)
{
    ProcessImage(msg, left_rois_, left_roi_mtx_, left_ai_pub_, 0);
}

void FaceLandmarksDetNode::RightImgCallback(const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg)
{
    ProcessImage(msg, right_rois_, right_roi_mtx_, right_ai_pub_, 1);
}

void FaceLandmarksDetNode::ProcessImage(
    const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr img_msg,
    std::vector<AdaptiveRoi>& rois,
    std::mutex& roi_mtx,
    rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr publisher,
    int channel_id)
{
    if (!img_msg || !rclcpp::ok()) return;

    stat_img_count_++;

    // 记录回调入口时间
    struct timespec time_start = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_start);

    // 计算传输延迟（图像时间戳 -> 回调入口）
    int64_t img_ts_ns = static_cast<int64_t>(img_msg->time_stamp.sec) * 1000000000LL +
                        static_cast<int64_t>(img_msg->time_stamp.nanosec);
    int64_t callback_enter_ns = static_cast<int64_t>(time_start.tv_sec) * 1000000000LL +
                                static_cast<int64_t>(time_start.tv_nsec);
    double transport_delay_ms = static_cast<double>(callback_enter_ns - img_ts_ns) / 1000000.0;

    // 检查图像格式
    std::string encoding(reinterpret_cast<const char *>(img_msg->encoding.data()));
    if (encoding != "nv12")
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "[通道%d] 不支持的格式: %s, 期望nv12", channel_id, encoding.c_str());
        return;
    }

    // 更新图像尺寸
    image_width_ = img_msg->width;
    image_height_ = img_msg->height;

    // 记录金字塔构建开始时间
    struct timespec time_pyramid_start = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_pyramid_start);

    // 构建金字塔输入
    auto pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char *>(img_msg->data.data()),
        img_msg->height, img_msg->width,
        img_msg->height, img_msg->width);

    struct timespec time_pyramid_end = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_pyramid_end);
    double pyramid_ms = (time_pyramid_end.tv_sec - time_pyramid_start.tv_sec) * 1000.0 +
                        (time_pyramid_end.tv_nsec - time_pyramid_start.tv_nsec) / 1000000.0;

    if (!pyramid)
    {
        RCLCPP_ERROR(this->get_logger(), "[通道%d] 获取NV12金字塔失败!", channel_id);
        return;
    }

    // 记录获取ROI开始时间
    struct timespec time_roi_start = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_roi_start);

    // 获取当前ROI
    auto current_rois = GetCurrentROIs(rois, roi_mtx);
    auto rois_ptr = std::make_shared<std::vector<hbDNNRoi>>(current_rois);

    struct timespec time_roi_end = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_roi_end);
    double roi_ms = (time_roi_end.tv_sec - time_roi_start.tv_sec) * 1000.0 +
                    (time_roi_end.tv_nsec - time_roi_start.tv_nsec) / 1000000.0;

    // 创建推理输出
    auto dnn_output = std::make_shared<FaceLandmarksDetOutput>();
    dnn_output->image_msg_header = std::make_shared<std_msgs::msg::Header>();
    dnn_output->image_msg_header->set__frame_id(std::to_string(img_msg->index));
    dnn_output->image_msg_header->set__stamp(img_msg->time_stamp);
    dnn_output->perf_preprocess.stamp_start.sec = time_start.tv_sec;
    dnn_output->perf_preprocess.stamp_start.nanosec = time_start.tv_nsec;
    dnn_output->perf_preprocess.set__type(model_name_ + "_preprocess");
    dnn_output->channel_id = channel_id;
    dnn_output->valid_rois = rois_ptr;

    // 构建valid_roi_idx映射
    for (size_t i = 0; i < rois_ptr->size(); i++)
    {
        dnn_output->valid_roi_idx[i] = i;
    }

    // 记录构建推理输入开始时间
    struct timespec time_input_start = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_input_start);

    // 构建推理输入
    auto model_manage = GetModel();
    if (!model_manage) return;

    std::vector<std::shared_ptr<DNNInput>> inputs;
    inputs.reserve(rois_ptr->size() * model_manage->GetInputCount());
    for (size_t i = 0; i < rois_ptr->size(); i++)
    {
        for (int32_t j = 0; j < model_manage->GetInputCount(); j++)
        {
            inputs.push_back(pyramid);
        }
    }

    struct timespec time_input_end = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_input_end);
    double input_ms = (time_input_end.tv_sec - time_input_start.tv_sec) * 1000.0 +
                      (time_input_end.tv_nsec - time_input_start.tv_nsec) / 1000000.0;

    // 记录预处理结束时间
    struct timespec time_now = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_now);
    dnn_output->perf_preprocess.stamp_end.sec = time_now.tv_sec;
    dnn_output->perf_preprocess.stamp_end.nanosec = time_now.tv_nsec;

    double preprocess_total_ms = (time_now.tv_sec - time_start.tv_sec) * 1000.0 +
                                 (time_now.tv_nsec - time_start.tv_nsec) / 1000000.0;

    // 输出预处理阶段耗时日志
    RCLCPP_INFO(this->get_logger(),
        "[通道%d] 预处理: 传输=%.2fms, 金字塔=%.2fms, 获取ROI=%.2fms, 构建输入=%.2fms, 总计=%.2fms",
        channel_id, transport_delay_ms, pyramid_ms, roi_ms, input_ms, preprocess_total_ms);

    // 记录提交推理时间
    struct timespec time_predict_start = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_predict_start);

    // 执行推理
    int predict_ret = Predict(inputs, rois_ptr, dnn_output);

    struct timespec time_predict_end = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_predict_end);
    double predict_submit_ms = (time_predict_end.tv_sec - time_predict_start.tv_sec) * 1000.0 +
                               (time_predict_end.tv_nsec - time_predict_start.tv_nsec) / 1000000.0;

    RCLCPP_INFO(this->get_logger(),
        "[通道%d] 提交推理: %.2fms, 返回值=%d",
        channel_id, predict_submit_ms, predict_ret);

    if (predict_ret == 0)
    {
        stat_infer_count_++;
    }
}

// ==================== ROI归一化处理 ====================

int FaceLandmarksDetNode::NormalizeRoi(
    const hbDNNRoi *src, hbDNNRoi *dst,
    float norm_ratio, uint32_t total_w, uint32_t total_h)
{
    *dst = *src;
    float box_w = dst->right - dst->left;
    float box_h = dst->bottom - dst->top;
    float center_x = (dst->left + dst->right) / 2.0f;
    float center_y = (dst->top + dst->bottom) / 2.0f;

    // 按比例扩展ROI
    float w_new = box_w * norm_ratio;
    float h_new = box_h * norm_ratio;
    dst->left = center_x - w_new / 2;
    dst->right = center_x + w_new / 2;
    dst->top = center_y - h_new / 2;
    dst->bottom = center_y + h_new / 2;

    // 裁剪到图像边界
    dst->left = std::max(0.0f, static_cast<float>(dst->left));
    dst->top = std::max(0.0f, static_cast<float>(dst->top));
    dst->right = std::min(static_cast<float>(total_w), static_cast<float>(dst->right));
    dst->bottom = std::min(static_cast<float>(total_h), static_cast<float>(dst->bottom));

    // ROI边界对齐
    dst->left += (dst->left % 2 == 0 ? 0 : 1);
    dst->top += (dst->top % 2 == 0 ? 0 : 1);
    dst->right -= (dst->right % 2 == 1 ? 0 : 1);
    dst->bottom -= (dst->bottom % 2 == 1 ? 0 : 1);

    // 尺寸过滤
    int32_t roi_w = dst->right - dst->left;
    int32_t roi_h = dst->bottom - dst->top;
    int32_t max_size = std::max(roi_w, roi_h);
    int32_t min_size = std::min(roi_w, roi_h);

    if (max_size < roi_size_max_ && min_size > roi_size_min_)
    {
        return 0;
    }
    return -1;
}

// ==================== 后处理函数 ====================

int FaceLandmarksDetNode::PostProcess(const std::shared_ptr<DnnNodeOutput> &node_output)
{
    if (!rclcpp::ok()) return -1;

    auto output = std::dynamic_pointer_cast<FaceLandmarksDetOutput>(node_output);
    if (!output) return -1;

    // 记录后处理开始时间
    struct timespec time_post_start = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_post_start);

    // 根据channel_id选择对应的publisher和rois
    auto& publisher = (output->channel_id == 0) ? left_ai_pub_ : right_ai_pub_;
    auto& rois = (output->channel_id == 0) ? left_rois_ : right_rois_;
    auto& roi_mtx = (output->channel_id == 0) ? left_roi_mtx_ : right_roi_mtx_;

    if (!publisher) return -1;

    struct timespec time_now = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_now);

    // 记录解析关键点开始时间
    struct timespec time_parse_start = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_parse_start);

    // 解析关键点
    auto parser = std::make_shared<FaceLandmarksDetOutputParser>(this->get_logger());
    auto landmarks_result = std::make_shared<FaceLandmarksDetResult>();
    if (output->valid_rois)
    {
        parser->Parse(landmarks_result, output->output_tensors, output->valid_rois);
    }

    struct timespec time_parse_end = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_parse_end);
    double parse_ms = (time_parse_end.tv_sec - time_parse_start.tv_sec) * 1000.0 +
                      (time_parse_end.tv_nsec - time_parse_start.tv_nsec) / 1000000.0;

    // 构建输出消息
    ai_msgs::msg::PerceptionTargets::UniquePtr ai_msg(new ai_msgs::msg::PerceptionTargets());
    ai_msg->header.set__stamp(output->image_msg_header->stamp);
    ai_msg->header.set__frame_id(output->image_msg_header->frame_id);

    if (node_output->rt_stat)
    {
        ai_msg->set__fps(round(node_output->rt_stat->output_fps));
    }

    // 记录构建消息开始时间
    struct timespec time_msg_start = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_msg_start);

    // 处理每个ROI的关键点结果
    for (size_t i = 0; i < landmarks_result->values.size(); i++)
    {
        const auto& landmarks = landmarks_result->values[i];
        if (landmarks.empty()) continue;

        ai_msgs::msg::Target target;
        target.set__type("face");
        target.set__track_id(i);

        // 添加ROI
        if (i < output->valid_rois->size())
        {
            ai_msgs::msg::Roi roi;
            roi.type = "face";
            roi.rect.set__x_offset((*output->valid_rois)[i].left);
            roi.rect.set__y_offset((*output->valid_rois)[i].top);
            roi.rect.set__width((*output->valid_rois)[i].right - (*output->valid_rois)[i].left);
            roi.rect.set__height((*output->valid_rois)[i].bottom - (*output->valid_rois)[i].top);
            target.rois.push_back(roi);
        }

        // 添加关键点
        ai_msgs::msg::Point face_landmarks;
        face_landmarks.set__type("face_kps");
        for (const auto &pt : landmarks)
        {
            geometry_msgs::msg::Point32 p;
            p.set__x(pt.x);
            p.set__y(pt.y);
            face_landmarks.point.emplace_back(p);
        }
        target.points.push_back(face_landmarks);

        ai_msg->targets.emplace_back(target);
        stat_detect_count_++;
    }

    struct timespec time_msg_end = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_msg_end);
    double msg_build_ms = (time_msg_end.tv_sec - time_msg_start.tv_sec) * 1000.0 +
                          (time_msg_end.tv_nsec - time_msg_start.tv_nsec) / 1000000.0;

    // 记录更新ROI开始时间
    struct timespec time_update_roi_start = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_update_roi_start);

    // 更新ROI缓存
    UpdateROIs(rois, roi_mtx, *ai_msg);

    struct timespec time_update_roi_end = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_update_roi_end);
    double update_roi_ms = (time_update_roi_end.tv_sec - time_update_roi_start.tv_sec) * 1000.0 +
                           (time_update_roi_end.tv_nsec - time_update_roi_start.tv_nsec) / 1000000.0;

    // 添加性能信息
    output->perf_preprocess.set__time_ms_duration(
        CalTimeMsDuration(output->perf_preprocess.stamp_start, output->perf_preprocess.stamp_end));
    ai_msg->perfs.push_back(output->perf_preprocess);

    if (output->rt_stat)
    {
        ai_msgs::msg::Perf perf;
        perf.set__type(model_name_ + "_predict_infer");
        perf.set__stamp_start(ConvertToRosTime(output->rt_stat->infer_timespec_start));
        perf.set__stamp_end(ConvertToRosTime(output->rt_stat->infer_timespec_end));
        perf.set__time_ms_duration(output->rt_stat->infer_time_ms);
        ai_msg->perfs.push_back(perf);
    }

    ai_msgs::msg::Perf perf_post;
    perf_post.set__type(model_name_ + "_postprocess");
    perf_post.set__stamp_start(ConvertToRosTime(time_now));
    clock_gettime(CLOCK_REALTIME, &time_now);
    perf_post.set__stamp_end(ConvertToRosTime(time_now));
    perf_post.set__time_ms_duration(CalTimeMsDuration(perf_post.stamp_start, perf_post.stamp_end));
    ai_msg->perfs.push_back(perf_post);

    // 记录发布消息开始时间
    struct timespec time_pub_start = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_pub_start);

    publisher->publish(std::move(ai_msg));

    struct timespec time_pub_end = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_pub_end);
    double publish_ms = (time_pub_end.tv_sec - time_pub_start.tv_sec) * 1000.0 +
                        (time_pub_end.tv_nsec - time_pub_start.tv_nsec) / 1000000.0;

    // 计算后处理总耗时
    double post_total_ms = (time_pub_end.tv_sec - time_post_start.tv_sec) * 1000.0 +
                           (time_pub_end.tv_nsec - time_post_start.tv_nsec) / 1000000.0;

    // 计算BPU推理耗时
    double infer_ms = 0.0;
    if (node_output->rt_stat)
    {
        infer_ms = static_cast<double>(node_output->rt_stat->infer_time_ms);
    }

    // 输出后处理阶段耗时日志
    RCLCPP_INFO(this->get_logger(),
        "[通道%d] 后处理: 解析=%.2fms, 构建消息=%.2fms, 更新ROI=%.2fms, 发布=%.2fms, 总计=%.2fms, BPU推理=%dms",
        output->channel_id, parse_ms, msg_build_ms, update_roi_ms, publish_ms, post_total_ms,
        node_output->rt_stat ? node_output->rt_stat->infer_time_ms : 0);

    // 性能统计输出 (每5秒)
    if (node_output->rt_stat && node_output->rt_stat->fps_updated)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "[Perf] in_fps: %.1f, out_fps: %.1f, infer: %dms, detect: %lu, img: %lu",
            node_output->rt_stat->input_fps,
            node_output->rt_stat->output_fps,
            node_output->rt_stat->infer_time_ms,
            stat_detect_count_.load(),
            stat_img_count_.load());
    }

    return 0;
}

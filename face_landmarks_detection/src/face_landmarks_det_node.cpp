// Copyright (c) 2024，D-Robotics.
// 精简版：仅保留在线模式，SharedMem+NV12输入

#include "face_landmarks_det_node.h"

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
    // 双路topic参数
    this->declare_parameter<std::string>("left_img_topic", left_img_topic_);
    this->declare_parameter<std::string>("left_ai_sub_topic", left_ai_sub_topic_);
    this->declare_parameter<std::string>("left_ai_pub_topic", left_ai_pub_topic_);
    this->declare_parameter<std::string>("right_img_topic", right_img_topic_);
    this->declare_parameter<std::string>("right_ai_sub_topic", right_ai_sub_topic_);
    this->declare_parameter<std::string>("right_ai_pub_topic", right_ai_pub_topic_);
    this->declare_parameter<double>("expand_scale", expand_scale_);
    this->declare_parameter<int>("roi_size_min", roi_size_min_);
    this->declare_parameter<int>("roi_size_max", roi_size_max_);
    this->declare_parameter<int>("cache_len_limit", static_cast<int>(cache_len_limit_));
    this->declare_parameter<int>("ai_msg_timeout_ms", ai_msg_timeout_ms_);
    this->declare_parameter<double>("score_threshold", score_threshold_);

    this->get_parameter<int>("is_sync_mode", is_sync_mode_);
    this->get_parameter<std::string>("landmarks_model_file_name", landmarks_model_file_name_);
    this->get_parameter<std::string>("left_img_topic", left_img_topic_);
    this->get_parameter<std::string>("left_ai_sub_topic", left_ai_sub_topic_);
    this->get_parameter<std::string>("left_ai_pub_topic", left_ai_pub_topic_);
    this->get_parameter<std::string>("right_img_topic", right_img_topic_);
    this->get_parameter<std::string>("right_ai_sub_topic", right_ai_sub_topic_);
    this->get_parameter<std::string>("right_ai_pub_topic", right_ai_pub_topic_);
    this->get_parameter<float>("expand_scale", expand_scale_);
    this->get_parameter<int>("roi_size_min", roi_size_min_);
    this->get_parameter<int>("roi_size_max", roi_size_max_);
    int cache_len = static_cast<int>(cache_len_limit_);
    this->get_parameter<int>("cache_len_limit", cache_len);
    cache_len_limit_ = static_cast<size_t>(cache_len);
    this->get_parameter<int>("ai_msg_timeout_ms", ai_msg_timeout_ms_);
    this->get_parameter<float>("score_threshold", score_threshold_);

    // 打印配置信息
    RCLCPP_WARN(this->get_logger(),
        "\n========== 人脸关键点检测 (双路) ==========\n"
        " landmarks_model_file_name: %s\n"
        " is_sync_mode: %d (%s)\n"
        " score_threshold: %.2f\n"
        " expand_scale: %.2f\n"
        " roi_size: [%d, %d]\n"
        " cache_len_limit: %zu\n"
        " 左IR: %s -> %s -> %s\n"
        " 右IR: %s -> %s -> %s\n"
        "============================================",
        landmarks_model_file_name_.c_str(),
        is_sync_mode_, is_sync_mode_ == 0 ? "异步" : "同步",
        score_threshold_,
        expand_scale_,
        roi_size_min_, roi_size_max_,
        cache_len_limit_,
        left_img_topic_.c_str(), left_ai_sub_topic_.c_str(), left_ai_pub_topic_.c_str(),
        right_img_topic_.c_str(), right_ai_sub_topic_.c_str(), right_ai_pub_topic_.c_str());

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

    // 初始化双路组件
    left_ai_manage_ = std::make_shared<AiMsgManage>(this->get_logger());
    right_ai_manage_ = std::make_shared<AiMsgManage>(this->get_logger());
    predict_task_ = std::make_shared<std::thread>(std::bind(&FaceLandmarksDetNode::RunPredict, this));

    // 创建双路发布者
    left_ai_pub_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(
        left_ai_pub_topic_, 10);
    right_ai_pub_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(
        right_ai_pub_topic_, 10);

    // 创建双路AI消息订阅
    left_ai_sub_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
        left_ai_sub_topic_, 10,
        std::bind(&FaceLandmarksDetNode::LeftAiCallback, this, std::placeholders::_1));
    right_ai_sub_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
        right_ai_sub_topic_, 10,
        std::bind(&FaceLandmarksDetNode::RightAiCallback, this, std::placeholders::_1));

    // 创建双路SharedMem图像订阅
    left_img_sub_ = this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
        left_img_topic_, rclcpp::SensorDataQoS(),
        std::bind(&FaceLandmarksDetNode::LeftImgCallback, this, std::placeholders::_1));
    right_img_sub_ = this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
        right_img_topic_, rclcpp::SensorDataQoS(),
        std::bind(&FaceLandmarksDetNode::RightImgCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Node initialized successfully");
}

FaceLandmarksDetNode::~FaceLandmarksDetNode()
{
    // 通知双路缓存退出
    {
        std::unique_lock<std::mutex> lg(left_mtx_img_);
        left_cv_img_.notify_all();
    }
    {
        std::unique_lock<std::mutex> lg(right_mtx_img_);
        right_cv_img_.notify_all();
    }

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

// ==================== AI消息处理 ====================

void FaceLandmarksDetNode::LeftAiCallback(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg)
{
    if (!msg || !rclcpp::ok() || !left_ai_manage_) return;
    left_ai_manage_->Feed(msg);
}

void FaceLandmarksDetNode::RightAiCallback(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg)
{
    if (!msg || !rclcpp::ok() || !right_ai_manage_) return;
    right_ai_manage_->Feed(msg);
}

// ==================== 双路图像回调 ====================

void FaceLandmarksDetNode::LeftImgCallback(const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg)
{
    ProcessImage(msg, left_ai_manage_, left_ai_pub_,
                 left_cache_img_, left_mtx_img_, left_cv_img_, 0);
}

void FaceLandmarksDetNode::RightImgCallback(const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg)
{
    ProcessImage(msg, right_ai_manage_, right_ai_pub_,
                 right_cache_img_, right_mtx_img_, right_cv_img_, 1);
}

void FaceLandmarksDetNode::ProcessImage(
    const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr img_msg,
    std::shared_ptr<AiMsgManage> ai_manage,
    rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr publisher,
    std::queue<CacheImgType>& cache_img,
    std::mutex& mtx_img,
    std::condition_variable& cv_img,
    int channel_id)
{
    (void)ai_manage;
    if (!img_msg || !rclcpp::ok()) return;

    stat_img_count_++;
    struct timespec time_start = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_start);

    // 检查图像格式
    std::string encoding(reinterpret_cast<const char *>(img_msg->encoding.data()));
    if (encoding != "nv12")
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "[通道%d] 不支持的格式: %s, 期望nv12", channel_id, encoding.c_str());
        return;
    }

    // 构建金字塔输入
    auto pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char *>(img_msg->data.data()),
        img_msg->height, img_msg->width,
        img_msg->height, img_msg->width);
    if (!pyramid)
    {
        RCLCPP_ERROR(this->get_logger(), "[通道%d] 获取NV12金字塔失败!", channel_id);
        return;
    }

    // 创建推理输出
    auto dnn_output = std::make_shared<FaceLandmarksDetOutput>();
    dnn_output->image_msg_header = std::make_shared<std_msgs::msg::Header>();
    dnn_output->image_msg_header->set__frame_id(std::to_string(img_msg->index));
    dnn_output->image_msg_header->set__stamp(img_msg->time_stamp);
    dnn_output->perf_preprocess.stamp_start.sec = time_start.tv_sec;
    dnn_output->perf_preprocess.stamp_start.nanosec = time_start.tv_nsec;
    dnn_output->perf_preprocess.set__type(model_name_ + "_preprocess");
    dnn_output->channel_id = channel_id;

    // 加入缓存队列
    std::unique_lock<std::mutex> lg(mtx_img);
    if (cache_img.size() >= cache_len_limit_)
    {
        auto drop = cache_img.front();
        cache_img.pop();
        stat_drop_count_++;
        if (drop.first->ai_msg && publisher)
        {
            publisher->publish(std::move(drop.first->ai_msg));
        }
    }
    cache_img.push(std::make_pair(std::move(dnn_output), std::move(pyramid)));
    cv_img.notify_one();
}

// ==================== 推理线程 ====================

void FaceLandmarksDetNode::RunPredict()
{
    RCLCPP_INFO(this->get_logger(), "推理线程启动");
    while (rclcpp::ok())
    {
        CacheImgType img_data;
        int channel_id = -1;
        std::shared_ptr<AiMsgManage> ai_manage = nullptr;
        rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr publisher = nullptr;

        // 尝试从左路缓存获取
        {
            std::unique_lock<std::mutex> lg(left_mtx_img_);
            if (!left_cache_img_.empty())
            {
                img_data = std::move(left_cache_img_.front());
                left_cache_img_.pop();
                channel_id = 0;
                ai_manage = left_ai_manage_;
                publisher = left_ai_pub_;
            }
        }

        // 如果左路为空，尝试从右路缓存获取
        if (channel_id < 0)
        {
            std::unique_lock<std::mutex> lg(right_mtx_img_);
            if (!right_cache_img_.empty())
            {
                img_data = std::move(right_cache_img_.front());
                right_cache_img_.pop();
                channel_id = 1;
                ai_manage = right_ai_manage_;
                publisher = right_ai_pub_;
            }
        }

        // 如果双路都为空，等待
        if (channel_id < 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        auto dnn_output = img_data.first;
        auto pyramid = img_data.second;

        // 从AI消息获取ROI
        std::shared_ptr<std::vector<hbDNNRoi>> rois = nullptr;
        std::map<size_t, size_t> valid_roi_idx;
        ai_msgs::msg::PerceptionTargets::UniquePtr ai_msg = nullptr;

        if (ai_manage->GetTargetRois(
                dnn_output->image_msg_header->stamp, rois, valid_roi_idx, ai_msg,
                std::bind(&FaceLandmarksDetNode::NormalizeRoi, this,
                    std::placeholders::_1, std::placeholders::_2,
                    expand_scale_, pyramid->width, pyramid->height),
                ai_msg_timeout_ms_) < 0 || ai_msg == nullptr)
        {
            continue;
        }

        stat_match_count_++;

        // 检查ROI有效性
        if (!rois || rois->empty())
        {
            if (publisher) publisher->publish(std::move(ai_msg));
            continue;
        }

        dnn_output->valid_rois = rois;
        dnn_output->valid_roi_idx = valid_roi_idx;
        dnn_output->ai_msg = std::move(ai_msg);

        // 构建推理输入
        auto model_manage = GetModel();
        if (!model_manage) continue;

        std::vector<std::shared_ptr<DNNInput>> inputs;
        inputs.reserve(rois->size() * model_manage->GetInputCount());
        for (size_t i = 0; i < rois->size(); i++)
        {
            for (int32_t j = 0; j < model_manage->GetInputCount(); j++)
            {
                inputs.push_back(pyramid);
            }
        }

        // 记录预处理结束时间
        struct timespec time_now = {0, 0};
        clock_gettime(CLOCK_REALTIME, &time_now);
        dnn_output->perf_preprocess.stamp_end.sec = time_now.tv_sec;
        dnn_output->perf_preprocess.stamp_end.nanosec = time_now.tv_nsec;

        // 执行推理
        if (Predict(inputs, rois, dnn_output) == 0)
        {
            stat_infer_count_++;
        }
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

    // ROI边界对齐：left/top必须为偶数，right/bottom必须为奇数
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
        return 0;  // 有效ROI
    }
    else
    {
        stat_filter_count_++;
        return -1;  // 过滤掉
    }
}

// ==================== 后处理函数 ====================

int FaceLandmarksDetNode::PostProcess(const std::shared_ptr<DnnNodeOutput> &node_output)
{
    if (!rclcpp::ok()) return -1;

    auto output = std::dynamic_pointer_cast<FaceLandmarksDetOutput>(node_output);
    if (!output) return -1;

    // 根据channel_id选择对应的publisher
    auto& publisher = (output->channel_id == 0) ? left_ai_pub_ : right_ai_pub_;
    if (!publisher) return -1;

    // 性能统计输出 (每5秒)
    if (node_output->rt_stat && node_output->rt_stat->fps_updated)
    {
        float match_rate = stat_img_count_ > 0 ?
            (100.0f * stat_match_count_ / stat_img_count_) : 0.0f;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "[Perf] in_fps: %.1f, out_fps: %.1f, infer: %dms, "
            "match_rate: %.1f%%, drop: %lu, filter: %lu",
            node_output->rt_stat->input_fps,
            node_output->rt_stat->output_fps,
            node_output->rt_stat->infer_time_ms,
            match_rate,
            stat_drop_count_.load(),
            stat_filter_count_.load());
    }

    struct timespec time_now = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_now);

    // 解析关键点
    auto parser = std::make_shared<FaceLandmarksDetOutputParser>(this->get_logger());
    auto landmarks_result = std::make_shared<FaceLandmarksDetResult>();
    if (output->valid_rois)
    {
        parser->Parse(landmarks_result, output->output_tensors, output->valid_rois);
    }

    // 构建输出消息
    ai_msgs::msg::PerceptionTargets::UniquePtr &msg = output->ai_msg;
    if (!msg) return -1;

    // 检查数据一致性
    if (landmarks_result->values.size() != output->valid_rois->size() ||
        output->valid_rois->size() != output->valid_roi_idx.size())
    {
        publisher->publish(std::move(msg));
        return 0;
    }

    // 构建新的AI消息
    ai_msgs::msg::PerceptionTargets::UniquePtr ai_msg(new ai_msgs::msg::PerceptionTargets());
    ai_msg->set__header(msg->header);
    ai_msg->set__disappeared_targets(msg->disappeared_targets);
    if (node_output->rt_stat)
    {
        ai_msg->set__fps(round(node_output->rt_stat->output_fps));
    }

    // 处理每个target
    int face_roi_idx = 0;
    const auto &valid_roi_idx = output->valid_roi_idx;

    for (const auto &in_target : msg->targets)
    {
        ai_msgs::msg::Target target;
        target.set__type(in_target.type);
        target.set__attributes(in_target.attributes);
        target.set__captures(in_target.captures);
        target.set__track_id(in_target.track_id);

        std::vector<ai_msgs::msg::Point> landmarks_points;
        std::vector<ai_msgs::msg::Roi> rois;

        for (const auto &roi : in_target.rois)
        {
            if (roi.type != "face") continue;

            rois.push_back(roi);
            target.set__rois(rois);

            // 检查索引有效性
            if (valid_roi_idx.find(face_roi_idx) == valid_roi_idx.end())
            {
                face_roi_idx++;
                continue;
            }

            auto idx = valid_roi_idx.at(face_roi_idx);
            if (idx >= landmarks_result->values.size())
            {
                break;
            }

            // 添加关键点
            ai_msgs::msg::Point face_landmarks;
            face_landmarks.set__type("face_kps");
            for (const auto &pt : landmarks_result->values[idx])
            {
                geometry_msgs::msg::Point32 p;
                p.set__x(pt.x);
                p.set__y(pt.y);
                face_landmarks.point.emplace_back(p);
            }
            landmarks_points.push_back(face_landmarks);
            face_roi_idx++;
        }

        target.set__points(landmarks_points);
        if (!target.rois.empty())
        {
            ai_msg->targets.emplace_back(target);
        }
    }

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

    publisher->publish(std::move(ai_msg));
    return 0;
}

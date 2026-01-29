// Copyright (c) 2024，D-Robotics.
// 精简版：仅保留在线模式，SharedMem+NV12输入

#ifndef FACE_LANDMARKS_DET_NODE_H
#define FACE_LANDMARKS_DET_NODE_H

#include <atomic>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "dnn_node/dnn_node.h"
#include "dnn_node/util/image_proc.h"
#include "ai_msgs/msg/perception_targets.hpp"
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#include "face_landmarks_det_output_parser.h"
#include "ai_msg_manage.h"

using rclcpp::NodeOptions;
using hobot::dnn_node::DNNInput;
using hobot::dnn_node::DnnNode;
using hobot::dnn_node::DnnNodeOutput;
using hobot::dnn_node::DNNTensor;
using hobot::dnn_node::ModelTaskType;
using hobot::dnn_node::NV12PyramidInput;
using ai_msgs::msg::PerceptionTargets;

// 推理输出结构
struct FaceLandmarksDetOutput : public DnnNodeOutput
{
    std::shared_ptr<std_msgs::msg::Header> image_msg_header = nullptr;
    std::shared_ptr<std::vector<hbDNNRoi>> valid_rois;
    std::map<size_t, size_t> valid_roi_idx;
    ai_msgs::msg::PerceptionTargets::UniquePtr ai_msg;
    ai_msgs::msg::Perf perf_preprocess;
};

// 精简版人脸关键点检测节点
class FaceLandmarksDetNode : public DnnNode
{
public:
    explicit FaceLandmarksDetNode(
        const std::string &node_name = "face_landmarks_det_node",
        const NodeOptions &options = NodeOptions());
    ~FaceLandmarksDetNode() override;

protected:
    int SetNodePara() override;
    int PostProcess(const std::shared_ptr<DnnNodeOutput> &outputs) override;

private:
    // ========== 模型配置 ==========
    std::string model_file_name_ = "./config/faceLandmark106pts.hbm";
    std::string model_name_ = "faceLandmark106pts";
    int model_input_width_ = -1;
    int model_input_height_ = -1;
    ModelTaskType model_task_type_ = ModelTaskType::ModelRoiInferType;

    // ========== 推理配置 ==========
    int is_sync_mode_ = 0;  // 0=异步, 1=同步

    // ========== ROI处理配置 ==========
    float expand_scale_ = 1.25;      // ROI扩展比例
    int32_t roi_size_min_ = 16;      // ROI最小尺寸
    int32_t roi_size_max_ = 255;     // ROI最大尺寸
    float score_threshold_ = 0.5;    // 上游人脸检测置信度阈值(用于日志/校验)

    // ========== 缓存配置 ==========
    size_t cache_len_limit_ = 8;     // 图像缓存上限
    int ai_msg_timeout_ms_ = 200;    // AI消息匹配超时(ms)

    // ========== 话题配置 ==========
    std::string sharedmem_img_topic_name_ = "/hbmem_img";
    std::string ai_msg_sub_topic_name_ = "/hobot_mono2d_body_detection";
    std::string ai_msg_pub_topic_name_ = "/face_landmarks_detection";

    // ========== 订阅/发布 ==========
    rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::ConstSharedPtr sharedmem_img_subscription_ = nullptr;
    rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr ai_msg_subscription_ = nullptr;
    rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr ai_msg_publisher_ = nullptr;

    // ========== 内部组件 ==========
    std::shared_ptr<AiMsgManage> ai_msg_manage_ = nullptr;
    std::shared_ptr<std::thread> predict_task_ = nullptr;

    // ========== 图像缓存 ==========
    using CacheImgType = std::pair<std::shared_ptr<FaceLandmarksDetOutput>, std::shared_ptr<NV12PyramidInput>>;
    std::queue<CacheImgType> cache_img_;
    std::mutex mtx_img_;
    std::condition_variable cv_img_;

    // ========== 性能统计 ==========
    std::atomic<uint64_t> stat_img_count_{0};      // 接收图像计数
    std::atomic<uint64_t> stat_match_count_{0};    // 匹配成功计数
    std::atomic<uint64_t> stat_infer_count_{0};    // 推理成功计数
    std::atomic<uint64_t> stat_drop_count_{0};     // 丢帧计数
    std::atomic<uint64_t> stat_filter_count_{0};   // ROI过滤计数

    // ========== 私有方法 ==========
    void SharedMemImgProcess(const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg);
    void AiMsgProcess(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg);
    void RunPredict();
    int Predict(std::vector<std::shared_ptr<DNNInput>> &inputs,
                const std::shared_ptr<std::vector<hbDNNRoi>> rois,
                std::shared_ptr<DnnNodeOutput> dnn_output);
    int NormalizeRoi(const hbDNNRoi *src, hbDNNRoi *dst,
                     float norm_ratio, uint32_t total_w, uint32_t total_h);
};

#endif  // FACE_LANDMARKS_DET_NODE_H

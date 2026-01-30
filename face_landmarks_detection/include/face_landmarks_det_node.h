// Copyright (c) 2024，D-Robotics.
// 自适应ROI版：移除人脸检测依赖，使用自适应ROI追踪

#ifndef FACE_LANDMARKS_DET_NODE_H
#define FACE_LANDMARKS_DET_NODE_H

#include <atomic>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "dnn_node/dnn_node.h"
#include "dnn_node/util/image_proc.h"
#include "ai_msgs/msg/perception_targets.hpp"
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#include "face_landmarks_det_output_parser.h"

using rclcpp::NodeOptions;
using hobot::dnn_node::DNNInput;
using hobot::dnn_node::DnnNode;
using hobot::dnn_node::DnnNodeOutput;
using hobot::dnn_node::DNNTensor;
using hobot::dnn_node::ModelTaskType;
using hobot::dnn_node::NV12PyramidInput;
using ai_msgs::msg::PerceptionTargets;

// 自适应ROI结构体
struct AdaptiveRoi {
    hbDNNRoi roi;           // ROI区域
    uint64_t track_id;      // 追踪ID
    int lost_count;         // 连续丢失帧数

    AdaptiveRoi() : track_id(0), lost_count(0) {
        roi.left = roi.top = roi.right = roi.bottom = 0;
    }

    AdaptiveRoi(int left, int top, int right, int bottom, uint64_t id = 0)
        : track_id(id), lost_count(0) {
        roi.left = left;
        roi.top = top;
        roi.right = right;
        roi.bottom = bottom;
    }
};

// 推理输出结构
struct FaceLandmarksDetOutput : public DnnNodeOutput
{
    std::shared_ptr<std_msgs::msg::Header> image_msg_header = nullptr;
    std::shared_ptr<std::vector<hbDNNRoi>> valid_rois;
    std::map<size_t, size_t> valid_roi_idx;
    ai_msgs::msg::PerceptionTargets::UniquePtr ai_msg;
    ai_msgs::msg::Perf perf_preprocess;
    int channel_id = 0;  // 通道标识 (0=左IR, 1=右IR)
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
    std::string landmarks_model_file_name_ = "./config/faceLandmark106pts.hbm";
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
    int max_lost_frames_ = 3;        // 最大丢失帧数，超过则移除追踪

    // ========== 图像尺寸配置 ==========
    int image_width_ = 1280;
    int image_height_ = 800;

    // ========== 话题配置 (双路) ==========
    // 左IR
    std::string left_img_topic_ = "/hbmem_img_left";
    std::string left_ai_pub_topic_ = "/face_landmarks_detection_left";
    // 右IR
    std::string right_img_topic_ = "/hbmem_img_right";
    std::string right_ai_pub_topic_ = "/face_landmarks_detection_right";

    // ========== 订阅/发布 (双路) ==========
    // 左IR
    rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::ConstSharedPtr left_img_sub_ = nullptr;
    rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr left_ai_pub_ = nullptr;
    // 右IR
    rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::ConstSharedPtr right_img_sub_ = nullptr;
    rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr right_ai_pub_ = nullptr;

    // ========== 自适应ROI管理 (双路) ==========
    std::vector<AdaptiveRoi> left_rois_;      // 左IR当前ROI列表
    std::vector<AdaptiveRoi> right_rois_;     // 右IR当前ROI列表
    std::mutex left_roi_mtx_;                 // 左IR ROI锁
    std::mutex right_roi_mtx_;                // 右IR ROI锁
    std::atomic<uint64_t> next_track_id_{1};  // 下一个追踪ID

    // ========== 内部组件 ==========
    std::shared_ptr<std::thread> predict_task_ = nullptr;
    std::mutex bpu_infer_mutex_;  // BPU推理互斥锁，防止双通道并发冲突

    // ========== 性能统计 ==========
    std::atomic<uint64_t> stat_img_count_{0};      // 接收图像计数
    std::atomic<uint64_t> stat_infer_count_{0};    // 推理成功计数
    std::atomic<uint64_t> stat_detect_count_{0};   // 检测到人脸计数

    // ========== 私有方法 ==========
    // 双路回调
    void LeftImgCallback(const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg);
    void RightImgCallback(const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg);
    // 通用处理
    void ProcessImage(const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg,
                      std::vector<AdaptiveRoi>& rois,
                      std::mutex& roi_mtx,
                      rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr publisher,
                      int channel_id);
    // ROI管理
    std::vector<hbDNNRoi> GetCurrentROIs(std::vector<AdaptiveRoi>& rois, std::mutex& roi_mtx);
    hbDNNRoi GetDefaultROI();
    hbDNNRoi ComputeRoiFromLandmarks(const std::vector<geometry_msgs::msg::Point32>& landmarks);
    void UpdateROIs(std::vector<AdaptiveRoi>& rois, std::mutex& roi_mtx,
                    const ai_msgs::msg::PerceptionTargets& results);
    // 推理
    int Predict(std::vector<std::shared_ptr<DNNInput>> &inputs,
                const std::shared_ptr<std::vector<hbDNNRoi>> rois,
                std::shared_ptr<DnnNodeOutput> dnn_output);
    int NormalizeRoi(const hbDNNRoi *src, hbDNNRoi *dst,
                     float norm_ratio, uint32_t total_w, uint32_t total_h);
};

#endif  // FACE_LANDMARKS_DET_NODE_H

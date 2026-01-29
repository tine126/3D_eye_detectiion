// Copyright (c) 2024，D-Robotics.
// 精简版：只保留人脸检测功能，仅支持SharedMem+NV12输入

#include "include/mono2d_body_det_node.h"

#include <memory>
#include <string>
#include <vector>
#include <sstream>

#include "dnn_node/dnn_node.h"
#include "dnn_node/util/image_proc.h"
#include "rclcpp/rclcpp.hpp"
#include "hobot_cv/hobotcv_imgproc.h"
#include "rcpputils/env.hpp"

// ==================== 工具函数 ====================

builtin_interfaces::msg::Time ConvertToRosTime(const struct timespec& time_spec) {
  builtin_interfaces::msg::Time stamp;
  stamp.set__sec(time_spec.tv_sec);
  stamp.set__nanosec(time_spec.tv_nsec);
  return stamp;
}

int CalTimeMsDuration(const builtin_interfaces::msg::Time& start,
                      const builtin_interfaces::msg::Time& end) {
  return (end.sec - start.sec) * 1000 + end.nanosec / 1000 / 1000 -
         start.nanosec / 1000 / 1000;
}

// ==================== NodeOutputManage 实现 ====================

void NodeOutputManage::Feed(uint64_t ts_ms) {
  std::unique_lock<std::mutex> lk(mtx_);
  cache_frame_.insert(ts_ms);
  if (cache_frame_.size() > cache_size_limit_) {
    cache_frame_.erase(cache_frame_.begin());
  }
}

std::vector<std::shared_ptr<DnnNodeOutput>> NodeOutputManage::Feed(
    const std::shared_ptr<DnnNodeOutput>& in_node_output) {
  std::vector<std::shared_ptr<DnnNodeOutput>> node_outputs;
  auto fasterRcnn_output = std::dynamic_pointer_cast<FasterRcnnOutput>(in_node_output);
  if (!fasterRcnn_output || !fasterRcnn_output->image_msg_header) {
    return node_outputs;
  }

  uint64_t ts_ms = fasterRcnn_output->image_msg_header->stamp.sec * 1000 +
                   fasterRcnn_output->image_msg_header->stamp.nanosec / 1000 / 1000;

  uint8_t loop_num = cache_size_limit_;
  {
    std::unique_lock<std::mutex> lk(mtx_);
    cache_node_output_[ts_ms] = in_node_output;
    if (cache_node_output_.size() > cache_size_limit_) {
      cache_node_output_.erase(cache_node_output_.begin());
    }
    if (cache_frame_.empty()) {
      return node_outputs;
    }
    loop_num = cache_node_output_.size();
  }

  // 按时间戳顺序输出推理结果
  for (uint8_t idx = 0; idx < loop_num; idx++) {
    std::shared_ptr<DnnNodeOutput> node_output = nullptr;
    {
      std::unique_lock<std::mutex> lk(mtx_);
      if (cache_frame_.empty() || cache_node_output_.empty()) {
        break;
      }

      auto first_frame = cache_frame_.begin();
      auto first_output = cache_node_output_.begin();
      if (*first_frame == first_output->first) {
        node_output = in_node_output;
        cache_frame_.erase(first_frame);
        cache_node_output_.erase(first_output);
      } else {
        if (first_output->first > *first_frame) {
          uint64_t time_ms_diff = first_output->first - *first_frame;
          if (time_ms_diff > smart_output_timeout_ms_) {
            cache_frame_.erase(first_frame);
          }
        } else if (*first_frame > first_output->first) {
          uint64_t time_ms_diff = *first_frame - first_output->first;
          if (time_ms_diff > smart_output_timeout_ms_) {
            cache_node_output_.erase(first_output);
          }
        } else {
          break;
        }
      }
    }

    if (node_output) {
      node_outputs.emplace_back(node_output);
    }
  }

  return node_outputs;
}

// ==================== Mono2dBodyDetNode 构造函数 ====================

Mono2dBodyDetNode::Mono2dBodyDetNode(const NodeOptions& options)
    : DnnNode("mono2d_body_det", options) {

  // 声明参数
  this->declare_parameter<int>("is_sync_mode", is_sync_mode_);
  this->declare_parameter<std::string>("model_file_name", model_file_name_);
  this->declare_parameter<std::string>("left_img_topic", left_img_topic_);
  this->declare_parameter<std::string>("left_pub_topic", left_pub_topic_);
  this->declare_parameter<std::string>("right_img_topic", right_img_topic_);
  this->declare_parameter<std::string>("right_pub_topic", right_pub_topic_);
  this->declare_parameter<double>("score_threshold", score_threshold_);

  // 获取参数
  this->get_parameter<int>("is_sync_mode", is_sync_mode_);
  this->get_parameter<std::string>("model_file_name", model_file_name_);
  this->get_parameter<std::string>("left_img_topic", left_img_topic_);
  this->get_parameter<std::string>("left_pub_topic", left_pub_topic_);
  this->get_parameter<std::string>("right_img_topic", right_img_topic_);
  this->get_parameter<std::string>("right_pub_topic", right_pub_topic_);
  this->get_parameter<float>("score_threshold", score_threshold_);

  // 打印关键配置
  RCLCPP_WARN(rclcpp::get_logger("mono2d_body_det"),
    "\n========== 人脸检测节点 (双路) ==========\n"
    " model_file_name: %s\n"
    " is_sync_mode: %d (%s)\n"
    " score_threshold: %.2f\n"
    " 左IR: %s -> %s\n"
    " 右IR: %s -> %s\n"
    "==========================================",
    model_file_name_.c_str(),
    is_sync_mode_, is_sync_mode_ == 0 ? "异步" : "同步",
    score_threshold_,
    left_img_topic_.c_str(), left_pub_topic_.c_str(),
    right_img_topic_.c_str(), right_pub_topic_.c_str());

  // 初始化模型
  if (Init() != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_body_det"), "Init failed!");
    rclcpp::shutdown();
    return;
  }

  // 获取模型管理器
  auto model_manage = GetModel();
  if (!model_manage) {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_body_det"), "Invalid model");
    rclcpp::shutdown();
    return;
  }

  // 获取模型名称
  if (model_name_.empty()) {
    model_name_ = GetModel()->GetName();
    RCLCPP_INFO(rclcpp::get_logger("mono2d_body_det"),
                "Model name: %s", model_name_.c_str());
  }

  // 创建双路发布者
  left_publisher_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(
      left_pub_topic_, 10);
  right_publisher_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(
      right_pub_topic_, 10);

  // 获取模型输入尺寸
  if (GetModelInputSize(0, model_input_width_, model_input_height_) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_body_det"), "Get model input size fail!");
    rclcpp::shutdown();
    return;
  }
  RCLCPP_INFO(rclcpp::get_logger("mono2d_body_det"),
              "Model input: %dx%d", model_input_width_, model_input_height_);

  // 检查零拷贝环境变量
  std::string ros_zerocopy_env = rcpputils::get_env_var("RMW_FASTRTPS_USE_QOS_FROM_XML");
  if (ros_zerocopy_env.empty() || ros_zerocopy_env != "1") {
    RCLCPP_WARN(rclcpp::get_logger("mono2d_body_det"),
      "Zero-copy not enabled! Set RMW_FASTRTPS_USE_QOS_FROM_XML=1 for best performance");
  }

  // 创建双路SharedMem订阅
  RCLCPP_INFO(rclcpp::get_logger("mono2d_body_det"),
              "订阅左IR: %s", left_img_topic_.c_str());
  left_img_subscription_ =
      this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
          left_img_topic_,
          rclcpp::SensorDataQoS(),
          std::bind(&Mono2dBodyDetNode::LeftImgCallback, this, std::placeholders::_1));

  RCLCPP_INFO(rclcpp::get_logger("mono2d_body_det"),
              "订阅右IR: %s", right_img_topic_.c_str());
  right_img_subscription_ =
      this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
          right_img_topic_,
          rclcpp::SensorDataQoS(),
          std::bind(&Mono2dBodyDetNode::RightImgCallback, this, std::placeholders::_1));
}

Mono2dBodyDetNode::~Mono2dBodyDetNode() {}

int Mono2dBodyDetNode::SetNodePara() {
  if (!dnn_node_para_ptr_) {
    return -1;
  }
  dnn_node_para_ptr_->model_file = model_file_name_;
  dnn_node_para_ptr_->model_name = model_name_;
  dnn_node_para_ptr_->model_task_type = model_task_type_;
  dnn_node_para_ptr_->task_num = 2;
  return 0;
}

// ==================== 推理函数 ====================

int Mono2dBodyDetNode::Predict(
    std::vector<std::shared_ptr<DNNInput>>& inputs,
    std::shared_ptr<DnnNodeOutput> dnn_output) {
  return Run(inputs, dnn_output, nullptr, is_sync_mode_ == 1);
}

// ==================== 双路图像回调 ====================

void Mono2dBodyDetNode::LeftImgCallback(
    const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg) {
  ProcessImage(msg, left_publisher_, "左IR", 0);
}

void Mono2dBodyDetNode::RightImgCallback(
    const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg) {
  ProcessImage(msg, right_publisher_, "右IR", 1);
}

// ==================== 图像处理 ====================

void Mono2dBodyDetNode::ProcessImage(
    const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr img_msg,
    rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr publisher,
    const std::string& channel_name,
    int channel_id) {
  (void)publisher;
  (void)channel_name;
  if (!img_msg || !rclcpp::ok()) {
    return;
  }

  struct timespec time_start = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_start);

  // 检查图像格式 (只支持NV12)
  std::string encoding(reinterpret_cast<const char*>(img_msg->encoding.data()));
  if (encoding != "nv12") {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_body_det"),
                 "Unsupported encoding: %s, only nv12 supported!", encoding.c_str());
    return;
  }

  // 计算缩放比例
  width_scale_ = static_cast<double>(img_msg->width) / model_input_width_;
  height_scale_ = static_cast<double>(img_msg->height) / model_input_height_;

  // 构建金字塔输入
  std::shared_ptr<hobot::dnn_node::NV12PyramidInput> pyramid = nullptr;

  if (img_msg->height != static_cast<uint32_t>(model_input_height_) ||
      img_msg->width != static_cast<uint32_t>(model_input_width_)) {
    // 需要resize
    auto resize_img = hobot_cv::hobotcv_resize(
        reinterpret_cast<const char*>(img_msg->data.data()),
        img_msg->height, img_msg->width,
        model_input_height_, model_input_width_);
    if (!resize_img) {
      RCLCPP_ERROR(rclcpp::get_logger("mono2d_body_det"), "Resize failed!");
      return;
    }
    pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char*>(resize_img->imageAddr),
        resize_img->height, resize_img->width,
        model_input_height_, model_input_width_);
  } else {
    // 尺寸匹配，直接使用
    pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char*>(img_msg->data.data()),
        img_msg->height, img_msg->width,
        model_input_height_, model_input_width_);
  }

  if (!pyramid) {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_body_det"), "Get NV12 pyramid fail!");
    return;
  }

  // 创建推理输入
  auto inputs = std::vector<std::shared_ptr<DNNInput>>{pyramid};
  auto dnn_output = std::make_shared<FasterRcnnOutput>();
  dnn_output->image_msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->image_msg_header->set__frame_id(std::to_string(img_msg->index));
  dnn_output->image_msg_header->set__stamp(img_msg->time_stamp);
  dnn_output->preprocess_timespec_start = time_start;
  dnn_output->channel_id = channel_id;

  struct timespec time_now = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_now);
  dnn_output->preprocess_timespec_end = time_now;

  // 记录帧时间戳
  if (node_output_manage_ptr_) {
    node_output_manage_ptr_->Feed(img_msg->time_stamp.sec * 1000 +
                                  img_msg->time_stamp.nanosec / 1000 / 1000);
  }

  // 执行推理
  if (Predict(inputs, dnn_output) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_body_det"), "Predict failed!");
  }
}

// ==================== 后处理函数 ====================

int Mono2dBodyDetNode::PostProcess(const std::shared_ptr<DnnNodeOutput>& output) {
  if (!rclcpp::ok()) {
    return -1;
  }

  // 输出排序
  std::vector<std::shared_ptr<DnnNodeOutput>> node_outputs{};
  if (node_output_manage_ptr_) {
    node_outputs = node_output_manage_ptr_->Feed(output);
    if (node_outputs.empty()) {
      auto fasterRcnn_output = std::dynamic_pointer_cast<FasterRcnnOutput>(output);
      if (!fasterRcnn_output || !fasterRcnn_output->image_msg_header) {
        node_outputs.push_back(output);
      }
    }
  } else {
    node_outputs.push_back(output);
  }

  for (const auto& node_output : node_outputs) {
    if (!node_output) continue;

    auto fasterRcnn_output = std::dynamic_pointer_cast<FasterRcnnOutput>(node_output);

    // 解析模型输出 (只解析face)
    std::vector<std::shared_ptr<hobot::dnn_node::parser_fasterrcnn::Filter2DResult>> results;
    std::shared_ptr<hobot::dnn_node::parser_fasterrcnn::LandmarksResult> landmarks_result;
    int ret = hobot::dnn_node::parser_fasterrcnn::Parse(
        node_output, parser_para_, box_outputs_index_, -1, -1, results, landmarks_result);

    if (ret < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("mono2d_body_det"), "Parse failed!");
      return -1;
    }

    struct timespec time_start = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_start);

    // 构建输出消息
    ai_msgs::msg::PerceptionTargets::UniquePtr pub_data(
        new ai_msgs::msg::PerceptionTargets());

    if (fasterRcnn_output->image_msg_header) {
      pub_data->header.set__stamp(fasterRcnn_output->image_msg_header->stamp);
      pub_data->header.set__frame_id(fasterRcnn_output->image_msg_header->frame_id);
    }
    if (output->rt_stat) {
      pub_data->set__fps(round(output->rt_stat->output_fps));
    }

    // 处理face检测结果
    if (face_box_output_index_ < static_cast<int32_t>(results.size())) {
      auto filter2d_result = results.at(face_box_output_index_);
      if (filter2d_result) {
        // 按置信度降序排序
        std::sort(filter2d_result->boxes.begin(), filter2d_result->boxes.end(),
            [](const auto& a, const auto& b) { return a.conf > b.conf; });

        for (const auto& rect : filter2d_result->boxes) {
          // 置信度过滤
          if (rect.conf < score_threshold_) continue;

          ai_msgs::msg::Target target;
          target.set__type("face");

          ai_msgs::msg::Roi roi;
          roi.type = "face";
          roi.rect.set__x_offset(static_cast<int>(rect.left / width_scale_));
          roi.rect.set__y_offset(static_cast<int>(rect.top / height_scale_));
          roi.rect.set__width(static_cast<int>((rect.right - rect.left) / width_scale_));
          roi.rect.set__height(static_cast<int>((rect.bottom - rect.top) / height_scale_));
          roi.set__confidence(rect.conf);

          target.rois.emplace_back(roi);
          pub_data->targets.emplace_back(std::move(target));
        }
      }
    }

    // 填充性能统计
    // preprocess
    ai_msgs::msg::Perf perf_preprocess;
    perf_preprocess.set__type(model_name_ + "_preprocess");
    perf_preprocess.set__stamp_start(ConvertToRosTime(fasterRcnn_output->preprocess_timespec_start));
    perf_preprocess.set__stamp_end(ConvertToRosTime(fasterRcnn_output->preprocess_timespec_end));
    perf_preprocess.set__time_ms_duration(
        CalTimeMsDuration(perf_preprocess.stamp_start, perf_preprocess.stamp_end));
    pub_data->perfs.emplace_back(perf_preprocess);

    // predict
    if (output->rt_stat) {
      ai_msgs::msg::Perf perf;
      perf.set__type(model_name_ + "_predict_infer");
      perf.set__stamp_start(ConvertToRosTime(output->rt_stat->infer_timespec_start));
      perf.set__stamp_end(ConvertToRosTime(output->rt_stat->infer_timespec_end));
      perf.set__time_ms_duration(output->rt_stat->infer_time_ms);
      pub_data->perfs.push_back(perf);
    }

    // postprocess
    struct timespec time_now = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_now);
    ai_msgs::msg::Perf perf_postprocess;
    perf_postprocess.set__type(model_name_ + "_postprocess");
    perf_postprocess.set__stamp_start(ConvertToRosTime(time_start));
    perf_postprocess.set__stamp_end(ConvertToRosTime(time_now));
    perf_postprocess.set__time_ms_duration(
        CalTimeMsDuration(perf_postprocess.stamp_start, perf_postprocess.stamp_end));
    pub_data->perfs.emplace_back(perf_postprocess);

    // pipeline
    ai_msgs::msg::Perf perf_pipeline;
    perf_pipeline.set__type(model_name_ + "_pipeline");
    perf_pipeline.set__stamp_start(pub_data->header.stamp);
    perf_pipeline.set__stamp_end(perf_postprocess.stamp_end);
    perf_pipeline.set__time_ms_duration(
        CalTimeMsDuration(perf_pipeline.stamp_start, perf_pipeline.stamp_end));
    pub_data->perfs.push_back(perf_pipeline);

    // 性能日志 (每5秒输出一次)
    if (node_output->rt_stat && node_output->rt_stat->fps_updated) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "[Perf] in_fps: %.1f, out_fps: %.1f, infer: %dms, post: %dms, pipeline: %dms, faces: %zu",
          node_output->rt_stat->input_fps,
          node_output->rt_stat->output_fps,
          node_output->rt_stat->infer_time_ms,
          static_cast<int>(perf_postprocess.time_ms_duration),
          static_cast<int>(perf_pipeline.time_ms_duration),
          pub_data->targets.size());
    }

    // 发布结果 (根据channel_id选择对应的publisher)
    auto& publisher = (fasterRcnn_output->channel_id == 0) ? left_publisher_ : right_publisher_;
    if (publisher) {
      publisher->publish(std::move(pub_data));
    }
  }

  return 0;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(Mono2dBodyDetNode)

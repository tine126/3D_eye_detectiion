// Copyright (c) 2024，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// 精简版：只保留人脸检测功能，仅支持SharedMem+NV12输入

#ifndef MONO2D_BODY_DET_NODE_H_
#define MONO2D_BODY_DET_NODE_H_

#include <atomic>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>

#include "rclcpp/rclcpp.hpp"
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "dnn_node/dnn_node.h"
#include "dnn_node/util/output_parser/detection/fasterrcnn_output_parser.h"

using rclcpp::NodeOptions;
using hobot::dnn_node::DNNInput;
using hobot::dnn_node::DnnNode;
using hobot::dnn_node::DnnNodeOutput;
using hobot::dnn_node::ModelTaskType;
using hobot::dnn_node::NV12PyramidInput;
using hobot::dnn_node::parser_fasterrcnn::FasterRcnnKpsParserPara;

// 输出排序管理器：解决异步推理输出乱序问题
class NodeOutputManage {
 public:
  void Feed(uint64_t ts_ms);
  std::vector<std::shared_ptr<DnnNodeOutput>> Feed(
      const std::shared_ptr<DnnNodeOutput>& node_output);

 private:
  std::set<uint64_t> cache_frame_;
  std::map<uint64_t, std::shared_ptr<DnnNodeOutput>> cache_node_output_;
  const uint8_t cache_size_limit_ = 10;  // 最大缓存10帧
  std::mutex mtx_;
  const uint64_t smart_output_timeout_ms_ = 1000;
};

// 推理输出结构
struct FasterRcnnOutput : public DnnNodeOutput {
  std::shared_ptr<std_msgs::msg::Header> image_msg_header = nullptr;
  struct timespec preprocess_timespec_start;
  struct timespec preprocess_timespec_end;
  // 标识数据来源通道 (0=左IR, 1=右IR)
  int channel_id = 0;
};

// 精简版人脸检测节点
class Mono2dBodyDetNode : public DnnNode {
 public:
  Mono2dBodyDetNode(const NodeOptions& options = NodeOptions());
  ~Mono2dBodyDetNode() override;

 protected:
  int SetNodePara() override;
  int PostProcess(const std::shared_ptr<DnnNodeOutput>& outputs) override;

 private:
  // ========== 模型配置 ==========
  // 模型文件路径 (FasterRCNN多任务模型)
  std::string model_file_name_ = "config/multitask_body_head_face_hand_kps_960x544.hbm";
  std::string model_name_ = "";
  ModelTaskType model_task_type_ = ModelTaskType::ModelInferType;

  // 模型输入尺寸
  int model_input_width_ = -1;
  int model_input_height_ = -1;

  // 缩放比例 (原图尺寸 / 模型输入尺寸)
  std::atomic<double> width_scale_{1.0};
  std::atomic<double> height_scale_{1.0};

  // ========== 输出配置 (只保留face) ==========
  const int32_t face_box_output_index_ = 5;  // face输出索引固定为5
  std::vector<int32_t> box_outputs_index_ = {face_box_output_index_};

  // face检测置信度阈值 (可通过参数配置)
  float score_threshold_ = 0.5;

  // ========== 推理配置 ==========
  // 同步/异步推理模式: 0=异步(默认), 1=同步
  int is_sync_mode_ = 0;

  // ========== 订阅配置 (双路SharedMem) ==========
  // 左IR
  std::string left_img_topic_ = "/hbmem_img_left";
  rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::ConstSharedPtr
      left_img_subscription_ = nullptr;
  // 右IR
  std::string right_img_topic_ = "/hbmem_img_right";
  rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::ConstSharedPtr
      right_img_subscription_ = nullptr;

  // ========== 发布配置 (双路输出) ==========
  // 左IR (参数名加 mono2d_ 前缀避免与其他节点冲突)
  std::string mono2d_left_pub_topic_ = "/hobot_mono2d_body_detection_left";
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr left_publisher_ = nullptr;
  // 右IR
  std::string mono2d_right_pub_topic_ = "/hobot_mono2d_body_detection_right";
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr right_publisher_ = nullptr;

  // ========== 内部组件 ==========
  std::shared_ptr<FasterRcnnKpsParserPara> parser_para_ = nullptr;
  std::shared_ptr<NodeOutputManage> node_output_manage_ptr_ = std::make_shared<NodeOutputManage>();

  // ========== 私有方法 ==========
  int Predict(std::vector<std::shared_ptr<DNNInput>>& inputs,
              std::shared_ptr<DnnNodeOutput> dnn_output);

  void LeftImgCallback(const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg);
  void RightImgCallback(const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg);
  void ProcessImage(const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg,
                    rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr publisher,
                    const std::string& channel_name,
                    int channel_id);
};

#endif  // MONO2D_BODY_DET_NODE_H_

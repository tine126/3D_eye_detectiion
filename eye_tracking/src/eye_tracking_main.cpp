// Copyright (c) 2024, TCL.
// 人眼三维定位系统 - 整合主文件
// 订阅人脸关键点，提取眼睛2D坐标，计算3D坐标

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ai_msgs/msg/perception_targets.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <fstream>
#include <sstream>
#include <regex>

// 眼睛关键点索引 (106点人脸关键点)
constexpr int kLeftEyeStartIdx = 33;   // 左眼起始点
constexpr int kLeftEyeEndIdx = 41;     // 左眼结束点
constexpr int kRightEyeStartIdx = 42;  // 右眼起始点
constexpr int kRightEyeEndIdx = 50;    // 右眼结束点

class EyeTrackingMainNode : public rclcpp::Node {
public:
  EyeTrackingMainNode() : Node("eye_tracking_main") {
    // 声明参数
    this->declare_parameter<std::string>("left_topic", "/face_landmarks_left");
    this->declare_parameter<std::string>("right_topic", "/face_landmarks_right");
    this->declare_parameter<std::string>("calibration_file", "");
    this->declare_parameter<int>("log_interval", 10);

    // 获取参数
    this->get_parameter("left_topic", left_topic_);
    this->get_parameter("right_topic", right_topic_);
    this->get_parameter("calibration_file", calibration_file_);
    this->get_parameter("log_interval", log_interval_);

    // 加载标定参数
    if (!calibration_file_.empty()) {
      LoadCalibration(calibration_file_);
    }

    // 创建订阅者
    left_sub_.subscribe(this, left_topic_);
    right_sub_.subscribe(this, right_topic_);

    // 设置时间同步器
    sync_ = std::make_shared<Synchronizer>(SyncPolicy(10), left_sub_, right_sub_);
    sync_->registerCallback(std::bind(&EyeTrackingMainNode::SyncCallback, this,
                                       std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "人眼三维定位系统启动");
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "订阅话题:");
    RCLCPP_INFO(this->get_logger(), "  左路: %s", left_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  右路: %s", right_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "相机参数:");
    RCLCPP_INFO(this->get_logger(), "  fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f", fx_, fy_, cx_, cy_);
    RCLCPP_INFO(this->get_logger(), "  baseline=%.4f m", baseline_);
    RCLCPP_INFO(this->get_logger(), "========================================");
  }

private:
  // 消息同步类型定义
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      ai_msgs::msg::PerceptionTargets, ai_msgs::msg::PerceptionTargets>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  // 成员变量
  message_filters::Subscriber<ai_msgs::msg::PerceptionTargets> left_sub_;
  message_filters::Subscriber<ai_msgs::msg::PerceptionTargets> right_sub_;
  std::shared_ptr<Synchronizer> sync_;

  std::string left_topic_;
  std::string right_topic_;
  std::string calibration_file_;
  int log_interval_ = 10;
  int frame_count_ = 0;

  // 相机标定参数 (默认值)
  double fx_ = 600.0;
  double fy_ = 600.0;
  double cx_ = 640.0;
  double cy_ = 400.0;
  double baseline_ = 0.095;  // 95mm基线

  // 加载标定文件
  void LoadCalibration(const std::string& path) {
    try {
      std::ifstream file(path);
      if (!file.is_open()) {
        RCLCPP_WARN(this->get_logger(), "无法打开标定文件: %s", path.c_str());
        return;
      }

      std::stringstream buffer;
      buffer << file.rdbuf();
      std::string content = buffer.str();

      auto parse = [&content](const std::string& key, double def) -> double {
        std::regex pattern("\"" + key + "\"\\s*:\\s*([0-9.\\-]+)");
        std::smatch match;
        if (std::regex_search(content, match, pattern) && match.size() > 1) {
          return std::stod(match[1].str());
        }
        return def;
      };

      fx_ = parse("fx", fx_);
      fy_ = parse("fy", fy_);
      cx_ = parse("cx", cx_);
      cy_ = parse("cy", cy_);
      baseline_ = parse("baseline", baseline_);

      RCLCPP_INFO(this->get_logger(), "已加载标定文件: %s", path.c_str());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "加载标定文件失败: %s", e.what());
    }
  }

  // 从人脸关键点提取眼睛中心坐标
  bool ExtractEyeCenter(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr& msg,
                        float& left_x, float& left_y,
                        float& right_x, float& right_y) {
    if (msg->targets.empty()) {
      return false;
    }

    // 使用第一个检测到的目标
    const auto& target = msg->targets[0];

    // 查找人脸关键点
    for (const auto& pt : target.points) {
      if (pt.type != "face_kps") {
        continue;
      }

      const auto& points = pt.point;
      if (points.size() < 106) {
        RCLCPP_WARN(this->get_logger(), "关键点数量不足: %zu", points.size());
        return false;
      }

      // 计算左眼中心 (点33-41)
      left_x = 0; left_y = 0;
      for (int i = kLeftEyeStartIdx; i <= kLeftEyeEndIdx; ++i) {
        left_x += points[i].x;
        left_y += points[i].y;
      }
      int left_count = kLeftEyeEndIdx - kLeftEyeStartIdx + 1;
      left_x /= left_count;
      left_y /= left_count;

      // 计算右眼中心 (点42-50)
      right_x = 0; right_y = 0;
      for (int i = kRightEyeStartIdx; i <= kRightEyeEndIdx; ++i) {
        right_x += points[i].x;
        right_y += points[i].y;
      }
      int right_count = kRightEyeEndIdx - kRightEyeStartIdx + 1;
      right_x /= right_count;
      right_y /= right_count;

      return true;
    }

    return false;
  }

  // 计算3D坐标
  void Calculate3D(float left_cam_x, float right_cam_x, float y,
                   double& X, double& Y, double& Z) {
    float disparity = left_cam_x - right_cam_x;
    if (disparity > 0.1) {
      Z = fx_ * baseline_ / disparity;
      X = (left_cam_x - cx_) * Z / fx_;
      Y = (y - cy_) * Z / fy_;
    } else {
      X = Y = Z = 0;
    }
  }

  // 同步回调函数
  void SyncCallback(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr& left_msg,
                    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr& right_msg) {
    frame_count_++;

    // 从左路图像提取眼睛坐标
    float left_eye_x_L, left_eye_y_L, right_eye_x_L, right_eye_y_L;
    bool left_valid = ExtractEyeCenter(left_msg,
                                        left_eye_x_L, left_eye_y_L,
                                        right_eye_x_L, right_eye_y_L);

    // 从右路图像提取眼睛坐标
    float left_eye_x_R, left_eye_y_R, right_eye_x_R, right_eye_y_R;
    bool right_valid = ExtractEyeCenter(right_msg,
                                         left_eye_x_R, left_eye_y_R,
                                         right_eye_x_R, right_eye_y_R);

    if (!left_valid || !right_valid) {
      if (frame_count_ % log_interval_ == 0) {
        RCLCPP_WARN(this->get_logger(), "[帧%d] 未检测到人脸关键点", frame_count_);
      }
      return;
    }

    // 计算左眼3D坐标
    double left_eye_X, left_eye_Y, left_eye_Z;
    Calculate3D(left_eye_x_L, left_eye_x_R, left_eye_y_L,
                left_eye_X, left_eye_Y, left_eye_Z);

    // 计算右眼3D坐标
    double right_eye_X, right_eye_Y, right_eye_Z;
    Calculate3D(right_eye_x_L, right_eye_x_R, right_eye_y_L,
                right_eye_X, right_eye_Y, right_eye_Z);

    // 计算视差
    float left_disparity = left_eye_x_L - left_eye_x_R;
    float right_disparity = right_eye_x_L - right_eye_x_R;

    // 输出结果
    if (frame_count_ % log_interval_ == 0) {
      RCLCPP_INFO(this->get_logger(), "========================================");
      RCLCPP_INFO(this->get_logger(), "[帧%d] 人眼三维坐标", frame_count_);
      RCLCPP_INFO(this->get_logger(), "----------------------------------------");
      RCLCPP_INFO(this->get_logger(), "左眼 2D: 左相机(%.1f, %.1f) 右相机(%.1f, %.1f)",
                  left_eye_x_L, left_eye_y_L, left_eye_x_R, left_eye_y_R);
      RCLCPP_INFO(this->get_logger(), "左眼 3D: X=%.3fm Y=%.3fm Z=%.3fm (视差=%.1f)",
                  left_eye_X, left_eye_Y, left_eye_Z, left_disparity);
      RCLCPP_INFO(this->get_logger(), "----------------------------------------");
      RCLCPP_INFO(this->get_logger(), "右眼 2D: 左相机(%.1f, %.1f) 右相机(%.1f, %.1f)",
                  right_eye_x_L, right_eye_y_L, right_eye_x_R, right_eye_y_R);
      RCLCPP_INFO(this->get_logger(), "右眼 3D: X=%.3fm Y=%.3fm Z=%.3fm (视差=%.1f)",
                  right_eye_X, right_eye_Y, right_eye_Z, right_disparity);
      RCLCPP_INFO(this->get_logger(), "========================================");
    }
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EyeTrackingMainNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

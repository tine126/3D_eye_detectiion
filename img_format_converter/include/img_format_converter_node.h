// Copyright (c) 2024，D-Robotics.
// img_format_converter: mono8 -> nv12 格式转换节点

#ifndef IMG_FORMAT_CONVERTER_NODE_H_
#define IMG_FORMAT_CONVERTER_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <hbm_img_msgs/msg/hbm_msg1080_p.hpp>
#include <atomic>
#include <chrono>

class ImgFormatConverterNode : public rclcpp::Node
{
public:
    explicit ImgFormatConverterNode(
        const std::string &node_name = "img_format_converter_node",
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~ImgFormatConverterNode() override = default;

private:
    // ========== 话题配置 ==========
    std::string sub_topic_name_ = "/camera/ir/image_raw";
    std::string pub_topic_name_ = "/hbmem_img";

    // ========== 图像配置 ==========
    int image_width_ = 1280;
    int image_height_ = 800;

    // ========== 订阅/发布 ==========
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_ = nullptr;
    rclcpp::Publisher<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr hbmem_publisher_ = nullptr;

    // ========== 性能统计 ==========
    std::atomic<uint64_t> stat_frame_count_{0};
    std::chrono::steady_clock::time_point stat_start_time_;
    static constexpr int STAT_INTERVAL_SEC = 5;

    // 耗时统计 (转换耗时)
    std::atomic<uint64_t> stat_total_convert_us_{0};
    std::atomic<uint64_t> stat_max_convert_us_{0};
    std::atomic<uint64_t> stat_min_convert_us_{UINT64_MAX};

    // 延迟统计 (端到端延迟: 图像时间戳 -> 发布完成)
    std::atomic<int64_t> stat_total_e2e_us_{0};
    std::atomic<int64_t> stat_max_e2e_us_{0};
    std::atomic<int64_t> stat_min_e2e_us_{INT64_MAX};

    // ========== 私有方法 ==========
    void ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void ConvertMono8ToNV12(const uint8_t* mono8_data, uint8_t* nv12_data,
                            int width, int height);
    void PrintStatistics();
};

#endif  // IMG_FORMAT_CONVERTER_NODE_H_

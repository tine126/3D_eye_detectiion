// Copyright (c) 2024，D-Robotics.
// img_format_converter: mono8 -> nv12 格式转换节点

#include "img_format_converter_node.h"
#include <cstring>

ImgFormatConverterNode::ImgFormatConverterNode(
    const std::string &node_name,
    const rclcpp::NodeOptions &options)
    : Node(node_name, options)
{
    // 声明并获取参数
    this->declare_parameter("sub_topic_name", sub_topic_name_);
    this->declare_parameter("pub_topic_name", pub_topic_name_);
    this->declare_parameter("image_width", image_width_);
    this->declare_parameter("image_height", image_height_);

    this->get_parameter("sub_topic_name", sub_topic_name_);
    this->get_parameter("pub_topic_name", pub_topic_name_);
    this->get_parameter("image_width", image_width_);
    this->get_parameter("image_height", image_height_);

    RCLCPP_INFO(this->get_logger(), "Parameters:");
    RCLCPP_INFO(this->get_logger(), "  sub_topic_name: %s", sub_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "  pub_topic_name: %s", pub_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "  image_size: %dx%d", image_width_, image_height_);

    // 创建SharedMem发布者
    hbmem_publisher_ = this->create_publisher<hbm_img_msgs::msg::HbmMsg1080P>(
        pub_topic_name_, rclcpp::SensorDataQoS());

    // 创建图像订阅者
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        sub_topic_name_, rclcpp::SensorDataQoS(),
        std::bind(&ImgFormatConverterNode::ImageCallback, this, std::placeholders::_1));

    // 初始化统计
    stat_start_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(this->get_logger(), "Node initialized successfully");
}

void ImgFormatConverterNode::ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    if (!msg || !rclcpp::ok()) return;

    auto start_time = std::chrono::steady_clock::now();

    // 检查输入格式
    if (msg->encoding != "mono8")
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Unsupported encoding: %s, expected mono8", msg->encoding.c_str());
        return;
    }

    // 检查图像尺寸
    if (static_cast<int>(msg->width) != image_width_ ||
        static_cast<int>(msg->height) != image_height_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Image size mismatch: got %ux%u, expected %dx%d",
            msg->width, msg->height, image_width_, image_height_);
    }

    // 创建输出消息
    auto hbmem_msg = std::make_unique<hbm_img_msgs::msg::HbmMsg1080P>();

    // 设置消息头
    hbmem_msg->time_stamp = msg->header.stamp;
    hbmem_msg->index = stat_frame_count_;
    hbmem_msg->width = msg->width;
    hbmem_msg->height = msg->height;

    // 设置编码为nv12
    std::string encoding = "nv12";
    std::memcpy(hbmem_msg->encoding.data(), encoding.c_str(), encoding.size() + 1);

    // 转换mono8到nv12
    ConvertMono8ToNV12(msg->data.data(), hbmem_msg->data.data(),
                       msg->width, msg->height);

    // 设置数据大小 (NV12 = width * height * 1.5)
    hbmem_msg->data_size = msg->width * msg->height * 3 / 2;

    // 发布消息
    hbmem_publisher_->publish(std::move(hbmem_msg));

    // 计算转换耗时
    auto end_time = std::chrono::steady_clock::now();
    auto convert_us = std::chrono::duration_cast<std::chrono::microseconds>(
        end_time - start_time).count();

    // 计算端到端延迟 (图像时间戳 -> 发布完成)
    auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    auto img_ns = static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
                  static_cast<int64_t>(msg->header.stamp.nanosec);
    auto e2e_us = (now_ns - img_ns) / 1000;

    // 更新统计
    stat_frame_count_++;
    stat_total_convert_us_ += convert_us;
    stat_total_e2e_us_ += e2e_us;

    // 更新最大/最小转换耗时
    uint64_t cur_max = stat_max_convert_us_.load();
    while (static_cast<uint64_t>(convert_us) > cur_max &&
           !stat_max_convert_us_.compare_exchange_weak(cur_max, convert_us)) {}
    uint64_t cur_min = stat_min_convert_us_.load();
    while (static_cast<uint64_t>(convert_us) < cur_min &&
           !stat_min_convert_us_.compare_exchange_weak(cur_min, convert_us)) {}

    // 更新最大/最小端到端延迟
    int64_t cur_max_e2e = stat_max_e2e_us_.load();
    while (e2e_us > cur_max_e2e &&
           !stat_max_e2e_us_.compare_exchange_weak(cur_max_e2e, e2e_us)) {}
    int64_t cur_min_e2e = stat_min_e2e_us_.load();
    while (e2e_us < cur_min_e2e &&
           !stat_min_e2e_us_.compare_exchange_weak(cur_min_e2e, e2e_us)) {}

    // 定期打印统计
    PrintStatistics();
}

void ImgFormatConverterNode::ConvertMono8ToNV12(
    const uint8_t* mono8_data, uint8_t* nv12_data,
    int width, int height)
{
    // Y平面：直接复制mono8数据（灰度值=亮度值）
    int y_size = width * height;
    std::memcpy(nv12_data, mono8_data, y_size);

    // UV平面：填充128（中性灰，无色度）
    int uv_size = y_size / 2;
    std::memset(nv12_data + y_size, 128, uv_size);
}

void ImgFormatConverterNode::PrintStatistics()
{
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        now - stat_start_time_).count();

    if (elapsed >= STAT_INTERVAL_SEC)
    {
        uint64_t frame_count = stat_frame_count_.load();
        if (frame_count == 0)
        {
            stat_start_time_ = now;
            return;
        }

        // 转换耗时统计
        uint64_t total_convert = stat_total_convert_us_.load();
        uint64_t max_convert = stat_max_convert_us_.load();
        uint64_t min_convert = stat_min_convert_us_.load();
        double avg_convert_ms = static_cast<double>(total_convert) / frame_count / 1000.0;
        double max_convert_ms = static_cast<double>(max_convert) / 1000.0;
        double min_convert_ms = static_cast<double>(min_convert) / 1000.0;

        // 端到端延迟统计
        int64_t total_e2e = stat_total_e2e_us_.load();
        int64_t max_e2e = stat_max_e2e_us_.load();
        int64_t min_e2e = stat_min_e2e_us_.load();
        double avg_e2e_ms = static_cast<double>(total_e2e) / frame_count / 1000.0;
        double max_e2e_ms = static_cast<double>(max_e2e) / 1000.0;
        double min_e2e_ms = static_cast<double>(min_e2e) / 1000.0;

        double fps = static_cast<double>(frame_count) / elapsed;

        RCLCPP_INFO(this->get_logger(),
            "统计 [%ld秒]: 帧数=%lu, 帧率=%.1f",
            elapsed, frame_count, fps);
        RCLCPP_INFO(this->get_logger(),
            "  转换耗时: 平均=%.2fms, 最小=%.2fms, 最大=%.2fms",
            avg_convert_ms, min_convert_ms, max_convert_ms);
        RCLCPP_INFO(this->get_logger(),
            "  端到端延迟: 平均=%.2fms, 最小=%.2fms, 最大=%.2fms",
            avg_e2e_ms, min_e2e_ms, max_e2e_ms);

        // 重置统计
        stat_frame_count_ = 0;
        stat_total_convert_us_ = 0;
        stat_max_convert_us_ = 0;
        stat_min_convert_us_ = UINT64_MAX;
        stat_total_e2e_us_ = 0;
        stat_max_e2e_us_ = 0;
        stat_min_e2e_us_ = INT64_MAX;
        stat_start_time_ = now;
    }
}

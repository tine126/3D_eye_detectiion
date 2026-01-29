// Copyright (c) 2024，D-Robotics.
// 精简版：仅保留在线模式

#ifndef AI_MSG_MANAGE_H
#define AI_MSG_MANAGE_H

#include <queue>
#include <unordered_map>
#include <mutex>
#include <condition_variable>

#include "rclcpp/rclcpp.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "dnn_node/dnn_node_data.h"

using ai_msgs::msg::PerceptionTargets;

// 时间戳比较器
struct compare_msg
{
    bool operator()(const std_msgs::msg::Header::_stamp_type m1,
                    const std_msgs::msg::Header::_stamp_type m2)
    {
        return ((m1.sec > m2.sec) || ((m1.sec == m2.sec) && (m1.nanosec > m2.nanosec)));
    }
};

// AI消息缓存
class FaceLandmarksDetFeedCache
{
public:
    explicit FaceLandmarksDetFeedCache(const rclcpp::Logger &logger, size_t cache_limit = 20)
        : logger_(logger), cache_limit_(cache_limit) {}

    int Feed(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg)
    {
        ai_msgs::msg::PerceptionTargets::UniquePtr ai_msg(new ai_msgs::msg::PerceptionTargets());
        ai_msg->set__header(msg->header);
        ai_msg->set__fps(msg->fps);
        ai_msg->set__targets(msg->targets);
        ai_msg->set__disappeared_targets(msg->disappeared_targets);
        ai_msg->set__perfs(msg->perfs);

        std::string ts = std::to_string(msg->header.stamp.sec) + "." +
                         std::to_string(msg->header.stamp.nanosec);

        std::unique_lock<std::mutex> lg(cache_mtx_);
        // 超限清理
        while (recved_aimsg_cache_.size() >= cache_limit_)
        {
            auto top_ts = std::to_string(recved_aimsg_ts_.top().sec) + "." +
                          std::to_string(recved_aimsg_ts_.top().nanosec);
            recved_aimsg_cache_.erase(top_ts);
            recved_aimsg_ts_.pop();
        }

        recved_aimsg_cache_[ts] = std::move(ai_msg);
        recved_aimsg_ts_.push(msg->header.stamp);
        cache_cv_.notify_one();
        return 0;
    }

    ai_msgs::msg::PerceptionTargets::UniquePtr Get(
        const std_msgs::msg::Header::_stamp_type &msg_ts, int timeout_ms = 200)
    {
        std::string ts = std::to_string(msg_ts.sec) + "." + std::to_string(msg_ts.nanosec);
        ai_msgs::msg::PerceptionTargets::UniquePtr result = nullptr;

        std::unique_lock<std::mutex> lg(cache_mtx_);
        cache_cv_.wait_for(lg, std::chrono::milliseconds(timeout_ms),
            [&]() { return recved_aimsg_cache_.find(ts) != recved_aimsg_cache_.end(); });

        if (recved_aimsg_cache_.find(ts) != recved_aimsg_cache_.end())
        {
            result = std::move(recved_aimsg_cache_.at(ts));
            recved_aimsg_cache_.erase(ts);

            // 清理过期时间戳
            while (!recved_aimsg_ts_.empty() && recved_aimsg_ts_.size() > recved_aimsg_cache_.size())
            {
                auto &top = recved_aimsg_ts_.top();
                if ((top == msg_ts) || (top.sec < msg_ts.sec) ||
                    (top.sec == msg_ts.sec && top.nanosec < msg_ts.nanosec))
                {
                    auto top_ts = std::to_string(top.sec) + "." + std::to_string(top.nanosec);
                    recved_aimsg_cache_.erase(top_ts);
                    recved_aimsg_ts_.pop();
                }
                else break;
            }
        }
        return result;
    }

private:
    std::priority_queue<std_msgs::msg::Header::_stamp_type,
        std::vector<std_msgs::msg::Header::_stamp_type>, compare_msg> recved_aimsg_ts_;
    std::unordered_map<std::string, ai_msgs::msg::PerceptionTargets::UniquePtr> recved_aimsg_cache_;
    std::mutex cache_mtx_;
    std::condition_variable cache_cv_;
    rclcpp::Logger logger_;
    size_t cache_limit_;
};

// AI消息管理器
class AiMsgManage
{
public:
    AiMsgManage(const rclcpp::Logger &logger) : logger_(logger), cache_(logger) {}

    void Feed(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg)
    {
        if (msg) cache_.Feed(msg);
    }

    int GetTargetRois(
        const std_msgs::msg::Header::_stamp_type &msg_ts,
        std::shared_ptr<std::vector<hbDNNRoi>> &rois,
        std::map<size_t, size_t> &valid_roi_idx,
        ai_msgs::msg::PerceptionTargets::UniquePtr &ai_msg,
        std::function<int(const hbDNNRoi*, hbDNNRoi*)> norm_func,
        int timeout_ms = 200)
    {
        ai_msg = cache_.Get(msg_ts, timeout_ms);
        if (!ai_msg) return -1;
        if (ai_msg->targets.empty()) return 0;

        size_t face_roi_idx = 0;
        for (const auto &target : ai_msg->targets)
        {
            for (const auto &roi : target.rois)
            {
                if (roi.type != "face") continue;

                hbDNNRoi raw_roi{
                    static_cast<int32_t>(roi.rect.x_offset),
                    static_cast<int32_t>(roi.rect.y_offset),
                    static_cast<int32_t>(roi.rect.x_offset + roi.rect.width),
                    static_cast<int32_t>(roi.rect.y_offset + roi.rect.height)
                };
                hbDNNRoi normed_roi;

                if (norm_func(&raw_roi, &normed_roi) == 0)
                {
                    if (!rois) rois = std::make_shared<std::vector<hbDNNRoi>>();
                    rois->push_back(normed_roi);
                    valid_roi_idx[face_roi_idx] = rois->size() - 1;
                }
                face_roi_idx++;
            }
        }
        return 0;
    }

private:
    rclcpp::Logger logger_;
    FaceLandmarksDetFeedCache cache_;
};

#endif

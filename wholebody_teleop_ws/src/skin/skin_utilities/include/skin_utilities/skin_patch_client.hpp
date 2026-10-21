#ifndef SKIN_UTILITIES_SKIN_PATCH_CLIENT_HPP_
#define SKIN_UTILITIES_SKIN_PATCH_CLIENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <tf2_eigen/tf2_eigen.hpp>

#include <skin_msgs/msg/skin_patch_info.hpp>

namespace skin_utilities
{

template <typename DataMsgT>
class SkinPatchClient
{
public:
    SkinPatchClient(rclcpp::Node::SharedPtr node,
                    const std::string& topic_prefix,
                    const std::string& data_topic_suffix,
                    rclcpp::CallbackGroup::SharedPtr cbg = nullptr)
        : node_(node), has_info_(false), has_data_(false)
    {
        auto info_qos = rclcpp::QoS(1).transient_local().reliable();
        info_sub_ = node_->create_subscription<skin_msgs::msg::SkinPatchInfo>(
            topic_prefix + "/info", info_qos,
            std::bind(&SkinPatchClient::infoCallback, this, std::placeholders::_1));

        rclcpp::SubscriptionOptions data_opts;
        data_opts.callback_group = cbg;
        data_sub_ = node_->create_subscription<DataMsgT>(
            topic_prefix + data_topic_suffix, rclcpp::SensorDataQoS(),
            std::bind(&SkinPatchClient::dataCallback, this, std::placeholders::_1),
            data_opts);

        RCLCPP_INFO(node_->get_logger(), "SkinPatchClient initialized for: %s",
                    topic_prefix.c_str());
    }

    void setOnDataCallback(std::function<void(const typename DataMsgT::SharedPtr&)> cb)
    {
        on_data_cb_ = std::move(cb);
    }

    bool waitForReady(double timeout_sec = 5.0)
    {
        auto start = std::chrono::steady_clock::now();
        auto timeout = std::chrono::duration<double>(timeout_sec);
        while (rclcpp::ok())
        {
            if (isReady()) return true;
            if ((std::chrono::steady_clock::now() - start) > timeout) return false;
            rclcpp::spin_some(node_);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return false;
    }

    bool isReady() const { return has_info_ && has_data_; }

    const skin_msgs::msg::SkinPatchInfo& info() const { return latest_info_; }
    const std::vector<Eigen::Affine3d>& poses() const { return poses_; }

    DataMsgT data() const
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return latest_data_;
    }

    int numCells() const { return static_cast<int>(poses_.size()); }
    const std::string& baseFrame() const { return latest_info_.header.frame_id; }
    std::string topicName() const { return data_sub_->get_topic_name(); }

private:
    void infoCallback(const skin_msgs::msg::SkinPatchInfo::SharedPtr msg)
    {
        latest_info_ = *msg;
        poses_.resize(latest_info_.poses.size());
        for (size_t k = 0; k < latest_info_.poses.size(); ++k)
        {
            tf2::fromMsg(latest_info_.poses[k], poses_[k]);
        }
        has_info_ = true;
        info_sub_.reset();
    }

    void dataCallback(const typename DataMsgT::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            latest_data_ = *msg;
            has_data_ = true;
        }
        if (on_data_cb_) on_data_cb_(msg);
    }

private:
    rclcpp::Node::SharedPtr node_;
    bool has_info_;
    bool has_data_;

    rclcpp::Subscription<skin_msgs::msg::SkinPatchInfo>::SharedPtr info_sub_;
    typename rclcpp::Subscription<DataMsgT>::SharedPtr data_sub_;

    std::function<void(const typename DataMsgT::SharedPtr&)> on_data_cb_;

    mutable std::mutex data_mutex_;
    skin_msgs::msg::SkinPatchInfo latest_info_;
    DataMsgT latest_data_;
    std::vector<Eigen::Affine3d> poses_;
};

}  // namespace skin_utilities

#endif  // SKIN_UTILITIES_SKIN_PATCH_CLIENT_HPP_

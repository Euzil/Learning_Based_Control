#include "skin_feature_generator/skin_feature.hpp"

namespace skin_feature_generator
{

// ---
// AdaptiveRangeTracker Implementation
// ---
double AdaptiveRangeTracker::update(double dt, double val, double tau_fast,
                                    double tau_lb_up, double tau_ub_down,
                                    double min_range)
{
    if (!initialized)
    {
        val_lb = val;
        val_ub = val;
        initialized = true;
        return 0.0;
    }

    double alpha_fast = 1.0 - std::exp(-dt / tau_fast);
    double alpha_lb_up = 1.0 - std::exp(-dt / tau_lb_up);
    double alpha_ub_down = 1.0 - std::exp(-dt / tau_ub_down);

    // val_lb: snap DOWN fast on new minimum, drift UP with tau_lb_up.
    // Separate upward tau lets the floor recover quickly from noise-induced
    // dips without being slow to re-anchor when genuine air-phase minimums
    // appear.
    if (val < val_lb)
    {
        val_lb += alpha_fast * (val - val_lb);
    }
    else
    {
        val_lb += alpha_lb_up * (val - val_lb);
    }

    // val_ub: snap UP fast on new maximum, drift DOWN with tau_ub_down.
    // Long downward tau keeps the ceiling remembering contact peaks across
    // extended air phases without losing the normalization range.
    if (val > val_ub)
    {
        val_ub += alpha_fast * (val - val_ub);
    }
    else
    {
        val_ub += alpha_ub_down * (val - val_ub);
    }

    // min_range prevents noise amplification when val_lb ≈ val_ub
    // (e.g. sensor in air with no contact variation).
    double range = val_ub - val_lb;
    double denom = (range > min_range) ? range : min_range;
    double normalized = (val - val_lb) / denom;

    return std::max(0.0, std::min(1.0, normalized));
}

// ---
// SkinFeature Implementation

SkinFeature::SkinFeature(rclcpp::Node::SharedPtr node,
                         const ConfigLoader& config, const std::string& patch)
    : node_(node), config_(config)
{
    cbg_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    feature_pub_ = node_->create_publisher<skin_msgs::msg::SkinFeature>(
        patch + "/features", 10);

    client_ = std::make_shared<SkinPatchClient>(node_, patch, "/raw", cbg_);
}

bool SkinFeature::start()
{
    if (!client_->waitForReady(5.0))
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "[SkinFeature] Timeout connecting '%s'",
                     client_->topicName().c_str());
        return false;
    }

    int num_cells = client_->numCells();
    prox_tracker_.resize(num_cells);
    features_msg_.prox_distribution.resize(num_cells, 0.0);

    // Register the data callback only after the tracker is fully initialized
    // so no partial state can be seen by onData().
    client_->setOnDataCallback(
        [this](const skin_msgs::msg::SkinPatchData::SharedPtr& msg)
        { onData(msg); });

    return true;
}

void SkinFeature::onData(const skin_msgs::msg::SkinPatchData::SharedPtr& msg)
{
    rclcpp::Time stamp(msg->header.stamp);

    if (!stamp_initialized_)
    {
        last_stamp_ = stamp;
        stamp_initialized_ = true;
    }
    double dt = (stamp - last_stamp_).seconds();
    last_stamp_ = stamp;
    if (dt < 0.0) return;

    int n = client_->numCells();
    const auto& poses = client_->poses();
    Eigen::Vector3d link_acc = Eigen::Vector3d::Zero();
    features_msg_.avg_prox = 0.0;

    for (int k = 0; k < n; ++k)
    {
        double norm_val = prox_tracker_[k].update(
            dt, msg->prox[k],
            config_.norm_prox_tau_fast_sec,
            config_.norm_prox_tau_lb_up_sec,
            config_.norm_prox_tau_ub_down_sec,
            config_.norm_prox_min_range);
        features_msg_.prox_distribution[k] = norm_val;
        features_msg_.avg_prox += norm_val;

        Eigen::Vector3d cell_acc(msg->acc_x[k], msg->acc_y[k], msg->acc_z[k]);
        link_acc += poses[k].rotation() * cell_acc;
    }

    features_msg_.avg_prox /= static_cast<double>(n);
    features_msg_.contact = static_cast<int>(
        features_msg_.avg_prox > config_.binary_prox_thres);

    link_acc /= static_cast<double>(n);
    features_msg_.link_acc.x = link_acc.x();
    features_msg_.link_acc.y = link_acc.y();
    features_msg_.link_acc.z = link_acc.z();

    features_msg_.header.stamp = stamp;
    feature_pub_->publish(features_msg_);
}

}  // namespace skin_feature_generator
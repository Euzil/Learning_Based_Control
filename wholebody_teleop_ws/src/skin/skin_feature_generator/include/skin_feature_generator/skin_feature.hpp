#ifndef SKIN_FEATURE_GENERATOR_SKIN_FEATURE_H_
#define SKIN_FEATURE_GENERATOR_SKIN_FEATURE_H_

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>
#include <memory>
#include <cmath>

#include "skin_feature_generator/config_loader.hpp"

#include <skin_utilities/skin_patch_client.hpp>
#include <skin_msgs/msg/skin_feature.hpp>
#include <skin_msgs/msg/skin_patch_data.hpp>

namespace skin_feature_generator
{

// Asymmetric peak-valley tracker.
// Floor chases the signal DOWN fast / UP slow  → stays anchored to the no-contact baseline.
// Ceiling chases the signal UP fast / DOWN slow → stays anchored to the contact peak.
// Output = (val - floor) / max(ceiling - floor, min_range), clamped to [0, 1].
struct AdaptiveRangeTracker
{
    double val_lb = 0.0;
    double val_ub = 0.0;
    bool initialized = false;

    double update(double dt, double val,
                  double tau_fast,
                  double tau_lb_up,     // floor recovery upward (moderate)
                  double tau_ub_down, // ceiling relaxation downward (long)
                  double min_range);
};

class SkinFeature
{
   public:
    using SkinPatchClient = skin_utilities::SkinPatchClient<skin_msgs::msg::SkinPatchData>;

   public:
    SkinFeature(rclcpp::Node::SharedPtr node, const ConfigLoader& config,
                const std::string& patch);

    bool start();

   private:
    void onData(const skin_msgs::msg::SkinPatchData::SharedPtr& msg);

    rclcpp::Node::SharedPtr node_;
    const ConfigLoader& config_;

    rclcpp::CallbackGroup::SharedPtr cbg_;
    rclcpp::Time last_stamp_;
    bool stamp_initialized_ = false;

    rclcpp::Publisher<skin_msgs::msg::SkinFeature>::SharedPtr feature_pub_;
    skin_msgs::msg::SkinFeature features_msg_;
    std::shared_ptr<SkinPatchClient> client_;

    std::vector<AdaptiveRangeTracker> prox_tracker_;
};

}  // namespace skin_feature_generator

#endif
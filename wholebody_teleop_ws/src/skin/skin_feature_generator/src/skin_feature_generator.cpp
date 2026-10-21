#include "skin_feature_generator/skin_feature_generator.hpp"

#include <skin_utilities/utilities.hpp>

namespace skin_feature_generator
{

SkinFeatureGenerator::SkinFeatureGenerator(rclcpp::Node::SharedPtr& node, const ConfigLoader& config)
    : node_{node}, config_{config}
{
}

bool SkinFeatureGenerator::start()
{
    auto active_prefixes = skin_utilities::get_available_patches(*node_);

    for (const auto& prefix : active_prefixes)
    {
        auto feature = std::make_shared<SkinFeature>(node_, config_, prefix);
        if(!feature->start()) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to start '%s'", prefix.c_str());
            return false;
        }

        features_.push_back(feature);
    }
    return true;
}


}  // namespace skin_feature_generator
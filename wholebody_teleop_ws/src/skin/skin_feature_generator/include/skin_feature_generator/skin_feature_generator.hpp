#ifndef SKIN_FEATURE_GENERATOR_FEATURE_GENERATOR_H_
#define SKIN_FEATURE_GENERATOR_FEATURE_GENERATOR_H_

#include <rclcpp/rclcpp.hpp>

#include "skin_feature_generator/skin_feature.hpp"
#include "skin_feature_generator/config_loader.hpp"

namespace skin_feature_generator
{

class SkinFeatureGenerator
{
   public:
    SkinFeatureGenerator(rclcpp::Node::SharedPtr& node, const ConfigLoader& config);

    bool start();

   private:
    rclcpp::Node::SharedPtr node_;
    ConfigLoader config_;

    std::vector<std::shared_ptr<SkinFeature> > features_;
};

}  // namespace skin_feature_generator

#endif
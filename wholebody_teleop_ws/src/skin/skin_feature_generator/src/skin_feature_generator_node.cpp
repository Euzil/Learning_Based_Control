#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "skin_feature_generator/config_loader.hpp"
#include "skin_feature_generator/skin_feature_generator.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("skin_feature_generator_node");

    if (argc < 2)
    {
        RCLCPP_ERROR(node->get_logger(), "Usage: skin_feature_generator_node <config_file>");
        return 1;
    }
    skin_feature_generator::ConfigLoader config(argv[1]);

    // Brief spin to let patch /info topics be discovered before start().
    rclcpp::Rate discovery_rate(10);
    auto start_time = node->now();
    while (rclcpp::ok() && (node->now() - start_time).seconds() < 0.5)
    {
        rclcpp::spin_some(node);
        discovery_rate.sleep();
    }

    auto generator = std::make_shared<skin_feature_generator::SkinFeatureGenerator>(node, config);
    if (!generator->start())
    {
        RCLCPP_ERROR(node->get_logger(), "[skin_feature_generator_node] connection failed");
        rclcpp::shutdown();
        return 1;
    }

    // Each SkinFeature has its own MutuallyExclusiveCallbackGroup, so the
    // executor can process them on separate threads simultaneously.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

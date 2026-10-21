#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "pm01_ros_bridge/config_loader.hpp"
#include "pm01_ros_bridge/pm01_ros_bridge.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("pm01_ros_bridge");

    const auto config_file = node->declare_parameter<std::string>(
        "config_file", "");

    if (config_file.empty())
    {
        RCLCPP_FATAL(node->get_logger(),
                     "[pm01_ros_bridge] 'config_file' parameter is required.");
        return 1;
    }

    std::vector<std::string> joint_names;
    try
    {
        joint_names = pm01_ros_bridge::loadJointNames(config_file);
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(node->get_logger(), "%s", e.what());
        return 1;
    }

    RCLCPP_INFO(node->get_logger(),
                "[pm01_ros_bridge] loaded %zu joint names from %s",
                joint_names.size(), config_file.c_str());

    auto bridge = std::make_shared<pm01_ros_bridge::Pm01RosBridge>(node, std::move(joint_names));

    // 3 threads — one per subscription so no callback blocks another.
    rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions{}, 3};
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

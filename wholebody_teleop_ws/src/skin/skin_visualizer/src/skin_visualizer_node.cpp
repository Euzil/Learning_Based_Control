#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "skin_visualizer/skin_visualizer.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("skin_visualizer_node");

    // Config file path from command line argument
    if (argc < 2)
    {
        RCLCPP_ERROR(node->get_logger(),
            "Usage: visualizer_node <config_file>");
        return 1;
    }
    std::string config_file = argv[1];

    // Wait for 2 seconds
    rclcpp::Rate rate(10);
    auto start_time = node->now();
    while (rclcpp::ok() && (node->now() - start_time).seconds() < 2.0) 
    {
        rclcpp::spin_some(node);
        rate.sleep();
    }

    // Initialize the visualizer (passing the dereferenced node)
    auto visualizer = std::make_shared<skin_visualizer::SkinVisualizer>(*node, config_file);

    // update loop using a standard timer
    auto update_timer = node->create_wall_timer(
        std::chrono::milliseconds(50),
        [&visualizer, &node]() {
            visualizer->update(node->now());
        }
    );

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
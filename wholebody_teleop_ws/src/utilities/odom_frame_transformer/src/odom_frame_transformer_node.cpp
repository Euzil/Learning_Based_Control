#include <rclcpp/rclcpp.hpp>
#include "odom_frame_transformer/odom_frame_transformer.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("odom_frame_transformer");

    const auto input_topic    = node->declare_parameter<std::string>("input_topic",    "odom");
    const auto output_topic   = node->declare_parameter<std::string>("output_topic",   "odom_transformed");
    const auto camera_frame   = node->declare_parameter<std::string>("camera_frame",   "camera");
    const auto reference_frame= node->declare_parameter<std::string>("reference_frame","map");

    auto transformer = std::make_shared<odom_frame_transformer::OdomFrameTransformer>(
        node, input_topic, output_topic, camera_frame, reference_frame);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

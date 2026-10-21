#pragma once

#include <rclcpp/rclcpp.hpp>

#include <interface_protocol/msg/gamepad_keys.hpp>
#include <interface_protocol/msg/imu_info.hpp>
#include <interface_protocol/msg/joint_state.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <string>
#include <vector>

namespace pm01_ros_bridge
{

class Pm01RosBridge
{
public:
    Pm01RosBridge(rclcpp::Node::SharedPtr node, std::vector<std::string> joint_names);

private:
    void gamepadCallback(const interface_protocol::msg::GamepadKeys::ConstSharedPtr& msg);
    void imuCallback(const interface_protocol::msg::ImuInfo::ConstSharedPtr& msg);
    void jointStateCallback(const interface_protocol::msg::JointState::ConstSharedPtr& msg);

    rclcpp::Node::SharedPtr node_;
    const std::size_t n_joints_;

    rclcpp::Subscription<interface_protocol::msg::GamepadKeys>::SharedPtr  gamepad_sub_;
    rclcpp::Subscription<interface_protocol::msg::ImuInfo>::SharedPtr      imu_sub_;
    rclcpp::Subscription<interface_protocol::msg::JointState>::SharedPtr   joint_sub_;

    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr        joy_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr        imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

    // Pre-allocated output messages — static fields set once in constructor.
    sensor_msgs::msg::Joy        joy_msg_;
    sensor_msgs::msg::Imu        imu_msg_;
    sensor_msgs::msg::JointState joint_msg_;
};

} // namespace pm01_ros_bridge

#ifndef MUJOCO_SIMULATION_ROS_INTERFACE_HPP
#define MUJOCO_SIMULATION_ROS_INTERFACE_HPP

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <interface_protocol/msg/motion_state.hpp>
#include <interface_protocol/msg/gamepad_keys.hpp>

#include <mujoco/mujoco.h>

#include <mujoco_simulation/config_loader.h>
#include <mujoco_simulation/robot_interface.h>

namespace mujoco
{

// Simulation-level ROS interface.
// Owns the /clock publisher, joystick subscriber, and one RobotInterface per configured robot.
class RosInterface
{
public:
    RosInterface(const rclcpp::Node::SharedPtr &node,
                 std::shared_ptr<ConfigLoader> config_loader);

    bool initialize(mjModel *model, mjData *data);
    void update(const mjModel *model, mjData *data);
    void updateCamera(const mjModel *model, mjData *data, mjrContext *con);
    void torqueControl(const mjModel *model, mjData *data);

    rclcpp::Node::SharedPtr node() const { return node_; }

private:
    void motionStateTimerCallback();
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<ConfigLoader> config_loader_;

    // One entry per robot in the config
    bool has_all_cmds_{false};
    std::vector<std::unique_ptr<RobotInterface>> robot_interfaces_;

    // Simulation-global publishers
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr              clock_pub_;
    rclcpp::Publisher<interface_protocol::msg::MotionState>::SharedPtr  motion_state_pub_;
    rclcpp::Publisher<interface_protocol::msg::GamepadKeys>::SharedPtr  gamepad_keys_pub_;

    // Simulation-global subscribers
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    // Messages and timers
    interface_protocol::msg::GamepadKeys gamepad_keys_msg_;
    interface_protocol::msg::MotionState motion_state_msg_;
    rclcpp::TimerBase::SharedPtr         motion_state_timer_;
};

} // namespace mujoco

#endif // MUJOCO_SIMULATION_ROS_INTERFACE_HPP

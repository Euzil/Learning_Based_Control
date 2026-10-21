#include <mujoco_simulation/ros_interface.h>

#include <chrono>
#include <functional>

#include <rclcpp/logging.hpp>

namespace mujoco
{

RosInterface::RosInterface(const rclcpp::Node::SharedPtr &node,
                            std::shared_ptr<ConfigLoader> config_loader)
    : node_(node), config_loader_(config_loader) {}

bool RosInterface::initialize(mjModel *model, mjData *data)
{
    // --- Simulation-global publishers
    clock_pub_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);
    motion_state_pub_ = node_->create_publisher<interface_protocol::msg::MotionState>(
        "/motion/motion_state", 10);
    gamepad_keys_pub_ = node_->create_publisher<interface_protocol::msg::GamepadKeys>(
        "/hardware/gamepad_keys", 10);

    // --- Simulation-global subscribers
    using std::placeholders::_1;
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    joy_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", qos, std::bind(&RosInterface::joyCallback, this, _1));

    motion_state_msg_.current_motion_task = "joint_bridge";
    motion_state_timer_ = node_->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&RosInterface::motionStateTimerCallback, this));

    gamepad_keys_msg_.analog_states.fill(0.0);
    gamepad_keys_msg_.digital_states.fill(0);

    // --- Per-robot interfaces
    for (const auto &robot_config : config_loader_->robots()) {
        auto ri = std::make_unique<RobotInterface>();
        if (!ri->initialize(model, data, robot_config, node_)) {
            RCLCPP_ERROR(node_->get_logger(),
                "[RosInterface] Failed to initialize robot '%s'", robot_config.name.c_str());
            return false;
        }
        robot_interfaces_.push_back(std::move(ri));
    }

    RCLCPP_INFO(node_->get_logger(),
        "[RosInterface] Initialized with %zu robot(s)", robot_interfaces_.size());
    return true;
}

void RosInterface::update(const mjModel *model, mjData *data)
{
    const auto now = node_->now();

    // Clock
    auto clock_msg = std::make_unique<rosgraph_msgs::msg::Clock>();
    clock_msg->clock = rclcpp::Time(static_cast<int64_t>(data->time * 1e9), RCL_ROS_TIME);
    clock_pub_->publish(std::move(clock_msg));

    // Inputs
    gamepad_keys_msg_.header.stamp = now;
    gamepad_keys_pub_->publish(gamepad_keys_msg_);

    for (auto &ri : robot_interfaces_)
        ri->update(model, data, now);

    if(!has_all_cmds_)
    {
        // hold robots inplace
        mju_copy(data->qpos, model->qpos0, model->nq);
        mju_zero(data->qvel, model->nv);
        // check if all robots have cmd feedback
        bool all_cmds = true;
        for(auto &ri : robot_interfaces_) {
            if(!ri->hasCommand()) {
                all_cmds = false;
                break;
            }
        }
        has_all_cmds_ = all_cmds;
    }
}

void RosInterface::updateCamera(const mjModel *model, mjData *data, mjrContext *con)
{
    for (auto &ri : robot_interfaces_)
        ri->updateCamera(model, data, con);
}

void RosInterface::torqueControl(const mjModel *model, mjData *data)
{
    if(has_all_cmds_)
    {
        for (auto &ri : robot_interfaces_)
            ri->torqueControl(model, data);
    }
}

void RosInterface::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    // Note: this expects the joy stick in 'X' configuration

    if(msg->buttons.size() < 6 || msg->axes.size() < 3)
        return;

    using K = interface_protocol::msg::GamepadKeys;

    gamepad_keys_msg_.analog_states[K::LEFT_STICK_X]  = msg->axes[1];
    gamepad_keys_msg_.analog_states[K::LEFT_STICK_Y]  = msg->axes[0];
    gamepad_keys_msg_.analog_states[K::RIGHT_STICK_X] = msg->axes[4];
    gamepad_keys_msg_.analog_states[K::RIGHT_STICK_Y] = msg->axes[3];
    gamepad_keys_msg_.analog_states[K::LT]            = msg->axes[2];
    gamepad_keys_msg_.analog_states[K::RT]            = msg->axes[5];

    gamepad_keys_msg_.digital_states[K::BACK]  = msg->buttons[6];
    gamepad_keys_msg_.digital_states[K::START]  = msg->buttons[7];

    gamepad_keys_msg_.digital_states[K::A]  = msg->buttons[0];
    gamepad_keys_msg_.digital_states[K::B]  = msg->buttons[1];
    gamepad_keys_msg_.digital_states[K::X]  = msg->buttons[2];
    gamepad_keys_msg_.digital_states[K::Y]  = msg->buttons[3];
    gamepad_keys_msg_.digital_states[K::LB] = msg->buttons[4];
    gamepad_keys_msg_.digital_states[K::RB] = msg->buttons[5];

    gamepad_keys_msg_.digital_states[K::CROSS_X_UP]    = int32_t(msg->axes[7] >  0.5f);
    gamepad_keys_msg_.digital_states[K::CROSS_X_DOWN]  = int32_t(msg->axes[7] < -0.5f);
    gamepad_keys_msg_.digital_states[K::CROSS_Y_LEFT]  = int32_t(msg->axes[6] >  0.5f);
    gamepad_keys_msg_.digital_states[K::CROSS_Y_RIGHT] = int32_t(msg->axes[6] < -0.5f);
}

void RosInterface::motionStateTimerCallback()
{
    motion_state_pub_->publish(motion_state_msg_);
}

} // namespace mujoco

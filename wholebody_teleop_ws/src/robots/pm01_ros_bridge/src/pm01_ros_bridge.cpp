#include "pm01_ros_bridge/pm01_ros_bridge.hpp"

namespace pm01_ros_bridge
{

Pm01RosBridge::Pm01RosBridge(rclcpp::Node::SharedPtr node,
                              std::vector<std::string> joint_names)
    : node_{node}
    , n_joints_{joint_names.size()}
{
    // --- Pre-allocate Joy message ---
    // 12 digital buttons, 6 analog axes — sizes are fixed by the protocol.
    joy_msg_.buttons.resize(12, 0);
    joy_msg_.axes.resize(6, 0.0f);

    // --- Pre-allocate Imu message ---
    // Signal unknown covariance once via REP-145 convention (first element = -1).
    imu_msg_.orientation_covariance[0]         = -1.0;
    imu_msg_.angular_velocity_covariance[0]    = -1.0;
    imu_msg_.linear_acceleration_covariance[0] = -1.0;

    // --- Pre-allocate JointState message ---
    joint_msg_.name     = std::move(joint_names);
    joint_msg_.position.resize(n_joints_, 0.0);
    joint_msg_.velocity.resize(n_joints_, 0.0);
    joint_msg_.effort.resize(n_joints_, 0.0);

    // --- Publishers ---
    joy_pub_   = node_->create_publisher<sensor_msgs::msg::Joy>("joy", 10);
    imu_pub_   = node_->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    joint_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // Reentrant group: all three callbacks may fire concurrently on separate
    // executor threads.  Safe because each callback only touches its own msg_.
    auto cb_group = node_->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions opts;
    opts.callback_group = cb_group;

    const auto qos = rclcpp::SensorDataQoS();  // best-effort, depth 10 — drops stale msgs

    gamepad_sub_ = node_->create_subscription<interface_protocol::msg::GamepadKeys>(
        "/hardware/gamepad_keys", qos,
        [this](const interface_protocol::msg::GamepadKeys::ConstSharedPtr& msg)
        { gamepadCallback(msg); }, opts);

    imu_sub_ = node_->create_subscription<interface_protocol::msg::ImuInfo>(
        "/hardware/imu_info", qos,
        [this](const interface_protocol::msg::ImuInfo::ConstSharedPtr& msg)
        { imuCallback(msg); }, opts);

    joint_sub_ = node_->create_subscription<interface_protocol::msg::JointState>(
        "/hardware/joint_state", qos,
        [this](const interface_protocol::msg::JointState::ConstSharedPtr& msg)
        { jointStateCallback(msg); }, opts);

    RCLCPP_INFO(node_->get_logger(),
                "[Pm01RosBridge] ready — bridging gamepad, imu, joint_state (%zu joints)",
                n_joints_);
}

// ---------------------------------------------------------------------------

void Pm01RosBridge::gamepadCallback(
    const interface_protocol::msg::GamepadKeys::ConstSharedPtr& msg)
{
    joy_msg_.header.stamp = node_->get_clock()->now();

    for (std::size_t i = 0; i < 12; ++i)
        joy_msg_.buttons[i] = msg->digital_states[i];

    for (std::size_t i = 0; i < 6; ++i)
        joy_msg_.axes[i] = static_cast<float>(msg->analog_states[i]);

    joy_pub_->publish(joy_msg_);
}

void Pm01RosBridge::imuCallback(
    const interface_protocol::msg::ImuInfo::ConstSharedPtr& msg)
{
    imu_msg_.header.frame_id = msg->header.frame_id;
    imu_msg_.header.stamp = node_->get_clock()->now();

    imu_msg_.orientation.x = msg->quaternion.x;
    imu_msg_.orientation.y = msg->quaternion.y;
    imu_msg_.orientation.z = msg->quaternion.z;
    imu_msg_.orientation.w = msg->quaternion.w;

    imu_msg_.angular_velocity.x = msg->angular_velocity.x;
    imu_msg_.angular_velocity.y = msg->angular_velocity.y;
    imu_msg_.angular_velocity.z = msg->angular_velocity.z;

    imu_msg_.linear_acceleration.x = msg->linear_acceleration.x;
    imu_msg_.linear_acceleration.y = msg->linear_acceleration.y;
    imu_msg_.linear_acceleration.z = msg->linear_acceleration.z;

    imu_pub_->publish(imu_msg_);
}

void Pm01RosBridge::jointStateCallback(
    const interface_protocol::msg::JointState::ConstSharedPtr& msg)
{
    if (msg->position.size() < n_joints_ ||
        msg->velocity.size() < n_joints_ ||
        msg->torque.size()   < n_joints_)
    {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                             "[Pm01RosBridge] joint_state has fewer fields than expected "
                             "(%zu joints, got pos=%zu vel=%zu torque=%zu)",
                             n_joints_, msg->position.size(),
                             msg->velocity.size(), msg->torque.size());
        return;
    }

    joint_msg_.header.stamp = node_->get_clock()->now();
    for (std::size_t i = 0; i < n_joints_; ++i)
    {
        joint_msg_.position[i] = msg->position[i];
        joint_msg_.velocity[i] = msg->velocity[i];
        joint_msg_.effort[i]   = msg->torque[i];
    }

    joint_pub_->publish(joint_msg_);
}

} // namespace pm01_ros_bridge

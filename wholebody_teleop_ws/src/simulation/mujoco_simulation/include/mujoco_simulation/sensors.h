#pragma once

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <mujoco/mujoco.h>

#include <interface_protocol/msg/imu_info.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <skin_msgs/msg/skin_patch_data.hpp>
#include <skin_msgs/msg/skin_patch_info.hpp>
#include <skin_msgs/msg/ray_caster_data.hpp>

namespace mujoco
{

// Utility: resolves the body name attached to a sensor (via site or body).
bool sensor_find_attached_body_name(mjModel* model, int sensor_id, std::string& name);


class ImuSensor
{
public:
    bool initialize(mjModel* model, const std::string& name,
                    rclcpp::Node::SharedPtr node, const std::string& topic);
    void update(const mjData* data, const rclcpp::Time& stamp);

private:
    int quat_addr_{-1}, gyro_addr_{-1}, accel_addr_{-1};
    interface_protocol::msg::ImuInfo msg_;
    rclcpp::Publisher<interface_protocol::msg::ImuInfo>::SharedPtr pub_;
};


class FtSensor
{
public:
    bool initialize(mjModel* model, const std::string& name,
                    rclcpp::Node::SharedPtr node, const std::string& topic);
    void update(const mjData* data, const rclcpp::Time& stamp);

private:
    int force_addr_{-1}, torque_addr_{-1};
    geometry_msgs::msg::WrenchStamped msg_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_;
};


class OdomSensor
{
public:
    bool initialize(mjModel* model, const std::string& name,
                    rclcpp::Node::SharedPtr node, const std::string& topic,
                    const std::string& world_frame);
    void update(const mjData* data, const rclcpp::Time& stamp);

private:
    int pos_addr_{-1}, quat_addr_{-1}, linvel_addr_{-1}, angvel_addr_{-1};
    mjtNum q_inv_[4], linvel_b_[3], angvel_b_[3];

    nav_msgs::msg::Odometry msg_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
};


class SkinSensor
{
public:
    bool initialize(mjModel* model, const std::string& name,
                    rclcpp::Node::SharedPtr node,
                    const std::string& topic);
    void update(const mjData* data, const rclcpp::Time& stamp);

private:
    int addr_{-1};
    bool info_published_{false};
    skin_msgs::msg::SkinPatchData data_msg_;
    skin_msgs::msg::SkinPatchInfo info_msg_;
    rclcpp::Publisher<skin_msgs::msg::SkinPatchData>::SharedPtr data_pub_;
    rclcpp::Publisher<skin_msgs::msg::SkinPatchInfo>::SharedPtr info_pub_;
};


class RayCasterSensor
{
public:
    bool initialize(mjModel* model, const std::string& name,
                    rclcpp::Node::SharedPtr node, const std::string& topic);
    void update(const mjData* data, const rclcpp::Time& stamp);

private:
    int addr_{-1};
    skin_msgs::msg::RayCasterData msg_;
    rclcpp::Publisher<skin_msgs::msg::RayCasterData>::SharedPtr pub_;
};

} // namespace mujoco

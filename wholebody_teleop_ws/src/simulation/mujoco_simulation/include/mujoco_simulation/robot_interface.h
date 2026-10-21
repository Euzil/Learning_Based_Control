#pragma once

#include <array>
#include <chrono>
#include <mutex>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <interface_protocol/msg/joint_command.hpp>
#include <interface_protocol/msg/joint_state.hpp>

#include <mujoco_simulation/config_loader.h>
#include <mujoco_simulation/sensors.h>

namespace mujoco
{

// Owns all per-robot ROS state: joints, sensors, and optional depth/RGB cameras.
// One instance per entry in the config 'robots' list.
class RobotInterface
{
public:
    RobotInterface() = default;
    ~RobotInterface();

    // Non-copyable; unique_ptr ownership only.
    RobotInterface(const RobotInterface &) = delete;
    RobotInterface &operator=(const RobotInterface &) = delete;

    bool initialize(mjModel *model, mjData *data,
                    const RobotConfig &config,
                    rclcpp::Node::SharedPtr node);

    // Called every physics step (physics thread).
    void update(const mjModel *model, const mjData *data, const rclcpp::Time &stamp);

    // Called from the render thread; no-op when no camera is configured.
    void updateCamera(const mjModel *model, mjData *data, mjrContext *con);

    // PD torque controller applied before mj_step (via mjcb_control).
    void torqueControl(const mjModel *model, mjData *data);

    bool hasCommand() const { return has_command_; }
    bool hasCamera() const { return has_camera_ || rgb_pub_ != nullptr; }
    const std::string &name() const { return name_; }

private:
    bool registerJoints(mjModel *model, mjData *data, const RobotConfig &config);
    bool registerDepthCamera(mjModel *model, const std::string &sensor_name,
                              int width, int height, const std::string &topic);
    bool registerRgbCamera(mjModel *model, const DepthCameraSensorTopic &config);
    void updateRgbCamera(const mjModel *model, mjData *data, mjrContext *con);
    void jointCommandCallback(const interface_protocol::msg::JointCommand::SharedPtr msg);
    interface_protocol::msg::JointCommand command();

    std::string name_;
    rclcpp::Node::SharedPtr node_;

    bool has_command_{false};
    bool has_camera_{false};
    size_t num_joints_{0};

    // Actuated joints
    std::vector<int> joint_qpos_addr_;
    std::vector<int> joint_qvel_addr_;
    std::vector<int> joint_ctrl_addr_;
    interface_protocol::msg::JointState   joint_state_msg_;
    interface_protocol::msg::JointCommand joint_command_msg_;

    // Sensors
    std::vector<ImuSensor>       imu_sensors_;
    std::vector<FtSensor>        ft_sensors_;
    std::vector<OdomSensor>      odom_sensors_;
    std::vector<SkinSensor>      skin_sensors_;
    std::vector<RayCasterSensor> ray_caster_sensors_;

    // ROS communication
    rclcpp::Subscription<interface_protocol::msg::JointCommand>::SharedPtr  joint_cmd_sub_;
    rclcpp::Publisher<interface_protocol::msg::JointState>::SharedPtr       joint_state_pub_;

    // Protects joint_command_msg_ / has_command_ (written from ROS callback thread,
    // read from physics thread).
    std::mutex mtx_;

    // RGB camera: all buffers are owned by the render thread, independent of depth.
    mjvScene rgb_scn_{};
    mjvCamera rgb_cam_{};
    mjvOption rgb_opt_{};
    std::vector<unsigned char> rgb_buffer_;
    sensor_msgs::msg::Image rgb_img_msg_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
    std::chrono::steady_clock::time_point last_rgb_frame_{};

    // Depth camera (optional, max one per robot)
    mjvScene  scn_{};
    mjvCamera cam_{};
    mjvOption opt_{};
    std::mutex         depth_mtx_;
    bool               depth_updated_{false};
    std::vector<float> depth_buffer_;
    sensor_msgs::msg::Image depth_img_msg_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
};

} // namespace mujoco

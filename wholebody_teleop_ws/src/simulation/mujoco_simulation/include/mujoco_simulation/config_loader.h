#ifndef MUJOCO_SIMULATION_CONFIG_HPP
#define MUJOCO_SIMULATION_CONFIG_HPP

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

struct SensorTopic
{
    std::string name;
    std::string topic;
};

struct DepthCameraSensorTopic : SensorTopic
{
    unsigned int width{0};
    unsigned int height{0};
};

struct RobotConfig
{
    std::string name;

    // Joints
    std::string root_joint;
    std::string joint_state_topic;
    std::string joint_command_topic;
    std::vector<std::string> joint_names;
    std::vector<double> default_joint_stiffness;
    std::vector<double> default_joint_damping;
    std::vector<double> default_joint_position;

    // Sensors
    std::vector<SensorTopic> imu_sensors;
    std::vector<SensorTopic> ft_sensors;
    std::vector<SensorTopic> odom_sensors;
    std::vector<SensorTopic> skin_sensors;
    std::vector<SensorTopic> ray_caster_sensors;
    std::vector<DepthCameraSensorTopic> depth_camera_sensors;
    std::vector<DepthCameraSensorTopic> rgb_camera_sensors;
};

class ConfigLoader
{
public:
    ConfigLoader() = default;
    ~ConfigLoader() = default;

    ConfigLoader(const ConfigLoader &) = delete;
    ConfigLoader &operator=(const ConfigLoader &) = delete;

    bool load(const std::string &filename);

    bool isLoaded() const { return is_loaded_; }
    const std::string& scene() const { return scene_; }
    const std::vector<RobotConfig> &robots() const { return robots_; }

private:
    // Resolves relative topics: if topic starts with '/' it is used as-is,
    // otherwise "/<prefix>/<topic>" is returned (or "/<topic>" if prefix is empty).
    static std::string applyPrefix(const std::string &topic, const std::string &prefix);

    RobotConfig parseRobotNode(const YAML::Node &robot_node);
    void parseJointGroup(const YAML::Node &node, RobotConfig &robot);
    void parseSensorGroup(const YAML::Node &node, RobotConfig &robot);
    void parseSensorList(const YAML::Node &node, std::vector<SensorTopic> &out,
                         const std::string &prefix);
    void parseSensorList(const YAML::Node &node, std::vector<DepthCameraSensorTopic> &out,
                         const std::string &prefix);

    bool is_loaded_{false};
    std::string scene_;
    std::vector<RobotConfig> robots_;
};

#endif // MUJOCO_SIMULATION_CONFIG_HPP

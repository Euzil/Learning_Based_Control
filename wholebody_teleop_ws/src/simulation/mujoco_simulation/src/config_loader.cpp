#include <mujoco_simulation/config_loader.h>

#include <filesystem>
#include <sstream>
#include <stdexcept>

std::string ConfigLoader::applyPrefix(const std::string &topic, const std::string &prefix)
{
    if (topic.empty() || topic[0] == '/')
        return topic;
    if (prefix.empty())
        return "/" + topic;
    return "/" + prefix + "/" + topic;
}

bool ConfigLoader::load(const std::string &filename)
{
    YAML::Node config;
    try {
        config = YAML::LoadFile(filename);
    } catch (const YAML::Exception &e) {
        throw std::runtime_error("[ConfigLoader] YAML parse error: " + std::string(e.what()));
    }

    if (config["scene"]) {
        const std::string relative = config["scene"].as<std::string>();
        const std::filesystem::path config_dir = std::filesystem::path(filename).parent_path();
        scene_ = std::filesystem::weakly_canonical(config_dir / relative).string();
    }

    if (config["robots"]) {
        const auto &robots_node = config["robots"];
        if (!robots_node.IsSequence())
            throw std::runtime_error("[ConfigLoader] 'robots' must be a sequence");
        for (const auto &robot_node : robots_node)
            robots_.push_back(parseRobotNode(robot_node));
    } else {
        // Legacy flat format: single unnamed robot at the top level.
        if (!config["joints"])
            throw std::runtime_error("[ConfigLoader] Missing 'joints' entry in config");
        RobotConfig robot;
        parseJointGroup(config["joints"], robot);
        if (config["sensors"])
            parseSensorGroup(config["sensors"], robot);
        robots_.push_back(std::move(robot));
    }

    is_loaded_ = true;
    return true;
}

RobotConfig ConfigLoader::parseRobotNode(const YAML::Node &robot_node)
{
    RobotConfig robot;
    robot.name = robot_node["name"].as<std::string>("");
    if (!robot_node["joints"])
        throw std::runtime_error("[ConfigLoader] Robot '" + robot.name + "' missing 'joints'");
    parseJointGroup(robot_node["joints"], robot);
    if (robot_node["sensors"])
        parseSensorGroup(robot_node["sensors"], robot);
    return robot;
}

void ConfigLoader::parseJointGroup(const YAML::Node &node, RobotConfig &robot)
{
    const std::string &prefix = robot.name;

    robot.root_joint         = node["root_joint"].as<std::string>("");
    robot.joint_state_topic   = applyPrefix(node["state_topic"].as<std::string>(""),   prefix);
    robot.joint_command_topic = applyPrefix(node["command_topic"].as<std::string>(""), prefix);
    robot.joint_names         = node["names"].as<std::vector<std::string>>(std::vector<std::string>());
    robot.default_joint_stiffness = node["default_stiffness"].as<std::vector<double>>(std::vector<double>());
    robot.default_joint_damping   = node["default_damping"].as<std::vector<double>>(std::vector<double>());
    robot.default_joint_position  = node["default_position"].as<std::vector<double>>(std::vector<double>());

    const size_t n = robot.joint_names.size();

    if (robot.default_joint_position.empty())
        robot.default_joint_position.assign(n, 0.0);

    if (robot.default_joint_stiffness.size() != n ||
        robot.default_joint_damping.size()   != n ||
        robot.default_joint_position.size()  != n)
    {
        std::ostringstream ss;
        ss << "[ConfigLoader] Robot '" << robot.name << "': joint parameter size mismatch "
           << "(names=" << n
           << " stiffness=" << robot.default_joint_stiffness.size()
           << " damping="   << robot.default_joint_damping.size()
           << " position="  << robot.default_joint_position.size() << ")";
        throw std::runtime_error(ss.str());
    }
}

void ConfigLoader::parseSensorGroup(const YAML::Node &node, RobotConfig &robot)
{
    const std::string &prefix = robot.name;
    parseSensorList(node["imu_sensors"],          robot.imu_sensors,          prefix);
    parseSensorList(node["ft_sensors"],           robot.ft_sensors,           prefix);
    parseSensorList(node["odom_sensors"],         robot.odom_sensors,         prefix);
    parseSensorList(node["skin_sensors"],         robot.skin_sensors,         prefix);
    parseSensorList(node["ray_caster_sensors"],   robot.ray_caster_sensors,   prefix);
    parseSensorList(node["depth_camera_sensors"], robot.depth_camera_sensors, prefix);
    parseSensorList(node["rgb_camera_sensors"], robot.rgb_camera_sensors, prefix);
}

void ConfigLoader::parseSensorList(const YAML::Node &node, std::vector<SensorTopic> &out,
                                    const std::string &prefix)
{
    if (!node || !node.IsSequence())
        return;
    for (const auto &s : node) {
        SensorTopic st;
        st.name  = s["name"].as<std::string>();
        st.topic = applyPrefix(s["topic"].as<std::string>(), prefix);
        out.push_back(st);
    }
}

void ConfigLoader::parseSensorList(const YAML::Node &node, std::vector<DepthCameraSensorTopic> &out,
                                    const std::string &prefix)
{
    if (!node || !node.IsSequence())
        return;
    for (const auto &s : node) {
        DepthCameraSensorTopic st;
        st.name   = s["name"].as<std::string>();
        st.topic  = applyPrefix(s["topic"].as<std::string>(), prefix);
        st.width  = s["width"].as<unsigned int>();
        st.height = s["height"].as<unsigned int>();
        out.push_back(st);
    }
}

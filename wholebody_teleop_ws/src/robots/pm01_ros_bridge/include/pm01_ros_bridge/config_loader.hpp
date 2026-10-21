#pragma once

#include <stdexcept>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace pm01_ros_bridge
{

// Reads robots[0].joints.names from a pm01_description-style YAML config.
inline std::vector<std::string> loadJointNames(const std::string& filepath)
{
    YAML::Node root;
    try
    {
        root = YAML::LoadFile(filepath);
    }
    catch (const YAML::BadFile&)
    {
        throw std::runtime_error("pm01_ros_bridge: cannot open config file: " + filepath);
    }
    catch (const YAML::ParserException& e)
    {
        throw std::runtime_error("pm01_ros_bridge: malformed YAML in " + filepath +
                                 " -> " + e.what());
    }

    const auto robots = root["robots"];
    if (!robots || !robots.IsSequence() || robots.size() == 0)
        throw std::runtime_error("pm01_ros_bridge: 'robots' sequence missing or empty in " +
                                 filepath);

    const auto names_node = robots[0]["joints"]["names"];
    if (!names_node || !names_node.IsSequence())
        throw std::runtime_error(
            "pm01_ros_bridge: robots[0].joints.names missing or not a sequence in " + filepath);

    std::vector<std::string> names;
    names.reserve(names_node.size());
    for (const auto& n : names_node)
        names.push_back(n.as<std::string>());

    return names;
}

} // namespace pm01_ros_bridge

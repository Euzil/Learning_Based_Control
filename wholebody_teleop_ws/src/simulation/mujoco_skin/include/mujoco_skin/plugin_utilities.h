#ifndef MUJOCO_PLUGIN_SENSOR_SKIN_PLUGIN_UTILITITES_H_
#define MUJOCO_PLUGIN_SENSOR_SKIN_PLUGIN_UTILITITES_H_

#include <array>
#include <functional>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <iostream>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

namespace mujoco::plugin::sensor
{

// Checks that a plugin config attribute exists.
bool check_attr(const std::string& input);

std::vector<std::string> read_string_vector(const std::string& input);

// Converts a string into a numeric vector
template <typename T>
inline std::vector<T> read_vector(const std::string& input)
{
    std::vector<T> output;
    std::stringstream ss(input);
    std::string item;
    char delim = ' ';
    while (getline(ss, item, delim))
    {
        check_attr(item);
        output.push_back(strtod(item.c_str(), nullptr));
    }
    return output;
}

template <typename T>
inline std::vector<T> read_vector(const std::string& input, int min_num)
{
    std::vector<T> output;
    std::stringstream ss(input);
    std::string item;
    char delim = ' ';
    while (getline(ss, item, delim))
    {
        check_attr(item);
        output.push_back(strtod(item.c_str(), nullptr));
    }
    if (output.empty())
        return output;
    if (output.size() < min_num)
        output.clear();
    return output;
}

}  // namespace mujoco::plugin::sensor

#endif
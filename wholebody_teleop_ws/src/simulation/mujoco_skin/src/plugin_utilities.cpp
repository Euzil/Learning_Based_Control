#include <mujoco_skin/plugin_utilities.h>

namespace mujoco::plugin::sensor
{

bool check_attr(const std::string& input)
{
    char* end;
    std::string value = input;
    value.erase(std::remove_if(value.begin(), value.end(), isspace),
                value.end());
    strtod(value.c_str(), &end);
    return end == value.data() + value.size();
}

std::vector<std::string> read_string_vector(const std::string& input)
{
    std::vector<std::string> output;
    std::stringstream ss(input);
    std::string item;
    char delim = ' ';
    while (getline(ss, item, delim))
    {
        if (item.empty())
        {
            continue;
        }
        check_attr(item);
        output.push_back(item);
    }
    return output;
}

}  // namespace mujoco::plugin::sensor
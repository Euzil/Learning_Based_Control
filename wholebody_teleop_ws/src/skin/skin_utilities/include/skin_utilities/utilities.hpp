#include <vector>
#include <string>
#include <algorithm>
#include <map>

#include <rclcpp/node.hpp>

namespace skin_utilities
{

/**
 * @brief Scans the ROS 2 graph for active skin patches based on the latched
 * /info topic.
 * * @param node Reference to the node performing the query
 * @param ns Optional namespace filter (e.g. "/pm01")
 * @return std::vector<std::string> Sorted vector of base topic prefixes
 */
inline std::vector<std::string> get_available_patches(
    const rclcpp::Node& node, const std::string& ns = "")
{
    std::vector<std::string> patch_prefixes;

    // Ensure the namespace starts with a '/' for matching fully qualified ROS 2
    // topics
    std::string target_ns = ns;
    if (!target_ns.empty() && target_ns[0] != '/')
    {
        target_ns = "/" + target_ns;
    }

    // Query the ROS 2 graph for all known topics and their message types
    auto topic_names_and_types = node.get_topic_names_and_types();

    for (const auto& [topic_name, topic_types] : topic_names_and_types)
    {
        // Filter by namespace (if provided)
        if (!target_ns.empty() && topic_name.find(target_ns) != 0)
        {
            continue;
        }

        // Check if the topic ends with "/info"
        const std::string suffix = "/info";
        if (topic_name.length() >= suffix.length() &&
            topic_name.compare(topic_name.length() - suffix.length(),
                               suffix.length(), suffix) == 0)
        {
            // Verify the datatype matches skin type
            auto it = std::find(topic_types.begin(), topic_types.end(),
                                "skin_msgs/msg/SkinPatchInfo");
            if (it != topic_types.end())
            {
                // Extract the base prefix by slicing off the "/info" string
                std::string prefix =
                    topic_name.substr(0, topic_name.length() - suffix.length());
                patch_prefixes.push_back(prefix);
            }
        }
    }

    // Sort alphabetically to guarantee consistent ordering
    std::sort(patch_prefixes.begin(), patch_prefixes.end());
    return patch_prefixes;
}

}  // namespace skin_utilities
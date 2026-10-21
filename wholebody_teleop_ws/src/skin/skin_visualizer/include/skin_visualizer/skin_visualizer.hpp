#ifndef SKIN_VISUALIZER_SKIN_VISUALIZER_H_
#define SKIN_VISUALIZER_SKIN_VISUALIZER_H_

#include <rclcpp/rclcpp.hpp>

#include <tf2/exceptions.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <skin_msgs/msg/skin_feature.hpp>

#include <skin_utilities/skin_patch_client.hpp>

namespace skin_visualizer
{
/**
 * @brief Subscribe to skin feature generator and display information visually.
 * A merged pointcloud for cells (pos+normal) and proximity (color encoded)
 */
class SkinVisualizer
{
   public:
    using SkinPatchClient = skin_utilities::SkinPatchClient<skin_msgs::msg::SkinFeature>;

   public:
    using CellPoint = pcl::PointNormal;
    using CellCloud = pcl::PointCloud<CellPoint>;
    using DistancePoint = pcl::PointXYZRGB;
    using DistanceCloud = pcl::PointCloud<DistancePoint>;
    using VectorRGB = Eigen::Vector3i;

   public:
    SkinVisualizer(rclcpp::Node& node, const std::string& config_file);

    void update(const rclcpp::Time& time);

   private:
    void loadParameters(const std::string& config_file);

    void mergePatches(const rclcpp::Time& time);

    double normalizedProximityToDistance(double normalized_prox) const;

    void scanForNewPatches();

   private:
    rclcpp::Node& node_;
    std::string target_frame_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // point clouds
    CellCloud::Ptr merged_cell_cloud_;      // cell locations
    DistanceCloud::Ptr merged_dist_cloud_;  // proximity locations
    CellCloud::Ptr contact_cloud_;          // active cell proximity only

    // skin connection
    std::vector<std::shared_ptr<SkinPatchClient>> clients_;

    // publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        distance_cloud_pub_;  // all cells distances
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        cell_cloud_pub_;  // all cells position
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        contact_cloud_pub_;  // active only

    // timer to check for new patches
    rclcpp::TimerBase::SharedPtr discovery_timer_;
    std::set<std::string> known_patches_names_;

    // configurable parameters (loaded from ROS2 params)
   private:
    double max_dist_;
    double prox_transform_exp_;
    double prox_transform_off_;
    double contact_threshold_;
    double sensor_radius_;

    // Force sensor layout (3 sensors at 120° apart)
    static constexpr double FORCE_ANGLES[3] = {
        M_PI / 4.0,                     // 45°
        M_PI / 4.0 - 2.0 * M_PI / 3.0, // -75° (285°)
        M_PI / 4.0 + 2.0 * M_PI / 3.0  // 165°
    };

    // Colors
    VectorRGB prox_idle_color_;
    VectorRGB prox_contact_color_;
    VectorRGB force_idle_color_;
    VectorRGB force_active_color_;
};

}  // namespace skin_visualizer

#endif
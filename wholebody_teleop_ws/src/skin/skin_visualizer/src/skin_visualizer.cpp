#include "skin_visualizer/skin_visualizer.hpp"

#include <skin_utilities/utilities.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/common/transforms.h>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <cmath>

namespace skin_visualizer
{

SkinVisualizer::SkinVisualizer(rclcpp::Node& node, const std::string& config_file)
    : node_{node}, target_frame_{"base_link"}
{
    // Load tunable parameters from YAML config file
    loadParameters(config_file);

    // Initialize TF2
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_.get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize PCL Cloud Pointers
    merged_cell_cloud_ = std::make_shared<CellCloud>();
    merged_dist_cloud_ = std::make_shared<DistanceCloud>();
    contact_cloud_ = std::make_shared<CellCloud>();

    // Initialize Publishers
    distance_cloud_pub_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>(
        "~/distance_cloud", 10);
    cell_cloud_pub_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>(
        "~/cell_cloud", 10);
    contact_cloud_pub_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>(
        "~/contact_cloud", 10);

    // --- check for new patches every 4 seconds ---
    discovery_timer_ = node_.create_wall_timer(
        std::chrono::seconds(4),
        std::bind(&SkinVisualizer::scanForNewPatches, this)
    );
    scanForNewPatches();

    RCLCPP_INFO(node_.get_logger(), "SkinVisualizer initialized.");
}

void SkinVisualizer::loadParameters(const std::string& config_file)
{
    YAML::Node config;
    try {
        config = YAML::LoadFile(config_file);
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(node_.get_logger(), "Failed to load config '%s': %s",
                     config_file.c_str(), e.what());
        throw;
    }

    max_dist_           = config["max_dist"].as<double>(0.04);
    prox_transform_exp_ = config["prox_transform_exp"].as<double>(-9.59);
    prox_transform_off_ = config["prox_transform_off"].as<double>(-0.016);
    contact_threshold_  = config["contact_threshold"].as<double>(0.5);
    sensor_radius_      = config["sensor_radius"].as<double>(0.008);

    auto to_rgb = [&](const std::string& key, VectorRGB def) -> VectorRGB {
        if (!config[key]) return def;
        auto v = config[key].as<std::vector<int>>();
        return VectorRGB(v[0], v[1], v[2]);
    };
    prox_idle_color_    = to_rgb("prox_idle_color",    VectorRGB(0, 255, 0));
    prox_contact_color_ = to_rgb("prox_contact_color", VectorRGB(255, 0, 0));
    force_idle_color_   = to_rgb("force_idle_color",   VectorRGB(200, 200, 200));
    force_active_color_ = to_rgb("force_active_color", VectorRGB(255, 0, 0));

    RCLCPP_INFO(node_.get_logger(), "Loaded config: %s", config_file.c_str());
    RCLCPP_INFO(node_.get_logger(), "Config: max_dist=%.3f prox_exp=%.2f prox_off=%.3f"
        " contact_thres=%.3f",
        max_dist_, prox_transform_exp_, prox_transform_off_,
        contact_threshold_);
}

void SkinVisualizer::update(const rclcpp::Time& time)
{    
    // Merge and transform all patches
    mergePatches(time);

    // Publish Cell Cloud
    if (!merged_cell_cloud_->empty())
    {
        sensor_msgs::msg::PointCloud2 cell_msg;
        pcl::toROSMsg(*merged_cell_cloud_, cell_msg);
        cell_msg.header.frame_id = target_frame_;
        cell_msg.header.stamp = time;
        cell_cloud_pub_->publish(cell_msg);
    }

    // Publish Distance Cloud (Colorized)
    if (!merged_dist_cloud_->empty())
    {
        sensor_msgs::msg::PointCloud2 dist_msg;
        pcl::toROSMsg(*merged_dist_cloud_, dist_msg);
        dist_msg.header.frame_id = target_frame_;
        dist_msg.header.stamp = time;
        distance_cloud_pub_->publish(dist_msg);
    }

    // Publish Contact Cloud
    if (!contact_cloud_->empty())
    {
        sensor_msgs::msg::PointCloud2 contact_msg;
        pcl::toROSMsg(*contact_cloud_, contact_msg);
        contact_msg.header.frame_id = target_frame_;
        contact_msg.header.stamp = time;
        contact_cloud_pub_->publish(contact_msg);
    }
}

void SkinVisualizer::mergePatches(const rclcpp::Time& /*time*/)
{
    // Clear the accumulated clouds for this frame
    merged_cell_cloud_->clear();
    merged_dist_cloud_->clear();
    contact_cloud_->clear();

    for (const auto& client : clients_)
    {
        // Skip if the client hasn't received its static geometry yet
        if (!client->isReady())
        {
            RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
                        "Waiting for info topic '%s'",
                        client->topicName().c_str());
            continue;
        }

        const auto& poses = client->poses();
        auto data = client->data();

        // --- GET THE TRANSFORM ---
        const auto& source_frame = client->baseFrame();
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            // Lookup transform from the patch frame to the target frame
            transform_stamped = tf_buffer_->lookupTransform(
                target_frame_, source_frame, rclcpp::Time(0),
                rclcpp::Duration::from_seconds(0.1));
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
                                 "Could not transform '%s' to '%s': %s",
                                 source_frame.c_str(), target_frame_.c_str(),
                                 ex.what());
            continue;
        }

        // Convert the ROS transform to an Eigen matrix
        Eigen::Isometry3d eigen_transform =
            tf2::transformToEigen(transform_stamped);

        // --- BUILD LOCAL CLOUDS ---
        CellCloud local_cell_cloud;
        DistanceCloud local_dist_cloud;

        size_t num_cells = poses.size();
        local_cell_cloud.reserve(num_cells);
        local_dist_cloud.reserve(num_cells * 4);  // 4 points per cell

        CellPoint cp;
        DistancePoint dp;
        for (size_t i = 0; i < num_cells; ++i)
        {
            // Use pre-normalized values from skin_real [0, 1]
            double norm_prox = (i < data.prox_distribution.size()) ? data.prox_distribution[i] : 0.0;
            double norm_f1 = 0.0; // (i < data.force1.size()) ? data.force1[i] : 0.0;
            double norm_f2 = 0.0; // (i < data.force2.size()) ? data.force2[i] : 0.0;
            double norm_f3 = 0.0; // (i < data.force3.size()) ? data.force3[i] : 0.0;

            // Build Cell Point (Position + Normal)
            const auto& pose = poses[i];
            auto t = pose.translation();
            auto R = pose.rotation();

            cp.x = t.x(); cp.y = t.y(); cp.z = t.z();
            cp.normal_x = R(0, 2); cp.normal_x = R(1, 2); cp.normal_x = R(2, 2);
            local_cell_cloud.push_back(cp);

            // --- PROX POINT (color-encoded normalized proximity) ---
            float prox_factor = static_cast<float>(
                std::min(1.0, std::max(0.0, norm_prox)));
            double dist = normalizedProximityToDistance(norm_prox);
            dp.r = prox_factor * prox_contact_color_[0] + (1.f - prox_factor) * prox_idle_color_[0];
            dp.g = prox_factor * prox_contact_color_[1] + (1.f - prox_factor) * prox_idle_color_[1];
            dp.b = prox_factor * prox_contact_color_[2] + (1.f - prox_factor) * prox_idle_color_[2];
            dp.x = t.x() + dist * R(0, 2);
            dp.y = t.y() + dist * R(1, 2);
            dp.z = t.z() + dist * R(2, 2);
            local_dist_cloud.push_back(dp);

            // --- THREE FORCE SENSOR POINTS (120° apart, individual normalized forces) ---
            double forces[3] = {norm_f1, norm_f2, norm_f3};
            for (int j = 0; j < 3; ++j)
            {
                float force_factor = static_cast<float>(
                    std::min(1.0, std::max(0.0, forces[j])));
                uint8_t fr = force_factor * force_active_color_[0] + (1.f - force_factor) * force_idle_color_[0];
                uint8_t fg = force_factor * force_active_color_[1] + (1.f - force_factor) * force_idle_color_[1];
                uint8_t fb = force_factor * force_active_color_[2] + (1.f - force_factor) * force_idle_color_[2];

                double angle = FORCE_ANGLES[j];
                Eigen::Vector3d offset = sensor_radius_ *
                    (std::cos(angle) * R.col(0) + std::sin(angle) * R.col(1));
                dp.x = t.x() + offset.x();
                dp.y = t.y() + offset.y();
                dp.z = t.z() + offset.z();
                dp.r = fr;  dp.g = fg;  dp.b = fb;
                local_dist_cloud.push_back(dp);
            }
        }

        // --- APPLY TRANSFORMATIONS ---
        CellCloud transf_cell_cloud;
        DistanceCloud transf_dist_cloud;

        // Apply Eigen transform to the points AND the normals simultaneously
        pcl::transformPointCloudWithNormals(local_cell_cloud, transf_cell_cloud,
                                            eigen_transform.matrix());
        pcl::transformPointCloud(local_dist_cloud, transf_dist_cloud,
                                 eigen_transform.matrix());

        // --- MERGE INTO GLOBAL CLOUDS ---
        *merged_cell_cloud_ += transf_cell_cloud;
        *merged_dist_cloud_ += transf_dist_cloud;

        // --- EXTRACT ACTIVE CONTACTS (using normalized prox threshold) ---
        for (size_t i = 0; i < num_cells; ++i)
        {
            double norm_prox = (i < data.prox_distribution.size()) ? data.prox_distribution[i] : 0.0;
            if (norm_prox > contact_threshold_)
            {
                contact_cloud_->push_back(transf_cell_cloud[i]);
            }
        }
    }
}

double SkinVisualizer::normalizedProximityToDistance(double normalized_prox) const
{
    double clamped = std::min(1.0, std::max(0.0, normalized_prox));

    return max_dist_ * (1.0 - clamped);
}

void SkinVisualizer::scanForNewPatches()
{
    auto active_prefixes = skin_utilities::get_available_patches(node_);
    for (const auto& prefix : active_prefixes)
    {
        // If this prefix is not in our known list, add it!
        if (known_patches_names_.find(prefix) == known_patches_names_.end())
        {
            RCLCPP_INFO(node_.get_logger(), "Discovered new skin patch: %s", prefix.c_str());
            
            // Create a new client. 
            // Note: node_.shared_from_this() gets a SharedPtr from the Node reference
            auto new_client = std::make_shared<SkinPatchClient>(
                node_.shared_from_this(),
                prefix,
                "/features"
            );
            
            clients_.push_back(new_client);
            known_patches_names_.insert(prefix);
        }
    }
}

}  // namespace skin_visualizer
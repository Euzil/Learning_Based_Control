#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace odom_frame_transformer
{

/**
 * @brief OdomFrameTransformer
 * 
 * 
 * 
 */
class OdomFrameTransformer
{
public:
    OdomFrameTransformer(rclcpp::Node::SharedPtr node,
                         const std::string& input_topic,
                         const std::string& output_topic,
                         const std::string& camera_frame,
                         const std::string& reference_frame);

private:
    void lookupStaticTransform();
    void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);

    rclcpp::Node::SharedPtr node_;

    std::string camera_frame_;
    std::string reference_frame_;

    // Static TF: reference_frame <- camera_frame
    bool tf_ready_{false};
    Eigen::Isometry3d T_ref_cam_{Eigen::Isometry3d::Identity()};
    Eigen::Isometry3d T_odom_cam_{Eigen::Isometry3d::Identity()};
    Eigen::Isometry3d T_odom_ref_{Eigen::Isometry3d::Identity()};
        
    // Inital transform
    size_t cnt_{0};
    bool has_init_{false};
    Eigen::Isometry3d T_init_{Eigen::Isometry3d::Identity()};

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr  pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;

    // Pre-allocated output messages — static fields set once in constructor.
    nav_msgs::msg::Odometry               odom_msg_;
    geometry_msgs::msg::TransformStamped  tf_msg_;

    tf2_ros::Buffer                tf_buffer_;
    tf2_ros::TransformListener     tf_listener_;
    tf2_ros::TransformBroadcaster  tf_broadcaster_;
};

} // namespace odom_frame_transformer

#include "odom_frame_transformer/odom_frame_transformer.hpp"

#include <tf2_eigen/tf2_eigen.hpp>
#include <stdexcept>

namespace odom_frame_transformer
{

OdomFrameTransformer::OdomFrameTransformer(rclcpp::Node::SharedPtr node,
                                           const std::string& input_topic,
                                           const std::string& output_topic,
                                           const std::string& camera_frame,
                                           const std::string& reference_frame)
    : node_{node}
    , camera_frame_{camera_frame}
    , reference_frame_{reference_frame}
    , tf_buffer_{node_->get_clock()}
    , tf_listener_{tf_buffer_}
    , tf_broadcaster_{node_}
{
    // Static fields that never change between callbacks.
    // Convention: odom is the fixed parent, reference_frame (base_link) is the child.
    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id  = reference_frame;
    tf_msg_.header.frame_id   = "odom";
    tf_msg_.child_frame_id    = reference_frame;

    pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(output_topic, 10);

    // Attempt lookup immediately; fall back to first-message lookup if TF not yet available.
    lookupStaticTransform();

    sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        input_topic, rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr& msg) { odomCallback(msg); });

    RCLCPP_INFO(node_->get_logger(),
                "[OdomFrameTransformer] %s -> %s  |  TF %s->%s  |  tf_ready=%s",
                input_topic.c_str(), output_topic.c_str(),
                camera_frame_.c_str(), reference_frame_.c_str(),
                tf_ready_ ? "true" : "false (will retry)");
}

void OdomFrameTransformer::lookupStaticTransform()
{
    try
    {
        // Wait up to 5 s for the static TF to become available.
        const auto stamped = tf_buffer_.lookupTransform(
            reference_frame_, camera_frame_,
            tf2::TimePointZero,
            tf2::durationFromSec(5.0));

        T_ref_cam_ = tf2::transformToEigen(stamped);
        tf_ready_  = true;

        RCLCPP_INFO(node_->get_logger(),
                    "[OdomFrameTransformer] static TF '%s'->'%s' acquired: "
                    "t=[%.4f, %.4f, %.4f]",
                    camera_frame_.c_str(), reference_frame_.c_str(),
                    T_ref_cam_.translation().x(), T_ref_cam_.translation().y(), T_ref_cam_.translation().z());
    }
    catch (const tf2::TransformException& e)
    {
        RCLCPP_WARN(node_->get_logger(),
                    "[OdomFrameTransformer] TF not yet available: %s  (will retry per message)",
                    e.what());
    }
}

void OdomFrameTransformer::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
    if (!tf_ready_)
    {
        lookupStaticTransform();
        if (!tf_ready_) return;
    }

    // --- Transform pose ---

    // cam wrt. odom frame
    const auto& p_cam = msg->pose.pose.position;
    const auto& q_cam = msg->pose.pose.orientation;
    T_odom_cam_.translation() << p_cam.x, p_cam.y, p_cam.z;
    T_odom_cam_.linear() = Eigen::Quaterniond(q_cam.w, q_cam.x, q_cam.y, q_cam.z).toRotationMatrix();

    // ref wrt. odom frame
    T_odom_ref_ = T_odom_cam_ * T_ref_cam_.inverse() * T_init_;

    // calibrate offsets after 0.5 sec
    if(!has_init_ && cnt_ > 100)
    {
        has_init_ = true;
        T_init_ = T_odom_ref_.inverse();
        RCLCPP_WARN_STREAM(node_->get_logger(),
                    "[OdomFrameTransformer] TF init: eul=" << T_odom_ref_.rotation().eulerAngles(0,1,2).transpose());
    }
    cnt_++;

    // --- Transform twist ---
    // The twist is in the child frame (body frame). To re-express it in the
    // reference frame we apply the Adjoint of T_ref_cam:
    //   omega_ref = R_ref_cam * omega_cam
    //   v_ref     = R_ref_cam * v_cam + p_ref_cam x omega_ref   (cross-product coupling)
    const auto& lin_vel_msg = msg->twist.twist.linear;
    const auto& ang_vel_msg = msg->twist.twist.angular;

    const Eigen::Vector3d lin_vel_cam{lin_vel_msg.x, lin_vel_msg.y, lin_vel_msg.z};
    const Eigen::Vector3d ang_vel_cam{ang_vel_msg.x, ang_vel_msg.y, ang_vel_msg.z};

    const Eigen::Vector3d ang_vel_ref = T_ref_cam_.rotation() * ang_vel_cam;
    const Eigen::Vector3d lin_vel_ref = T_ref_cam_.rotation() * lin_vel_cam + T_ref_cam_.translation().cross(ang_vel_ref);

    // --- Fill and publish odometry ---
    odom_msg_.header.stamp = node_->get_clock()->now();

    auto Q_odom_ref = Eigen::Quaterniond(T_odom_ref_.rotation());
    odom_msg_.pose.pose.position.x    = T_odom_ref_.translation().x();
    odom_msg_.pose.pose.position.y    = T_odom_ref_.translation().y();
    odom_msg_.pose.pose.position.z    = T_odom_ref_.translation().z();
    odom_msg_.pose.pose.orientation.w = Q_odom_ref.w();
    odom_msg_.pose.pose.orientation.x = Q_odom_ref.x();
    odom_msg_.pose.pose.orientation.y = Q_odom_ref.y();
    odom_msg_.pose.pose.orientation.z = Q_odom_ref.z();

    odom_msg_.twist.twist.linear.x  = lin_vel_ref.x();
    odom_msg_.twist.twist.linear.y  = lin_vel_ref.y();
    odom_msg_.twist.twist.linear.z  = lin_vel_ref.z();
    odom_msg_.twist.twist.angular.x = ang_vel_ref.x();
    odom_msg_.twist.twist.angular.y = ang_vel_ref.y();
    odom_msg_.twist.twist.angular.z = ang_vel_ref.z();

    odom_msg_.pose.covariance  = msg->pose.covariance;
    odom_msg_.twist.covariance = msg->twist.covariance;

    pub_->publish(odom_msg_);

    // --- Broadcast reference_frame -> odom TF ---
    tf_msg_.header.stamp             = node_->get_clock()->now();
    tf_msg_.transform.translation.x  = T_odom_ref_.translation().x();
    tf_msg_.transform.translation.y  = T_odom_ref_.translation().y();
    tf_msg_.transform.translation.z  = T_odom_ref_.translation().z();
    tf_msg_.transform.rotation.w     = Q_odom_ref.w();
    tf_msg_.transform.rotation.x     = Q_odom_ref.x();
    tf_msg_.transform.rotation.y     = Q_odom_ref.y();
    tf_msg_.transform.rotation.z     = Q_odom_ref.z();
    tf_broadcaster_.sendTransform(tf_msg_);
}

} // namespace odom_frame_transformer

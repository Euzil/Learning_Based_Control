#include <mujoco_simulation/sensors.h>
#include <rclcpp/logging.hpp>

namespace mujoco
{

bool sensor_find_attached_body_name(mjModel* model, int sensor_id, std::string& name)
{
    int obj_id   = model->sensor_objid[sensor_id];
    int obj_type = model->sensor_objtype[sensor_id];

    int body_id = -1;
    if (obj_type == mjOBJ_SITE)
        body_id = model->site_bodyid[obj_id];
    else if (obj_type == mjOBJ_BODY)
        body_id = obj_id;

    if (body_id < 0)
        return false;

    name = std::string(mj_id2name(model, mjOBJ_BODY, body_id));
    return true;
}

// -----------------------------------------------------------------------------
// ImuSensor

bool ImuSensor::initialize(mjModel* model, const std::string& name,
                            rclcpp::Node::SharedPtr node, const std::string& topic)
{
    int q_id = mj_name2id(model, mjOBJ_SENSOR, (name + "_quat").c_str());
    int g_id = mj_name2id(model, mjOBJ_SENSOR, (name + "_gyro").c_str());
    int a_id = mj_name2id(model, mjOBJ_SENSOR, (name + "_accel").c_str());

    if (q_id < 0 || g_id < 0 || a_id < 0) {
        RCLCPP_ERROR(node->get_logger(), "[ImuSensor] Missing components for '%s'", name.c_str());
        return false;
    }
    if (model->sensor_objid[q_id] != model->sensor_objid[g_id] ||
        model->sensor_objid[g_id] != model->sensor_objid[a_id]) {
        RCLCPP_ERROR(node->get_logger(), "[ImuSensor] Body mismatch for '%s'", name.c_str());
        return false;
    }

    std::string frame_id;
    if (!sensor_find_attached_body_name(model, q_id, frame_id)) return false;

    quat_addr_  = model->sensor_adr[q_id];
    gyro_addr_  = model->sensor_adr[g_id];
    accel_addr_ = model->sensor_adr[a_id];
    msg_.header.frame_id = frame_id;
    pub_ = node->create_publisher<interface_protocol::msg::ImuInfo>(topic, 10);

    RCLCPP_INFO(node->get_logger(), "[ImuSensor] Registered: %s", name.c_str());
    return true;
}

void ImuSensor::update(const mjData* data, const rclcpp::Time& stamp)
{
    msg_.header.stamp = stamp;

    msg_.quaternion.w = data->sensordata[quat_addr_ + 0];
    msg_.quaternion.x = data->sensordata[quat_addr_ + 1];
    msg_.quaternion.y = data->sensordata[quat_addr_ + 2];
    msg_.quaternion.z = data->sensordata[quat_addr_ + 3];

    msg_.angular_velocity.x = data->sensordata[gyro_addr_ + 0];
    msg_.angular_velocity.y = data->sensordata[gyro_addr_ + 1];
    msg_.angular_velocity.z = data->sensordata[gyro_addr_ + 2];

    msg_.linear_acceleration.x = data->sensordata[accel_addr_ + 0];
    msg_.linear_acceleration.y = data->sensordata[accel_addr_ + 1];
    msg_.linear_acceleration.z = data->sensordata[accel_addr_ + 2];

    pub_->publish(msg_);
}

// -----------------------------------------------------------------------------
// FtSensor

bool FtSensor::initialize(mjModel* model, const std::string& name,
                           rclcpp::Node::SharedPtr node, const std::string& topic)
{
    int f_id = mj_name2id(model, mjOBJ_SENSOR, (name + "_force").c_str());
    int t_id = mj_name2id(model, mjOBJ_SENSOR, (name + "_torque").c_str());

    if (f_id < 0 || t_id < 0 || model->sensor_objid[f_id] != model->sensor_objid[t_id]) {
        RCLCPP_ERROR(node->get_logger(), "[FtSensor] Missing or mismatched components for '%s'", name.c_str());
        return false;
    }

    std::string frame_id;
    if (!sensor_find_attached_body_name(model, f_id, frame_id)) return false;

    force_addr_  = model->sensor_adr[f_id];
    torque_addr_ = model->sensor_adr[t_id];
    msg_.header.frame_id = frame_id;
    pub_ = node->create_publisher<geometry_msgs::msg::WrenchStamped>(topic, 10);

    RCLCPP_INFO(node->get_logger(), "[FtSensor] Registered: %s", name.c_str());
    return true;
}

void FtSensor::update(const mjData* data, const rclcpp::Time& stamp)
{
    msg_.header.stamp = stamp;

    msg_.wrench.force.x  = data->sensordata[force_addr_ + 0];
    msg_.wrench.force.y  = data->sensordata[force_addr_ + 1];
    msg_.wrench.force.z  = data->sensordata[force_addr_ + 2];

    msg_.wrench.torque.x = data->sensordata[torque_addr_ + 0];
    msg_.wrench.torque.y = data->sensordata[torque_addr_ + 1];
    msg_.wrench.torque.z = data->sensordata[torque_addr_ + 2];

    pub_->publish(msg_);
}

// -----------------------------------------------------------------------------
// OdomSensor

bool OdomSensor::initialize(mjModel* model, const std::string& name,
                             rclcpp::Node::SharedPtr node, const std::string& topic,
                             const std::string& world_frame)
{
    int p_id = mj_name2id(model, mjOBJ_SENSOR, (name + "_pos").c_str());
    int q_id = mj_name2id(model, mjOBJ_SENSOR, (name + "_quat").c_str());
    int l_id = mj_name2id(model, mjOBJ_SENSOR, (name + "_linvel").c_str());
    int a_id = mj_name2id(model, mjOBJ_SENSOR, (name + "_angvel").c_str());

    if (p_id < 0 || q_id < 0 || l_id < 0 || a_id < 0) {
        RCLCPP_ERROR(node->get_logger(), "[OdomSensor] Missing components for '%s'", name.c_str());
        return false;
    }

    int obj_id = model->sensor_objid[p_id];
    if (model->sensor_objid[q_id] != obj_id ||
        model->sensor_objid[l_id] != obj_id ||
        model->sensor_objid[a_id] != obj_id) {
        RCLCPP_ERROR(node->get_logger(), "[OdomSensor] Body mismatch for '%s'", name.c_str());
        return false;
    }

    std::string frame_id;
    if (!sensor_find_attached_body_name(model, p_id, frame_id)) return false;

    pos_addr_    = model->sensor_adr[p_id];
    quat_addr_   = model->sensor_adr[q_id];
    linvel_addr_ = model->sensor_adr[l_id];
    angvel_addr_ = model->sensor_adr[a_id];
    msg_.header.frame_id = world_frame;
    msg_.child_frame_id  = frame_id;
    pub_ = node->create_publisher<nav_msgs::msg::Odometry>(topic, 10);

    RCLCPP_INFO(node->get_logger(), "[OdomSensor] Registered: %s", name.c_str());
    return true;
}

void OdomSensor::update(const mjData* data, const rclcpp::Time& stamp)
{
    msg_.header.stamp = stamp;

    msg_.pose.pose.position.x = data->sensordata[pos_addr_ + 0];
    msg_.pose.pose.position.y = data->sensordata[pos_addr_ + 1];
    msg_.pose.pose.position.z = data->sensordata[pos_addr_ + 2];

    const mjtNum* q = &data->sensordata[quat_addr_];
    msg_.pose.pose.orientation.w = q[0];
    msg_.pose.pose.orientation.x = q[1];
    msg_.pose.pose.orientation.y = q[2];
    msg_.pose.pose.orientation.z = q[3];

    // framelinvel / frameangvel are in the world frame; rotate into body frame
    // to match the nav_msgs/Odometry twist convention (child_frame_id).    
    mju_negQuat(q_inv_, q);
    mju_rotVecQuat(linvel_b_, data->sensordata + linvel_addr_, q_inv_);
    mju_rotVecQuat(angvel_b_, data->sensordata + angvel_addr_, q_inv_);

    msg_.twist.twist.linear.x  = linvel_b_[0];
    msg_.twist.twist.linear.y  = linvel_b_[1];
    msg_.twist.twist.linear.z  = linvel_b_[2];

    msg_.twist.twist.angular.x = angvel_b_[0];
    msg_.twist.twist.angular.y = angvel_b_[1];
    msg_.twist.twist.angular.z = angvel_b_[2];

    pub_->publish(msg_);
}

// -----------------------------------------------------------------------------
// SkinSensor

bool SkinSensor::initialize(mjModel* model, const std::string& name,
                             rclcpp::Node::SharedPtr node,
                             const std::string& topic)
{
    int s_id = mj_name2id(model, mjOBJ_SENSOR, name.c_str());
    std::string frame_id;
    if (s_id < 0 || !sensor_find_attached_body_name(model, s_id, frame_id)) {
        RCLCPP_ERROR(node->get_logger(), "[SkinSensor] Sensor not found: '%s'", name.c_str());
        return false;
    }

    addr_ = model->sensor_adr[s_id];
    int num_cells = int(model->sensor_dim[s_id] / 12);

    data_msg_.header.frame_id = frame_id;
    data_msg_.prox.resize(num_cells, 0.0f);
    data_msg_.force1.resize(num_cells, 0.0f);
    data_msg_.force2.resize(num_cells, 0.0f);
    data_msg_.force3.resize(num_cells, 0.0f);
    data_msg_.acc_x.resize(num_cells, 0.0f);
    data_msg_.acc_y.resize(num_cells, 0.0f);
    data_msg_.acc_z.resize(num_cells, 0.0f);

    info_msg_.name = name;
    info_msg_.header.frame_id = frame_id;
    info_msg_.poses.resize(num_cells);

    data_pub_ = node->create_publisher<skin_msgs::msg::SkinPatchData>(topic + "/raw", 1);

    auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    info_pub_ = node->create_publisher<skin_msgs::msg::SkinPatchInfo>(topic + "/info", latched_qos);

    RCLCPP_INFO(node->get_logger(), "[SkinSensor] Registered: %s", name.c_str());
    return true;
}

void SkinSensor::update(const mjData* data, const rclcpp::Time& stamp)
{
    int addr = addr_;
    int num_cells = static_cast<int>(data_msg_.prox.size());

    // Publish static cell poses on the first update (latched topic)
    if (!info_published_) {
        info_msg_.header.stamp = stamp;
        for (int k = 0; k < num_cells; ++k) {
            auto& pose = info_msg_.poses[k];
            pose.position.x    = data->sensordata[addr + k*7 + 0];
            pose.position.y    = data->sensordata[addr + k*7 + 1];
            pose.position.z    = data->sensordata[addr + k*7 + 2];
            pose.orientation.w = data->sensordata[addr + k*7 + 3];
            pose.orientation.x = data->sensordata[addr + k*7 + 4];
            pose.orientation.y = data->sensordata[addr + k*7 + 5];
            pose.orientation.z = data->sensordata[addr + k*7 + 6];
        }
        info_pub_->publish(info_msg_);
        info_published_ = true;
    }
    addr += 7 * num_cells;

    data_msg_.header.stamp = stamp;
    std::copy_n(data->sensordata + addr, data_msg_.prox.size(), data_msg_.prox.data());
    addr += num_cells;
    std::copy_n(data->sensordata + addr, data_msg_.force1.size(), data_msg_.force1.data());
    addr += num_cells;
    for (int k = 0; k < num_cells; ++k) {
        data_msg_.acc_x[k] = data->sensordata[addr + k*3 + 0];
        data_msg_.acc_y[k] = data->sensordata[addr + k*3 + 1];
        data_msg_.acc_z[k] = data->sensordata[addr + k*3 + 2];
    }
    data_pub_->publish(data_msg_);
}

// -----------------------------------------------------------------------------
// RayCasterSensor

bool RayCasterSensor::initialize(mjModel* model, const std::string& name,
                                  rclcpp::Node::SharedPtr node, const std::string& topic)
{
    int s_id = mj_name2id(model, mjOBJ_SENSOR, name.c_str());
    std::string frame_id;
    if (s_id < 0 || !sensor_find_attached_body_name(model, s_id, frame_id)) {
        RCLCPP_ERROR(node->get_logger(), "[RayCasterSensor] Sensor not found: '%s'", name.c_str());
        return false;
    }

    addr_ = model->sensor_adr[s_id];
    msg_.header.frame_id = frame_id;
    msg_.distance.resize(model->sensor_dim[s_id], 0.0f);
    pub_ = node->create_publisher<skin_msgs::msg::RayCasterData>(topic, 1);

    RCLCPP_INFO(node->get_logger(), "[RayCasterSensor] Registered: %s", name.c_str());
    return true;
}

void RayCasterSensor::update(const mjData* data, const rclcpp::Time& stamp)
{
    msg_.header.stamp = stamp;
    std::copy_n(data->sensordata + addr_, msg_.distance.size(), msg_.distance.data());
    pub_->publish(msg_);
}

} // namespace mujoco

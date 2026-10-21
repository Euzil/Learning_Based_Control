#include <mujoco_simulation/robot_interface.h>

#include <algorithm>
#include <limits>
#include <string>

#include <rclcpp/logging.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace mujoco
{

RobotInterface::~RobotInterface()
{
    if (has_camera_)
        mjv_freeScene(&scn_);
    if (rgb_pub_)
        mjv_freeScene(&rgb_scn_);
}

bool RobotInterface::initialize(mjModel *model, mjData *data,
                                 const RobotConfig &config,
                                 rclcpp::Node::SharedPtr node)
{
    name_ = config.name;
    node_ = node;

    const std::string world_frame = "world";
    const std::string log_prefix  = "[RobotInterface:" + name_ + "] ";

    // --- Joints
    joint_state_pub_ = node_->create_publisher<interface_protocol::msg::JointState>(
        config.joint_state_topic, 10);

    if (!registerJoints(model, data, config)) {
        RCLCPP_ERROR(node_->get_logger(), "%sError registering joints", log_prefix.c_str());
        return false;
    }

    // --- Sensors
    for (const auto &cfg : config.imu_sensors) {
        ImuSensor s;
        if (!s.initialize(model, cfg.name, node_, cfg.topic)) return false;
        imu_sensors_.push_back(std::move(s));
    }
    for (const auto &cfg : config.ft_sensors) {
        FtSensor s;
        if (!s.initialize(model, cfg.name, node_, cfg.topic)) return false;
        ft_sensors_.push_back(std::move(s));
    }
    for (const auto &cfg : config.odom_sensors) {
        OdomSensor s;
        if (!s.initialize(model, cfg.name, node_, cfg.topic, world_frame)) return false;
        odom_sensors_.push_back(std::move(s));
    }
    for (const auto &cfg : config.skin_sensors) {
        SkinSensor s;
        if (!s.initialize(model, cfg.name, node_, cfg.topic)) return false;
        skin_sensors_.push_back(std::move(s));
    }
    for (const auto &cfg : config.ray_caster_sensors) {
        RayCasterSensor s;
        if (!s.initialize(model, cfg.name, node_, cfg.topic)) return false;
        ray_caster_sensors_.push_back(std::move(s));
    }

    // --- Depth camera (at most one per robot)
    if (!config.depth_camera_sensors.empty()) {
        if (config.depth_camera_sensors.size() > 1)
            RCLCPP_WARN(node_->get_logger(),
                "%sMultiple depth cameras configured – only the first will be used",
                log_prefix.c_str());
        const auto &cam_cfg = config.depth_camera_sensors.front();
        if (!registerDepthCamera(model, cam_cfg.name, cam_cfg.width, cam_cfg.height, cam_cfg.topic)) {
            RCLCPP_ERROR(node_->get_logger(),
                "%sError registering depth camera '%s'", log_prefix.c_str(), cam_cfg.name.c_str());
            return false;
        }
    }

    if (config.rgb_camera_sensors.size() > 1) {
        RCLCPP_ERROR(node_->get_logger(), "Only one RGB camera per robot is supported");
        return false;
    }
    if (!config.rgb_camera_sensors.empty() &&
        !registerRgbCamera(model, config.rgb_camera_sensors.front()))
        return false;

    // --- Joint command subscriber
    using std::placeholders::_1;
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    joint_cmd_sub_ = node_->create_subscription<interface_protocol::msg::JointCommand>(
        config.joint_command_topic, qos,
        std::bind(&RobotInterface::jointCommandCallback, this, _1));

    // --- Default command
    num_joints_ = config.joint_names.size();
    joint_command_msg_.position.assign(num_joints_, 0.0);
    joint_command_msg_.velocity.assign(num_joints_, 0.0);
    joint_command_msg_.torque.assign(num_joints_, 0.0);
    joint_command_msg_.feed_forward_torque.assign(num_joints_, 0.0);
    joint_command_msg_.stiffness = config.default_joint_stiffness;
    joint_command_msg_.damping   = config.default_joint_damping;

    RCLCPP_INFO(node_->get_logger(), "%sInitialized successfully", log_prefix.c_str());
    return true;
}

void RobotInterface::update(const mjModel *model, const mjData *data, const rclcpp::Time &stamp)
{
    // Publish joint states
    joint_state_msg_.header.stamp = stamp;
    for (size_t i = 0; i < num_joints_; ++i) {
        joint_state_msg_.position[i] = data->qpos[joint_qpos_addr_[i]];
        joint_state_msg_.velocity[i] = data->qvel[joint_qvel_addr_[i]];
        joint_state_msg_.torque[i]   = data->actuator_force[joint_ctrl_addr_[i]];
    }
    joint_state_pub_->publish(joint_state_msg_);

    // Sensors
    for (auto &s : imu_sensors_)        s.update(data, stamp);
    for (auto &s : ft_sensors_)         s.update(data, stamp);
    for (auto &s : odom_sensors_)       s.update(data, stamp);
    for (auto &s : skin_sensors_)       s.update(data, stamp);
    for (auto &s : ray_caster_sensors_) s.update(data, stamp);

    // Depth camera: publish when the render thread has deposited a new frame
    if (has_camera_) {
        std::unique_lock<std::mutex> lock(depth_mtx_, std::try_to_lock);
        if (lock.owns_lock() && depth_updated_) {
            depth_pub_->publish(depth_img_msg_);
            depth_updated_ = false;
        }
    }
}

void RobotInterface::updateCamera(const mjModel *model, mjData *data, mjrContext *con)
{
    updateRgbCamera(model, data, con);
    if (!has_camera_)
        return;

    mjv_updateScene(model, data, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
    mjr_setBuffer(mjFB_OFFSCREEN, con);
    mjrRect viewport = {0, 0,
                        static_cast<int>(depth_img_msg_.width),
                        static_cast<int>(depth_img_msg_.height)};
    mjr_render(viewport, &scn_, con);
    {
        std::lock_guard<std::mutex> lock(depth_mtx_);
        mjr_readPixels(nullptr, depth_buffer_.data(), viewport, con);

        // Convert raw depth buffer (NDC) to metric depth (metres)
        double extent = model->stat.extent;
        double znear  = model->vis.map.znear * extent;
        double zfar   = model->vis.map.zfar  * extent;
        double diff   = zfar - znear;
        double prod   = znear * zfar;
        float *dst    = reinterpret_cast<float *>(depth_img_msg_.data.data());
        for (size_t i = 0; i < depth_buffer_.size(); ++i) {
            float d = depth_buffer_[i];
            dst[i]  = (d >= 1.0f)
                ? std::numeric_limits<float>::infinity()
                : static_cast<float>(prod / (zfar - d * diff));
        }
        depth_updated_ = true;
    }
    mjr_setBuffer(mjFB_WINDOW, con);
}

void RobotInterface::torqueControl(const mjModel *model, mjData *data)
{
    auto cmd = command();
    for (size_t i = 0; i < num_joints_; ++i) {
        double pos   = data->qpos[joint_qpos_addr_[i]];
        double vel   = data->qvel[joint_qvel_addr_[i]];
        double e_pos = cmd.position[i] - pos;
        double e_vel = cmd.velocity[i] - vel;
        data->ctrl[joint_ctrl_addr_[i]] =
            cmd.feed_forward_torque[i] +
            cmd.stiffness[i] * e_pos +
            cmd.damping[i]   * e_vel;
    }
}

bool RobotInterface::registerJoints(mjModel *model, mjData *data, const RobotConfig &config)
{
    const std::string log_prefix = "[RobotInterface:" + name_ + "] ";
    const auto &joint_names   = config.joint_names;
    const auto &joint_values  = config.default_joint_position;

    const size_t n = joint_names.size();
    joint_qpos_addr_.assign(n, 0);
    joint_qvel_addr_.assign(n, 0);
    joint_ctrl_addr_.assign(n, 0);
    joint_state_msg_.position.assign(n, 0.0);
    joint_state_msg_.velocity.assign(n, 0.0);
    joint_state_msg_.torque.assign(n, 0.0);

    for (int i = 0; i < model->nu; ++i) {
        if (model->actuator_trntype[i] != mjTRN_JOINT)
            continue;
        int bias_type = model->actuator_biastype[i];
        if (bias_type != 0 && bias_type != 1)
            continue;
        int mj_jnt_id = model->actuator_trnid[i * 2];
        if (mj_jnt_id < 0)
            continue;

        const char *jnt_name = mj_id2name(model, mjOBJ_JOINT, mj_jnt_id);
        auto it = std::find(joint_names.begin(), joint_names.end(), jnt_name);
        if (it == joint_names.end()) {
            RCLCPP_INFO(node_->get_logger(),
                "%sActuator joint '%s' not in config – skipping", log_prefix.c_str(), jnt_name);
            continue;
        }

        size_t idx = static_cast<size_t>(std::distance(joint_names.begin(), it));
        joint_qpos_addr_[idx] = model->jnt_qposadr[mj_jnt_id];
        joint_qvel_addr_[idx] = model->jnt_dofadr[mj_jnt_id];
        joint_ctrl_addr_[idx] = i;

        if (idx < joint_values.size())
            data->qpos[joint_qpos_addr_[idx]] = joint_values[idx];

        RCLCPP_INFO(node_->get_logger(), "%sRegistered joint '%s'", log_prefix.c_str(), jnt_name);
    }
    return true;
}

bool RobotInterface::registerDepthCamera(mjModel *model, const std::string &sensor_name,
                                          int width, int height, const std::string &topic)
{
    int cam_id = mj_name2id(model, mjOBJ_CAMERA, sensor_name.c_str());
    if (cam_id < 0) {
        RCLCPP_ERROR(node_->get_logger(),
            "[RobotInterface:%s] Camera '%s' not found in model",
            name_.c_str(), sensor_name.c_str());
        return false;
    }

    std::string frame_id;
    if (!sensor_find_attached_body_name(model, cam_id, frame_id))
        return false;

    mjv_defaultCamera(&cam_);
    cam_.type        = mjCAMERA_FIXED;
    cam_.fixedcamid  = cam_id;
    mjv_defaultOption(&opt_);
    mjv_defaultScene(&scn_);
    mjv_makeScene(model, &scn_, 1000);
    depth_buffer_.resize(static_cast<size_t>(width * height));

    depth_img_msg_.height           = static_cast<unsigned int>(height);
    depth_img_msg_.width            = static_cast<unsigned int>(width);
    depth_img_msg_.header.frame_id  = frame_id;
    depth_img_msg_.encoding         = sensor_msgs::image_encodings::TYPE_32FC1;
    depth_img_msg_.step             = static_cast<unsigned int>(width) * sizeof(float);
    depth_img_msg_.data.resize(depth_buffer_.size() * sizeof(float));

    depth_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(topic, 1);
    has_camera_ = true;
    return true;
}

bool RobotInterface::registerRgbCamera(mjModel *model, const DepthCameraSensorTopic &config)
{
    const int id = mj_name2id(model, mjOBJ_CAMERA, config.name.c_str());
    if (id < 0 || config.width == 0 || config.height == 0 ||
        config.width > 4096 || config.height > 4096) {
        RCLCPP_ERROR(node_->get_logger(), "Invalid RGB camera '%s' or image dimensions",
                     config.name.c_str());
        return false;
    }
    mjv_defaultCamera(&rgb_cam_);
    rgb_cam_.type = mjCAMERA_FIXED;
    rgb_cam_.fixedcamid = id;
    mjv_defaultOption(&rgb_opt_);
    // Robot visual meshes are group 1; hide the collision proxies in group 3.
    rgb_opt_.geomgroup[3] = 0;
    mjv_defaultScene(&rgb_scn_);
    mjv_makeScene(model, &rgb_scn_, 10000);
    rgb_img_msg_.width = config.width;
    rgb_img_msg_.height = config.height;
    rgb_img_msg_.step = config.width * 3;
    rgb_img_msg_.encoding = sensor_msgs::image_encodings::RGB8;
    rgb_img_msg_.header.frame_id = config.name + "_optical_frame";
    rgb_img_msg_.data.resize(static_cast<size_t>(rgb_img_msg_.step) * config.height);
    rgb_buffer_.resize(rgb_img_msg_.data.size());
    rgb_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
        config.topic, rclcpp::SensorDataQoS().keep_last(1));
    RCLCPP_INFO(node_->get_logger(), "RGB camera %s -> %s (%ux%u)",
                config.name.c_str(), config.topic.c_str(), config.width, config.height);
    return true;
}

void RobotInterface::updateRgbCamera(const mjModel *model, mjData *data, mjrContext *con)
{
    if (!rgb_pub_ || rgb_pub_->get_subscription_count() == 0)
        return;
    const auto now = std::chrono::steady_clock::now();
    if (now - last_rgb_frame_ < std::chrono::milliseconds(33))
        return;
    last_rgb_frame_ = now;
    const int width = static_cast<int>(rgb_img_msg_.width);
    const int height = static_cast<int>(rgb_img_msg_.height);
    if (con->offWidth < width || con->offHeight < height)
        mjr_resizeOffscreen(std::max(con->offWidth, width),
                            std::max(con->offHeight, height), con);
    const int previous_buffer = con->currentBuffer;
    mjr_setBuffer(mjFB_OFFSCREEN, con);
    if (con->currentBuffer != mjFB_OFFSCREEN) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                             "RGB camera requires an offscreen framebuffer");
        mjr_setBuffer(previous_buffer, con);
        return;
    }
    mjv_updateScene(model, data, &rgb_opt_, nullptr, &rgb_cam_, mjCAT_ALL, &rgb_scn_);
    const mjrRect viewport{0, 0, width, height};
    mjr_render(viewport, &rgb_scn_, con);
    mjr_readPixels(rgb_buffer_.data(), nullptr, viewport, con);
    mjr_setBuffer(previous_buffer, con);
    // OpenGL starts at the bottom; ROS images start at the top.
    const size_t stride = rgb_img_msg_.step;
    for (int row = 0; row < height; ++row)
        std::copy_n(rgb_buffer_.data() + (height - 1 - row) * stride, stride,
                    rgb_img_msg_.data.data() + row * stride);
    rgb_img_msg_.header.stamp = rclcpp::Time(
        static_cast<int64_t>(data->time * 1e9), RCL_ROS_TIME);
    rgb_pub_->publish(rgb_img_msg_);
}

interface_protocol::msg::JointCommand RobotInterface::command()
{
    std::lock_guard<std::mutex> lock(mtx_);
    return joint_command_msg_;
}

void RobotInterface::jointCommandCallback(
    const interface_protocol::msg::JointCommand::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    has_command_      = true;
    joint_command_msg_ = *msg;
}

} // namespace mujoco

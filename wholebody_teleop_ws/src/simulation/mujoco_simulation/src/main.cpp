#include <rclcpp/rclcpp.hpp>
#include <mujoco_simulation/sim_manager.h>

// todo do not load like this!
#include <mujoco_skin/skin_sensor_plugin.h>
#include <mujoco_skin/ray_sensor_plugin.h>
#include <mujoco_skin/ray_camera_sensor_plugin.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // register the skin sensor plugin
    mujoco::plugin::sensor::SkinSensorPlugin::RegisterPlugin();
    mujoco::plugin::sensor::RaySensorPlugin::RegisterPlugin();
    mujoco::plugin::sensor::RayCameraSensorPlugin::RegisterPlugin();

    // load the parameters
    if(argc < 2) 
    {
        mju_error("mujoco_simulation: Missing Args, Expecting: config_file [str]");
        return -1;
    }
    std::string config_file;
    config_file = argv[1];

    // start simulation
    auto &sim_manager = SimManager::Instance();
    if (!sim_manager.initialize(config_file))
    {
        mju_error("mujoco_simulation: Failed to inialize sim_manager");
        return 1;
    }

    // run
    sim_manager.run();
    rclcpp::shutdown();
    return 0;
}

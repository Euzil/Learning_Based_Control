#ifndef MUJOCO_PLUGIN_RAY_CAMERA_SENSOR_PLUGIN_H_
#define MUJOCO_PLUGIN_RAY_CAMERA_SENSOR_PLUGIN_H_

#include <array>
#include <memory>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>

#include <mujoco_skin/ray_camera_sensor.h>

namespace mujoco::plugin::sensor
{

/**
 * @brief RayCameraSensorPlugin
 * 
 * Setup plugin for RaySensor class.
 */
class RayCameraSensorPlugin
{
   public:
    static RayCameraSensorPlugin* Create(const mjModel* m, mjData* d, int instance);

    RayCameraSensorPlugin(RayCameraSensorPlugin&&) = default;
    ~RayCameraSensorPlugin() = default;

    static void RegisterPlugin();

    static constexpr std::array<const char*, 7> attributes = {
        "update_period", 
        "focal_length",
        "horizontal_aperture",
        "vertical_aperture",
        "size",
        "dis_range",
        "geomgroup"
    };

    void Reset(const mjModel* m, int instance);
    void Compute(const mjModel* m, mjData* d, int instance);
    void Visualize(const mjModel* m, mjData* d, const mjvOption* opt,
                mjvScene* scn, int instance);

   private:
    RayCameraSensorPlugin(const RayCameraSensorCfg &cfg);

   private:
    mjtNum prev_time_, dt_;
    std::unique_ptr<RayCameraSensor> sensor_;
};

}  // namespace mujoco::plugin::sensor

#endif  // MUJOCO_PLUGIN_SKIN_SENSOR_PLUGIN_H_

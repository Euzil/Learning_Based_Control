#ifndef MUJOCO_PLUGIN_RAY_SENSOR_PLUGIN_H_
#define MUJOCO_PLUGIN_RAY_SENSOR_PLUGIN_H_

#include <array>
#include <memory>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>

#include <mujoco_skin/ray_sensor.h>

namespace mujoco::plugin::sensor
{

/**
 * @brief RaySensorPlugin
 * 
 * Setup plugin for RaySensor class.
 */
class RaySensorPlugin
{
   public:
    static RaySensorPlugin* Create(const mjModel* m, mjData* d, int instance);

    RaySensorPlugin(RaySensorPlugin&&) = default;
    ~RaySensorPlugin() = default;

    static void RegisterPlugin();

    static constexpr std::array<const char*, 6> attributes = {
        "update_period", 
        "resolution",
        "size",
        "dis_range",
        "type",
        "geomgroup"
    };

    void Reset(const mjModel* m, int instance);
    void Compute(const mjModel* m, mjData* d, int instance);
    void Visualize(const mjModel* m, mjData* d, const mjvOption* opt,
                mjvScene* scn, int instance);

   private:
    RaySensorPlugin(const RaySensorCfg &cfg);

   private:
    mjtNum prev_time_, dt_;
    std::unique_ptr<RaySensor> sensor_;
};

}  // namespace mujoco::plugin::sensor

#endif  // MUJOCO_PLUGIN_SKIN_SENSOR_PLUGIN_H_

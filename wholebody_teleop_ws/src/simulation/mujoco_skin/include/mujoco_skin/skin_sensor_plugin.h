#ifndef MUJOCO_PLUGIN_SKIN_SENSOR_PLUGIN_H_
#define MUJOCO_PLUGIN_SKIN_SENSOR_PLUGIN_H_

#include <array>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>

#include <mujoco_skin/skin_sensor.h>

namespace mujoco::plugin::sensor
{

/**
 * @brief SkinSensorPlugin
 * 
 * Setup plugin for SkinSensor class.
 */
class SkinSensorPlugin
{
   public:
    static SkinSensorPlugin* Create(const mjModel* m, mjData* d, int instance);

    SkinSensorPlugin(SkinSensorPlugin&&) = default;
    ~SkinSensorPlugin() = default;

    static void RegisterPlugin();

    static constexpr std::array<const char*, 6> attributes = {
        "update_period",
        "pkg", 
        "config", 
        "offset", 
        "dis_range", 
        "geomgroup"
    };

    void Reset(const mjModel* m, int instance);
    void Compute(const mjModel* m, mjData* d, int instance);
    void Visualize(const mjModel* m, mjData* d, const mjvOption* opt,
                mjvScene* scn, int instance);

   private:
    SkinSensorPlugin(const SkinSensorCfg &cfg);

   private:
    mjtNum prev_time_, dt_;
    std::unique_ptr<SkinSensor> sensor_;
};

}  // namespace mujoco::plugin::sensor

#endif  // MUJOCO_PLUGIN_SKIN_SENSOR_PLUGIN_H_

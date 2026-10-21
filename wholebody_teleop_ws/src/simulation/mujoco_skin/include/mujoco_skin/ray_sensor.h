#ifndef MUJOCO_PLUGIN_SENSOR_RAY_SENSOR_H_
#define MUJOCO_PLUGIN_SENSOR_RAY_SENSOR_H_

#include <optional>
#include <vector>
#include <string>
#include <array>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

namespace mujoco::plugin::sensor
{

enum RaySensorType
{
    BASE,
    YAW,
    WORLD
};

/**
 * @brief RaySensorCfg
 *
 */
struct RaySensorCfg
{
    const mjModel* m = nullptr;
    mjData* d = nullptr;
    int sensor_id = -1;
    int obj_type = -1;
    int obj_id = -1;
    int body_id = -1;

    mjtNum update_period;
    mjtNum resolution;
    std::array<mjtNum, 2> size;
    std::array<mjtNum, 2> dis_range;
    RaySensorType type;
    mjtByte geom_group[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
};

/**
 * @brief RaySensor
 */
class RaySensor
{
   public:
    RaySensor(const RaySensorCfg& cfg);
    ~RaySensor() = default;

    void compute();

    void visualize(mjvScene* scn);

   private:
    void createRays();

    void createAABB();

    void computeRays(const mjModel* m, mjData* d, mjtNum* sensordata,
                     mjtNum* pos_w, mjtNum* mat_w);

   private:
    RaySensorCfg cfg_;

    // rays
    size_t ray_num_, h_num_, v_num_;
    std::vector<int> geom_eliminate_;
    std::vector<double> positions_, normals_;
    mjtNum aabb_[6];
};

}  // namespace mujoco::plugin::sensor

#endif  // MUJOCO_PLUGIN_SENSOR_RAY_SENSOR_H_

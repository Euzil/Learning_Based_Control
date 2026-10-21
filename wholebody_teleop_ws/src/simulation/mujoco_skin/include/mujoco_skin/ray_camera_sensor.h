#ifndef MUJOCO_PLUGIN_SENSOR_RAY_CAMERA_H_
#define MUJOCO_PLUGIN_SENSOR_RAY_CAMERA_H_

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

/**
 * @brief RayCameraSensorCfg 
 * 
 */
struct RayCameraSensorCfg
{
    const mjModel* m = nullptr;
    mjData* d = nullptr;
    int sensor_id = -1;
    int obj_type = -1;
    int obj_id = -1;
    int body_id = -1;

    mjtNum update_period;
    mjtNum focal_length;
    mjtNum horizontal_aperture;
    mjtNum vertical_aperture;
    int h_ray_num;
    int v_ray_num;
    std::array<mjtNum, 2> dis_range;
    mjtByte geom_group[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
};

/**
 * @brief RayCameraSensor
 */
class RayCameraSensor
{
   public:
    RayCameraSensor(const RayCameraSensorCfg& cfg);
    ~RayCameraSensor();

    void compute();
    void visualize(mjvScene* scn);

   private:
    void createRays();

    void createAABB();

    void computeRays(int start, int end);

   private:
    RayCameraSensorCfg cfg_;

    // mujoco
    mjtNum *pos_w_; 
    mjtNum *mat_w_;
    mjtNum *sensordata_;

    // multi theading
    int num_thread_;
    mjThreadPool* pool_;
    struct RayTaskData
    {
        RayCameraSensor* instance;
        int start;
        int end;
    };
    std::vector<RayTaskData> ray_task_datas_;
    static void* ray_task_func(void* user_data)
    {
        RayTaskData* data = static_cast<RayTaskData*>(user_data);
        data->instance->computeRays(data->start, data->end);
        return nullptr;
    }

    // rays
    mjtNum focal_length_;
    mjtNum horizontal_aperture_;
    mjtNum vertical_aperture_;
    mjtNum aspect_ratio_;
    mjtNum h_pixel_size_;
    mjtNum v_pixel_size_;

    size_t ray_num_;
    std::vector<int> geom_eliminate_;
    std::vector<double> normals_;
    mjtNum position_[3];

    mjtNum aabb_[6];
};

}  // namespace mujoco::plugin::sensor

#endif  // MUJOCO_PLUGIN_SENSOR_RAY_CAMERA_H_

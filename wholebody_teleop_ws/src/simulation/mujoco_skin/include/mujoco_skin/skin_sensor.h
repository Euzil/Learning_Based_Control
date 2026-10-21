#ifndef MUJOCO_PLUGIN_SENSOR_SKIN_H_
#define MUJOCO_PLUGIN_SENSOR_SKIN_H_

#include <optional>
#include <vector>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

#include <mujoco_skin/skin_utilities.h>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace mujoco::plugin::sensor
{

/**
 * @brief SkinSensorCfg
 */
struct SkinSensorCfg
{
    const mjModel* m = nullptr;
    mjData* d = nullptr;
    int sensor_id = -1;
    int obj_type = -1;
    int obj_id = -1;
    int body_id = -1;

    mjtNum update_period;
    std::string pkg_name;
    std::string config_name;
    mjtNum offset = 0.01;
    std::array<mjtNum, 2> dis_range = {0.0, 0.1};
    mjtByte geom_group[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
};

/**
 * @brief SkinSensor
 *
 * Simulates a single skinpatch described by its yaml file.
 * Contact points are mapped to cell centroids through a kd tree.
 * Distance measurements are obtained by raycasts against the world.
 *
 * Note: contact force unfortunatly sparse
 *
 * Note: Sensor dimension = (2 + 3 + 3 + 4) * num_cells in patch
 * [Position (3) | Rotation (4) | Proximity (1) | Force (1) | Acceleration (3)]
 */
class SkinSensor
{
   public:
    SkinSensor(const SkinSensorCfg& cfg);
    ~SkinSensor() = default;

    void compute();

    void visualize(mjvScene* scn);

   private:
    void computePoses(const mjModel* m, mjData* d, mjtNum* sensordata);

    void computeForces(const mjModel* m, mjData* d, mjtNum* sensordata,
                       mjtNum* pos_w, mjtNum* mat_w);

    void computeProximities(const mjModel* m, mjData* d, mjtNum* sensordata,
                            mjtNum* pos_w, mjtNum* mat_w);

    void computeAccelerations(const mjModel* m, mjData* d, mjtNum* sensordata,
                              mjtNum* pos_w, mjtNum* mat_w);

   private:
    SkinSensorCfg cfg_;

    bool is_first_iter_;

    // search method
    pcl::PointCloud<pcl::PointXYZ>::Ptr centroids_;
    pcl::KdTreeFLANN<pcl::PointXYZ> tree_;

    // data
    float sqd_cell_;
    std::vector<int> nn_indices_;
    std::vector<float> nn_sqd_;
    std::vector<int> geom_eliminate_;
    std::vector<double> positions_offseted_, normals_, rotations_;
    mjtNum aabb_[6];

    int n_cells_;
    int n_contacts_;
    double max_force_;
    std::vector<int> active_ids_;
};

}  // namespace mujoco::plugin::sensor

#endif  // MUJOCO_PLUGIN_SENSOR_SKIN_H_

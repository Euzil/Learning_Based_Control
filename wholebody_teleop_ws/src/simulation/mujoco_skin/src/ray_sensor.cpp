#include <mujoco_skin/ray_sensor.h>
#include <mujoco_skin/ray_cast_utilities.h>
#include <mujoco_skin/plugin_utilities.h>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <sstream>
#include <iostream>
#include <optional>
#include <string>
#include <utility>
#include <filesystem>
#include <cassert>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <iostream>

namespace mujoco::plugin::sensor
{

RaySensor::RaySensor(const RaySensorCfg& cfg) : cfg_{cfg}
{
    // setup the rays
    h_num_ = (int)(cfg_.size[0] / cfg_.resolution) + 1;
    v_num_ = (int)(cfg_.size[1] / cfg_.resolution) + 1;
    ray_num_ = h_num_ * v_num_;
    normals_.resize(ray_num_ * 3);
    positions_.resize(ray_num_ * 3);
    createRays();
    createAABB();
}

void RaySensor::createRays()
{
    if (cfg_.type != RaySensorType::WORLD)
    {
        mjtNum start_x = -cfg_.size[0] / 2;
        mjtNum start_y = cfg_.size[1] / 2;
        for (int i = 0; i < v_num_; i++)
        {
            for (int j = 0; j < h_num_; j++)
            {
                int idx = (i * h_num_ + j) * 3;
                normals_[idx + 0] = 0.0;
                normals_[idx + 1] = 0.0;
                normals_[idx + 2] = -1.0;
                positions_[idx + 0] = start_x;
                positions_[idx + 1] = start_y;
                positions_[idx + 2] = 0.0;
                start_x += cfg_.resolution;
            }
            start_x = -cfg_.size[0] / 2;
            start_y -= cfg_.resolution;
        }
    }
    else
    {
        mjtNum start_x = cfg_.size[0] / 2;
        mjtNum start_y = cfg_.size[1] / 2;
        int tmp = h_num_;
        h_num_ = v_num_;
        v_num_ = tmp;
        for (int i = 0; i < v_num_; i++)
        {
            for (int j = 0; j < h_num_; j++)
            {
                int idx = (i * h_num_ + j) * 3;
                normals_[idx + 0] = 0.0;
                normals_[idx + 1] = 0.0;
                normals_[idx + 2] = -1.0;
                positions_[idx + 0] = start_x;
                positions_[idx + 1] = start_y;
                positions_[idx + 2] = 0.0;
                start_y -= cfg_.resolution;
            }
            start_x -= cfg_.resolution;
            start_y = cfg_.size[1] / 2;
        }
    }
}

void RaySensor::createAABB()
{
    double min_b[3] = {std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::infinity()};
    double max_b[3] = {-std::numeric_limits<double>::infinity(),
                       -std::numeric_limits<double>::infinity(),
                       -std::numeric_limits<double>::infinity()};

    for (int i = 0; i < ray_num_; ++i)
    {
        int idx = i * 3;
        for (int k = 0; k < 3; k++)
        {
            double start_pt =
                positions_[idx + k] * normals_[idx + k] * cfg_.dis_range[0];
            double end_pt =
                positions_[idx + k] + normals_[idx + k] * cfg_.dis_range[1];
            double ray_min = std::min(start_pt, end_pt);
            double ray_max = std::max(start_pt, end_pt);
            min_b[k] = std::min(min_b[k], ray_min);
            max_b[k] = std::max(max_b[k], ray_max);
        }
    }
    for (int k = 0; k < 3; ++k)
    {
        aabb_[k + 0] = (max_b[k] + min_b[k]) / 2.0;
        aabb_[k + 3] = (max_b[k] - min_b[k]) / 2.0;
    }
}

void RaySensor::compute()
{
    mjtNum* sensordata =
        cfg_.d->sensordata + cfg_.m->sensor_adr[cfg_.sensor_id];
    assert(cfg_.m->sensor_dim[cfg_.sensor_id] == ray_num_);

    // get the world transformation of body
    mjtNum *pos_w, *mat_w;
    if (cfg_.obj_type == mjOBJ_SITE)
    {
        pos_w = cfg_.d->site_xpos + 3 * cfg_.obj_id;
        mat_w = cfg_.d->site_xmat + 9 * cfg_.obj_id;
    }
    else
    {
        pos_w = cfg_.d->xpos + 3 * cfg_.obj_id;
        mat_w = cfg_.d->xmat + 9 * cfg_.obj_id;
    }

    // transform based on sensor mode
    if (cfg_.type == RaySensorType::YAW)
    {
        mjtNum mat_yaw_w[9];
        mjtNum yaw = yaw_from_mat(mat_w);
        mat_from_yaw(mat_yaw_w, yaw);
        computeRays(cfg_.m, cfg_.d, sensordata + 0, pos_w, mat_yaw_w);
    }
    else
    {
        computeRays(cfg_.m, cfg_.d, sensordata + 0, pos_w, mat_w);
    }
}

void RaySensor::computeRays(const mjModel* m, mjData* d, mjtNum* sensordata,
                            mjtNum* pos_w, mjtNum* mat_w)
{
    // reset all prox data
    mju_fill(sensordata, cfg_.dis_range[1], ray_num_);

    // setup geometry mask
    geom_eliminate_.resize(m->ngeom);
    mju_multiRayPrepare_2(m, d, pos_w, mat_w, aabb_, cfg_.geom_group, 1,
                          cfg_.body_id, geom_eliminate_.data());

    // ray casting
    mjtNum dist;
    mjtNum pos_world[3];
    mjtNum normal_world[3];

    for (size_t i = 0; i < ray_num_; ++i)
    {
        // position world
        const double* pos_local = &positions_[i * 3];
        mju_mulMatVec3(pos_world, mat_w, pos_local);
        mju_addTo3(pos_world, pos_w);

        // orientation in world
        const double* normal_local = &normals_[i * 3];
        mju_mulMatVec3(normal_world, mat_w, normal_local);

        // raycast
        int geomid = -1;
        dist = mju_singleRay_2(m, d, pos_world, normal_world,
                               geom_eliminate_.data(), &geomid);
        if (dist < 0)
        {
            sensordata[i] = cfg_.dis_range[1];
        }
        else
        {
            sensordata[i] =
                std::min(cfg_.dis_range[1], std::max(cfg_.dis_range[0], dist));
        }
    }
}

void RaySensor::visualize(mjvScene* scn)
{
    const static float ray_rgba[4] = {0, 1, 0, 0.3};
    const static mjtNum sphere_size[3] = {0.02, 0.02, 0.02};
    const static mjtNum ray_mat[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    // Get sensor id and data
    mjtNum* data = cfg_.d->sensordata + cfg_.m->sensor_adr[cfg_.sensor_id] + 0;

    // get the world transformation
    mjtNum *pos_w, *_mat_w;
    if (cfg_.obj_type == mjOBJ_SITE)
    {
        pos_w = cfg_.d->site_xpos + 3 * cfg_.obj_id;
        _mat_w = cfg_.d->site_xmat + 9 * cfg_.obj_id;
    }
    else
    {
        pos_w = cfg_.d->xpos + 3 * cfg_.obj_id;
        _mat_w = cfg_.d->xmat + 9 * cfg_.obj_id;
    }

    // transform based on sensor mode
    mjtNum mat_w[9];
    if (cfg_.type == RaySensorType::YAW)
    {
        mjtNum yaw = yaw_from_mat(_mat_w);
        mat_from_yaw(mat_w, yaw);
    }
    else
    {
        mju_copy(mat_w, _mat_w, 9);
    }

    // draw geoms
    mjtNum ray_n_w[3];
    mjtNum ray_pos_w[3];
    mjtNum hit_pos_w[3];
    for (size_t i = 0; i < ray_num_; ++i)
    {
        // position world
        const double* ray_pos_local = &positions_[i * 3];
        mju_mulMatVec3(ray_pos_w, mat_w, ray_pos_local);
        mju_addTo3(ray_pos_w, pos_w);

        // orientation in world
        const double* ray_normal_local = &normals_[i * 3];
        mju_mulMatVec3(ray_n_w, mat_w, ray_normal_local);
        mju_scl(hit_pos_w, ray_n_w, data[i], 3);
        mju_addTo3(hit_pos_w, ray_pos_w);

        mjvGeom* geom = scn->geoms + scn->ngeom;
        mjv_initGeom(geom, mjGEOM_SPHERE, sphere_size, hit_pos_w, ray_mat,
                     ray_rgba);
        geom->objid = i;
        geom->category = mjCAT_DECOR;
        geom->segid = scn->ngeom;
        scn->ngeom++;
    }
}

}  // namespace mujoco::plugin::sensor
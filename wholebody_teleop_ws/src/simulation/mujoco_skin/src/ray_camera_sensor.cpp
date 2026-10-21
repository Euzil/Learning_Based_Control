// Copyright 2023 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <mujoco_skin/ray_camera_sensor.h>
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
RayCameraSensor::RayCameraSensor(const RayCameraSensorCfg& cfg) : cfg_{cfg}
{
    // setup the rays
    ray_num_ = cfg_.h_ray_num * cfg_.v_ray_num;
    normals_.resize(ray_num_ * 3);
    mju_zero3(position_);
    createRays();
    createAABB();

    // setup the threading
    num_thread_ = 4;
    ray_task_datas_.resize(num_thread_);
    pool_ = mju_threadPoolCreate(num_thread_);
    for (int i = 1; i < (num_thread_ + 1); i++)
    {
        int idx = (i + 1) * ray_num_ / (num_thread_ + 1);
        RayTaskData rd;
        rd.instance = this;
        rd.start = i * ray_num_ / (num_thread_ + 1);
        rd.end = idx;
        ray_task_datas_[i - 1] = rd;
    }
}

RayCameraSensor::~RayCameraSensor()
{
    if (pool_ != nullptr)
        mju_threadPoolDestroy(pool_);
}

void RayCameraSensor::createRays()
{
    mjtNum aspect_ratio, vertical_aperture, horizontal_aperture;
    vertical_aperture = cfg_.vertical_aperture;
    horizontal_aperture = cfg_.horizontal_aperture;
    if (vertical_aperture == 0)
    {
        aspect_ratio = double(cfg_.h_ray_num) / double(cfg_.v_ray_num);
        vertical_aperture = horizontal_aperture / aspect_ratio;
    }
    else
    {
        aspect_ratio = horizontal_aperture / vertical_aperture;
    }

    mjtNum h_pixel_size = horizontal_aperture / cfg_.h_ray_num;
    mjtNum v_pixel_size = (horizontal_aperture / aspect_ratio) / cfg_.v_ray_num;

    mjtNum x, y, z;
    for (int i = 0; i < cfg_.v_ray_num; ++i)
    {
        for (int j = 0; j < cfg_.h_ray_num; ++j)
        {
            int idx = (i * cfg_.h_ray_num + j) * 3;

            x = (j + 0.5 - cfg_.h_ray_num / 2.0) * h_pixel_size;
            y = (cfg_.v_ray_num / 2.0 - i - 0.5) * v_pixel_size;
            z = -cfg_.focal_length;
            mjtNum pnt_px[3] = {x, y, z};
            mjtNum norm = mju_norm3(pnt_px);

            normals_[idx + 0] = x / norm;
            normals_[idx + 1] = y / norm;
            normals_[idx + 2] = z / norm;
        }
    }
}

void RayCameraSensor::createAABB()
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
                position_[k] * normals_[idx + k] * cfg_.dis_range[0];
            double end_pt =
                position_[k] + normals_[idx + k] * cfg_.dis_range[1];
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

void RayCameraSensor::compute()
{
    sensordata_ = cfg_.d->sensordata + cfg_.m->sensor_adr[cfg_.sensor_id];
    assert(cfg_.m->sensor_dim[cfg_.sensor_id] == ray_num_);

    // get the world transformation of body
    if (cfg_.obj_type == mjOBJ_SITE)
    {
        pos_w_ = cfg_.d->site_xpos + 3 * cfg_.obj_id;
        mat_w_ = cfg_.d->site_xmat + 9 * cfg_.obj_id;
    }
    else
    {
        pos_w_ = cfg_.d->xpos + 3 * cfg_.obj_id;
        mat_w_ = cfg_.d->xmat + 9 * cfg_.obj_id;
    }

    // reset all data
    mju_fill(sensordata_, cfg_.dis_range[1], ray_num_);

    // setup geometry mask
    geom_eliminate_.resize(cfg_.m->ngeom);
    mju_multiRayPrepare_2(cfg_.m, cfg_.d, pos_w_, mat_w_, aabb_,
                          cfg_.geom_group, 1, cfg_.body_id, geom_eliminate_.data());

    // do parallel raycasting
    std::vector<mjTask> tasks(num_thread_);
    for (int i = 0; i < num_thread_; i++)
    {
        mju_defaultTask(&tasks[i]);
        tasks[i].func = ray_task_func;
        tasks[i].args = &ray_task_datas_[i];
        mju_threadPoolEnqueue(pool_, &tasks[i]);
    }
    computeRays(0, ray_task_datas_[0].start);
    for (int i = 0; i < num_thread_; i++)
    {
        mju_taskJoin(&tasks[i]);
    }
}

void RayCameraSensor::computeRays(int start, int end)
{
    // ray casting
    mjtNum dist;
    mjtNum normal_world[3];
    for (size_t i = start; i < end; ++i)
    {
        // orientation in world
        const double* normal_local = &normals_[i * 3];
        mju_mulMatVec3(normal_world, mat_w_, normal_local);
        // raycast
        int geomid = -1;
        dist = mju_singleRay_2(cfg_.m, cfg_.d, pos_w_, normal_world,
                               geom_eliminate_.data(), &geomid);
        if (dist < 0)
        {
            sensordata_[i] = cfg_.dis_range[1];
        }
        else
        {
            sensordata_[i] =
                std::min(cfg_.dis_range[1], std::max(cfg_.dis_range[0], dist));
        }
    }
}

void RayCameraSensor::visualize(mjvScene* scn)
{
    const static float ray_rgba[4] = {1, 0, 0, 0.05};
    // const static mjtNum sphere_size[3] = {0.02, 0.02, 0.02};
    const static mjtNum ray_size[3] = {1, 1, 1};
    const static mjtNum ray_mat[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    // Get sensor id and data
    mjtNum* data = cfg_.d->sensordata + cfg_.m->sensor_adr[cfg_.sensor_id] + 0;

    // get the world transformation
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

    // draw geoms
    mjtNum ray_n_w[3];
    mjtNum ray_pos_w[3];
    mjtNum hit_pos_w[3];

    // position world
    mju_mulMatVec3(ray_pos_w, mat_w, position_);
    mju_addTo3(ray_pos_w, pos_w);

    for (size_t i = 0; i < ray_num_; ++i)
    {
        // orientation in world
        const double* ray_normal_local = &normals_[i * 3];
        mju_mulMatVec3(ray_n_w, mat_w, ray_normal_local);
        mju_scl(hit_pos_w, ray_n_w, data[i], 3);
        mju_addTo3(hit_pos_w, ray_pos_w);

        mjvGeom* geom = scn->geoms + scn->ngeom;
        mjv_initGeom(geom, mjGEOM_LINE, ray_size, ray_pos_w, ray_mat, ray_rgba);
        mjv_connector(geom, mjGEOM_LINE, 3, ray_pos_w, hit_pos_w);
        geom->objid = i;
        geom->category = mjCAT_DECOR;
        geom->segid = scn->ngeom;
        scn->ngeom++;

        // mjvGeom *geom = scn->geoms + scn->ngeom;
        // mjv_initGeom(geom, mjGEOM_SPHERE, sphere_size, hit_pos_w, ray_mat,
        // ray_rgba); geom->objid = i; geom->category = mjCAT_DECOR; geom->segid
        // = scn->ngeom; scn->ngeom++;
    }
}

}  // namespace mujoco::plugin::sensor
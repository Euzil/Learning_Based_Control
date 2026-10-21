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

#include <mujoco_skin/skin_sensor.h>
#include <mujoco_skin/ray_cast_utilities.h>

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
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <iostream>

namespace mujoco::plugin::sensor
{

SkinSensor::SkinSensor(const SkinSensorCfg& cfg) : cfg_{cfg}
{
    // read the skin patch config
    skin_sensor::SkinPatchConfig patch_cfg;
    read_skin_cfg_from_pkg(cfg_.pkg_name, cfg_.config_name, patch_cfg);

    // setup variables
    n_cells_ = patch_cfg.cells.size();

    // cell positions, normals and centroids in body frame
    centroids_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    centroids_->points.resize(n_cells_);
    normals_.resize(n_cells_ * 3);
    rotations_.resize(n_cells_ * 9);
    positions_offseted_.resize(n_cells_ * 3);

    // setup the aabb of in joint frame
    Eigen::Vector3d p_min =
        Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector3d p_max =
        Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());

    Eigen::Vector3d p_local_offseted(0, 0, -cfg_.offset);
    Eigen::Vector3d n_local(0, 0, 1);
    Eigen::Affine3d T_cell_joint;

    // setup geometry
    // Note: cells positions are offseted infwards by -cfg.offset to make sure
    // rays interesect other touching bodies
    double max_ray_dist = cfg_.dis_range[1] + cfg_.offset;
    for (size_t i = 0; i < n_cells_; ++i)
    {
        // compute the position in joint frame
        T_cell_joint = patch_cfg.T_base_rsc * patch_cfg.cells[i].T_cell_root;
        Eigen::Vector3d p_joint_offseted = T_cell_joint * p_local_offseted;

        (*centroids_)[i].getVector3fMap() = T_cell_joint.translation().cast<float>();
        std::copy_n(p_joint_offseted.data(), 3, &positions_offseted_[i * 3]);

        // compute the normal in joint frame
        Eigen::Vector3d n_joint = T_cell_joint.rotation() * n_local;
        std::copy_n(n_joint.data(), 3, &normals_[i * 3]);

        // compute the rotation matrix
        Eigen::Matrix3d R = T_cell_joint.rotation();
        std::copy_n(R.data(), 9, &rotations_[i * 9]);

        // compute min and max bounds in joint frame
        Eigen::Vector3d p_end_joint = p_joint_offseted + n_joint * max_ray_dist;
        p_min = p_min.cwiseMin(p_joint_offseted).cwiseMin(p_end_joint);
        p_max = p_max.cwiseMax(p_joint_offseted).cwiseMax(p_end_joint);
    }
    tree_.setInputCloud(centroids_);

    // compute the aabb in joint frame
    Eigen::Vector3d aabb_center = (p_max + p_min) / 2.0;
    Eigen::Vector3d aabb_size = (p_max - p_min) / 2.0;
    std::copy_n(aabb_center.data(), 3, aabb_);
    std::copy_n(aabb_size.data(), 3, aabb_ + 3);

    // knn k=1
    nn_indices_.resize(1);
    nn_sqd_.resize(1);
    sqd_cell_ = 0.04 * 0.04;

    is_first_iter_ = true;
}

void SkinSensor::compute()
{
    // get the memoery
    mjtNum* sensordata =
        cfg_.d->sensordata + cfg_.m->sensor_adr[cfg_.sensor_id];
    assert(cfg_.m->sensor_dim[cfg_.sensor_id] == 12 * n_cells_);

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

    // positon and rotation values (7)
    if (is_first_iter_)
    {
        is_first_iter_ = false;
        computePoses(cfg_.m, cfg_.d, sensordata);
    }
    sensordata += 7 * n_cells_;

    // proximity values (1)
    computeProximities(cfg_.m, cfg_.d, sensordata, pos_w, mat_w);
    sensordata += n_cells_;

    // contacts forces (1)
    computeForces(cfg_.m, cfg_.d, sensordata, pos_w, mat_w);
    sensordata += n_cells_;

    // acceleration values (3)
    computeAccelerations(cfg_.m, cfg_.d, sensordata, pos_w, mat_w);
    sensordata += n_cells_;
}

void SkinSensor::computeForces(const mjModel* m, mjData* d, mjtNum* sensordata,
                               mjtNum* pos_w, mjtNum* mat_w)
{
    // reset all force data
    n_contacts_ = 0;
    max_force_ = 0.0;
    active_ids_.clear();
    mju_fill(sensordata, 0.0, n_cells_);

    // Count contacts that fall into this body
    int ncon = 0;
    int parent_weld = m->body_weldid[cfg_.body_id];
    for (int i = 0; i < d->ncon; i++)
    {
        int body1 = m->body_weldid[m->geom_bodyid[d->contact[i].geom1]];
        int body2 = m->body_weldid[m->geom_bodyid[d->contact[i].geom2]];
        if (body1 == parent_weld || body2 == parent_weld)
        {
            ncon++;
        }
    }

    // sort contacts into cells
    mjtNum force[6], pos[3];
    mjtNum tmp_force[6], tmp1[3];
    pcl::PointXYZ pt;
    for (int i = 0; i < d->ncon; ++i)
    {
        int body1 = m->geom_bodyid[d->contact[i].geom1];
        int weld1 = m->body_weldid[body1];
        int body2 = m->geom_bodyid[d->contact[i].geom2];
        int weld2 = m->body_weldid[body2];
        if (weld1 == parent_weld || weld2 == parent_weld)
        {
            // get contact force in contact frame
            mj_contactForce(m, d, i, tmp_force);

            // rotate force/torque into body frame
            mju_mulMatTVec3(tmp1, d->contact[i].frame, tmp_force);
            mju_mulMatTVec3(force, mat_w, tmp1);
            mju_mulMatTVec3(tmp1, d->contact[i].frame, tmp_force + 3);
            mju_mulMatTVec3(force + 3, mat_w, tmp1);

            // Forces point from the smaller to larger body, so flip sign if
            // the parent body has smaller id.
            if (cfg_.body_id < mjMAX(body1, body2))
                mju_scl(force, force, -1, 6);

            // get the contact position in local frame
            mju_sub3(tmp1, d->contact[i].pos, pos_w);
            mju_mulMatTVec3(pos, mat_w, tmp1);

            // search for corresponding cell
            pt.x = pos[0];
            pt.y = pos[1];
            pt.z = pos[2];
            if (tree_.nearestKSearch(pt, 1, nn_indices_, nn_sqd_))
            {
                int cell_idx = nn_indices_[0];

                // if(nn_sqd_[0] < sqd_cell_)
                {
                    if (sensordata[cell_idx] == 0.0)
                    {
                        active_ids_.push_back(cell_idx);
                    }
                    sensordata[cell_idx] += force[2];

                    // remember patch data
                    active_ids_.push_back(cell_idx);
                    if (max_force_ < sensordata[cell_idx])
                    {
                        max_force_ = sensordata[cell_idx];
                    }
                    n_contacts_++;
                }
            }
        }
    }
}

void SkinSensor::computeProximities(const mjModel* m, mjData* d,
                                    mjtNum* sensordata, mjtNum* pos_w,
                                    mjtNum* mat_w)
{
    // reset all prox data
    mju_fill(sensordata, 0.0, n_cells_);

    // setup geometry mask
    geom_eliminate_.resize(m->ngeom);
    mju_multiRayPrepare_2(m, d, pos_w, mat_w, aabb_, cfg_.geom_group, 1,
                          cfg_.body_id, geom_eliminate_.data());

    // ray casting
    mjtNum dist, prox;
    mjtNum cell_pos_world[3];
    mjtNum cell_normal_world[3];
    for (size_t i = 0; i < n_cells_; ++i)
    {
        // position world
        const double* cell_pos_local = &positions_offseted_[i * 3];
        mju_mulMatVec3(cell_pos_world, mat_w, cell_pos_local);
        mju_addTo3(cell_pos_world, pos_w);

        // orientation in world
        const double* cell_normal_local = &normals_[i * 3];
        mju_mulMatVec3(cell_normal_world, mat_w, cell_normal_local);

        // raycast
        int geomid = -1;
        dist = mju_singleRay_2(m, d, cell_pos_world, cell_normal_world,
                               geom_eliminate_.data(), &geomid);
        if (dist < 0)
        {
            // no hit
            sensordata[i] = 0.0;
        }
        else
        {
            // hit
            dist = std::min(cfg_.dis_range[1],
                         std::max(cfg_.dis_range[0], dist - cfg_.offset));
            sensordata[i] = 1.0 - dist / cfg_.dis_range[1];
        }
    }
}

void SkinSensor::computeAccelerations(const mjModel* m, mjData* d,
                                      mjtNum* sensordata, mjtNum* pos_w,
                                      mjtNum* mat_w)
{
    // reset all acc data
    // mju_fill(sensordata, 0.0, 3 * n_cells_);

    // [wx, wy, wz, vx, vy, vz]
    mjtNum obj_vel[6];
    mj_objectVelocity(m, d, mjOBJ_XBODY, cfg_.obj_id, obj_vel, 1);

    // [alphax, alphay, alphaz, ax, ay, az]
    mjtNum obj_acc[6];
    mj_objectAcceleration(m, d, mjOBJ_XBODY, cfg_.obj_id, obj_acc, 1);

    mjtNum* w = obj_vel;        // Angular velocity
    mjtNum* alpha = obj_acc;    // Angular acceleration
    mjtNum* a_B = obj_acc + 3;  // Linear acceleration

    mjtNum term1[3];
    mjtNum w_x_r[3];
    mjtNum term2[3];
    mjtNum obj_a_lin[3];
    mjtNum cell_a_lin[3];
    for (size_t i = 0; i < n_cells_; ++i)
    {
        mjtNum* res = &sensordata[i * 3];

        // local transformation
        const mjtNum* r = &positions_offseted_[i * 3];
        const mjtNum* R = &rotations_[i * 9];

        // alpha x r
        mju_cross(term1, alpha, r);

        // w x (w x r)
        mju_cross(w_x_r, w, r);
        mju_cross(term2, w, w_x_r);

        // linear acceleration in a_lin
        mju_add3(obj_a_lin, a_B, term1);
        mju_addTo3(obj_a_lin, term2);
        mju_mulMatTVec3(cell_a_lin, R, obj_a_lin);

        // convert to g, clip and filter
        for (int j = 0; j < 3; j++)
        {
            res[j] = 0.7 * res[j] +
                     0.3 * std::min(std::max(cell_a_lin[j] / 9.81, -4.0), 4.0);
        }
    }
}

void SkinSensor::computePoses(const mjModel* m, mjData* d, mjtNum* sensordata)
{
    // [x, y, z, qw, qx, qy, qz]
    Eigen::Vector3d p_local_offseted(0, 0, -cfg_.offset);
    for (size_t i = 0; i < n_cells_; ++i)
    {
        Eigen::Matrix3d R = Eigen::Map<const Eigen::Matrix3d>(&rotations_[i * 9]);
        Eigen::Vector3d p_joint_offseted = R * p_local_offseted;
        sensordata[i * 7 + 0] = positions_offseted_[i * 3 + 0] - p_joint_offseted.x();
        sensordata[i * 7 + 1] = positions_offseted_[i * 3 + 1] - p_joint_offseted.x();
        sensordata[i * 7 + 2] = positions_offseted_[i * 3 + 2] - p_joint_offseted.x();
        mju_mat2Quat(&sensordata[i * 7 + 3], &rotations_[i * 9]);
    }
}

void SkinSensor::visualize(mjvScene* scn)
{
    mjtNum* posedata = 
        cfg_.d->sensordata + cfg_.m->sensor_adr[cfg_.sensor_id];
    mjtNum* proxdata =
        cfg_.d->sensordata + cfg_.m->sensor_adr[cfg_.sensor_id] + n_cells_ * 7;
    mjtNum* forcedata =
        cfg_.d->sensordata + cfg_.m->sensor_adr[cfg_.sensor_id] + n_cells_ * 8;

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

    // hardcoded shape
    mjtNum cell_size[3] = {0.005, 0.005, 0.01};
    float cell_rgba[4] = {1, 0, 0, 1.0};
    float ray_rgba[4] = {0, 1, 0, 0.3};

    // draw geoms
    mjtNum cell_pos_w[3];
    mjtNum cell_n_w[3];
    mjtNum cell_quat_w[4];
    mjtNum cell_mat_w[9];

    mjtNum target_w[3];

    // draw the active contacts
    for (size_t i = 0; i < active_ids_.size(); ++i)
    {
        if (scn->ngeom >= scn->maxgeom)
        {
            mj_warning(cfg_.d, mjWARN_VGEOMFULL, scn->maxgeom);
            return;
        }

        int id = active_ids_[i];

        // position world
        const double* cell_pos_local = &posedata[id * 7];
        mju_mulMatVec3(cell_pos_w, mat_w, cell_pos_local);
        mju_addTo3(cell_pos_w, pos_w);

        // orientation in world
        const double* cell_normal_local = &normals_[i * 3];
        mju_mulMatVec3(cell_n_w, mat_w, cell_normal_local);
        mju_axisAngle2Quat(cell_quat_w, cell_n_w, 0.0);
        mju_quat2Mat(cell_mat_w, cell_quat_w);

        // color
        cell_rgba[0] = mju_abs(forcedata[id]) / max_force_;

        // draw circle geom
        mjvGeom* geom = scn->geoms + scn->ngeom;
        mjv_initGeom(geom, mjGEOM_CYLINDER, cell_size, cell_pos_w, cell_mat_w,
                     cell_rgba);
        geom->objtype = mjOBJ_UNKNOWN;
        geom->objid = id;
        geom->category = mjCAT_DECOR;
        geom->segid = scn->ngeom;
        scn->ngeom++;
    }

    // draw the lidar ranges
    mjtNum size[3] = {1, 1, 1};
    mjtNum dist = 0.0;
    for (size_t i = 0; i < n_cells_; ++i)
    {
        // distance
        dist = (1.0 - proxdata[i]) * cfg_.dis_range[1];

        // position world
        const double* cell_pos_local = &posedata[i * 7];
        mju_mulMatVec3(cell_pos_w, mat_w, cell_pos_local);
        mju_addTo3(cell_pos_w, pos_w);

        // orientation in world
        const double* cell_normal_local = &normals_[i * 3];
        mju_mulMatVec3(cell_n_w, mat_w, cell_normal_local);
        mju_scl(target_w, cell_n_w, dist, 3);
        mju_addTo3(target_w, cell_pos_w);

        mjvGeom* geom = scn->geoms + scn->ngeom;
        mjv_initGeom(geom, mjGEOM_LINE, size, cell_pos_w, cell_mat_w,
                     ray_rgba);
        mjv_connector(geom, mjGEOM_LINE, 3, cell_pos_w, target_w);
        geom->objid = i;
        geom->category = mjCAT_DECOR;
        geom->segid = scn->ngeom;
        scn->ngeom++;
    }
}

}  // namespace mujoco::plugin::sensor
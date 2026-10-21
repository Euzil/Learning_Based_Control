#ifndef MUJOCO_PLUGIN_SENSOR_SKIN_RAY_CASTING_H_
#define MUJOCO_PLUGIN_SENSOR_SKIN_RAY_CASTING_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>

namespace mujoco::plugin::sensor
{

inline int ray_eliminate_2(const mjModel* m, const mjData* d, int geomid,
                           const mjtByte* geomgroup, mjtByte flg_static,
                           int bodyexclude)
{
    // body exclusion
    if (m->geom_bodyid[geomid] == bodyexclude)
    {
        return 1;
    }

    // invisible geom exclusion
    if (m->geom_matid[geomid] < 0 && m->geom_rgba[4 * geomid + 3] == 0)
    {
        return 1;
    }

    // invisible material exclusion
    if (m->geom_matid[geomid] >= 0 &&
        m->mat_rgba[4 * m->geom_matid[geomid] + 3] == 0)
    {
        return 1;
    }

    // static exclusion
    if (!flg_static && m->body_weldid[m->geom_bodyid[geomid]] == 0)
    {
        return 1;
    }

    // no geomgroup inclusion
    if (!geomgroup)
    {
        return 0;
    }

    // group inclusion/exclusion
    int groupid = mjMIN(mjNGROUP - 1, mjMAX(0, m->geom_group[geomid]));
    return (geomgroup[groupid] == 0);
}

inline void mju_multiRayPrepare_2(const mjModel* m, const mjData* d,
                                  const mjtNum* pos, const mjtNum* mat,
                                  const mjtNum* ray_aabb,
                                  const mjtByte* geomgroup, mjtByte flg_static,
                                  int bodyexclude, int* geom_eliminate)
{
    // compute eliminate flag for all geoms
    for (int geomid = 0; geomid < m->ngeom; geomid++)
    {
        geom_eliminate[geomid] =
            ray_eliminate_2(m, d, geomid, geomgroup, flg_static, bodyexclude);
    }

    // check all bodies
    mjtNum vec_world[3], vec_local[3];
    for (int b = 0; b < m->nbody; b++)
    {
        // skip precomputation if no bounding volume is available
        if (m->body_bvhadr[b] == -1)
        {
            continue;
        }

        // loop over child geoms, compute bounding angles
        for (int i = 0; i < m->body_geomnum[b]; i++)
        {
            int g = i + m->body_geomadr[b];

            // keep planes
            if (m->geom_type[g] == mjtGeom::mjGEOM_PLANE)
            {
                continue;
            }

            // skip if eliminated by flags
            if (geom_eliminate[g])
            {
                continue;
            }

            // Target geometry world position and bounding radius
            const mjtNum* target_world_pos = d->geom_xpos + 3 * g;
            mjtNum target_rad = m->geom_rbound[g];

            mju_sub3(vec_world, target_world_pos, pos);
            mju_mulMatTVec3(vec_local, mat, vec_world);

            // AABB check in the LOCAL frame
            bool intersect = true;
            for (int j = 0; j < 3; j++)
            {
                if (mju_abs(vec_local[j] - ray_aabb[j]) >
                    ray_aabb[j + 3] + target_rad)
                {
                    intersect = false;
                    break;
                }
            }
            if (!intersect)
            {
                geom_eliminate[g] = 1;
            }
        }
    }
}

inline mjtNum mju_singleRay_2(const mjModel* m, mjData* d, const mjtNum pnt[3],
                       const mjtNum vec[3], int* geom_eliminate, int geomid[1])
{
    mjtNum dist, newdist;

    // clear result
    dist = -1;
    *geomid = -1;

    // loop over geoms not eliminated by mask and bodyexclude
    for (int i = 0; i < m->ngeom; i++)
    {
        if (!geom_eliminate[i])
        {
            // handle mesh and hfield separately
            if (m->geom_type[i] == mjGEOM_MESH)
            {
                newdist = mj_rayMesh(m, d, i, pnt, vec);
            }
            else if (m->geom_type[i] == mjGEOM_HFIELD)
            {
                newdist = mj_rayHfield(m, d, i, pnt, vec);
            }

            // otherwise general dispatch
            else
            {
                newdist = mju_rayGeom(
                    d->geom_xpos + 3 * i, d->geom_xmat + 9 * i,
                    m->geom_size + 3 * i, pnt, vec, m->geom_type[i]);
            }

            // update if closer intersection found
            if (newdist >= 0 && (newdist < dist || dist < 0))
            {
                dist = newdist;
                *geomid = i;
            }
        }
    }
    return dist;
}

inline void mat_from_yaw(mjtNum mat[9], mjtNum yaw)
{
    mjtNum c = std::cos(yaw);
    mjtNum s = std::sin(yaw);
    mat[0] = c;
    mat[1] = -s;
    mat[2] = 0.0;
    mat[3] = s;
    mat[4] = c;
    mat[5] = 0.0;
    mat[6] = 0.0;
    mat[7] = 0.0;
    mat[8] = 1.0;
}

inline mjtNum yaw_from_mat(const mjtNum mat[9]) 
{
    return std::atan2(mat[3], mat[0]);
}

inline void rotate_vector_with_yaw(mjtNum result[3], mjtNum yaw, const mjtNum vec[3])
{
    mjtNum c = cos(yaw);
    mjtNum s = sin(yaw);
    result[0] = c * vec[0] - s * vec[1];
    result[1] = s * vec[0] + c * vec[1];
    result[2] = vec[2];
}

}  // namespace mujoco::plugin::sensor

#endif
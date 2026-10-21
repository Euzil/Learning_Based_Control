#include <mujoco_skin/skin_sensor_plugin.h>
#include <mujoco_skin/plugin_utilities.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>

namespace mujoco::plugin::sensor
{

SkinSensorPlugin* SkinSensorPlugin::Create(const mjModel* m, mjData* d,
                                           int instance)
{
    SkinSensorCfg cfg;

    // ---
    // read configuration parameter
    auto update_period = 
        read_vector<double>(mj_getPluginConfig(m, instance, attributes[0]));
    if(update_period.empty())
        cfg.update_period = 0.001;
    else
        cfg.update_period = update_period[0];

    cfg.pkg_name = std::string(mj_getPluginConfig(m, instance, attributes[1]));
    if (cfg.pkg_name.empty())
    {
        mju_error("Invalid or missing parameters '%s' in skin sensor plugin",
                  attributes[1]);
        return nullptr;
    }

    cfg.config_name =
        std::string(mj_getPluginConfig(m, instance, attributes[2]));
    if (cfg.config_name.empty())
    {
        mju_error("Invalid or missing parameters '%s' in skin sensor plugin",
                  attributes[2]);
        return nullptr;
    }

    auto offset =
        read_vector<double>(mj_getPluginConfig(m, instance, attributes[3]), 1);
    if (offset.empty())
    {
        mju_error("Invalid or missing parameters '%s' in skin sensor plugin",
                  attributes[3]);
        return nullptr;
    }
    cfg.offset = offset[0];

    auto dis_range =
        read_vector<double>(mj_getPluginConfig(m, instance, attributes[4]), 2);
    if (dis_range.empty())
    {
        mju_error("Invalid or missing parameters '%s' in skin sensor plugin",
                  attributes[4]);
        return nullptr;
    }
    cfg.dis_range[0] = dis_range[0];
    cfg.dis_range[1] = dis_range[1];

    auto geomgroup =
        read_vector<int>(mj_getPluginConfig(m, instance, attributes[5]), 6);
    if (geomgroup.empty())
    {
        mju_error("Invalid or missing parameters '%s' in skin sensor plugin",
                  attributes[5]);
        return nullptr;
    }
    for (size_t i = 0; i < 6; ++i)
        cfg.geom_group[i] = mjtByte(geomgroup[i]);

    // ---
    // setup mujoco ids
    int id = -1;
    for (id = 0; id < m->nsensor; ++id)
    {
        if (m->sensor_type[id] == mjSENS_PLUGIN &&
            m->sensor_plugin[id] == instance)
        {
            break;
        }
    }
    cfg.sensor_id = id;
    cfg.obj_type = m->sensor_objtype[id];
    cfg.obj_id = m->sensor_objid[id];
    if (cfg.obj_type == mjOBJ_SITE)
    {
        cfg.body_id = m->site_bodyid[cfg.obj_id];
    }
    else if (cfg.obj_type == mjOBJ_BODY)
    {
        cfg.body_id = cfg.obj_id;
    }
    else
    {
        mju_error("Cant find attached body");
        return nullptr;
    }
    cfg.m = m;
    cfg.d = d;

    // ---
    // create class
    return new SkinSensorPlugin(cfg);
}

void SkinSensorPlugin::RegisterPlugin()
{
    mjpPlugin plugin;
    mjp_defaultPlugin(&plugin);

    plugin.name = "mujoco.sensor.skin";
    plugin.capabilityflags |= mjPLUGIN_SENSOR;

    plugin.nattribute = attributes.size();
    plugin.attributes = attributes.data();

    // Stateless.
    plugin.nstate = +[](const mjModel* m, int instance) { return 0; };

    // Sensor dimension = num cells x 12
    plugin.nsensordata = +[](const mjModel* m, int instance, int sensor_id)
    {
        auto pkg_name =
            std::string(mj_getPluginConfig(m, instance, attributes[1]));
        if (pkg_name.empty())
        {
            mju_error(
                "Invalid or missing parameters '%s' in skin sensor plugin",
                attributes[1]);
            return 0;
        }
        auto config_name =
            std::string(mj_getPluginConfig(m, instance, attributes[2]));
        if (config_name.empty())
        {
            mju_error(
                "Invalid or missing parameters '%s' in skin sensor plugin",
                attributes[2]);
            return 0;
        }
        skin_sensor::SkinPatchConfig patch_cfg;
        if (!read_skin_cfg_from_pkg(pkg_name, config_name, patch_cfg))
        {
            mju_error(
                "Failed to read skin patch config from pkg='%s', name='%s'",
                pkg_name.c_str(), config_name.c_str());
            return 0;
        }

        // reserve memory for skin data:
        // pos (3) + quat (4) + proximity (1) + force (1) + accelerometer (3)
        return int(patch_cfg.cells.size()) * 12;
    };

    // Can only run after forces have been computed.
    plugin.needstage = mjSTAGE_ACC;

    // Initialization callback.
    plugin.init = +[](const mjModel* m, mjData* d, int instance)
    {
        auto* sensor = SkinSensorPlugin::Create(m, d, instance);
        if (!sensor)
        {
            return -1;
        }
        d->plugin_data[instance] = reinterpret_cast<uintptr_t>(sensor);
        return 0;
    };

    // Destruction callback.
    plugin.destroy = +[](mjData* d, int instance)
    {
        delete reinterpret_cast<SkinSensorPlugin*>(d->plugin_data[instance]);
        d->plugin_data[instance] = 0;
    };

    // Reset callback.
    plugin.reset = +[](const mjModel* m, double* plugin_state,
                       void* plugin_data, int instance)
    {
        auto* sensor = reinterpret_cast<class SkinSensorPlugin*>(plugin_data);
        sensor->Reset(m, instance);
    };

    // Destruction callback.
    plugin.destroy = +[](mjData* d, int instance)
    {
        delete reinterpret_cast<SkinSensorPlugin*>(d->plugin_data[instance]);
        d->plugin_data[instance] = 0;
    };

    // Reset callback.
    plugin.reset = +[](const mjModel* m, double* plugin_state,
                       void* plugin_data, int instance)
    {
        auto* sensor = reinterpret_cast<class SkinSensorPlugin*>(plugin_data);
        sensor->Reset(m, instance);
    };

    // Compute callback.
    plugin.compute =
        +[](const mjModel* m, mjData* d, int instance, int capability_bit)
    {
        auto* sensor =
            reinterpret_cast<class SkinSensorPlugin*>(d->plugin_data[instance]);
        sensor->Compute(m, d, instance);
    };

    // Visualization callback.
    plugin.visualize = +[](const mjModel* m, mjData* d, const mjvOption* opt,
                           mjvScene* scn, int instance)
    {
        auto* sensor =
            reinterpret_cast<class SkinSensorPlugin*>(d->plugin_data[instance]);
        sensor->Visualize(m, d, opt, scn, instance);
    };

    // Register the plugin.
    mjp_registerPlugin(&plugin);
}

void SkinSensorPlugin::Reset(const mjModel* m, int instance)
{
}

void SkinSensorPlugin::Compute(const mjModel* m, mjData* d, int instance)
{
    if(d->time - prev_time_ < dt_)
        return;
    prev_time_ = d->time;

    mj_markStack(d);
    sensor_->compute();
    mj_freeStack(d);
}

void SkinSensorPlugin::Visualize(const mjModel* m, mjData* d,
                                 const mjvOption* opt, mjvScene* scn,
                                 int instance)
{
    mj_markStack(d);
    sensor_->visualize(scn);
    mj_freeStack(d);
}

SkinSensorPlugin::SkinSensorPlugin(const SkinSensorCfg& cfg)
{
    sensor_ = std::make_unique<SkinSensor>(cfg);
    prev_time_ = -1.0;
    dt_ = cfg.update_period;
}

}  // namespace mujoco::plugin::sensor
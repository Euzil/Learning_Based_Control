#ifndef MUJOCO_SKIN_UTILITIES_H_
#define MUJOCO_SKIN_UTILITIES_H_

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <mujoco/mujoco.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace skin_sensor
{

    /**
     * @brief SkinCell configuration structure
     */
    struct SkinCellConfig 
    {
        int id;
        Eigen::Affine3d T_cell_root;
    };


    /**
     * @brief SkinPatch configuration structure
     */
    struct SkinPatchConfig 
    {
        int id;
        std::string base_frame;
        std::string topic_prefix;
        Eigen::Affine3d T_base_rsc;
        std::vector<SkinCellConfig> cells;
    };


    // Helper function to convert YAML 2D array to Eigen::Affine3d
    inline Eigen::Affine3d parseAffine3d(const YAML::Node& node) 
    {
        Eigen::Matrix4d matrix;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                matrix(i, j) = node[i][j].as<double>();
            }
        }
        Eigen::Affine3d affine;
        affine.matrix() = matrix;
        return affine;
    }


    inline SkinPatchConfig read_skin_cfg_from_file(const std::string& file)
    {
        SkinPatchConfig patch;
        YAML::Node config = YAML::LoadFile(file);

        // Parse Driver Settings (Prefix)
        if (config["sp_driver_settings"]) {
            patch.topic_prefix = config["sp_driver_settings"]["prefix"].as<std::string>();
        }

        // Parse Extrinsics
        if (config["sp_extrinsics"]) {
            const auto& ext = config["sp_extrinsics"];
            patch.id = ext["sp_id"].as<int>();
            patch.base_frame = ext["base_frame"].as<std::string>();
            patch.T_base_rsc = parseAffine3d(ext["tf_base_rsc"]);
        }

        // Parse Patch Data (Cells and Neighbors)
        if (config["sc_patch"]) {
            const auto& patch_node = config["sc_patch"];

            // Parse individual cell poses
            for (const auto& pose_node : patch_node["sc_poses"]) {
                SkinCellConfig cell;
                cell.id = pose_node["sc_id"].as<int>();
                cell.T_cell_root = parseAffine3d(pose_node["htf"]); 
                patch.cells.push_back(cell);
            }
        }
        return patch;
    }


    /**
     * @brief read the skin config from given ros pkg
     */
    inline bool read_skin_cfg_from_pkg(
        const std::string& pkg_name, 
        const std::string& cfg_name, 
        skin_sensor::SkinPatchConfig& cfg)
    {
        // Find the full package path
        std::string pkg_path;
        try
        {
            pkg_path = ament_index_cpp::get_package_share_directory(pkg_name);
        }
        catch (const std::exception &e)
        {
            mju_error("read_skin_cfg_from_pkg: Failed to find ros pkg: '%s'", pkg_name.c_str());
            return false;
        }
        // load the configuration file
        std::string cfg_path = pkg_path + '/' + cfg_name;
        try
        {
            cfg = skin_sensor::read_skin_cfg_from_file(cfg_path);
        }
        catch (const std::exception &e)
        {
            mju_error("read_skin_cfg_from_pkg: Failed to load file: '%s'", cfg_path.c_str());
            return false;
        }
        return true;
    }

}

#endif
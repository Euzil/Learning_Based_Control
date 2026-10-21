#ifndef SKIN_FEATURE_GENERATOR_CONFIG_LOADER_H_
#define SKIN_FEATURE_GENERATOR_CONFIG_LOADER_H_

#include <string>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace skin_feature_generator
{

struct FeatureConfig 
{
    std::string name;
    std::vector<std::string> patches;
};

class ConfigLoader
{
   public:
    // Adaptive normalization parameters
    double norm_prox_tau_fast_sec;          // chase new extreme: lower bound down, upper bound up
    double norm_prox_tau_lb_up_sec;         // lower bound recovery upward (moderate, ~10 s)
    double norm_prox_tau_ub_down_sec;       // upper bound relaxation downward (long, ~60 s)
    double norm_prox_min_range;             // raw-unit min denominator; prevents noise amplification

    // Binary threshold parameters
    double binary_prox_thres;
    double binary_force_thres;

    /**
     * @brief Construct a new Config Loader object and attempt to load the YAML
     */
    explicit ConfigLoader(const std::string& filepath)
    {
        load(filepath);
    }

   private:
    void load(const std::string& filepath)
    {
        try
        {
            // Attempt to load the file
            YAML::Node config = YAML::LoadFile(filepath);

            // Helper lambda to safely extract a double and throw a descriptive
            // error if missing
            auto get_double = [&config,
                               &filepath](const std::string& key) -> double
            {
                if (!config[key])
                {
                    throw std::runtime_error(
                        "Config Error: Missing required key '" + key + "' in " +
                        filepath);
                }
                return config[key].as<double>();
            };

            // Parse Adaptive normalization
            norm_prox_tau_fast_sec         = get_double("norm_prox_tau_fast_sec");
            norm_prox_tau_lb_up_sec     = get_double("norm_prox_tau_lb_up_sec");
            norm_prox_tau_ub_down_sec = get_double("norm_prox_tau_ub_down_sec");
            norm_prox_min_range            = get_double("norm_prox_min_range");

            // Parse Binary thresholds
            binary_prox_thres = get_double("binary_prox_thres");
            binary_force_thres = get_double("binary_force_thres");
        }
        catch (const YAML::BadFile& e)
        {
            throw std::runtime_error(
                "Config Error: Failed to find or read file: " + filepath);
        }
        catch (const YAML::ParserException& e)
        {
            throw std::runtime_error("Config Error: Malformed YAML syntax in " +
                                     filepath + " -> " + e.what());
        }
        catch (const YAML::BadConversion& e)
        {
            throw std::runtime_error(
                "Config Error: Data type mismatch (expected a number) in " +
                filepath + " -> " + e.what());
        }
    }
};

}

#endif
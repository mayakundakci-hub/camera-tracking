#pragma once

#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

class Config {
public:
    // Locates config.json by walking up from the current working directory,
    // then chdirs into whatever directory it was found in. Every relative
    // path *inside* the config (calibration_file, logger.output_dir, ...) is
    // written as if config.json's directory were the cwd, so without this
    // chdir those paths would silently break whenever the exe is launched
    // from somewhere other than the repo root (e.g. a debugger's default
    // working directory of the build output folder).
    static const nlohmann::json& load(const std::string& path = "config.json")
    {
        static nlohmann::json cfg = loadFromDisk(findConfigFile(path));
        return cfg;
    }

private:
    static nlohmann::json loadFromDisk(const std::string& path)
    {
        std::ifstream f(path);
        if (!f.is_open())
            throw std::runtime_error("Config: could not open " + path);
        nlohmann::json j;
        f >> j;
        return j;
    }

    static std::string findConfigFile(const std::string& filename)
    {
        std::string candidate = filename;
        for (int i = 0; i < 5; ++i)
        {
            std::ifstream test(candidate);
            if (test.good())
            {
                const auto dir = std::filesystem::absolute(candidate).parent_path();
                std::filesystem::current_path(dir);
                return filename;  // now resolvable directly from the new cwd
            }
            candidate = "../" + candidate;
        }
        throw std::runtime_error("Config: could not find " + filename +
                                  " within 5 parent directories");
    }
};

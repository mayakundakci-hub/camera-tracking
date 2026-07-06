#pragma once

#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>
#include <string>

class Config {
public:
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
            if (test.good()) return candidate;
            candidate = "../" + candidate;
        }
        throw std::runtime_error("Config: could not find " + filename +
                                  " within 5 parent directories");
    }
};

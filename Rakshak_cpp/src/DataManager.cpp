#include "DataManager.h"
#include <fstream>
#include <iostream>

DataManager::DataManager() {}

void DataManager::load_all() {
    load_fov_data("data/fov_data_m.json");
    load_pid_data("data/pid_data.json");
    load_depth_data("data/person_depth_data.json", "person");
    load_depth_data("data/target_depth_data.json", "target");
}

void DataManager::load_fov_data(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "Error opening " << path << std::endl;
        return;
    }
    json data = json::parse(f);
    auto fov_root = data["fov_data"];

    auto process_fov = [&](const json& j, std::map<int, ZoomFOV>& target_map) {
        for (auto& [key, value] : j.items()) {
            ZoomFOV fov;
            fov.horizontal = value["horizontal"].get<std::vector<double>>();
            fov.vertical = value["vertical"].get<std::vector<double>>();
            target_map[std::stoi(key)] = fov;
        }
    };

    process_fov(fov_root["static_fov"], static_fov_map);
    process_fov(fov_root["pid_fov"], pid_fov_map);
}

void DataManager::load_pid_data(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) return;
    json data = json::parse(f);
    auto pid_root = data["pid_data"];

    for (auto& [key, value] : pid_root.items()) {
        ZoomPID pid;
        pid.yaw_i = value[0].get<double>();
        pid.pitch_i = value[1].get<double>();
        pid_data_map[std::stoi(key)] = pid;
    }
}

void DataManager::load_depth_data(const std::string& path, const std::string& target_type) {
    std::ifstream f(path);
    if (!f.is_open()) return;
    json data = json::parse(f);
    std::string root_key = (target_type == "person") ? "person_data" : "target_data";
    auto depth_root = data[root_key];

    for (auto& [key, value] : depth_root.items()) {
        ZoomDepth zd;
        zd.h = value["h"].get<std::vector<double>>();
        zd.depth = value["depth"].get<std::vector<double>>();
        depth_data_maps[target_type][std::stoi(key)] = zd;
    }
}

ZoomFOV DataManager::get_static_fov(int zoom_level) const {
    if (static_fov_map.count(zoom_level)) return static_fov_map.at(zoom_level);
    return {{ -31.87, 31.87 }, { -19.23, 19.23 }}; // Default fallback
}

ZoomFOV DataManager::get_pid_fov(int zoom_level) const {
    if (pid_fov_map.count(zoom_level)) return pid_fov_map.at(zoom_level);
    return {{ -31.87, 31.87 }, { -19.22, 19.22 }};
}

ZoomPID DataManager::get_pid_data(int zoom_level) const {
    if (pid_data_map.count(zoom_level)) return pid_data_map.at(zoom_level);
    return { 2.0, 3.1 }; // Default fallback
}

double DataManager::get_depth_estimate(int zoom_level, double detected_height, const std::string& target_type) const {
    if (depth_data_maps.count(target_type) && depth_data_maps.at(target_type).count(zoom_level)) {
        const auto& zd = depth_data_maps.at(target_type).at(zoom_level);
        
        std::vector<MathUtils::Point> points;
        for (size_t i = 0; i < zd.h.size(); ++i) {
            points.push_back({ zd.h[i], zd.depth[i] });
        }

        // Sort by h (detected_height)
        std::sort(points.begin(), points.end(), [](const MathUtils::Point& a, const MathUtils::Point& b) {
            return a.x < b.x;
        });

        return MathUtils::interpolateLinear(points, detected_height);
    }
    return 0.0;
}

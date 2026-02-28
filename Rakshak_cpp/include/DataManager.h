#pragma once

#include <string>
#include <vector>
#include <map>
#include <nlohmann/json.hpp>
#include "MathUtils.h"

using json = nlohmann::json;

struct ZoomFOV {
    std::vector<double> horizontal;
    std::vector<double> vertical;
};

struct ZoomPID {
    double yaw_i;
    double pitch_i;
};

struct ZoomDepth {
    std::vector<double> h;
    std::vector<double> depth;
    // x, y, w also exist in JSON but h/depth are used for interpolation
};

class DataManager {
public:
    DataManager();
    ~DataManager() = default;

    void load_all();
    
    // FOV Access
    ZoomFOV get_static_fov(int zoom_level) const;
    ZoomFOV get_pid_fov(int zoom_level) const;

    // PID Access
    ZoomPID get_pid_data(int zoom_level) const;

    // Depth Estimation
    double get_depth_estimate(int zoom_level, double detected_height, const std::string& target_type) const;

private:
    void load_fov_data(const std::string& path);
    void load_pid_data(const std::string& path);
    void load_depth_data(const std::string& path, const std::string& target_type);

    std::map<int, ZoomFOV> static_fov_map;
    std::map<int, ZoomFOV> pid_fov_map;
    std::map<int, ZoomPID> pid_data_map;
    
    // Map<target_type, Map<zoom_level, ZoomDepth>>
    std::map<std::string, std::map<int, ZoomDepth>> depth_data_maps;
};

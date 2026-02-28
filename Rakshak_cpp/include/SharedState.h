#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <atomic>
#include <mutex>
#include <shared_mutex>
#include <map>
#include "OpMode.h"
#include "Globals.h"

struct DetectedObject {
    int id;
    int cls;
    double fx; // Center x
    double fy; // Center y
    double w;
    double h;
    double hh; // Head height
    double distance;
    double latitude;
    double longitude;
};

class SharedState {
public:
    SharedState();
    ~SharedState() = default;

    // Prevention of copying
    SharedState(const SharedState&) = delete;
    SharedState& operator=(const SharedState&) = delete;

    // --- State Variables (Atomic for thread safety) ---
    std::atomic<bool> stop_flag{false};
    std::atomic<OpMode> mode{OpMode::SEMI_AUTO};
    std::atomic<int> zoom_level{1};
    std::atomic<int> target_id{-1};
    
    std::atomic<bool> top_target_locked{false};
    std::atomic<bool> static_position_sent{false};
    std::atomic<bool> pid_enabled{true};
    std::atomic<bool> zoom_set{false};
    std::atomic<bool> digital_zoom_enabled{false};
    std::atomic<bool> calib_set{false};
    std::atomic<bool> stop_motor_flag{false};
    
    std::atomic<double> manual_x{0.0};
    std::atomic<double> manual_y{0.0};
    
    std::atomic<double> current_yaw_deg{0.0};
    std::atomic<double> current_pitch_deg{0.0};

    // --- Complex Objects (Mutex protected) ---
    void set_detections(const std::vector<DetectedObject>& new_detections);
    std::vector<DetectedObject> get_detections();

    // --- Helper Methods ---
    static double map_value(double min1, double max1, double min2, double max2, double value);

private:
    std::vector<DetectedObject> detected_objects;
    mutable std::shared_mutex detections_mutex;
};

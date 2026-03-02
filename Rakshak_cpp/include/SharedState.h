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

    // --- Motor Feedback (Actual physical state) ---
    std::atomic<double> actual_yaw_pos{0.0};
    std::atomic<double> actual_yaw_vel{0.0};
    std::atomic<double> actual_pitch_pos{0.0};
    std::atomic<double> actual_pitch_vel{0.0};
    std::atomic<bool> motors_ready{false};

    // --- Firing Control ---
    std::atomic<double> auto_target_hold_time{0.2};
    std::atomic<double> burst_time{1.0};
    std::atomic<bool> fire_status{false};
    std::atomic<bool> solenoid_triggered{false};
    std::atomic<double> hold_start_time{0.0};
    std::atomic<int> acc_percentage_threshold{5}; // 5% error threshold

    // --- Complex Objects (Mutex protected) ---
    void set_detections(const std::vector<DetectedObject>& new_detections);
    std::vector<DetectedObject> get_detections();

    // --- State Persistence ---
    void load_state(const std::string& path);
    void persist_state(const std::string& path);

    // --- Helper Methods ---
    static double map_value(double min1, double max1, double min2, double max2, double value);

private:
    std::vector<DetectedObject> detected_objects;
    mutable std::shared_mutex detections_mutex;
};

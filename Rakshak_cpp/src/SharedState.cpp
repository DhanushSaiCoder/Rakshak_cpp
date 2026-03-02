#include "SharedState.h"
#include "StateStore.h"
#include <cmath>

SharedState::SharedState() {
    // Initialization of atomic variables is handled in the header
}

void SharedState::set_detections(const std::vector<DetectedObject>& new_detections) {
    std::unique_lock<std::shared_mutex> lock(detections_mutex);
    detected_objects = new_detections;
}

std::vector<DetectedObject> SharedState::get_detections() {
    std::shared_lock<std::shared_mutex> lock(detections_mutex);
    return detected_objects;
}

double SharedState::map_value(double min1, double max1, double min2, double max2, double value) {
    if (std::abs(max1 - min1) < 1e-9) {
        return min2;
    }
    return (value - min1) * (max2 - min2) / (max1 - min1) + min2;
}

void SharedState::load_state(const std::string& path) {
    RuntimeState rs;
    if (StateStore::load(path, rs)) {
        mode = static_cast<OpMode>(rs.op_mode);
        zoom_level = rs.zoom_level;
        pid_enabled = rs.pid_enable;
        auto_target_hold_time = rs.auto_target_hold_time;
        burst_time = rs.short_burst_value; // mapping short burst to default burst
        acc_percentage_threshold = rs.auto_accuracy_value;
        current_yaw_deg = rs.yaw_value;
        current_pitch_deg = rs.pitch_value;
    }
}

void SharedState::persist_state(const std::string& path) {
    RuntimeState rs;
    rs.op_mode = static_cast<int>(mode.load());
    rs.zoom_level = zoom_level.load();
    rs.pid_enable = pid_enabled.load();
    rs.auto_target_hold_time = auto_target_hold_time.load();
    rs.short_burst_value = burst_time.load();
    rs.auto_accuracy_value = acc_percentage_threshold.load();
    rs.yaw_value = current_yaw_deg.load();
    rs.pitch_value = current_pitch_deg.load();
    
    StateStore::save(path, rs);
}

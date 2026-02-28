#include "SharedState.h"
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

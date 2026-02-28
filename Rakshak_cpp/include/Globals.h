#pragma once
#include <string>
#include <vector>
#include <map>

namespace Globals {
    // Kp scales
    inline constexpr double KP_SCALE_YAW = 10.0;
    inline constexpr double KP_SCALE_PITCH = 10.0;

    inline constexpr double REVOLUTION = 360.0;
    inline constexpr double YAW_REDUCTION = 27.0;
    inline constexpr double PITCH_REDUCTION = 192.0;
    inline constexpr double CAM_REDUCTION = 30.0;

    inline constexpr double VELOCITY_LIMIT = 50.0; // Default fallback
    inline constexpr double ACCELERATION_LIMIT = 50.0;

    inline constexpr double YAW_VELOCITY_LIMIT = 7.19;
    inline constexpr double YAW_ACCELERATION_LIMIT = 7.19;
    inline constexpr double PITCH_VELOCITY_LIMIT = 51.2;
    inline constexpr double PITCH_ACCELERATION_LIMIT = 51.2;

    inline constexpr int YAW_POS_CONST = 1;
    inline constexpr int PITCH_POS_CONST = 1;

    inline constexpr double YAW_LIMIT = 34.0;   // degrees of 1x horizontal FOV
    inline constexpr double PITCH_LIMIT = 23.0; // degrees of 1x vertical FOV

    const std::string TOP_SERIAL = "USB3Neo_09936";
    const std::string STATIC_SERIAL = "USB3Neo_09247";
    const std::string LEFT_SERIAL = "USB3Neo_09936";
    const std::string RIGHT_SERIAL = "USB3Neo_07703";
    const std::string THERMAL_SERIAL = "TV251964YF003";

    const std::string TRIGGER_PORT = "/dev/arduino_trigger";
    inline constexpr double SHORT_HOLD_TIME = 2.0;
    inline constexpr double LONG_HOLD_TIME = 6.0;
    inline constexpr double SHORT_BURST_TIME = 1.0;
    inline constexpr double LONG_BURST_TIME = 3.0;
    inline constexpr double KNOWN_HEIGHT = 1.7; // Meters

    // Model paths
    const std::string MODELS_DIR = "models";
    const std::string DEFAULT_MODEL = "models/yolo11n.engine"; // Prefer engine for C++/TensorRT
    const std::string FACE_MODEL = "models/yolov11n-face.engine";
    const std::string FIG_MODEL = "models/fig11_best_11n_12k_without_params.engine";

    // Sensor data variables
    const std::string SYNC_WORD = "START";
    inline constexpr int BAUD_RATE = 250000;
    const std::string SENSORS_PORT = "/dev/arduino_sensors";

    const std::string TARGET_TYPE = "person"; // "person" or "target"

    // MQ Base Variables
    const std::string BASE = "rakshak";
    const std::string SYSTEM_ID = "system1";
    const std::string RABBIT_EXCHANGE = "tracking.events";
    const std::string RABBIT_HOST = "192.168.1.100";
    inline constexpr int RABBIT_PORT = 5672;
    const std::string RABBIT_USER = "jetson";
    const std::string RABBIT_PASS = "123";
}

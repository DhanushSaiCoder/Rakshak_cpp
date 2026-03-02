#include "RakshakSystem.h"
#include <iostream>
#include <sstream>
#include <chrono>
#include <cmath>

RakshakSystem::RakshakSystem() {
    data_manager.load_all();
    
    // Load persistent state
    state.load_state("data/runtime_state.json");

    static_cam = std::make_unique<CameraReader>("0", "StaticCam");
    top_cam = std::make_unique<CameraReader>("1", "TopCam");
    streamer = std::make_unique<MJPEGStreamer>(8443);
    trigger = std::make_unique<TriggerController>(Globals::TRIGGER_PORT);
    zoom_ctrl = std::make_unique<ZoomController>();
    motors = std::make_unique<MotorController>(state);
    pid = std::make_unique<PIDController>();
    yolo = std::make_unique<YoloEngine>(Globals::DEFAULT_MODEL);
    mq_bridge = std::make_unique<MessageBridge>(state, *motors);
    sensors = std::make_unique<SensorReader>(Globals::SENSORS_PORT, Globals::BAUD_RATE);
}

RakshakSystem::~RakshakSystem() {
    stop();
}

bool RakshakSystem::start() {
    if (running) return true;
    
    bool static_ready = static_cam->start();
    if (!static_ready) {
        std::cerr << "[SYSTEM][FATAL] Static Camera (Required) failed to start." << std::endl;
    }

    bool top_ready = top_cam->start();
    if (!top_ready) {
        std::cout << "[SYSTEM][WARN] Top Camera (Optional) not found. Using fallback." << std::endl;
    }

    sensors->connect();
    streamer->start();
    mq_bridge->start();
    
    bool yolo_ready = yolo->load_model();
    if (!yolo_ready) {
        std::cerr << "[SYSTEM][FATAL] YOLO Inference (Required) failed to initialize." << std::endl;
    }

    if (!static_ready || !yolo_ready) {
        return false;
    }

    running = true;
    vision_thread = std::thread(&RakshakSystem::vision_thread_func, this);
    control_thread = std::thread(&RakshakSystem::control_thread_func, this);
    health_thread = std::thread(&RakshakSystem::health_thread_func, this);

    return true;
}

void RakshakSystem::stop() {
    running = false;
    if (vision_thread.joinable()) vision_thread.join();
    if (control_thread.joinable()) control_thread.join();
    if (health_thread.joinable()) health_thread.join();

    mq_bridge->stop();
    sensors->disconnect();
    static_cam->stop();
    top_cam->stop();
    streamer->stop();
    
    // Save state on exit
    state.persist_state("data/runtime_state.json");
}

void RakshakSystem::vision_thread_func() {
    while (running) {
        cv::Mat frame = static_cam->get_latest_frame();
        if (!frame.empty()) {
            std::vector<DetectedObject> detections = yolo->detect(frame);
            
            for (const auto& obj : detections) {
                int x1 = static_cast<int>(obj.fx - obj.w / 2);
                int y1 = static_cast<int>(obj.fy - obj.h / 2);
                
                cv::Scalar color = (obj.id == state.target_id.load()) ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
                cv::rectangle(frame, cv::Rect(x1, y1, static_cast<int>(obj.w), static_cast<int>(obj.h)), color, 2);
                
                std::string label = "ID: " + std::to_string(obj.id);
                cv::putText(frame, label, cv::Point(x1, y1 - 10), cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
            }

            state.set_detections(detections);
            streamer->set_frame("static_cam", frame);
            
            if (!top_cam->is_running()) {
                streamer->set_frame("top_cam", frame);
            }
        }

        if (top_cam->is_running()) {
            cv::Mat top_frame = top_cam->get_latest_frame();
            if (!top_frame.empty()) {
                streamer->set_frame("top_cam", top_frame);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void RakshakSystem::control_thread_func() {
    auto last_time = std::chrono::steady_clock::now();
    double prev_Ix = 0, prev_Iy = 0;

    while (running) {
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last_time).count();
        last_time = now;

        OpMode mode = state.mode.load();
        auto detections = state.get_detections();
        
        DetectedObject* target = nullptr;
        int tid = state.target_id.load();
        if (tid != -1) {
            for (auto& d : detections) {
                if (d.id == tid) {
                    target = &d;
                    break;
                }
            }
        }

        if (target) {
            int zoom = state.zoom_level.load();
            ZoomFOV fov = data_manager.get_static_fov(zoom);
            
            double error_x = target->fx - (1920 / 2);
            double error_y = target->fy - (1080 / 2);

            if (!state.static_position_sent) {
                auto [y_cmd, p_cmd] = pid->calculate_pos_cmd(1920/2, 1080/2, 
                                                            fov.horizontal[0], fov.horizontal[1],
                                                            fov.vertical[0], fov.vertical[1],
                                                            error_x, error_y);
                motors->set_position(y_cmd, p_cmd, Globals::VELOCITY_LIMIT, Globals::ACCELERATION_LIMIT);
                state.static_position_sent = true;
                state.hold_start_time = 0.0;
            } else {
                PIDResult res = pid->calculate(error_x, error_y, dt, prev_Ix, prev_Iy, zoom, data_manager);
                prev_Ix = res.yaw_Ix;
                prev_Iy = res.pitch_Iy;
                
                motors->set_position(state.current_yaw_deg.load() + res.yaw_IX, 
                                     state.current_pitch_deg.load() + res.pitch_IY,
                                     Globals::VELOCITY_LIMIT, Globals::ACCELERATION_LIMIT);

                double hit_acc_x = std::abs(error_x) / (1920.0 / 2.0) * 100.0;
                double hit_acc_y = std::abs(error_y) / (1080.0 / 2.0) * 100.0;

                if (hit_acc_x < state.acc_percentage_threshold.load() && hit_acc_y < state.acc_percentage_threshold.load()) {
                    if (state.hold_start_time.load() == 0.0) {
                        state.hold_start_time = std::chrono::duration<double>(now.time_since_epoch()).count();
                    } else {
                        double hold_duration = std::chrono::duration<double>(now.time_since_epoch()).count() - state.hold_start_time.load();
                        
                        if (hold_duration >= state.auto_target_hold_time.load() && !state.fire_status.load()) {
                            std::cout << "[CONTROL] TARGET LOCKED. FIRING!" << std::endl;
                            std::thread([this]() {
                                state.fire_status = true;
                                trigger->solenoid_on();
                                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(state.burst_time.load() * 1000)));
                                trigger->solenoid_off();
                                state.fire_status = false;
                                state.solenoid_triggered = true;
                            }).detach();
                            
                            if (mode == OpMode::AUTO) {
                                state.target_id = -1; 
                            }
                        }
                    }
                } else {
                    state.hold_start_time = 0.0;
                }
            }
        } else {
            state.static_position_sent = false;
            prev_Ix = 0; prev_Iy = 0;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void RakshakSystem::health_thread_func() {
    while (running) {
        MotorState yaw = motors->get_yaw_state();
        MotorState pitch = motors->get_pitch_state();
        
        SensorData s_data;
        if (sensors->read_latest(s_data)) {
            // Future: state.latitude = s_data.latitude;
        }

        std::stringstream ss;
        ss << "[{\"motor_id\":2, \"position\":" << yaw.position << "}, "
           << "{\"motor_id\":1, \"position\":" << pitch.position << "}]";
        
        mq_bridge->publish("health", "health_data", ss.str());

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

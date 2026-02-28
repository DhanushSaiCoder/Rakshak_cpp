#include "RakshakSystem.h"
#include <iostream>

RakshakSystem::RakshakSystem() {
    data_manager.load_all();
    
    static_cam = std::make_unique<CameraReader>("0", "StaticCam");
    top_cam = std::make_unique<CameraReader>("1", "TopCam");
    streamer = std::make_unique<MJPEGStreamer>(8443);
    trigger = std::make_unique<TriggerController>(Globals::TRIGGER_PORT);
    zoom_ctrl = std::make_unique<ZoomController>();
    motors = std::make_unique<MotorController>(state);
    pid = std::make_unique<PIDController>();
    yolo = std::make_unique<YoloEngine>(Globals::DEFAULT_MODEL);
}

RakshakSystem::~RakshakSystem() {
    stop();
}

void RakshakSystem::start() {
    if (running) return;
    running = true;

    static_cam->start();
    top_cam->start();
    streamer->start();
    yolo->load_model();

    vision_thread = std::thread(&RakshakSystem::vision_thread_func, this);
    control_thread = std::thread(&RakshakSystem::control_thread_func, this);
    health_thread = std::thread(&RakshakSystem::health_thread_func, this);
}

void RakshakSystem::stop() {
    running = false;
    if (vision_thread.joinable()) vision_thread.join();
    if (control_thread.joinable()) control_thread.join();
    if (health_thread.joinable()) health_thread.join();

    static_cam->stop();
    top_cam->stop();
    streamer->stop();
}

void RakshakSystem::vision_thread_func() {
    while (running) {
        cv::Mat frame = static_cam->get_latest_frame();
        if (!frame.empty()) {
            // Run Detection
            std::vector<DetectedObject> detections = yolo->detect(frame);
            state.set_detections(detections);
            
            // Feed streamer
            streamer->set_frame("static_cam", frame);
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

        if (mode == OpMode::SEMI_AUTO && !detections.empty()) {
            // Logic migrated from main.py semi-auto block
            DetectedObject target = detections[0]; // Simplification: track first object
            
            int zoom = state.zoom_level.load();
            ZoomFOV fov = data_manager.get_static_fov(zoom);
            
            // Calculate error (simplification)
            double error_x = target.fx - (1920 / 2); // Assuming 1920 width
            double error_y = target.fy - (1080 / 2);

            if (!state.static_position_sent) {
                auto [y_cmd, p_cmd] = pid->calculate_pos_cmd(1920/2, 1080/2, 
                                                            fov.horizontal[0], fov.horizontal[1],
                                                            fov.vertical[0], fov.vertical[1],
                                                            error_x, error_y);
                motors->set_position(y_cmd, p_cmd, Globals::VELOCITY_LIMIT, Globals::ACCELERATION_LIMIT);
                state.static_position_sent = true;
            } else {
                // Run PID fine tracking
                PIDResult res = pid->calculate(error_x, error_y, dt, prev_Ix, prev_Iy, zoom, data_manager);
                prev_Ix = res.yaw_Ix;
                prev_Iy = res.pitch_Iy;
                
                motors->set_position(state.current_yaw_deg + res.yaw_IX, 
                                     state.current_pitch_deg + res.pitch_IY,
                                     Globals::VELOCITY_LIMIT, Globals::ACCELERATION_LIMIT);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void RakshakSystem::health_thread_func() {
    while (running) {
        // Publish health data (migrated from publish_health in motor_controller.py)
        MotorState yaw = motors->get_yaw_state();
        MotorState pitch = motors->get_pitch_state();
        
        // In real app, publish to RabbitMQ here
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

#include "MotorController.h"
#include "Globals.h"
#include <iostream>
#include <chrono>

MotorController::MotorController(SharedState& shared_state) : state(shared_state) {
    driver = std::make_unique<MoteusDriver>("can0");
    if (driver->open()) {
        ready = true;
        state.motors_ready = true;
        std::cout << "[Motor] SocketCAN initialized on can0" << std::endl;
    } else {
        ready = false;
        state.motors_ready = false;
        std::cerr << "[Motor] Failed to initialize SocketCAN" << std::endl;
    }

    feedback_thread = std::thread(&MotorController::feedback_loop, this);
}

MotorController::~MotorController() {
    run_feedback = false;
    if (feedback_thread.joinable()) {
        feedback_thread.join();
    }
    stop_motors();
    if (driver) {
        driver->close();
    }
}

bool MotorController::stop_motors() {
    if (!ready) return false;
    bool s1 = driver->send_stop_cmd(YAW_ID);
    bool s2 = driver->send_stop_cmd(PITCH_ID);
    return s1 && s2;
}

bool MotorController::set_position(double yaw_pos, double pitch_pos, double vel_limit, double accel_limit) {
    if (!ready) return false;

    // reduction scaling (from Python)
    double yaw_moteus_pos = yaw_pos * (Globals::YAW_REDUCTION / 360.0);
    double pitch_moteus_pos = pitch_pos * (Globals::PITCH_REDUCTION / 360.0);

    bool s1 = driver->send_position_cmd(YAW_ID, yaw_moteus_pos, 0.0, 
                                        Globals::KP_SCALE_YAW, 1.0, 
                                        vel_limit, accel_limit);
    bool s2 = driver->send_position_cmd(PITCH_ID, pitch_moteus_pos, 0.0, 
                                        Globals::KP_SCALE_PITCH, 1.0, 
                                        vel_limit, accel_limit);
    
    // Update state for internal tracking (though feedback loop will also update it)
    state.current_yaw_deg = yaw_pos;
    state.current_pitch_deg = pitch_pos;
    
    return s1 && s2;
}

bool MotorController::set_velocity(double yaw_vel, double pitch_vel) {
    if (!ready) return false;
    
    bool s1 = driver->send_velocity_cmd(YAW_ID, yaw_vel, Globals::YAW_ACCELERATION_LIMIT);
    bool s2 = driver->send_velocity_cmd(PITCH_ID, pitch_vel, Globals::PITCH_ACCELERATION_LIMIT);
    
    return s1 && s2;
}

void MotorController::feedback_loop() {
    while (run_feedback) {
        if (ready) {
            MotorState y_s, p_s;
            if (driver->query_state(YAW_ID, y_s)) {
                std::lock_guard<std::mutex> lock(feedback_mutex);
                yaw_feedback = y_s;
                state.actual_yaw_pos = y_s.position * (360.0 / Globals::YAW_REDUCTION);
                state.actual_yaw_vel = y_s.velocity * (360.0 / Globals::YAW_REDUCTION);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            
            if (driver->query_state(PITCH_ID, p_s)) {
                std::lock_guard<std::mutex> lock(feedback_mutex);
                pitch_feedback = p_s;
                state.actual_pitch_pos = p_s.position * (360.0 / Globals::PITCH_REDUCTION);
                state.actual_pitch_vel = p_s.velocity * (360.0 / Globals::PITCH_REDUCTION);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

MotorState MotorController::get_yaw_state() {
    std::lock_guard<std::mutex> lock(feedback_mutex);
    return yaw_feedback;
}

MotorState MotorController::get_pitch_state() {
    std::lock_guard<std::mutex> lock(feedback_mutex);
    return pitch_feedback;
}

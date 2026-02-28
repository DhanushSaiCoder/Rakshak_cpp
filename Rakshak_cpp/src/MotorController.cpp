#include "MotorController.h"
#include <iostream>

MotorController::MotorController(SharedState& shared_state) : state(shared_state) {
    // In a real implementation, you would initialize moteus::Controller(id) here
    ready = true; 
}

bool MotorController::stop_motors() {
    // moteus command: set_stop()
    std::cout << "[Motor] Stop motors" << std::endl;
    return true;
}

bool MotorController::set_position(double yaw_pos, double pitch_pos, double vel_limit, double accel_limit) {
    // moteus command: set_position(position=yaw_pos, velocity_limit=vel_limit, accel_limit=accel_limit)
    // std::cout << "[Motor] Set position Yaw: " << yaw_pos << " Pitch: " << pitch_pos << std::endl;
    
    // Update state for feedback loops
    state.current_yaw_deg = yaw_pos;
    state.current_pitch_deg = pitch_pos;
    return true;
}

bool MotorController::set_velocity(double yaw_vel, double pitch_vel) {
    // moteus command: set_position(position=nan, velocity=yaw_vel)
    return true;
}

MotorState MotorController::get_yaw_state() {
    // In real implementation, return result of moteus query
    return { state.current_yaw_deg.load(), 0.0, 0.0, 24.0, 35.0, 0 };
}

MotorState MotorController::get_pitch_state() {
    return { state.current_pitch_deg.load(), 0.0, 0.0, 24.0, 35.0, 0 };
}

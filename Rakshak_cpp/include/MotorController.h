#pragma once

#include "SharedState.h"
#include <string>
#include <memory>

struct MotorState {
    double position;
    double velocity;
    double torque;
    double voltage;
    double temperature;
    int fault;
};

class MotorController {
public:
    MotorController(SharedState& shared_state);
    ~MotorController() = default;

    // Command methods
    bool stop_motors();
    bool set_position(double yaw_pos, double pitch_pos, double vel_limit, double accel_limit);
    bool set_velocity(double yaw_vel, double pitch_vel);
    
    // Feedback
    MotorState get_yaw_state();
    MotorState get_pitch_state();

    bool is_ready() const { return ready; }

private:
    SharedState& state;
    bool ready{false};

    // Moteus instances would go here
    // std::unique_ptr<moteus::Controller> yaw_motor;
    // std::unique_ptr<moteus::Controller> pitch_motor;
};

#pragma once

#include "SharedState.h"
#include "MoteusDriver.h"
#include <string>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>

class MotorController {
public:
    MotorController(SharedState& shared_state);
    ~MotorController();

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

    std::unique_ptr<MoteusDriver> driver;
    
    // Motor IDs (From Globals or typical setup)
    const uint32_t YAW_ID = 2;
    const uint32_t PITCH_ID = 1;

    // Feedback thread
    std::thread feedback_thread;
    std::atomic<bool> run_feedback{true};
    void feedback_loop();

    MotorState yaw_feedback;
    MotorState pitch_feedback;
    std::mutex feedback_mutex;
};

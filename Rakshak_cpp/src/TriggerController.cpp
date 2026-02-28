#include "TriggerController.h"
#include <thread>
#include <chrono>

TriggerController::TriggerController(const std::string& port, int baud_rate) 
    : serial(port, baud_rate) {
    if (serial.open_port()) {
        std::cout << "[INFO] TriggerController connected to " << port << std::endl;
        // Allow Arduino to initialize
        std::this_thread::sleep_for(std::chrono::seconds(2));
    } else {
        std::cerr << "[ERROR] TriggerController failed to open " << port << std::endl;
    }
}

void TriggerController::laser_on() {
    serial.write_data("8\n");
}

void TriggerController::laser_off() {
    serial.write_data("9\n");
}

void TriggerController::solenoid_on() {
    serial.write_data("6\n");
}

void TriggerController::solenoid_off() {
    serial.write_data("7\n");
}

void TriggerController::solenoid_on_off() {
    serial.write_data("5\n");
}

void TriggerController::set_solenoid_duration(int duration_ms) {
    std::string cmd = "D" + std::to_string(duration_ms) + "\n";
    serial.write_data(cmd);
}

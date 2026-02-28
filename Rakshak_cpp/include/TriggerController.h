#pragma once

#include "SerialPort.h"
#include <string>

class TriggerController {
public:
    TriggerController(const std::string& port, int baud_rate = 115200);
    ~TriggerController() = default;

    void laser_on();
    void laser_off();
    
    void solenoid_on();
    void solenoid_off();
    void solenoid_on_off();

    void set_solenoid_duration(int duration_ms);

private:
    SerialPort serial;
};

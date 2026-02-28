#pragma once

#include "SharedState.h"
#include "DataManager.h"
#include "Globals.h"

struct PIDResult {
    double yaw_IX;
    double pitch_IY;
    double yaw_Ix; // Accumulated integral
    double pitch_Iy;
};

class PIDController {
public:
    PIDController();
    ~PIDController() = default;

    PIDResult calculate(double error_x, double error_y, double delta_t, 
                        double prev_Ix, double prev_Iy, int zoom_level,
                        const DataManager& data_manager);

    std::pair<double, double> calculate_pos_cmd(int width, int height, 
                                               double hfov_min, double hfov_max,
                                               double vfov_min, double vfov_max,
                                               double offset_x, double error_y);

private:
    double manual_yaw_limit;
    double manual_pitch_limit;
};

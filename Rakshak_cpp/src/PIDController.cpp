#include "PIDController.h"
#include <cmath>

PIDController::PIDController() {
    manual_yaw_limit = SharedState::map_value(-Globals::REVOLUTION, Globals::REVOLUTION, 
                                              -Globals::YAW_REDUCTION, Globals::YAW_REDUCTION, 
                                              Globals::YAW_LIMIT);
    manual_pitch_limit = SharedState::map_value(-Globals::REVOLUTION, Globals::REVOLUTION, 
                                                -Globals::PITCH_REDUCTION, Globals::PITCH_REDUCTION, 
                                                Globals::PITCH_LIMIT);
}

PIDResult PIDController::calculate(double error_x, double error_y, double delta_t, 
                                   double prev_Ix, double prev_Iy, int zoom_level,
                                   const DataManager& data_manager) {
    ZoomPID gains = data_manager.get_pid_data(zoom_level);
    
    PIDResult res;
    res.yaw_Ix = prev_Ix + (error_x * delta_t);
    res.yaw_IX = gains.yaw_i * res.yaw_Ix;

    res.pitch_Iy = prev_Iy + (error_y * delta_t);
    res.pitch_IY = gains.pitch_i * res.pitch_Iy;

    return res;
}

std::pair<double, double> PIDController::calculate_pos_cmd(int width, int height, 
                                                           double hfov_min, double hfov_max,
                                                           double vfov_min, double vfov_max,
                                                           double offset_x, double error_y) {
    double h_map = SharedState::map_value(-width, width, hfov_min, hfov_max, offset_x);
    double v_map = SharedState::map_value(-height, height, vfov_min, vfov_max, error_y);

    // Clamp to limits
    if (std::abs(h_map) > Globals::YAW_LIMIT) h_map = (h_map > 0 ? 1 : -1) * Globals::YAW_LIMIT;
    if (std::abs(v_map) > Globals::PITCH_LIMIT) v_map = (v_map > 0 ? 1 : -1) * Globals::PITCH_LIMIT;

    double yaw_pos_cmd = SharedState::map_value(-Globals::REVOLUTION, Globals::REVOLUTION, 
                                                -Globals::YAW_REDUCTION, Globals::YAW_REDUCTION, h_map);
    double pitch_pos_cmd = SharedState::map_value(-Globals::REVOLUTION, Globals::REVOLUTION, 
                                                  -Globals::PITCH_REDUCTION, Globals::PITCH_REDUCTION, v_map);

    return {yaw_pos_cmd, pitch_pos_cmd};
}

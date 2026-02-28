#include "ZoomController.h"
#include <sstream>
#include <iomanip>

ZoomController::ZoomController() {
    // Initialize zoom level hex commands from zoom_controller.py
    zoom_levels[1] = "81 01 04 47 00 00 00 00 FF";
    zoom_levels[2] = "81 01 04 47 01 06 0A 01 FF";
    zoom_levels[3] = "81 01 04 47 02 00 06 03 FF";
    zoom_levels[4] = "81 01 04 47 02 06 02 08 FF";
    zoom_levels[5] = "81 01 04 47 02 0A 01 0D FF";
    zoom_levels[6] = "81 01 04 47 02 0D 01 03 FF";
    zoom_levels[7] = "81 01 04 47 02 0F 06 0D FF";
    zoom_levels[8] = "81 01 04 47 03 01 06 01 FF";
    zoom_levels[9] = "81 01 04 47 03 03 00 0D FF";
    zoom_levels[10] = "81 01 04 47 03 04 08 06 FF";
    zoom_levels[11] = "81 01 04 47 03 05 0D 07 FF";
    zoom_levels[12] = "81 01 04 47 03 07 00 09 FF";
    zoom_levels[13] = "81 01 04 47 03 08 02 00 FF";
    zoom_levels[14] = "81 01 04 47 03 09 02 00 FF";
    zoom_levels[15] = "81 01 04 47 03 0A 00 0A FF";
    zoom_levels[16] = "81 01 04 47 03 0A 0D 0D FF";
    zoom_levels[17] = "81 01 04 47 03 0B 09 0C FF";
    zoom_levels[18] = "81 01 04 47 03 0C 04 06 FF";
    zoom_levels[19] = "81 01 04 47 03 0C 0D 0C FF";
    zoom_levels[20] = "81 01 04 47 03 0D 06 00 FF";
    zoom_levels[21] = "81 01 04 47 03 0D 0D 04 FF";
    zoom_levels[22] = "81 01 04 47 03 0E 03 09 FF";
    zoom_levels[23] = "81 01 04 47 03 0E 09 00 FF";
    zoom_levels[24] = "81 01 04 47 03 0E 0D 0C FF";
    zoom_levels[25] = "81 01 04 47 03 0F 01 0E FF";
    zoom_levels[26] = "81 01 04 47 03 0F 05 07 FF";
    zoom_levels[27] = "81 01 04 47 03 0F 08 0A FF";
    zoom_levels[28] = "81 01 04 47 03 0F 0B 06 FF";
    zoom_levels[29] = "81 01 04 47 03 0F 0D 0C FF";
    zoom_levels[30] = "81 01 04 47 04 00 00 00 FF";
}

bool ZoomController::set_zoom(int zoom_level, const std::string& port_top, const std::string& port_static) {
    if (zoom_levels.count(zoom_level)) {
        std::cout << "[Zoom] Setting level " << zoom_level << std::endl;
        send_command(port_top, zoom_levels[zoom_level]);
        send_command(port_static, zoom_levels[zoom_level]);
        return true;
    }
    return false;
}

bool ZoomController::toggle_autofocus(const std::string& port_top, const std::string& port_static, bool enabled) {
    std::string cmd = enabled ? "81 01 04 38 02 FF" : "81 01 04 38 03 FF";
    send_command(port_top, cmd);
    send_command(port_static, cmd);
    return true;
}

bool ZoomController::toggle_stabilization(const std::string& port_top, const std::string& port_static, bool enabled) {
    std::string cmd = enabled ? "81 01 04 34 02 FF" : "81 01 04 34 03 FF";
    send_command(port_top, cmd);
    send_command(port_static, cmd);
    return true;
}

void ZoomController::send_command(const std::string& port, const std::string& hex_cmd) {
    SerialPort serial(port, 115200);
    if (serial.open_port()) {
        serial.write_data(hex_to_string(hex_cmd));
    }
}

std::string ZoomController::hex_to_string(const std::string& hex) {
    std::string out;
    std::stringstream ss(hex);
    std::string word;
    while (ss >> word) {
        out += static_cast<char>(std::stoi(word, nullptr, 16));
    }
    return out;
}

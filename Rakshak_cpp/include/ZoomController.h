#pragma once

#include "SerialPort.h"
#include <string>
#include <map>

class ZoomController {
public:
    ZoomController();
    ~ZoomController() = default;

    bool set_zoom(int zoom_level, const std::string& port_top, const std::string& port_static);
    bool toggle_autofocus(const std::string& port_top, const std::string& port_static, bool enabled);
    bool toggle_stabilization(const std::string& port_top, const std::string& port_static, bool enabled);

private:
    void send_command(const std::string& port, const std::string& hex_cmd);
    std::string hex_to_string(const std::string& hex);

    std::map<int, std::string> zoom_levels;
};

#pragma once

#include <string>
#include <iostream>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

class SerialPort {
public:
    SerialPort(const std::string& port, int baud_rate);
    ~SerialPort();

    bool open_port();
    void close_port();
    bool is_open() const { return fd != -1; }

    int write_data(const std::string& data);
    std::string read_line();
    int read_raw(uint8_t* buffer, int len);

private:
    std::string port_name;
    int baud;
    int fd;
};

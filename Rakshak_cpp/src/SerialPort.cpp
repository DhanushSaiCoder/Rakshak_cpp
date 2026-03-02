#include "SerialPort.h"
#include <cstring>

SerialPort::SerialPort(const std::string& port, int baud_rate) 
    : port_name(port), baud(baud_rate), fd(-1) {}

SerialPort::~SerialPort() {
    close_port();
}

bool SerialPort::open_port() {
    fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("open_port: Unable to open serial port");
        return false;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        return false;
    }

    speed_t speed;
    switch (baud) {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        case 250000: speed = B230400; break; // Closest standard or use custom
        default: speed = B115200; break;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        return false;
    }

    return true;
}

void SerialPort::close_port() {
    if (fd != -1) {
        close(fd);
        fd = -1;
    }
}

int SerialPort::write_data(const std::string& data) {
    if (fd == -1) return -1;
    return write(fd, data.c_str(), data.size());
}

std::string SerialPort::read_line() {
    if (fd == -1) return "";
    std::string line;
    char c;
    while (read(fd, &c, 1) > 0) {
        if (c == '\n') break;
        line += c;
    }
    return line;
}

int SerialPort::read_raw(uint8_t* buffer, int len) {
    if (fd == -1) return -1;
    return read(fd, buffer, len);
}

// macOS/Darwin specific serial implementation for JaQ Robot
// Platform-specific serial communication for SCServo

#include "SCServo.h"
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string>

int SCSerial::OpenPort(const char* portname, int baudrate) {
    // Open serial port for macOS/Darwin
    fd = open(portname, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1) {
        return -1;
    }
    
    struct termios options;
    tcgetattr(fd, &options);
    
    // Set baud rate
    speed_t speed;
    switch(baudrate) {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        default: speed = B115200; break;
    }
    
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);
    
    // Configure port settings
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;
    
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    options.c_oflag &= ~OPOST;
    
    tcsetattr(fd, TCSANOW, &options);
    
    return fd;
}

void SCSerial::ClosePort() {
    if (fd != -1) {
        close(fd);
        fd = -1;
    }
}

int SCSerial::WriteData(unsigned char* data, int length) {
    if (fd == -1) return -1;
    return write(fd, data, length);
}

int SCSerial::ReadData(unsigned char* data, int length) {
    if (fd == -1) return -1;
    return read(fd, data, length);
}
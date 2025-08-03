#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>

int main() {
    const char* port = "/dev/tty.usbmodem5A680113791";
    
    std::cout << "Testing simple port open..." << std::endl;
    
    int fd = open(port, O_RDWR | O_NOCTTY);
    if (fd == -1) {
        std::cout << "Failed to open: " << strerror(errno) << std::endl;
        return 1;
    }
    
    std::cout << "Port opened successfully: " << fd << std::endl;
    
    struct termios options;
    tcgetattr(fd, &options);
    
    // Try standard baud rate first
    cfsetispeed(&options, B230400);
    cfsetospeed(&options, B230400);
    
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;
    
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 5;
    
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        std::cout << "Failed to set attributes: " << strerror(errno) << std::endl;
        close(fd);
        return 1;
    }
    
    std::cout << "Port configured with 230400 baud" << std::endl;
    
    // Send a simple test byte
    char testByte = 0xFF;
    ssize_t written = write(fd, &testByte, 1);
    std::cout << "Write result: " << written << std::endl;
    
    if (written == -1) {
        std::cout << "Write error: " << strerror(errno) << std::endl;
    }
    
    close(fd);
    std::cout << "Test complete" << std::endl;
    
    return 0;
}
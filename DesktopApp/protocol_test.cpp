/*
 * Simple SCS Protocol Test
 * Tests basic connectivity with minimal protocol implementation
 */

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>
#include <sys/ioctl.h>
#include <IOKit/serial/ioss.h>

class SimpleServo {
private:
    int fd;
    
public:
    bool open(const char* port, speed_t baud) {
        std::cout << "Opening " << port << " at baud rate " << baud << std::endl;
        
        fd = ::open(port, O_RDWR | O_NOCTTY);
        if (fd == -1) {
            std::cout << "Open failed: " << strerror(errno) << std::endl;
            return false;
        }
        
        struct termios options;
        tcgetattr(fd, &options);
        
        // Configure for servo communication
        cfsetispeed(&options, baud);
        cfsetospeed(&options, baud);
        
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;     // No parity
        options.c_cflag &= ~CSTOPB;     // 1 stop bit
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;         // 8 data bits
        options.c_cflag &= ~CRTSCTS;    // No flow control
        
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_oflag &= ~OPOST;
        
        // Short timeout for testing
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 2; // 200ms timeout
        
        if (tcsetattr(fd, TCSANOW, &options) != 0) {
            std::cout << "Configure failed: " << strerror(errno) << std::endl;
            ::close(fd);
            return false;
        }
        
        // For 1M baud, try custom setting
        if (baud == B230400) {  // Use this as placeholder for 1M
            speed_t customBaud = 1000000;
            if (ioctl(fd, IOSSIOSPEED, &customBaud) == -1) {
                std::cout << "Custom baud failed: " << strerror(errno) << std::endl;
            } else {
                std::cout << "Set custom baud 1000000" << std::endl;
            }
        }
        
        tcflush(fd, TCIOFLUSH);
        std::cout << "Port configured successfully" << std::endl;
        return true;
    }
    
    void close() {
        if (fd != -1) {
            ::close(fd);
            fd = -1;
        }
    }
    
    bool sendPing(int servoID) {
        std::cout << "Sending ping to servo " << servoID << std::endl;
        
        // SCS Ping packet: [0xFF 0xFF ID LEN INST CHECKSUM]
        unsigned char packet[6];
        packet[0] = 0xFF;
        packet[1] = 0xFF;
        packet[2] = servoID;
        packet[3] = 2;      // Length
        packet[4] = 0x01;   // PING instruction
        
        // Calculate checksum
        unsigned char checksum = packet[2] + packet[3] + packet[4];
        packet[5] = ~checksum;
        
        std::cout << "Ping packet: ";
        for (int i = 0; i < 6; i++) {
            printf("0x%02X ", packet[i]);
        }
        std::cout << std::endl;
        
        // Send packet
        ssize_t written = write(fd, packet, 6);
        if (written != 6) {
            std::cout << "Write failed: " << written << " bytes, error: " << strerror(errno) << std::endl;
            return false;
        }
        
        std::cout << "Ping sent, waiting for response..." << std::endl;
        
        // Wait for response
        unsigned char response[64];
        ssize_t bytesRead = read(fd, response, sizeof(response));
        
        std::cout << "Read " << bytesRead << " bytes" << std::endl;
        
        if (bytesRead > 0) {
            std::cout << "Response: ";
            for (int i = 0; i < bytesRead; i++) {
                printf("0x%02X ", response[i]);
            }
            std::cout << std::endl;
            
            // Check for valid response header
            if (bytesRead >= 4 && response[0] == 0xFF && response[1] == 0xFF) {
                std::cout << "Valid response header found!" << std::endl;
                return true;
            }
        } else if (bytesRead == 0) {
            std::cout << "Timeout - no response" << std::endl;
        } else {
            std::cout << "Read error: " << strerror(errno) << std::endl;
        }
        
        return false;
    }
};

int main() {
    const char* port = "/dev/tty.usbmodem5A680113791";
    SimpleServo servo;
    
    std::cout << "=== SCS PROTOCOL TEST ===" << std::endl;
    
    // Test different baud rates
    speed_t baudRates[] = {B115200, B230400, B9600, B57600, B38400};
    const char* baudNames[] = {"115200", "1000000", "9600", "57600", "38400"};
    
    for (int b = 0; b < 5; b++) {
        std::cout << "\n--- Testing baud rate: " << baudNames[b] << " ---" << std::endl;
        
        if (!servo.open(port, baudRates[b])) {
            continue;
        }
        
        // Test servo IDs 1-5
        bool found = false;
        for (int id = 1; id <= 5 && !found; id++) {
            if (servo.sendPing(id)) {
                std::cout << "*** SERVO FOUND: ID " << id << " at baud " << baudNames[b] << " ***" << std::endl;
                found = true;
            }
            usleep(100000); // 100ms between tests
        }
        
        servo.close();
        
        if (found) {
            std::cout << "Success! Exiting..." << std::endl;
            break;
        }
    }
    
    std::cout << "\nTest complete." << std::endl;
    return 0;
}
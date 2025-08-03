// macOS/Darwin specific serial header for JaQ Robot
// Platform-specific serial communication declarations

#ifndef SCSERIAL_DARWIN_H
#define SCSERIAL_DARWIN_H

class SCSerial {
private:
    int fd;
    
public:
    SCSerial() : fd(-1) {}
    ~SCSerial() { ClosePort(); }
    
    int OpenPort(const char* portname, int baudrate);
    void ClosePort();
    int WriteData(unsigned char* data, int length);
    int ReadData(unsigned char* data, int length);
};

#endif // SCSERIAL_DARWIN_H
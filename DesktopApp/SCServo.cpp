// macOS-only implementation of SCServo
// Serial communication for servo control

#include "SCServo.h"
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <cstring>
#include <iostream>
#include <errno.h>
#include <sys/ioctl.h>
#include <IOKit/serial/ioss.h>

unsigned long millis()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
}

SCSProtocol::SCSProtocol()
{
    Level = 1;
    End = 1;
}

void SCSProtocol::Host2SCS(u8* DataL, u8* DataH, int Data)
{
    if(End){
        *DataL = (Data>>8);
        *DataH = (Data&0xff);
    }else{
        *DataH = (Data>>8);
        *DataL = (Data&0xff);
    }
}

int SCSProtocol::SCS2Host(u8 DataL, u8 DataH)
{
    u16 Data;
    if(End){
        Data = DataL;
        Data<<=8;
        Data |= DataH;
    }else{
        Data = DataH;
        Data<<=8;
        Data |= DataL;
    }
    return Data;
}

void SCSProtocol::writeBuf(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen, u8 Fun)
{
    u8 msgLen = 2;
    u8 bBuf[6];
    u8 CheckSum = 0;
    bBuf[0] = 0xff;
    bBuf[1] = 0xff;
    bBuf[2] = ID;
    bBuf[4] = Fun;
    if(nDat){
        msgLen += nLen + 1;
        bBuf[3] = msgLen;
        bBuf[5] = MemAddr;
        writeSCS(bBuf, 6);
        
    }else{
        bBuf[3] = msgLen;
        bBuf[5] = MemAddr;
        writeSCS(bBuf, 6);
    }
    CheckSum = ID + msgLen + Fun + MemAddr;
    if(nDat){
        for(u8 i=0; i<nLen; i++){
            CheckSum += nDat[i];
        }
        writeSCS(nDat, nLen);
    }
    writeSCS(~CheckSum);
}

int SCSProtocol::Read(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen)
{
    writeBuf(ID, MemAddr, NULL, nLen, INST_READ);
    return Ack(ID);
}

int SCSProtocol::Ack(u8 ID)
{
    if(ID != 0xfe && Level){
        u8 bBuf[64];
        u8 calSum;
        u8 i;
        
        // Read first 4 bytes of response
        int bytesRead = readSCS(bBuf, 4);
        if(bytesRead != 4){
            std::cout << "[SCSProtocol] Ack timeout/error: expected 4 bytes, got " << bytesRead << std::endl;
            return 0;
        }
        
        // Check header bytes
        if(bBuf[0]!=0xff || bBuf[1]!=0xff){
            std::cout << "[SCSProtocol] Invalid header: got 0x" << std::hex << (int)bBuf[0] << " 0x" << (int)bBuf[1] << std::dec << std::endl;
            return 0;
        }
        
        // Read remaining data
        bytesRead = readSCS(bBuf+4, bBuf[3]);
        if(bytesRead != bBuf[3]){
            std::cout << "[SCSProtocol] Data read error: expected " << (int)bBuf[3] << " bytes, got " << bytesRead << std::endl;
            return 0;
        }
        
        // Verify checksum
        calSum = bBuf[2]+bBuf[3];
        for(i=4; i<bBuf[3]+3; i++){
            calSum += bBuf[i];
        }
        calSum = ~calSum;
        if(calSum!=bBuf[bBuf[3]+3]){
            std::cout << "[SCSProtocol] Checksum error: calculated=0x" << std::hex << (int)calSum 
                      << " received=0x" << (int)bBuf[bBuf[3]+3] << std::dec << std::endl;
            return 0;
        }
        
        std::cout << "[SCSProtocol] Valid response received from ID " << (int)ID << std::endl;
        return 1;
    }
    return 1;
}

SCServo::SCServo()
{
    pSerial = -1;
    IOTimeOut = 100;
}

int SCServo::open(const char* port)
{
    close();
    
    std::cout << "[SCServo] Attempting to open serial port: " << port << std::endl;
    
    pSerial = ::open(port, O_RDWR | O_NOCTTY);
    if (pSerial == -1) {
        std::cout << "[SCServo ERROR] Failed to open port " << port << " - Error: " << strerror(errno) << std::endl;
        return -1;
    }
    
    std::cout << "[SCServo] Port opened successfully, handle: " << pSerial << std::endl;
    
    struct termios options;
    if (tcgetattr(pSerial, &options) != 0) {
        std::cout << "[SCServo ERROR] Failed to get terminal attributes - Error: " << strerror(errno) << std::endl;
        close();
        return -1;
    }
    
    // Set baud rate to 1000000 (1M baud for SCS225)
    // On macOS, we need to use custom baud rate setting
    if (cfsetispeed(&options, B230400) != 0 || cfsetospeed(&options, B230400) != 0) {
        std::cout << "[SCServo ERROR] Failed to set initial baud rate" << std::endl;
        close();
        return -1;
    }
    
    if (tcsetattr(pSerial, TCSANOW, &options) != 0) {
        std::cout << "[SCServo ERROR] Failed to set terminal attributes - Error: " << strerror(errno) << std::endl;
        close();
        return -1;
    }
    
    // Now set custom baud rate to 1000000
    speed_t customBaud = 1000000;
    if (ioctl(pSerial, IOSSIOSPEED, &customBaud) == -1) {
        std::cout << "[SCServo ERROR] Failed to set custom baud rate 1000000 - Error: " << strerror(errno) << std::endl;
        close();
        return -1;
    }
    std::cout << "[SCServo] Baud rate set to 1000000" << std::endl;
    
    // Configure port settings
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;
    
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    options.c_oflag &= ~OPOST;
    
    // Set timeout - VTIME in tenths of seconds (1 = 100ms)
    options.c_cc[VMIN] = 0;  // Don't wait for minimum characters
    options.c_cc[VTIME] = 5; // 500ms timeout
    
    std::cout << "[SCServo] Configuring port: 8N1, no flow control, timeout=" << (IOTimeOut/100) << "0ms" << std::endl;
    
    // Flush any existing data
    tcflush(pSerial, TCIOFLUSH);
    std::cout << "[SCServo] Serial port configured and flushed successfully" << std::endl;
    
    return pSerial;
}

void SCServo::close()
{
    if (pSerial != -1) {
        std::cout << "[SCServo] Closing serial port, handle: " << pSerial << std::endl;
        ::close(pSerial);
        pSerial = -1;
        std::cout << "[SCServo] Serial port closed" << std::endl;
    }
}

int SCServo::writeSCS(unsigned char* data, int length)
{
    if (pSerial == -1) {
        std::cout << "[SCServo ERROR] Attempt to write to closed port" << std::endl;
        return 0;
    }
    
    int written = write(pSerial, data, length);
    if (written != length) {
        std::cout << "[SCServo WARNING] Write incomplete: requested=" << length 
                  << " written=" << written << " errno=" << strerror(errno) << std::endl;
    }
    return written;
}

int SCServo::readSCS(unsigned char* data, int length)
{
    if (pSerial == -1) {
        std::cout << "[SCServo ERROR] Attempt to read from closed port" << std::endl;
        return 0;
    }
    
    int bytesRead = read(pSerial, data, length);
    if (bytesRead < 0) {
        std::cout << "[SCServo ERROR] Read failed: " << strerror(errno) << std::endl;
        return 0;
    } else if (bytesRead == 0) {
        // This is normal for timeouts with non-blocking reads
    } else if (bytesRead != length) {
        std::cout << "[SCServo DEBUG] Read partial: requested=" << length 
                  << " received=" << bytesRead << std::endl;
    }
    return bytesRead;
}

int SCServo::writeSCS(unsigned char data)
{
    return writeSCS(&data, 1);
}

void SCServo::flushSCS()
{
    if (pSerial != -1) {
        tcflush(pSerial, TCIOFLUSH);
    }
}

// Additional missing SCSProtocol methods
int SCSProtocol::EnableTorque(u8 ID, u8 Enable)
{
    std::cout << "[SCSProtocol] " << (Enable ? "Enabling" : "Disabling") << " torque for servo ID " << (int)ID << std::endl;
    int result = writeByte(ID, P_TORQUE_ENABLE, Enable);
    std::cout << "[SCSProtocol] Torque command result for ID " << (int)ID << ": " << (result ? "SUCCESS" : "FAILED") << std::endl;
    return result;
}

int SCSProtocol::Ping(u8 ID)
{
    std::cout << "[SCSProtocol] Pinging servo ID " << (int)ID << std::endl;
    writeBuf(ID, 0, NULL, 0, INST_PING);
    int result = Ack(ID);
    std::cout << "[SCSProtocol] Ping result for ID " << (int)ID << ": " << (result ? "SUCCESS" : "FAILED") << std::endl;
    return result;
}

int SCSProtocol::ReadPos(u8 ID)
{
    return readWord(ID, P_PRESENT_POSITION_L);
}

int SCSProtocol::writeWord(u8 ID, u8 MemAddr, u16 wDat)
{
    u8 bBuf[2];
    Host2SCS(bBuf, bBuf+1, wDat);
    return genWrite(ID, MemAddr, bBuf, 2);
}

int SCSProtocol::writeByte(u8 ID, u8 MemAddr, u8 bDat)
{
    return genWrite(ID, MemAddr, &bDat, 1);
}

int SCSProtocol::readWord(u8 ID, u8 MemAddr)
{
    u8 bBuf[2];
    std::cout << "[SCSProtocol] Reading word from servo ID " << (int)ID << " address " << (int)MemAddr << std::endl;
    if(Read(ID, MemAddr, bBuf, 2)){
        int value = SCS2Host(bBuf[0], bBuf[1]);
        std::cout << "[SCSProtocol] Read value " << value << " from servo ID " << (int)ID << std::endl;
        return value;
    }
    std::cout << "[SCSProtocol] Failed to read from servo ID " << (int)ID << std::endl;
    return -1;
}

int SCSProtocol::genWrite(u8 ID, u8 MemAddr, u8* nDat, u8 nLen)
{
    writeBuf(ID, MemAddr, nDat, nLen, INST_WRITE);
    return Ack(ID);
}

int SCSProtocol::ReadVoltage(u8 ID)
{
    return readByte(ID, P_PRESENT_VOLTAGE);
}

int SCSProtocol::ReadTemper(u8 ID)
{
    return readByte(ID, P_PRESENT_TEMPERATURE);
}

int SCSProtocol::readByte(u8 ID, u8 MemAddr)
{
    u8 bDat;
    std::cout << "[SCSProtocol] Reading byte from servo ID " << (int)ID << " address " << (int)MemAddr << std::endl;
    if(Read(ID, MemAddr, &bDat, 1)){
        std::cout << "[SCSProtocol] Read value " << (int)bDat << " from servo ID " << (int)ID << std::endl;
        return bDat;
    }
    std::cout << "[SCSProtocol] Failed to read byte from servo ID " << (int)ID << std::endl;
    return -1;
}

int SCSProtocol::WritePos(u8 ID, u16 Position, u16 Time, u16 Speed)
{
    return writePos(ID, Position, Time, Speed, INST_WRITE);
}

int SCSProtocol::writePos(u8 ID, u16 Position, u16 Time, u16 Speed, u8 Fun)
{
    u8 bBuf[7];
    Host2SCS(bBuf+0, bBuf+1, Position);
    Host2SCS(bBuf+2, bBuf+3, Time);
    Host2SCS(bBuf+4, bBuf+5, Speed);
    bBuf[6] = 0;
    writeBuf(ID, P_GOAL_POSITION_L, bBuf, 7, Fun);
    return Ack(ID);
}
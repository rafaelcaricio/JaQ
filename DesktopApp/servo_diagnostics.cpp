/*
 * SCS Servo Diagnostics CLI Tool
 * Tests and validates SCS225 servo communication
 * 
 * Usage: ./servo_diagnostics [port] [servo_id]
 * Example: ./servo_diagnostics /dev/tty.usbmodem5A680113791 1
 */

#include "SCServo.h"
#include <iostream>
#include <string>
#include <unistd.h>
#include <cstdlib>
#include <chrono>
#include <cmath>

class ServoDiagnostics {
private:
    SCServo servo;
    std::string portName;
    int servoID;
    
public:
    ServoDiagnostics(const std::string& port, int id) : portName(port), servoID(id) {}
    
    bool openConnection() {
        std::cout << "=== SCS SERVO DIAGNOSTICS TOOL ===" << std::endl;
        std::cout << "Port: " << portName << std::endl;
        std::cout << "Target Servo ID: " << servoID << std::endl;
        std::cout << "======================================" << std::endl;
        
        std::cout << "\n[1/7] Opening serial port..." << std::endl;
        servo.End = 0; // Little endian
        
        int result = servo.open(portName.c_str());
        if (result <= 0) {
            std::cout << "❌ FAILED: Cannot open port " << portName << std::endl;
            std::cout << "   Check:" << std::endl;
            std::cout << "   - Port exists and is accessible" << std::endl;
            std::cout << "   - You have permission (try: sudo)" << std::endl;
            std::cout << "   - Port not in use by another application" << std::endl;
            std::cout << "   - USB cable is connected properly" << std::endl;
            return false;
        }
        
        std::cout << "✅ SUCCESS: Serial port opened (handle: " << result << ")" << std::endl;
        return true;
    }
    
    bool testPing() {
        std::cout << "\n[2/7] Testing servo ping..." << std::endl;
        
        int result = servo.Ping(servoID);
        if (result == 1) {
            std::cout << "✅ SUCCESS: Servo ID " << servoID << " responded to ping" << std::endl;
            return true;
        } else {
            std::cout << "❌ FAILED: Servo ID " << servoID << " did not respond" << std::endl;
            std::cout << "   Check:" << std::endl;
            std::cout << "   - Servo is powered on" << std::endl;
            std::cout << "   - Servo ID is correct (try scanning 1-254)" << std::endl;
            std::cout << "   - Baud rate matches (1000000)" << std::endl;
            std::cout << "   - Wiring is correct (TX/RX not swapped)" << std::endl;
            return false;
        }
    }
    
    void scanForServos() {
        std::cout << "\n[SCAN] Scanning for servos (IDs 1-20)..." << std::endl;
        std::cout << "This may take a moment..." << std::endl;
        
        bool foundAny = false;
        for (int id = 1; id <= 20; id++) {
            std::cout << "Testing ID " << id << "... ";
            std::cout.flush();
            
            int result = servo.Ping(id);
            if (result == 1) {
                std::cout << "✅ FOUND!" << std::endl;
                foundAny = true;
                
                // Read some basic info
                int pos = servo.ReadPos(id);
                int voltage = servo.ReadVoltage(id);
                int temp = servo.ReadTemper(id);
                
                std::cout << "   Position: " << (pos >= 0 ? std::to_string(pos) : "Error") << std::endl;
                std::cout << "   Voltage: " << (voltage >= 0 ? std::to_string(voltage) + "V" : "Error") << std::endl;
                std::cout << "   Temperature: " << (temp >= 0 ? std::to_string(temp) + "°C" : "Error") << std::endl;
            } else {
                std::cout << "No response" << std::endl;
            }
            usleep(50000); // 50ms delay between pings
        }
        
        if (!foundAny) {
            std::cout << "❌ No servos found in scan range" << std::endl;
        }
    }
    
    bool readServoInfo() {
        std::cout << "\n[3/7] Reading servo information..." << std::endl;
        
        // Read current position
        int position = servo.ReadPos(servoID);
        if (position >= 0) {
            std::cout << "✅ Position: " << position << " (range: 0-4095)" << std::endl;
        } else {
            std::cout << "❌ Failed to read position" << std::endl;
            return false;
        }
        
        // Read voltage
        int voltage = servo.ReadVoltage(servoID);
        if (voltage >= 0) {
            std::cout << "✅ Voltage: " << voltage << " (should be ~12V for SCS225)" << std::endl;
        } else {
            std::cout << "❌ Failed to read voltage" << std::endl;
        }
        
        // Read temperature
        int temperature = servo.ReadTemper(servoID);
        if (temperature >= 0) {
            std::cout << "✅ Temperature: " << temperature << "°C" << std::endl;
        } else {
            std::cout << "❌ Failed to read temperature" << std::endl;
        }
        
        return true;
    }
    
    bool testTorqueControl() {
        std::cout << "\n[4/7] Testing torque control..." << std::endl;
        
        // Disable torque first
        std::cout << "Disabling torque..." << std::endl;
        int result = servo.EnableTorque(servoID, 0);
        if (result != 1) {
            std::cout << "❌ Failed to disable torque" << std::endl;
            return false;
        }
        std::cout << "✅ Torque disabled" << std::endl;
        
        usleep(500000); // 500ms delay
        
        // Enable torque
        std::cout << "Enabling torque..." << std::endl;
        result = servo.EnableTorque(servoID, 1);
        if (result != 1) {
            std::cout << "❌ Failed to enable torque" << std::endl;
            return false;
        }
        std::cout << "✅ Torque enabled - servo should be stiff now" << std::endl;
        
        return true;
    }
    
    bool testBasicMovement() {
        std::cout << "\n[5/7] Testing basic movement..." << std::endl;
        std::cout << "WARNING: Servo will move! Ensure it's safe to move." << std::endl;
        std::cout << "Press Enter to continue or Ctrl+C to abort...";
        std::cin.get();
        
        // Read current position
        int startPos = servo.ReadPos(servoID);
        if (startPos < 0) {
            std::cout << "❌ Cannot read starting position" << std::endl;
            return false;
        }
        std::cout << "Starting position: " << startPos << std::endl;
        
        // Move to center position (2048)
        std::cout << "Moving to center position (2048)..." << std::endl;
        int result = servo.WritePos(servoID, 2048, 1000, 0); // 1 second movement
        if (result != 1) {
            std::cout << "❌ Failed to send move command" << std::endl;
            return false;
        }
        
        usleep(1500000); // Wait 1.5 seconds for movement
        
        // Check new position
        int newPos = servo.ReadPos(servoID);
        if (newPos >= 0) {
            std::cout << "✅ Movement complete. New position: " << newPos << std::endl;
            if (abs(newPos - 2048) < 100) {
                std::cout << "✅ Position accuracy good (within 100 units)" << std::endl;
            } else {
                std::cout << "⚠️  Position accuracy poor (error: " << abs(newPos - 2048) << " units)" << std::endl;
            }
        } else {
            std::cout << "❌ Cannot read new position" << std::endl;
            return false;
        }
        
        // Return to original position
        std::cout << "Returning to original position..." << std::endl;
        servo.WritePos(servoID, startPos, 1000, 0);
        usleep(1500000);
        
        return true;
    }
    
    void testCommunicationSpeed() {
        std::cout << "\n[6/7] Testing communication speed..." << std::endl;
        
        auto start = std::chrono::high_resolution_clock::now();
        int successCount = 0;
        const int testCount = 10;
        
        for (int i = 0; i < testCount; i++) {
            if (servo.Ping(servoID) == 1) {
                successCount++;
            }
            usleep(10000); // 10ms between tests
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        std::cout << "Communication test: " << successCount << "/" << testCount << " successful" << std::endl;
        std::cout << "Average response time: " << (duration.count() / testCount) << "ms" << std::endl;
        
        if (successCount == testCount) {
            std::cout << "✅ Communication reliability: Excellent" << std::endl;
        } else if (successCount >= testCount * 0.8) {
            std::cout << "⚠️  Communication reliability: Good" << std::endl;
        } else {
            std::cout << "❌ Communication reliability: Poor" << std::endl;
        }
    }
    
    void cleanup() {
        std::cout << "\n[7/7] Cleanup..." << std::endl;
        
        // Disable torque for safety
        std::cout << "Disabling servo torque for safety..." << std::endl;
        servo.EnableTorque(servoID, 0);
        
        // Close connection
        servo.close();
        std::cout << "✅ Connection closed" << std::endl;
    }
    
    void runFullDiagnostics() {
        if (!openConnection()) {
            return;
        }
        
        if (!testPing()) {
            std::cout << "\nSince ping failed, running servo scan..." << std::endl;
            scanForServos();
            cleanup();
            return;
        }
        
        readServoInfo();
        testTorqueControl();
        testBasicMovement();
        testCommunicationSpeed();
        cleanup();
        
        std::cout << "\n=== DIAGNOSTICS COMPLETE ===" << std::endl;
        std::cout << "If all tests passed, your SCS225 servo is working correctly!" << std::endl;
    }
};

int main(int argc, char* argv[]) {
    std::string port = "/dev/tty.usbmodem5A680113791"; // Default port
    int servoID = 1; // Default servo ID
    
    if (argc >= 2) {
        port = argv[1];
    }
    if (argc >= 3) {
        servoID = std::atoi(argv[2]);
    }
    
    if (servoID < 1 || servoID > 254) {
        std::cerr << "Error: Servo ID must be between 1 and 254" << std::endl;
        return 1;
    }
    
    ServoDiagnostics diagnostics(port, servoID);
    diagnostics.runFullDiagnostics();
    
    return 0;
}
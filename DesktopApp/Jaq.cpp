// Jaq.cpp : Defines the entry point for the application.
//

/* TODO:
 *
 * + implement RAW to Angle and Angle to Raw
 * - implement Angle to Position (Forward Kinematics)
 * - implement Position to Angle (Inverse Kinematics)
 * - add manual mode
 * - add full servo settings and emergency shutoff
 * - add keyframes
 * - add keyframe interpolation
 * - add keyframe sequences
 * - add IK and UI to move and rotate body
 * - move everything into the robot and add a remote control system (battery, DC-DC converter, switches, display, charging)
 * - add communication to rotational sensor
 * - add motor current feeedback
 * - add balance
 * - add task-based motion
 * - add reactive motion and walk cycle
 * - add vision
 * - add surface recognition and adapted walking
 * - add target recognintion
 * - add path planning
 * ...
 */

#include "Jaq.h"

#include "JaqUI.h"
#include "JQServo.h"
#include "JQLeg.h"
#include "JQBot.h"

#include "SCServo.h"

#define _USE_MATH_DEFINES
#include <cmath>

#include <FL/Fl_Widget.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Text_Display.H>
#include <FL/Fl_Preferences.H>
#include <FL/fl_ask.H>
#include <FL/Fl_Text_Buffer.H>
#include <unistd.h>
#include <cstring>
#include <string>
#include <iostream>

using namespace std;

JQBot gBot;
SCServo sc;

int gAppWindowX = 0x7fffffff;
int gAppWindowY = 0x7fffffff;

// Terminal logging
Fl_Text_Buffer* terminalBuffer = nullptr;

void logToTerminal(const std::string& message) {
    if (terminalBuffer && wTerminal) {
        std::string timestamped = message + "\n";
        terminalBuffer->append(timestamped.c_str());
        // Auto-scroll to bottom
        wTerminal->scroll(terminalBuffer->count_lines(0, terminalBuffer->length()), 0);
        Fl::check(); // Process GUI events
    }
    // Also output to console
    std::cout << message << std::endl;
}


// ---- Preferences -----------------------------------------------------------
/**
 * Load calibration data and other preferences.
 * This function does not update the UI.
 */
void loadCalibration()
{
#if 0
    Fl_Preferences prefs(Fl_Preferences::USER, "matthiasm.com", "Jaq");
    Fl_Preferences ui(prefs, "UI");
    Fl_Preferences appWindow(ui, "appWindow");
    appWindow.get("x", gAppWindowX, 0x7fffffff);
    appWindow.get("y", gAppWindowY, 0x7fffffff);
    //appWindow.get("w", gAppWindowW, 0x7fffffff);
    //appWindow.get("h", gAppWindowH, 0x7fffffff);
    Fl_Preferences calibration(prefs, "calibration");
    for (int i = 1; i <= 12; i++) {
        calibration.get(Fl_Preferences::Name(i), gServoCal[i], 0x7fffffff);
        //gServo[i].setTrim(gServoCal[i], gServoDir[i]);
    }
#endif
}

/**
 * Save calibration data to the preferences database.
 * This function does not update the UI.
 */
void writeCalibration()
{
#if 0
    Fl_Preferences prefs(Fl_Preferences::USER, "matthiasm.com", "Jaq");
    Fl_Preferences calibration(prefs, "calibration");
    for (int i = 1; i <= 12; i++) {
        calibration.set(Fl_Preferences::Name(i), gServoCal[i]);
    }
#endif
}


void updateLegPositionUI(JQLeg* leg)
{
    int32_t ix = (int32_t)leg->ID() * 3;
    wPosition[ix]->value(leg->getX());
    wPosition[ix + 1]->value(leg->getY());
    wPosition[ix + 2]->value(leg->getZ());
}

void updateLegAngleUI(JQLeg* leg)
{
    JQServo* servo;
    servo = leg->getHipServo();
    wRawAngle[servo->ID()]->value(servo->getAngleRaw());
    wAngle[servo->ID()]->value(servo->getAngleDeg());
    servo = leg->getKneeServo();
    wRawAngle[servo->ID()]->value(servo->getAngleRaw());
    wAngle[servo->ID()]->value(servo->getAngleDeg());
    servo = leg->getAnkleServo();
    wRawAngle[servo->ID()]->value(servo->getAngleRaw());
    wAngle[servo->ID()]->value(servo->getAngleDeg());
}



// ---- Callbacks -------------------------------------------------------------

void quitCB(Fl_Widget *w, void*)
{
    closeComCB(nullptr, nullptr);
    w->hide();
}


void openComCB(Fl_Button*, void*)
{
    const char* portName = wComName->value();
    logToTerminal("\n=== OPENING SERIAL CONNECTION ===");
    logToTerminal(std::string("[GUI] Attempting to open port: ") + portName);
    
    if (strlen(portName) == 0) {
        logToTerminal("[GUI ERROR] No port name specified!");
        return;
    }
    
    sc.End = 0;
    int ret = sc.open(portName);
    
    if (ret > 0) {
        logToTerminal("[GUI] Serial port opened successfully");
        
        // Test initial communication
        logToTerminal("[GUI] Testing servo communication...");
        ret = sc.Ping(1);
        
        if (ret == 1) {
            logToTerminal("[GUI] SUCCESS: Servo ID 1 responded to ping");
        } else {
            logToTerminal("[GUI] WARNING: Servo ID 1 did not respond to ping");
        }
        
        usleep(100000);
        logToTerminal("[GUI] Running communication test...");
        testComCB(nullptr, nullptr);
        usleep(100000);
        
        logToTerminal("[GUI] Enabling servo power...");
        servosPowerOnCB(nullptr, nullptr);
        
    } else {
        logToTerminal(std::string("[GUI ERROR] Failed to open serial port: ") + portName);
        logToTerminal("[GUI] Check that:");
        logToTerminal("  - Port exists and is accessible");
        logToTerminal("  - You have permission to access the port");
        logToTerminal("  - Port is not in use by another application");
        logToTerminal("  - Servo hardware is connected and powered");
    }
}

void servosPowerOnCB(Fl_Button*, void*)
{
    if (!sc.isOpen()) {
        std::cout << "[GUI ERROR] Cannot enable servos - serial port not open" << std::endl;
        return;
    }
    
    std::cout << "[GUI] Enabling torque for servos 1-12..." << std::endl;
    int successCount = 0;
    
    for (int i = 1; i <= 12; i++) {
        int result = sc.EnableTorque(i, 1);
        if (result == 1) {
            successCount++;
            std::cout << "[GUI] Servo " << i << ": Torque enabled" << std::endl;
        } else {
            std::cout << "[GUI] Servo " << i << ": Failed to enable torque" << std::endl;
        }
    }
    
    std::cout << "[GUI] Torque enable complete: " << successCount << "/12 servos responded" << std::endl;
}

void servosPowerOffCB(Fl_Button*, void*)
{
    if (!sc.isOpen()) {
        std::cout << "[GUI ERROR] Cannot disable servos - serial port not open" << std::endl;
        return;
    }
    
    std::cout << "[GUI] Disabling torque for servos 1-12..." << std::endl;
    int successCount = 0;
    
    for (int i = 1; i <= 12; i++) {
        int result = sc.EnableTorque(i, 0);
        if (result == 1) {
            successCount++;
            std::cout << "[GUI] Servo " << i << ": Torque disabled" << std::endl;
        } else {
            std::cout << "[GUI] Servo " << i << ": Failed to disable torque" << std::endl;
        }
    }
    
    std::cout << "[GUI] Torque disable complete: " << successCount << "/12 servos responded" << std::endl;
}

void testComCB(Fl_Button*, void*)
{
    if (!sc.isOpen()) {
        std::cout << "[GUI ERROR] Cannot test communication - serial port not open" << std::endl;
        return;
    }
    
    std::cout << "\n=== TESTING SERVO COMMUNICATION ===" << std::endl;
    int responseCount = 0;
    
    // read all servo positions and fill the corresponding sliders with that value
    for (int i = 0; i < 12; i++) {
        JQServo& servo = gBot.servo((JQServoID)i);
        std::cout << "[GUI] Reading position from servo " << (i+1) << "..." << std::endl;
        
        int pos = servo.requestAngle();
        if (pos != -1) {
            servo.setAngleRaw(pos, false);
            responseCount++;
            std::cout << "[GUI] Servo " << (i+1) << ": Position = " << pos << std::endl;
        } else {
            std::cout << "[GUI] Servo " << (i+1) << ": No response or error" << std::endl;
        }
    }

    std::cout << "[GUI] Communication test complete: " << responseCount << "/12 servos responded" << std::endl;

    // update all sliders based on the values read from the servos
    for (int i = 0; i < 4; i++) {
        JQLeg& leg = gBot.leg((JQLegID)i);
        leg.updatePositionFromAngles();
        updateLegAngleUI(&leg);
        updateLegPositionUI(&leg);
    }
    
    std::cout << "[GUI] UI updated with servo positions" << std::endl;
}

void closeComCB(Fl_Button*, void*)
{
    if (sc.isOpen()) {
        std::cout << "\n=== CLOSING SERIAL CONNECTION ===" << std::endl;
        std::cout << "[GUI] Disabling all servos before closing..." << std::endl;
        servosPowerOffCB(nullptr, nullptr);
        usleep(100000);
        std::cout << "[GUI] Closing serial port..." << std::endl;
        sc.close();
        std::cout << "[GUI] Serial connection closed" << std::endl;
    } else {
        std::cout << "[GUI] Serial port already closed" << std::endl;
    }
}


void calibrateCB(Fl_Button*, void*)
{
#if 0
    const int DEG90 = (int)(90.0 / 360.0 * 4096);
    const int DEG45 = (int)(45.0 / 360.0 * 4096);
    static int preset[13] = { 0,  DEG90, DEG90, DEG90, DEG90,  -DEG45, -DEG45, -DEG45, -DEG45,  0, 0, 0, 0 };

    const char *code = fl_input("This will overwrite previous calibrations.\nPlease enter calibration security code.\n(Hint: it's 9999)");
    if (code && strcmp(code, "9999") == 0) {
        for (int i = 1; i <= 12; i++) {
            int raw = (int)wRawAngle[i]->value();
            gServoCal[i] = raw - gServoDir[i] * preset[i];
            //wAngle[i]->value((wRawAngle[i]->value() - gServoCal[i]) * gServoDir[i] / 4096.0 * 360.0);
        }
        // store new calibration data in database
        writeCalibration();
        // update all vanilla sliders based on raw values and calibration
        rawToVanilla();
        // update all positions based on the vanilla sliders
        angleToPosition();
        fl_message("Calibration overwritten");
    }
#endif
}

void setRawAngleCB(Fl_Value_Slider* w, long ix)
{
    JQServo &servo = gBot.servo((JQServoID)ix);

    // update the servo (also calculates AngleRad and Leg Positions)
    servo.setAngleRaw((int32_t)w->value());

    // update the UI
    wAngle[servo.ID()]->value(servo.getAngleDeg());
    updateLegPositionUI(servo.leg());

    servo.sendAngle();
}

void setAngleCB(Fl_Value_Slider* w, long ix)
{
    JQServo& servo = gBot.servo((JQServoID)ix);

    // update the servo (also calculates AngleRad and Leg Positions)
    servo.setAngleDeg(w->value());

    // update the UI
    wRawAngle[servo.ID()]->value(servo.getAngleRaw());
    updateLegPositionUI(servo.leg());

    servo.sendAngle();
}

void setPositionCB(Fl_Value_Slider* w, long ix)
{
    JQLeg& leg = gBot.leg(JQLegID(ix / 3));

    // update the leg (also calculates all angles)
    switch (ix % 3) {
    case 0: leg.setX(w->value()); break;
    case 1: leg.setY(w->value()); break;
    case 2: leg.setZ(w->value()); wAllZ->value(w->value()); break;
    }

    // update the UI
    updateLegAngleUI(&leg);

    // update servo positions
    leg.getHipServo()->sendAngle();
    leg.getKneeServo()->sendAngle();
    leg.getAnkleServo()->sendAngle();
}

void setAllZCB(Fl_Value_Slider* w, void*)
{
    double z = w->value();
    wPosition[2]->value(z);
    wPosition[2]->do_callback();
    wPosition[5]->value(z);
    wPosition[5]->do_callback();
    wPosition[8]->value(z);
    wPosition[8]->do_callback();
    wPosition[11]->value(z);
    wPosition[11]->do_callback();
#if 0
    int ix;
    for (ix = 1; ix <= 4; ix++) {
        wPosition[ix]->value(z);
    }
    positionToAngle();
    vanillaToRaw();
    uint32_t t0 = GetTickCount();
    for (ix = 1; ix <= 4; ix++) {
        sc.RegWritePos(ix, wRawAngle[ix]->value(), 0, 0);
        sc.RegWritePos(ix+4, wRawAngle[ix+4]->value(), 0, 0);
        sc.RegWritePos(ix+8, wRawAngle[ix+8]->value(), 0, 0);
        //sc.writeWord(ix, 42, wRawAngle[ix]->value());
        //sc.writeWord(ix + 4, 42, wRawAngle[ix + 4]->value());
        //sc.writeWord(ix + 8, 42, wRawAngle[ix + 8]->value());
    }
    sc.RegWriteAction();
    uint32_t t1 = GetTickCount();
    wTerminal->printf("That took %dms.\n", t1 - t0);
#endif
    Fl::flush(); // FIXME: ouch!
}

int main(int argc, char** argv)
{
    Fl_Window* win = createMainWindow();
    if (gAppWindowX != 0x7fffffff) win->position(gAppWindowX, gAppWindowY);

    // Initialize terminal buffer
    terminalBuffer = new Fl_Text_Buffer();
    wTerminal->buffer(terminalBuffer);

    // load calibration data from last session
    //loadCalibration();

    for (int i = 0; i < 4; i++) {
        JQLeg& leg = gBot.leg((JQLegID)i);
        leg.updatePositionFromAngles();
        updateLegAngleUI(&leg);
        updateLegPositionUI(&leg);
    }

    win->show(argc, argv);
    win->callback(quitCB);
    
    // Welcome message
    logToTerminal("=== JAQ ROBOT CONTROL APPLICATION ===");
    logToTerminal("Enter serial port name and click 'Open' to connect");
    logToTerminal("Example ports: /dev/ttyUSB0, /dev/ttyACM0 (Linux/macOS)");
    
    Fl::lock();
    Fl::run();
    
    // Cleanup
    delete terminalBuffer;
    return 0;
}

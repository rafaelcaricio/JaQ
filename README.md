
# JaQ - Just another Quadruped

JaQ is a robotic quadruped dog project featuring both 3D printable CAD files and a C++ desktop application for servo control. The project includes a cross-platform GUI application built with FLTK for controlling 12 servos arranged in a 4-leg configuration.

![Test Assembly](html/JaQ_TestAssembly.jpg)
![Deck Assembly](html/JaQ_Deck.jpg)
![Parts Overview](html/JaQ_Parts.jpg)

## Project Structure

```
JaQ_robotdog/
├── CADFiles/           # 3D printable STL files and FreeCAD source files
├── DesktopApp/         # C++ application source code
├── html/               # Project images and documentation assets
└── README.md           # This file
```

## Hardware Overview

The JaQ quadruped robot uses:
- **12 Servos Total**: 3 servos per leg (shoulder, knee, ankle)
- **4 Legs**: Front Left (FL), Front Right (FR), Back Left (BL), Back Right (BR)
- **Serial Communication**: RS485/TTL communication protocol
- **Servo Protocol**: SCS (Serial Control Servo) protocol

## Prerequisites

### macOS Requirements
- macOS 10.14 or later (tested on macOS with M1 Pro)
- Xcode Command Line Tools
- Homebrew package manager
- CMake 3.10 or later

### Dependencies
- **FLTK**: Fast Light Toolkit for GUI (installed via Homebrew)
- **Standard C++ libraries**: For cross-platform development

## Installation & Setup

### 1. Install Dependencies

```bash
# Install Homebrew (if not already installed)
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install required packages
brew install fltk cmake
```

### 2. Build the Application

```bash
# Navigate to the project directory
cd /path/to/JaQ_robotdog/DesktopApp

# Build using CMake
cmake . && make
```

### 3. Run the Application

```bash
# Run the built application
./Jaq.app/Contents/MacOS/Jaq
```

## Usage

### GUI Controls

The application provides several control modes:

#### 1. Serial Connection
- **Com Port**: Enter the serial port (e.g., `/dev/tty.usbserial-*`)
- **Open**: Establish connection to servo controller
- **Test**: Send ping command to test communication
- **Close**: Close serial connection
- **Power/Power Off**: Enable/disable servo torque

#### 2. Control Modes

**RAW Mode**: Direct servo position control (0-4095 raw values)
**Angles Mode**: Angle-based control in degrees
**Position Mode**: 3D position control (X, Y, Z coordinates)

#### 3. Individual Servo Control
Each leg has three servos:
- **Hip**: Side-to-side movement
- **Knee**: Forward-backward leg movement  
- **Ankle**: Foot positioning

### Serial Communication Setup

1. Connect the servo controller to your Mac via USB-to-Serial adapter
2. Identify the port: `ls /dev/tty.usbserial-*` or `ls /dev/cu.usbmodem*`
3. Enter the port name in the GUI
4. Click "Open" to establish connection
5. Use "Test" to verify communication

## Architecture

### Core Components

#### Robot Control Classes
- **JQBot**: Main robot controller managing 4 legs and 12 servos
- **JQLeg**: Individual leg control (4 instances per robot)
- **JQServo**: Individual servo motor control (12 servos total)
- **JQServoController**: High-level servo coordination

#### Communication Protocol
- **SCServo**: Servo communication protocol implementation
- **SCSProtocol**: Low-level serial command protocol

#### GUI Interface
- **JaqUI**: FLTK-based user interface (generated from .fl files)

### Servo Communication Protocol

The application uses the SCS (Serial Control Servo) protocol with the following features:

#### Register Map
Key servo registers include:
- `P_GOAL_POSITION_L/H` (42/43): Target position
- `P_GOAL_TIME_L/H` (44/45): Movement time
- `P_GOAL_SPEED_L/H` (46/47): Movement speed
- `P_PRESENT_POSITION_L/H` (56/57): Current position
- `P_TORQUE_ENABLE` (40): Enable/disable servo

#### Communication Commands
- **INST_PING** (0x01): Check servo connectivity
- **INST_READ** (0x02): Read servo data
- **INST_WRITE** (0x03): Write servo data
- **INST_REG_WRITE** (0x04): Prepare write command
- **INST_ACTION** (0x05): Execute prepared commands

## Development

### Building from Source

The project uses CMake for cross-platform building:

```bash
# Clean build
rm -rf CMakeCache.txt CMakeFiles/
cmake . && make

# Debug build
cmake -DCMAKE_BUILD_TYPE=Debug . && make
```

### GUI Development

The GUI is created using FLTK's Fluid tool:

```bash
# Edit GUI layout
fluid JaqUI.fl

# The .fl file generates JaqUI.h and JaqUI.cxx automatically during build
```

### Code Style

- Follow existing C++ conventions in the codebase
- Use meaningful variable names
- Comment complex servo protocol implementations
- Maintain platform-specific code separation

## Troubleshooting

### Build Issues

**FLTK not found:**
```bash
# Ensure FLTK is properly installed
brew reinstall fltk
```

**CMake configuration errors:**
```bash
# Clear cache and rebuild
rm -rf CMakeCache.txt CMakeFiles/
cmake . && make
```

### Runtime Issues

**Serial connection fails:**
- Check cable connections
- Verify correct port name (`ls /dev/tty.*`)
- Ensure servo controller is powered
- Try different baud rates

**Servo not responding:**
- Verify servo ID configuration
- Check power supply to servos
- Test with individual servo ping commands
- Ensure proper RS485 wiring

**GUI display issues:**
- Update to latest macOS
- Reinstall FLTK: `brew reinstall fltk`

### Hardware Debugging

**No servo movement:**
1. Check torque enable status
2. Verify servo power supply (adequate current)
3. Test individual servo communication
4. Check for physical obstructions

**Erratic movement:**
1. Verify stable power supply
2. Check for electromagnetic interference
3. Ensure proper grounding
4. Test with lower movement speeds

## Hardware Assembly

### 3D Printing
- Print parts from `CADFiles/` directory
- Recommended: PLA or PETG plastic
- 0.2mm layer height for good detail
- Support material for overhangs

### Servo Installation
- Mount servos according to CAD assembly
- Ensure proper alignment for joint movement
- Use appropriate screws for servo mounting
- Route cables to avoid pinching during movement

### Wiring
- Connect servos in series using RS485 protocol
- Assign unique IDs to each servo (1-12)
- Provide adequate power supply (6-7.4V, 10A+ recommended)
- Use proper gauge wire for power distribution

## Contributing

1. Fork the repository
2. Create feature branch
3. Follow existing code style
4. Test thoroughly on macOS
5. Submit pull request with detailed description

## License

This project is open source. Please respect the original design and contribute improvements back to the community.

## Support

For issues and questions:
1. Check troubleshooting section above
2. Review servo protocol documentation in source code
3. Test with minimal servo configuration first
4. Create detailed issue reports with:
   - macOS version
   - Build logs
   - Serial communication logs
   - Hardware configuration

---

**Note**: This project is specifically optimized for macOS. Windows support has been deprecated in favor of focusing on the macOS ecosystem.

# Sunray - Autonomous Robotic Mower

[![Arduino](https://img.shields.io/badge/Arduino-Due-blue.svg)](https://www.arduino.cc/en/Main/ArduinoBoardDue)
[![License](https://img.shields.io/badge/License-GPL%20v3-blue.svg)](LICENSE)
[![Build Status](https://img.shields.io/badge/Build-Passing-green.svg)](#)

Sunray is an open-source autonomous robotic lawn mower project designed to run on Arduino Due microcontrollers. The project provides a complete solution for autonomous lawn mowing with advanced navigation, perimeter detection, and intelligent path planning.

## 🌟 Features

### Core Functionality
- **Autonomous Navigation**: Advanced pathfinding with obstacle avoidance
- **Perimeter Detection**: Wire-based boundary recognition
- **GPS Integration**: Precise positioning and mapping
- **IMU Support**: Orientation and tilt detection
- **Battery Management**: Intelligent charging and power monitoring
- **Weather Resistance**: Designed for outdoor operation

### Recent Improvements
- **WiFi Auto-Restart**: Automatic WiFi connection recovery with configurable parameters
- **Enhanced AT+ Commands**: Extended communication protocol for better integration
- **Improved Motor Control**: Better PID implementation and acceleration limiting
- **Modular Architecture**: Clean separation of concerns for easier maintenance

## 🏗️ Architecture

The project consists of several key components:

```
Sunray/
├── sunray/           # Main Arduino firmware
├── alfred/           # Raspberry Pi companion software
├── esp32_ble/        # ESP32 Bluetooth module
├── ros/              # ROS integration
├── python/           # Python utilities
└── tools/            # Development and debugging tools
```

### Hardware Platforms
- **Primary**: Arduino Due (STM32-based)
- **Companion**: Raspberry Pi (Alfred software)
- **Communication**: ESP32 for Bluetooth/WiFi

## 🚀 Quick Start

### Prerequisites
- Arduino IDE or Arduino CLI
- Arduino Due board
- Required libraries (see `sunray/sunray.ino` for dependencies)

### Installation

1. **Clone the repository**:
   ```bash
   git clone https://github.com/Starsurfer78/sunray.git
   cd sunray
   ```

2. **Install Arduino CLI** (if not using Arduino IDE):
   ```bash
   # Windows (PowerShell)
   winget install Arduino.ArduinoCLI
   
   # Or download from: https://arduino.github.io/arduino-cli/
   ```

3. **Install required board packages**:
   ```bash
   arduino-cli core update-index
   arduino-cli core install arduino:sam
   ```

4. **Compile and upload**:
   ```bash
   cd sunray
   arduino-cli compile --fqbn arduino:sam:arduino_due_x .
   arduino-cli upload -p COM3 --fqbn arduino:sam:arduino_due_x .
   ```

## ⚙️ Configuration

### Main Configuration
Edit `sunray/config.h` to configure your robot:

```cpp
// WiFi Settings
#define WIFI_SSID "YourWiFiNetwork"
#define WIFI_PASS "YourPassword"

// Robot Dimensions
#define ROBOT_WIDTH 0.3  // meters
#define CUTTING_WIDTH 0.2  // meters

// Motor Configuration
#define MOTOR_MAX_PWM 255
#define MOTOR_DEADZONE 8
```

### WiFi Auto-Restart Configuration
The project includes automatic WiFi recovery with configurable parameters:

```cpp
// WiFi restart parameters (in config.h)
#define WIFI_RESTART_CHECK_INTERVAL 30000  // Check every 30 seconds
#define WIFI_RESTART_MAX_FAILURES 3        // Restart after 3 failures
#define WIFI_RESTART_DELAY 5000            // Wait 5s after restart
```

To enable WiFi auto-restart, define:
```cpp
#define ENABLE_SIMPLE_WIFI_RESTART
```

### AT+ Commands
The robot supports various AT+ commands for communication:

- `AT+WIFI_STATUS` - Check WiFi connection status
- `AT+WIFI_RESTART` - Force WiFi restart
- `AT+WIFI_CONFIG` - Display WiFi configuration
- `AT+VERSION` - Get firmware version
- `AT+SUMMARY` - Get robot status summary

## 🔧 Development

### Project Structure
```
sunray/src/
├── driver/           # Hardware drivers
│   ├── SerialRobotDriver.cpp
│   ├── SimpleWifiRestart.cpp
│   └── ...
├── op/               # Operation modes
├── config/           # Configuration files
└── test/             # Unit tests
```

### Building for Different Platforms

**Arduino Due** (Primary target):
```bash
arduino-cli compile --fqbn arduino:sam:arduino_due_x sunray
```

**Alfred (Raspberry Pi)**:
```bash
cd alfred
make
```

### Code Style
The project follows these conventions:
- **Classes**: `PascalCase` (e.g., `MotorDriver`)
- **Functions/Variables**: `camelCase` (e.g., `calculateDistance`)
- **Constants**: `UPPER_SNAKE_CASE` (e.g., `MAX_SPEED`)
- **Indentation**: 4 spaces, no tabs

## 🧪 Testing

### Smoke Tests
Before deploying:
1. Compile without errors
2. Test startup sequence
3. Verify sensor initialization
4. Test AT+ command responses

### Field Testing
1. Place robot in safe, enclosed area
2. Start mowing operation
3. Monitor for 2-3 minutes
4. Check for proper perimeter detection
5. Verify obstacle avoidance

## 📊 Monitoring

### Serial Console
Connect via serial (115200 baud) to monitor robot status:
```
[INFO] Robot initialized
[GPS] Satellites: 8, HDOP: 1.2
[WIFI] Connected to YourNetwork
[MOTOR] Left: 120, Right: 125
```

### Web Interface
When connected to WiFi, access the web interface at:
```
http://[robot-ip]/
```

## 🤝 Contributing

### Development Guidelines
1. **Small Steps**: Make atomic, reviewable changes
2. **Test First**: Ensure changes compile and pass smoke tests
3. **Document**: Update relevant documentation
4. **Compatibility**: Maintain AT+ command compatibility

### Submitting Changes
1. Fork the repository
2. Create a feature branch: `git checkout -b feature/your-feature`
3. Make your changes following the coding standards
4. Test thoroughly on Arduino Due
5. Submit a pull request

## 📋 Roadmap

### Completed ✅
- [x] WiFi auto-restart functionality
- [x] Configurable WiFi parameters
- [x] Enhanced AT+ command set
- [x] Modular driver architecture

### In Progress 🚧
- [ ] Perimeter offset configuration
- [ ] Enhanced PID control
- [ ] Blockage detection
- [ ] Code simplification

### Planned 📅
- [ ] Dynamic loop timing
- [ ] Feed-forward motor control
- [ ] Acceleration limiting
- [ ] Advanced obstacle detection

## 🐛 Troubleshooting

### Common Issues

**Robot doesn't start**:
- Check power connections
- Verify Arduino Due is properly flashed
- Check serial console for error messages

**WiFi connection issues**:
- Verify SSID and password in `config.h`
- Check signal strength
- Use `AT+WIFI_STATUS` to diagnose
- Enable `ENABLE_SIMPLE_WIFI_RESTART` for auto-recovery

**GPS not working**:
- Ensure clear sky view
- Check GPS module connections
- Wait for satellite acquisition (can take several minutes)

**Perimeter detection problems**:
- Verify wire loop is complete
- Check perimeter sensor connections
- Calibrate perimeter signal strength

## 📄 License

This project is licensed under the GNU General Public License v3.0 - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- Original Sunray project contributors
- Arduino and open-source community
- All contributors and testers

## 📞 Support

- **Issues**: [GitHub Issues](https://github.com/Starsurfer78/sunray/issues)
- **Discussions**: [GitHub Discussions](https://github.com/Starsurfer78/sunray/discussions)
- **Documentation**: [Wiki](https://github.com/Starsurfer78/sunray/wiki)

---

**Note**: This is an active development project. Always test changes in a safe environment before deploying to your lawn mower.
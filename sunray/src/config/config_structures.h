// Ardumower Sunray - Alfred Platform
// Structured Configuration System
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH

#ifndef CONFIG_STRUCTURES_H
#define CONFIG_STRUCTURES_H

#include "../../config.h"

// Version information for configuration compatibility
#define CONFIG_STRUCTURES_VERSION "1.0.0"
#define AT_PROTOCOL_VERSION "1.6"

// Motor Configuration Structure
struct MotorConfig {
    // PID Parameters
    float pidKp = MOTOR_PID_KP;           // Proportional gain
    float pidKi = MOTOR_PID_KI;           // Integral gain  
    float pidKd = MOTOR_PID_KD;           // Derivative gain
    float pidLimit = MOTOR_PID_LIMIT;     // Output limit
    float pidRamp = MOTOR_PID_RAMP;       // Output derivative limit
    float pidLowPass = MOTOR_PID_LP;      // Low-pass filter coefficient
    
    // Current Limits
    float faultCurrent = MOTOR_FAULT_CURRENT;         // Fault current threshold (A)
    float overloadCurrent = MOTOR_OVERLOAD_CURRENT;   // Overload current threshold (A)
    float tooLowCurrent = MOTOR_TOO_LOW_CURRENT;      // Too low current threshold (A)
    
    // Mow Motor Limits
    float mowFaultCurrent = MOW_FAULT_CURRENT;        // Mow motor fault current (A)
    float mowOverloadCurrent = MOW_OVERLOAD_CURRENT;  // Mow motor overload current (A)
    float mowTooLowCurrent = MOW_TOO_LOW_CURRENT;     // Mow motor too low current (A)
    
    // Mechanical Parameters
    int ticksPerRevolution = TICKS_PER_REVOLUTION;    // Encoder ticks per wheel revolution
    float wheelDiameter = WHEEL_DIAMETER;             // Wheel diameter (mm)
    float wheelBase = WHEEL_BASE_CM;                  // Wheel-to-wheel distance (cm)
    
    // Speed and Control
    bool useLinearSpeedRamp = USE_LINEAR_SPEED_RAMP;  // Use linear speed ramping
    float overloadSpeed = MOTOR_OVERLOAD_SPEED;       // Speed at motor overload (m/s)
    int maxMowPwm = 255;                              // Maximum mow motor PWM
    
    // Direction Swapping
    bool leftSwapDirection = false;                   // Swap left motor direction
    bool rightSwapDirection = false;                  // Swap right motor direction
    
    // Feature Flags
    bool enableOverloadDetection = ENABLE_OVERLOAD_DETECTION;
    bool enableFaultDetection = ENABLE_FAULT_DETECTION;
    bool enableMowMotor = ENABLE_MOW_MOTOR;
    
    // Apply compile-time direction swapping
    void applyCompileTimeSettings() {
        #ifdef MOTOR_LEFT_SWAP_DIRECTION
            leftSwapDirection = !leftSwapDirection;
        #endif
        #ifdef MOTOR_RIGHT_SWAP_DIRECTION
            rightSwapDirection = !rightSwapDirection;
        #endif
        #ifdef MAX_MOW_PWM
            if (MAX_MOW_PWM <= 255) {
                maxMowPwm = MAX_MOW_PWM;
            }
        #endif
    }
};

// AT Protocol Configuration
struct ATProtocolConfig {
    // Command Identifiers
    static constexpr char MOTOR_CMD = 'M';            // Motor control command
    static constexpr char SUMMARY_CMD = 'S';          // Summary/sensor data command
    static constexpr char VERSION_CMD = 'V';          // Version query command
    static constexpr char INTELLIGENT_CMD = 'I';     // Intelligent functions command
    static constexpr char LEARNING_CMD = 'L';        // Learning data command
    static constexpr char DIAGNOSTIC_CMD = 'X';      // Extended diagnostics command
    
    // Communication Parameters
    unsigned long baudRate = ROBOT_BAUDRATE;         // Serial communication baud rate
    unsigned long motorTimeout = 100;                // Motor command timeout (ms)
    unsigned long summaryTimeout = 200;              // Summary command timeout (ms)
    unsigned long responseTimeout = 1000;            // General response timeout (ms)
    
    // Protocol Settings
    bool enableCrcCheck = true;                      // Enable CRC validation
    bool enableDebugOutput = false;                  // Enable debug output
    int maxRetries = 3;                             // Maximum command retries
    
    // Buffer Sizes
    static constexpr int CMD_BUFFER_SIZE = 500;      // Command buffer size
    static constexpr int RESPONSE_BUFFER_SIZE = 500; // Response buffer size
};

// Sensor Configuration
struct SensorConfig {
    // Bumper Settings
    bool bumperEnable = BUMPER_ENABLE;
    bool bumperInvert = BUMPER_INVERT;
    unsigned long bumperDeadTime = BUMPER_DEADTIME;          // ms
    unsigned long bumperTriggerDelay = BUMPER_TRIGGER_DELAY; // ms
    unsigned long bumperMaxTriggerTime = BUMPER_MAX_TRIGGER_TIME * 1000; // ms
    
    // Lift Sensor Settings
    bool liftEnable = ENABLE_LIFT_DETECTION;
    bool liftInvert = LIFT_INVERT;
    bool liftObstacleAvoidance = LIFT_OBSTACLE_AVOIDANCE;
    
    // Rain Sensor Settings
    bool rainEnable = RAIN_ENABLE;
    
    // Current Sensing
    float currentFactor = CURRENT_FACTOR;
    
    // Temperature Settings
    bool tempSensorEnable = USE_TEMP_SENSOR;
    float dockOverheatTemp = DOCK_OVERHEAT_TEMP;     // °C
    float dockTooColdTemp = DOCK_TOO_COLD_TEMP;      // °C
};

// Battery Configuration
struct BatteryConfig {
    // Voltage Thresholds
    float goHomeVoltage = GO_HOME_VOLTAGE;           // V
    float fullVoltage = BAT_FULL_VOLTAGE;            // V
    float underVoltage = BAT_UNDERVOLTAGE;           // V
    
    // Current Thresholds
    float fullCurrent = BAT_FULL_CURRENT;            // A
    float fullSlope = BAT_FULL_SLOPE;                // V/min
    
    // Power Management
    bool switchOffIdle = BAT_SWITCH_OFF_IDLE;
    bool switchOffUndervoltage = BAT_SWITCH_OFF_UNDERVOLTAGE;
};

// System Configuration Container
struct SystemConfig {
    MotorConfig motor;
    ATProtocolConfig atProtocol;
    SensorConfig sensor;
    BatteryConfig battery;
    
    // Configuration metadata
    String configVersion = CONFIG_STRUCTURES_VERSION;
    unsigned long lastModified = 0;
    bool isValid = false;
    
    // Initialize with compile-time settings
    void initialize() {
        motor.applyCompileTimeSettings();
        lastModified = millis();
        isValid = true;
    }
    
    // Validate configuration consistency
    bool validate() const {
        // Motor validation
        if (motor.pidKp <= 0 || motor.pidKi < 0 || motor.pidKd < 0) return false;
        if (motor.faultCurrent <= motor.overloadCurrent) return false;
        if (motor.wheelDiameter <= 0 || motor.wheelBase <= 0) return false;
        if (motor.ticksPerRevolution <= 0) return false;
        
        // AT Protocol validation
        if (atProtocol.baudRate < 9600 || atProtocol.baudRate > 115200) return false;
        if (atProtocol.motorTimeout == 0 || atProtocol.summaryTimeout == 0) return false;
        
        // Battery validation
        if (battery.underVoltage >= battery.goHomeVoltage) return false;
        if (battery.goHomeVoltage >= battery.fullVoltage) return false;
        
        return true;
    }
};

// Error Codes for unified error handling
enum class RobotErrorCode : uint16_t {
    NO_ERROR = 0,
    
    // Motor Errors (1-99)
    ERROR_MOTOR_FAULT = 1,
    ERROR_MOTOR_OVERLOAD = 2,
    ERROR_MOTOR_LOW_CURRENT = 3,
    ERROR_MOTOR_ENCODER = 4,
    ERROR_MOTOR_PID_UNSTABLE = 5,
    
    // Communication Errors (100-199)
    ERROR_COMM_LOST = 100,
    ERROR_COMM_TIMEOUT = 101,
    ERROR_COMM_CRC = 102,
    ERROR_COMM_INVALID_RESPONSE = 103,
    
    // Sensor Errors (200-299)
    ERROR_SENSOR_BUMPER_STUCK = 200,
    ERROR_SENSOR_LIFT_FAULT = 201,
    ERROR_SENSOR_RAIN_FAULT = 202,
    ERROR_SENSOR_CURRENT_FAULT = 203,
    
    // Battery Errors (300-399)
    ERROR_BATTERY_UNDERVOLTAGE = 300,
    ERROR_BATTERY_OVERVOLTAGE = 301,
    ERROR_BATTERY_TEMP_HIGH = 302,
    ERROR_BATTERY_TEMP_LOW = 303,
    
    // System Errors (400-499)
    ERROR_CONFIG_INVALID = 400,
    ERROR_CONFIG_VERSION_MISMATCH = 401,
    ERROR_SYSTEM_OVERHEATED = 402,
    ERROR_SYSTEM_MEMORY_LOW = 403
};

// Error Information Structure
struct RobotError {
    RobotErrorCode code;
    String description;
    unsigned long timestamp;
    bool isRecoverable;
    uint8_t severity;  // 1=Info, 2=Warning, 3=Error, 4=Critical
    
    RobotError() : code(RobotErrorCode::NO_ERROR), timestamp(0), isRecoverable(true), severity(1) {}
    
    RobotError(RobotErrorCode c, const String& desc, bool recoverable = true, uint8_t sev = 2) 
        : code(c), description(desc), timestamp(millis()), isRecoverable(recoverable), severity(sev) {}
    
    String toString() const {
        return String("Error ") + String((uint16_t)code) + ": " + description + 
               " (" + String(timestamp) + "ms, " + 
               (isRecoverable ? "recoverable" : "critical") + ")";
    }
};

// Global configuration instance declaration
extern SystemConfig g_systemConfig;

#endif // CONFIG_STRUCTURES_H
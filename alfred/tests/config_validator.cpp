#include "config_validator.h"
#include <sstream>
#include <algorithm>
#include <cmath>

ConfigValidator::ConfigValidator() {
    clearValidationHistory();
}

void ConfigValidator::clearValidationHistory() {
    lastErrors_.clear();
    lastWarnings_.clear();
}

bool ConfigValidator::validateMotorConfig(const MotorConfig& config) {
    clearValidationHistory();
    
    bool isValid = true;
    
    // Validate PID parameters
    if (!validatePIDParameters(config.pidKp, config.pidKi, config.pidKd, "Motor")) {
        isValid = false;
    }
    
    // Validate current limits
    if (!validateCurrentLimits(config.tooLowCurrent, config.overloadCurrent, 
                              config.faultCurrent, "Motor")) {
        isValid = false;
    }
    
    // Validate mechanical parameters
    if (!validateMechanicalParameters(config)) {
        isValid = false;
    }
    
    return isValid;
}

ConfigValidator::ValidationResult ConfigValidator::validateMotorConfigDetailed(const MotorConfig& config) {
    ValidationResult result;
    
    clearValidationHistory();
    result.isValid = validateMotorConfig(config);
    result.errors = lastErrors_;
    result.warnings = lastWarnings_;
    
    return result;
}

bool ConfigValidator::validateATProtocolConfig(const ATProtocolConfig& config) {
    clearValidationHistory();
    
    return validateCommunicationParameters(config);
}

ConfigValidator::ValidationResult ConfigValidator::validateATProtocolConfigDetailed(const ATProtocolConfig& config) {
    ValidationResult result;
    
    clearValidationHistory();
    result.isValid = validateATProtocolConfig(config);
    result.errors = lastErrors_;
    result.warnings = lastWarnings_;
    
    return result;
}

bool ConfigValidator::validateRobotConfig(const RobotConfig& config) {
    clearValidationHistory();
    
    bool isValid = true;
    
    // Validate that robot name is not empty
    if (config.name.empty()) {
        lastErrors_.push_back("Robot name cannot be empty");
        isValid = false;
    }
    
    // Validate robot dimensions
    if (config.length <= 0.0f || config.width <= 0.0f || config.height <= 0.0f) {
        lastErrors_.push_back("Robot dimensions must be positive");
        isValid = false;
    }
    
    // Validate weight
    if (config.weight <= 0.0f) {
        lastErrors_.push_back("Robot weight must be positive");
        isValid = false;
    }
    
    return isValid;
}

bool ConfigValidator::validateSensorConfig(const SensorConfig& config) {
    clearValidationHistory();
    
    return validateSensorRanges(config);
}

bool ConfigValidator::validateBatteryConfig(const BatteryConfig& config) {
    clearValidationHistory();
    
    return validateBatteryLimits(config);
}

bool ConfigValidator::validateConfigurationConsistency(const MotorConfig& motor,
                                                     const ATProtocolConfig& atProtocol,
                                                     const RobotConfig& robot) {
    clearValidationHistory();
    
    bool isValid = true;
    
    // Check that communication timeout is reasonable for motor control
    if (atProtocol.timeout > 1000) {  // 1 second is too long for motor control
        lastWarnings_.push_back("AT protocol timeout may be too long for real-time motor control");
    }
    
    // Check that wheel base is reasonable for robot dimensions
    if (motor.wheelBase > robot.length) {
        lastErrors_.push_back("Wheel base cannot be larger than robot length");
        isValid = false;
    }
    
    // Check that wheel diameter is reasonable for robot dimensions
    if (motor.wheelDiameter > robot.height) {
        lastWarnings_.push_back("Wheel diameter is larger than robot height - check configuration");
    }
    
    return isValid;
}

bool ConfigValidator::validatePIDParameters(float kp, float ki, float kd, const std::string& context) {
    bool isValid = true;
    
    // Validate Kp
    if (kp < 0.0f) {
        lastErrors_.push_back(context + " PID Kp must be non-negative");
        isValid = false;
    } else if (kp < MIN_PID_KP) {
        lastWarnings_.push_back(context + " PID Kp is very small, may result in poor control");
    } else if (kp > MAX_PID_KP) {
        lastWarnings_.push_back(context + " PID Kp is very large, may cause instability");
    }
    
    // Validate Ki
    if (ki < 0.0f) {
        lastErrors_.push_back(context + " PID Ki must be non-negative");
        isValid = false;
    } else if (ki > MAX_PID_KI) {
        lastWarnings_.push_back(context + " PID Ki is very large, may cause oscillations");
    }
    
    // Validate Kd
    if (kd < 0.0f) {
        lastErrors_.push_back(context + " PID Kd must be non-negative");
        isValid = false;
    } else if (kd > MAX_PID_KD) {
        lastWarnings_.push_back(context + " PID Kd is very large, may amplify noise");
    }
    
    return isValid;
}

bool ConfigValidator::validateCurrentLimits(float tooLow, float overload, float fault, const std::string& context) {
    bool isValid = true;
    
    // Check individual limits
    if (tooLow <= 0.0f) {
        lastErrors_.push_back(context + " too-low current limit must be positive");
        isValid = false;
    }
    
    if (overload <= 0.0f) {
        lastErrors_.push_back(context + " overload current limit must be positive");
        isValid = false;
    }
    
    if (fault <= 0.0f) {
        lastErrors_.push_back(context + " fault current limit must be positive");
        isValid = false;
    }
    
    // Check logical ordering
    if (tooLow >= overload) {
        lastErrors_.push_back(context + " too-low current must be less than overload current");
        isValid = false;
    }
    
    if (overload >= fault) {
        lastErrors_.push_back(context + " overload current must be less than fault current");
        isValid = false;
    }
    
    // Check reasonable ranges
    if (fault > MAX_CURRENT) {
        lastWarnings_.push_back(context + " fault current is very high");
    }
    
    if (tooLow < MIN_CURRENT) {
        lastWarnings_.push_back(context + " too-low current is very small");
    }
    
    return isValid;
}

bool ConfigValidator::validateMechanicalParameters(const MotorConfig& config) {
    bool isValid = true;
    
    // Validate ticks per revolution
    if (config.ticksPerRevolution <= 0) {
        lastErrors_.push_back("Ticks per revolution must be positive");
        isValid = false;
    } else if (config.ticksPerRevolution < MIN_TICKS_PER_REV) {
        lastWarnings_.push_back("Ticks per revolution is very low, may affect precision");
    } else if (config.ticksPerRevolution > MAX_TICKS_PER_REV) {
        lastWarnings_.push_back("Ticks per revolution is very high, may affect performance");
    }
    
    // Validate wheel diameter
    if (config.wheelDiameter <= 0.0f) {
        lastErrors_.push_back("Wheel diameter must be positive");
        isValid = false;
    } else if (config.wheelDiameter < MIN_WHEEL_DIAMETER) {
        lastWarnings_.push_back("Wheel diameter is very small");
    } else if (config.wheelDiameter > MAX_WHEEL_DIAMETER) {
        lastWarnings_.push_back("Wheel diameter is very large");
    }
    
    // Validate wheel base
    if (config.wheelBase <= 0.0f) {
        lastErrors_.push_back("Wheel base must be positive");
        isValid = false;
    } else if (config.wheelBase < MIN_WHEEL_BASE) {
        lastWarnings_.push_back("Wheel base is very small, may affect stability");
    } else if (config.wheelBase > MAX_WHEEL_BASE) {
        lastWarnings_.push_back("Wheel base is very large");
    }
    
    // Check logical relationship
    if (config.wheelBase <= config.wheelDiameter) {
        lastWarnings_.push_back("Wheel base should typically be larger than wheel diameter");
    }
    
    return isValid;
}

bool ConfigValidator::validateCommunicationParameters(const ATProtocolConfig& config) {
    bool isValid = true;
    
    // Validate baudrate
    if (config.baudrate < MIN_BAUDRATE) {
        lastErrors_.push_back("Baudrate is too low");
        isValid = false;
    } else if (config.baudrate > MAX_BAUDRATE) {
        lastErrors_.push_back("Baudrate is too high");
        isValid = false;
    }
    
    // Check for common baudrates
    std::vector<int> commonBaudrates = {9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
    if (std::find(commonBaudrates.begin(), commonBaudrates.end(), config.baudrate) == commonBaudrates.end()) {
        lastWarnings_.push_back("Baudrate is not a common standard value");
    }
    
    // Validate timeout
    if (config.timeout < MIN_TIMEOUT) {
        lastErrors_.push_back("Timeout is too short");
        isValid = false;
    } else if (config.timeout > MAX_TIMEOUT) {
        lastErrors_.push_back("Timeout is too long");
        isValid = false;
    }
    
    // Validate retries
    if (config.retries < MIN_RETRIES) {
        lastErrors_.push_back("Retry count is too low");
        isValid = false;
    } else if (config.retries > MAX_RETRIES) {
        lastWarnings_.push_back("Retry count is very high, may cause delays");
    }
    
    return isValid;
}

bool ConfigValidator::validateSensorRanges(const SensorConfig& config) {
    bool isValid = true;
    
    // Validate IMU ranges
    if (config.imuAccelRange <= 0.0f || config.imuGyroRange <= 0.0f) {
        lastErrors_.push_back("IMU ranges must be positive");
        isValid = false;
    }
    
    // Validate GPS accuracy
    if (config.gpsAccuracyThreshold <= 0.0f) {
        lastErrors_.push_back("GPS accuracy threshold must be positive");
        isValid = false;
    }
    
    // Validate ultrasonic range
    if (config.ultrasonicMaxRange <= 0.0f) {
        lastErrors_.push_back("Ultrasonic max range must be positive");
        isValid = false;
    }
    
    return isValid;
}

bool ConfigValidator::validateBatteryLimits(const BatteryConfig& config) {
    bool isValid = true;
    
    // Validate voltage limits
    if (config.minVoltage <= 0.0f || config.maxVoltage <= 0.0f) {
        lastErrors_.push_back("Battery voltage limits must be positive");
        isValid = false;
    }
    
    if (config.minVoltage >= config.maxVoltage) {
        lastErrors_.push_back("Minimum voltage must be less than maximum voltage");
        isValid = false;
    }
    
    // Validate capacity
    if (config.capacity <= 0.0f) {
        lastErrors_.push_back("Battery capacity must be positive");
        isValid = false;
    }
    
    // Check reasonable ranges for typical robot batteries
    if (config.maxVoltage > 50.0f) {
        lastWarnings_.push_back("Battery voltage is very high - ensure safety measures");
    }
    
    if (config.minVoltage < 6.0f) {
        lastWarnings_.push_back("Minimum battery voltage is very low");
    }
    
    return isValid;
}
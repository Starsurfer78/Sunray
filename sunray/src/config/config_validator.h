// Ardumower Sunray - Alfred Platform
// Configuration Validator
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH

#ifndef CONFIG_VALIDATOR_H
#define CONFIG_VALIDATOR_H

#include "config_structures.h"
#include "../../config.h"
#include <vector>

class ConfigValidator {
public:
    struct ValidationResult {
        bool isValid;
        std::vector<RobotError> errors;
        std::vector<RobotError> warnings;
        String summary;
        
        ValidationResult() : isValid(true) {}
        
        void addError(RobotErrorCode code, const String& description) {
            errors.push_back(RobotError(code, description, false, 3));
            isValid = false;
        }
        
        void addWarning(RobotErrorCode code, const String& description) {
            warnings.push_back(RobotError(code, description, true, 2));
        }
        
        String getReport() const {
            String report = "Configuration Validation Report\n";
            report += "======================================\n";
            report += "Status: " + String(isValid ? "VALID" : "INVALID") + "\n";
            report += "Errors: " + String(errors.size()) + "\n";
            report += "Warnings: " + String(warnings.size()) + "\n\n";
            
            if (!errors.empty()) {
                report += "ERRORS:\n";
                for (const auto& error : errors) {
                    report += "  - " + error.toString() + "\n";
                }
                report += "\n";
            }
            
            if (!warnings.empty()) {
                report += "WARNINGS:\n";
                for (const auto& warning : warnings) {
                    report += "  - " + warning.toString() + "\n";
                }
                report += "\n";
            }
            
            if (!summary.isEmpty()) {
                report += "SUMMARY:\n" + summary + "\n";
            }
            
            return report;
        }
    };
    
    // Main validation methods
    static ValidationResult validateSystemConfig(const SystemConfig& config);
    static ValidationResult validateMotorConfig(const MotorConfig& config);
    static ValidationResult validateATProtocolConfig(const ATProtocolConfig& config);
    static ValidationResult validateSensorConfig(const SensorConfig& config);
    static ValidationResult validateBatteryConfig(const BatteryConfig& config);
    
    // Cross-component validation
    static ValidationResult validateMotorSensorIntegration(const MotorConfig& motor, const SensorConfig& sensor);
    static ValidationResult validateBatteryMotorIntegration(const BatteryConfig& battery, const MotorConfig& motor);
    
    // Hardware compatibility checks
    static ValidationResult validateHardwareCompatibility(const SystemConfig& config);
    
    // Performance validation
    static ValidationResult validatePerformanceSettings(const SystemConfig& config);
    
    // Configuration migration
    static bool migrateFromLegacyConfig(SystemConfig& config);
    
    // Utility methods
    static void printConfigSummary(const SystemConfig& config);
    static void printValidationReport(const ValidationResult& result);
    static bool saveValidationReport(const ValidationResult& result, const String& filename);
    
private:
    // Internal validation helpers
    static bool isValidPIDParameters(float kp, float ki, float kd);
    static bool isValidCurrentRange(float fault, float overload, float tooLow);
    static bool isValidVoltageRange(float under, float goHome, float full);
    static bool isValidMechanicalParameters(float diameter, float base, int ticks);
    static bool isValidBaudRate(unsigned long baudRate);
    static bool isValidTimeout(unsigned long timeout);
    
    // Hardware-specific validation
    static bool validateLinuxSpecificSettings(const SystemConfig& config);
    static bool validateArduinoSpecificSettings(const SystemConfig& config);
    
    // Performance analysis
    static float calculateExpectedPIDPerformance(const MotorConfig& config);
    static float calculateCommunicationOverhead(const ATProtocolConfig& config);
    static float calculatePowerConsumption(const SystemConfig& config);
};

// Implementation of validation methods
inline ConfigValidator::ValidationResult ConfigValidator::validateSystemConfig(const SystemConfig& config) {
    ValidationResult result;
    
    // Validate individual components
    auto motorResult = validateMotorConfig(config.motor);
    auto atResult = validateATProtocolConfig(config.atProtocol);
    auto sensorResult = validateSensorConfig(config.sensor);
    auto batteryResult = validateBatteryConfig(config.battery);
    
    // Merge results
    result.errors.insert(result.errors.end(), motorResult.errors.begin(), motorResult.errors.end());
    result.errors.insert(result.errors.end(), atResult.errors.begin(), atResult.errors.end());
    result.errors.insert(result.errors.end(), sensorResult.errors.begin(), sensorResult.errors.end());
    result.errors.insert(result.errors.end(), batteryResult.errors.begin(), batteryResult.errors.end());
    
    result.warnings.insert(result.warnings.end(), motorResult.warnings.begin(), motorResult.warnings.end());
    result.warnings.insert(result.warnings.end(), atResult.warnings.begin(), atResult.warnings.end());
    result.warnings.insert(result.warnings.end(), sensorResult.warnings.begin(), sensorResult.warnings.end());
    result.warnings.insert(result.warnings.end(), batteryResult.warnings.begin(), batteryResult.warnings.end());
    
    // Cross-component validation
    auto motorSensorResult = validateMotorSensorIntegration(config.motor, config.sensor);
    auto batteryMotorResult = validateBatteryMotorIntegration(config.battery, config.motor);
    
    result.errors.insert(result.errors.end(), motorSensorResult.errors.begin(), motorSensorResult.errors.end());
    result.errors.insert(result.errors.end(), batteryMotorResult.errors.begin(), batteryMotorResult.errors.end());
    
    // Hardware compatibility
    auto hardwareResult = validateHardwareCompatibility(config);
    result.errors.insert(result.errors.end(), hardwareResult.errors.begin(), hardwareResult.errors.end());
    result.warnings.insert(result.warnings.end(), hardwareResult.warnings.begin(), hardwareResult.warnings.end());
    
    // Performance validation
    auto performanceResult = validatePerformanceSettings(config);
    result.warnings.insert(result.warnings.end(), performanceResult.warnings.begin(), performanceResult.warnings.end());
    
    result.isValid = result.errors.empty();
    
    // Generate summary
    result.summary = "Configuration contains " + String(result.errors.size()) + " errors and " + 
                    String(result.warnings.size()) + " warnings.";
    if (result.isValid) {
        result.summary += " Configuration is valid for use.";
    } else {
        result.summary += " Configuration requires fixes before use.";
    }
    
    return result;
}

inline ConfigValidator::ValidationResult ConfigValidator::validateMotorConfig(const MotorConfig& config) {
    ValidationResult result;
    
    // PID parameter validation
    if (!isValidPIDParameters(config.pidKp, config.pidKi, config.pidKd)) {
        result.addError(RobotErrorCode::CONFIG_INVALID, "Invalid PID parameters");
    }
    
    if (config.pidKp <= 0) {
        result.addError(RobotErrorCode::CONFIG_INVALID, "PID Kp must be positive");
    }
    
    if (config.pidKi < 0) {
        result.addError(RobotErrorCode::CONFIG_INVALID, "PID Ki must be non-negative");
    }
    
    if (config.pidKd < 0) {
        result.addError(RobotErrorCode::CONFIG_INVALID, "PID Kd must be non-negative");
    }
    
    // Current validation
    if (!isValidCurrentRange(config.faultCurrent, config.overloadCurrent, config.tooLowCurrent)) {
        result.addError(RobotErrorCode::CONFIG_INVALID, "Invalid current thresholds");
    }
    
    if (config.faultCurrent <= config.overloadCurrent) {
        result.addError(RobotErrorCode::CONFIG_INVALID, "Fault current must be higher than overload current");
    }
    
    // Mow motor current validation
    if (!isValidCurrentRange(config.mowFaultCurrent, config.mowOverloadCurrent, config.mowTooLowCurrent)) {
        result.addError(RobotErrorCode::CONFIG_INVALID, "Invalid mow motor current thresholds");
    }
    
    // Mechanical parameter validation
    if (!isValidMechanicalParameters(config.wheelDiameter, config.wheelBase, config.ticksPerRevolution)) {
        result.addError(RobotErrorCode::CONFIG_INVALID, "Invalid mechanical parameters");
    }
    
    // Performance warnings
    if (config.pidKp > 5.0) {
        result.addWarning(RobotErrorCode::MOTOR_PID_UNSTABLE, "High PID Kp value may cause instability");
    }
    
    if (config.overloadSpeed > 0.5) {
        result.addWarning(RobotErrorCode::CONFIG_INVALID, "High overload speed may damage motors");
    }
    
    return result;
}

inline ConfigValidator::ValidationResult ConfigValidator::validateATProtocolConfig(const ATProtocolConfig& config) {
    ValidationResult result;
    
    // Baud rate validation
    if (!isValidBaudRate(config.baudRate)) {
        result.addError(RobotErrorCode::CONFIG_INVALID, "Invalid baud rate");
    }
    
    // Timeout validation
    if (!isValidTimeout(config.motorTimeout) || !isValidTimeout(config.summaryTimeout) || 
        !isValidTimeout(config.responseTimeout)) {
        result.addError(RobotErrorCode::CONFIG_INVALID, "Invalid timeout values");
    }
    
    if (config.motorTimeout > config.responseTimeout) {
        result.addWarning(RobotErrorCode::CONFIG_INVALID, "Motor timeout should be less than response timeout");
    }
    
    if (config.maxRetries > 10) {
        result.addWarning(RobotErrorCode::CONFIG_INVALID, "High retry count may cause delays");
    }
    
    return result;
}

inline ConfigValidator::ValidationResult ConfigValidator::validateSensorConfig(const SensorConfig& config) {
    ValidationResult result;
    
    // Bumper validation
    if (config.bumperDeadTime > 5000) {
        result.addWarning(RobotErrorCode::CONFIG_INVALID, "Long bumper dead time may affect responsiveness");
    }
    
    if (config.bumperMaxTriggerTime > 60000) {
        result.addWarning(RobotErrorCode::CONFIG_INVALID, "Long bumper max trigger time may cause false errors");
    }
    
    // Temperature validation
    if (config.dockOverheatTemp <= config.dockTooColdTemp) {
        result.addError(RobotErrorCode::CONFIG_INVALID, "Overheat temperature must be higher than too cold temperature");
    }
    
    if (config.dockOverheatTemp > 100) {
        result.addWarning(RobotErrorCode::CONFIG_INVALID, "Very high overheat temperature may damage components");
    }
    
    return result;
}

inline ConfigValidator::ValidationResult ConfigValidator::validateBatteryConfig(const BatteryConfig& config) {
    ValidationResult result;
    
    // Voltage validation
    if (!isValidVoltageRange(config.underVoltage, config.goHomeVoltage, config.fullVoltage)) {
        result.addError(RobotErrorCode::CONFIG_INVALID, "Invalid battery voltage thresholds");
    }
    
    if (config.underVoltage >= config.goHomeVoltage) {
        result.addError(RobotErrorCode::CONFIG_INVALID, "Undervoltage must be less than go-home voltage");
    }
    
    if (config.goHomeVoltage >= config.fullVoltage) {
        result.addError(RobotErrorCode::CONFIG_INVALID, "Go-home voltage must be less than full voltage");
    }
    
    // Current validation
    if (config.fullCurrent > 0) {
        result.addWarning(RobotErrorCode::CONFIG_INVALID, "Full current should typically be negative (charging)");
    }
    
    return result;
}

// Helper function implementations
inline bool ConfigValidator::isValidPIDParameters(float kp, float ki, float kd) {
    return (kp > 0 && kp < 100) && (ki >= 0 && ki < 10) && (kd >= 0 && kd < 10);
}

inline bool ConfigValidator::isValidCurrentRange(float fault, float overload, float tooLow) {
    return (fault > overload) && (overload > tooLow) && (tooLow >= 0) && (fault < 50);
}

inline bool ConfigValidator::isValidVoltageRange(float under, float goHome, float full) {
    return (under > 0) && (under < goHome) && (goHome < full) && (full < 50);
}

inline bool ConfigValidator::isValidMechanicalParameters(float diameter, float base, int ticks) {
    return (diameter > 50 && diameter < 500) && (base > 10 && base < 100) && (ticks > 0 && ticks < 10000);
}

inline bool ConfigValidator::isValidBaudRate(unsigned long baudRate) {
    const unsigned long validRates[] = {9600, 19200, 38400, 57600, 115200};
    for (auto rate : validRates) {
        if (baudRate == rate) return true;
    }
    return false;
}

inline bool ConfigValidator::isValidTimeout(unsigned long timeout) {
    return timeout > 0 && timeout < 60000; // 0-60 seconds
}

inline void ConfigValidator::printConfigSummary(const SystemConfig& config) {
    CONSOLE.println("=== Alfred Configuration Summary ===");
    CONSOLE.println("Config Version: " + config.configVersion);
    CONSOLE.println("Valid: " + String(config.isValid ? "Yes" : "No"));
    CONSOLE.println();
    
    CONSOLE.println("Motor Configuration:");
    CONSOLE.println("  PID: Kp=" + String(config.motor.pidKp) + ", Ki=" + String(config.motor.pidKi) + ", Kd=" + String(config.motor.pidKd));
    CONSOLE.println("  Current Limits: Fault=" + String(config.motor.faultCurrent) + "A, Overload=" + String(config.motor.overloadCurrent) + "A");
    CONSOLE.println("  Wheel: Diameter=" + String(config.motor.wheelDiameter) + "mm, Base=" + String(config.motor.wheelBase) + "cm");
    CONSOLE.println();
    
    CONSOLE.println("AT Protocol Configuration:");
    CONSOLE.println("  Baud Rate: " + String(config.atProtocol.baudRate));
    CONSOLE.println("  Timeouts: Motor=" + String(config.atProtocol.motorTimeout) + "ms, Summary=" + String(config.atProtocol.summaryTimeout) + "ms");
    CONSOLE.println();
    
    CONSOLE.println("Battery Configuration:");
    CONSOLE.println("  Voltages: Under=" + String(config.battery.underVoltage) + "V, GoHome=" + String(config.battery.goHomeVoltage) + "V, Full=" + String(config.battery.fullVoltage) + "V");
    CONSOLE.println();
}

#endif // CONFIG_VALIDATOR_H
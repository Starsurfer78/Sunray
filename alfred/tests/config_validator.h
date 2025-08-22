#ifndef CONFIG_VALIDATOR_H
#define CONFIG_VALIDATOR_H

#include "config_structures.h"
#include <string>
#include <vector>

/**
 * @brief Configuration validator for Alfred robot components
 * 
 * This class provides validation methods for all configuration structures
 * used in the Alfred robot system. It ensures that configuration parameters
 * are within valid ranges and logically consistent.
 */
class ConfigValidator {
public:
    struct ValidationResult {
        bool isValid;
        std::vector<std::string> errors;
        std::vector<std::string> warnings;
        
        ValidationResult() : isValid(true) {}
    };
    
    ConfigValidator();
    ~ConfigValidator() = default;
    
    // Motor configuration validation
    bool validateMotorConfig(const MotorConfig& config);
    ValidationResult validateMotorConfigDetailed(const MotorConfig& config);
    
    // AT Protocol configuration validation
    bool validateATProtocolConfig(const ATProtocolConfig& config);
    ValidationResult validateATProtocolConfigDetailed(const ATProtocolConfig& config);
    
    // Robot configuration validation
    bool validateRobotConfig(const RobotConfig& config);
    ValidationResult validateRobotConfigDetailed(const RobotConfig& config);
    
    // Sensor configuration validation
    bool validateSensorConfig(const SensorConfig& config);
    ValidationResult validateSensorConfigDetailed(const SensorConfig& config);
    
    // Battery configuration validation
    bool validateBatteryConfig(const BatteryConfig& config);
    ValidationResult validateBatteryConfigDetailed(const BatteryConfig& config);
    
    // Cross-configuration consistency checks
    bool validateConfigurationConsistency(const MotorConfig& motor,
                                        const ATProtocolConfig& atProtocol,
                                        const RobotConfig& robot);
    
    // Get last validation errors
    const std::vector<std::string>& getLastErrors() const { return lastErrors_; }
    const std::vector<std::string>& getLastWarnings() const { return lastWarnings_; }
    
    // Clear validation history
    void clearValidationHistory();

private:
    std::vector<std::string> lastErrors_;
    std::vector<std::string> lastWarnings_;
    
    // Helper methods for specific validations
    bool validatePIDParameters(float kp, float ki, float kd, const std::string& context);
    bool validateCurrentLimits(float tooLow, float overload, float fault, const std::string& context);
    bool validateMechanicalParameters(const MotorConfig& config);
    bool validateCommunicationParameters(const ATProtocolConfig& config);
    bool validateSensorRanges(const SensorConfig& config);
    bool validateBatteryLimits(const BatteryConfig& config);
    
    // Validation constants
    static constexpr float MIN_PID_KP = 0.001f;
    static constexpr float MAX_PID_KP = 10.0f;
    static constexpr float MAX_PID_KI = 5.0f;
    static constexpr float MAX_PID_KD = 2.0f;
    
    static constexpr float MIN_CURRENT = 0.1f;
    static constexpr float MAX_CURRENT = 20.0f;
    
    static constexpr int MIN_BAUDRATE = 9600;
    static constexpr int MAX_BAUDRATE = 1000000;
    
    static constexpr int MIN_TIMEOUT = 10;
    static constexpr int MAX_TIMEOUT = 5000;
    
    static constexpr int MIN_RETRIES = 1;
    static constexpr int MAX_RETRIES = 10;
    
    static constexpr float MIN_WHEEL_DIAMETER = 0.05f;  // 5cm
    static constexpr float MAX_WHEEL_DIAMETER = 0.5f;   // 50cm
    
    static constexpr float MIN_WHEEL_BASE = 0.1f;       // 10cm
    static constexpr float MAX_WHEEL_BASE = 2.0f;       // 2m
    
    static constexpr int MIN_TICKS_PER_REV = 100;
    static constexpr int MAX_TICKS_PER_REV = 10000;
};

#endif // CONFIG_VALIDATOR_H
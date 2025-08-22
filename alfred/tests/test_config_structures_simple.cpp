#include <iostream>
#include <limits>
#include <cmath>
#include <chrono>

// Include the configuration files
#include "config_alfred.h"
#include "config_structures.h"
#include "config_validator.h"

// Test macros
#define EXPECT_TRUE(condition) \
    if (!(condition)) { \
        std::cout << "ASSERTION FAILED: " << #condition << " at line " << __LINE__ << std::endl; \
        return false; \
    }

#define EXPECT_FALSE(condition) \
    if (condition) { \
        std::cout << "ASSERTION FAILED: " << #condition << " should be false at line " << __LINE__ << std::endl; \
        return false; \
    }

#define EXPECT_EQ(expected, actual) \
    if ((expected) != (actual)) { \
        std::cout << "ASSERTION FAILED: Expected " << (expected) << " but got " << (actual) << " at line " << __LINE__ << std::endl; \
        return false; \
    }

#define EXPECT_GT(val1, val2) \
    if (!((val1) > (val2))) { \
        std::cout << "ASSERTION FAILED: " << (val1) << " should be greater than " << (val2) << " at line " << __LINE__ << std::endl; \
        return false; \
    }

#define EXPECT_LT(val1, val2) \
    if (!((val1) < (val2))) { \
        std::cout << "ASSERTION FAILED: " << (val1) << " should be less than " << (val2) << " at line " << __LINE__ << std::endl; \
        return false; \
    }

// Test functions implementation

bool test_config_structures_default_values() {
    MotorConfig motorConfig;
    ATProtocolConfig atProtocolConfig;
    
    // Test that default values are correctly set from config_alfred.h
    EXPECT_EQ(motorConfig.pidKp, MOTOR_PID_KP);
    EXPECT_EQ(motorConfig.pidKi, MOTOR_PID_KI);
    EXPECT_EQ(motorConfig.pidKd, MOTOR_PID_KD);
    EXPECT_EQ(motorConfig.faultCurrent, MOTOR_FAULT_CURRENT);
    EXPECT_EQ(motorConfig.overloadCurrent, MOTOR_OVERLOAD_CURRENT);
    EXPECT_EQ(motorConfig.tooLowCurrent, MOTOR_TOO_LOW_CURRENT);
    EXPECT_EQ(motorConfig.ticksPerRevolution, TICKS_PER_REVOLUTION);
    EXPECT_EQ(motorConfig.wheelDiameter, WHEEL_DIAMETER);
    EXPECT_EQ(motorConfig.wheelBase, WHEEL_BASE);
    
    EXPECT_EQ(atProtocolConfig.baudrate, ROBOT_BAUDRATE);
    EXPECT_EQ(atProtocolConfig.timeout, AT_PROTOCOL_TIMEOUT);
    EXPECT_EQ(atProtocolConfig.retries, AT_PROTOCOL_RETRIES);
    EXPECT_TRUE(atProtocolConfig.crcEnabled);
    EXPECT_TRUE(atProtocolConfig.debugEnabled);
    
    return true;
}

bool test_config_structures_validation() {
    ConfigValidator validator;
    MotorConfig motorConfig;
    ATProtocolConfig atProtocolConfig;
    
    // Test valid configurations
    EXPECT_TRUE(validator.validateMotorConfig(motorConfig));
    EXPECT_TRUE(validator.validateATProtocolConfig(atProtocolConfig));
    
    // Test invalid PID parameters
    motorConfig.pidKp = -1.0f;  // Negative Kp should be invalid
    EXPECT_FALSE(validator.validateMotorConfig(motorConfig));
    
    // Reset to valid value
    motorConfig.pidKp = MOTOR_PID_KP;
    
    // Test invalid current limits
    motorConfig.faultCurrent = 0.0f;  // Zero fault current should be invalid
    EXPECT_FALSE(validator.validateMotorConfig(motorConfig));
    
    motorConfig.faultCurrent = MOTOR_FAULT_CURRENT;
    motorConfig.overloadCurrent = motorConfig.faultCurrent + 1.0f;  // Overload > fault should be invalid
    EXPECT_FALSE(validator.validateMotorConfig(motorConfig));
    
    return true;
}

bool test_config_structures_edge_cases() {
    ConfigValidator validator;
    MotorConfig config;
    
    // Test with very small positive values
    config.pidKp = 0.001f;
    config.pidKi = 0.0f;    // Zero Ki should be valid
    config.pidKd = 0.0f;    // Zero Kd should be valid
    EXPECT_TRUE(validator.validateMotorConfig(config));
    
    // Test zero mechanical parameters (should be invalid)
    config.ticksPerRevolution = 0;
    EXPECT_FALSE(validator.validateMotorConfig(config));
    
    config.ticksPerRevolution = TICKS_PER_REVOLUTION;  // Reset
    config.wheelDiameter = 0.0f;
    EXPECT_FALSE(validator.validateMotorConfig(config));
    
    config.wheelDiameter = WHEEL_DIAMETER;  // Reset
    config.wheelBase = 0.0f;
    EXPECT_FALSE(validator.validateMotorConfig(config));
    
    return true;
}

bool test_config_validator_motor_config() {
    ConfigValidator validator;
    MotorConfig config;
    
    // Test current limit ordering
    config.tooLowCurrent = 2.0f;
    config.overloadCurrent = 2.0f;  // Same as tooLow (should be invalid)
    config.faultCurrent = 5.0f;
    EXPECT_FALSE(validator.validateMotorConfig(config));
    
    // Test proper ordering
    config.tooLowCurrent = 1.0f;
    config.overloadCurrent = 3.0f;
    config.faultCurrent = 5.0f;
    EXPECT_TRUE(validator.validateMotorConfig(config));
    
    // Test mechanical parameter validation
    config.wheelBase = config.wheelDiameter;  // Should generate warning but be valid
    EXPECT_TRUE(validator.validateMotorConfig(config));
    
    return true;
}

bool test_config_validator_at_protocol() {
    ConfigValidator validator;
    ATProtocolConfig config;
    
    // Test invalid baudrates
    config.baudrate = 1200;  // Too low
    EXPECT_FALSE(validator.validateATProtocolConfig(config));
    
    config.baudrate = 5000000;  // Too high
    EXPECT_FALSE(validator.validateATProtocolConfig(config));
    
    // Test valid baudrate
    config.baudrate = 115200;
    EXPECT_TRUE(validator.validateATProtocolConfig(config));
    
    // Test invalid timeout
    config.timeout = 0;
    EXPECT_FALSE(validator.validateATProtocolConfig(config));
    
    config.timeout = 10000;  // Too high
    EXPECT_FALSE(validator.validateATProtocolConfig(config));
    
    // Test valid timeout
    config.timeout = 1000;
    EXPECT_TRUE(validator.validateATProtocolConfig(config));
    
    return true;
}

bool test_config_validator_consistency() {
    ConfigValidator validator;
    MotorConfig motorConfig;
    ATProtocolConfig atConfig;
    RobotConfig robotConfig;
    
    // Set up inconsistent configuration
    motorConfig.wheelBase = 1.0f;
    robotConfig.length = 0.5f;  // Wheel base larger than robot length
    robotConfig.width = 0.4f;
    robotConfig.height = 0.3f;
    robotConfig.weight = 10.0f;
    robotConfig.name = "TestRobot";
    
    EXPECT_FALSE(validator.validateConfigurationConsistency(motorConfig, atConfig, robotConfig));
    
    // Fix the inconsistency
    robotConfig.length = 1.5f;
    EXPECT_TRUE(validator.validateConfigurationConsistency(motorConfig, atConfig, robotConfig));
    
    return true;
}

bool test_config_performance() {
    ConfigValidator validator;
    MotorConfig motorConfig;
    ATProtocolConfig atConfig;
    
    auto start = std::chrono::high_resolution_clock::now();
    
    // Validate configuration multiple times
    for (int i = 0; i < 1000; ++i) {
        validator.validateMotorConfig(motorConfig);
        validator.validateATProtocolConfig(atConfig);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    // Validation should be fast (less than 10ms for 1000 validations)
    EXPECT_LT(duration.count(), 10000);
    
    return true;
}
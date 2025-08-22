#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <limits>
#include <cmath>

#include "config_structures.h"
#include "config_validator.h"

class ConfigErrorHandlingTest : public ::testing::Test {
protected:
    void SetUp() override {
        validator_ = std::make_unique<ConfigValidator>();
    }
    
    void TearDown() override {
        validator_.reset();
    }
    
    std::unique_ptr<ConfigValidator> validator_;
};

// Test extreme values and edge cases
TEST_F(ConfigErrorHandlingTest, ExtremeValues) {
    MotorConfig config;
    
    // Test with infinity
    config.pidKp = std::numeric_limits<float>::infinity();
    EXPECT_FALSE(validator_->validateMotorConfig(config));
    
    // Test with NaN
    config.pidKp = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(validator_->validateMotorConfig(config));
    
    // Test with very large values
    config.pidKp = std::numeric_limits<float>::max();
    EXPECT_FALSE(validator_->validateMotorConfig(config));
    
    // Test with very small positive values
    config.pidKp = std::numeric_limits<float>::min();
    EXPECT_TRUE(validator_->validateMotorConfig(config));  // Should be valid but may warn
}

TEST_F(ConfigErrorHandlingTest, NegativeValues) {
    MotorConfig config;
    
    // Test negative PID values
    config.pidKp = -1.0f;
    EXPECT_FALSE(validator_->validateMotorConfig(config));
    
    config.pidKp = MOTOR_PID_KP;  // Reset
    config.pidKi = -0.5f;
    EXPECT_FALSE(validator_->validateMotorConfig(config));
    
    config.pidKi = MOTOR_PID_KI;  // Reset
    config.pidKd = -0.1f;
    EXPECT_FALSE(validator_->validateMotorConfig(config));
}

TEST_F(ConfigErrorHandlingTest, ZeroValues) {
    MotorConfig config;
    
    // Zero PID values should be valid (except Kp)
    config.pidKp = 0.0f;
    EXPECT_FALSE(validator_->validateMotorConfig(config));  // Kp = 0 should be invalid
    
    config.pidKp = MOTOR_PID_KP;  // Reset
    config.pidKi = 0.0f;
    EXPECT_TRUE(validator_->validateMotorConfig(config));   // Ki = 0 should be valid
    
    config.pidKd = 0.0f;
    EXPECT_TRUE(validator_->validateMotorConfig(config));   // Kd = 0 should be valid
    
    // Zero mechanical parameters should be invalid
    config.ticksPerRevolution = 0;
    EXPECT_FALSE(validator_->validateMotorConfig(config));
    
    config.ticksPerRevolution = TICKS_PER_REVOLUTION;  // Reset
    config.wheelDiameter = 0.0f;
    EXPECT_FALSE(validator_->validateMotorConfig(config));
    
    config.wheelDiameter = WHEEL_DIAMETER;  // Reset
    config.wheelBase = 0.0f;
    EXPECT_FALSE(validator_->validateMotorConfig(config));
}

TEST_F(ConfigErrorHandlingTest, CurrentLimitEdgeCases) {
    MotorConfig config;
    
    // Test equal current limits (should be invalid)
    config.tooLowCurrent = 2.0f;
    config.overloadCurrent = 2.0f;  // Same as tooLow
    config.faultCurrent = 5.0f;
    EXPECT_FALSE(validator_->validateMotorConfig(config));
    
    // Test overload >= fault (should be invalid)
    config.tooLowCurrent = 1.0f;
    config.overloadCurrent = 5.0f;
    config.faultCurrent = 5.0f;  // Same as overload
    EXPECT_FALSE(validator_->validateMotorConfig(config));
    
    // Test overload > fault (should be invalid)
    config.overloadCurrent = 6.0f;
    config.faultCurrent = 5.0f;  // Less than overload
    EXPECT_FALSE(validator_->validateMotorConfig(config));
}

TEST_F(ConfigErrorHandlingTest, ATProtocolEdgeCases) {
    ATProtocolConfig config;
    
    // Test invalid baudrates
    config.baudrate = 0;
    EXPECT_FALSE(validator_->validateATProtocolConfig(config));
    
    config.baudrate = -115200;
    EXPECT_FALSE(validator_->validateATProtocolConfig(config));
    
    config.baudrate = 1200;  // Too low
    EXPECT_FALSE(validator_->validateATProtocolConfig(config));
    
    config.baudrate = 5000000;  // Too high
    EXPECT_FALSE(validator_->validateATProtocolConfig(config));
    
    // Test invalid timeouts
    config.baudrate = ROBOT_BAUDRATE;  // Reset
    config.timeout = 0;
    EXPECT_FALSE(validator_->validateATProtocolConfig(config));
    
    config.timeout = -100;
    EXPECT_FALSE(validator_->validateATProtocolConfig(config));
    
    // Test invalid retries
    config.timeout = AT_PROTOCOL_TIMEOUT;  // Reset
    config.retries = 0;
    EXPECT_FALSE(validator_->validateATProtocolConfig(config));
    
    config.retries = -1;
    EXPECT_FALSE(validator_->validateATProtocolConfig(config));
}

TEST_F(ConfigErrorHandlingTest, DetailedValidationResults) {
    MotorConfig config;
    config.pidKp = -1.0f;  // Invalid
    config.pidKi = 10.0f;  // Warning level
    config.faultCurrent = 0.0f;  // Invalid
    
    auto result = validator_->validateMotorConfigDetailed(config);
    
    EXPECT_FALSE(result.isValid);
    EXPECT_GT(result.errors.size(), 0);
    EXPECT_GT(result.warnings.size(), 0);
    
    // Check that specific errors are reported
    bool foundKpError = false;
    bool foundCurrentError = false;
    
    for (const auto& error : result.errors) {
        if (error.find("Kp") != std::string::npos) {
            foundKpError = true;
        }
        if (error.find("fault current") != std::string::npos) {
            foundCurrentError = true;
        }
    }
    
    EXPECT_TRUE(foundKpError);
    EXPECT_TRUE(foundCurrentError);
}

TEST_F(ConfigErrorHandlingTest, ConsistencyValidation) {
    MotorConfig motorConfig;
    ATProtocolConfig atConfig;
    RobotConfig robotConfig;
    
    // Set up inconsistent configuration
    motorConfig.wheelBase = 1.0f;
    robotConfig.length = 0.5f;  // Wheel base larger than robot length
    
    EXPECT_FALSE(validator_->validateConfigurationConsistency(motorConfig, atConfig, robotConfig));
    
    // Fix the inconsistency
    robotConfig.length = 1.5f;
    EXPECT_TRUE(validator_->validateConfigurationConsistency(motorConfig, atConfig, robotConfig));
}

TEST_F(ConfigErrorHandlingTest, RobotConfigValidation) {
    RobotConfig config;
    
    // Test empty name
    config.name = "";
    EXPECT_FALSE(validator_->validateRobotConfig(config));
    
    // Test negative dimensions
    config.name = "TestRobot";
    config.length = -1.0f;
    EXPECT_FALSE(validator_->validateRobotConfig(config));
    
    config.length = 1.0f;
    config.width = -0.5f;
    EXPECT_FALSE(validator_->validateRobotConfig(config));
    
    config.width = 0.5f;
    config.height = -0.3f;
    EXPECT_FALSE(validator_->validateRobotConfig(config));
    
    // Test zero dimensions
    config.height = 0.0f;
    EXPECT_FALSE(validator_->validateRobotConfig(config));
    
    // Test negative weight
    config.height = 0.3f;
    config.weight = -10.0f;
    EXPECT_FALSE(validator_->validateRobotConfig(config));
    
    // Test zero weight
    config.weight = 0.0f;
    EXPECT_FALSE(validator_->validateRobotConfig(config));
}

TEST_F(ConfigErrorHandlingTest, SensorConfigValidation) {
    SensorConfig config;
    
    // Test negative IMU ranges
    config.imuAccelRange = -16.0f;
    EXPECT_FALSE(validator_->validateSensorConfig(config));
    
    config.imuAccelRange = 16.0f;
    config.imuGyroRange = -2000.0f;
    EXPECT_FALSE(validator_->validateSensorConfig(config));
    
    // Test zero ranges
    config.imuGyroRange = 0.0f;
    EXPECT_FALSE(validator_->validateSensorConfig(config));
    
    // Test negative GPS accuracy
    config.imuGyroRange = 2000.0f;
    config.gpsAccuracyThreshold = -1.0f;
    EXPECT_FALSE(validator_->validateSensorConfig(config));
    
    // Test negative ultrasonic range
    config.gpsAccuracyThreshold = 3.0f;
    config.ultrasonicMaxRange = -4.0f;
    EXPECT_FALSE(validator_->validateSensorConfig(config));
}

TEST_F(ConfigErrorHandlingTest, BatteryConfigValidation) {
    BatteryConfig config;
    
    // Test negative voltages
    config.minVoltage = -12.0f;
    EXPECT_FALSE(validator_->validateBatteryConfig(config));
    
    config.minVoltage = 10.0f;
    config.maxVoltage = -14.4f;
    EXPECT_FALSE(validator_->validateBatteryConfig(config));
    
    // Test min >= max voltage
    config.maxVoltage = 10.0f;  // Same as min
    EXPECT_FALSE(validator_->validateBatteryConfig(config));
    
    config.maxVoltage = 9.0f;   // Less than min
    EXPECT_FALSE(validator_->validateBatteryConfig(config));
    
    // Test negative capacity
    config.maxVoltage = 14.4f;
    config.capacity = -5000.0f;
    EXPECT_FALSE(validator_->validateBatteryConfig(config));
    
    // Test zero capacity
    config.capacity = 0.0f;
    EXPECT_FALSE(validator_->validateBatteryConfig(config));
}

TEST_F(ConfigErrorHandlingTest, ValidationHistoryManagement) {
    MotorConfig config;
    config.pidKp = -1.0f;  // This will generate an error
    
    // First validation
    EXPECT_FALSE(validator_->validateMotorConfig(config));
    EXPECT_GT(validator_->getLastErrors().size(), 0);
    
    // Clear history
    validator_->clearValidationHistory();
    EXPECT_EQ(validator_->getLastErrors().size(), 0);
    EXPECT_EQ(validator_->getLastWarnings().size(), 0);
    
    // Validate again
    EXPECT_FALSE(validator_->validateMotorConfig(config));
    EXPECT_GT(validator_->getLastErrors().size(), 0);
}

// Test performance with many validations
TEST_F(ConfigErrorHandlingTest, ValidationPerformanceStress) {
    MotorConfig config;
    ATProtocolConfig atConfig;
    
    auto start = std::chrono::high_resolution_clock::now();
    
    // Perform many validations
    for (int i = 0; i < 10000; ++i) {
        validator_->validateMotorConfig(config);
        validator_->validateATProtocolConfig(atConfig);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    // Should complete in reasonable time (less than 100ms for 10000 validations)
    EXPECT_LT(duration.count(), 100);
}
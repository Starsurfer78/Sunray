#include <gtest/gtest.h>
#include <gmock/gmock.h>

// Include the config structures we want to test
#include "config_structures.h"
#include "config_validator.h"
#include "config_alfred.h"

class ConfigStructuresTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize with default values
        motorConfig_ = MotorConfig();
        atProtocolConfig_ = ATProtocolConfig();
        robotConfig_ = RobotConfig();
        validator_ = std::make_unique<ConfigValidator>();
    }
    
    void TearDown() override {
        validator_.reset();
    }
    
    MotorConfig motorConfig_;
    ATProtocolConfig atProtocolConfig_;
    RobotConfig robotConfig_;
    std::unique_ptr<ConfigValidator> validator_;
};

// Test MotorConfig structure initialization and validation
TEST_F(ConfigStructuresTest, MotorConfigDefaultValues) {
    // Test that default values are correctly set from config_alfred.h
    EXPECT_EQ(motorConfig_.pidKp, MOTOR_PID_KP);
    EXPECT_EQ(motorConfig_.pidKi, MOTOR_PID_KI);
    EXPECT_EQ(motorConfig_.pidKd, MOTOR_PID_KD);
    EXPECT_EQ(motorConfig_.faultCurrent, MOTOR_FAULT_CURRENT);
    EXPECT_EQ(motorConfig_.overloadCurrent, MOTOR_OVERLOAD_CURRENT);
    EXPECT_EQ(motorConfig_.tooLowCurrent, MOTOR_TOO_LOW_CURRENT);
    EXPECT_EQ(motorConfig_.ticksPerRevolution, TICKS_PER_REVOLUTION);
    EXPECT_EQ(motorConfig_.wheelDiameter, WHEEL_DIAMETER);
    EXPECT_EQ(motorConfig_.wheelBase, WHEEL_BASE);
}

TEST_F(ConfigStructuresTest, MotorConfigValidation) {
    // Test valid configuration
    EXPECT_TRUE(validator_->validateMotorConfig(motorConfig_));
    
    // Test invalid PID parameters
    motorConfig_.pidKp = -1.0f;  // Negative Kp should be invalid
    EXPECT_FALSE(validator_->validateMotorConfig(motorConfig_));
    
    motorConfig_.pidKp = 10.0f;  // Too high Kp should trigger warning
    EXPECT_TRUE(validator_->validateMotorConfig(motorConfig_));  // Still valid but with warning
    
    // Reset to valid value
    motorConfig_.pidKp = MOTOR_PID_KP;
    
    // Test invalid current limits
    motorConfig_.faultCurrent = 0.0f;  // Zero fault current should be invalid
    EXPECT_FALSE(validator_->validateMotorConfig(motorConfig_));
    
    motorConfig_.faultCurrent = MOTOR_FAULT_CURRENT;
    motorConfig_.overloadCurrent = motorConfig_.faultCurrent + 1.0f;  // Overload > fault should be invalid
    EXPECT_FALSE(validator_->validateMotorConfig(motorConfig_));
}

TEST_F(ConfigStructuresTest, MotorConfigMechanicalParameters) {
    // Test mechanical parameter validation
    motorConfig_.ticksPerRevolution = 0;  // Invalid
    EXPECT_FALSE(validator_->validateMotorConfig(motorConfig_));
    
    motorConfig_.ticksPerRevolution = TICKS_PER_REVOLUTION;
    motorConfig_.wheelDiameter = 0.0f;  // Invalid
    EXPECT_FALSE(validator_->validateMotorConfig(motorConfig_));
    
    motorConfig_.wheelDiameter = WHEEL_DIAMETER;
    motorConfig_.wheelBase = 0.0f;  // Invalid
    EXPECT_FALSE(validator_->validateMotorConfig(motorConfig_));
}

// Test ATProtocolConfig structure
TEST_F(ConfigStructuresTest, ATProtocolConfigDefaultValues) {
    EXPECT_EQ(atProtocolConfig_.baudrate, ROBOT_BAUDRATE);
    EXPECT_EQ(atProtocolConfig_.timeout, AT_PROTOCOL_TIMEOUT);
    EXPECT_EQ(atProtocolConfig_.retries, AT_PROTOCOL_RETRIES);
    EXPECT_TRUE(atProtocolConfig_.crcEnabled);
    EXPECT_TRUE(atProtocolConfig_.debugEnabled);
}

TEST_F(ConfigStructuresTest, ATProtocolConfigValidation) {
    // Test valid configuration
    EXPECT_TRUE(validator_->validateATProtocolConfig(atProtocolConfig_));
    
    // Test invalid baudrate
    atProtocolConfig_.baudrate = 1200;  // Too low
    EXPECT_FALSE(validator_->validateATProtocolConfig(atProtocolConfig_));
    
    atProtocolConfig_.baudrate = 2000000;  // Too high
    EXPECT_FALSE(validator_->validateATProtocolConfig(atProtocolConfig_));
    
    // Reset to valid value
    atProtocolConfig_.baudrate = ROBOT_BAUDRATE;
    
    // Test invalid timeout
    atProtocolConfig_.timeout = 0;  // Zero timeout should be invalid
    EXPECT_FALSE(validator_->validateATProtocolConfig(atProtocolConfig_));
    
    atProtocolConfig_.timeout = 10000;  // Too high timeout
    EXPECT_FALSE(validator_->validateATProtocolConfig(atProtocolConfig_));
}

// Test RobotErrorCode enum
TEST_F(ConfigStructuresTest, RobotErrorCodeValues) {
    // Test that error codes have expected values
    EXPECT_EQ(static_cast<int>(RobotErrorCode::NONE), 0);
    EXPECT_EQ(static_cast<int>(RobotErrorCode::MOTOR_FAULT), 1);
    EXPECT_EQ(static_cast<int>(RobotErrorCode::MOTOR_OVERLOAD), 2);
    EXPECT_EQ(static_cast<int>(RobotErrorCode::COMM_LOST), 3);
    EXPECT_EQ(static_cast<int>(RobotErrorCode::COMM_TIMEOUT), 4);
    EXPECT_EQ(static_cast<int>(RobotErrorCode::COMM_CRC_ERROR), 5);
}

// Test configuration consistency between structures
TEST_F(ConfigStructuresTest, ConfigurationConsistency) {
    // Test that motor configuration is consistent with AT protocol
    EXPECT_GT(atProtocolConfig_.timeout, 0);
    EXPECT_GT(motorConfig_.pidKp, 0.0f);
    
    // Test that current limits are in logical order
    EXPECT_LT(motorConfig_.tooLowCurrent, motorConfig_.overloadCurrent);
    EXPECT_LT(motorConfig_.overloadCurrent, motorConfig_.faultCurrent);
    
    // Test that mechanical parameters make sense
    EXPECT_GT(motorConfig_.wheelDiameter, 0.0f);
    EXPECT_GT(motorConfig_.wheelBase, motorConfig_.wheelDiameter);
    EXPECT_GT(motorConfig_.ticksPerRevolution, 0);
}

// Test configuration serialization/deserialization (if implemented)
TEST_F(ConfigStructuresTest, ConfigurationSerialization) {
    // This test would verify that configurations can be saved/loaded
    // Implementation depends on whether serialization is needed
    
    // For now, just test that structures can be copied
    MotorConfig copy = motorConfig_;
    EXPECT_EQ(copy.pidKp, motorConfig_.pidKp);
    EXPECT_EQ(copy.pidKi, motorConfig_.pidKi);
    EXPECT_EQ(copy.pidKd, motorConfig_.pidKd);
}

// Test edge cases and boundary conditions
TEST_F(ConfigStructuresTest, BoundaryConditions) {
    // Test minimum valid values
    motorConfig_.pidKp = 0.001f;  // Very small but positive
    motorConfig_.pidKi = 0.0f;    // Zero Ki should be valid
    motorConfig_.pidKd = 0.0f;    // Zero Kd should be valid
    EXPECT_TRUE(validator_->validateMotorConfig(motorConfig_));
    
    // Test maximum reasonable values
    motorConfig_.pidKp = 5.0f;    // High but reasonable Kp
    motorConfig_.pidKi = 1.0f;    // High but reasonable Ki
    motorConfig_.pidKd = 1.0f;    // High but reasonable Kd
    EXPECT_TRUE(validator_->validateMotorConfig(motorConfig_));
}

// Test configuration migration (if implemented)
TEST_F(ConfigStructuresTest, ConfigurationMigration) {
    // This would test migration from old config formats
    // For now, just verify that default initialization works
    MotorConfig defaultConfig;
    EXPECT_TRUE(validator_->validateMotorConfig(defaultConfig));
    
    ATProtocolConfig defaultATConfig;
    EXPECT_TRUE(validator_->validateATProtocolConfig(defaultATConfig));
}

// Performance test for configuration validation
TEST_F(ConfigStructuresTest, ValidationPerformance) {
    auto start = std::chrono::high_resolution_clock::now();
    
    // Validate configuration multiple times
    for (int i = 0; i < 1000; ++i) {
        validator_->validateMotorConfig(motorConfig_);
        validator_->validateATProtocolConfig(atProtocolConfig_);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    // Validation should be fast (less than 1ms for 1000 validations)
    EXPECT_LT(duration.count(), 1000);
}
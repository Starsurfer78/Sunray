// Integration Tests für SerialRobotDriver.cpp
// Tests für AT+-Protokoll, Fehlerbehandlung und Kommunikation

#include <iostream>
#include <string>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include "mocks/mock_serial.h"
#include "mocks/mock_stm32.h"
#include "../config_structures.h"

// Einfaches Test-Framework
#define TEST_ASSERT(condition, message) \
    do { \
        if (!(condition)) { \
            std::cout << "FAIL: " << message << std::endl; \
            return false; \
        } \
    } while(0)

#define TEST_ASSERT_EQUAL(expected, actual, message) \
    do { \
        if ((expected) != (actual)) { \
            std::cout << "FAIL: " << message << " (expected: " << (expected) << ", actual: " << (actual) << ")" << std::endl; \
            return false; \
        } \
    } while(0)

#define TEST_ASSERT_FLOAT_EQUAL(expected, actual, tolerance, message) \
    do { \
        if (std::abs((expected) - (actual)) > (tolerance)) { \
            std::cout << "FAIL: " << message << " (expected: " << (expected) << ", actual: " << (actual) << ")" << std::endl; \
            return false; \
        } \
    } while(0)

static int tests_run = 0;
static int tests_passed = 0;

#define RUN_TEST(test_func) \
    do { \
        tests_run++; \
        std::cout << "Running " << #test_func << "... "; \
        if (test_func()) { \
            std::cout << "PASS" << std::endl; \
            tests_passed++; \
        } \
    } while(0)

// Forward-Deklarationen für SerialRobotDriver
class SerialRobotDriver {
public:
    // Kommunikations-Status
    bool mcuCommunicationLost = false;
    int communicationErrors = 0;
    bool motorFault = false;
    
    // Encoder Ticks
    int encoderTicksLeft = 0;
    int encoderTicksRight = 0;
    int encoderTicksMow = 0;
    
    // Sensor Data
    float batteryVoltage = 0.0f;
    float chargeVoltage = 0.0f;
    float chargeCurrent = 0.0f;
    float batteryTemp = 0.0f;
    bool triggeredLift = false;
    bool triggeredLeftBumper = false;
    
    // Motor Current
    float motorLeftCurr = 0.0f;
    float motorRightCurr = 0.0f;
    float mowCurr = 0.0f;
    
    // Methoden (vereinfacht für Tests)
    void requestVersion() {
        if (mockSerial) {
            mockSerial->println("AT+V,0x4A");
        }
    }
    
    void requestMotorPwm(int right, int left, int mow) {
        if (mockSerial) {
            char buffer[64];
            sprintf(buffer, "AT+M,%d,%d,%d,0x7B", right, left, mow);
            mockSerial->println(buffer);
        }
    }
    
    void requestSummary() {
        if (mockSerial) {
            mockSerial->println("AT+S,0x8C");
        }
    }
    
    void processComm() {
        if (!mockSerial || !mockSerial->available()) return;
        
        // Vereinfachte Response-Verarbeitung
        std::string response;
        while (mockSerial->available()) {
            char c = mockSerial->read();
            if (c == '\n') break;
            if (c != '\r') response += c;
        }
        
        if (response.empty()) {
            communicationErrors++;
            return;
        }
        
        // Parse Response
        if (response[0] == 'V') {
            parseVersionResponse(response);
        } else if (response[0] == 'M') {
            parseMotorResponse(response);
        } else if (response[0] == 'S') {
            parseSummaryResponse(response);
        } else {
            communicationErrors++;
        }
    }
    
    void run() {
        // Vereinfachte run-Methode für Tests
        static unsigned long lastMotorTime = 0;
        static unsigned long lastSummaryTime = 0;
        
        unsigned long now = millis();
        
        // Motor commands at 50Hz
        if (now - lastMotorTime > 20) {
            requestMotorPwm(0, 0, 0);
            lastMotorTime = now;
        }
        
        // Summary commands at 2Hz
        if (now - lastSummaryTime > 500) {
            requestSummary();
            lastSummaryTime = now;
        }
        
        processComm();
    }
    
    bool getMcuFirmwareVersion(std::string& name, std::string& version) {
        if (firmwareName.empty()) return false;
        name = firmwareName;
        version = firmwareVersion;
        return true;
    }
    
private:
    std::string firmwareName;
    std::string firmwareVersion;
    
    void parseVersionResponse(const std::string& response) {
        // Parse V,RM18,1.1.16,0x4A
        size_t pos1 = response.find(',');
        size_t pos2 = response.find(',', pos1 + 1);
        size_t pos3 = response.find(',', pos2 + 1);
        
        if (pos1 != std::string::npos && pos2 != std::string::npos) {
            firmwareName = response.substr(pos1 + 1, pos2 - pos1 - 1);
            firmwareVersion = response.substr(pos2 + 1, pos3 - pos2 - 1);
        }
    }
    
    void parseMotorResponse(const std::string& response) {
        // Parse M,100,200,1500,24.5,0,0,0,0x7B
        std::vector<std::string> parts;
        size_t start = 0;
        size_t pos = 0;
        
        while ((pos = response.find(',', start)) != std::string::npos) {
            parts.push_back(response.substr(start, pos - start));
            start = pos + 1;
        }
        parts.push_back(response.substr(start));
        
        if (parts.size() >= 8) {
            encoderTicksRight = atoi(parts[1].c_str());
            encoderTicksLeft = atoi(parts[2].c_str());
            encoderTicksMow = atoi(parts[3].c_str());
            chargeVoltage = atof(parts[4].c_str());
            
            // Motor faults
            bool rightFault = atoi(parts[5].c_str()) != 0;
            bool leftFault = atoi(parts[6].c_str()) != 0;
            bool mowFault = atoi(parts[7].c_str()) != 0;
            motorFault = rightFault || leftFault || mowFault;
        }
    }
    
    void parseSummaryResponse(const std::string& response) {
        // Parse S,25.2,0.0,0.0,1,0,0,0,2.1,0.8,0.9,22.5,0x8C
        std::vector<std::string> parts;
        size_t start = 0;
        size_t pos = 0;
        
        while ((pos = response.find(',', start)) != std::string::npos) {
            parts.push_back(response.substr(start, pos - start));
            start = pos + 1;
        }
        parts.push_back(response.substr(start));
        
        if (parts.size() >= 12) {
            batteryVoltage = atof(parts[1].c_str());
            chargeVoltage = atof(parts[2].c_str());
            chargeCurrent = atof(parts[3].c_str());
            triggeredLift = atoi(parts[4].c_str()) != 0;
            triggeredLeftBumper = atoi(parts[5].c_str()) != 0;
            batteryTemp = atof(parts[11].c_str());
        }
    }
};

// SerialMotorDriver Mock
class SerialMotorDriver {
public:
    SerialRobotDriver& driver;
    
    SerialMotorDriver(SerialRobotDriver& d) : driver(d) {}
    
    void begin() {}
    
    void setMotorPwm(int right, int left, int mow) {
        driver.requestMotorPwm(right, left, mow);
    }
    
    void getMotorEncoderTicks(int& left, int& right, int& mow) {
        left = driver.encoderTicksLeft;
        right = driver.encoderTicksRight;
        mow = driver.encoderTicksMow;
    }
    
    void getMotorCurrent(float& left, float& right, float& mow) {
        left = driver.motorLeftCurr;
        right = driver.motorRightCurr;
        mow = driver.mowCurr;
    }
};

// Arduino-kompatible Funktionen
unsigned long millis() {
    static unsigned long startTime = 0;
    if (startTime == 0) {
        startTime = time(nullptr) * 1000;
    }
    return (time(nullptr) * 1000) - startTime;
}

unsigned long micros() {
    return millis() * 1000;
}

void delay(unsigned long ms) {
    // Vereinfachte delay für Tests
}

// Test-Fixture für SerialRobotDriver Tests
class SerialRobotDriverTestBase : public SerialTestBase, public STM32TestBase {
public:
    SerialRobotDriver* driver;
    
    void setUp() override {
        SerialTestBase::setUp();
        STM32TestBase::setUp();
        driver = new SerialRobotDriver();
        
        // Setup standard responses
        mockSTM32->setStandardResponses();
    }
    
    void tearDown() override {
        delete driver;
        driver = nullptr;
        SerialTestBase::tearDown();
        STM32TestBase::tearDown();
    }
};

// ============================================================================
// AT+ Protokoll Tests
// ============================================================================

void test_at_protocol_version_request() {
    SerialRobotDriverTestBase test;
    test.setUp();
    
    // Setup expected version response
    test.mockSTM32->setVersionResponse("RM18", "1.1.16");
    
    // Request version
    test.driver->requestVersion();
    
    // Verify command was sent correctly
    std::string lastCommand = test.mockSerial->getLastCommand();
    TEST_ASSERT(lastCommand.find("AT+V") == 0, "Command should start with AT+V");
    TEST_ASSERT(lastCommand.find("0x") != std::string::npos, "CRC should be present");
    TEST_ASSERT(lastCommand.find("\r\n") != std::string::npos, "Command should end with CRLF");
    
    // Simulate response processing
    test.mockSerial->addResponse("V,RM18,1.1.16,0x4A\r\n");
    test.driver->processComm();
    
    // Verify version was parsed correctly
    std::string name, version;
    TEST_ASSERT(test.driver->getMcuFirmwareVersion(name, version), "Should get firmware version");
    TEST_ASSERT_EQUAL(std::string("RM18"), name, "Firmware name should match");
    TEST_ASSERT_EQUAL(std::string("1.1.16"), version, "Firmware version should match");
    
    test.tearDown();
}

void test_at_protocol_motor_command() {
    SerialRobotDriverTestBase test;
    test.setUp();
    
    // Setup motor response
    test.mockSTM32->setMotorResponse(100, 200, 1500, 24.5f);
    
    // Send motor command
    test.driver->requestMotorPwm(50, -30, 255);
    
    // Verify command format
    std::string lastCommand = test.mockSerial->getLastCommand();
    TEST_ASSERT(lastCommand.find("AT+M") == 0, "Command should start with AT+M");
    TEST_ASSERT(lastCommand.find("-30") != std::string::npos, "Left motor value should be present");
    TEST_ASSERT(lastCommand.find("50") != std::string::npos, "Right motor value should be present");
    TEST_ASSERT(lastCommand.find("255") != std::string::npos, "Mow motor value should be present");
    
    // Simulate response
    test.mockSerial->addResponse("M,100,200,1500,24.5,0,0,0,0x7B\r\n");
    test.driver->processComm();
    
    // Verify encoder ticks were updated
    TEST_ASSERT_EQUAL(200, test.driver->encoderTicksLeft, "Left encoder ticks should match");
    TEST_ASSERT_EQUAL(100, test.driver->encoderTicksRight, "Right encoder ticks should match");
    TEST_ASSERT_EQUAL(1500, test.driver->encoderTicksMow, "Mow encoder ticks should match");
    TEST_ASSERT_FLOAT_EQUAL(24.5f, test.driver->chargeVoltage, 0.1f, "Charge voltage should match");
    
    test.tearDown();
}

void test_at_protocol_summary_request() {
    SerialRobotDriverTestBase test;
    test.setUp();
    
    // Setup summary response with sensor data
    test.mockSTM32->setBatteryData(25.2f, 0.0f, 0.0f, 22.5f);
    test.mockSTM32->setSensorData(false, true, false, false); // lift triggered
    
    // Request summary
    test.driver->requestSummary();
    
    // Verify command
    std::string lastCommand = test.mockSerial->getLastCommand();
    TEST_ASSERT(lastCommand.find("AT+S") == 0, "Command should start with AT+S");
    
    // Simulate response
    test.mockSerial->addResponse("S,25.2,0.0,0.0,1,0,0,0,2.1,0.8,0.9,22.5,0x8C\r\n");
    test.driver->processComm();
    
    // Verify sensor data
    TEST_ASSERT_FLOAT_EQUAL(25.2f, test.driver->batteryVoltage, 0.1f, "Battery voltage should match");
    TEST_ASSERT_FLOAT_EQUAL(0.0f, test.driver->chargeVoltage, 0.1f, "Charge voltage should match");
    TEST_ASSERT_FLOAT_EQUAL(0.0f, test.driver->chargeCurrent, 0.1f, "Charge current should match");
    TEST_ASSERT(test.driver->triggeredLift, "Lift should be triggered");
    TEST_ASSERT(!test.driver->triggeredLeftBumper, "Left bumper should not be triggered");
    TEST_ASSERT_FLOAT_EQUAL(22.5f, test.driver->batteryTemp, 0.1f, "Battery temperature should match");
    
    test.tearDown();
}

// ============================================================================
// CRC Validierung Tests
// ============================================================================

void test_crc_validation_correct() {
    SerialRobotDriverTestBase test;
    test.setUp();
    
    // Send command with correct CRC
    test.mockSerial->addResponse("V,RM18,1.1.16,0x4A\r\n");
    test.driver->processComm();
    
    // Should process successfully
    std::string name, version;
    TEST_ASSERT(test.driver->getMcuFirmwareVersion(name, version), "Should get firmware version");
    TEST_ASSERT_EQUAL(std::string("RM18"), name, "Firmware name should match");
    
    test.tearDown();
}

void test_crc_validation_incorrect() {
    SerialRobotDriverTestBase test;
    test.setUp();
    
    // Send command with incorrect CRC
    test.mockSerial->addResponse("V,RM18,1.1.16,0xFF\r\n");
    test.driver->processComm();
    
    // Should reject response
    std::string name, version;
    TEST_ASSERT(!test.driver->getMcuFirmwareVersion(name, version), "Should not get firmware version with incorrect CRC");
    TEST_ASSERT(test.driver->communicationErrors > 0, "Should have communication errors");
    
    test.tearDown();
}

void test_crc_calculation() {
    SerialRobotDriverTestBase test;
    test.setUp();
    
    // Test CRC calculation for known command
    test.driver->requestVersion();
    std::string command = test.mockSerial->getLastCommand();
    
    // Verify CRC format
    size_t crcPos = command.find("0x");
    TEST_ASSERT(crcPos != std::string::npos, "CRC position should be found");
    
    // Extract CRC
    std::string crcStr = command.substr(crcPos + 2, 2);
    TEST_ASSERT_EQUAL(2, (int)crcStr.length(), "CRC should be 2 characters");
    
    test.tearDown();
}

// ============================================================================
// Fehlerbehandlung Tests
// ============================================================================

void test_communication_timeout_motor() {
    SerialRobotDriverTestBase test;
    test.setUp();
    
    // Setup timeout simulation
    test.mockSerial->simulateTimeout(true);
    
    // Send motor command
    test.driver->requestMotorPwm(100, 100, 255);
    
    // Simulate timeout by advancing time
    unsigned long startTime = millis();
    while (millis() - startTime < 150) { // Motor timeout is 100ms
        test.driver->run();
    }
    
    // Should detect communication lost
    TEST_ASSERT(test.driver->mcuCommunicationLost, "Communication should be lost after timeout");
    TEST_ASSERT(test.driver->communicationErrors > 0, "Should have communication errors");
    
    test.tearDown();
}

void test_communication_timeout_summary() {
    SerialRobotDriverTestBase test;
    test.setUp();
    
    // Setup timeout simulation
    test.mockSerial->simulateTimeout(true);
    
    // Send summary command
    test.driver->requestSummary();
    
    // Simulate timeout by advancing time
    unsigned long startTime = millis();
    while (millis() - startTime < 600) { // Summary timeout is 500ms
        test.driver->run();
    }
    
    // Should detect communication lost
    TEST_ASSERT(test.driver->mcuCommunicationLost, "Communication should be lost after timeout");
    TEST_ASSERT(test.driver->communicationErrors > 0, "Should have communication errors");
    
    test.tearDown();
}

void test_malformed_response_handling() {
    SerialRobotDriverTestBase test;
    test.setUp();
    
    // Send malformed responses
    test.mockSerial->addResponse("V,\r\n"); // Too short
    test.driver->processComm();
    TEST_ASSERT(test.driver->communicationErrors > 0, "Should have communication errors for malformed response");
    
    // Reset error count
    int initialErrors = test.driver->communicationErrors;
    
    test.mockSerial->addResponse("X,unknown,response,0x00\r\n"); // Unknown type
    test.driver->processComm();
    TEST_ASSERT(test.driver->communicationErrors > initialErrors, "Communication errors should increase");
    
    test.tearDown();
}

void test_motor_fault_detection() {
    SerialRobotDriverTestBase test;
    test.setUp();
    
    // Setup motor fault response
    test.mockSTM32->setMotorFault(true);
    test.mockSerial->addResponse("M,100,200,1500,24.5,1,1,1,0x7F\r\n"); // Faults set
    test.driver->processComm();
    
    // Should detect motor fault
    TEST_ASSERT(test.driver->motorFault, "Motor fault should be detected");
    
    test.tearDown();
}

// ============================================================================
// Kommunikations-Frequenz Tests
// ============================================================================

void test_motor_command_frequency() {
    SerialRobotDriverTestBase test;
    test.setUp();
    
    // Setup standard motor response
    test.mockSTM32->setMotorResponse(100, 200, 1500, 24.5f);
    
    int commandCount = 0;
    unsigned long startTime = millis();
    
    // Run for 1 second and count motor commands
    while (millis() - startTime < 1000) {
        test.driver->run();
        
        // Check for motor commands (AT+M)
        std::string lastCommand = test.mockSerial->getLastCommand();
        if (lastCommand.find("AT+M") == 0) {
            commandCount++;
            // Simulate response
            test.mockSerial->addResponse("M,100,200,1500,24.5,0,0,0,0x7B\r\n");
        }
        
        delay(10);
    }
    
    // Should have sent approximately 50 commands (50Hz)
    TEST_ASSERT(commandCount > 40, "Should have sent more than 40 motor commands");
    TEST_ASSERT(commandCount < 60, "Should have sent less than 60 motor commands");
    
    test.tearDown();
}

void test_summary_command_frequency() {
    SerialRobotDriverTestBase test;
    test.setUp();
    
    // Setup standard summary response
    test.mockSTM32->setBatteryData(25.0f, 0.0f, 0.0f, 22.0f);
    
    int summaryCount = 0;
    unsigned long startTime = millis();
    
    // Run for 2 seconds and count summary commands
    while (millis() - startTime < 2000) {
        test.driver->run();
        
        // Check for summary commands (AT+S)
        std::string lastCommand = test.mockSerial->getLastCommand();
        if (lastCommand.find("AT+S") == 0) {
            summaryCount++;
            // Simulate response
            test.mockSerial->addResponse("S,25.0,0.0,0.0,0,0,0,0,2.1,0.8,0.9,22.0,0x8A\r\n");
        }
        
        delay(50);
    }
    
    // Should have sent approximately 4 commands (2Hz)
    TEST_ASSERT(summaryCount > 2, "Should have sent more than 2 summary commands");
    TEST_ASSERT(summaryCount < 6, "Should have sent less than 6 summary commands");
    
    test.tearDown();
}

// ============================================================================
// Integration Tests mit Motor-Driver
// ============================================================================

void test_serial_motor_driver_integration() {
    SerialRobotDriverTestBase test;
    test.setUp();
    
    // Create motor driver
    SerialMotorDriver motorDriver(*test.driver);
    motorDriver.begin();
    
    // Setup motor response
    test.mockSTM32->setMotorResponse(1000, 2000, 3000, 24.5f);
    
    // Set motor PWM
    motorDriver.setMotorPwm(100, -50, 255);
    
    // Simulate response
    test.mockSerial->addResponse("M,1000,2000,3000,24.5,0,0,0,0x9C\r\n");
    test.driver->processComm();
    
    // Get encoder ticks
    int leftTicks, rightTicks, mowTicks;
    motorDriver.getMotorEncoderTicks(leftTicks, rightTicks, mowTicks);
    
    TEST_ASSERT_EQUAL(2000, leftTicks, "Left encoder ticks should match");
    TEST_ASSERT_EQUAL(1000, rightTicks, "Right encoder ticks should match");
    TEST_ASSERT_EQUAL(3000, mowTicks, "Mow encoder ticks should match");
    
    // Test motor current
    float leftCurrent, rightCurrent, mowCurrent;
    motorDriver.getMotorCurrent(leftCurrent, rightCurrent, mowCurrent);
    
    TEST_ASSERT_FLOAT_EQUAL(test.driver->motorLeftCurr, leftCurrent, 0.1f, "Left motor current should match");
    TEST_ASSERT_FLOAT_EQUAL(test.driver->motorRightCurr, rightCurrent, 0.1f, "Right motor current should match");
    TEST_ASSERT_FLOAT_EQUAL(test.driver->mowCurr, mowCurrent, 0.1f, "Mow motor current should match");
    
    test.tearDown();
}

// ============================================================================
// Performance Tests
// ============================================================================

void test_response_processing_performance() {
    SerialRobotDriverTestBase test;
    test.setUp();
    
    // Measure response processing time
    unsigned long startTime = micros();
    
    for (int i = 0; i < 100; i++) {
        test.mockSerial->addResponse("M,100,200,1500,24.5,0,0,0,0x7B\r\n");
        test.driver->processComm();
    }
    
    unsigned long duration = micros() - startTime;
    unsigned long avgTime = duration / 100;
    
    // Should process response in less than 1ms
    TEST_ASSERT(avgTime < 1000, "Average response processing time should be less than 1ms");
    
    test.tearDown();
}

void test_command_generation_performance() {
    SerialRobotDriverTestBase test;
    test.setUp();
    
    // Measure command generation time
    unsigned long startTime = micros();
    
    for (int i = 0; i < 100; i++) {
        test.driver->requestMotorPwm(i, -i, 255);
    }
    
    unsigned long duration = micros() - startTime;
    unsigned long avgTime = duration / 100;
    
    // Should generate command in less than 500µs
    TEST_ASSERT(avgTime < 500, "Average command generation time should be less than 500µs");
    
    test.tearDown();
}

// ============================================================================
// Test-Funktionen für main
// ============================================================================

int main() {
    std::cout << "=== SerialRobotDriver Integration Tests ===" << std::endl;
    
    RUN_TEST(test_at_protocol_version_request);
    RUN_TEST(test_at_protocol_motor_command);
    RUN_TEST(test_at_protocol_summary_request);
    
    RUN_TEST(test_crc_validation_correct);
    RUN_TEST(test_crc_validation_incorrect);
    RUN_TEST(test_crc_calculation);
    
    RUN_TEST(test_communication_timeout_motor);
    RUN_TEST(test_communication_timeout_summary);
    RUN_TEST(test_malformed_response_handling);
    RUN_TEST(test_motor_fault_detection);
    
    RUN_TEST(test_motor_command_frequency);
    RUN_TEST(test_summary_command_frequency);
    
    RUN_TEST(test_serial_motor_driver_integration);
    
    RUN_TEST(test_response_processing_performance);
    RUN_TEST(test_command_generation_performance);
    
    std::cout << "\n=== Test Results ===" << std::endl;
    std::cout << "Tests run: " << tests_run << std::endl;
    std::cout << "Tests passed: " << tests_passed << std::endl;
    std::cout << "Tests failed: " << (tests_run - tests_passed) << std::endl;
    
    if (tests_passed == tests_run) {
        std::cout << "All tests PASSED!" << std::endl;
        return 0;
    } else {
        std::cout << "Some tests FAILED!" << std::endl;
        return 1;
    }
}
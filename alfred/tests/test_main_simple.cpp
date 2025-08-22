#include <iostream>
#include <string>
#include <vector>
#include <cassert>
#include <chrono>

// Simple test framework
class SimpleTest {
public:
    static int totalTests;
    static int passedTests;
    static int failedTests;
    static std::vector<std::string> failedTestNames;
    
    static void runTest(const std::string& testName, bool (*testFunc)()) {
        totalTests++;
        std::cout << "Running test: " << testName << "... ";
        
        try {
            bool result = testFunc();
            if (result) {
                passedTests++;
                std::cout << "PASSED" << std::endl;
            } else {
                failedTests++;
                failedTestNames.push_back(testName);
                std::cout << "FAILED" << std::endl;
            }
        } catch (const std::exception& e) {
            failedTests++;
            failedTestNames.push_back(testName);
            std::cout << "FAILED (Exception: " << e.what() << ")" << std::endl;
        } catch (...) {
            failedTests++;
            failedTestNames.push_back(testName);
            std::cout << "FAILED (Unknown exception)" << std::endl;
        }
    }
    
    static void printSummary() {
        std::cout << "\n=== Test Summary ===" << std::endl;
        std::cout << "Total tests: " << totalTests << std::endl;
        std::cout << "Passed: " << passedTests << std::endl;
        std::cout << "Failed: " << failedTests << std::endl;
        
        if (failedTests > 0) {
            std::cout << "\nFailed tests:" << std::endl;
            for (const auto& testName : failedTestNames) {
                std::cout << "  - " << testName << std::endl;
            }
        }
        
        std::cout << "\nSuccess rate: " << 
            (totalTests > 0 ? (passedTests * 100 / totalTests) : 0) << "%" << std::endl;
    }
    
    static int getExitCode() {
        return (failedTests == 0) ? 0 : 1;
    }
};

// Static member definitions
int SimpleTest::totalTests = 0;
int SimpleTest::passedTests = 0;
int SimpleTest::failedTests = 0;
std::vector<std::string> SimpleTest::failedTestNames;

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

// Forward declarations of test functions
bool test_config_structures_default_values();
bool test_config_structures_validation();
bool test_config_structures_edge_cases();
bool test_config_validator_motor_config();
bool test_config_validator_at_protocol();
bool test_config_validator_consistency();
bool test_config_performance();

int main() {
    std::cout << "=== Alfred Component Tests ===" << std::endl;
    std::cout << "Testing configuration structures and validation..." << std::endl << std::endl;
    
    auto startTime = std::chrono::high_resolution_clock::now();
    
    // Run all tests
    SimpleTest::runTest("ConfigStructures_DefaultValues", test_config_structures_default_values);
    SimpleTest::runTest("ConfigStructures_Validation", test_config_structures_validation);
    SimpleTest::runTest("ConfigStructures_EdgeCases", test_config_structures_edge_cases);
    SimpleTest::runTest("ConfigValidator_MotorConfig", test_config_validator_motor_config);
    SimpleTest::runTest("ConfigValidator_ATProtocol", test_config_validator_at_protocol);
    SimpleTest::runTest("ConfigValidator_Consistency", test_config_validator_consistency);
    SimpleTest::runTest("Config_Performance", test_config_performance);
    
    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    
    std::cout << "\nTest execution time: " << duration.count() << " ms" << std::endl;
    
    SimpleTest::printSummary();
    
    return SimpleTest::getExitCode();
}
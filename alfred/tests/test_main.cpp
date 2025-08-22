#include <gtest/gtest.h>
#include <gmock/gmock.h>

// Include necessary headers for Alfred components
#include "Arduino.h"
#include "Console.h"

// Mock initialization for Arduino environment
void setup() {
    // Initialize console for testing
    CONSOLE.begin(115200);
}

void loop() {
    // Not used in tests
}

// Global test environment setup
class AlfredTestEnvironment : public ::testing::Environment {
public:
    void SetUp() override {
        // Initialize Arduino-like environment for tests
        setup();
        
        // Set up any global test fixtures
        std::cout << "Alfred Test Environment initialized" << std::endl;
    }
    
    void TearDown() override {
        // Clean up global test fixtures
        std::cout << "Alfred Test Environment cleaned up" << std::endl;
    }
};

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);
    
    // Add global test environment
    ::testing::AddGlobalTestEnvironment(new AlfredTestEnvironment);
    
    return RUN_ALL_TESTS();
}
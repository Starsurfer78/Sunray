// Mock-Objekte für serielle Kommunikation
// Simuliert Arduino Serial-Interface für Tests ohne GMock

#ifndef MOCK_SERIAL_H
#define MOCK_SERIAL_H

#include <vector>
#include <string>
#include <queue>
#include <cstring>
#include <cstdio>

// Mock Serial-Klasse für Tests
class MockSerial {
public:
    MockSerial() : commandCount(0), timeoutSimulation(false), readPosition(0) {}
    
    // Arduino Serial Interface
    void begin(unsigned long baud) {
        baudRate = baud;
        isInitialized = true;
    }
    
    void end() {
        isInitialized = false;
    }
    
    int available() {
        if (timeoutSimulation) return 0;
        if (responseQueue.empty()) return 0;
        
        const std::string& response = responseQueue.front();
        return response.length() - readPosition;
    }
    
    int read() {
        if (responseQueue.empty() || timeoutSimulation) return -1;
        
        std::string& response = responseQueue.front();
        if (readPosition >= response.length()) {
            responseQueue.pop();
            readPosition = 0;
            return -1;
        }
        
        return response[readPosition++];
    }
    
    size_t write(uint8_t data) {
        currentCommand += (char)data;
        return 1;
    }
    
    size_t write(const char* str) {
        currentCommand += str;
        return strlen(str);
    }
    
    size_t print(const char* str) {
        return write(str);
    }
    
    size_t print(int value) {
        char buffer[16];
        sprintf(buffer, "%d", value);
        return write(buffer);
    }
    
    size_t print(float value, int decimals = 2) {
        char buffer[32];
        sprintf(buffer, "%.*f", decimals, value);
        return write(buffer);
    }
    
    size_t println(const char* str) {
        size_t result = write(str);
        result += write("\r\n");
        
        // Command complete - store it
        if (!currentCommand.empty()) {
            lastCommand = currentCommand;
            commandHistory.push_back(currentCommand);
            commandCount++;
            currentCommand.clear();
        }
        
        return result;
    }
    
    void flush() {
        // Simulate flush delay
    }
    
    // Test-spezifische Methoden
    void addResponse(const std::string& response) {
        responseQueue.push(response);
        readPosition = 0;
    }
    
    std::string getLastCommand() const {
        return lastCommand;
    }
    
    const std::vector<std::string>& getCommandHistory() const {
        return commandHistory;
    }
    
    int getCommandCount() const {
        return commandCount;
    }
    
    void clearHistory() {
        commandHistory.clear();
        lastCommand.clear();
        commandCount = 0;
        currentCommand.clear();
        while (!responseQueue.empty()) {
            responseQueue.pop();
        }
        readPosition = 0;
    }
    
    void simulateTimeout(bool enable) {
        timeoutSimulation = enable;
    }
    
    bool isReady() const {
        return isInitialized;
    }
    
private:
    std::string currentCommand;
    std::string lastCommand;
    std::vector<std::string> commandHistory;
    std::queue<std::string> responseQueue;
    size_t readPosition;
    int commandCount;
    unsigned long baudRate = 0;
    bool isInitialized = false;
    bool timeoutSimulation;
};

// Global mock serial instance
extern MockSerial* mockSerial;

// Test-Base-Klasse für serielle Tests
class SerialTestBase {
public:
    virtual void setUp() {
        mockSerial = new MockSerial();
        mockSerial->begin(115200);
    }
    
    virtual void tearDown() {
        delete mockSerial;
        mockSerial = nullptr;
    }
};

// Arduino Serial Makros für Tests
#define Serial (*mockSerial)

#endif // MOCK_SERIAL_H
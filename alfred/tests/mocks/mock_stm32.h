// Mock-Objekte für STM32-Simulation
// Simuliert STM32-Antworten für AT+-Protokoll Tests ohne GMock

#ifndef MOCK_STM32_H
#define MOCK_STM32_H

#include <string>
#include <cstdio>
#include <cstring>
#include <cstdlib>

// Mock STM32-Klasse für Tests
class MockSTM32 {
public:
    MockSTM32() {
        resetToDefaults();
    }
    
    // Standard-Antworten setzen
    void setStandardResponses() {
        setVersionResponse("RM18", "1.1.16");
        setMotorResponse(0, 0, 0, 24.0f);
        setBatteryData(25.0f, 0.0f, 0.0f, 22.0f);
        setSensorData(false, false, false, false);
        setMotorFault(false);
    }
    
    // Version Response
    void setVersionResponse(const std::string& name, const std::string& version) {
        firmwareName = name;
        firmwareVersion = version;
    }
    
    std::string getVersionResponse() {
        char buffer[128];
        sprintf(buffer, "V,%s,%s,0x%02X\r\n", 
                firmwareName.c_str(), 
                firmwareVersion.c_str(),
                calculateCRC("V," + firmwareName + "," + firmwareVersion));
        return std::string(buffer);
    }
    
    // Motor Response
    void setMotorResponse(int rightTicks, int leftTicks, int mowTicks, float chargeVolt) {
        encoderTicksRight = rightTicks;
        encoderTicksLeft = leftTicks;
        encoderTicksMow = mowTicks;
        chargeVoltage = chargeVolt;
    }
    
    std::string getMotorResponse() {
        char buffer[256];
        sprintf(buffer, "M,%d,%d,%d,%.1f,%d,%d,%d,0x%02X\r\n",
                encoderTicksRight, encoderTicksLeft, encoderTicksMow,
                chargeVoltage,
                motorRightFault ? 1 : 0,
                motorLeftFault ? 1 : 0,
                motorMowFault ? 1 : 0,
                calculateCRC("M"));
        return std::string(buffer);
    }
    
    // Battery und Sensor Data
    void setBatteryData(float battVolt, float chargeVolt, float chargeCurr, float battTemp) {
        batteryVoltage = battVolt;
        chargeVoltage = chargeVolt;
        chargeCurrent = chargeCurr;
        batteryTemp = battTemp;
    }
    
    void setSensorData(bool lift, bool leftBumper, bool rightBumper, bool stop) {
        liftSensor = lift;
        leftBumperSensor = leftBumper;
        rightBumperSensor = rightBumper;
        stopSensor = stop;
    }
    
    std::string getSummaryResponse() {
        char buffer[512];
        sprintf(buffer, "S,%.1f,%.1f,%.1f,%d,%d,%d,%d,%.1f,%.1f,%.1f,%.1f,0x%02X\r\n",
                batteryVoltage, chargeVoltage, chargeCurrent,
                liftSensor ? 1 : 0,
                leftBumperSensor ? 1 : 0,
                rightBumperSensor ? 1 : 0,
                stopSensor ? 1 : 0,
                imuAccelX, imuAccelY, imuAccelZ,
                batteryTemp,
                calculateCRC("S"));
        return std::string(buffer);
    }
    
    // Motor Fault Simulation
    void setMotorFault(bool fault) {
        motorRightFault = fault;
        motorLeftFault = fault;
        motorMowFault = fault;
    }
    
    void setMotorFault(bool rightFault, bool leftFault, bool mowFault) {
        motorRightFault = rightFault;
        motorLeftFault = leftFault;
        motorMowFault = mowFault;
    }
    
    // IMU Data
    void setIMUData(float accelX, float accelY, float accelZ) {
        imuAccelX = accelX;
        imuAccelY = accelY;
        imuAccelZ = accelZ;
    }
    
    // Error Simulation
    void simulateCommError(bool enable) {
        commErrorSimulation = enable;
    }
    
    void simulateCRCError(bool enable) {
        crcErrorSimulation = enable;
    }
    
    // Response Generation basierend auf Command
    std::string generateResponse(const std::string& command) {
        if (commErrorSimulation) {
            return ""; // Keine Antwort bei Kommunikationsfehler
        }
        
        if (command.find("AT+V") == 0) {
            return getVersionResponse();
        }
        else if (command.find("AT+M") == 0) {
            // Parse motor command und update encoder ticks
            parseMotorCommand(command);
            return getMotorResponse();
        }
        else if (command.find("AT+S") == 0) {
            return getSummaryResponse();
        }
        else if (command.find("AT+Y") == 0) {
            return getYawResponse();
        }
        
        return "ERROR,Unknown Command,0xFF\r\n";
    }
    
    // Reset zu Standardwerten
    void resetToDefaults() {
        firmwareName = "RM18";
        firmwareVersion = "1.1.16";
        encoderTicksRight = 0;
        encoderTicksLeft = 0;
        encoderTicksMow = 0;
        chargeVoltage = 24.0f;
        batteryVoltage = 25.0f;
        chargeCurrent = 0.0f;
        batteryTemp = 22.0f;
        liftSensor = false;
        leftBumperSensor = false;
        rightBumperSensor = false;
        stopSensor = false;
        motorRightFault = false;
        motorLeftFault = false;
        motorMowFault = false;
        imuAccelX = 2.1f;
        imuAccelY = 0.8f;
        imuAccelZ = 0.9f;
        commErrorSimulation = false;
        crcErrorSimulation = false;
        lastMotorRight = 0;
        lastMotorLeft = 0;
        lastMotorMow = 0;
    }
    
    // Getter für Test-Verifikation
    int getLastMotorRight() const { return lastMotorRight; }
    int getLastMotorLeft() const { return lastMotorLeft; }
    int getLastMotorMow() const { return lastMotorMow; }
    
private:
    // Firmware Info
    std::string firmwareName;
    std::string firmwareVersion;
    
    // Motor Data
    int encoderTicksRight;
    int encoderTicksLeft;
    int encoderTicksMow;
    float chargeVoltage;
    bool motorRightFault;
    bool motorLeftFault;
    bool motorMowFault;
    int lastMotorRight;
    int lastMotorLeft;
    int lastMotorMow;
    
    // Battery Data
    float batteryVoltage;
    float chargeCurrent;
    float batteryTemp;
    
    // Sensor Data
    bool liftSensor;
    bool leftBumperSensor;
    bool rightBumperSensor;
    bool stopSensor;
    
    // IMU Data
    float imuAccelX;
    float imuAccelY;
    float imuAccelZ;
    
    // Error Simulation
    bool commErrorSimulation;
    bool crcErrorSimulation;
    
    // CRC Berechnung (vereinfacht)
    uint8_t calculateCRC(const std::string& data) {
        if (crcErrorSimulation) {
            return 0xFF; // Falscher CRC
        }
        
        uint8_t crc = 0;
        for (char c : data) {
            crc ^= c;
        }
        return crc;
    }
    
    // Motor Command Parsing
    void parseMotorCommand(const std::string& command) {
        // Parse AT+M,right,left,mow,0x??
        size_t pos = command.find("AT+M,");
        if (pos == std::string::npos) return;
        
        std::string params = command.substr(pos + 5);
        
        // Simple parsing (für Tests ausreichend)
        char* paramsCopy = new char[params.length() + 1];
        strcpy(paramsCopy, params.c_str());
        
        char* token = strtok(paramsCopy, ",");
        if (token) lastMotorRight = atoi(token);
        
        token = strtok(nullptr, ",");
        if (token) lastMotorLeft = atoi(token);
        
        token = strtok(nullptr, ",");
        if (token) lastMotorMow = atoi(token);
        
        delete[] paramsCopy;
        
        // Simulate encoder tick changes
        encoderTicksRight += abs(lastMotorRight) / 10;
        encoderTicksLeft += abs(lastMotorLeft) / 10;
        encoderTicksMow += abs(lastMotorMow) / 20;
    }
    
    // Yaw Response (vereinfacht)
    std::string getYawResponse() {
        char buffer[128];
        sprintf(buffer, "Y,0.0,0x%02X\r\n", calculateCRC("Y,0.0"));
        return std::string(buffer);
    }
};

// Global mock STM32 instance
extern MockSTM32* mockSTM32;

// Test-Base-Klasse für STM32 Tests
class STM32TestBase {
public:
    virtual void setUp() {
        mockSTM32 = new MockSTM32();
        mockSTM32->setStandardResponses();
    }
    
    virtual void tearDown() {
        delete mockSTM32;
        mockSTM32 = nullptr;
    }
};

#endif // MOCK_STM32_H
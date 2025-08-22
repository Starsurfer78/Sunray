// Ardumower Sunray - Alfred Platform
// AT+ Protocol Handler
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH

#ifndef AT_PROTOCOL_HANDLER_H
#define AT_PROTOCOL_HANDLER_H

#include "../config/config_manager.h"
#include "../../config.h"
#include <vector>
#include <functional>

class ATProtocolHandler {
public:
    enum class CommandType {
        MOTOR_CONTROL,      // M,leftPWM,rightPWM,mowPWM
        SENSOR_REQUEST,     // S (request sensor data)
        VERSION_REQUEST,    // V (request version)
        SUMMARY_REQUEST,    // T (request summary)
        RESET_COMMAND,      // R (reset)
        CUSTOM_COMMAND      // User-defined commands
    };
    
    enum class ResponseType {
        MOTOR_RESPONSE,     // M,leftTicks,rightTicks,leftPWM,rightPWM,mowPWM,CRC
        SENSOR_RESPONSE,    // S,battV,chargeV,leftI,rightI,mowI,bumperL,bumperR,lift,rain,CRC
        VERSION_RESPONSE,   // V,version,CRC
        SUMMARY_RESPONSE,   // T,summary_data,CRC
        ERROR_RESPONSE,     // E,error_code,description,CRC
        UNKNOWN_RESPONSE
    };
    
    struct ATCommand {
        CommandType type;
        String command;
        std::vector<String> parameters;
        unsigned long timestamp;
        int retryCount;
        bool requiresResponse;
        unsigned long timeout;
        
        ATCommand(CommandType t, const String& cmd, bool needsResponse = true, unsigned long timeoutMs = 2000) 
            : type(t), command(cmd), timestamp(millis()), retryCount(0), requiresResponse(needsResponse), timeout(timeoutMs) {}
    };
    
    struct ATResponse {
        ResponseType type;
        String rawResponse;
        std::vector<String> parameters;
        unsigned long timestamp;
        bool isValid;
        bool crcValid;
        String errorMessage;
        
        ATResponse() : type(ResponseType::UNKNOWN_RESPONSE), timestamp(millis()), isValid(false), crcValid(false) {}
    };
    
    struct ProtocolStats {
        unsigned long commandsSent;
        unsigned long responsesReceived;
        unsigned long timeouts;
        unsigned long crcErrors;
        unsigned long retries;
        unsigned long averageResponseTime;
        float successRate;
        
        ProtocolStats() : commandsSent(0), responsesReceived(0), timeouts(0), crcErrors(0), retries(0), averageResponseTime(0), successRate(0.0) {}
        
        void updateSuccessRate() {
            if (commandsSent > 0) {
                successRate = (float)(responsesReceived) / commandsSent * 100.0;
            }
        }
    };
    
    // Callback types
    using ResponseCallback = std::function<void(const ATResponse&)>;
    using ErrorCallback = std::function<void(const String&, int)>;
    using TimeoutCallback = std::function<void(const ATCommand&)>;
    
private:
    Stream* serialPort;
    String inputBuffer;
    std::vector<ATCommand> pendingCommands;
    std::vector<ATResponse> responseHistory;
    ProtocolStats stats;
    
    // Configuration
    ATProtocolConfig config;
    
    // Callbacks
    ResponseCallback responseCallback;
    ErrorCallback errorCallback;
    TimeoutCallback timeoutCallback;
    
    // State management
    bool initialized;
    unsigned long lastActivity;
    unsigned long lastStatsUpdate;
    
    // CRC calculation
    static const uint16_t CRC_POLYNOMIAL = 0x1021;
    static uint16_t crcTable[256];
    static bool crcTableInitialized;
    
public:
    ATProtocolHandler(Stream* port = nullptr);
    ~ATProtocolHandler();
    
    // Initialization
    bool initialize(Stream* port);
    bool configure(const ATProtocolConfig& cfg);
    void shutdown();
    
    // Command sending
    bool sendCommand(const ATCommand& command);
    bool sendMotorCommand(int leftPWM, int rightPWM, int mowPWM);
    bool sendSensorRequest();
    bool sendVersionRequest();
    bool sendSummaryRequest();
    bool sendResetCommand();
    bool sendCustomCommand(const String& command, const std::vector<String>& params = {});
    
    // Response handling
    void processIncomingData();
    ATResponse getLastResponse(ResponseType type = ResponseType::UNKNOWN_RESPONSE);
    std::vector<ATResponse> getResponseHistory(ResponseType type = ResponseType::UNKNOWN_RESPONSE, int maxCount = 10);
    
    // Callback registration
    void setResponseCallback(ResponseCallback callback) { responseCallback = callback; }
    void setErrorCallback(ErrorCallback callback) { errorCallback = callback; }
    void setTimeoutCallback(TimeoutCallback callback) { timeoutCallback = callback; }
    
    // Status and diagnostics
    ProtocolStats getStats() const { return stats; }
    bool isHealthy() const;
    String getHealthReport() const;
    void resetStats();
    
    // Configuration access
    ATProtocolConfig getConfig() const { return config; }
    bool updateConfig(const ATProtocolConfig& newConfig);
    
    // Utility methods
    static String commandTypeToString(CommandType type);
    static String responseTypeToString(ResponseType type);
    static CommandType stringToCommandType(const String& str);
    static ResponseType stringToResponseType(const String& str);
    
private:
    // Internal processing methods
    void processResponse(const String& response);
    ATResponse parseResponse(const String& response);
    ResponseType detectResponseType(const String& response);
    std::vector<String> parseParameters(const String& response);
    
    // CRC handling
    static void initializeCRCTable();
    static uint16_t calculateCRC(const String& data);
    static bool verifyCRC(const String& response);
    String addCRC(const String& command);
    
    // Command management
    void addPendingCommand(const ATCommand& command);
    void removePendingCommand(const String& commandStr);
    void checkTimeouts();
    void retryCommand(const ATCommand& command);
    
    // Buffer management
    void processInputBuffer();
    void clearInputBuffer();
    bool isCompleteResponse(const String& buffer);
    
    // Statistics
    void updateStats(const ATCommand& command, const ATResponse& response);
    void updateResponseTime(unsigned long commandTime, unsigned long responseTime);
    
    // Error handling
    void handleError(const String& error, int code = 0);
    void handleTimeout(const ATCommand& command);
    void handleCRCError(const String& response);
};

// Implementation of key methods
inline ATProtocolHandler::ATProtocolHandler(Stream* port) 
    : serialPort(port), initialized(false), lastActivity(0), lastStatsUpdate(0) {
    
    if (!crcTableInitialized) {
        initializeCRCTable();
    }
    
    // Load configuration from ConfigManager
    if (ConfigManager::isInitialized()) {
        config = ConfigManager::getATProtocolConfig();
    } else {
        // Use default configuration
        config.baudRate = 115200;
        config.motorTimeout = 8000;
        config.summaryTimeout = 4000;
        config.responseTimeout = 2000;
        config.maxRetries = 3;
        config.bufferSize = 512;
        config.enableCRC = true;
    }
    
    inputBuffer.reserve(config.bufferSize);
}

inline bool ATProtocolHandler::initialize(Stream* port) {
    if (!port) {
        handleError("Invalid serial port", -1);
        return false;
    }
    
    serialPort = port;
    
    // Clear buffers and reset state
    inputBuffer = "";
    pendingCommands.clear();
    responseHistory.clear();
    
    // Reset statistics
    stats = ProtocolStats();
    
    lastActivity = millis();
    lastStatsUpdate = millis();
    initialized = true;
    
    CONSOLE.println("AT Protocol Handler initialized");
    return true;
}

inline bool ATProtocolHandler::sendMotorCommand(int leftPWM, int rightPWM, int mowPWM) {
    if (!initialized || !serialPort) {
        handleError("Protocol handler not initialized", -2);
        return false;
    }
    
    String command = "M," + String(leftPWM) + "," + String(rightPWM) + "," + String(mowPWM);
    
    if (config.enableCRC) {
        command = addCRC(command);
    }
    
    ATCommand cmd(CommandType::MOTOR_CONTROL, command, true, config.motorTimeout);
    cmd.parameters = {String(leftPWM), String(rightPWM), String(mowPWM)};
    
    return sendCommand(cmd);
}

inline bool ATProtocolHandler::sendSensorRequest() {
    if (!initialized || !serialPort) {
        handleError("Protocol handler not initialized", -2);
        return false;
    }
    
    String command = "S";
    
    if (config.enableCRC) {
        command = addCRC(command);
    }
    
    ATCommand cmd(CommandType::SENSOR_REQUEST, command, true, config.responseTimeout);
    
    return sendCommand(cmd);
}

inline bool ATProtocolHandler::sendCommand(const ATCommand& command) {
    if (!initialized || !serialPort) {
        handleError("Protocol handler not initialized", -2);
        return false;
    }
    
    try {
        // Send command to serial port
        serialPort->println(command.command);
        serialPort->flush();
        
        // Add to pending commands if response is expected
        if (command.requiresResponse) {
            addPendingCommand(command);
        }
        
        // Update statistics
        stats.commandsSent++;
        lastActivity = millis();
        
        return true;
        
    } catch (...) {
        handleError("Failed to send command: " + command.command, -3);
        return false;
    }
}

inline void ATProtocolHandler::processIncomingData() {
    if (!initialized || !serialPort) {
        return;
    }
    
    // Read available data
    while (serialPort->available()) {
        char c = serialPort->read();
        
        if (c == '\n' || c == '\r') {
            if (inputBuffer.length() > 0) {
                processResponse(inputBuffer);
                inputBuffer = "";
            }
        } else if (inputBuffer.length() < config.bufferSize - 1) {
            inputBuffer += c;
        } else {
            // Buffer overflow - clear and start over
            inputBuffer = "";
            handleError("Input buffer overflow", -4);
        }
        
        lastActivity = millis();
    }
    
    // Check for timeouts
    checkTimeouts();
    
    // Update statistics periodically
    unsigned long now = millis();
    if (now - lastStatsUpdate > 10000) { // Every 10 seconds
        stats.updateSuccessRate();
        lastStatsUpdate = now;
    }
}

inline void ATProtocolHandler::processResponse(const String& response) {
    if (response.isEmpty()) {
        return;
    }
    
    ATResponse parsedResponse = parseResponse(response);
    
    // Verify CRC if enabled
    if (config.enableCRC) {
        parsedResponse.crcValid = verifyCRC(response);
        if (!parsedResponse.crcValid) {
            handleCRCError(response);
            return;
        }
    } else {
        parsedResponse.crcValid = true; // Assume valid if CRC not used
    }
    
    // Add to response history
    responseHistory.push_back(parsedResponse);
    
    // Keep only last 100 responses
    if (responseHistory.size() > 100) {
        responseHistory.erase(responseHistory.begin());
    }
    
    // Remove corresponding pending command
    if (!pendingCommands.empty()) {
        // Find and remove the oldest pending command of matching type
        for (auto it = pendingCommands.begin(); it != pendingCommands.end(); ++it) {
            bool matches = false;
            
            switch (parsedResponse.type) {
                case ResponseType::MOTOR_RESPONSE:
                    matches = (it->type == CommandType::MOTOR_CONTROL);
                    break;
                case ResponseType::SENSOR_RESPONSE:
                    matches = (it->type == CommandType::SENSOR_REQUEST);
                    break;
                case ResponseType::VERSION_RESPONSE:
                    matches = (it->type == CommandType::VERSION_REQUEST);
                    break;
                case ResponseType::SUMMARY_RESPONSE:
                    matches = (it->type == CommandType::SUMMARY_REQUEST);
                    break;
                default:
                    break;
            }
            
            if (matches) {
                updateStats(*it, parsedResponse);
                pendingCommands.erase(it);
                break;
            }
        }
    }
    
    // Update statistics
    stats.responsesReceived++;
    
    // Call response callback if registered
    if (responseCallback) {
        responseCallback(parsedResponse);
    }
}

inline ATProtocolHandler::ATResponse ATProtocolHandler::parseResponse(const String& response) {
    ATResponse result;
    result.rawResponse = response;
    result.timestamp = millis();
    result.type = detectResponseType(response);
    result.parameters = parseParameters(response);
    result.isValid = (result.type != ResponseType::UNKNOWN_RESPONSE && !result.parameters.empty());
    
    return result;
}

inline ATProtocolHandler::ResponseType ATProtocolHandler::detectResponseType(const String& response) {
    if (response.isEmpty()) {
        return ResponseType::UNKNOWN_RESPONSE;
    }
    
    char firstChar = response.charAt(0);
    
    switch (firstChar) {
        case 'M': return ResponseType::MOTOR_RESPONSE;
        case 'S': return ResponseType::SENSOR_RESPONSE;
        case 'V': return ResponseType::VERSION_RESPONSE;
        case 'T': return ResponseType::SUMMARY_RESPONSE;
        case 'E': return ResponseType::ERROR_RESPONSE;
        default: return ResponseType::UNKNOWN_RESPONSE;
    }
}

inline std::vector<String> ATProtocolHandler::parseParameters(const String& response) {
    std::vector<String> params;
    
    int start = 0;
    int commaIndex = response.indexOf(',');
    
    // Skip the command character (first parameter)
    if (commaIndex > 0) {
        start = commaIndex + 1;
    } else {
        return params; // No parameters
    }
    
    while (start < response.length()) {
        commaIndex = response.indexOf(',', start);
        
        if (commaIndex == -1) {
            // Last parameter
            String param = response.substring(start);
            param.trim();
            if (!param.isEmpty()) {
                params.push_back(param);
            }
            break;
        } else {
            String param = response.substring(start, commaIndex);
            param.trim();
            if (!param.isEmpty()) {
                params.push_back(param);
            }
            start = commaIndex + 1;
        }
    }
    
    return params;
}

// CRC implementation
inline void ATProtocolHandler::initializeCRCTable() {
    for (int i = 0; i < 256; i++) {
        uint16_t crc = i << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ CRC_POLYNOMIAL;
            } else {
                crc <<= 1;
            }
        }
        crcTable[i] = crc;
    }
    crcTableInitialized = true;
}

inline uint16_t ATProtocolHandler::calculateCRC(const String& data) {
    uint16_t crc = 0;
    
    for (int i = 0; i < data.length(); i++) {
        uint8_t byte = data.charAt(i);
        crc = (crc << 8) ^ crcTable[((crc >> 8) ^ byte) & 0xFF];
    }
    
    return crc;
}

inline String ATProtocolHandler::addCRC(const String& command) {
    uint16_t crc = calculateCRC(command);
    return command + "," + String(crc, HEX);
}

inline bool ATProtocolHandler::verifyCRC(const String& response) {
    int lastComma = response.lastIndexOf(',');
    if (lastComma == -1) {
        return false; // No CRC found
    }
    
    String dataPart = response.substring(0, lastComma);
    String crcPart = response.substring(lastComma + 1);
    
    uint16_t expectedCRC = calculateCRC(dataPart);
    uint16_t receivedCRC = strtol(crcPart.c_str(), nullptr, 16);
    
    return (expectedCRC == receivedCRC);
}

// Static member initialization
uint16_t ATProtocolHandler::crcTable[256];
bool ATProtocolHandler::crcTableInitialized = false;

#endif // AT_PROTOCOL_HANDLER_H
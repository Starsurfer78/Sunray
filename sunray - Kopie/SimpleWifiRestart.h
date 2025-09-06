// SimpleWifiRestart.h
// Simple WiFi restart functionality for Sunray robot
// Provides automatic WiFi reconnection with configurable parameters

#ifndef SIMPLE_WIFI_RESTART_H
#define SIMPLE_WIFI_RESTART_H

#include "Arduino.h"

class SimpleWifiRestart {
private:
    unsigned long lastCheckTime;
    unsigned long checkInterval;
    int maxFailures;
    int currentFailures;
    bool wifiConnected;
    unsigned long restartDelay;
    unsigned long lastRestartTime;
    bool restartInProgress;
    
    // Internal methods
    bool checkWifiStatus();
    void performWifiRestart();
    void updateConnectionStatus();
    
public:
    SimpleWifiRestart();
    
    // Configuration methods
    void setCheckInterval(unsigned long intervalMs);
    void setMaxFailures(int maxFail);
    void setRestartDelay(unsigned long delayMs);
    
    // Main functionality
    void checkAndRestart();
    void forceRestart();
    
    // Status methods
    bool isWifiConnected() const;
    int getFailureCount() const;
    unsigned long getLastCheckTime() const;
    bool isRestartInProgress() const;
    
    // Reset methods
    void resetFailureCount();
    void reset();
};

#endif // SIMPLE_WIFI_RESTART_H
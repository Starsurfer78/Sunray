// SimpleWifiRestart.cpp
// Implementation of simple WiFi restart functionality

#include "SimpleWifiRestart.h"
#include "robot.h"

#ifdef __linux__
#include "Process.h"
#endif

SimpleWifiRestart::SimpleWifiRestart() {
    lastCheckTime = 0;
    checkInterval = 60000; // Default: 60 seconds
    maxFailures = 2;
    currentFailures = 0;
    wifiConnected = false;
    restartDelay = 5000; // Default: 5 seconds
    lastRestartTime = 0;
    restartInProgress = false;
}

void SimpleWifiRestart::setCheckInterval(unsigned long intervalMs) {
    checkInterval = intervalMs;
}

void SimpleWifiRestart::setMaxFailures(int maxFail) {
    maxFailures = maxFail;
}

void SimpleWifiRestart::setRestartDelay(unsigned long delayMs) {
    restartDelay = delayMs;
}

bool SimpleWifiRestart::checkWifiStatus() {
#ifdef __linux__
    // Use wpa_cli to check WiFi status
    Process wifiCheck;
    wifiCheck.runShellCommand("wpa_cli -i wlan0 status | grep wpa_state | cut -d '=' -f2");
    
    String status = "";
    while (wifiCheck.available()) {
        status += (char)wifiCheck.read();
    }
    
    status.trim();
    return (status == "COMPLETED");
#else
    // For non-Linux platforms, assume connected
    return true;
#endif
}

void SimpleWifiRestart::performWifiRestart() {
#ifdef __linux__
    CONSOLE.println("SimpleWifiRestart: Performing WiFi restart...");
    
    // Restart WiFi interface
    Process restartProcess;
    restartProcess.runShellCommand("sudo ifdown wlan0 && sleep 2 && sudo ifup wlan0");
    
    // Alternative method if ifup/ifdown not available
    // restartProcess.runShellCommand("sudo systemctl restart wpa_supplicant");
    
    lastRestartTime = millis();
    restartInProgress = true;
    
    CONSOLE.println("SimpleWifiRestart: WiFi restart initiated");
#else
    CONSOLE.println("SimpleWifiRestart: WiFi restart not supported on this platform");
#endif
}

void SimpleWifiRestart::updateConnectionStatus() {
    bool newStatus = checkWifiStatus();
    
    if (newStatus != wifiConnected) {
        wifiConnected = newStatus;
        
        if (wifiConnected) {
            CONSOLE.println("SimpleWifiRestart: WiFi connected");
            currentFailures = 0; // Reset failure count on successful connection
            restartInProgress = false;
        } else {
            CONSOLE.println("SimpleWifiRestart: WiFi disconnected");
            currentFailures++;
        }
    }
}

void SimpleWifiRestart::checkAndRestart() {
    unsigned long currentTime = millis();
    
    // Check if it's time for a status check
    if (currentTime - lastCheckTime >= checkInterval) {
        lastCheckTime = currentTime;
        updateConnectionStatus();
        
        // Check if restart is needed
        if (!wifiConnected && currentFailures >= maxFailures && !restartInProgress) {
            // Check if enough time has passed since last restart
            if (currentTime - lastRestartTime >= restartDelay) {
                performWifiRestart();
            }
        }
    }
    
    // Check if restart process has completed
    if (restartInProgress && (currentTime - lastRestartTime >= restartDelay)) {
        updateConnectionStatus();
        if (wifiConnected) {
            restartInProgress = false;
        }
    }
}

void SimpleWifiRestart::forceRestart() {
    CONSOLE.println("SimpleWifiRestart: Force restart requested");
    currentFailures = maxFailures; // Ensure restart condition is met
    lastRestartTime = 0; // Allow immediate restart
    restartInProgress = false;
    performWifiRestart();
}

bool SimpleWifiRestart::isWifiConnected() const {
    return wifiConnected;
}

int SimpleWifiRestart::getFailureCount() const {
    return currentFailures;
}

unsigned long SimpleWifiRestart::getLastCheckTime() const {
    return lastCheckTime;
}

bool SimpleWifiRestart::isRestartInProgress() const {
    return restartInProgress;
}

void SimpleWifiRestart::resetFailureCount() {
    currentFailures = 0;
    CONSOLE.println("SimpleWifiRestart: Failure count reset");
}

void SimpleWifiRestart::reset() {
    lastCheckTime = 0;
    currentFailures = 0;
    wifiConnected = false;
    lastRestartTime = 0;
    restartInProgress = false;
    CONSOLE.println("SimpleWifiRestart: Reset complete");
}
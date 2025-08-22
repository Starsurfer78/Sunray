#ifndef SIMPLE_WIFI_RESTART_H
#define SIMPLE_WIFI_RESTART_H

#include "Arduino.h"

/**
 * Einfache WLAN-Neustart-Klasse für Sunray
 * 
 * Bietet minimale Funktionalität zum Neustart des WLAN-Adapters
 * bei Verbindungsproblemen - viel einfacher als komplexes Management.
 */
class SimpleWifiRestart {
public:
    enum WifiStatus {
        CONNECTED,
        DISCONNECTED,
        RESTARTING,
        FAILED
    };
    
    SimpleWifiRestart();
    
    // Initialization
    void begin();
    
    // Checks WiFi status and restarts if needed
    void checkAndRestart();
    
    // Forces a WiFi restart
    void forceRestart();
    
    // Konfiguration
    void setCheckInterval(unsigned long intervalMs) { checkIntervalMs = intervalMs; }
    void setMaxFailures(int maxFail) { maxConsecutiveFailures = maxFail; }
    
    // Status
    bool isWifiConnected() const { return wifiConnected; }
    WifiStatus getStatus() const { return currentStatus; }
    int getFailureCount() const { return consecutiveFailures; }
    unsigned long getCheckInterval() const { return checkIntervalMs; }
    unsigned long getRestartTimeout() const { return maxConsecutiveFailures; }
    
private:
    unsigned long lastCheckTime;
    unsigned long checkIntervalMs;
    int consecutiveFailures;
    int maxConsecutiveFailures;
    bool wifiConnected;
    WifiStatus currentStatus;
    
    // Hilfsfunktionen
    bool checkWifiStatus();
    void restartWifiAdapter();
    void updateStatus();
};

#endif // SIMPLE_WIFI_RESTART_H
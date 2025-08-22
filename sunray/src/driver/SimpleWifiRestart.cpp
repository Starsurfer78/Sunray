#include "SimpleWifiRestart.h"
#include "../../config.h"

#ifdef __linux__
#include <cstdlib>
#endif

SimpleWifiRestart::SimpleWifiRestart() {
    lastCheckTime = 0;
    checkIntervalMs = WIFI_RESTART_CHECK_INTERVAL; // configurable check interval
    consecutiveFailures = 0;
    maxConsecutiveFailures = WIFI_RESTART_MAX_FAILURES; // configurable max failures
    wifiConnected = false;
    currentStatus = DISCONNECTED;
}

void SimpleWifiRestart::begin() {
    Serial.println("SimpleWifiRestart: Initializing...");
    updateStatus();
}

void SimpleWifiRestart::checkAndRestart() {
    unsigned long currentTime = millis();
    
    // Check only at defined intervals
    if (currentTime - lastCheckTime < checkIntervalMs) {
        return;
    }
    
    lastCheckTime = currentTime;
    
    // Check WiFi status
    bool connected = checkWifiStatus();
    
    if (connected) {
        consecutiveFailures = 0;
        wifiConnected = true;
        currentStatus = CONNECTED;
    } else {
        consecutiveFailures++;
        wifiConnected = false;
        currentStatus = DISCONNECTED;
        
        // After multiple failures: restart
        if (consecutiveFailures >= maxConsecutiveFailures) {
            Serial.println("WiFi: Multiple connection errors - restarting adapter");
            currentStatus = RESTARTING;
            forceRestart();
            consecutiveFailures = 0; // Reset after restart
        }
    }
}

void SimpleWifiRestart::forceRestart() {
    Serial.println("WiFi: Restarting adapter...");
    currentStatus = RESTARTING;
    restartWifiAdapter();
    
    // Wait after restart (configurable delay)
    delay(WIFI_RESTART_DELAY);
    
    Serial.println("WiFi: Restart completed");
    updateStatus();
}

bool SimpleWifiRestart::checkWifiStatus() {
#ifdef __linux__
    // Simple check: ping to gateway
    int result = system("ping -c 1 -W 2 $(ip route | grep default | awk '{print $3}' | head -1) > /dev/null 2>&1");
    return (result == 0);
#else
    // On Arduino: always consider connected (no Linux WiFi)
    return true;
#endif
}

void SimpleWifiRestart::restartWifiAdapter() {
#ifdef __linux__
    Serial.println("WiFi: Stopping wpa_supplicant...");
    system("sudo killall wpa_supplicant 2>/dev/null");
    
    delay(2000);
    
    Serial.println("WiFi: Interface down/up...");
    system("sudo ifconfig wlan0 down");
    delay(1000);
    system("sudo ifconfig wlan0 up");
    
    delay(2000);
    
    Serial.println("WiFi: Restarting wpa_supplicant...");
    system("sudo wpa_supplicant -B -i wlan0 -c /etc/wpa_supplicant/wpa_supplicant.conf");
    
    delay(3000);
    
    Serial.println("WiFi: Starting DHCP...");
    system("sudo dhclient wlan0");
#else
    Serial.println("WiFi restart only available on Linux");
#endif
}

void SimpleWifiRestart::updateStatus() {
    bool connected = checkWifiStatus();
    if (connected) {
        currentStatus = CONNECTED;
        wifiConnected = true;
    } else {
        if (currentStatus == RESTARTING) {
            currentStatus = FAILED;
        } else {
            currentStatus = DISCONNECTED;
        }
        wifiConnected = false;
    }
}
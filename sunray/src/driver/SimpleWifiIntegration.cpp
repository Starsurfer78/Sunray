/**
 * Simple WiFi restart integration for SerialRobotDriver
 * 
 * This file shows how SimpleWifiRestart can be integrated into the existing
 * SerialRobotDriver - much simpler than the complex WifiManager solution.
 */

#include "SimpleWifiRestart.h"
#include "SerialRobotDriver.h"
#include "../../config.h"

// Global instance for easy usage
SimpleWifiRestart simpleWifiRestart;

/**
 * Integration into SerialRobotDriver::begin()
 * 
 * Add this line to SerialRobotDriver::begin():
 */
void initSimpleWifiRestart() {
    // Adjust configuration if desired
    simpleWifiRestart.setCheckInterval(60000);  // Check every 60 seconds
    simpleWifiRestart.setMaxFailures(2);        // Restart after 2 failures
}

/**
 * Replacement for updateWifiConnectionState()
 * 
 * Replace the content of SerialRobotDriver::updateWifiConnectionState()
 * with this simple call:
 */
void updateSimpleWifiConnectionState() {
    // Simple call - does everything automatically
    simpleWifiRestart.checkAndRestart();
    
    // Set LED status based on WiFi state
    // (These variables must be available in SerialRobotDriver)
    // ledStateWifiConnected = simpleWifiRestart.isWifiConnected();
    // ledStateWifiInactive = !simpleWifiRestart.isWifiConnected();
}

/**
 * New AT+ commands for manual WiFi restart
 * 
 * Add these commands to the AT+ commands:
 */
void cmdWifiRestart() {
    Serial.println("AT+WIFI_RESTART");
    simpleWifiRestart.forceRestart();
    Serial.println("OK");
}

void cmdWifiStatus() {
    Serial.println("AT+WIFI_STATUS");
    Serial.print("Connected: ");
    Serial.println(simpleWifiRestart.isWifiConnected() ? "YES" : "NO");
    Serial.print("Failures: ");
    Serial.println(simpleWifiRestart.getFailureCount());
    Serial.println("OK");
}

/**
 * Usage in AT+ command processing:
 * 
 * In SerialRobotDriver - add to existing AT+ commands:
 * 
 * if (cmd == "AT+WIFI_RESTART") {
 *     cmdWifiRestart();
 * } else if (cmd == "AT+WIFI_STATUS") {
 *     cmdWifiStatus();
 * }
 */

/**
 * INTEGRATION GUIDE:
 * 
 * 1. In SerialRobotDriver.h:
 *    #include "SimpleWifiRestart.h"
 *    extern SimpleWifiRestart simpleWifiRestart;
 * 
 * 2. In SerialRobotDriver::begin():
 *    initSimpleWifiRestart();
 * 
 * 3. In SerialRobotDriver::updateWifiConnectionState():
 *    Replace the entire content with:
 *    simpleWifiRestart.checkAndRestart();
 *    ledStateWifiConnected = simpleWifiRestart.isWifiConnected();
 *    ledStateWifiInactive = !simpleWifiRestart.isWifiConnected();
 * 
 * 4. Add the new AT+ commands to command processing
 * 
 * That's it! Much simpler than the complex WifiManager solution.
 */
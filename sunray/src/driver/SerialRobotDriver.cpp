// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include "SerialRobotDriver.h"
#include "../../config.h"
#include "../../ioboard.h"
#include "../config/config_structures.h"

#define COMM  ROBOT

//#define DEBUG_SERIAL_ROBOT 1

// AT+ Protocol Error Codes
enum ATProtocolError {
  AT_NO_ERROR = 0,
  AT_CRC_ERROR = 1,
  AT_TIMEOUT_ERROR = 2,
  AT_PARSE_ERROR = 3,
  AT_COMM_LOST = 4
};

void SerialRobotDriver::begin(){
  CONSOLE.println("using robot driver: SerialRobotDriver");
  COMM.begin(ROBOT_BAUDRATE);
  
  // Initialize AT+ protocol parameters from config_alfred.h
  motorTimeout = 100;  // Motor command timeout from config
  summaryTimeout = 500; // Summary command timeout from config
  maxRetries = 3;      // Maximum retries from config
  enableCrcCheck = true; // Enable CRC validation
  encoderTicksLeft = 0;
  encoderTicksRight = 0;
  encoderTicksMow = 0;
  chargeVoltage = 0;
  chargeCurrent = 0;  
  batteryVoltage = 28;
  cpuTemp = 30;
  mowCurr = 0;
  motorLeftCurr = 0;
  motorRightCurr = 0;
  resetMotorTicks = true;
  batteryTemp = 0;
  triggeredLeftBumper = false;
  triggeredRightBumper = false;
  triggeredRain = false;
  triggeredStopButton = false;
  triggeredLift = false;
  motorFault = false;
  mcuCommunicationLost = true;
  nextSummaryTime = 0;
  nextConsoleTime = 0; 
  nextMotorTime = 0;
  nextTempTime = 0;
  nextWifiTime = 0;
  nextLedTime = 0;
  lastMotorCommandTime = 0;
  lastSummaryCommandTime = 0;
  communicationErrors = 0;
  lastErrorTime = 0;
  lastErrorCode = 0;
  ledPanelInstalled = true;
  cmdMotorResponseCounter = 0;
  cmdSummaryResponseCounter = 0;
  cmdMotorCounter = 0;
  cmdSummaryCounter = 0;
  requestLeftPwm = requestRightPwm = requestMowPwm = 0;
  robotID = "XX";
  ledStateWifiInactive = false;
  ledStateWifiConnected = false;
  ledStateGpsFix = false;
  ledStateGpsFloat = false;
  ledStateShutdown = false;  
  ledStateError = false;
  ledStateShutdown = false;

  #ifdef __linux__
    Process p2;
    p2.runShellCommand("pwd");
	  String workingDir = p2.readString();    
    CONSOLE.print("linux working dir (pwd): ");
    CONSOLE.println(workingDir);

    CONSOLE.println("reading robot ID...");
    Process p;
    p.runShellCommand("ip link show eth0 | grep link/ether | awk '{print $2}'");
	  robotID = p.readString();    
    robotID.trim();
    
    CONSOLE.println("ioboard init");

    // IMU/fan power-on code (Alfred-PCB-specific) 

    // switch-on IMU via port-expander PCA9555     
    setImuPowerState(true);
    
    // switch-on fan via port-expander PCA9555     
    setFanPowerState(true);
    
    // select IMU via multiplexer TCA9548A 
    ioI2cMux(MUX_I2C_ADDR, SLAVE_IMU_MPU, true);  // Alfred dev PCB with buzzer
    ioI2cMux(MUX_I2C_ADDR, SLAVE_BUS0, true); // Alfred dev PCB without buzzer    

    // select EEPROM via multiplexer TCA9548A 
    ioI2cMux(MUX_I2C_ADDR, SLAVE_EEPROM, true);  

    // select ADC via multiplexer TCA9548A 
    ioI2cMux(MUX_I2C_ADDR, SLAVE_ADC, true);
    
    // buzzer test
    if (false){
      CONSOLE.println("buzzer test");    
      ioExpanderOut(EX2_I2C_ADDR, EX2_BUZZER_PORT, EX2_BUZZER_PIN, true);
      delay(500);
      ioExpanderOut(EX2_I2C_ADDR, EX2_BUZZER_PORT, EX2_BUZZER_PIN, false);    
    }

    // LEDs
    CONSOLE.println("turning LEDs green");
    if (!setLedState(1, true, false)){
      CONSOLE.println("LED panel communication failed - assuming no LED panel installed");
    }
    setLedState(2, true, false);
    setLedState(3, true, false);
  
    // start ADC
    CONSOLE.println("starting ADC");    
    ioAdcStart(ADC_I2C_ADDR, false, true);

    // ADC test    
    if (true){    
      for (int idx=1; idx < 9; idx++){
        ioAdcMux(idx);            
        ioAdcTrigger(ADC_I2C_ADDR);
        delay(5);
        float v = ioAdc(ADC_I2C_ADDR);
        CONSOLE.print("ADC S");
        CONSOLE.print(idx);
        CONSOLE.print("=");
        CONSOLE.println(v);   
      }
    }    

    // EEPROM test
    if (false){
      CONSOLE.println("EEPROM test");
      ioEepromWriteByte( EEPROM_I2C_ADDR, 0, 42);
      delay(50);
      int v = ioEepromReadByte( EEPROM_I2C_ADDR, 0);
      CONSOLE.print("EEPROM=");
      CONSOLE.println(v);
    }    

  #endif
  
  #ifdef ENABLE_SIMPLE_WIFI_RESTART
    CONSOLE.println("initializing SimpleWifiRestart...");
    simpleWifiRestart.begin();
  #endif
}

bool SerialRobotDriver::setLedState(int ledNumber, bool greenState, bool redState){
  if (!ledPanelInstalled) return false;
  if (ledNumber == 1){
    ledPanelInstalled = ioExpanderOut(EX3_I2C_ADDR, EX3_LED1_GREEN_PORT, EX3_LED1_GREEN_PIN, greenState);
    if (!ledPanelInstalled) return false;
    ledPanelInstalled = ioExpanderOut(EX3_I2C_ADDR, EX3_LED1_RED_PORT, EX3_LED1_RED_PIN, redState);        
    if (!ledPanelInstalled) return false;  
  }
  else if (ledNumber == 2){
    ledPanelInstalled = ioExpanderOut(EX3_I2C_ADDR, EX3_LED2_GREEN_PORT, EX3_LED2_GREEN_PIN, greenState);
    if (!ledPanelInstalled) return false;    
    ledPanelInstalled = ioExpanderOut(EX3_I2C_ADDR, EX3_LED2_RED_PORT, EX3_LED2_RED_PIN, redState);        
    if (!ledPanelInstalled) return false;    
  }
  else if (ledNumber == 3){
    ledPanelInstalled = ioExpanderOut(EX3_I2C_ADDR, EX3_LED3_GREEN_PORT, EX3_LED3_GREEN_PIN, greenState);
    if (!ledPanelInstalled) return false;    
    ledPanelInstalled = ioExpanderOut(EX3_I2C_ADDR, EX3_LED3_RED_PORT, EX3_LED3_RED_PIN, redState);        
    if (!ledPanelInstalled) return false;    
  }
  return true;
}

bool SerialRobotDriver::setFanPowerState(bool state){
  CONSOLE.print("FAN POWER STATE ");
  CONSOLE.println(state);
  return ioExpanderOut(EX1_I2C_ADDR, EX1_FAN_POWER_PORT, EX1_FAN_POWER_PIN, state);
}

bool SerialRobotDriver::setImuPowerState(bool state){
  CONSOLE.print("IMU POWER STATE ");
  CONSOLE.println(state);  
  return ioExpanderOut(EX1_I2C_ADDR, EX1_IMU_POWER_PORT, EX1_IMU_POWER_PIN, state);
}  

bool SerialRobotDriver::getRobotID(String &id){
  id = robotID;
  return true;
}

bool SerialRobotDriver::getMcuFirmwareVersion(String &name, String &ver){
  name = mcuFirmwareName;
  ver = mcuFirmwareVersion;
  return true;
}

float SerialRobotDriver::getCpuTemperature(){
  #ifdef __linux__
    return cpuTemp;
  #else
    return -9999;
  #endif
}

void SerialRobotDriver::updateCpuTemperature(){
  #ifdef __linux__
    //unsigned long startTime = millis();
    String s;        
    while (cpuTempProcess.available()) s+= (char)cpuTempProcess.read();
    if (s.length() > 0) {
      cpuTemp = s.toFloat() / 1000.0;    
      //CONSOLE.print("updateCpuTemperature cpuTemp=");
      //CONSOLE.println(cpuTemp);
    }
    cpuTempProcess.runShellCommand("cat /sys/class/thermal/thermal_zone0/temp");      
    //unsigned long duration = millis() - startTime;        
    //CONSOLE.print("updateCpuTemperature duration: ");
    //CONSOLE.println(duration);        
  #endif
}

void SerialRobotDriver::updateWifiConnectionState(){
  #ifdef ENABLE_SIMPLE_WIFI_RESTART
    // Use SimpleWifiRestart for WiFi management
    simpleWifiRestart.checkAndRestart();
    
    // Update LED states based on WiFi status
    SimpleWifiRestart::WifiStatus status = simpleWifiRestart.getStatus();
    ledStateWifiConnected = (status == SimpleWifiRestart::CONNECTED);
    ledStateWifiInactive = (status == SimpleWifiRestart::DISCONNECTED || status == SimpleWifiRestart::FAILED);
  #else
    // Original WiFi status checking
    #ifdef __linux__
      //unsigned long startTime = millis();   
      String s; 
      while (wifiStatusProcess.available()) s+= (char)wifiStatusProcess.read(); 
      if (s.length() > 0){    
        s.trim();
        //CONSOLE.print("updateWifiConnectionState state=");
        //CONSOLE.println(s);
        // DISCONNECTED, SCANNING, INACTIVE, COMPLETED 
        //CONSOLE.println(s);
        ledStateWifiConnected = (s == "COMPLETED");
        ledStateWifiInactive = (s == "INACTIVE");                   
      }  
      wifiStatusProcess.runShellCommand("wpa_cli -i wlan0 status | grep wpa_state | cut -d '=' -f2");  
      //unsigned long duration = millis() - startTime;        
      //CONSOLE.print("updateWifiConnectionState duration: ");
      //CONSOLE.println(duration);
    #endif
  #endif
}

// send serial request to MCU with enhanced error handling
void SerialRobotDriver::sendRequest(String s){
  // Check for communication timeout
  unsigned long currentTime = millis();
  
  // Calculate CRC
  byte crc = 0;
  for (int i=0; i < s.length(); i++) crc += s[i];
  
  // Format command with CRC
  s += F(",0x");
  if (crc <= 0xF) s += F("0");
  s += String(crc, HEX);  
  s += F("\r\n");             
  
  #ifdef DEBUG_SERIAL_ROBOT
    CONSOLE.print("SerialRobot request: ");
    CONSOLE.println(s);  
  #endif
  
  // Send command
  COMM.print(s);
  
  // Update timing for timeout detection
  if (s.startsWith("AT+M")) {
    lastMotorCommandTime = currentTime;
  } else if (s.startsWith("AT+S")) {
    lastSummaryCommandTime = currentTime;
  }
}


// request MCU SW version
void SerialRobotDriver::requestVersion(){
  String req;
  req += "AT+V";  
  sendRequest(req);
}


// request MCU summary
void SerialRobotDriver::requestSummary(){
  String req;
  req += "AT+S";  
  sendRequest(req);
  cmdSummaryCounter++;
}


// request MCU motor PWM
void SerialRobotDriver::requestMotorPwm(int leftPwm, int rightPwm, int mowPwm){
  String req;
  req += "AT+M,";
  req += rightPwm;      
  req += ",";
  req += leftPwm;    
  req += ",";  
  req += mowPwm;
  //if (abs(mowPwm) > 0)
  //  req += "1";
  //else
  //  req += "0";  
  sendRequest(req);
  cmdMotorCounter++;
}

void SerialRobotDriver::motorResponse(){
  if (cmd.length() < 6) {
    CONSOLE.println("WARN: Motor response too short");
    communicationErrors++;
    return;
  }
  
  int counter = 0;
  int lastCommaIdx = 0;
  
  for (int idx=0; idx < cmd.length(); idx++){
    char ch = cmd[idx];
    
    if ((ch == ',') || (idx == cmd.length()-1)){
      String valueStr = cmd.substring(lastCommaIdx+1, ch==',' ? idx : idx+1);
      int intValue = valueStr.toInt();
      float floatValue = valueStr.toFloat();
      
      switch(counter) {
        case 1:
          encoderTicksRight = intValue;
          break;
        case 2:
          encoderTicksLeft = intValue;
          break;
        case 3:
          encoderTicksMow = intValue;
          break;
        case 4:
          chargeVoltage = floatValue;
          break;
        case 5:
          triggeredLeftBumper = (intValue != 0);
          break;
        case 6:
          triggeredLift = (intValue != 0);
          break;
        case 7:
          triggeredStopButton = (intValue != 0);
          if (triggeredStopButton) {
            CONSOLE.println("INFO: Stop button triggered");
          }
          break;
        default:
          // Handle unexpected parameters
          #ifdef DEBUG_SERIAL_ROBOT
            CONSOLE.print("WARN: Unexpected motor parameter at position ");
            CONSOLE.print(counter);
            CONSOLE.print(": ");
            CONSOLE.println(valueStr);
          #endif
          break;
      }
      
      counter++;
      lastCommaIdx = idx;
    }    
  }
  
  // Update response statistics
  cmdMotorResponseCounter++;
  mcuCommunicationLost = false;
  
  #ifdef DEBUG_SERIAL_ROBOT
    CONSOLE.print("Motor response: L=");
    CONSOLE.print(encoderTicksLeft);
    CONSOLE.print(", R=");
    CONSOLE.print(encoderTicksRight);
    CONSOLE.print(", M=");
    CONSOLE.println(encoderTicksMow);
  #endif
}


void SerialRobotDriver::versionResponse(){
  if (cmd.length() < 6) {
    CONSOLE.println("WARN: Version response too short");
    communicationErrors++;
    return;
  }
  
  int counter = 0;
  int lastCommaIdx = 0;
  
  for (int idx=0; idx < cmd.length(); idx++){
    char ch = cmd[idx];
    
    if ((ch == ',') || (idx == cmd.length()-1)){
      String s = cmd.substring(lastCommaIdx+1, ch==',' ? idx : idx+1);
      s.trim(); // Remove whitespace
      
      switch(counter) {
        case 1:
          mcuFirmwareName = s;
          break;
        case 2:
          mcuFirmwareVersion = s;
          break;
        default:
          #ifdef DEBUG_SERIAL_ROBOT
            CONSOLE.print("WARN: Unexpected version parameter at position ");
            CONSOLE.print(counter);
            CONSOLE.print(": ");
            CONSOLE.println(s);
          #endif
          break;
      }
      
      counter++;
      lastCommaIdx = idx;
    }    
  }
  
  // Update communication status
  mcuCommunicationLost = false;
  
  CONSOLE.print("MCU FIRMWARE: ");
  CONSOLE.print(mcuFirmwareName);
  CONSOLE.print(",");
  CONSOLE.println(mcuFirmwareVersion);
  
  #ifdef DEBUG_SERIAL_ROBOT
    CONSOLE.println("Version response processed successfully");
  #endif
}


void SerialRobotDriver::summaryResponse(){
  if (cmd.length() < 6) {
    CONSOLE.println("WARN: Summary response too short");
    communicationErrors++;
    return;
  }
  
  int counter = 0;
  int lastCommaIdx = 0;
  
  for (int idx=0; idx < cmd.length(); idx++){
    char ch = cmd[idx];
    
    if ((ch == ',') || (idx == cmd.length()-1)){
      String valueStr = cmd.substring(lastCommaIdx+1, ch==',' ? idx : idx+1);
      int intValue = valueStr.toInt();
      float floatValue = valueStr.toFloat();
      
      switch(counter) {
        case 1:
          batteryVoltage = floatValue;
          break;
        case 2:
          chargeVoltage = floatValue;
          break;
        case 3:
          chargeCurrent = floatValue;
          break;
        case 4:
          triggeredLift = (intValue != 0);
          break;
        case 5:
          triggeredLeftBumper = (intValue != 0);
          break;
        case 6:
          triggeredRain = (intValue != 0);
          break;
        case 7:
          motorFault = (intValue != 0);
          if (motorFault) {
            CONSOLE.println("WARN: Motor fault detected");
          }
          break;
        case 8:
          mowCurr = floatValue;
          // Check for mow motor overcurrent using config_alfred.h constant
          #ifdef MOTOR_FAULT_CURRENT
          if (abs(mowCurr) > MOTOR_FAULT_CURRENT) {
            CONSOLE.print("WARN: Mow motor overcurrent: ");
            CONSOLE.print(mowCurr);
            CONSOLE.print("A > ");
            CONSOLE.print(MOTOR_FAULT_CURRENT);
            CONSOLE.println("A");
          }
          #endif
          break;
        case 9:
          motorLeftCurr = floatValue;
          // Check for left motor overcurrent
          #ifdef MOTOR_FAULT_CURRENT
          if (abs(motorLeftCurr) > MOTOR_FAULT_CURRENT) {
            CONSOLE.print("WARN: Left motor overcurrent: ");
            CONSOLE.print(motorLeftCurr);
            CONSOLE.print("A > ");
            CONSOLE.print(MOTOR_FAULT_CURRENT);
            CONSOLE.println("A");
          }
          #endif
          break;
        case 10:
          motorRightCurr = floatValue;
          // Check for right motor overcurrent
          #ifdef MOTOR_FAULT_CURRENT
          if (abs(motorRightCurr) > MOTOR_FAULT_CURRENT) {
            CONSOLE.print("WARN: Right motor overcurrent: ");
            CONSOLE.print(motorRightCurr);
            CONSOLE.print("A > ");
            CONSOLE.print(MOTOR_FAULT_CURRENT);
            CONSOLE.println("A");
          }
          #endif
          break;
        case 11:
          batteryTemp = floatValue;
          break;
        default:
          // Handle unexpected parameters
          #ifdef DEBUG_SERIAL_ROBOT
            CONSOLE.print("WARN: Unexpected summary parameter at position ");
            CONSOLE.print(counter);
            CONSOLE.print(": ");
            CONSOLE.println(valueStr);
          #endif
          break;
      }
      
      counter++;
      lastCommaIdx = idx;
    }    
  }
  
  // Update response statistics
  cmdSummaryResponseCounter++;
  mcuCommunicationLost = false;
  
  #ifdef DEBUG_SERIAL_ROBOT
    CONSOLE.print("Summary: Batt=");
    CONSOLE.print(batteryVoltage);
    CONSOLE.print("V, Currents: L=");
    CONSOLE.print(motorLeftCurr);
    CONSOLE.print("A, R=");
    CONSOLE.print(motorRightCurr);
    CONSOLE.print("A, M=");
    CONSOLE.print(mowCurr);
    CONSOLE.println("A");
  #endif
}

// process response with enhanced error handling
void SerialRobotDriver::processResponse(bool checkCrc){
  cmdResponse = "";      
  if (cmd.length() < 4) {
    communicationErrors++;
    lastErrorTime = millis();
    return;
  }
  
  byte expectedCrc = 0;
  int idx = cmd.lastIndexOf(',');
  
  if (idx < 1){
    if (checkCrc && enableCrcCheck){
      CONSOLE.println("SerialRobot: CRC ERROR - No CRC found");
      communicationErrors++;
      lastErrorTime = millis();
      return;
    }
  } else {
    // Calculate expected CRC
    for (int i=0; i < idx; i++) expectedCrc += cmd[i];  
    String s = cmd.substring(idx+1, idx+5);
    int crc = strtol(s.c_str(), NULL, 16);  
    
    if (expectedCrc != crc){
      if (checkCrc && enableCrcCheck){
        CONSOLE.print("SerialRobot: CRC ERROR - Expected: 0x");
        CONSOLE.print(expectedCrc,HEX);
        CONSOLE.print(", Got: 0x");
        CONSOLE.print(crc,HEX);
        CONSOLE.println();
        communicationErrors++;
        lastErrorTime = millis();
        lastErrorCode = static_cast<uint16_t>(RobotErrorCode::ERROR_COMM_CRC);
        return;  
      }      
    } else {
      #ifdef DEBUG_SERIAL_ROBOT
        CONSOLE.print("SerialRobot resp:");
        CONSOLE.println(cmd);
      #endif
      // remove CRC      
      cmd = cmd.substring(0, idx);
      
      // Reset communication lost flag on successful response
      mcuCommunicationLost = false;
    }    
  }     
  
  // Process different response types
  if (cmd[0] == 'M') motorResponse();
  else if (cmd[0] == 'S') summaryResponse();
  else if (cmd[0] == 'V') versionResponse();
  #ifdef ENABLE_SIMPLE_WIFI_RESTART
  else if (cmd.startsWith("AT+WIFI_RESTART")) {
    // Manual WiFi restart command
    simpleWifiRestart.forceRestart();
    COMM.println("OK");
  }
  else if (cmd.startsWith("AT+WIFI_STATUS")) {
    // WiFi status query command
    SimpleWifiRestart::WifiStatus status = simpleWifiRestart.getStatus();
    COMM.print("WIFI_STATUS:");
    switch(status) {
      case SimpleWifiRestart::CONNECTED:
        COMM.println("CONNECTED");
        break;
      case SimpleWifiRestart::DISCONNECTED:
        COMM.println("DISCONNECTED");
        break;
      case SimpleWifiRestart::RESTARTING:
        COMM.println("RESTARTING");
        break;
      case SimpleWifiRestart::FAILED:
        COMM.println("FAILED");
        break;
      default:
        COMM.println("UNKNOWN");
        break;
    }
  }
  else if (cmd.startsWith("AT+WIFI_CONFIG")) {
    // WiFi configuration command
    COMM.print("WIFI_CONFIG:timeout=");
    COMM.print(simpleWifiRestart.getRestartTimeout());
    COMM.print(",interval=");
    COMM.println(simpleWifiRestart.getCheckInterval());
  }
  #endif
  else {
    CONSOLE.print("SerialRobot: Unknown response type: ");
    CONSOLE.println(cmd[0]);
    communicationErrors++;
    lastErrorTime = millis();
  }
}


// process console input
void SerialRobotDriver::processComm(){
  char ch;      
  if (COMM.available()){
    //battery.resetIdle();  
    while ( COMM.available() ){               
      ch = COMM.read();          
      if ((ch == '\r') || (ch == '\n')) {        
        //CONSOLE.println(cmd);
        processResponse(true);              
        //CONSOLE.print(cmdResponse);    
        cmd = "";
      } else if (cmd.length() < 500){
        cmd += ch;
      }
    }
  }     
}

void SerialRobotDriver::updatePanelLEDs(){
  if (ledStateShutdown) {
    setLedState(1, false, false);
    setLedState(2, false, false);
    setLedState(3, false, false);        
    return;    
  }
  // panel led numbers (top-down): 2,3,1   
  // idle/error status
  if (ledStateError){
    setLedState(2, false, true);
  } else {
    setLedState(2, true, false);
  }
  // gps status
  if (ledStateGpsFix){
    setLedState(3, true, false); 
  } 
  else if (ledStateGpsFloat) {
    setLedState(3, false, true);
  } else {
    setLedState(3, false, false);    
  }
  // wifi status
  if (ledStateWifiConnected){ 
    setLedState(1, true, false);
  } else if (ledStateWifiInactive) {
    setLedState(1, false, true);
  } else {
    setLedState(1, false, false);
  }
}

void SerialRobotDriver::run(){  
  processComm();
  
  unsigned long currentTime = millis();
  
  // Motor control loop - 50 Hz
  if (currentTime > nextMotorTime){
    nextMotorTime = currentTime + 20;
    requestMotorPwm(requestLeftPwm, requestRightPwm, requestMowPwm);
    
    // Check for motor command timeout
    if (lastMotorCommandTime > 0 && (currentTime - lastMotorCommandTime) > motorTimeout) {
      CONSOLE.println("WARN: Motor command timeout");
      mcuCommunicationLost = true;
      communicationErrors++;
      lastErrorCode = static_cast<uint16_t>(RobotErrorCode::ERROR_COMM_TIMEOUT);
    }
  }
  
  // Summary request loop - 2 Hz
  if (currentTime > nextSummaryTime){
    nextSummaryTime = currentTime + 500;
    requestSummary();
    
    // Check for summary command timeout
    if (lastSummaryCommandTime > 0 && (currentTime - lastSummaryCommandTime) > summaryTimeout) {
      CONSOLE.println("WARN: Summary command timeout");
      mcuCommunicationLost = true;
      communicationErrors++;
      lastErrorCode = static_cast<uint16_t>(RobotErrorCode::ERROR_COMM_TIMEOUT);
    }
  }
  
  // Status monitoring loop - 1 Hz
  if (currentTime > nextConsoleTime){
    nextConsoleTime = currentTime + 1000;
    
    // Request version if not known and communication is working
    if (!mcuCommunicationLost && mcuFirmwareName == ""){
      requestVersion();
    }
    
    // Check communication health
    if ((cmdMotorCounter > 0) && (cmdMotorResponseCounter == 0)){
      CONSOLE.println("WARN: No motor responses - resetting motor ticks");
      resetMotorTicks = true;
      mcuCommunicationLost = true;
    }
    
    // Communication frequency monitoring
    if (cmdMotorResponseCounter < 30) {
      CONSOLE.print("WARN: Low communication frequency - Motor: ");
      CONSOLE.print(cmdMotorResponseCounter);
      CONSOLE.print("/");
      CONSOLE.print(cmdMotorCounter);
      CONSOLE.print(", Summary: ");
      CONSOLE.print(cmdSummaryResponseCounter);
      CONSOLE.print("/");
      CONSOLE.print(cmdSummaryCounter);
      CONSOLE.print(", Errors: ");
      CONSOLE.println(communicationErrors);
      
      if (cmdMotorResponseCounter == 0){
        // Reset motor control on complete communication loss
        mcuCommunicationLost = true;
      }
    }
    
    // Reset counters
    cmdMotorCounter = cmdMotorResponseCounter = cmdSummaryCounter = cmdSummaryResponseCounter = 0;
    communicationErrors = 0;
  }  
  if (millis() > nextLedTime){
    nextLedTime = millis() + 3000;  // 3 sec
    updatePanelLEDs();
  }
  if (millis() > nextTempTime){
    nextTempTime = millis() + 59000; // 59 sec
    updateCpuTemperature();
    if (cpuTemp < 60){      
      setFanPowerState(false);
    } else if (cpuTemp > 65){
      setFanPowerState(true);
    }
  }
  if (millis() > nextWifiTime){
    nextWifiTime = millis() + 7000; // 7 sec
    updateWifiConnectionState();
  }
}


// ------------------------------------------------------------------------------------

SerialMotorDriver::SerialMotorDriver(SerialRobotDriver &sr): serialRobot(sr){
} 

void SerialMotorDriver::begin(){
  lastEncoderTicksLeft=0;
  lastEncoderTicksRight=0;
  lastEncoderTicksMow=0;         
}

void SerialMotorDriver::run(){
}

void SerialMotorDriver::setMowHeight(int mowHeightMillimeter){
}

void SerialMotorDriver::setMotorPwm(int leftPwm, int rightPwm, int mowPwm){  
  //serialRobot.requestMotorPwm(leftPwm, rightPwm, mowPwm);
  serialRobot.requestLeftPwm = leftPwm;
  serialRobot.requestRightPwm = rightPwm;
  // Alfred mowing motor driver seem to start start mowing motor more successfully with full PWM (100%) values...  
  if (mowPwm > 0) mowPwm = 255;
    else if (mowPwm < 0) mowPwm = -255;
  serialRobot.requestMowPwm = mowPwm;
}

void SerialMotorDriver::getMotorFaults(bool &leftFault, bool &rightFault, bool &mowFault){
  leftFault = serialRobot.motorFault;
  rightFault = serialRobot.motorFault;
  if (serialRobot.motorFault){
    CONSOLE.print("serialRobot: motorFault (lefCurr=");
    CONSOLE.print(serialRobot.motorLeftCurr);
    CONSOLE.print(" rightCurr=");
    CONSOLE.print(serialRobot.motorRightCurr);
    CONSOLE.print(" mowCurr=");
    CONSOLE.println(serialRobot.mowCurr);
  }
  mowFault = false;
}

void SerialMotorDriver::resetMotorFaults(){
  CONSOLE.println("serialRobot: resetting motor fault");
  //serialRobot.requestMotorPwm(1, 1, 0);
  //delay(1);
  //serialRobot.requestMotorPwm(0, 0, 0);
}

void SerialMotorDriver::getMotorCurrent(float &leftCurrent, float &rightCurrent, float &mowCurrent) {  
  //leftCurrent = 0.5;
  //rightCurrent = 0.5;
  //mowCurrent = 0.8;
  leftCurrent = serialRobot.motorLeftCurr;
  rightCurrent = serialRobot.motorRightCurr;
  mowCurrent = serialRobot.mowCurr;
}

void SerialMotorDriver::getMotorEncoderTicks(int &leftTicks, int &rightTicks, int &mowTicks){
  if (serialRobot.mcuCommunicationLost) {
    //CONSOLE.println("getMotorEncoderTicks: no ticks!");    
    leftTicks = rightTicks = 0; mowTicks = 0;
    return;
  }
  if (serialRobot.resetMotorTicks){
    serialRobot.resetMotorTicks = false;
    //CONSOLE.println("getMotorEncoderTicks: resetMotorTicks");
    lastEncoderTicksLeft = serialRobot.encoderTicksLeft;
    lastEncoderTicksRight = serialRobot.encoderTicksRight;
    lastEncoderTicksMow = serialRobot.encoderTicksMow;
  }
  leftTicks = serialRobot.encoderTicksLeft - lastEncoderTicksLeft;
  rightTicks = serialRobot.encoderTicksRight - lastEncoderTicksRight;
  mowTicks = serialRobot.encoderTicksMow - lastEncoderTicksMow;
  if (leftTicks > 1000){
    leftTicks = 0;
  }
  if (rightTicks > 1000){
    rightTicks = 0;
  } 
  if (mowTicks > 1000){
    mowTicks = 0;
  }
  lastEncoderTicksLeft = serialRobot.encoderTicksLeft;
  lastEncoderTicksRight = serialRobot.encoderTicksRight;
  lastEncoderTicksMow = serialRobot.encoderTicksMow;
}


// ------------------------------------------------------------------------------------

SerialBatteryDriver::SerialBatteryDriver(SerialRobotDriver &sr) : serialRobot(sr){
  mcuBoardPoweredOn = true;
  nextADCTime = 0;
  nextTempTime = 0;
  batteryTemp = 0;
  adcTriggered = false;
  linuxShutdownTime = 0;
}

void SerialBatteryDriver::begin(){
}

void SerialBatteryDriver::run(){
  if (millis() > nextTempTime){
    nextTempTime = millis() + 57000; // 57 sec
    updateBatteryTemperature();
  }
}    

void SerialBatteryDriver::updateBatteryTemperature(){
  #ifdef __linux__
    //unsigned long startTime = millis();
    String s;        
    while (batteryTempProcess.available()) s+= (char)batteryTempProcess.read();
    if (s.length() > 0) {
      batteryTemp = s.toFloat() / 1000.0;    
      //CONSOLE.print("updateBatteryTemperature batteryTemp=");
      //CONSOLE.println(batteryTemp);
    }
    batteryTempProcess.runShellCommand("cat /sys/class/thermal/thermal_zone1/temp");  
    //unsigned long duration = millis() - startTime;        
    //CONSOLE.print("updateBatteryTemperature duration: ");
    //CONSOLE.println(duration);        
  #endif
}


float SerialBatteryDriver::getBatteryTemperature(){
  #ifdef __linux__
    return -9999; //batteryTemp; // linux reported bat temp not useful as seem to be constant 31 degree
  #else
    return -9999;
  #endif
}

float SerialBatteryDriver::getBatteryVoltage(){
  #ifdef __linux__
    // detect if MCU PCB is switched-off
    if (millis() > nextADCTime){
      if (!adcTriggered){
        // trigger ADC measurement (mcuAna)
        ioAdcMux(ADC_MCU_ANA);
        ioAdcTrigger(ADC_I2C_ADDR);   
        adcTriggered = true; 
        nextADCTime = millis() + 5;    
      } else {           
        nextADCTime = millis() + 1000;
        adcTriggered = false;
        float v = ioAdc(ADC_I2C_ADDR);
        mcuBoardPoweredOn = true;
        if (v < 0){
          CONSOLE.println("ERROR reading ADC channel mcuAna!");
          // reset ADC
          ioAdcStart(ADC_I2C_ADDR, false, true);
        } else {
          if ((v >0) && (v < 0.8)){
            // no mcuAna, MCU PCB is probably switched off
            CONSOLE.print("mcuAna=");
            CONSOLE.println(v);      
            CONSOLE.println("MCU PCB powered OFF!");
            mcuBoardPoweredOn = false;        
          }
        }
      }
    }    
    if (serialRobot.mcuCommunicationLost){
      // return 0 volt if MCU PCB is connected and powered-off (Linux will shutdown)
      //if (!mcuBoardPoweredOn) return 0;
      // return 28 volts if MCU PCB is not connected (so Linux can be tested without MCU PCB 
      // and will not shutdown if mower is not connected)      
      return 28;      
    }
  #endif         
  return serialRobot.batteryVoltage;
}

float SerialBatteryDriver::getChargeVoltage(){
  return serialRobot.chargeVoltage;
}
    
float SerialBatteryDriver::getChargeCurrent(){
  return serialRobot.chargeCurrent;
} 

void SerialBatteryDriver::enableCharging(bool flag){
}


void SerialBatteryDriver::keepPowerOn(bool flag){
  #ifdef __linux__
    if (flag){
      // keep power on
      linuxShutdownTime = 0;
      serialRobot.ledStateShutdown = false;
    } else {
      // shutdown linux - request could be for two reasons:
      // 1. battery voltage sent by MUC-PCB seem to be too low 
      // 2. MCU-PCB is powered-off 
      if (linuxShutdownTime == 0){
        linuxShutdownTime = millis() + 5000; // some timeout 
        // turn off panel LEDs
        serialRobot.ledStateShutdown = true;
        serialRobot.updatePanelLEDs();        
      }
      if (millis() > linuxShutdownTime){
        linuxShutdownTime = millis() + 10000; // re-trigger linux command after 10 secs
        CONSOLE.println("LINUX will SHUTDOWN!");
        // switch-off fan via port-expander PCA9555     
        serialRobot.setFanPowerState(false);
        Process p;
        p.runShellCommand("shutdown now");
      }
    }   
  #endif  
}


// ------------------------------------------------------------------------------------

SerialBumperDriver::SerialBumperDriver(SerialRobotDriver &sr): serialRobot(sr){
}

void SerialBumperDriver::begin(){
}

void SerialBumperDriver::run(){

}

bool SerialBumperDriver::nearObstacle(){
  return false;
}

bool SerialBumperDriver::obstacle(){
  return (serialRobot.triggeredLeftBumper || serialRobot.triggeredRightBumper); 
}

bool SerialBumperDriver::getLeftBumper(){
  return (serialRobot.triggeredLeftBumper);
}

bool SerialBumperDriver::getRightBumper(){
  return (serialRobot.triggeredRightBumper);
}	

void SerialBumperDriver::getTriggeredBumper(bool &leftBumper, bool &rightBumper){
  leftBumper = serialRobot.triggeredLeftBumper;
  rightBumper = serialRobot.triggeredRightBumper;
}  	  		    


// ------------------------------------------------------------------------------------


SerialStopButtonDriver::SerialStopButtonDriver(SerialRobotDriver &sr): serialRobot(sr){
}

void SerialStopButtonDriver::begin(){
}

void SerialStopButtonDriver::run(){

}

bool SerialStopButtonDriver::triggered(){
  return (serialRobot.triggeredStopButton); 
}

// ------------------------------------------------------------------------------------


SerialRainSensorDriver::SerialRainSensorDriver(SerialRobotDriver &sr): serialRobot(sr){
}

void SerialRainSensorDriver::begin(){
}

void SerialRainSensorDriver::run(){

}

bool SerialRainSensorDriver::triggered(){
  return (serialRobot.triggeredRain); 
}

// ------------------------------------------------------------------------------------

SerialLiftSensorDriver::SerialLiftSensorDriver(SerialRobotDriver &sr): serialRobot(sr){
}

void SerialLiftSensorDriver::begin(){
}

void SerialLiftSensorDriver::run(){
}

bool SerialLiftSensorDriver::triggered(){
  return (serialRobot.triggeredLift);
}


// ------------------------------------------------------------------------------------

SerialBuzzerDriver::SerialBuzzerDriver(SerialRobotDriver &sr): serialRobot(sr){
}

void SerialBuzzerDriver::begin(){
}

void SerialBuzzerDriver::run(){
}

void SerialBuzzerDriver::noTone(){
  ioExpanderOut(EX2_I2C_ADDR, EX2_BUZZER_PORT, EX2_BUZZER_PIN, false);
}

void SerialBuzzerDriver::tone(int freq){
  ioExpanderOut(EX2_I2C_ADDR, EX2_BUZZER_PORT, EX2_BUZZER_PIN, true);
}




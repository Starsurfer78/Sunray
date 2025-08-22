// Ardumower Sunray
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#ifndef CONFIG_STRUCTURES_H
#define CONFIG_STRUCTURES_H

// Configuration structures for Sunray Arduino firmware
// This file contains data structures used for configuration management
// and communication between Arduino and external systems (like Alfred)

// Robot error codes for communication and diagnostics
enum class RobotErrorCode : uint16_t {
  ERROR_NONE = 0,
  ERROR_COMM_CRC = 1,
  ERROR_COMM_TIMEOUT = 2,
  ERROR_COMM_INVALID = 3,
  ERROR_MOTOR_FAULT = 4,
  ERROR_SENSOR_FAULT = 5
};

// AT+ Protocol configuration
struct ATProtocolConfig {
  unsigned long timeout;
  unsigned int maxRetries;
  bool enableCRC;
};

// Motor configuration structure
struct MotorConfig {
  int maxPWM;
  int minPWM;
  float maxCurrent;
  unsigned long timeout;
};

// Sensor configuration structure
struct SensorConfig {
  bool enableBumper;
  bool enableLift;
  bool enableRain;
  float threshold;
};

// Communication configuration
struct CommConfig {
  unsigned long baudRate;
  unsigned long timeout;
  bool enableDebug;
};

// Main configuration structure
struct RobotConfig {
  ATProtocolConfig atProtocol;
  MotorConfig motor;
  SensorConfig sensor;
  CommConfig comm;
};

// Default configuration values
extern const RobotConfig DEFAULT_CONFIG;

#endif // CONFIG_STRUCTURES_H
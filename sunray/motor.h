// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#ifndef MOTOR_H
#define MOTOR_H

#include "pid.h"
#include "lowpass_filter.h"

// selected motor
enum MotorSelect {MOTOR_LEFT, MOTOR_RIGHT, MOTOR_MOW} ;
typedef enum MotorSelect MotorSelect;

// Motor sensing constants
static const float CURRENT_LP_FILTER_COEFF = 0.995f;     // Low-pass filter coefficient for current sensing
static const float MOW_PWM_LP_FILTER_COEFF = 0.99f;      // Low-pass filter coefficient for mow PWM
static const unsigned long SENSE_INTERVAL_MS = 20;       // Motor sensing interval in milliseconds
static const float ROBOT_MASS_FACTOR = 1.0f;             // Robot mass factor for normalized current calculation

// Motor run() function constants
static const unsigned long MOTOR_CONTROL_INTERVAL_MS = 50;        // Motor control loop interval in milliseconds
static const unsigned long MOTOR_FAULT_RECOVERY_DELAY_MS = 1000;  // Delay before attempting motor fault recovery
static const unsigned long MOTOR_FAULT_RECOVERY_TIMEOUT_MS = 10000; // Timeout for motor fault recovery attempts
static const float RPM_LP_FILTER_COEFF = 0.9f;                   // Low-pass filter coefficient for RPM calculation
static const float MOW_TICKS_PER_REVOLUTION = 6.0f;              // Assumed ticks per revolution for mow motor
static const int MAX_MOTOR_FAULT_RECOVERY_ATTEMPTS = 10;          // Maximum number of motor fault recovery attempts
static const int ZERO_TICK_THRESHOLD = 2;                         // Threshold for detecting zero ticks before setting RPM to 0


class Motor {
  public:
    float robotPitch;  // robot pitch (rad)
    float wheelBaseCm;  // wheel-to-wheel diameter
    int wheelDiameter;   // wheel diameter (mm)
    int ticksPerRevolution; // ticks per revolution
    float ticksPerCm;  // ticks per cm
    bool activateLinearSpeedRamp;  // activate ramp to accelerate/slow down linear speed?
    bool toggleMowDir; // toggle mowing motor direction each mow motor start?    
    bool motorLeftSwapDir;
    bool motorRightSwapDir;
    bool motorError;
    bool motorLeftOverload; 
    bool motorRightOverload; 
    bool motorMowOverload; 
    bool tractionMotorsEnabled;       
    bool enableMowMotor;
    bool motorMowForwardSet; 
    bool odometryError;    
    unsigned long motorOverloadDuration; // accumulated duration (ms)
    int  pwmMax;
    int  pwmMaxMow;
    int mowHeightMillimeter;  
    float  pwmSpeedOffset;
    float mowMotorCurrentAverage;
    float currentFactor;
    bool pwmSpeedCurveDetection;
    unsigned long motorLeftTicks;
    unsigned long motorRightTicks;
    unsigned long motorMowTicks;    
    float linearSpeedSet; // m/s
    float angularSpeedSet; // rad/s
    float motorLeftSense; // left motor current (amps)
    float motorRightSense; // right  motor current (amps)
    float motorMowSense;  // mower motor current (amps)         
    float motorLeftSenseLP; // left motor current (amps, low-pass)
    float motorRightSenseLP; // right  motor current (amps, low-pass)
    float motorMowSenseLP;  // mower motor current (amps, low-pass)       
    float motorsSenseLP; // all motors current (amps, low-pass)
    float motorLeftSenseLPNorm; 
    float motorRightSenseLPNorm;
    unsigned long motorMowSpinUpTime;
    bool motorRecoveryState;    
    PID motorLeftPID;
    PID motorRightPID;    
    LowPassFilter motorLeftLpf;
    LowPassFilter motorRightLpf;        
    void begin();
    void run();      
    void test();
    void plot();
    void enableTractionMotors(bool enable);
    void setLinearAngularSpeed(float linear, float angular, bool useLinearRamp = true);
    void setMowState(bool switchOn);   
    void setMowMaxPwm( int val );
    void setMowHeightMillimeter( int val );
    void stopImmediately(bool includeMowerMotor);
  protected: 
    float motorLeftRpmSet; // set speed
    float motorRightRpmSet;   
    float motorLeftRpmCurr;
    float motorRightRpmCurr;
    float motorMowRpmCurr;    
    float motorLeftRpmCurrLP;
    float motorRightRpmCurrLP;    
    float motorMowRpmCurrLP;    
    float motorLeftRpmLast;
    float motorRightRpmLast;
    float motorMowPWMSet;  
    float motorMowPWMCurr; 
    int motorLeftPWMCurr;
    int motorRightPWMCurr;    
    float motorMowPWMCurrLP; 
    float motorLeftPWMCurrLP;
    float motorRightPWMCurrLP;    
    unsigned long lastControlTime;    
    unsigned long nextSenseTime;          
    bool recoverMotorFault;
    int recoverMotorFaultCounter;
    unsigned long nextRecoverMotorFaultTime;
    int motorLeftTicksZero;    
    int motorRightTicksZero;    
    bool setLinearAngularSpeedTimeoutActive;
    unsigned long setLinearAngularSpeedTimeout;    
    void speedPWM ( int pwmLeft, int pwmRight, int pwmMow );
    void control();    
    bool checkFault();
    void checkOverload();
    bool checkOdometryError();
    bool checkMowRpmFault();
    bool checkCurrentTooHighError();    
    bool checkCurrentTooLowError();
    void sense();
    void dumpOdoTicks(int seconds);
    float calculatePitchFactor(int motorPWMCurr, float cosPitch) const;
    void applyCurrentFiltering();
    void calculateNormalizedCurrents();
    bool detectMotorFaults();
    void handleMotorRecovery();
    void handleLinearAngularSpeedTimeout();
    void processEncoderTicksAndCalculateRpm();
    void computeMotorPidControl(PID& motorPID, LowPassFilter& motorLpf, float rpmCurr, float rpmSet, int& pwmCurr, const char* motorName);
    void logMotorCurrentError(const char* motorName, float currentValue, float thresholdValue, const char* comparison, int pwmValue = -1);
    void logMotorFaultError(const char* motorName, const char* faultType);
    void applyPwmZeroingForLowRpm();
    void initializePidAndLpfControllers();
};


#endif

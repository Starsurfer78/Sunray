// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include "LineTracker.h"
#include "robot.h"
#include "StateEstimator.h"
#include "helper.h"
#include "pid.h"
#include "src/op/op.h"
#include "Stats.h"
#include "events.h"

// LineTracker constants
const float SMOOTH_CURVE_TARGET_TOLERANCE = 0.2;        // target reached tolerance for smooth curves
const float ROTATION_DISTANCE_THRESHOLD = 0.5;          // distance threshold for allowing rotations
const float ROTATION_ANGULAR_SPEED = 29.0 / 180.0 * PI; // 29 degree/s rotation speed
const float ROTATION_ANGLE_RESET_THRESHOLD = 90.0;      // angle threshold for resetting rotation direction
const float APPROACH_SPEED_DISTANCE = 0.5;              // distance for speed reduction when approaching waypoints
const float APPROACH_SPEED_VALUE = 0.1;                 // reduced speed value when approaching
const float FLOAT_SOLUTION_MAX_SPEED = 0.1;             // max speed for GPS float solution
const float OBSTACLE_SLOW_SPEED = 0.1;                  // speed when near obstacles
const float GROUND_SPEED_THRESHOLD = 0.03;              // minimum ground speed threshold
const float LINEAR_MOTION_TIMEOUT = 5000;               // timeout for linear motion in ms
const float REFLECTOR_TAG_MAX_ANGULAR = 0.015;          // max angular speed for reflector tag mode
const float REFLECTOR_TAG_MAX_LINEAR = 0.05;            // max linear speed for reflector tag mode
const float MOW_MOTOR_SPINUP_TIMEOUT = 10000;           // mow motor spin-up timeout in ms

Polygon circle(8);

float stanleyTrackingNormalK = STANLEY_CONTROL_K_NORMAL;
float stanleyTrackingNormalP = STANLEY_CONTROL_P_NORMAL;    
float stanleyTrackingSlowK = STANLEY_CONTROL_K_SLOW;
float stanleyTrackingSlowP = STANLEY_CONTROL_P_SLOW;    

float setSpeed = 0.1; // linear speed (m/s)
bool rotateLeft = false;
bool rotateRight = false;
bool angleToTargetFits = false;
bool langleToTargetFits = false;
bool targetReached = false;
float trackerDiffDelta = 0;
bool stateKidnapped = false;
bool printmotoroverload = false;
bool trackerDiffDelta_positive = false;
float lastLineDist = 0;

// Calculate linear speed based on current conditions
float calculateLinearSpeed(float setSpeed, bool straight, bool mow, bool trackslow_allowed) {
  float linear = 1.0;
  
  if (maps.trackSlow && trackslow_allowed) {
    // planner forces slow tracking (e.g. docking etc)
    linear = DOCK_LINEAR_SPEED; // 0.1           
  } else if (     ((setSpeed > 0.2) && (maps.distanceToTargetPoint(stateX, stateY) < APPROACH_SPEED_DISTANCE) && (!straight))   // approaching
        || ((linearMotionStartTime != 0) && (millis() < linearMotionStartTime + 3000))                      // leaving  
     ) 
  {
    linear = APPROACH_SPEED_VALUE; // reduce speed when approaching/leaving waypoints          
  } 
  else {
    if ((stateLocalizationMode == LOC_GPS) && (gps.solution == SOL_FLOAT)){        
      linear = min(setSpeed, FLOAT_SOLUTION_MAX_SPEED); // reduce speed for float solution
    } else
      linear = setSpeed;         // desired speed
    if (bumperDriver.nearObstacle()){
      linear = OBSTACLE_SLOW_SPEED;  // slow down near obstacles 
    }
    if (lidarBumper.nearObstacle()){
      linear = OBSTACLE_SLOW_SPEED;  // slow down near obstacles 
    }
    if (sonar.nearObstacle()) {
      linear = OBSTACLE_SLOW_SPEED; // slow down near obstacles
    }
  }      
  // slow down speed in case of overload and overwrite all prior speed 
  if ( (motor.motorLeftOverload) || (motor.motorRightOverload) || (motor.motorMowOverload) ){
    if (!printmotoroverload) {
        Logger.event(EVT_MOTOR_OVERLOAD_REDUCE_SPEED);
        CONSOLE.println("motor overload detected: reducing linear speed");
    }
    printmotoroverload = true;
    linear = min(linear, MOTOR_OVERLOAD_SPEED);  
  } else {
    printmotoroverload = false;
  }
  
  return linear;
}

// Calculate angular speed using Stanley controller
float calculateStanleyControl(float trackerDiffDelta, float lateralError, bool trackslow_allowed) {
  float k = stanleyTrackingNormalK;
  float p = stanleyTrackingNormalP;    
  if (maps.trackSlow && trackslow_allowed) {
    k = stanleyTrackingSlowK;   
    p = stanleyTrackingSlowP;          
  }
  return p * trackerDiffDelta + atan2(k * lateralError, (0.001 + fabs(motor.linearSpeedSet)));
}

// control robot velocity (linear,angular) to track line to next waypoint (target)
// uses a stanley controller for line tracking
// https://medium.com/@dingyan7361/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081
void trackLine(bool runControl){  
  Point target = maps.targetPoint;
  Point lastTarget = maps.lastTargetPoint;
  float linear = 1.0;  
  bool mow = true;
  if (stateOp == OP_DOCK) mow = false;
  float angular = 0;      
  float targetDelta = pointsAngle(stateX, stateY, target.x(), target.y());      
  if (maps.trackReverse) targetDelta = scalePI(targetDelta + PI);  
  targetDelta = scalePIangles(targetDelta, stateDelta);
  trackerDiffDelta = distancePI(stateDelta, targetDelta);                         
  lateralError = distanceLineInfinite(stateX, stateY, lastTarget.x(), lastTarget.y(), target.x(), target.y());        
  float distToPath = distanceLine(stateX, stateY, lastTarget.x(), lastTarget.y(), target.x(), target.y());        

  float lineDist = maps.distanceToTargetPoint(lastTarget.x(), lastTarget.y());

  float targetDist = maps.distanceToTargetPoint(stateX, stateY);
  
  float lastTargetDist = maps.distanceToLastTargetPoint(stateX, stateY);  
  if (SMOOTH_CURVES)
    targetReached = (targetDist < SMOOTH_CURVE_TARGET_TOLERANCE);    
  else 
    targetReached = (targetDist < TARGET_REACHED_TOLERANCE);

  
  // allow rotations only near last or next waypoint or if too far away from path
  // it might race between rotating mower and targetDist check below
  // if we race we still have rotateLeft or rotateRight true
  if ( (targetDist < ROTATION_DISTANCE_THRESHOLD) || (lastTargetDist < ROTATION_DISTANCE_THRESHOLD) || (fabs(distToPath) > ROTATION_DISTANCE_THRESHOLD) ||
       rotateLeft || rotateRight ) {
    if (SMOOTH_CURVES)
      angleToTargetFits = (fabs(trackerDiffDelta)/PI*180.0 < 120);
    else     
      angleToTargetFits = (fabs(trackerDiffDelta)/PI*180.0 < 20);
  } else {
    // while tracking the mowing line do allow rotations if angle to target increases (e.g. due to gps jumps)
    angleToTargetFits = (fabs(trackerDiffDelta)/PI*180.0 < 45);       
    //angleToTargetFits = true;
  }

  //if (!angleToTargetFits) CONSOLE.println("!angleToTargetFits");

  if (!angleToTargetFits){
    // angular control (if angle to far away, rotate to next waypoint)
    linear = 0;
    angular = ROTATION_ANGULAR_SPEED;               
     // decide for one rotation direction (and keep it)
    if ((!rotateLeft) && (!rotateRight)) {
      if (trackerDiffDelta < 0) rotateLeft = true;
        else rotateRight = true;      
    }
    if (rotateLeft) angular *= -1;
    if (fabs(trackerDiffDelta)/PI*180.0 < ROTATION_ANGLE_RESET_THRESHOLD){
      rotateLeft = false;  // reset rotate direction
      rotateRight = false;
    }   
  } 
  else {
    // line control (stanley)    
    bool straight = maps.nextPointIsStraight();
    bool trackslow_allowed = true;

    rotateLeft = false;
    rotateRight = false;

    // in case of docking or undocking - check if trackslow is allowed
    if ( maps.isUndocking() || maps.isDocking() ) {
        float dockX = 0;
        float dockY = 0;
        float dockDelta = 0;
        maps.getDockingPos(dockX, dockY, dockDelta);
        float dist_dock = distance(dockX, dockY, stateX, stateY);
        // only allow trackslow if we are near dock (below DOCK_UNDOCK_TRACKSLOW_DISTANCE)
        if (dist_dock > DOCK_UNDOCK_TRACKSLOW_DISTANCE) {
            trackslow_allowed = false;
        }
    }

    linear = calculateLinearSpeed(setSpeed, straight, mow, trackslow_allowed);   
          
    angular = calculateStanleyControl(trackerDiffDelta, lateralError, trackslow_allowed);           

    //CONSOLE.print(lateralError);        
    //CONSOLE.print(",");        
    //CONSOLE.println(angular/PI*180.0);            
    if (maps.trackReverse) linear *= -1;   // reverse line tracking needs negative speed
    // restrict steering angle for stanley  (not required anymore after last state estimation bugfix)
    //if (!SMOOTH_CURVES) angular = max(-PI/16, min(PI/16, angular)); 
  }
  // check some pre-conditions that can make linear+angular speed zero
  if ((stateLocalizationMode == LOC_GPS) && (fixTimeout != 0)){
    if (millis() > lastFixTime + fixTimeout * 1000.0){
      activeOp->onGpsFixTimeout();        
    }           
  }     

  if (stateLocalizationMode == LOC_GPS){
    if  ((gps.solution == SOL_FIXED) || (gps.solution == SOL_FLOAT)){        
      if (abs(linear) > 0.06) {
        if ((millis() > linearMotionStartTime + LINEAR_MOTION_TIMEOUT) && (stateGroundSpeed < GROUND_SPEED_THRESHOLD)){
          // if in linear motion and not enough ground speed => obstacle
          //if ( (GPS_SPEED_DETECTION) && (!maps.isUndocking()) ) { 
          if (GPS_SPEED_DETECTION) {         
            CONSOLE.println("gps no speed => obstacle!");
            statMowGPSNoSpeedCounter++;
            Logger.event(EVT_NO_GPS_SPEED_OBSTACLE);
            triggerObstacle();
            return;
          }
        }
      }  
    } else {
      // no gps solution
      if (REQUIRE_VALID_GPS){
        CONSOLE.println("WARN: no gps solution!");
        activeOp->onGpsNoSignal();
      }
    }
  }
  if (stateLocalizationMode == LOC_APRIL_TAG){
    if (!stateAprilTagFound){
      linear = 0; // wait until april-tag found 
      angular = 0; 
    } else {
      if (!buzzer.isPlaying()) buzzer.sound(SND_WARNING, true);
      //linear = 0; // wait until april-tag found 
      //angular = 0; 
    }
  }
  if (stateLocalizationMode == LOC_REFLECTOR_TAG){
    if (!stateReflectorTagFound){
      linear = 0; // wait until reflector-tag found 
      angular = 0; 
    } else {
      if (!buzzer.isPlaying()) buzzer.sound(SND_WARNING, true);
      float maxAngular = REFLECTOR_TAG_MAX_ANGULAR;
      float maxLinear = REFLECTOR_TAG_MAX_LINEAR;      
      angular =  max(min(1.0 * trackerDiffDelta, maxAngular), -maxAngular);
      angular =  max(min(angular, maxAngular), -maxAngular);      
      linear = REFLECTOR_TAG_MAX_LINEAR;      
      if (maps.trackReverse) linear = -REFLECTOR_TAG_MAX_LINEAR;   // reverse line tracking needs negative speed           
    }
  }
  if (stateLocalizationMode == LOC_GUIDANCE_SHEET){
      if (!buzzer.isPlaying()) buzzer.sound(SND_WARNING, true);
      angular = 0;
  }

  // gps-jump/false fix check
  if (KIDNAP_DETECT){
    float allowedPathTolerance = KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE;     
    if ( maps.isUndocking() || maps.isDocking() ) {
        float dockX = 0;
        float dockY = 0;
        float dockDelta = 0;
        maps.getDockingPos(dockX, dockY, dockDelta);
        float dist = distance(dockX, dockY, stateX, stateY);
        // check if current distance to docking station is below
        // KIDNAP_DETECT_DISTANCE_DOCK_UNDOCK to trigger KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE_DOCK_UNDOCK
        if (dist < KIDNAP_DETECT_DISTANCE_DOCK_UNDOCK) {
            allowedPathTolerance = KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE_DOCK_UNDOCK;
        }
    }    
    if ((stateLocalizationMode == LOC_GPS) && (fabs(distToPath) > allowedPathTolerance)){ // actually, this should not happen (except on false GPS fixes or robot being kidnapped...)
      if (!stateKidnapped){
        stateKidnapped = true;
        CONSOLE.print("KIDNAP_DETECT: stateKidnapped=");
        CONSOLE.print(stateKidnapped);
        CONSOLE.print(" distToPath=");
        CONSOLE.println(distToPath);
        activeOp->onKidnapped(stateKidnapped);
      }            
    } else {
      if (stateKidnapped) {
        stateKidnapped = false;
        CONSOLE.print("KIDNAP_DETECT: stateKidnapped=");
        CONSOLE.print(stateKidnapped);
        CONSOLE.print(" distToPath=");
        CONSOLE.println(distToPath);
        activeOp->onKidnapped(stateKidnapped);        
      }
    }
  }
   
  // in any case, turn off mower motor if lifted 
  // also, if lifted, do not turn on mowing motor so that the robot will drive and can do obstacle avoidance 
  if (detectLift()) mow = false;
  
  if (mow)  { 
    if (millis() < motor.motorMowSpinUpTime + MOW_MOTOR_SPINUP_TIMEOUT){
       // wait until mowing motor is running
      if (!buzzer.isPlaying()) buzzer.sound(SND_WARNING, true);
      linear = 0;
      angular = 0;          
    }
  }

  if (runControl){
    if (angleToTargetFits != langleToTargetFits) {
        //CONSOLE.print("angleToTargetFits: ");
        //CONSOLE.print(angleToTargetFits);
        //CONSOLE.print(" trackerDiffDelta: ");
        //CONSOLE.println(trackerDiffDelta);
        langleToTargetFits = angleToTargetFits;
    }

    #ifdef DOCK_REFLECTOR_TAG
      if (stateLocalizationMode == LOC_REFLECTOR_TAG){             
        CONSOLE.print("loc=");
        if (stateLocalizationMode == LOC_APRIL_TAG) CONSOLE.print("april");
        if (stateLocalizationMode == LOC_GPS) CONSOLE.print("gps");
        if (stateLocalizationMode == LOC_GUIDANCE_SHEET) CONSOLE.print("guide");    
        if (stateLocalizationMode == LOC_REFLECTOR_TAG) CONSOLE.print("reflector");        
        CONSOLE.print(" tagFound=");
        CONSOLE.print(stateReflectorTagFound);
        CONSOLE.print(" tagOut=");
        CONSOLE.print(stateReflectorTagOutsideFound);      
        CONSOLE.print(" reflX=");
        CONSOLE.print(stateXReflectorTag);
        CONSOLE.print(" reflY=");
        CONSOLE.print(stateYReflectorTag);
        CONSOLE.print(" mow=");
        CONSOLE.print(mow);      
        CONSOLE.print(" shouldDock=");
        CONSOLE.print(maps.shouldDock);      
        CONSOLE.print(" trackRev=");
        CONSOLE.print(maps.trackReverse);
        CONSOLE.print(" lin=");
        CONSOLE.print(linear);
        CONSOLE.print(" ang=");
        CONSOLE.print(angular);    
        CONSOLE.print(" isBetwLNTLDockPt=");
        CONSOLE.print(maps.isBetweenLastAndNextToLastDockPoint());
        CONSOLE.print(" dockPtIdx=");
        CONSOLE.print(maps.dockPointsIdx);
        CONSOLE.print(" freePtIdx=");
        CONSOLE.print(maps.freePointsIdx);
        CONSOLE.print(" wayMode=");
        if (maps.wayMode == WAY_DOCK) CONSOLE.print("WAY_DOCK");
        if (maps.wayMode == WAY_MOW) CONSOLE.print("WAY_MOW");
        if (maps.wayMode == WAY_FREE) CONSOLE.print("WAY_FREE");
        CONSOLE.println();
      }
    #endif

    motor.setLinearAngularSpeed(linear, angular);      
    motor.setMowState(mow);    
  }

  //if (!maps.isTargetingLastDockPoint()){
  if (stateLocalizationMode != LOC_REFLECTOR_TAG){
    if (targetReached){
      rotateLeft = false;
      rotateRight = false;
      activeOp->onTargetReached();
      bool straight = maps.nextPointIsStraight();
      if (!maps.nextPoint(false,stateX,stateY)){
        // finish        
        activeOp->onNoFurtherWaypoints();      
      } else {      
        // next waypoint          
        //if (!straight) angleToTargetFits = false;      
      }
    }
  }  
}



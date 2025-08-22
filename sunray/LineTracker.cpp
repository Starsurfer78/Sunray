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

// Line tracking constants
const float SMOOTH_CURVES_TARGET_DISTANCE = 0.2;
const float NORMAL_TARGET_DISTANCE = TARGET_REACHED_TOLERANCE;
const float ROTATION_ANGULAR_SPEED = 29.0 / 180.0 * PI; // 29 degree/s (0.5 rad/s)
const float ROTATION_ANGLE_THRESHOLD = 90.0;
const float APPROACH_DISTANCE_THRESHOLD = 0.5;
const float APPROACH_SPEED = 0.1;
const float SLOW_SPEED_NEAR_OBSTACLES = 0.1;
const float FLOAT_SOLUTION_MAX_SPEED = 0.1;
const float MINIMUM_SPEED_THRESHOLD = 0.06;
const unsigned long LINEAR_MOTION_TIMEOUT = 5000;
const float MINIMUM_GROUND_SPEED = 0.03;
const float REFLECTOR_TAG_MAX_ANGULAR = 0.015;
const float REFLECTOR_TAG_MAX_LINEAR = 0.05;
const float REFLECTOR_TAG_LINEAR_SPEED = 0.05;
const float STANLEY_CONTROL_MIN_SPEED = 0.001;



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
bool hasLoggedMotorOverload = false;
float lastLineDist = 0;

// Calculate basic tracking parameters (target angles, distances, errors)
struct TrackingParams {
  Point target;
  Point lastTarget;
  float targetDelta;
  float targetDist;
  float lastTargetDist;
  float distToPath;
};

TrackingParams calculateTrackingParams() {
  TrackingParams params;
  params.target = maps.targetPoint;
  params.lastTarget = maps.lastTargetPoint;
  
  params.targetDelta = pointsAngle(stateX, stateY, params.target.x(), params.target.y());      
  if (maps.trackReverse) params.targetDelta = scalePI(params.targetDelta + PI);  
  params.targetDelta = scalePIangles(params.targetDelta, stateDelta);
  trackerDiffDelta = distancePI(stateDelta, params.targetDelta);                         
  lateralError = distanceLineInfinite(stateX, stateY, params.lastTarget.x(), params.lastTarget.y(), params.target.x(), params.target.y());        
  params.distToPath = distanceLine(stateX, stateY, params.lastTarget.x(), params.lastTarget.y(), params.target.x(), params.target.y());        

  params.targetDist = maps.distanceToTargetPoint(stateX, stateY);
  params.lastTargetDist = maps.distanceToLastTargetPoint(stateX, stateY);  
  
  if (SMOOTH_CURVES)
    targetReached = (params.targetDist < SMOOTH_CURVES_TARGET_DISTANCE);    
  else 
    targetReached = (params.targetDist < NORMAL_TARGET_DISTANCE);
    
  return params;
}

// Calculate angle to target and determine if rotation is needed
bool calculateAngleToTargetFits(const TrackingParams& params) {
  // allow rotations only near last or next waypoint or if too far away from path
  // it might race between rotating mower and targetDist check below
  // if we race we still have rotateLeft or rotateRight true
  if ( (params.targetDist < APPROACH_DISTANCE_THRESHOLD) || (params.lastTargetDist < APPROACH_DISTANCE_THRESHOLD) || (fabs(params.distToPath) > APPROACH_DISTANCE_THRESHOLD) ||
       rotateLeft || rotateRight ) {
    if (SMOOTH_CURVES)
      return (fabs(trackerDiffDelta)/PI*180.0 < 120);
    else     
      return (fabs(trackerDiffDelta)/PI*180.0 < 20);
  } else {
    // while tracking the mowing line do allow rotations if angle to target increases (e.g. due to gps jumps)
    return (fabs(trackerDiffDelta)/PI*180.0 < 45);       
  }
}

// Handle rotation control when angle to target doesn't fit
void handleRotationControl(float& linear, float& angular) {
  // angular control (if angle to far away, rotate to next waypoint)
  linear = 0;
  angular = ROTATION_ANGULAR_SPEED; //  29 degree/s (0.5 rad/s);               
   // decide for one rotation direction (and keep it)
  if ((!rotateLeft) && (!rotateRight)) {
    if (trackerDiffDelta < 0) rotateLeft = true;
      else rotateRight = true;      
  }
  if (rotateLeft) angular *= -1;
  if (fabs(trackerDiffDelta)/PI*180.0 < ROTATION_ANGLE_THRESHOLD){
    rotateLeft = false;  // reset rotate direction
    rotateRight = false;
  }   
}

// Calculate linear speed based on various conditions
float calculateLinearSpeed() {
  bool straight = maps.nextPointIsStraight();
  bool trackslow_allowed = true;
  float linear = setSpeed;

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

  if (maps.trackSlow && trackslow_allowed) {
    // planner forces slow tracking (e.g. docking etc)
    linear = DOCK_LINEAR_SPEED; // 0.1           
  } else if (     ((setSpeed > 0.2) && (maps.distanceToTargetPoint(stateX, stateY) < 0.5) && (!straight))   // approaching
        || ((linearMotionStartTime != 0) && (millis() < linearMotionStartTime + 3000))                      // leaving  
     ) 
  {
    linear = APPROACH_SPEED; // reduce speed when approaching/leaving waypoints          
  } 
  else {
    if ((stateLocalizationMode == LOC_GPS) && (gps.solution == SOL_FLOAT)){        
      linear = min(setSpeed, FLOAT_SOLUTION_MAX_SPEED); // reduce speed for float solution
    } else
      linear = setSpeed;         // desired speed
    if (bumperDriver.nearObstacle()){
      linear = SLOW_SPEED_NEAR_OBSTACLES;  // slow down near obstacles 
    }
    if (lidarBumper.nearObstacle()){
      linear = SLOW_SPEED_NEAR_OBSTACLES;  // slow down near obstacles 
    }
    if (sonar.nearObstacle()) {
      linear = SLOW_SPEED_NEAR_OBSTACLES; // slow down near obstacles
    }
  }      
  // slow down speed in case of overload and overwrite all prior speed 
  if ( (motor.motorLeftOverload) || (motor.motorRightOverload) || (motor.motorMowOverload) ){
    if (!hasLoggedMotorOverload) {
        Logger.event(EVT_MOTOR_OVERLOAD_REDUCE_SPEED);
        CONSOLE.println("motor overload detected: reducing linear speed");
    }
    hasLoggedMotorOverload = true;
    linear = min(linear, MOTOR_OVERLOAD_SPEED);  
  } else {
    hasLoggedMotorOverload = false;
  }   
  
  if (maps.trackReverse) linear *= -1;   // reverse line tracking needs negative speed
  return linear;
}

// Calculate angular speed using Stanley controller
float calculateAngularSpeed() {
  bool trackslow_allowed = true;
  
  // Check if trackslow is allowed for docking/undocking
  if ( maps.isUndocking() || maps.isDocking() ) {
      float dockX = 0;
      float dockY = 0;
      float dockDelta = 0;
      maps.getDockingPos(dockX, dockY, dockDelta);
      float dist_dock = distance(dockX, dockY, stateX, stateY);
      if (dist_dock > DOCK_UNDOCK_TRACKSLOW_DISTANCE) {
          trackslow_allowed = false;
      }
  }
  
  float k = stanleyTrackingNormalK;
  float p = stanleyTrackingNormalP;    
  if (maps.trackSlow && trackslow_allowed) {
    k = stanleyTrackingSlowK;
    p = stanleyTrackingSlowP;          
  }
  return p * trackerDiffDelta + atan2(k * lateralError, (STANLEY_CONTROL_MIN_SPEED + fabs(motor.linearSpeedSet)));
}

// control robot velocity (linear,angular) to track line to next waypoint (target)
// uses a stanley controller for line tracking
// https://medium.com/@dingyan7361/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081
void trackLine(bool runControl){  
  TrackingParams params = calculateTrackingParams();
  float linear = 1.0;  
  bool mow = true;
  if (stateOp == OP_DOCK) mow = false;
  float angular = 0;

  angleToTargetFits = calculateAngleToTargetFits(params);
  //if (!angleToTargetFits) CONSOLE.println("!angleToTargetFits");

  if (!angleToTargetFits){
     handleRotationControl(linear, angular);
  } 
  else {
    // line control (stanley)    
    rotateLeft = false;
    rotateRight = false;

    linear = calculateLinearSpeed();
    angular = calculateAngularSpeed();
  }
  // check some pre-conditions that can make linear+angular speed zero
  if ((stateLocalizationMode == LOC_GPS) && (fixTimeout != 0)){
    if (millis() > lastFixTime + fixTimeout * 1000.0){
      activeOp->onGpsFixTimeout();        
    }           
  }     

  if (stateLocalizationMode == LOC_GPS){
    if  ((gps.solution == SOL_FIXED) || (gps.solution == SOL_FLOAT)){        
      if (abs(linear) > MINIMUM_SPEED_THRESHOLD) {
        if ((millis() > linearMotionStartTime + LINEAR_MOTION_TIMEOUT) && (stateGroundSpeed < MINIMUM_GROUND_SPEED)){
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
      float maxAngular = REFLECTOR_TAG_MAX_ANGULAR;  // 0.02
      float maxLinear = REFLECTOR_TAG_MAX_LINEAR;      
      angular =  max(min(1.0 * trackerDiffDelta, maxAngular), -maxAngular);
      angular =  max(min(angular, maxAngular), -maxAngular);      
      linear = REFLECTOR_TAG_LINEAR_SPEED;      
      if (maps.trackReverse) linear = -REFLECTOR_TAG_LINEAR_SPEED;   // reverse line tracking needs negative speed           
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
    if ((stateLocalizationMode == LOC_GPS) && (fabs(params.distToPath) > allowedPathTolerance)){ // actually, this should not happen (except on false GPS fixes or robot being kidnapped...)
      if (!stateKidnapped){
        stateKidnapped = true;
        CONSOLE.print("KIDNAP_DETECT: stateKidnapped=");
        CONSOLE.print(stateKidnapped);
        CONSOLE.print(" distToPath=");
        CONSOLE.println(params.distToPath);
        activeOp->onKidnapped(stateKidnapped);
      }            
    } else {
      if (stateKidnapped) {
        stateKidnapped = false;
        CONSOLE.print("KIDNAP_DETECT: stateKidnapped=");
        CONSOLE.print(stateKidnapped);
        CONSOLE.print(" distToPath=");
        CONSOLE.println(params.distToPath);
        activeOp->onKidnapped(stateKidnapped);        
      }
    }
  }
   
  // in any case, turn off mower motor if lifted 
  // also, if lifted, do not turn on mowing motor so that the robot will drive and can do obstacle avoidance 
  if (detectLift()) mow = false;
  
  if (mow)  { 
    if (millis() < motor.motorMowSpinUpTime + 10000){
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



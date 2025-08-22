// Ardumower Sunray 


#include "config.h"
#include "rcmodel.h"
#include "robot.h"


volatile unsigned long PPM_start_lin = 0;
volatile unsigned long PPM_end_lin = 0;                
volatile unsigned long PPM_start_ang = 0;
volatile unsigned long PPM_end_ang = 0 ;        
      
#ifdef RCMODEL_ENABLE      

void get_lin_PPM()                                                        // Interrupt Service Routine
{
  if (digitalRead(pinRemoteMow)==HIGH) PPM_start_lin = micros();  
  else                                   PPM_end_lin = micros();    
}

void get_ang_PPM()                                                        // Interrupt Service Routine
{
  if (digitalRead(pinRemoteSteer)==HIGH) PPM_start_ang = micros();  
  else                                   PPM_end_ang = micros();  
}

#endif


void RCModel::begin(){  
#ifdef RCMODEL_ENABLE
  CONSOLE.println("RCModel enabled in config");  
  lin_PPM = 0;                                            
  linearPPM = 0;                                         
  ang_PPM = 0;                                            
  angularPPM = 0;                                         
  RC_Mode = false; 
  nextControlTime = 0;
  // R/C
  pinMode(pinRemoteSteer, INPUT);
  pinMode(pinRemoteMow, INPUT); 
  #ifdef RC_DEBUG
    nextOutputTime = millis() + 1000;
  #endif
#else
  CONSOLE.println("RCModel disabled in config");  
#endif
} 

void RCModel::run(){
#ifdef RCMODEL_ENABLE
  unsigned long t = millis();
  if (!RCMODEL_ENABLE) return;
  if (t < nextControlTime) return;
  nextControlTime = t + 100;                                       // save CPU resources by running at 10 Hz
  
  if (stateButton == 3){                                           // 3 button beeps
      stateButton = 0;                                             // reset button state
      RC_Mode = !RC_Mode;                                                   // R/C-Mode toggle
      if (RC_Mode)  {                                                       // R/C-Mode is active
        CONSOLE.println("button mode 3 - RC Mode ON");
        buzzer.sound(SND_ERROR, true);                                      // 3x beep for R/C active        
        attachInterrupt(digitalPinToInterrupt(pinRemoteMow), get_lin_PPM, CHANGE);// Enable interrupt
        attachInterrupt(digitalPinToInterrupt(pinRemoteSteer), get_ang_PPM, CHANGE);// Enable interrupt 
      }
      if (!RC_Mode) {                 
        CONSOLE.println("button mode 3 - RC Mode OFF");                                      // R/C-Mode inactive
        buzzer.sound(SND_WARNING, true);                          // 2x long beep for R/C off
        motor.setLinearAngularSpeed(0, 0);                                 
        detachInterrupt(digitalPinToInterrupt(pinRemoteMow));             // Disable interrupt
        detachInterrupt(digitalPinToInterrupt(pinRemoteSteer));             // Disable interrupt
      }    
  }
  
  if (RC_Mode)    {       
    lin_PPM = 0;
    if (PPM_start_lin < PPM_end_lin) lin_PPM = PPM_end_lin - PPM_start_lin; 
    if (lin_PPM < 2000 && lin_PPM > 1000)   {                               // Value within 1100 to 1900µsec
      float value_l = (lin_PPM - 1500) / 1500;                                // PPM to range +0.30 to -0.30
      if ((value_l < 0.05) && (value_l > -0.05)) value_l = 0;                 // Enlarge neutral position         
      linearPPM = value_l;                                                    // Pass to debug
    }

    ang_PPM = 0;
    if (PPM_start_ang < PPM_end_ang) ang_PPM = PPM_end_ang - PPM_start_ang; 
    if (ang_PPM < 2000 && ang_PPM > 1000)   {                               // Value within 1100 to 1900µsec
      float value_a = (ang_PPM - 1500) / 950;                                 // PPM to range +0.50 to -0.50
      if ((value_a < 0.05) && (value_a > -0.05)) value_a = 0;                 // Enlarge neutral position         
      angularPPM = value_a;                                                   // Pass to debug
    }

#ifdef RC_DEBUG
    if (t >= nextOutputTime) {
      nextOutputTime = t + 1000;

      CONSOLE.print("RC: linearPPM= ");
      CONSOLE.print(linearPPM);
      CONSOLE.print("  angularPPM= ");
      CONSOLE.println(angularPPM);
    }
#endif
    motor.setLinearAngularSpeed(linearPPM, angularPPM, false);                     // R/C Signale an Motor leiten
  }
#endif
}



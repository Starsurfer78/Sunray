/*  
   How to find out P,I,D:
    1. Increase P until system starts to oscillate
    2. Set I =0.6 * P and D = 0.125 * P 
   
*/

#include "pid.h"
#include "config.h"

// Time conversion constants
static const float MILLIS_TO_SECONDS = 1000.0;
static const float MICROS_TO_SECONDS = 1000000.0;
static const float MAX_VELOCITY_PID_TA = 1.0;

PID::PID()
{
  Kp = 0.0;
  Ki = 0.0;
  Kd = 0.0;
  TaMax = 0.1;
  Ta = 0.0;
  w = 0.0;
  x = 0.0;
  esum = 0.0;
  eold = 0.0;
  y = 0.0;
  yold = 0.0;
  y_min = -255.0;
  y_max = 255.0;
  max_output = 255.0;
  output_ramp = 0.0;
  lastControlTime = 0;
  consoleWarnTimeout = 0;
}
    
PID::PID(float Kp, float Ki, float Kd)
{
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  TaMax = 0.1;
  Ta = 0.0;
  w = 0.0;
  x = 0.0;
  esum = 0.0;
  eold = 0.0;
  y = 0.0;
  yold = 0.0;
  y_min = -255.0;
  y_max = 255.0;
  max_output = 255.0;
  output_ramp = 0.0;
  lastControlTime = 0;
  consoleWarnTimeout = 0;
}


void PID::reset(void) {
  eold = 0;
  esum = 0;
  yold = 0;
  lastControlTime = millis();
}

float PID::compute() {
  unsigned long now = millis();
  Ta = calculateSampleTime(now, lastControlTime, TaMax);
  //printf("%.3f\n", Ta);

  // compute error
  float e = (w - x);
  // integrate error
  esum += e;
  // anti wind-up
  if (esum < -max_output)  esum = -max_output;
  if (esum > max_output)  esum = max_output;
  y = Kp * e
      + Ki * Ta * esum
      + Kd / Ta * (e - eold);
  eold = e;
  // restrict output to min/max
  applyOutputClamping(y, y_min, y_max);

  // if output ramp defined
  applyOutputRamping(y, yold, Ta, output_ramp);

  yold = y;  

  return y;
}

void VelocityPID::applyOutputClamping(int& output, int min_val, int max_val) {
  if (output > max_val) output = max_val;
  if (output < min_val) output = min_val;
}

void VelocityPID::applyOutputRamping(int& output, int old_output, float Ta, float ramp_rate) {
  if (ramp_rate > 0) {
    float output_rate = (float)(output - old_output) / Ta;
    if (output_rate > ramp_rate)
      output = old_output + (int)(ramp_rate * Ta);
    else if (output_rate < -ramp_rate)
      output = old_output - (int)(ramp_rate * Ta);
  }
}

float VelocityPID::calculateSampleTime(unsigned long current_time, unsigned long& last_time, float max_ta) {
  float ta = ((float)(current_time - last_time)) / MICROS_TO_SECONDS;
  last_time = current_time;
  if (ta > max_ta) ta = max_ta;   // should only happen for the very first call
  return ta;
}

void PID::applyOutputClamping(float& output, float min_val, float max_val) {
  if (output > max_val) output = max_val;
  if (output < min_val) output = min_val;
}

void PID::applyOutputRamping(float& output, float old_output, float Ta, float ramp_rate) {
  if (ramp_rate > 0) {
    float output_rate = (output - old_output) / Ta;
    if (output_rate > ramp_rate)
      output = old_output + ramp_rate * Ta;
    else if (output_rate < -ramp_rate)
      output = old_output - ramp_rate * Ta;
  }
}

float PID::calculateSampleTime(unsigned long current_time, unsigned long& last_time, float max_ta) {
  float ta = ((float)(current_time - last_time)) / MILLIS_TO_SECONDS;
  last_time = current_time;
  if (ta > max_ta) {
    if (millis() > consoleWarnTimeout) {
      consoleWarnTimeout = millis() + 1000;
      CONSOLE.print("WARN: PID unmet cycle time Ta=");
      CONSOLE.print(ta);
      CONSOLE.print(" TaMax=");
      CONSOLE.println(max_ta);
    }
    ta = max_ta;   // should only happen for the very first call
  }
  return ta;
}


// ---------------------------------

VelocityPID::VelocityPID()
{
  Kp = 0.0;
  Ki = 0.0;
  Kd = 0.0;
  Ta = 0.0;
  w = 0.0;
  x = 0.0;
  eold1 = 0.0;
  eold2 = 0.0;
  y = 0;
  yold = 0;
  y_min = -255;
  y_max = 255;
  max_output = 255;
  output_ramp = 0.0;
  lastControlTime = 0;
}
    
VelocityPID::VelocityPID(float Kp, float Ki, float Kd)
{
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  Ta = 0.0;
  w = 0.0;
  x = 0.0;
  eold1 = 0.0;
  eold2 = 0.0;
  y = 0;
  yold = 0;
  y_min = -255;
  y_max = 255;
  max_output = 255;
  output_ramp = 0.0;
  lastControlTime = 0;
}


float VelocityPID::compute()
{
  unsigned long now = micros();
  Ta = calculateSampleTime(now, lastControlTime, MAX_VELOCITY_PID_TA);

  // compute error
  float e = (w - x);

  // compute max/min output
  if (w < 0) {
    y_min = -max_output;
    y_max = 0;
  }
  if (w > 0) {
    y_min = 0;
    y_max = max_output;
  }

  y = yold
      + Kp * (e - eold1)
      + Ki * Ta * e
      + Kd / Ta * (e - 2 * eold1 + eold2);

  // restrict output to min/max
  applyOutputClamping(y, y_min, y_max);

  // if output ramp defined
  applyOutputRamping(y, yold, Ta, output_ramp);

  // save variable for next time
  eold2 = eold1;
  eold1 = e;
  yold = y;

  return y;
}


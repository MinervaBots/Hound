#include "PIDControler.h"


void PIDControler::Init(float Kp, float Kd, float Ki, float bsp)
{
  this->p = Kp;
  this->d = Kd;
  this->i = Ki;
  this->Bsp = bsp;
  lastRun = millis();
}

void PIDControler::setConstants(float Kp, float Kd, float Ki)
{
  
  this->p = Kp;
  this->d = Kd;
  this->i = Ki;
  integral = derivative = proportional = lastError = 0;
  lastRun = millis();
}

float PIDControler::run(float reference, float y)
{
  
  float error = reference - y;
  float dt = (millis () - lastRun) * 0.001;
  float output;
  
  lastRun = millis();
  integral += i * error * dt ;
  derivative = d*(error - lastError)/dt;
  lastError = error;
  
  proportional = p * (Bsp*reference - y);
  output = proportional + integral + derivative;
  return output;
}
void PIDControler::reset(){
  lastError = 0;
  integral = 0;
  lastRun = millis();
}



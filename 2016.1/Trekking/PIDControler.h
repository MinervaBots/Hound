#ifndef CONTROL_H
#define CONTROL_H
#include "Arduino.h"

class PIDControler
{
  public:
    void Init(float Kp, float Kd, float Ki, float bsp);
    void setConstants(float Kp, float Kd, float Ki);
    float run(float reference, float y);
    float integral, derivative, proportional;
    void reset();
  private:
    float p;
    float d, i;
    float Bsp;
    float lastError;
    unsigned long lastRun;
};

#endif
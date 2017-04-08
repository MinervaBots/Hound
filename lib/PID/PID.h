#ifndef PID_H
#define PID_H

#include "../Timer/timer.h"

class PID
{
public:
  PID();
  PID(float setPoint, float proportionalConstant, float integralConstant, float derivativeConstant);
  PID(float setPoint, float proportionalConstant, float integralConstant, float derivativeConstant, unsigned int delay);

  void SetConstants(float proportionalConstant, float integralConstant, float derivativeConstant);
  float Run(float input);
  float Run(float input, float deltaTime);
  float Run(float setPoint, float input, float deltaTime);
  void Reset();

private:
  Timer m_Timer;
  float m_SetPoint;
  float m_ProportionalConstant;
  float m_IntegralConstant;
  float m_DerivativeConstant;

  float m_ErrorSum;
  float m_LastError;
  unsigned long m_LastRun;
  float m_LastOutput;
  unsigned int m_Delay;
};

#endif // PID_H

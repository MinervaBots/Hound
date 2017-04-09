#include "PID.h"


PID::PID(float setPoint, float proportionalConstant, float integralConstant, float derivativeConstant, unsigned int delay) :
  m_SetPoint(setPoint),
  m_ProportionalConstant(proportionalConstant),
  m_IntegralConstant(integralConstant),
  m_DerivativeConstant(derivativeConstant),
  m_ErrorSum(0),
  m_LastError(0),
  m_LastRun(0),
  m_LastOutput(0),
  m_Delay(delay)
{
  m_Timer.start();
}

PID::PID(float setPoint, float proportionalConstant, float integralConstant, float derivativeConstant) :
  PID(setPoint, proportionalConstant, integralConstant, derivativeConstant, 0)
{

}

PID::PID() :
  PID(0, 0, 0, 0)
{

}

void PID::SetConstants(float proportionalConstant, float integralConstant, float derivativeConstant)
{
  m_ProportionalConstant = proportionalConstant;
  m_IntegralConstant = integralConstant;
  m_DerivativeConstant = derivativeConstant;
}

float PID::Run(float input)
{
  unsigned long elapsed = m_Timer.getElapsedTime();
  float deltaTime = (elapsed - m_LastRun);
  m_LastRun = elapsed;
  return Run(input, deltaTime);
}

float PID::Run(float input, float deltaTime)
{
  if(deltaTime < m_Delay)
  {
    return m_LastOutput;
  }
  return Run(m_SetPoint, input, deltaTime);
}


float PID::Run(float setPoint, float input, float deltaTime)
{
  float error = setPoint - input;
	m_ErrorSum += (error * deltaTime);
	float dErr = (error - m_LastError) / deltaTime;

	float output = m_ProportionalConstant * error;	// Proporcional
	output += m_IntegralConstant * m_ErrorSum;		  // Integrativo
	output += m_DerivativeConstant * dErr;			    // Derivativo

	m_LastError = error;
  m_LastOutput = output;
	return output;
}

void PID::Reset()
{
  m_LastError = 0;
  m_ErrorSum = 0;
  m_LastRun = m_Timer.getElapsedTime();
}

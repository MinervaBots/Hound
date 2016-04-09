#include "duodriver.h"

DuoDriver::DuoDriver(byte tx_pin, byte rx_pin, int timeOut, int address):
    roboclaw(rx_pin, tx_pin, timeOut)
{
    input = address;
    setStopPWM(64);
    setMinPWM(0,0);
    setMaxPWM(255,255);
}

void DuoDriver::moveForward(byte percentage)
{
    byte pwm = map(percentage,0,100,0,128);
    roboclaw.ForwardM1(input,pwm);
    roboclaw.ForwardM2(input,pwm);
}

void DuoDriver::moveBackwards(byte percentage)
{
    byte pwm = map(percentage,0,100,0,128);
    roboclaw.BackwardM1(input,pwm);
    roboclaw.BackwardM2(input,pwm);
}

void DuoDriver::setMinPWM(byte r_pwm, byte l_pwm)
{
    r_pwm_min = r_pwm;
    l_pwm_min = l_pwm;

    if(rpwm < r_pwm_min) rpwm = r_pwm_min;
    if(lpwm < l_pwm_min) lpwm = l_pwm_min;
}

void DuoDriver::setMaxPWM(byte r_pwm, byte l_pwm)
{
    r_pwm_max = r_pwm;
    l_pwm_max = l_pwm;

    if(rpwm > r_pwm_max) rpwm = r_pwm_max;
    if(lpwm > l_pwm_max) lpwm = l_pwm_max;
}

void DuoDriver::setRPWM(byte pwm, bool reverse)
{
    if (reverse) rpwm = map(pwm,0,255,64,0);
    else rpwm = map(pwm,0,255,64,128);
    roboclaw.ForwardBackwardM1(input,rpwm);
}

void DuoDriver::setLPWM(byte pwm, bool reverse)
{
    if (reverse) lpwm = map(pwm,0,255,64,0);
    else lpwm = map(pwm,0,255,64,128);
    roboclaw.ForwardBackwardM2(input,lpwm);
}

void DuoDriver::setStopPWM(byte pwm)
{
    pwm_stop = pwm;
}

byte DuoDriver::getRPWM()
{
    return rpwm;
}

byte DuoDriver::getLPWM()
{
    return lpwm;
}

void DuoDriver::stop()
{
	roboclaw.ForwardBackwardM1(input,pwm_stop);
	roboclaw.ForwardBackwardM2(input,pwm_stop);
}

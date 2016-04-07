#include "duodriver.h"

DuoDriver::DuoDriver(byte tx_pin, byte rx_pin, int timeOut, int address):
    roboclaw(rx_pin, tx_pin, timeOut)
{
    input = address;
    setStopPWM(64);
    setMinPWM(0);
    setMaxPWM(255);
}

void DuoDriver::moveForward(byte percentage)
{
    pwm = map(percentage,0,100,0,128);
    roboclaw.ForwardM1(input,pwm);
    roboclaw.ForwardM2(input,pwm);
}

void DuoDriver::moveBackwards(byte percentage)
{
    pwm = map(percentage,0,100,0,128);
    roboclaw.BackwardM1(input,pwm);
    roboclaw.BackwardM2(input,pwm);
}

void DuoDriver::setMinPWM(byte pwm):
{
    pwm_min = pwm;

    if(rpwm < pwm_min){
        rpwm = pwm_min;
    }
    if(lpwm < pwm_min){
        lpwm = pwm_min;
    }
}

DuoDriver::setMaxPWM(byte pwm)
{
    pwm_max = pwm;

    if(rpwm > pwm_max){
        rpwm = pwm_max;
    }
    if(lpwm > pwm_max){
        lpwm = pwm_max;
    }
}

void setRPWM(byte pwm, bool reverse)
{
    if (reverse) rpwm = map(pwm,0,255,64,0);
    else rpwm = map(pwm,0,255,64,128);
    roboclaw.ForwardBackwardM1(input,rpwm);
}

void setLPWM(byte pwm, bool reverse)
{
    if (reverse) lpwm = map(pwm,0,255,64,0);
    else lpwm = map(pwm,0,255,64,128);
    roboclaw.ForwardBackwardM2(input,lpwm);
}

void setStopPWM(byte pwm)
{
    pwm_stop = pwm;
}

void DuoDriver::stop()
{
	roboclaw.ForwardBackwardM1(input,pwm_stop);
	roboclaw.ForwardBackwardM2(input,pwm_stop);
}

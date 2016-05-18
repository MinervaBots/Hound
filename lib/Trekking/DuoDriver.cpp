#include "DuoDriver.h"


DuoDriver::DuoDriver(byte tx_pin, byte rx_pin, int timeOut, int address):
    roboclaw(rx_pin, tx_pin, timeOut)
{
    input = address;
    setStopPWM(64);
    setMinPWM(0,0);
    setCorrection(0, 0);
    setMaxPWM(255,255);
}

void DuoDriver::setAllRightPWM(byte pwm, bool reverse)
{
    // Se a funcao map limita a velocidade, faz mais sentido colocar a correcao
    // antes ou depois? Se ela nao limitar, nao importa onde a colocamos.
    pwm = this->r_pwm_gain*pwm;
    if (reverse) rpwm = map(pwm,0,255,64,0);
    else rpwm = map(pwm,0,255,64,128);
    roboclaw.ForwardBackwardM1(input,rpwm);
}

void DuoDriver::setAllLeftPWM(byte pwm, bool reverse)
{
    // Se a funcao map limita a velocidade, faz mais sentido colocar a correcao
    // antes ou depois? Se ela nao limitar, nao importa onde a colocamos.
    pwm = this->l_pwm_gain*pwm;
    if (reverse) lpwm = map(pwm,0,255,64,0);
    else lpwm = map(pwm,0,255,64,128);
    roboclaw.ForwardBackwardM2(input,lpwm);
}

void DuoDriver::setMinLeftPWM(byte pwm)
{
    l_pwm_min = pwm;
    if(lpwm < l_pwm_min) lpwm = l_pwm_min;
}

void DuoDriver::setMinRightPWM(byte pwm)
{
    r_pwm_min = pwm;
    if(rpwm < r_pwm_min) rpwm = r_pwm_min;
}


void DuoDriver::setMaxLeftPWM(byte pwm)
{
    l_pwm_max = pwm;
    if(lpwm > l_pwm_max) lpwm = l_pwm_max;
}

void DuoDriver::setMaxRightPWM(byte pwm)
{
    r_pwm_max = pwm;
    if(rpwm > r_pwm_max) rpwm = r_pwm_max;
}

void DuoDriver::stopAll()
{
    roboclaw.ForwardBackwardM1(input,pwm_stop);
    roboclaw.ForwardBackwardM2(input,pwm_stop);
}

void DuoDriver::setCorrection(float r_correction,float l_correction)
{
    this->r_pwm_gain = 1 + r_correction;
    this->l_pwm_gain = 1 + l_correction;
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
    setMinLeftPWM(l_pwm);
    setMinRightPWM(r_pwm);
}

void DuoDriver::setMaxPWM(byte r_pwm, byte l_pwm)
{
    setMaxLeftPWM(l_pwm);
    setMaxRightPWM(r_pwm);
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

// colocamos aqui mesmo?
int32_t DuoDriver::getRightEncoder(){
  return roboclaw.ReadSpeedM1(input, &status_right, &valid_right);
}

int32_t DuoDriver::getLeftEncoder(){
  return roboclaw.ReadSpeedM2(input, &status_left, &valid_left);
}

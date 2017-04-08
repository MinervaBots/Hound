#include "DuoDriver.h"

DuoDriver::DuoDriver(byte tx_pin, byte rx_pin, int timeOut, int address):
    roboclaw(rx_pin, tx_pin, timeOut)
{
    input = address;
    last_r_pps = 0;
    last_l_pps = 0;
    setStopPWM(64);
    setMinPWM(0,0);
    setCorrection(0, 0);
    setMaxPWM(255,255);
}

void DuoDriver::setMaxPPS(uint32_t MAX_PPS){
  this->MAX_PPS = MAX_PPS;
}

void DuoDriver::setPID(float kp, float ki, float kd){
  setLeftPID(kp, ki, kd);
  setRightPID(kp, ki, kd);
}

void DuoDriver::setLeftPID(float kp, float ki, float kd){
  kp_left = kp;
	ki_left = ki;
	kd_left = kd;
  l_pid.SetConstants(kp_left, kd_left, ki_left);
}

void DuoDriver::setRightPID(float kp, float ki, float kd){
  kp_right = kp;
	ki_right = ki;
	kd_right = kd;
  r_pid.SetConstants(kp_right, kd_right, ki_right);
}

void DuoDriver::setRightPPS(float pps, float dT){
  float pid_out = r_pid.Run(pps, getRightPPS(), dT);
  roboclaw.ForwardBackwardM1(input,mapPWM(pid_out));
}

void DuoDriver::setLeftPPS(float pps, float dT){
  float pid_out = r_pid.Run(pps, getLeftPPS(), dT);
  roboclaw.ForwardBackwardM2(input,mapPWM(pid_out));
}

void DuoDriver::setRightPPS(float pps){
  roboclaw.ForwardBackwardM1(input,mapPWM(pps));
}

void DuoDriver::setLeftPPS(float pps){
  roboclaw.ForwardBackwardM2(input,mapPWM(pps));
}


uint8_t DuoDriver::mapPWM(float pps){
  uint8_t pwm = 0;
  float speed = pps/MAX_PPS;
  pwm = 64*(1 + speed);
  pwm = max(min(128, pwm), 0);
  return pwm;
}




void DuoDriver::setAllRightPWM(byte pwm, bool reverse)
{
    // Se a funcao map limita a velocidade, faz mais sentido colocar a correcao
    // antes ou depois? Se ela nao limitar, nao importa onde a colocamos.
    pwm = this->r_pwm_gain*pwm;
    if (reverse) rpwm = map(pwm,0,255,64,0);
    else rpwm = map(pwm,0,255,64,128);
    roboclaw.ForwardBackwardM2(input,rpwm);
}

void DuoDriver::setAllLeftPWM(byte pwm, bool reverse)
{
    // Se a funcao map limita a velocidade, faz mais sentido colocar a correcao
    // antes ou depois? Se ela nao limitar, nao importa onde a colocamos.
    pwm = this->l_pwm_gain*pwm;
    if (reverse) lpwm = map(pwm,0,255,64,0);
    else lpwm = map(pwm,0,255,64,128);
    roboclaw.ForwardBackwardM1(input,lpwm);
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

uint32_t DuoDriver::getRightEncoder(){
  return roboclaw.ReadEncM1(input, &status_right, &valid_right);
}

uint32_t DuoDriver::getLeftEncoder(){
  return roboclaw.ReadEncM2(input, &status_left, &valid_left);
}

int DuoDriver::getRightPPS(){
  last_r_pps = roboclaw.ReadSpeedM1(input, &status_right, &valid_right);
  return last_r_pps;
}

int DuoDriver::getLeftPPS(){
  last_l_pps = roboclaw.ReadSpeedM2(input, &status_left, &valid_left);
  return last_l_pps;
}

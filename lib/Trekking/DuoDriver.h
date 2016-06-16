/*
	L298 based driver
*/

#ifndef DUODRIVER_H
#define DUODRIVER_H

// #include <Arduino.h>
#include "DualDriver.h"
// #include "RoboClaw.h"
// #include "../Robot/DualDriver.h"

// #include "BMSerial.h"
#include "PIDControler.h"
#include "../RoboClaw/RoboClaw.h"


class DuoDriver: public DualDriver
{

public:
  DuoDriver(byte tx_pin, byte rx_pin, int timeOut, int address);  // I am not sure the address is necessarily of the type int
  void setMaxPPS(uint32_t MAX_PPS);

  void setAllRightPWM(byte pwm, bool reverse = false);
	void setAllLeftPWM(byte pwm, bool reverse = false);

	void setMinLeftPWM(byte pwm);
	void setMinRightPWM(byte pwm);

	void setMaxLeftPWM(byte pwm);
	void setMaxRightPWM(byte pwm);

	void setMinPWM(byte r_pwm, byte l_pwm);
	void setMaxPWM(byte r_pwm, byte l_pwm);

  void stopAll();
  void setCorrection(float r_correction,float l_correction);

	void moveForward(byte percentage);
	void moveBackwards(byte percentage);

	void setStopPWM(byte pwm);
	byte getRPWM();
	byte getLPWM();

  void setPID(float kp, float ki, float kd, uint32_t max_pps);
  void setLeftPID(float kp, float ki, float kd, uint32_t MAX_PPS);
  void setRightPID(float kp, float ki, float kd, uint32_t MAX_PPS);

  void setRightPPS(float pps, float dT);
  void setRightPPS(float pps);

  void setLeftPPS(float pps, float dT);
  void setLeftPPS(float pps);

  uint8_t mapPWM(float pps);


  uint32_t getLeftEncoder();
  uint32_t getRightEncoder();

  uint32_t getLeftPPS();
  uint32_t getRightPPS();

	RoboClaw* roboclaw;
  // BMSerial terminal;

protected:
    byte pwm_stop;
    byte r_pwm_min, l_pwm_min;
    byte r_pwm_max, l_pwm_max;
    float r_pwm_gain, l_pwm_gain;

    float kp_right, ki_right, kd_right;
  	float kp_left, ki_left, kd_left;

    int input;
    byte rpwm;
    byte lpwm;

    uint32_t MAX_PPS;
    uint8_t status_right, status_left;
    uint32_t last_l_pps, last_r_pps;
    bool valid_right, valid_left;

    PIDControler r_pid;
  	PIDControler l_pid;
};

#endif

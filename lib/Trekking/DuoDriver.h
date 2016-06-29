#ifndef DUODRIVER_H
#define DUORDRIVER_H

// #include <Arduino.h>
#include "../Robot/DualDriver.h"
#include "../RoboClaw/RoboClaw.h"
#include "../BMSerial/BMSerial.h"

#include "PIDControler.h"


class DuoDriver: public DualDriver
{

public:
  DuoDriver(byte tx_pin, byte rx_pin, int timeOut, int address);

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

  uint32_t getLeftEncoder();
  uint32_t getRightEncoder();

  int getLeftPPS();
  int getRightPPS();

  void setMaxPPS(uint32_t MAX_PPS);

  void setPID(float kp, float ki, float kd);
  void setLeftPID(float kp, float ki, float kd);
  void setRightPID(float kp, float ki, float kd);

  void setRightPPS(float pps, float dT);
  void setRightPPS(float pps);

  void setLeftPPS(float pps, float dT);
  void setLeftPPS(float pps);

  uint8_t mapPWM(float pps);


	RoboClaw roboclaw;

protected:
    byte pwm_stop;
    byte r_pwm_min, l_pwm_min;
    byte r_pwm_max, l_pwm_max;
    float r_pwm_gain, l_pwm_gain;

    int input;
    byte rpwm;
    byte lpwm;

    uint8_t status_right, status_left;
    bool valid_right, valid_left;

    float kp_right, ki_right, kd_right;
    float kp_left, ki_left, kd_left;

    uint32_t MAX_PPS;
    int last_l_pps, last_r_pps;

    PIDControler r_pid;
  	PIDControler l_pid;
};

#endif

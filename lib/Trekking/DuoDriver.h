/*
	L298 based driver
*/

#ifndef DUODRIVER_H
#define DUORDRIVER_H

// #include <Arduino.h>
#include "../Robot/DualDriver.h"
#include "../RoboClaw/RoboClaw.h"
#include "../BMSerial/BMSerial.h"



class DuoDriver: public DualDriver
{

public:
  DuoDriver(byte tx_pin, byte rx_pin, int timeOut, int address);  // I am not sure the address is necessarily of the type int

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

  int32_t getLeftEncoder();
  int32_t getRightEncoder();

  int32_t getLeftPPS();
  int32_t getRightPPS();

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
};

#endif

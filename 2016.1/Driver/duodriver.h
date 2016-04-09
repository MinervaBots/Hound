/*
	L298 based driver
*/

#ifndef DUODRIVER_H
#define DUORDRIVER_H

#include <Arduino.h>
#include "RoboClaw.h"
#include "BMSerial.h"


class DuoDriver
{

public:
    DuoDriver(byte tx_pin, byte rx_pin, int timeOut, int address);  // I am not sure the address is necessarily of the type int
	void moveForward(byte percentage);
	void moveBackwards(byte percentage);
	void setMinPWM(byte r_pwm, byte l_pwm);
	void setMaxPWM(byte r_pwm, byte l_pwm);
	void setRPWM(byte pwm, bool reverse = false);
	void setLPWM(byte pwm, bool reverse = false);
	void setStopPWM(byte pwm);
	byte getRPWM();
	byte getLPWM();
	void stop();



	RoboClaw roboclaw;

protected:
    int input;
    byte rpwm;
    byte lpwm;
    byte r_pwm_min, l_pwm_min;
    byte r_pwm_max, l_pwm_max;
    byte pwm_stop;
};

#endif

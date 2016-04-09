/*
	controlboard.h - class to control 2 DC motors. In a next version it will
	be updated to control N DC motors.

	ControlBoard makes it easy to control motors.

*/

#ifndef CONTROLBOARD_H
#define CONTROLBOARD_H

#include "duodriver.h"


class ControlBoard
{
public:
	ControlBoard(byte tx_pin, byte rx_pin, int timeOut, int address);

    virtual void setRPWM(byte pwm, bool reverse = false);
	virtual void setLPWM(byte pwm, bool reverse = false);
	virtual void setPWM(byte r_pwm, byte l_pwm, bool r_reverse=false, bool l_reverse=false);
	virtual void setMinPWM(byte r_min_pwm, byte l_min_pwm);
	virtual void stop();

	//Driver
	DuoDriver driver;

protected:
    byte r_pwm, l_pwm;
};
#endif

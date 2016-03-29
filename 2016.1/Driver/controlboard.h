/*
	controlboard.h - class to control 2 DC motors. In a next version it will
	be updated to control N DC motors.

	ControlBoard makes it easy to control motors.

*/

#ifndef CONTROLBOARD_H
#define CONTROLBOARD_H

#include "duodriver.h"

#define ONEINPUT

class ControlBoard
{
public:
	#ifdef ONEINPUT
	ControlBoard(r_pin,l_pin);
	#else
	ControlBoard(byte r_enable,
		byte r_motor_1, byte r_motor_2,

		byte l_enable,
		byte l_motor_1, byte l_motor_2,

		byte r_vcc_ref=UNUSED, byte r_gnd_ref=UNUSED,
		byte l_vcc_ref=UNUSED, byte l_gnd_ref=UNUSED);
	#endif // ONEINPUT


	virtual void setRPWM(byte pwm, bool reverse=false);
	virtual void setLPWM(byte pwm, bool reverse=false);
	virtual void setPWM(byte r_pwm, byte l_pwm, bool r_reverse=false, bool l_reverse=false);
	virtual void setMinPWM(byte r_min_pwm, byte l_min_pwm);
	virtual void stop();

protected:
	//Driver
	DuoDriver driver;

	byte r_pwm, l_pwm;
};
#endif

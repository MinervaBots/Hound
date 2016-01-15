#include "controlboard.h"

ControlBoard::ControlBoard(byte r_enable, 
		byte r_motor_1, byte r_motor_2,

		byte l_enable,
		byte l_motor_1, byte l_motor_2,
		
		byte r_vcc_ref, byte r_gnd_ref,
		byte l_vcc_ref, byte l_gnd_ref):

	driver(r_enable, r_motor_1, r_motor_2,
		l_enable, l_motor_1, l_motor_2,
		r_vcc_ref, r_gnd_ref,
		l_vcc_ref, l_gnd_ref)
{
	setPWM(0,0);
	setMinPWM(0,0);
}


void ControlBoard::stop()
{
	driver.stop();
}

void ControlBoard::setRPWM(byte pwm, bool reverse)
{
	if(!reverse)
	{
		driver.r_motor.setPWM(pwm, 0);
	} else {
		driver.r_motor.setPWM(0, pwm);
	}
	r_pwm = pwm;
}

void ControlBoard::setLPWM(byte pwm, bool reverse)
{
	if(!reverse)
	{
		driver.l_motor.setPWM(pwm, 0);
	} else {
		driver.l_motor.setPWM(0, pwm);
	}
	l_pwm = pwm;
}

void ControlBoard::setPWM(byte r_pwm, byte l_pwm, bool r_reverse, bool l_reverse)
{
	/*
	 * Set the pwm manually. Actually, if any automatic method is called,this value
	 * will be the maximum pwm that the motor is subjected to, considering the speed
	 * and the correction variables in a range of 0 to 1. If the speed and the 
	 * correction are the default (1) and the minimum pwm is 0, this method will
	 * truly set the pwm to the automatic mode.
	 */
	setRPWM(r_pwm, r_reverse);
	setLPWM(l_pwm, l_reverse);
}

void ControlBoard::setMinPWM(byte r_min_pwm, byte l_min_pwm)
{
	/*
	 * The minimum pwm is a value used to avoid motor stall
	 * at low pwms. As soon as the pwm is below this value, 
	 * the program will write zero on the motor input
	 */
	driver.r_motor.setMinPWM(r_min_pwm);
	driver.l_motor.setMinPWM(l_min_pwm);
}
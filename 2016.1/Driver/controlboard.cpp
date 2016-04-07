#include "controlboard.h"


ControlBoard::ControlBoard(byte tx_pin, byte rx_pin, int timeOut, int address):
    driver(tx_pin, rx_pin, timeOut, address)
{
    setPWM(0,0);
	setMinPWM(0,0);
}

void ControlBoard::setRPWM(byte pwm, bool reverse)
{
    driver.setRPWM(byte pwm, bool reverse);
}

void ControlBoard::setLPWM(byte pwm, bool reverse)
{
    driver.setLPWM(byte pwm, bool reverse);
}

void ControlBoard::stop()
{
	driver.stop();
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

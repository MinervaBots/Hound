#include "autocontrolboard.h"

AutoControlBoard::AutoControlBoard(byte r_enable, 
		byte r_motor_1, byte r_motor_2,

		byte l_enable,
		byte l_motor_1, byte l_motor_2,
		
		byte r_vcc_ref, byte r_gnd_ref,
		byte l_vcc_ref, byte l_gnd_ref):

	ControlBoard(r_enable, 
		r_motor_1, r_motor_2,

		l_enable,
		l_motor_1, l_motor_2,

		r_vcc_ref, r_gnd_ref,
		l_vcc_ref, l_gnd_ref)
{
	setCorrection(100,100);
	setCurveFactor(3);
	setSpeed(100);
}

//Automatic control
void AutoControlBoard::moveForward()
{
	driver.r_motor.setPWM(r_pwm*r_correction*speed,0);
	driver.l_motor.setPWM(l_pwm*l_correction*speed,0);
}
void AutoControlBoard::moveForwardRight()
{
	driver.r_motor.setPWM(r_pwm*r_correction*speed/curve_factor,0);
	driver.l_motor.setPWM(l_pwm*l_correction*speed,0);
}
void AutoControlBoard::moveForwardLeft()
{
	driver.r_motor.setPWM(r_pwm*r_correction*speed,0);
	driver.l_motor.setPWM(l_pwm*l_correction*speed/curve_factor,0);
}

void AutoControlBoard::moveBackwards()
{
	driver.r_motor.setPWM(0, r_pwm*r_correction*speed);
	driver.l_motor.setPWM(0, l_pwm*l_correction*speed);
}

void AutoControlBoard::moveBackwardsRight()
{
	driver.r_motor.setPWM(0, r_pwm*r_correction*speed/curve_factor);
	driver.l_motor.setPWM(0, l_pwm*l_correction*speed);
}

void AutoControlBoard::moveBackwardsLeft()
{
	driver.r_motor.setPWM(0, r_pwm*r_correction*speed);
	driver.l_motor.setPWM(0, l_pwm*l_correction*speed/curve_factor);
}

void AutoControlBoard::rotateClockwise()
{
	driver.r_motor.setPWM(0,r_pwm*r_correction*speed);
	driver.l_motor.setPWM(l_pwm*l_correction*speed,0);
}

void AutoControlBoard::rotateAntiClockwise()
{
	driver.r_motor.setPWM(r_pwm*r_correction*speed,0);
	driver.l_motor.setPWM(0,l_pwm*l_correction*speed);
}

//Correction
void AutoControlBoard::setRCorrection(unsigned int correction)
{
	r_correction = correction/100.0;
}

void AutoControlBoard::setLCorrection(unsigned int correction)
{
	l_correction = correction/100.0;
}

void AutoControlBoard::setCorrection(unsigned int r_correction, unsigned int l_correction)
{
	/*
	 * The correction is applied to each motor individually.
	 * It is used when the motors have different speeds using 
	 * the same pwm
	 */
	setRCorrection(r_correction);
	setLCorrection(l_correction);
}

void AutoControlBoard::setSpeed(unsigned int speed)
{
	/*
	 * Speed as an argument is on the range 0 to 100.
	 * The real speed will be a number between 0 and 1 and
	 * it will multiply all the pwms
	 */

    if(speed > 100) {speed = 100;}
	this->speed = speed/100.0;
}

void AutoControlBoard::setCurveFactor(byte factor)
{
	/*
	 * The curve factor is a number that multiplies the PWM of a motor
	 * when the methods moveForwardRight, moveForwardLeft, moveBackwardsRight
	 * and moveBackwardsLeft are called. E.g, when the robot turns right using
	 * the method moveForwardRight, the left motor rotates X times faster then
	 * the right motor. This X is the factor and it must be greater than 1
	 */

	if(factor == 0)
	{
		factor = 255;
	}
	curve_factor = factor;
}
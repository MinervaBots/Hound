#include "autocontrolboard.h"


AutoControlBoard::AutoControlBoard(byte r_pin, byte l_pin):
    ControlBoard(r_pin, l_pin)
{
    setCorrection(100,100);
	setCurveFactor(3);
	setSpeed(100);
}

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
	setRPWM(r_pwm*r_correction*speed);
	setLPWM(l_pwm*l_correction*speed);
}
void AutoControlBoard::moveForwardRight()
{
	setRPWM(r_pwm*r_correction*speed/curve_factor);
	setLPWM(l_pwm*l_correction*speed);
}
void AutoControlBoard::moveForwardLeft()
{
	setRPWM(r_pwm*r_correction*speed);
	setLPWM(l_pwm*l_correction*speed/curve_factor);
}

void AutoControlBoard::moveBackwards()
{
	setRPWM(r_pwm*r_correction*speed, true);
	setLPWM(l_pwm*l_correction*speed, true);
}

void AutoControlBoard::moveBackwardsRight()
{
	setRPWM(r_pwm*r_correction*speed/curve_factor, true);
	setLPWM(l_pwm*l_correction*speed,true);
}

void AutoControlBoard::moveBackwardsLeft()
{
	setRPWM(r_pwm*r_correction*speed, false);
	setLPWM(l_pwm*l_correction*speed/curve_factor, false);
}

void AutoControlBoard::rotateClockwise()
{
	setRPWM(r_pwm*r_correction*speed, true);
	setLPWM(l_pwm*l_correction*speed, false);
}

void AutoControlBoard::rotateAntiClockwise()
{
	setRPWM(r_pwm*r_correction*speed, false);
	setLPWM(l_pwm*l_correction*speed, true);
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

/*
	autocontrolboard.h - class that provides easy methods to control anything that moves
	like a car.

	If the user needs to control a
	car or a robot, this class will provide easy methods to do it. There are two ways
	to control motors: the manual way, setting the PWM of each motor, or the automatic
	way, calling a method to move them in a direction.
*/

#ifndef AUTOCONTROLBOARD_H
#define AUTOCONTROLBOARD_H

#include "controlboard.h"

class AutoControlBoard: public ControlBoard
{
public:
	AutoControlBoard(byte r_enable, 
		byte r_motor_1, byte r_motor_2,

		byte l_enable,
		byte l_motor_1, byte l_motor_2,
		
		byte r_vcc_ref=UNUSED, byte r_gnd_ref=UNUSED,
		byte l_vcc_ref=UNUSED, byte l_gnd_ref=UNUSED);

	//Automatic control
	virtual void moveForward();
	virtual void moveForwardRight();
	virtual void moveForwardLeft();

	virtual void moveBackwards();
	virtual void moveBackwardsRight();
	virtual void moveBackwardsLeft();

	virtual void rotateClockwise();
	virtual void rotateAntiClockwise();

	//Correction
	virtual void setRCorrection(unsigned int correction);
	virtual void setLCorrection(unsigned int correction);
	virtual void setCorrection(unsigned int r_correction, unsigned int l_correction);
	
	virtual void setSpeed(unsigned int speed);
	virtual void setCurveFactor(byte factor);

protected:

	float r_correction, l_correction;		//Corrects the motors pwm if they are not the same

	float curve_factor;						//Indicates how much a motor must rotate more than
											// the other motor to make a curve. It must be >= 1
	float speed;							//Multiplies every pwm
};
#endif
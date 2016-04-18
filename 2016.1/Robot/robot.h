/*
	autocontrolboard.h - class that provides easy methods to control anything that moves
	like a car.

	If the user needs to control a
	car or a robot, this class will provide easy methods to do it. There are two ways
	to control motors: the manual way, setting the PWM of each motor, or the automatic
	way, calling a method to move them in a direction.
*/


/*
	controlboard.h - class to control 2 DC motors. In a next version it will
	be updated to control N DC motors.

	ControlBoard makes it easy to control motors.

*/

#ifndef ROBOT_H
#define ROBOT_H

#include "DualDriver.h"

class Robot
{

public:
	Robot(DualDriver* driver);


	void useCommand(char command);
	/*
	 * These commands are compatible with this android app:
	 * 		https://sites.google.com/site/bluetoothrccar/
	 *
	 * So the robot can be controlled with any android phone.
	 */
	


	/*----| Configurations for the Control Fucntions |-----------------------*/
	void setSpeed(unsigned int speed);
	/*
	 * Speed as an argument is on the range 0 to 100.
	 * The real speed will be a number between 0 and 1 and it will multiply all
	 * the pwms.
	 */

	void setCurveFactor(byte factor);
	/*
	 * The curve factor is a number that multiplies the PWM of a motor when
	 * these control functions are called:
	 *	- moveForwardRight
	 *	- moveForwardLeft
	 *	- moveBackwardsRight
	 *	- moveBackwardsLeft
	 *
	 * E.g, when the robot turns right using the method moveForwardRight, the
	 * left motor rotates X times faster then the right motor. This X is the 
	 * factor and it must be greater than 1.
	 */



	/*----| Standard Control Fucntions |-------------------------------------*/
	void moveForward();
	void moveForwardRight();
	void moveForwardLeft();

	void moveBackwards();
	void moveBackwardsRight();
	void moveBackwardsLeft();

	void rotateClockwise();
	void rotateAntiClockwise();

	void stop();

	DualDriver* getDriver();



	/*----| Custom Control Fucntions |---------------------------------------*/
	/*
	 * In the following methods, the accepted values for the pwm arugments are
	 * in a rage of 0 to 255.
	 *
	 * Those methods set the pwm manually. Actually, if any automatic method is
	 * called, this value will be the maximum pwm that the motor is subjected 
	 * to, considering the speed and the correction variables in a range of 0
	 * to 1. If the speed and the correction are the default (1) and the 
	 * minimum pwm is 0, this method will truly set the pwm to the automatic 
	 * mode.
	 */

	// void setPWM(byte r_pwm, byte l_pwm);
	void setPWM(byte r_pwm, byte l_pwm, bool r_reverse=false, bool l_reverse=false);
	void setRPWM(byte pwm, bool reverse=false);
	void setLPWM(byte pwm, bool reverse=false);
	
	void setMinPWM(byte r_min_pwm, byte l_min_pwm);
	/*
	 * The minimum pwm is a value used to avoid motor stall at low pwms. As 
	 * soon as the pwm is below this value, the program will write zero on the
	 * motor input.
	 */




protected:
	//Corrects the motors pwm if they are not the same
	float r_correction, l_correction;

	//Indicates how much a motor must rotate more than the other motor to make 
	//a curve. It must be >= 1
	float curve_factor;						
	
	//Multiplies every pwm
	float speed;

	//Driver
	DualDriver* driver;

	byte r_pwm, l_pwm;
};

#endif

#include "robot.h"

/* Include the other libraries needed by the robot class
 * because the arduino IDE is terrible
 */

#include "autocontrolboard.h"
#include "sensor.h"
#include "sensorboard.h"
#include "ultrasonic.h"
#include "simpleencoder.h"
#include "lightsensor.h"

/* Define constants */

#define BAUD_RATE		9600

#define R_ENABLE		4
#define R_MOTOR_1		5
#define R_MOTOR_2		6
#define R_MOTOR_MAX_PWM	255

#define L_ENABLE		8
#define L_MOTOR_1		9
#define L_MOTOR_2		10
#define L_MOTOR_MAX_PWM	255

//Robot object
 
Robot robot(R_ENABLE,R_MOTOR_1,R_MOTOR_2,L_ENABLE,L_MOTOR_1,L_MOTOR_2);

char command = 'S';

void setup()
{
	/*
	 *Setting the PWM as a constant indicates that the method used to
	 *change the robot speed will be using the method setSpeed, that
	 *accepts a number between 0 and 100.
	 */

	robot.setMinPWM(80, 80);
	robot.setPWM(R_MOTOR_MAX_PWM, L_MOTOR_MAX_PWM);
	robot.stop();

	Serial.begin(BAUD_RATE);
}

void loop()
{
	if(Serial.available() > 0)
	{
		/*
		 * Read data from serial and control the robot
		 */
		command = Serial.read();
		Serial.println(command);

		robot.useCommand(command);
	}
}
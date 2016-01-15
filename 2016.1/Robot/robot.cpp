#include "robot.h"

Robot::Robot(byte r_enable, 
		byte r_motor_1, byte r_motor_2,

		byte l_enable,
		byte l_motor_1, byte l_motor_2,
		
		byte r_vcc_ref, byte r_gnd_ref,
		byte l_vcc_ref, byte l_gnd_ref):

	SensorBoard(),
	AutoControlBoard(r_enable, r_motor_1, r_motor_2,
		l_enable, l_motor_1, l_motor_2,
		r_vcc_ref, r_gnd_ref,
		l_vcc_ref, l_gnd_ref)
{
}

void Robot::useCommand(char command)
{
	/*
	 * These commands are compatible with this android app
	 * https://sites.google.com/site/bluetoothrccar/
	 * So the robot can be controlled with any android phone 
	 */

	switch(command)
	{
		case 'F':  
			moveForward();
			break;
		case 'B':  
			moveBackwards();
			break;
		case 'L':  
			rotateAntiClockwise();
			break;
		case 'R':
			rotateClockwise();
			break;
		case 'S':  
			stop();
			break; 
		case 'I':  //FR  
			moveForwardRight();
			break; 
		case 'J':  //BR  
			moveBackwardsRight();
			break;        
		case 'G':  //FL  
			moveForwardLeft();
			break; 
		case 'H':  //BL
			moveBackwardsLeft();
			break;
		case 'W':  //Font ON 
			// digitalWrite(pinfrontLights, HIGH);
			break;
		case 'w':  //Font OFF
			// digitalWrite(pinfrontLights, LOW);
			break;
		case 'U':  //Back ON 
			// digitalWrite(pinbackLights, HIGH);
			break;
		case 'u':  //Back OFF 
			// digitalWrite(pinbackLights, LOW);
			break; 
		case 'D':  //Everything OFF 
			// digitalWrite(pinfrontLights, LOW);
			// digitalWrite(pinbackLights, LOW);
			stop();
			break;         
		default:  //Get velocity
			if(command=='q')
			{
				setSpeed(100);
			}
			else if((command >= 48) && (command <= 57))
			{ 
				//Chars '0' - '9' have an integer equivalence of 48 - 57, accordingly.
				//Subtracting 48 changes the range from 48-57 to 0-9.
				//Multiplying by 10 changes the range from 0-9 to 0-90.
				setSpeed((command - 48)*10);
			}
			break;
	}
}
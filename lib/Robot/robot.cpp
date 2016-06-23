#include "robot.h"

Robot::Robot(DualDriver* driver)
{
	this->driver = driver;
	setCurveFactor(3);
	setSpeed(100);
	this->l_pwm = 127; //pq 127??
	this->r_pwm = 127;
	setMinPWM(0,0);
}

void Robot::useCommand(char command)
{
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



/*----| Configurations for the Control Fucntions |-----------------------*/
void Robot::setSpeed(unsigned int speed)
{
    if(speed > 100) {speed = 100;}
	this->speed = speed/100.0;
}

void Robot::setCurveFactor(byte factor)
{
	if(factor == 0)
	{
		factor = 255;
	}
	curve_factor = factor;
}



/*----| Standard Control Fucntions |-------------------------------------*/
void Robot::moveForward()
{
	// Serial.print("PWM:");Serial.println(r_pwm);
	// Serial.print("Speed:");Serial.println(speed);
	setRPWM(r_pwm*speed);
	setLPWM(l_pwm*speed);
}

void Robot::moveForwardRight()
{
	setRPWM(r_pwm*speed/curve_factor);
	setLPWM(l_pwm*speed);
}

void Robot::moveForwardLeft()
{
	setRPWM(r_pwm*speed);
	setLPWM(l_pwm*speed/curve_factor);
}

void Robot::moveBackwards()
{
	setRPWM(r_pwm*speed, true);
	setLPWM(l_pwm*speed, true);
}

void Robot::moveBackwardsRight()
{
	setRPWM(r_pwm*speed/curve_factor, true);
	setLPWM(l_pwm*speed,true);
}

void Robot::moveBackwardsLeft()
{
	setRPWM(r_pwm*speed, false);
	setLPWM(l_pwm*speed/curve_factor, false);
}

void Robot::rotateClockwise()
{
	setRPWM(r_pwm*speed, true);
	setLPWM(l_pwm*speed, false);
}

void Robot::rotateAntiClockwise()
{
	setRPWM(r_pwm*speed, false);
	setLPWM(l_pwm*speed, true);
}



//----------//----------//----------//----------//----------//----------//----------
// ATEMCAO!! FUNCOES USANDO DE FATO O DRIVER!!

/*----| Custom Control Fucntions |---------------------------------------*/
void Robot::setPWM(byte r_pwm, byte l_pwm, bool r_reverse, bool l_reverse)
{
	setRPWM(r_pwm, r_reverse);
	setLPWM(l_pwm, l_reverse);
}

// void Robot::setPWM(byte r_pwm, byte l_pwm)
// {
// 	setRPWM(r_pwm, false);
// 	setLPWM(l_pwm, false);
// }


void Robot::setRPWM(byte pwm, bool reverse)
{
	driver->setAllRightPWM(pwm, reverse);
}

void Robot::setLPWM(byte pwm, bool reverse)
{
	driver->setAllLeftPWM(pwm, reverse);
}

void Robot::stop()
{
	driver->stopAll();
}

void Robot::setMinPWM(byte r_min_pwm, byte l_min_pwm)
{
	driver->setMinRightPWM(r_min_pwm);
	driver->setMinLeftPWM(l_min_pwm);
}

DualDriver*  Robot::getDriver()
{
	return driver;
}

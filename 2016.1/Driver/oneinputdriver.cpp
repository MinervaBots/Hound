#include "oneinputdriver.h"

OneInputDriver::OneInputDriver(){
	setMinPWM(0);
	setMaxPWM(255);
	setStopPWM(127);
}

void OneInputDriver::moveForward(byte pwm){
	int correct_pwm = pwm_stop + (pwm/(float)255.0)*(pwm_max - pwm_stop);
	setPWM(correct_pwm);
}

void OneInputDriver::moveBackwards(byte pwm){
	int correct_pwm = pwm_stop - (pwm/(float)255.0)*(pwm_stop - pwm_min);
	setPWM(correct_pwm);
}

void OneInputDriver::setPWM(byte pwm){
	//Change this.pwm
	DriverInterface::setPWM(pwm);
	//Write the valid pwm
	analogWrite(input, this->pwm);
}
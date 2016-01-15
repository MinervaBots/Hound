#include "driverinterface.h"

DriverInterface::DriverInterface(){
	pwm = 0;
	pwm_min = 0;
	pwm_max = 255;
	pwm_stop = 0;	
}

void DriverInterface::setMinPWM(byte pwm){
	pwm_min = pwm;

	if(this->pwm < pwm_min){
		this->pwm = pwm_min;
	}
}

void DriverInterface::setMaxPWM(byte pwm){
	pwm_max = pwm;
	
	if(this->pwm > pwm_max){
		this->pwm = pwm_max;
	}
}

void DriverInterface::setPWM(byte pwm){
	if(pwm > pwm_max){
		this->pwm = pwm_max;
	} else if (pwm < pwm_min) {
		this->pwm = pwm_min;
	} else {
		this->pwm = pwm;
	}
}

void DriverInterface::setStopPWM(byte pwm){
	pwm_stop = pwm;
}

void DriverInterface::stop(){
	pwm = pwm_stop;
}
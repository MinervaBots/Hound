/*
	TwoInputsDriver.h - 1 DC motor based driver

	TwoInputsDriver is a class used to control motors through a driver, with
	ports Input 1, Input 2 and enable.

*/

#ifndef TWOINPUTSDRIVER_H
#define TWOINPUTSDRIVER_H

// #include <inttypes.h>
// #include <Arduino.h>

// #define UNUSED	255

#include "driverinterface.h"

class TwoInputsDriver:public DriverInterface
{

public:
	TwoInputsDriver(byte enable,
		byte input_1, byte input_2,
		byte vcc_ref = UNUSED, byte gnd_ref = UNUSED);

	bool setPWM(byte pwm_1, byte pwm_2);
	// void setMinPWM(byte pwm_1, byte pwm_2);
	void moveForward(byte pwm);
	void moveBackwards(byte pwm);
	// void setPWM(byte pwm);
	void setEnable(bool state);
	void stop();

private:

	//Pins
	byte enable;
	byte input_1, input_2;		
	byte vcc_ref, gnd_ref;

	//PWMs
	byte pwm_1, pwm_2;
	// byte pwm_min_1, pwm_min_2;

	void setDigitalPin(byte pin, bool mode, bool state);
};

#endif

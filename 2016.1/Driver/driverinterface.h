#ifndef DRIVERINTERFACE_H
#define DRIVERINTERFACE_H

#include <inttypes.h>
#include <Arduino.h>

#define UNUSED	255

class DriverInterface{
	
	public:
		DriverInterface();
		virtual void moveForward(byte pwm) 		= 0;
		virtual void moveBackwards(byte pwm) 	= 0;
		virtual void setMinPWM(byte pwm);
		virtual void setMaxPWM(byte pwm);
		virtual void setPWM(byte pwm);
		virtual void setStopPWM(byte pwm);
		virtual void stop();

	protected:
		byte pwm;
		byte pwm_min;
		byte pwm_max;
		byte pwm_stop;

};

#endif
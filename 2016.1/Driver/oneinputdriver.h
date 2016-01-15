#ifndef ONEINPUTDRIVER_H
#define ONEINPUTDRIVER_H

#include "driverinterface.h"

class OneInputDriver: public DriverInterface{
	
	public:
		OneInputDriver();
		void moveForward(byte pwm);
		void moveBackwards(byte pwm);
		void setPWM(byte pwm);
	private:
		byte input;
};

#endif
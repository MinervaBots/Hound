#ifndef ONEINPUTDRIVER_H
#define ONEINPUTDRIVER_H

#include "driverinterface.h"

class OneInputDriver: public DriverInterface{

	public:
		OneInputDriver(byte pin);
		void moveForward(byte pwm);
		void moveBackwards(byte pwm);
		void setPWM(byte pwm);
		void stop();
	private:
		byte input;
};

#endif

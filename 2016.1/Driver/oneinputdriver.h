#ifndef ONEINPUTDRIVER_H
#define ONEINPUTDRIVER_H

#include "driverinterface.h"

class OneInputDriver: public DriverInterface{

	public:
		OneInputDriver(byte pin, byte extra1=0, byte extra2=0, byte extra3=0, byte extra4=0);
		void moveForward(byte pwm);
		void moveBackwards(byte pwm);
		void setPWM(byte pwm, byte extra = 0);
		void stop();
	private:
		byte input;
};

#endif

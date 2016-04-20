/*
 *HC-SR04 Ultrasonic sensor
 */
 
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "../Common/sensor.h"

#define TO_CM					58.2
#define TO_INCH					148.0

#define DEFAULT_TIMEOUT			20000

enum DistanceSystem
{
	CM,
	INCH
};

class Ultrasonic: public Sensor
{
public:
	Ultrasonic(int trigger_pin, int echo_pin);

	long getTimming();
	long getRawDistance();
	data_t getDistance();

	void setSystem(DistanceSystem system);
	void setTimeout(unsigned long timeout=DEFAULT_TIMEOUT);
	data_t getRawValue();

private:

	int trigger_pin, echo_pin;
	data_t system_conv;		//System convertion
	unsigned long timeout;
};

#endif

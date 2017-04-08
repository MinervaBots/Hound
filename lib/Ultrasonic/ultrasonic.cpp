#include "Ultrasonic.h"


Ultrasonic::Ultrasonic(int trigger_pin, int echo_pin) :
	trigger_pin(trigger_pin),
	echo_pin(echo_pin)
{
	pinMode(trigger_pin, OUTPUT);
	pinMode(echo_pin, INPUT);
	setTimeout(DEFAULT_TIMEOUT);
	setSystem(CM);
}

Ultrasonic::Ultrasonic(int trigger_pin, int echo_pin, double minRange, double maxRange) :
	Ultrasonic(trigger_pin, echo_pin)
{
		setRange(minRange, maxRange);
}

long Ultrasonic::getTimming()
{
	digitalWrite(trigger_pin, LOW);
	delayMicroseconds(2);

	digitalWrite(trigger_pin, HIGH);
	delayMicroseconds(10);

	digitalWrite(trigger_pin, LOW);

	return pulseIn(echo_pin, HIGH, timeout);
}

long Ultrasonic::getRawDistance()
{
	/*
	 * Get the distance in centimeters. If the distance is out of range,
	 * it will return SENSOR_OUT_OF_RANGE
	 */

	long distance = getTimming()/system_conv;

	if(distance >= maximum_range || distance <= minimum_range)
	{
		return SENSOR_OUT_OF_RANGE;
	}

	return distance;
}

double Ultrasonic::getDistance()
{
	return getMeanValue();
}

void Ultrasonic::setSystem(DistanceSystem system)
{
	if(system == CM)
	{
		system_conv = TO_CM;
	}
	else if(system == INCH)
	{
		system_conv = TO_INCH;
	}
}

void Ultrasonic::setTimeout(unsigned long timeout)
{
	this->timeout = timeout;
}

void Ultrasonic::setRange(double minimum, double maximum)
{
	Sensor::setRange(minimum, maximum);
	setTimeout(maximum * system_conv);
}

double Ultrasonic::getRawValue()
{
	return getRawDistance();
}

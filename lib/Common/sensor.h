#ifndef SENSOR_H
#define SENSOR_H

#include "sensorproperties.h"
#include <inttypes.h>
#include <Arduino.h>

class Sensor
{
public:
	Sensor(int samples=10);
	virtual double getRawValue() = 0;
	virtual double getMeanValue();
	virtual double getMeanValue(int samples);
	virtual void setRange(double minimum, double maximum);
	virtual void setNumberOfSamples(int samples);
//    virtual data_t getValue(FILTER TYPE); Must do it


protected:
	int samples;
	double minimum_range;
	double maximum_range;
//    data_t buffer; Must do it
//    LAST_FILTER; Must do it
};

#endif

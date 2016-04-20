#ifndef SENSOR_H
#define SENSOR_H

#include "sensorproperties.h"
#include <inttypes.h>
#include <Arduino.h>

class Sensor
{
public:
	Sensor(int samples=10);
	virtual data_t getRawValue() = 0;
	virtual data_t getMeanValue();
	virtual data_t getMeanValue(int samples);
	virtual void setRange(data_t minimum, data_t maximum);
	virtual void setNumberOfSamples(int samples);
//    virtual data_t getValue(FILTER TYPE); Must do it


protected:
	int samples;
	data_t minimum_range;
	data_t maximum_range;
//    data_t buffer; Must do it
//    LAST_FILTER; Must do it
};

#endif

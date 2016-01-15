#include "sensor.h"

Sensor::Sensor(int samples)
{
	this->samples = samples;
    setRange(0,1023);
}

data_t Sensor::getMeanValue()
{
	return getMeanValue(this->samples);
}

data_t Sensor::getMeanValue(int samples)
{
	data_t mean = 0;
	int used_samples = 0;

	for(int i = 0; i < samples; i++)
	{
		data_t current_value = getRawValue();
		if(current_value != SENSOR_OUT_OF_RANGE)
		{
			mean += current_value;
			used_samples ++;
		}
	}

	if(used_samples != 0)
	{
		mean /= used_samples;
		if(mean > minimum_range && mean < maximum_range)
		{
			return mean;
		}
	}
	return SENSOR_OUT_OF_RANGE;
}

void Sensor::setRange(data_t minimum, data_t maximum)
{
	minimum_range = minimum;
	maximum_range = maximum;
}

void Sensor::setNumberOfSamples(int samples)
{
	this->samples = samples;
}

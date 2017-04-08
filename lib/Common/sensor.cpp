#include "sensor.h"

Sensor::Sensor(int samples)
{
	this->samples = samples;
  setRange(0,1023);
}

double Sensor::getMeanValue()
{
	return getMeanValue(this->samples);
}

double Sensor::getMeanValue(int samples)
{
	double mean = 0;
	int used_samples = 0;

	for(int i = 0; i < samples; i++)
	{
		double current_value = getRawValue();
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

void Sensor::setRange(double minimum, double maximum)
{
	minimum_range = minimum;
	maximum_range = maximum;
}

void Sensor::setNumberOfSamples(int samples)
{
	this->samples = samples;
}

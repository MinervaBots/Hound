#include "simpleencoder.h"

SimpleEncoder::SimpleEncoder(int pulses_per_rotation)
{
	setPulsesPerRotation(pulses_per_rotation);
	reset();
}

void SimpleEncoder::add(int value)
{
	delta_pulses = value;
	pulses += value;
}

void SimpleEncoder::subtract(int value)
{
	delta_pulses = value;
	pulses -= value;
}

void SimpleEncoder::reset()
{
	pulses = 0;
}

long int SimpleEncoder::getPulses()
{
	return pulses;
}

int SimpleEncoder::getDeltaPulses() {
	return delta_pulses;
}

void SimpleEncoder::setPulsesPerRotation(int value)
{
	if(value == 0)
	{
		value = 1;
	}
	pulses_per_rotation = value;
}

float SimpleEncoder::getRotation()
{
	/*
	 * Returns how many times the encoder rotated
	 */
	return pulses/pulses_per_rotation;
}
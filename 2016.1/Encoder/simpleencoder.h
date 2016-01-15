#ifndef SIMPLEENCODER_H
#define SIMPLEENCODER_H

class SimpleEncoder
{
public:
	SimpleEncoder(int pulses_per_rotation=1);
	void add(int value=1);
	void subtract(int value=1);
	void reset();
	long int getPulses();
	void setPulsesPerRotation(int value);
	float getRotation();
	int getDeltaPulses();

private:
	volatile long int pulses;
	int delta_pulses;
	int pulses_per_rotation;
};

#endif
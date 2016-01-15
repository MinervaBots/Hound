#include "sensorboard.h"

SensorBoard::SensorBoard()
{
	initSensors();
}

SensorBoard::~SensorBoard()
{
	clearSensors();
}

bool SensorBoard::setUltrasonic(int trigger_pin, int echo_pin, SensorPosition position, unsigned long timeout)
{
	if(position >= NUMBER_OF_POSITIONS)
	{
		return false;
	}
	if(ultrasonic_sensors[position] != NULL)
	{
		delete ultrasonic_sensors[position];
	}
	
	ultrasonic_sensors[position] = new Ultrasonic(trigger_pin, echo_pin);
	ultrasonic_sensors[position]->setTimeout(timeout);

	return true;
}

data_t SensorBoard::getUltrasonicDistance(SensorPosition position, int number_of_samples)
{
	if(position < NUMBER_OF_POSITIONS)
	{
		if(ultrasonic_sensors[position] == NULL)
		{
			return INVALID_SENSOR;
		}
		return ultrasonic_sensors[position]->getMeanValue(number_of_samples);
	}
	return INVALID_POSITION;
}

void SensorBoard::initSensors()
{
	for(int i = 0; i < NUMBER_OF_POSITIONS; i++)
	{
		ultrasonic_sensors[i] = NULL;
	}
}


void SensorBoard::clearSensors()
{
	for(int i = 0; i < NUMBER_OF_POSITIONS; i++)
	{
		if(ultrasonic_sensors[i] != NULL)
		{
			delete ultrasonic_sensors[i];
			ultrasonic_sensors[i] = NULL;
		}
	}
	delete []ultrasonic_sensors;
}
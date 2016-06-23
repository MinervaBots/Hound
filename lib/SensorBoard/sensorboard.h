#ifndef SENSORBOARD_H
#define SENSORBOARD_H

#include "../Ultrasonic/ultrasonic.h"
#include "../Encoder/simpleencoder.h"
#include "../LightSensor/lightsensor.h"

class SensorBoard
{
public:
	SensorBoard();
	~SensorBoard();
	virtual bool setUltrasonic(int trigger_pin, int echo_pin, SensorPosition position, unsigned long timeout=DEFAULT_TIMEOUT);
	virtual data_t getUltrasonicDistance(SensorPosition position, int number_of_samples=1);

private:
	Ultrasonic *ultrasonic_sensors[NUMBER_OF_POSITIONS];
	void initSensors();
	void clearSensors();
};

#endif

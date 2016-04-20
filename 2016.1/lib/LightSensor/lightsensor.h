#ifndef LIGHTSENSOR_H
#define LIGHTSENSOR_H

#include "../Common/sensor.h"

class LightSensor: public Sensor
{
public:
    LightSensor(byte pin);
    data_t getRawValue();

private:
    byte pin;
};

#endif // LIGHTSENSOR_H

#include "lightsensor.h"

LightSensor::LightSensor(byte pin)
{
    this->pin = pin;
}

data_t LightSensor::getRawValue()
{
    return analogRead(pin);
}

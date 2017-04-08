#ifndef SENSOR_ARRAY_H
#define SENSOR_ARRAY_H

#include "../Common/sensor.h"
#include "../LinkedList/LinkedList.h"
#include "../Log/Log.h"

class SensorArray
{
public:
  int DetectedCount;
  SensorArray();
  void AddSensor(Sensor *pSensor, float weight, double valueForDetection);
  float Update();
  void PrintLog(Log log, const char delimiter);

private:
  struct SensorData
  {
    Sensor *pSensor;
    float Weight;
  };
  LinkedList<SensorData> m_Sensors;
};

#endif // SENSOR_ARRAY_H

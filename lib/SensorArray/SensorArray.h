#ifndef SENSOR_ARRAY_H
#define SENSOR_ARRAY_H

#include "../Common/sensor.h"
#include "../LinkedList/LinkedList.h"
#include "../Log/Log.h"

struct SensorData
{
  Sensor *pSensor;
  float Weight;
};

class SensorArray
{
public:
  unsigned int DetectedCount;
  SensorArray();
  ~SensorArray();

  size_t Count();
  SensorData GetSensorData(int index);
  void AddSensor(Sensor *pSensor, float weight, double valueForDetection);
  float Update();
  void PrintLog(Log log, const char delimiter);

private:
  LinkedList<SensorData> m_Sensors;
};

#endif // SENSOR_ARRAY_H

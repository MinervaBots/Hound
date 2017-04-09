#include "SensorArray.h"


SensorArray::SensorArray() :
  DetectedCount(0),
  m_Sensors(LinkedList<SensorData>())
{

}

SensorArray::~SensorArray()
{
  for(int i = m_Sensors.size() - 1; i >= 0; i--)
  {
		delete m_Sensors.remove(0).pSensor;
  }
}

size_t SensorArray::Count()
{
  return m_Sensors.size();
}


SensorData SensorArray::GetSensorData(int index)
{
  return m_Sensors.get(index);
}

void SensorArray::AddSensor(Sensor *pSensor, float weight, double valueForDetection)
{
  SensorData sensor = SensorData();
  sensor.pSensor = pSensor;
  sensor.pSensor->setRange(0, valueForDetection);

  sensor.Weight = weight;
  m_Sensors.add(sensor);
}

float SensorArray::Update()
{
  int errorsSum = 0;
	DetectedCount = 0;
  for(int i = 0; i < m_Sensors.size(); i++)
  {
    SensorData sensorData = m_Sensors.get(i);
    double meanValue = sensorData.pSensor->getMeanValue();
		if(meanValue != SENSOR_OUT_OF_RANGE)
		{
			DetectedCount += 1;
			errorsSum += sensorData.Weight;
		}
	}
	return DetectedCount == 0 ? 0 : errorsSum / DetectedCount;
}

void SensorArray::PrintLog(Log log, const char delimiter)
{
  for(int i = 0; i < m_Sensors.size(); i++)
  {
    log << delimiter << m_Sensors.get(i).pSensor->getMeanValue();
  }
	log << delimiter;
}

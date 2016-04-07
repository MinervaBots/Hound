#ifndef ROBOT_H
#define ROBOT_H

#include "../Driver/autocontrolboard.h"
#include "../SensorBoard/sensorboard.h"


class Robot: public SensorBoard, public AutoControlBoard
{

public:
    Robot(byte tx_pin, byte rx_pin, int timeOut, int address);

    void useCommand(char command);
};

#endif

#ifndef ROBOT_H
#define ROBOT_H

#include "../Driver/autocontrolboard.h"
#include "../SensorBoard/sensorboard.h"


class Robot: public SensorBoard, public AutoControlBoard
{

public:
    Robot(byte r_pin, byte l_pin);      // Constructor used when there is just one pin per motor

	Robot(byte r_enable,
		byte r_motor_1, byte r_motor_2,

		byte l_enable,
		byte l_motor_1, byte l_motor_2,

		byte r_vcc_ref=UNUSED, byte r_gnd_ref=UNUSED,
		byte l_vcc_ref=UNUSED, byte l_gnd_ref=UNUSED);      // Constructor used when there are two command pins to control each motor



	void useCommand(char command);
};

#endif

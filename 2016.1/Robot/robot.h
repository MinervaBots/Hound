#ifndef ROBOT_H
#define ROBOT_H

#include "../Driver/autocontrolboard.h"
#include "../SensorBoard/sensorboard.h"

#define ONEINPUT

class Robot: public SensorBoard, public AutoControlBoard
{

public:
    #ifdef ONEINPUT
    Robot(byte r_pin, byte l_pin);
    #else
	Robot(byte r_enable,
		byte r_motor_1, byte r_motor_2,

		byte l_enable,
		byte l_motor_1, byte l_motor_2,

		byte r_vcc_ref=UNUSED, byte r_gnd_ref=UNUSED,
		byte l_vcc_ref=UNUSED, byte l_gnd_ref=UNUSED);
    #endif // ONEINPUT

	void useCommand(char command);
};

#endif

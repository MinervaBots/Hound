/*
	L298 based driver
*/

#ifndef DUODRIVER_H
#define DUORDRIVER_H

#define ONEINPUT

#ifdef ONEINPUT
#include "oneinputdriver.h"
#else
#include "twoinputsdriver.h"
#endif // ONEINPUT


class DuoDriver
{

public:
    DuoDriver(
		byte r_enable,
		byte r_motor_1, byte r_motor_2,

		byte l_enable,
		byte l_motor_1, byte l_motor_2,

		byte r_vcc_ref=UNUSED, byte r_gnd_ref=UNUSED,
		byte l_vcc_ref=UNUSED, byte l_gnd_ref=UNUSED);

    DuoDriver(byte r_pin, byte l_pin);

	#ifndef ONEINPUT
	TwoInputsDriver	l_motor, r_motor;
	#else
	OneInputDriver l_motor, r_motor;
	#endif // ONEINPUT

	void stop();
    bool oneInput;
};

#endif

#include "duodriver.h"

#define ONEINPUT


#ifndef ONEINPUT
DuoDriver::DuoDriver(
		byte r_enable,
		byte r_motor_1, byte r_motor_2,

		byte l_enable,
		byte l_motor_1, byte l_motor_2,

		byte r_vcc_ref, byte r_gnd_ref,
		byte l_vcc_ref, byte l_gnd_ref):

	r_motor(r_enable,r_motor_1,r_motor_2,r_vcc_ref,r_gnd_ref),
	l_motor(l_enable,l_motor_1,l_motor_2,l_vcc_ref,l_gnd_ref)
{
}
#endif // ONEINPUT

#ifdef ONEINPUT
DuoDriver::DuoDriver(byte r_pin, byte l_pin):
    r_motor(r_pin),
    l_motor(l_pin)
{
}
#endif // ONEINPUT

void DuoDriver::stop()
{
	r_motor.stop();
	l_motor.stop();
}

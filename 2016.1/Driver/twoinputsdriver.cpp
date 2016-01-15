#include "twoinputsdriver.h"

TwoInputsDriver::TwoInputsDriver(
		byte enable,
		byte input_1, byte input_2,
		byte vcc_ref, byte gnd_ref)
{
	this->enable = enable;
	this->input_1 = input_1;
	this->input_2 = input_2;
	this->vcc_ref = vcc_ref;
	this->gnd_ref = gnd_ref;

	pwm_1 = 0;
	pwm_2 = 0;
	// setMinPWM(0,0);
	setMinPWM(0);
	setMaxPWM(255);
	setStopPWM(0);
	setDigitalPin(this->enable, OUTPUT, HIGH);
	setDigitalPin(this->vcc_ref, OUTPUT, HIGH);
	setDigitalPin(this->gnd_ref, OUTPUT, LOW);
}

bool TwoInputsDriver::setPWM(byte pwm_1, byte pwm_2)
{
	/* If the pwm given is shorter than the minimum pwm
	 * this method will clamp it to 0. If it's bigger
	 * than the maximum pwm, it will be equals to the
	 * maximum pwm.
	 */

	if(pwm_1 <= pwm_min)
	{
		pwm_1 = 0;
	}

	if(pwm_2 <= pwm_min)
	{
		pwm_2 = 0;
	}

	if(pwm_1 > pwm_max)
	{
		pwm_1 = pwm_max;
	}

	if(pwm_2 > pwm_max)
	{
		pwm_2 = pwm_max;
	}

	if(pwm_1 == 0)
	{
		/* Writes the low level first to avoid short circuit */
		analogWrite(input_1, pwm_1);
		analogWrite(input_2, pwm_2);

	} else if(pwm_2 == 0){

		/* Writes the low level first to avoid short circuit */
		analogWrite(input_2, pwm_2);
		analogWrite(input_1, pwm_1);
	} else {
		
		/* If both are non-zero pwms, a short circuit will occur! */
		return 0;
	}
	
	/* If both pwms are valid, update them */
	this->pwm_1 = pwm_1;
	this->pwm_2 = pwm_2;

	return 1;
}

// void TwoInputsDriver::setMinPWM(byte pwm_1, byte pwm_2)
// {
// 	 Set the minimum PWM. If any PWM value is set under this
// 	 * threshold, it will be clamped to zero.
	 

// 	this->pwm_min_1 = pwm_1;
// 	this->pwm_min_2 = pwm_2;
	
// 	//Updates the pwms
// 	setPWM(this->pwm_1, this->pwm_2);
// }

void TwoInputsDriver::setEnable(bool state)
{
	/* Set the enable pin. High level to turn it ON, and a Low level
	 * to turn it off
	 */

	if(enable != UNUSED)
	{
		digitalWrite(enable, state);
	}
}

void TwoInputsDriver::stop()
{
	/* Stops the motor
	 */

	setPWM(0,0);
}

void TwoInputsDriver::setDigitalPin(byte pin, bool mode, bool state)
{
	/* Auxiliar funtion to set up a digital pin
	 */
	
	if(pin != UNUSED)
	{
		pinMode(pin, mode);
		digitalWrite(pin, state);
	}
}

void TwoInputsDriver::moveForward(byte pwm){
	setPWM(pwm,0);
}

void TwoInputsDriver::moveBackwards(byte pwm){
	setPWM(0,pwm);
}
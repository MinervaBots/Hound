#ifndef DUALDRIVER_H
#define DUALDRIVER_H

#include <inttypes.h>
#include <Arduino.h>

#define UNUSED	255

class DualDriver{

	public:
		virtual void setAllLeftPWM(byte pwm, bool reverse)                = 0;
		virtual void setAllRightPWM(byte pwm, bool reverse)               = 0;

		virtual void setMinLeftPWM(byte pwm)                              = 0;
		virtual void setMinRightPWM(byte pwm)                             = 0;

		virtual void setMaxLeftPWM(byte pwm)                              = 0;
		virtual void setMaxRightPWM(byte pwm)                             = 0;

		virtual void stopAll()                                            = 0;

		virtual void setCorrection(float r_correction,float l_correction) = 0;

	// protected:
	// 	byte pwm_stop;
	// 	byte r_pwm_min, l_pwm_min;
 //    	byte r_pwm_max, l_pwm_max;

};

#endif

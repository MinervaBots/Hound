/*
	XLMaxSonarEZ.h - XLMaxSonarEZ implements trigger and the analog read method,
	as described on the datasheet.

	Created by Bruno Calou Alves, March, 2015.
	Read LICENSE for more information.
*/

#ifndef XLMAXSONAREZ_H
#define XLMAXSONAREZ_H

#include <inttypes.h>

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#ifndef NULL
#define NULL 0
#endif

//Define a pin number not present in any arduino
#ifndef UNUSED
#define UNUSED 255
#endif

/*
	Constants to convert the analog data to units of length
*/

const float SONAR_TO_CM = 100.0 / 95.0;

namespace Sonar {
	/*
		All operation modes defined on the datasheet.

		- Chain: First sensor is triggered before reading.
		
		- Chain Loop: First sensor is triggered only once, regardless the
			number of subsequent readings

		- Simultaneous: All the sensors use the same RX pin and are triggered at
			the same time

		-Single: There is a RX pin for each individual sensor. They are triggered
			before reading.
	*/

	enum OperationMode {
		CHAIN,
		CHAIN_LOOP,
		SIMULTANEOUS,
		SINGLE
	};
};

class XLMaxSonarEZ {
	public:
		XLMaxSonarEZ(byte tx, byte rx = UNUSED);
		~XLMaxSonarEZ();

		/*
			Trigger the RX pin for 20 microseconds, so the sensor starts
			reading
		*/
		void trigger();

		/*
			Perform an analog read on the TX pin and converts to
			a unit of length

			TODO_1: IMPLEMENT OTHER CONSTANTS AND CREATE A VARIABLE TO HOLD
			THE CURRENT CONVERSION

			TODO_2: IMPLEMENT OTHER METHODS TO READ THE SENSOR, AS DESCRIBED
			ON THE DATASHEET
		*/
		float read();

		/*
			Get the last distance read. This method will NOT read the sensor (use the
			read method instead)
		*/
		float getDistance();

		/*
			Get methods
		*/
		byte getRX();
		byte getTX();

	private:
		byte rx;
		byte tx;

		/*
			The last distance measured.
		*/
		float distance;
};

#endif

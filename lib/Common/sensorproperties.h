enum SensorPosition
{
	/*
	 *BE CAREFUL! FRONT MUST BE THE FIRST AND NONE THE LAST!
	 */
	FRONT,
	FRONT_RIGHT,
	FRONT_LEFT,
	BACK,
	BACK_RIGHT,
	BACK_LEFT,
	RIGHT,
	LEFT,
	CENTER,
	NONE
};

enum SensorType
{
	ULTRASONIC,
	INFRARED,
	LIGHT
};

#define NUMBER_OF_POSITIONS 	NONE - FRONT + 1
#define SENSOR_OUT_OF_RANGE		-1
#define INVALID_POSITION		-2
#define INVALID_SENSOR			-3

typedef double data_t;			//Data type
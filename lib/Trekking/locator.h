#ifndef LOCATOR_H
#define LOCATOR_H

#include "Arduino.h"
#include "inttypes.h"

#include "../Timer/timer.h"
#include "../EncoderList/encoderlist.h"
#include "../Encoder/simpleencoder.h"
#include "position.h"

#include <Wire.h>
#include "../I2CDev/I2Cdev.h"
#include "../MPU9150Lib/MPU9150Lib.h"
#include "../CalLib/CalLib.h"
#include <dmpKey.h>
#include <dmpmap.h>
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
// #include <EEPROM.h>

const float WHEEL_RADIUS = 0.075;
const float PULSES_PER_ROTATION = 1216.0;
const float DISTANCE_FROM_RX = 0.1375; // 137.5mm = distancia entre a roda e o eixo sagital do robo

#ifndef PI
#define PI 3.141592653589793238;
#endif

class Locator {
public:
	Locator(Stream *encoder_stream, Position initial_position);
	~Locator();

	//getters
//	const SimpleEncoder& getBackLeftEncoder() const;
//	const SimpleEncoder& getBackRightEncoder() const;
	const EncoderList& getEncoderList() const;
	const SimpleEncoder& getFrontLeftEncoder() const;
	const SimpleEncoder& getFrontRightEncoder() const;
	float getLastRobotAngularSpeed() const;
	float getLastRobotLinearSpeed() const;
	unsigned long getLastUpdateTime() const;
	float getRobotAngularSpeed() const;
	float getRobotLinearSpeed() const;
	float getRightSpeed() const;
	float getLeftSpeed() const;

	void start();
	void initMPU();
	void update();
	Position* getLastPosition();
	void reset(Position new_position);
	EncoderList encoder_list;

	void readMPU();
	// TimerForMethods<Locator> calibrate_angle_timer;
	float euler_radians[3];
	float initial_euler_radians;

private:
	SimpleEncoder front_left_encoder;
	SimpleEncoder front_right_encoder;
//	SimpleEncoder back_left_encoder;
//	SimpleEncoder back_right_encoder;


	unsigned long last_update_time;
	float robot_linear_speed;
	float robot_angular_speed;
	// Stream *encoder_stream;


	float last_robot_linear_speed;
	float last_robot_angular_speed;

	float left_speed, right_speed;
	Position last_position;

	float rps[2];
	MPU9150Lib MPU;
	bool mpu_first_time;



	void calculateSpeeds(float pps[]);
	void calcutePosition(float deta_t);

};

#endif

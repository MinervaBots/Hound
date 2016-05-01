#include "locator.h"

#define MPU_UPDATE_RATE 20

//  MAG_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the magnetometer data
//  MAG_UPDATE_RATE should be less than or equal to the MPU_UPDATE_RATE
#define MAG_UPDATE_RATE 10

//  MPU_LPF_RATE is the low pas filter rate and can be between 5 and 188Hz
#define MPU_LPF_RATE 40

#define  MPU_MAG_MIX_GYRO_AND_MAG 10

Locator::Locator(Stream *encoder_stream, Position initial_position):
	encoder_list(encoder_stream) {
		encoder_list.addEncoder(&front_left_encoder);
		encoder_list.addEncoder(&front_right_encoder);
		robot_linear_speed = 0;
		robot_angular_speed = 0;

		last_robot_linear_speed = 0;
		last_robot_angular_speed = 0;
		last_position = initial_position;
		last_update_time = 0;

		rps[0] = 0;
		rps[1] = 0;

		initial_euler_radians = 0;
		// Wire.begin();

}


Locator::~Locator() {}

void Locator::initMPU() {
	// Wire.begin();
	MPU.init(MPU_UPDATE_RATE, MPU_MAG_MIX_GYRO_AND_MAG, MAG_UPDATE_RATE, MPU_LPF_RATE);
}

void Locator::start() {
	encoder_list.start();
}

	//getters
const EncoderList& Locator::getEncoderList() const {
	return encoder_list;
}
const SimpleEncoder& Locator::getFrontLeftEncoder() const {
	return front_left_encoder;
}
const SimpleEncoder& Locator::getFrontRightEncoder() const {
	return front_right_encoder;
}
float Locator::getLastRobotAngularSpeed() const {
	return last_robot_angular_speed;
}
float Locator::getLastRobotLinearSpeed() const {
	return last_robot_linear_speed;
}
unsigned long Locator::getLastUpdateTime() const {
	return last_update_time;
}
float Locator::getRobotAngularSpeed() const {
	return robot_angular_speed;
}
float Locator::getRobotLinearSpeed() const {
	return robot_linear_speed;
}
float Locator::getRightSpeed() const {
	return right_speed;
}
float Locator::getLeftSpeed() const {
	return left_speed;
}

void Locator::readMPU() {
	// Serial.println("Reading mpu");
	if(MPU.read()) {
		euler_radians[0] = MPU.m_dmpEulerPose[0];
		euler_radians[1] = MPU.m_dmpEulerPose[1];
		euler_radians[2] = MPU.m_dmpEulerPose[2];
	}
}

void Locator::update() {
	//Get the current values
	encoder_list.read();
	unsigned long now = millis();
	float delta_t = now - last_update_time;
	int delta_pulses[2];

	for (int i=0; i < 2; i++){
		delta_pulses[i] = encoder_list.get(i)->getDeltaPulses();
		//Calculate rps (in rotations)
		rps[i] = delta_pulses[i];
		rps[i] /= delta_t;
		rps[i] /= PULSES_PER_ROTATION;
		rps[i] *= 1000;
	}

	//Calculate speed
	calculateSpeeds(rps);
	calcutePosition(delta_t / 1000.0); //Update position and velocity

	//Update values
	last_update_time = now;
}


void Locator::reset(Position new_position) {
	encoder_list.reset();
	mpu_first_time = true;

	robot_linear_speed = 0;
	robot_angular_speed = 0;

	last_robot_linear_speed = 0;
	last_robot_angular_speed = 0;
	last_position = new_position;
	last_update_time = 0;
}

Position* Locator::getLastPosition(){
	return &last_position;
}


void Locator::calculateSpeeds(float rps[]){
	left_speed = rps[0] * (2*PI); // in [RAD PER SEC]
	right_speed = rps[1] * (2*PI);// in [RAD PER SEC]

	// Linear Speed = (R/2pi)*(r_speed + l_speed)/2,
	// where r_speed and l_speed are in [RAD PER SEC]
	robot_linear_speed = (WHEEL_RADIUS)*(rps[1] + rps[0])/2; // [m/s]

	// Angular Speed = (R/2pi)*(r_speed - l_speed)/(2L),
	// where r_speed and l_speed are in [RAD PER SEC]
	robot_angular_speed = (WHEEL_RADIUS)*(rps[1] - rps[0])/(2*DISTANCE_FROM_RX); // 1/s
}


void Locator::calcutePosition(float dT){
	// float med_angular_speed = (robot_angular_speed + last_robot_angular_speed)/2;
	// float theta = last_position.getTheta() + med_angular_speed*dT;

	float theta = euler_radians[2] - initial_euler_radians;
	// theta *= DEG_TO_RAD;

	// float med_linear_speed = (robot_linear_speed + last_robot_linear_speed)/2;
	float med_linear_speed = robot_linear_speed;

	float x = last_position.getX() + med_linear_speed*cos(theta)*dT;//angulo em radiano
	float y = last_position.getY() + med_linear_speed*sin(theta)*dT;//angulo em radiano

	//Update values
	last_position.set(x, y, theta);
	last_robot_linear_speed = robot_linear_speed;
	last_robot_angular_speed = robot_angular_speed;
}

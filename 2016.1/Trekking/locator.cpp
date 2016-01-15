#include "locator.h"

#define MPU_UPDATE_RATE  (20)

//  MAG_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the magnetometer data
//  MAG_UPDATE_RATE should be less than or equal to the MPU_UPDATE_RATE

#define MAG_UPDATE_RATE  (10)

//  MPU_LPF_RATE is the low pas filter rate and can be between 5 and 188Hz

#define MPU_LPF_RATE   40

#define  MPU_MAG_MIX_GYRO_AND_MAG       10

Locator::Locator(Stream *encoder_stream, Position initial_position):
	encoder_list(encoder_stream) {
		encoder_list.addEncoder(&front_left_encoder);
//		encoder_list.addEncoder(&back_left_encoder);
		encoder_list.addEncoder(&front_right_encoder);
//		encoder_list.addEncoder(&back_right_encoder);
		robot_linear_speed = 0;
		robot_angular_speed = 0;

		last_robot_linear_speed = 0;
		last_robot_angular_speed = 0;
		last_position = initial_position;
		last_update_time = 0;

		rps[0] = 0;
		rps[1] = 0;
//		rps[2] = 0;
//		rps[3] = 0;
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
//const SimpleEncoder& Locator::getBackLeftEncoder() const {
//	return back_left_encoder;
//}
//const SimpleEncoder& Locator::getBackRightEncoder() const {
//	return back_right_encoder;
//}
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

void Locator::readMPU() {
	// Serial.println("Reading mpu");
	if(MPU.read()) {
		euler_radians[0] = MPU.m_dmpEulerPose[0];// * 180/PI;
		euler_radians[1] = MPU.m_dmpEulerPose[1];// * 180/PI;
		euler_radians[2] = MPU.m_dmpEulerPose[2];// * 180/PI + 180;//avoid negative angles
	}

	// if(mpu_first_time) {
	// 	initial_euler_radians = euler_radians[2];
	// 	mpu_first_time = false;
	// }
	//MPU.printAngles(MPU.m_dmpEulerPose);
	// Serial.print(euler_radians[0]);
	// Serial.print("\t");
	// Serial.print(euler_radians[1]);
	// Serial.print("\t");
	// Serial.print(euler_radians[2]);
	// Serial.print("\t");
	// Serial.println();

}

void Locator::update() {
	//Get the current values
	encoder_list.read();
	unsigned long now = millis();
	float delta_t = now - last_update_time;
	int delta_pulses[2];

	// Serial.print(delta_t);
	// Serial.print(" ");

	for (int i=0; i < 2; i++){
		delta_pulses[i] = encoder_list.get(i)->getDeltaPulses();
		//Calculate rps
		rps[i] = delta_pulses[i];
		// Serial.print(rps[i]);
		// Serial.print('\t');
		rps[i] /= delta_t;
		// Serial.print(rps[i]);
		// Serial.print('\t');
		rps[i] /= PULSES_PER_ROTATION;
		// Serial.print(rps[i]);
		// Serial.print('\t');
		rps[i] *= 1000;
		// Serial.print(rps[i]);
		// Serial.print('\t');
		// Serial.print(delta_pulses[i]);
		// Serial.print('\t');


		// Serial.print(" ");
	}
	// Serial.print(delta_t);
	// Serial.print('\t');
	// Serial.print(PULSES_PER_ROTATION);
	// Serial.print('\t');
	// Serial.print(1000 / (delta_t * PULSES_PER_ROTATION));
	// Serial.print('\t');
	// Serial.print(1000 * delta_pulses[0]/ (delta_t * PULSES_PER_ROTATION));	
	// Serial.println();
	
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
	left_speed = rps[0] * PI * WHEEL_RADIUS; // em [m/s]
	right_speed = rps[1] * PI * WHEEL_RADIUS;// em [m/s]
	
	robot_linear_speed = (right_speed + left_speed)/2;// m/s
	robot_angular_speed = (right_speed - left_speed)/DISTANCE_FROM_RX; // rad/s

	// Serial.print(left_speed);
	// Serial.print("\t");
	// Serial.print(right_speed);
	// Serial.print("\t");
	// Serial.print(robot_linear_speed);
	// Serial.print("\t");
	// Serial.print(robot_angular_speed);
	// Serial.println();
}


void Locator::calcutePosition(float dT){
	// float med_angular_speed = (robot_angular_speed + last_robot_angular_speed)/2;
	// float theta = last_position.getTheta() + med_angular_speed*dT;

	float theta = euler_radians[2] - initial_euler_radians;
	// theta *= DEG_TO_RAD;

	float med_linear_speed = (robot_linear_speed + last_robot_linear_speed)/2;
	
	float x = last_position.getX() + med_linear_speed*cos(theta)*dT;//angulo em radiano
	float y = last_position.getY() + med_linear_speed*sin(theta)*dT;//angulo em radiano


	// Serial.print(x);
	// Serial.print("\t");
	// Serial.print(last_position.getX());
	// Serial.print("\t");
	// Serial.print(med_linear_speed);
	// Serial.print("\t");
	// Serial.print(cos(theta));
	// Serial.print("\t");
	// Serial.print(theta);
	// Serial.print("\t");
	// Serial.print(dT);
	// Serial.print("\t");
	// Serial.print(med_linear_speed);
	// Serial.print('\t');
	// Serial.print(robot_linear_speed);
	// Serial.print('\t');
	// Serial.print(last_robot_linear_speed);
	// Serial.print('\t');

	// Serial.print(x);
	// Serial.print('\t');
	// Serial.print(y);
	// Serial.print('\t');
	// Serial.print(theta);
	
	// Serial.print('\t');
	// Serial.println(med_angular_speed);
	Serial.println();

	//Update values
	last_position.set(x, y, theta);
	last_robot_linear_speed = robot_linear_speed;
	last_robot_angular_speed = robot_angular_speed;
}

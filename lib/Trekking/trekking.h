#ifndef TREKKING_H
#define TREKKING_H


#include "TrekkingPins.h"
#include "TrekkingMath.h"
#include "Position.h"
#include "DuoDriver.h"

#include "../PID/PID.h"
#include "../SensorArray/SensorArray.h"

#include "../Log/Log.h"
#include "../LinkedList/LinkedList.h"
#include "../Robot/Robot.h"
#include "../Timer/Timer.h"
#include "../TCS230/TCS230.h"
#include "../MPU9150Lib/MPU9150Lib.h"

/*
//Deixa isso aqui porque aparentemente sem esse #include (mesmo que comentado), ele falha ao compilar
#include <inv_mpu.h>
//*/

class Trekking : public Robot
{
public:
	Trekking(float safety_factor, DuoDriver* driver, PID pidController, SensorArray *pSensorArray);
	~Trekking();

	void addTarget(Position *target);
	void start();
	void update();
	void emergency();

	/*----|Test related functions|-------------------------------------------*/
	void goStraight(bool enable_pid);
	void doCircle(bool enable_pid);
	void goStraightWithControl(float meters);

	void printSonarInfo();
	void printEncodersInfo();
	void printRotations();
	void printMPUInfo();
	void printAccelInfo();
	void printPosition();
	void printVelocities();
	void printGyroInfo();
	void printTime();
	void printColorsInfo();
	void finishLogLine();

private:
	PID m_PidController;
	SensorArray *m_pSensorArray;
	const char DELIMITER;
	const float GEAR_RATE;
	const float PULSES_PER_ROTATION;
	const float WHEEL_RADIUS;
	const float TWO_PI_R;
	const float MAX_RPM;
	const float MAX_RPS;
	const float MAX_PPS;

	const float SAFE_RPM;
	const float SAFE_RPS;
	const float SAFE_PPS;

	const float DISTANCE_FROM_RX; // = distancia entre a roda e o eixo sagital do robo


	//Velocities
	const float MAX_LINEAR_VELOCITY;  // [m/s]
	const float MAX_ANGULAR_VELOCITY; // [ang/s]

	//White color parameter for Color Sensors
	const int WHITE_VALUE;

	//Motors
/*
	const byte MAX_MOTOR_PWM;
	const byte MIN_MOTOR_PWM;

	const int COMMAND_BAUD_RATE;
	const int LOG_BAUD_RATE;
	const int ENCODER_BAUD_RATE;
//*/
	const int MPU_UPDATE_RATE; //defines the rate (in Hz) at which the MPU updates the magnetometer data
	const int MAG_UPDATE_RATE; // should be less than or equal to the MPU_UPDATE_RATE
	const int MPU_LPF_RATE; // is the low pas filter rate and can be between 5 and 188Hz
	const int  MPU_MAG_MIX_GYRO_AND_MAG;

	const int LIGHT_DURATION;
	//const float PROXIMITY_RADIUS;

	//const int READ_ENCODERS_TIME; //-
	const int READ_MPU_TIME;

	//const float G_FACTOR; //-

	//Color Sensors
	TCS230 right_color;
	TCS230 center_color;
	TCS230 left_color;

	LinkedList<Position *> targets;
	//LinkedList<Position *> obstacles;

	Position init_position;

	float desired_linear_velocity;
	float desired_angular_velocity;

	int current_target_index;
	//int current_partial_index;

	//Input states
	bool init_button;
	bool emergency_button;
	bool operation_mode_switch;
	/*
	int min_distance_to_enable_lights;
	int min_distance_to_refine_search;
	bool is_aligned;
	//*/

	char current_command;

	bool sirene_is_on;
	bool is_tracking;


	float distance_to_target;

	DuoDriver *driver;

	Log log;

	//Holds witch is the serial stream to receive the commands
	Stream *command_stream;
	//Stream *log_stream;
	//Stream *encoder_stream;

	//Timers
	TimerForMethods<Trekking> mpu_timer;
	TimerForMethods<Trekking> sirene_timer;
	// TimerForMethods<Trekking> tracking_regulation_timer;
	TimerForMethods<Trekking> calibrate_angle_timer;
	//Timer control_clk; //-
	float elapsed_time;
/*
	float kp_right, ki_right, kd_right, bsp_right;
	float kp_left, ki_left, kd_left, bsp_left;
	PIDControler right_pid;
	PIDControler left_pid;
	float left_vel_ref, right_vel_ref;
	float pid_convertion_const;
//*/
	float euler_radians[3];
	//float last_euler_radians[3];
/*
	double sonars[3];// esquerda,direita,centro
	double last_sonars[3];
	bool first_sonars_sample;

//*/
	//bool first_mpu_sample;
	float initial_euler_radians;
	MPU9150Lib MPU;
	//bool mpu_first_time;
	bool is_auto_message_sent =false;
	bool is_manual_message_sent =false;

	float l_rotations_per_sec, r_rotations_per_sec;
	Position current_position;
	unsigned long last_update_time;
	//unsigned long last_update_time_2;
	Position *q_desired;
	/*
	float kp;
	float ki;
	float kd;
	//*/
	bool is_testing_refinedSearch = false;
	bool is_testing_openloop = false;
	bool is_testing_search = false;
	bool is_testing_cartesian = false;
	bool is_testing_trackTrajectory = false;

	float tested_pps;

	float accel[3];
	float Wb[3];
	float accel_offset[3];
	float last_accel[3];

	float tracking_time;

	float last_desired_v;
/*
	bool correcao;
	float last_desired_refined_v;
	float last_desired_refined_w;
	float integral_error_v;
	float integral_error_w;

	bool is_turning_right = false;
	bool is_turning_left = false;
	bool is_turning = false;
//*/

	/*----|Matlab related functions|-----------------------------------------*/
	Position plannedPosition(bool is_trajectory_linear, unsigned long t);
	void controlMotors(float v, float w, bool enable_pid, float dT);
	void simpleControlMotors(float v, float w);
	void controlMotors2(float v, float w, bool enable_pid, float dT);
	void trackTrajectory(Position* q_desired, float tracking_time);
	float regulateControl(Position* q_desired, float dT);
	void cartesianControl(Position* q_desired, float dT);

	/*----|Position update related functions|---------------------------------*/
	void updatePosition(float dT);
	void resetPosition(Position new_position);
	void readMPU();
	bool readColors();
	void updateSpeeds();
	float ppsToRps(int32_t pps);
	float getRightSpeed();
	float getLeftSpeed();
	float getLinearSpeed();
	float getAngularSpeed();
	float getLastUpdateTime();



	/*----|Operation modes|--------------------------------------------------*/
	void standby(float dT);
	void search(float dT);
	void refinedSearch(float dT);
	void lighting(float dT);
	void (Trekking::*operation_mode)(float);

	/*----|Operation functions|----------------------------------------------*/
	void goToNextTarget();
	void reset();
	void stop();
	void readInputs();
	void turnOnSirene();
	void turnOffSirene();

	/*----|Timer functions|--------------------------------------------------*/
	void startTimers();
	void stopTimers();
	void resetTimers();
	void updateTimers();

	/*----|Auxiliar functions|-----------------------------------------------*/
	// checks inputs and updates timers for each Arduino's outer-loop iteration
	void loopCheck();
	bool checkSensors(); // returns 1 if all the sensors are working
	void calibrateAngle();
	void debug();

};
#endif //TREKKING_H

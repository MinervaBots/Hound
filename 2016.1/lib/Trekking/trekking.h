#ifndef TREKKING_H
#define TREKKING_H

#include "../Log/log.h"
#include "../XLMaxSonarEZ/sonarlist.h"
#include "../Robot/Robot.h"
// #include "../Timer/timer.h"
#include "trekkingpins.h"
#include "trekkingmath.h"
#include "position.h"
#include "locator.h"
#include "PIDControler.h"
#include "../Robot/DualDriver.h"
#include "DuoDriver.h"

#define LIGHT_ON 		'l'
#define LIGHT_OFF 		'o'

class Trekking : public Robot{
public:
	Trekking(float max_linear_velocity, float max_angular_velocity, DualDriver* driver);
	~Trekking();

	void addTarget(Position *target);
	void start();
	void update();
	void emergency();

	/*----|Test related functions|-------------------------------------------*/
	void goStraight(bool enable_pid);
	void doCircle(bool enable_pid);
	void goStraightWithControl(float meters);


private:
	const float MAX_LINEAR_VELOCITY;  // [m/s]
	const float MAX_ANGULAR_VELOCITY; // [ang/s]

	const float MAX_SONAR_DISTANCE;
	const float MIN_SONAR_DISTANCE; // [m]

	float desired_linear_velocity;
	float desired_angular_velocity;

	LinkedList<Position *> targets;
	Position init_position;
	int current_target_index;

	const int COMMAND_BAUD_RATE;
	const int LOG_BAUD_RATE;
	const int ENCODER_BAUD_RATE;

	const byte MAX_MOTOR_PWM;
	const byte MIN_MOTOR_PWM;
	const float MAX_RPS;
	const int LIGHT_DURATION;
	const float PROXIMITY_RADIUS;

	const int READ_ENCODERS_TIME;
	const int READ_MPU_TIME;

	/*
		Input states
	*/

	bool init_button;
	bool emergency_button;
	bool operation_mode_switch;

	int min_distance_to_enable_lights;
	int min_distance_to_refine_search;

	bool is_aligned;

	char current_command;

	bool sirene_is_on;
	bool is_tracking;

	float distance_to_target;

	// Robot robot;

	//Sonars
	SonarList sonar_list;
	XLMaxSonarEZ right_sonar;
	XLMaxSonarEZ left_sonar;
	XLMaxSonarEZ center_sonar;

	Log log;
	// Kalman kalman;
	// Radio radio;

	//Holds witch is the serial stream to receive
	//the commands
	Stream *command_stream;
	Stream *log_stream;
	Stream *encoder_stream;

	Locator locator;

	//Timers
	TimerForMethods<Locator> encoders_timer;
	TimerForMethods<Locator> mpu_timer;
	TimerForMethods<Trekking> sirene_timer;
	TimerForMethods<Trekking> tracking_regulation_timer;
	TimerForMethods<Trekking> calibrate_angle_timer;

	float kp_right, ki_right, kd_right, bsp_right;
	float kp_left, ki_left, kd_left, bsp_left;
	PIDControler right_pid;
	PIDControler left_pid;
	float left_vel_ref, right_vel_ref;
	float pid_convertion_const;


	/*----|Matlab related functions|-----------------------------------------*/
	Position plannedPosition(bool is_trajectory_linear, unsigned long t);
	void controlMotors(float v, float w, bool enable_pid);
	void trackTrajectory();
	void regulateControl();

	/*----|Operation modes|--------------------------------------------------*/
	void standby();
	void search();
	void refinedSearch();
	void lighting();
	void (Trekking::*operation_mode)(void);

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

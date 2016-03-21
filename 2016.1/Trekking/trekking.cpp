#include "trekking.h"
#include "trekkingpins.h"

/*----|Public|---------------------------------------------------------------*/
Trekking::Trekking(float max_linear_velocity, float max_angular_velocity):
	//Velocities
	MAX_LINEAR_VELOCITY(max_linear_velocity),
	MAX_ANGULAR_VELOCITY(max_angular_velocity),

       //Distances for Ultrasound
       MAX_SONAR_DISTANCE(500),
       MIN_SONAR_DISTANCE(10),

	//Motors
	MAX_MOTOR_PWM(130),
	MIN_MOTOR_PWM(100),
	MAX_RPS(3000),
	robot(R_ENABLE_PIN,R_MOTOR_1_PIN,R_MOTOR_2_PIN,
		  L_ENABLE_PIN,L_MOTOR_1_PIN,L_MOTOR_2_PIN),

	COMMAND_BAUD_RATE(9600),
	LOG_BAUD_RATE(9600),
	ENCODER_BAUD_RATE(57600),

	LIGHT_DURATION(3000),
	PROXIMITY_RADIUS(3.0),

	READ_ENCODERS_TIME(30),
	READ_MPU_TIME(30),

	right_sonar(RIGHT_SONAR_TX_PIN, RIGHT_SONAR_RX_PIN),
	left_sonar(LEFT_SONAR_TX_PIN, LEFT_SONAR_RX_PIN),
	center_sonar(CENTER_SONAR_TX_PIN, CENTER_SONAR_RX_PIN),

	sonar_list(Sonar::CHAIN),

	targets(),
	init_position(),
	encoder_stream(&Serial2),
	locator(encoder_stream, Position(0,0,0)),

	//Timers
	encoders_timer(&locator, &Locator::update),
	sirene_timer(this, &Trekking::goToNextTarget),
	tracking_regulation_timer(this, &Trekking::trackTrajectory),
	mpu_timer(&locator, &Locator::readMPU),
	calibrate_angle_timer(this, &Trekking::calibrateAngle)
{
	//Streams
	command_stream = &Serial; //bluetooth on Serial1
	log_stream = &Serial;

	log.setTarget(log_stream);

	robot.setMinPWM(80, 80);

	//MUST CHECK THE RIGHT ORDER ON THE BOARD
	sonar_list.addSonar(&left_sonar);
	sonar_list.addSonar(&center_sonar);
	sonar_list.addSonar(&right_sonar);

	//Timers
	mpu_timer.setInterval(READ_MPU_TIME);
	mpu_timer.start(); //Must read the mpu all the time

	encoders_timer.setInterval(READ_ENCODERS_TIME);
	sirene_timer.setTimeout(LIGHT_DURATION);
	tracking_regulation_timer.setInterval(READ_ENCODERS_TIME);
	calibrate_angle_timer.setTimeout(20000);
	calibrate_angle_timer.start();

	kp_left = 7.2;  kp_right = 7.2;
	ki_left = 8;    ki_right = 8;
	kd_left = 0;    kd_right = 0;
	bsp_left = 0.8; bsp_right = 0.8;

	pid_convertion_const = (1000 * 2 * 3.1415) / 1024.0;

	right_pid.Init(kp_right, kd_right, ki_right, bsp_right);
	left_pid.Init(kp_left, kd_left, ki_left, bsp_left);

	reset();
}

Trekking::~Trekking() {
	log.info("memory management", "freeing memory");

	for(int i = targets.size() - 1; i >= 0; i--) {
		Position *target = targets.remove(0);
		log.info("memory management", (int)target);
		log.info("target", "removed target");
		delete target;
	}
	log.info("memory management", "done");
}

void Trekking::addTarget(Position *target) {
	targets.add(target);
	log.info("target", "added target");
}

void Trekking::start() {
	emergency();
	//The alert led turns off when MPU is ready
	locator.initMPU();

}

void Trekking::update() {
	readInputs();   //Read all the inputs
	updateTimers(); //Update all the timers
	current_command = ' ';

	//Read the serial
	if(command_stream->available()) {
		current_command = command_stream->read();
		log.debug("received command", current_command);

	} else {//Read the radio
	}

	//Stop the robot if the emergency button is pressed or
	//the stop command was received
	if(current_command == 'D' || emergency_button) {
		emergency();
	}

	debug(); //Debug
	(this->*operation_mode)(); //Call the current operation
}

void Trekking::emergency() {
	stop();
	stopTimers();
	operation_mode = &Trekking::standby;
	log.assert("operation mode", "standby");
	current_command = ' ';
}

/*----|Public: Test related functions|---------------------------------------*/
void Trekking::goStraight(bool enable_pid){
	float v = 1;
	float w = 0;

	controlMotors(v, w, enable_pid);

// 	if(!is_tracking) {
// 		log.debug("TEST", "Go Straight");
// 		is_tracking = true;
//
// 		log << DEBUG << "" << log_endl;
// 		log << "\t" << "t";
//
// 		log << "\t" << "R_X"; //Robot X
// 		log << "\t" << "R_Y"; //Robot Y
//
// 		log << "\t" << "v"; //Lin Velocity
// 		log << "\t" << "w"; //Ang Velocity
//
// 		log << log_endl;
// 	}
//
// 	log << DEBUG << "" << log_endl;
// 	log << "\t" << t;
//
// 	log << "\t" << trekking_position->getX();
// 	log << "\t" << trekking_position->getY();
//
// 	log << "\t" << v;
// 	log << "\t" << w;
//
// 	log << log_endl;
}

void Trekking::doCircle(bool enable_pid){
	controlMotors(1, 1, enable_pid);

// 	if(!is_tracking) {
// 		log.debug("TEST", "Go Straight");
// 		is_tracking = true;
//
// 		log << DEBUG << "" << log_endl;
// 		log << "\t" << t;
//
// 		log << "\t" << "R_X"; //Robot X
// 		log << "\t" << "R_Y"; //Robot Y
//
// 		log << "\t" << "v"; //Lin Velocity
// 		log << "\t" << "w"; //Ang Velocity
//
// 		log << log_endl;
// 	}
//
// 	log << DEBUG << "" << log_endl;
// 	log << "\t" << t;
//
// 	log << "\t" << trekking_position->getX();
// 	log << "\t" << trekking_position->getY();
//
// 	log << "\t" << v;
// 	log << "\t" << w;
//
// 	log << log_endl;
}


void Trekking::goStraightWithControl(bool is_virtual_test, float meters){
// 	if(!is_tracking) {
// 		log.debug("TEST", "Go Straight");
// 		startTimers();
// 		is_tracking = true;
//
// 		log << DEBUG << "" << log_endl;
// 		log << "\t" << t;
//
// 		log << "\t" << "P_X"; //Planned X
// 		log << "\t" << "P_Y"; //Planned Y
//
// 		log << "\t" << "R_X"; //Robot X
// 		log << "\t" << "R_Y"; //Robot Y
//
// 		log << "\t" << "v"; //Lin Velocity
// 		log << "\t" << "w"; //Ang Velocity
//
// 		log << log_endl;
//
//
// 	}
	Position* trekking_position = locator.getLastPosition();
	float v, w = 0;
	Position* destination = targets.get(current_target_index);
	destination->set(meters, 0.0, 0.0);
	unsigned long t = tracking_regulation_timer.getElapsedTime();
	Position planned_position = plannedPosition(true, t);
	Position gap = trekking_position->calculateGap(planned_position);
// 	trackTrajectory();

// 	if(is_virtual_test){
// 		log << DEBUG << "" << log_endl;
// 		log << "\t" << t;
//
// 		log << "\t" << planned_position->getX();
// 		log << "\t" << planned_position->getY();
//
// 		log << "\t" << trekking_position->getX();
// 		log << "\t" << trekking_position->getY();
//
// 		log << "\t" << v;
// 		log << "\t" << w;
//
// 		log << log_endl;
//
// 	}else{
//
// 		controlMotors(v, w, true);
// 	}
}




/*----|Private: Matlab related functions|------------------------------------*/
Position Trekking::plannedPosition(bool is_trajectory_linear, unsigned long tempo){
	Position* destination = targets.get(current_target_index);
// 	Position* trekking_position = locator.getLastPosition();
	Position planned_position = Position();
	float t = (float) tempo;
	t /= 1000; //Mills to sec

	float dirx = 1;
	float diry = 1;

	// Verificando a direcao do caminho
	if (init_position.getX() > destination->getX()){
		dirx = -1;
	}
	if (init_position.getY() > destination->getY()){
		diry = -1;
	}
	float planned_distance = desired_linear_velocity * t;

	float p_x = init_position.getX() + planned_distance*cos(init_position.getTheta())*dirx;
	float p_y = init_position.getY() + planned_distance*sin(init_position.getTheta())*diry;
	float p_theta = init_position.getTheta();

	// A posicao planejada nao pode estar depois da posicao destino
	// X
	if (init_position.getX() > destination->getX()){
		p_x = max(destination->getX(),p_x);
	}
	else{
		p_x = min(destination->getX(),p_x);
	}
	// Y
	if (init_position.getY() > destination->getY()){
		p_x = max(destination->getX(),p_y);
	}
	else{
		p_x = min(destination->getX(),p_y);
	}

	planned_position.set(p_x, p_y, p_theta);

	return planned_position;
}

void Trekking::controlMotors(float v, float w, bool enable_pid){
	// v velocity in m/s and w the angle in rad/s

	//Calculating rps
	float right_desired_vel = (2*v + w*DISTANCE_FROM_RX);///(2*WHEEL_RADIUS);//[RPS]
	float left_desired_vel = (2*v - w*DISTANCE_FROM_RX);///(2*WHEEL_RADIUS);//[RPS]

	//PID
	float right_vel = 0;
	float left_vel = 0;
	if(enable_pid){
		right_vel = floor(right_pid.run(
			abs(right_desired_vel), locator.getRightSpeed() ));

		left_vel = floor(left_pid.run(
			abs(left_desired_vel), locator.getLeftSpeed() ));
	}
	else {
		right_vel = right_desired_vel;
		left_vel = left_desired_vel;
	}
	// calculating pwm
	byte right_pwm = right_vel*MAX_MOTOR_PWM/MAX_RPS;
	byte left_pwm = left_vel*MAX_MOTOR_PWM/MAX_RPS;

	bool right_reverse = (right_vel < 0);
	bool left_reverse = (left_vel < 0);

	if(right_pwm < MIN_MOTOR_PWM && !nearEquals(right_pwm, 0, 5)) {
		right_pwm = MIN_MOTOR_PWM;
	}
	if(left_pwm < MIN_MOTOR_PWM && !nearEquals(left_pwm, 0, 5)) {
		left_pwm = MIN_MOTOR_PWM;
	}

	// Setting PWM
	robot.setRPWM(right_pwm, right_reverse);
	robot.setLPWM(left_pwm, left_reverse);

	// log << DEBUG << "" << log_endl;
	// log << "\t" << right_desired_vel;
	// log << "\t" << left_desired_vel;

	// log << "\t" << locator.right_speed;
	// log << "\t" << locator.left_speed;

	// log << "\t" << right_pid_out;
	// log << "\t" << left_pid_out;

	// log << "\t" << right_pwm;
	// log << "\t" << left_pwm;
	// log << log_endl;

	/****** OLD *******/
	//calculating pwm
	// byte right_pwm = right_pid_out*MAX_MOTOR_PWM/MAX_RPS;
// 	byte right_pwm = right_pid_out;
	// bool right_reverse = (right_pid_out < 0);
// 	bool right_reverse = false;
// 	byte left_pwm = left_pid_out;
	// byte left_pwm = left_pid_out*MAX_MOTOR_PWM/MAX_RPS;
	// bool left_reverse = (left_pid_out < 0);
// 	bool left_reverse = false;

	// right_pwm = pidOut;


}

void Trekking::trackTrajectory() {
	bool linear = true;
	if(linear){
		desired_linear_velocity = MAX_LINEAR_VELOCITY;
		desired_angular_velocity = 0;
	}else{
		desired_linear_velocity = 0;
		desired_angular_velocity = MAX_ANGULAR_VELOCITY;
	}

	//constants to handle errors
	const int k1 = 2;
	const int k2 = 1;
	const int k3 = 2;

	Position* trekking_position = locator.getLastPosition();
	unsigned long t = tracking_regulation_timer.getElapsedTime();
	Position gap = trekking_position->calculateGap(plannedPosition(linear, t));

	//Angular transformation
	float e1 = gap.getX();
	e1 *= cos(trekking_position->getTheta());
	e1 += gap.getY()*sin(trekking_position->getTheta());

	float e2 = -gap.getX();
	e2 *= sin(trekking_position->getTheta());
	e2 += gap.getY()*cos(trekking_position->getTheta());
	float e3 = gap.getTheta();

	//Calculating linear velocity and angular velocity
	float v = desired_linear_velocity*cos(e3) + k1*e1;
	float w = desired_angular_velocity + k2*e2 + k3*e3;

	controlMotors(v, w, true);
	distance_to_target = trekking_position->distanceFrom(targets.get(current_target_index));
}


/*----|Private: Operations modes|--------------------------------------------*/
void Trekking::standby() {
	if(operation_mode_switch == AUTO_MODE) {
		// log.debug("mode switch", "auto");
		if(init_button) {
			// log.debug("init button", init_button);
			emergency();
			reset();
			if(checkSensors()) {
				calibrateAngle();
				operation_mode = &Trekking::search;
				startTimers();
				locator.start();
				log.assert("operation mode", "search stage");
			} else {
				log.error("sensors", "sensors not working as expected");
			}
		}
	} else if(current_command != ' ') {
		robot.useCommand(current_command);
		log.debug("using command", current_command);
	}
}

void Trekking::search() {
	if(!is_tracking) {
		log.debug("Search", "starting tracking timer");
		tracking_regulation_timer.start();
		is_tracking = true;
	}
	//Colocar a condicao de proximidade
	if(distance_to_target < PROXIMITY_RADIUS) {
	 	tracking_regulation_timer.stop();
	 	tracking_regulation_timer.reset();
	 	log.assert("operation mode", "refined search");
	 	operation_mode = &Trekking::refinedSearch;
	}
}

void Trekking::refinedSearch() {
       //MAX_SONAR_DISTANCE
       //MIN_SONAR_DISTANCE
       // MAX_LINEAR_VELOCITY and angular....

	float maxD,minD;
	float V,W;
					//  These constants must be tested and defined. maxD is the maximun distance read
					// by the ultrasound sensors and minD the minimum. V and W are the parameters to
					// control the motors.

	sonar_list.read();
	float c,l,r;
	c=center_sonar.getDistance();
	l=left_sonar.getDistance();
	r=right_sonar.getDistance();

	if (c<minD){
		operation_mode = &Trekking::lighting;
		controlMotors(0,0,false);
	}
	else if (c<maxD && l>maxD && r>maxD){
		controlMotors(V,0,true);
	}
	else if (r<maxD){
		controlMotors(V,-W,true);
	}
	else if (l<maxD){
		controlMotors(V,W,true);
	}
	else if (c>maxD && l>maxD && r>maxD){
		controlMotors(0,W,true);				//  Perhaps it would be best to use data from the gyroscope to
										// determine wheather to use +W or -W
	}
}

void Trekking::lighting() {
	if(!sirene_is_on) {
		sirene_timer.start();
		turnOnSirene();
	}
}


/*----|Private: Operations functions|----------------------------------------*/
void Trekking::goToNextTarget() {
	turnOffSirene();
	current_target_index ++;
	log.assert("current target", current_target_index);
	if(current_target_index >= targets.size()) {
		operation_mode = &Trekking::standby;
		log.assert("operation mode", "standby");
	} else {
		operation_mode = &Trekking::search;
		log.assert("operation mode", "search stage");
		//AQUIIIIIII
	}
}

void Trekking::reset() {
	log.assert("reset", "resetting...");

	//Trekking variables
	desired_linear_velocity = 0;
	desired_angular_velocity = 0;
	min_distance_to_enable_lights = 0;
	min_distance_to_refine_search = 0;
	current_command = ' ';
	is_aligned = false;
	is_tracking = false;
	current_target_index = 0;
	distance_to_target = locator.getLastPosition()->distanceFrom(targets.get(current_target_index));
	left_vel_ref = 0;
	right_vel_ref = 0;

	//Go to the standby state
	operation_mode = &Trekking::standby;
	log.assert("operation mode", "standby");

	//Trekking objects
	Position initial_position(0,0,0);
	locator.reset(initial_position);
	robot.setPWM(MAX_MOTOR_PWM, MAX_MOTOR_PWM);
	robot.stop();

	right_pid.reset();
	left_pid.reset();

	//Trekking methods
	stopTimers();
	resetTimers();
	turnOffSirene();

	log.assert("reset", "done.");

	readInputs();
}

void Trekking::stop() {
	log.assert("stop", "stopping robot");
	robot.stop();
}

void Trekking::readInputs() {
	init_button = digitalRead(INIT_BUTTON_PIN);
	emergency_button = digitalRead(EMERGENCY_BUTTON_PIN);
	operation_mode_switch = digitalRead(OPERATION_MODE_SWITCH_PIN);
}

void Trekking::turnOnSirene() {
	encoder_stream->println(LIGHT_ON);
	sirene_is_on = true;
}

void Trekking::turnOffSirene() {
	encoder_stream->println(LIGHT_OFF);
	sirene_is_on = false;
}


/*----|Private: Timer functions|---------------------------------------------*/
void Trekking::startTimers() {
	encoders_timer.start();
	// sirene_timer.start();
}

void Trekking::stopTimers() {
	encoders_timer.stop();
	sirene_timer.stop();
	tracking_regulation_timer.stop();
}

void Trekking::resetTimers() {
	encoders_timer.reset();
	sirene_timer.reset();
	tracking_regulation_timer.reset();
}

void Trekking::updateTimers() {
	mpu_timer.update();
	encoders_timer.update();
	sirene_timer.update();
	tracking_regulation_timer.update();
	calibrate_angle_timer.update();
}


/*----|Private: Auxiliar functions|------------------------------------------*/
bool Trekking::checkSensors() {return true;}

void Trekking::calibrateAngle() {
	locator.initial_euler_radians = locator.euler_radians[2];
	locator.getLastPosition()->setTheta(locator.euler_radians[2]);
	digitalWrite(ALERT_LED, LOW);

	Serial.print("Calibrated angle = ");
	Serial.println(locator.initial_euler_radians);
}

void Trekking::debug() {
	if(current_command == 's') {
		log.debug("debug command", "set to standby");
		operation_mode = &Trekking::standby;
		stopTimers();
	} else if(current_command == 'e') {
		log.debug("debug command", "set to search");
		// operation_mode = &Trekking::search;
		operation_mode_switch = AUTO_MODE;
		init_button = true;
		is_tracking = false;
		standby();
	} else if(current_command == 'f') {
		log.debug("debug command", "set to refined search");
		operation_mode = &Trekking::refinedSearch;

	} else if(current_command == 'l') {
		log.debug("debug command", "set to lighting");
		operation_mode = &Trekking::lighting;

	} else if(current_command == 'r') {
		log.debug("debug command", "reset");
		reset();

	} else if(current_command == 'W') {
		log.debug("debug command", "turn on sirene");
		turnOnSirene();

	} else if(current_command == 'w') {
		log.debug("debug command", "turn off sirene");
		turnOffSirene();
	} else if(current_command == 'n') {
		log.debug("debug command", "print encoders");
		for(int i = 0; i < locator.encoder_list.size(); i++) {
			Serial.print(locator.encoder_list.get(i)->getPulses());
			Serial.print("\t");
		}
		Serial.println();
	} else if(current_command == 'L') {
		log.debug("debug command", "print locator");
		log << DEBUG << "angular\tlinear\ttime" << log_endl;

		log << locator.getRobotAngularSpeed() << '\t';
		log << locator.getRobotLinearSpeed() << '\t';
		log << locator.getLastUpdateTime() << log_endl;
	} else if(current_command == 'm') {
		log.debug("debug command", "print mpu");
		log << DEBUG << "x\ty\tz" << log_endl;

		log << locator.euler_radians[0] << '\t';
		log << locator.euler_radians[1] << '\t';
		log << locator.euler_radians[2] << log_endl;
	} else if(current_command == 'o') {
		log.debug("debug command", "print sonars");
		log << DEBUG << "left\tcenter\tright" << log_endl;
		sonar_list.read();
		log << left_sonar.getDistance() << '\t';
		log << center_sonar.getDistance() << '\t';
		log << right_sonar.getDistance() << log_endl;
	} else if(current_command == 'b') {
		log.debug("debug command", "print buttons");
		log << DEBUG << "init\temergency\tmode" << log_endl;

		log << digitalRead(INIT_BUTTON_PIN) << '\t';
		log << digitalRead(EMERGENCY_BUTTON_PIN) << '\t';
		log << digitalRead(OPERATION_MODE_SWITCH_PIN) << log_endl;
	}
}

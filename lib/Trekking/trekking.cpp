#include "trekking.h"

/*----|Public|---------------------------------------------------------------*/
Trekking::Trekking(float max_linear_velocity, float max_angular_velocity,
	DuoDriver* driver_pointer):Robot(driver_pointer),

	GEAR_RATE(19),
	PULSES_PER_ROTATION(64),
	WHEEL_RADIUS(0.075),
	MAX_PPS(10133.33), // = GEAR_RATE * MAX_RPM/60 * POINTS = 19 * (500/60) * 64
	DISTANCE_FROM_RX(0.1375), // 137.5mm

	//Velocities
	MAX_LINEAR_VELOCITY(max_linear_velocity),
	MAX_ANGULAR_VELOCITY(max_angular_velocity),

	//Distances for Ultrasound
	MAX_SONAR_DISTANCE(200),
	MIN_SONAR_DISTANCE(30),

	//Motors
	MAX_MOTOR_PWM(130),
	MIN_MOTOR_PWM(100),
	MAX_RPS(8.33),

	COMMAND_BAUD_RATE(9600),
	LOG_BAUD_RATE(9600),
	ENCODER_BAUD_RATE(57600),

	MPU_UPDATE_RATE(20),
	MAG_UPDATE_RATE(10),
	MPU_LPF_RATE(40),
	MPU_MAG_MIX_GYRO_AND_MAG(10),

	LIGHT_DURATION(3000),
	PROXIMITY_RADIUS(3.0),

	READ_ENCODERS_TIME(30),
	READ_MPU_TIME(30),

	//Sonars
	right_sonar(RIGHT_SONAR_TX_PIN, RIGHT_SONAR_RX_PIN),
	left_sonar(LEFT_SONAR_TX_PIN, LEFT_SONAR_RX_PIN),
	center_sonar(CENTER_SONAR_TX_PIN, CENTER_SONAR_RX_PIN),

	sonar_list(Sonar::CHAIN),

	targets(),
	init_position(),
	encoder_stream(&Serial2),

	//Timers
	mpu_timer(this, &Trekking::readMPU),
	sirene_timer(this, &Trekking::goToNextTarget),
	tracking_regulation_timer(this, &Trekking::trackTrajectory),
	calibrate_angle_timer(this, &Trekking::calibrateAngle)
{
	//Streams
	command_stream = &Serial; //bluetooth on Serial1
	log_stream = &Serial;

	log.setTarget(log_stream);

	Robot::setMinPWM(80, 80);

	//MUST CHECK THE RIGHT ORDER ON THE BOARD
	sonar_list.addSonar(&left_sonar);
	sonar_list.addSonar(&center_sonar);
	sonar_list.addSonar(&right_sonar);

	//Timers
	// mpu_timer.setInterval(READ_MPU_TIME);
	// mpu_timer.start(); //Must read the mpu all the time

	resetPosition(Position(0,0,0));

	// encoders_timer.setInterval(READ_ENCODERS_TIME);
	sirene_timer.setTimeout(LIGHT_DURATION);
	// nao seria setTimeOut para comecar a fazer a funcao de transferencia?
	// verificar fazendo diagrama de sequencia envolvendo a chamada da funcao trackTrajectory
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

	driver = driver_pointer;
	// reset();
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
	control_clk.start();
	//The alert led turns off when MPU is ready
	MPU.init(MPU_UPDATE_RATE, MPU_MAG_MIX_GYRO_AND_MAG,
		MAG_UPDATE_RATE, MPU_LPF_RATE);
}

void Trekking::update() {
	loopCheck();
	(this->*operation_mode)(); //Call the current operation
}

void Trekking::emergency() {
	stop();
	stopTimers();
	operation_mode = &Trekking::standby;
	log.assert("operation mode", "standby");
	current_command = ' ';
}


/*----|Private: Matlab related functions|------------------------------------*/
Position Trekking::plannedPosition(bool is_trajectory_linear, unsigned long tempo){
	Position* destination = targets.get(current_target_index);
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
	// Serial.println("controling motors");
	// v velocity in m/s and w the angle in rad/s

	//Calculating rps
	float r_desired_vel = (v + w*DISTANCE_FROM_RX)*(2*PI)/WHEEL_RADIUS; //[RPS]
	float l_desired_vel = (v - w*DISTANCE_FROM_RX)*(2*PI)/WHEEL_RADIUS; //[RPS]

	//PID
	float right_vel = 0;
	float left_vel = 0;
	if(enable_pid){
		l_pwm = l_rotations_per_sec*255/MAX_PPS;
		r_pwm = l_rotations_per_sec*255/MAX_PPS;
		right_vel = floor(right_pid.run(abs(r_desired_vel), r_pwm));
		left_vel = floor(left_pid.run(abs(l_desired_vel), l_pwm));
	}
	else {
		right_vel = r_desired_vel;
		left_vel = l_desired_vel;
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
	Robot::setRPWM(right_pwm, right_reverse);
	Robot::setLPWM(left_pwm, left_reverse);
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

	unsigned long t = tracking_regulation_timer.getElapsedTime();
	Position gap = current_position.calculateGap(plannedPosition(linear, t));

	//Angular transformation
	float e1 = gap.getX();
	e1 *= cos(current_position.getTheta());
	e1 += gap.getY()*sin(current_position.getTheta());

	float e2 = -gap.getX();
	e2 *= sin(current_position.getTheta());
	e2 += gap.getY()*cos(current_position.getTheta());
	float e3 = gap.getTheta();

	//Calculating linear velocity and angular velocity
	float v = desired_linear_velocity*cos(e3) + k1*e1;
	float w = desired_angular_velocity + k2*e2 + k3*e3;

	controlMotors(v, w, true);
	distance_to_target = current_position.distanceFrom(targets.get(current_target_index));

}

void Trekking::regulateControl() {
	//constants to handle errors
	const int k1 = 1;
	const int k2 = 3;
	const int k3 = -1;

	// getting desired (final) configuration
	Position* q_desired = targets.get(current_target_index);

	// getting gap between configurations
	Position gap = current_position.calculateGap(*q_desired);

	//Angular transformation
	float rho = sqrt( pow(gap.getX(),2) + pow(gap.getY(),2) );
	float gamma = atan2(gap.getY(), gap.getX()) + gap.getTheta();
	float delta = gamma + gap.getTheta();

	//Calculating v e w
	float v = (k1 * rho) * cos(gamma);
	float w = k2*gamma + (k3*delta + gamma)*(k1*sin(gamma)*cos(gamma)/gamma);

	controlMotors(v, w, false);
	distance_to_target = rho;
}


/*----|Private: Position update related functions|---------------------------*/
void Trekking::updatePosition(){
	float delta_t = millis() - last_update_time;
	float dT = delta_t / 1000.0;

	//Get the current orientation and speeds
	readMPU();
	updateSpeeds();
	float theta = euler_radians[2] - initial_euler_radians;
	float linear_speed = getLinearSpeed();

	//Calculating new position
	float x = current_position.getX() + linear_speed*cos(theta)*dT;//angulo em radiano
	float y = current_position.getY() + linear_speed*sin(theta)*dT;//angulo em radiano

	//Update values
	current_position.set(x, y, theta);

	last_update_time = control_clk.getElapsedTime();
}

void Trekking::resetPosition(Position new_position){
	// encoder_list.reset();
	// mpu_first_time = true;

	l_rotations_per_sec = 0;
	r_rotations_per_sec = 0;

	current_position = new_position;
	control_clk.reset();
}

void Trekking::readMPU(){
	if(MPU.read()) {
		euler_radians[0] = MPU.m_dmpEulerPose[0];
		euler_radians[1] = MPU.m_dmpEulerPose[1];
		euler_radians[2] = MPU.m_dmpEulerPose[2];
	}
}

void Trekking::updateSpeeds(){
	l_rotations_per_sec = ppsToRps(driver->getLeftPPS());
	r_rotations_per_sec = ppsToRps(driver->getRightPPS());
}

//Pulses per Sec --> Rotaion Per Sec
float Trekking::ppsToRps(int32_t pps){
	// max_rps = max_pps/(PULSES_PER_ROTATION*GEAR_RATE) = 8.33 rps
	return pps/(PULSES_PER_ROTATION*GEAR_RATE);
}

float Trekking::getLinearSpeed(){
	// Linear Speed = (R)*(r_speed + l_speed)/2,
	// where r_speed and l_speed are in [ROTATIONS PER SEC]
	return (WHEEL_RADIUS)*(r_rotations_per_sec + l_rotations_per_sec)/2; // [m/s]
}

float Trekking::getAngularSpeed(){
	// Angular Speed = (R)*(r_speed - l_speed)/(2L),
	// where r_speed and l_speed are in [ROTATIONS PER SEC]
	return (WHEEL_RADIUS)*(r_rotations_per_sec - l_rotations_per_sec)/(2*DISTANCE_FROM_RX); // 1/s
}


/*----|Private: Operations modes|--------------------------------------------*/
void Trekking::standby() {
	if(operation_mode_switch == AUTO_MODE) {
		log.debug("mode switch", "auto");
		if(init_button) {
			log.debug("init button", init_button);
			emergency();
			reset();
			if(checkSensors()) {
				calibrateAngle();
				operation_mode = &Trekking::search;
				startTimers();
				log.assert("operation mode", "search stage");
			} else {
				log.error("sensors", "sensors not working as expected");
			}
		}
	} else if(current_command != ' ') {
		Robot::useCommand(current_command);
		log.debug("using command", current_command);
	}
}

void Trekking::search() {
	if(!is_tracking) {
		log.debug("Search", "starting tracking timer");
		// tracking_regulation_timer.start();
		is_tracking = true;
	}

	regulateControl();

	// Log for debug
	// printTime();
	// printPosition();
	// printVelocities();
	// printEncodersInfo();
	// finishLogLine();

	//Colocar a condicao de proximidade
	if(distance_to_target < 0.5) {
	// if(distance_to_target < PROXIMITY_RADIUS) {
	 	tracking_regulation_timer.stop();
	 	tracking_regulation_timer.reset();
	 	is_tracking = false;
	 	log.assert("operation mode", "refined search");
	 	operation_mode = &Trekking::refinedSearch;
	}
}

void Trekking::refinedSearch() {
	printEncodersInfo();
	finishLogLine();

	// data
	sonar_list.read();
	float sonars[3]; // esquerda,direita,centro
	sonars[0]=center_sonar.getDistance();
	sonars[1]=left_sonar.getDistance();
	sonars[2]=right_sonar.getDistance();
	float refLinear = MAX_LINEAR_VELOCITY/2; // it's the fastest it can go and still read the sonars well
	float minFactor = 0.5; // it's minimum linear velocity will be the reference multiplied by this factor
	float refAngular = MAX_ANGULAR_VELOCITY/2; // it's the fastest it can go and still read the sonars well
	float matrixW[] = {1, 0, -1};

	// processing
	if (sonars[1]<=MIN_SONAR_DISTANCE){
		operation_mode = &Trekking::lighting;
		controlMotors(0,0,false);
	}
	else if (sonars[0]>MAX_SONAR_DISTANCE && sonars[1]>MAX_SONAR_DISTANCE && sonars[2]>MAX_SONAR_DISTANCE){
		// WE COULD TRY TO USE THE MPU THETA DATUM TO DETERMINE TO WHICH SIDE WE SHOULD TURN!!
		controlMotors(0,refAngular,false);
	}
	else{
		float w,v;
		for (int i=0;i<3;i++){
			if (sonars[i]>MAX_SONAR_DISTANCE)
				sonars[i]=0;
			w = w + sonars[i]*matrixW[i];
			v = v + sonars[i];
		}
		w = (w*refAngular)/(sonars[0]+sonars[2]);
		v = refLinear*(exp(/*k*/-1.5*MAX_SONAR_DISTANCE/v)+minFactor); // we could add a small factor k to make it faster
		controlMotors(v,w,false);
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

	Robot::stop();
	resetPosition(init_position);
	right_pid.reset();
	left_pid.reset();
	stopTimers();
	resetTimers();
	turnOffSirene();

	//Trekking variables
	desired_linear_velocity = 0;
	desired_angular_velocity = 0;
	min_distance_to_enable_lights = 0;
	min_distance_to_refine_search = 0;
	current_command = ' ';
	is_aligned = false;
	is_tracking = false;
	current_target_index = 0;
	distance_to_target = current_position.distanceFrom(targets.get(current_target_index));
	left_vel_ref = 0;
	right_vel_ref = 0;

	//Go to the standby state
	operation_mode = &Trekking::standby;
	log.assert("operation mode", "standby");

	log.assert("reset", "done.");
	readInputs();
}

void Trekking::stop() {
	log.assert("stop", "stopping robot");
	Robot::stop();
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
	sirene_timer.start();
}

void Trekking::stopTimers() {
	sirene_timer.stop();
	tracking_regulation_timer.stop();
}

void Trekking::resetTimers() {
	sirene_timer.reset();
	tracking_regulation_timer.reset();
}

void Trekking::updateTimers() {
	// mpu_timer.update();
	// encoders_timer.update();
	sirene_timer.update();
	tracking_regulation_timer.update();
	calibrate_angle_timer.update();
}



/*----|Private: Auxiliar functions|------------------------------------------*/
void Trekking::loopCheck(){
	updatePosition();
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
}

bool Trekking::checkSensors() {return true;}

void Trekking::calibrateAngle() {
	initial_euler_radians = euler_radians[2];
	current_position.setTheta(euler_radians[2]);
	digitalWrite(ALERT_LED, LOW);

	Serial.print("Calibrated angle = ");
	Serial.println(initial_euler_radians);
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
		operation_mode_switch = AUTO_MODE;
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
		log <<	'\t' << driver->getLeftEncoder();
		log <<	'\t' << driver->getRightEncoder();
		log << log_endl;
	} else if(current_command == 'p') {
		log.debug("debug command", "print position");
		log << DEBUG << "\tx\ty\tTheta\t\tlinear\tangular\ttime" << log_endl;
		printPosition();
		printVelocities();
		finishLogLine();

	} else if(current_command == 'm') {
		log.debug("debug command", "print mpu");
		log << DEBUG << "[ANG]\talpha\tbeta\tTheta" << log_endl;
		printMPUInfo();
		finishLogLine();

	} else if(current_command == 'o') {
		log.debug("debug command", "print sonars");
		log << DEBUG << "\tleft\tcenter\tright" << log_endl;
		printSonarInfo();
		finishLogLine();

	} else if(current_command == 'b') {
		log.debug("debug command", "print buttons");
		log << DEBUG << "\tinit\temergency\tmode" << log_endl;

		log << '\t'<< digitalRead(INIT_BUTTON_PIN);
		log << '\t'<< digitalRead(EMERGENCY_BUTTON_PIN);
		log << '\t'<< digitalRead(OPERATION_MODE_SWITCH_PIN) << log_endl;
	}
}

void Trekking::printSonarInfo()
{
	sonar_list.read();
	log << '\t' << left_sonar.getDistance();
	log << '\t' << center_sonar.getDistance();
	log << '\t' << right_sonar.getDistance() << '\t';
}

void Trekking::printEncodersInfo()
{
	log << '\t' << driver->getLeftEncoder();
	log << '\t' << driver->getRightEncoder();
	log << '\t';
}

void Trekking::printMPUInfo()
{
	log << '\t' << euler_radians[0]*180/PI;
	log << '\t' << euler_radians[1]*180/PI;
	log << '\t' << euler_radians[2]*180/PI << '\t';
}

void Trekking::printPosition(){
	log << '\t' << current_position.getX();
	log << '\t' << current_position.getY();
	log << '\t' << current_position.getTheta() << '\t';
}

void Trekking::printVelocities()
{
	log << '\t' << getLinearSpeed();
	log << '\t' << getAngularSpeed() << '\t';
}

void Trekking::finishLogLine()
{
	log << log_endl;
}

void Trekking::printTime()
{
	log << '\t' << control_clk.getElapsedTime() << '\t';
}

/*----|Public: Test related functions
void Trekking::goStraight(bool enable_pid){
	loopCheck();
	controlMotors(1, 0, enable_pid);
}

void Trekking::doCircle(bool enable_pid){
	loopCheck();
	controlMotors(1, 1, enable_pid);
}

void Trekking::goStraightWithControl(float meters){
	loopCheck();

	unsigned long t = tracking_regulation_timer.getElapsedTime();
	log << DEBUG << "" << log_endl;
	log << "\t" << t;

	Position* destination = targets.get(current_target_index);
	destination->set(meters, 0.0, 0.0);

	Position planned_position = plannedPosition(true, t);
	Position gap = current_position.calculateGap(planned_position);

	if(!is_tracking) {
		log.debug("Search", "starting tracking timer");
		tracking_regulation_timer.start();
		is_tracking = true;
	}

}
|---------------------------------------*/

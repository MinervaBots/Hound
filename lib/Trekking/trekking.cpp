#include <Arduino.h>
#include "Trekking.h"

/*----|Public|---------------------------------------------------------------*/
Trekking::Trekking(float safety_factor, DuoDriver* driver_pointer, PID pidController, SensorArray *pSensorArray):
	Robot(driver_pointer),
	m_PidController(pidController),
	m_pSensorArray(pSensorArray),

	DELIMITER(';'),
	// DELIMITER('\t'),

	GEAR_RATE(70),
	PULSES_PER_ROTATION(64),
	WHEEL_RADIUS(0.075),
	TWO_PI_R(WHEEL_RADIUS*2*PI),
	MAX_RPM(150), // rotations per min
	MAX_RPS(MAX_RPM/60), // rotations per sec
	MAX_PPS(GEAR_RATE*PULSES_PER_ROTATION*MAX_RPS), // qpulses per sec

	SAFE_RPM(MAX_RPM*safety_factor),
	SAFE_RPS(MAX_RPS*safety_factor),
	SAFE_PPS(MAX_PPS*safety_factor),


	DISTANCE_FROM_RX(0.165), // 165mm


	//Velocities
	MAX_LINEAR_VELOCITY(TWO_PI_R*SAFE_RPS),
	MAX_ANGULAR_VELOCITY(TWO_PI_R*(2*SAFE_RPS)/(2*DISTANCE_FROM_RX)),

	//White color parameter for Color Sensors
	WHITE_VALUE(30),

	//Motors
	/*
	MAX_MOTOR_PWM(130),
	MIN_MOTOR_PWM(100),
	COMMAND_BAUD_RATE(9600),
	LOG_BAUD_RATE(9600),
	ENCODER_BAUD_RATE(57600),
	//*/
	MPU_UPDATE_RATE(20),
	MAG_UPDATE_RATE(10),
	MPU_LPF_RATE(40),
	MPU_MAG_MIX_GYRO_AND_MAG(10),

	LIGHT_DURATION(3000),
	//PROXIMITY_RADIUS(3.0),

	//READ_ENCODERS_TIME(30),
	READ_MPU_TIME(30),
	//G_FACTOR(160*9.8),

	//sonar_list(Sonar::CHAIN),

	//Color Sensors
	right_color(RIGHT_COLOR_S0,RIGHT_COLOR_S1,RIGHT_COLOR_S2,RIGHT_COLOR_S3,RIGHT_COLOR_OUTPUT,WHITE_VALUE),
	center_color(CENTER_COLOR_S0,CENTER_COLOR_S1,CENTER_COLOR_S2,CENTER_COLOR_S3,CENTER_COLOR_OUTPUT,WHITE_VALUE),
	left_color(LEFT_COLOR_S0,LEFT_COLOR_S1,LEFT_COLOR_S2,LEFT_COLOR_S3,LEFT_COLOR_OUTPUT,WHITE_VALUE),

	targets(),
	//obstacles(),
	init_position(0, 0, 0),
	//encoder_stream(&Serial2),

	//Timers
	mpu_timer(this, &Trekking::readMPU),
	sirene_timer(this, &Trekking::goToNextTarget),
	// tracking_regulation_timer(this, &Trekking::trackTrajectory),
	calibrate_angle_timer(this, &Trekking::calibrateAngle)
{
	driver = driver_pointer;

	//Streams
	operation_mode_switch = MANUAL_MODE;
	command_stream = &Serial1; //bluetooth on Serial1; USB on Serial
	//log_stream = &Serial1; //bluetooth on Serial1; USB on Serial
	log.setTarget(&Serial1);

	Robot::setMinPWM(80, 80);

	//MUST CHECK THE RIGHT ORDER ON THE BOARD
	//sonar_list.addSonar(&left_sonar);
	//sonar_list.addSonar(&center_sonar);
	//sonar_list.addSonar(&right_sonar);

	//Timers
	mpu_timer.setInterval(READ_MPU_TIME);
	mpu_timer.start(); //Must read the mpu all the time

	resetPosition(init_position);

	// encoders_timer.setInterval(READ_ENCODERS_TIME);
	sirene_timer.setTimeout(LIGHT_DURATION);
	// nao seria setTimeOut para comecar a fazer a funcao de transferencia?
	// verificar fazendo diagrama de sequencia envolvendo a chamada da funcao trackTrajectory
	// tracking_regulation_timer.setInterval(READ_ENCODERS_TIME);
	calibrate_angle_timer.setTimeout(20000);
	calibrate_angle_timer.start();
/*
	kp_left = 7.2;  kp_right = 7.2;
	ki_left = 8;    ki_right = 8;
	kd_left = 0;    kd_right = 0;
	bsp_left = 0.8; bsp_right = 0.8;

	last_desired_refined_v = 0;
//*/
	int kp = 1;
	int ki = 0;
	int kd = 0;

	//first_mpu_sample = true;
/*
	is_turning = 0;

	pid_convertion_const = (1000 * 2 * 3.1415) / 1024.0;

	right_pid.Init(kp_right, kd_right, ki_right, bsp_right);
	left_pid.Init(kp_left, kd_left, ki_left, bsp_left);
	*/
	driver->setPID(kp, ki, kd);

	driver->setMaxPPS(MAX_PPS);
}

Trekking::~Trekking() {
	log.info("memory management", "freeing memory");

	for(int i = targets.size() - 1; i >= 0; i--) {
		Position *target = targets.remove(0);
		log.info("memory management", (int)target);
		log.info("target", "removed target");
		delete target;
	}
	delete m_pSensorArray;
	log.info("memory management", "done");
}

void Trekking::addTarget(Position *target) {
	targets.add(target);
	log.info("target", "added target");
}

void Trekking::start() {
	emergency();
	delay(1500);

	//The alert led turns off when MPU is ready
	MPU.init(MPU_UPDATE_RATE, MPU_MAG_MIX_GYRO_AND_MAG,
		MAG_UPDATE_RATE, MPU_LPF_RATE);

	tested_pps = SAFE_PPS;

	// control_clk.start();
	elapsed_time = 0;
	last_update_time = millis();
	//last_update_time_2 = millis();
}

void Trekking::update() {
	loopCheck(); //read all possible inputs

	float delta_t = millis() - last_update_time;
  last_update_time = millis();
  float dT = delta_t/1000.00;

	elapsed_time += delta_t;

	updatePosition(dT); //updatePosition
	(this->*operation_mode)(dT); //Call the current operation

	if(is_testing_openloop){
		log << elapsed_time*1000;
		log << DELIMITER << dT*1000;
		log << DELIMITER << driver->getLeftPPS();
		log << DELIMITER << driver->getRightPPS();
		log << DELIMITER << tested_pps;
		printAccelInfo();
		printGyroInfo();
		// printVelocities();// [V e W]
		log << log_endl;
	}

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

	float planned_distance = MAX_LINEAR_VELOCITY * tempo;

	float p_theta = atan2(destination->getY() - init_position.getY(), destination->getX() - init_position.getX());
	float p_x = init_position.getX() + planned_distance*cos(p_theta);
	float p_y = init_position.getY() + planned_distance*sin(p_theta);

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
		p_y = max(destination->getY(),p_y);
	}
	else{
		p_y = min(destination->getY(),p_y);
	}

	planned_position.set(p_x, p_y, p_theta);
	return planned_position;
}

// v velocity in m/s and w the angle in rad/s
void Trekking::controlMotors(float v, float w, bool enable_pid, float dT){
	//Calculating rps (ROTATIONS Per Sec)
	float right_rotation = (v - w*DISTANCE_FROM_RX)/TWO_PI_R;
	float left_rotation = (v + w*DISTANCE_FROM_RX)/TWO_PI_R;

	float r_limited, l_limited = 0;



	//Limiting speeds
	if (left_rotation < 0 && right_rotation > 0){
		l_limited = max(left_rotation, -SAFE_RPS); // L negativo
		r_limited = min(right_rotation, SAFE_RPS); // R positivo
	}
	else if (left_rotation > 0 && right_rotation < 0){
		r_limited = max(right_rotation, -SAFE_RPS); // R negativo
		l_limited = min(left_rotation, SAFE_RPS);   // L positivo
	}
	else if (left_rotation > SAFE_RPS || right_rotation > SAFE_RPS){
		if (left_rotation > right_rotation){
			r_limited = SAFE_RPS*(1- right_rotation/left_rotation);
			l_limited = SAFE_RPS;
		}
		else if (left_rotation < right_rotation){
			l_limited = SAFE_RPS*(1- left_rotation/right_rotation);
			r_limited = SAFE_RPS;
		}
		else if (left_rotation == right_rotation){
			r_limited = SAFE_RPS;
			l_limited = SAFE_RPS;
		}
	}
	else if (left_rotation < -SAFE_RPS || right_rotation < -SAFE_RPS){
  	if (left_rotation < right_rotation){
    	r_limited = -SAFE_RPS*(1 - right_rotation/left_rotation);
      l_limited = -SAFE_RPS;
    }
    else if (left_rotation > right_rotation){
        l_limited = -SAFE_RPS*(1 - left_rotation/right_rotation);
        r_limited = -SAFE_RPS;
    }
    else if (left_rotation == right_rotation){
        r_limited = -SAFE_RPS;
        l_limited = -SAFE_RPS;
    }
	}
	else{
		r_limited = right_rotation;
		l_limited = left_rotation;
	}

	if(is_testing_search || is_testing_refinedSearch){
		log << "<DESEJADO>";
		log << DELIMITER << v;
		log << DELIMITER << w;
		log << DELIMITER << left_rotation;
		log << DELIMITER << right_rotation;
		log << DELIMITER << l_limited;
		log << DELIMITER << r_limited;
	}
	float r_qpps = r_limited*GEAR_RATE*PULSES_PER_ROTATION;
	float l_qpps = l_limited*GEAR_RATE*PULSES_PER_ROTATION;

	//PID
	if(enable_pid){
		driver->setRightPPS(r_qpps, dT);
		driver->setLeftPPS(l_qpps, dT);
	}
	else{
			driver->setRightPPS(r_qpps);
			driver->setLeftPPS(l_qpps);
	}
}

void Trekking::controlMotors2(float v, float w, bool enable_pid, float dT){
	//Calculating rps (ROTATIONS Per Sec)
	float right_rotation = (v - w*DISTANCE_FROM_RX)/TWO_PI_R;
	float left_rotation = (v + w*DISTANCE_FROM_RX)/TWO_PI_R;

	float r_limited, l_limited = 0;



	//Limiting speeds
	if (left_rotation < 0 && right_rotation > 0){
		l_limited = max(left_rotation, -SAFE_RPS); // L negativo
		r_limited = min(right_rotation, SAFE_RPS); // R positivo
	}
	else if (left_rotation > 0 && right_rotation < 0){
		r_limited = max(right_rotation, -SAFE_RPS); // R negativo
		l_limited = min(left_rotation, SAFE_RPS);   // L positivo
	}
	else if (left_rotation > SAFE_RPS || right_rotation > SAFE_RPS){
		if (left_rotation > right_rotation){
			r_limited = SAFE_RPS*(right_rotation/left_rotation);
			l_limited = SAFE_RPS;
		}
		else if (left_rotation < right_rotation){
			l_limited = SAFE_RPS*(left_rotation/right_rotation);
			r_limited = SAFE_RPS;
		}
		else if (left_rotation == right_rotation){
			r_limited = SAFE_RPS;
			l_limited = SAFE_RPS;
		}
	}
	else if (left_rotation < -SAFE_RPS || right_rotation < -SAFE_RPS){
  	if (left_rotation < right_rotation){
    	r_limited = -SAFE_RPS*(right_rotation/left_rotation);
      l_limited = -SAFE_RPS;
    }
    else if (left_rotation > right_rotation){
        l_limited = -SAFE_RPS*(left_rotation/right_rotation);
        r_limited = -SAFE_RPS;
    }
    else if (left_rotation == right_rotation){
        r_limited = -SAFE_RPS;
        l_limited = -SAFE_RPS;
    }
	}
	if(is_testing_search || is_testing_refinedSearch){
		log << "<DESEJADO>";
		log << DELIMITER << v;
		log << DELIMITER << w;
		log << DELIMITER << left_rotation;
		log << DELIMITER << right_rotation;
		log << DELIMITER << l_limited;
		log << DELIMITER << r_limited;
	}
	float r_qpps = r_limited*GEAR_RATE*PULSES_PER_ROTATION;
	float l_qpps = l_limited*GEAR_RATE*PULSES_PER_ROTATION;

	//PID
	if(enable_pid){
		driver->setRightPPS(r_qpps, dT);
		driver->setLeftPPS(l_qpps, dT);
	}
	else{
			driver->setRightPPS(r_qpps);
			driver->setLeftPPS(l_qpps);
	}
}


void Trekking::trackTrajectory(Position* q_desired, float tracking_time) {
	//constants to handle errors
	const int k1 = 2;
	const int k2 = 1;
	const int k3 = 2;

	Position gap = current_position.calculateGap(plannedPosition(true, tracking_time));

	// log << DELIMITER << gap.getX() << DELIMITER << gap.getY() << DELIMITER << gap.getTheta() << log_endl;

	float sin_theta = sin(current_position.getTheta());
	float cos_theta = cos(current_position.getTheta());

	float e1 =  cos_theta*gap.getX() + sin_theta*gap.getY();
	float e2 = -sin_theta*gap.getX() + cos_theta*gap.getY();
	float e3 = gap.getTheta();

	//Calculating linear velocity and angular velocity
	float v = desired_linear_velocity*cos(e3) + k1*e1;
	float w = desired_angular_velocity + k2*e2 + k3*e3;

	controlMotors2(v, -w, false, 0); // motores trocados
	distance_to_target = current_position.distanceFrom(targets.get(current_target_index));

}

float Trekking::regulateControl(Position* q_desired, float dT) {
	//constants to handle errors
	const int k1 = 3;
	const int k2 = 8;
	const int k3 = -1.5;

	// getting gap between configurations
	Position gap = current_position.calculateGap(*q_desired);

	//Angular transformation
	float rho = sqrt( pow(gap.getX(),2) + pow(gap.getY(),2) );
	float gamma = - current_position.getTheta() + atan2(gap.getY(), gap.getX());
	float delta = - current_position.getTheta() - gamma;

	//Calculating v e w
	float v = (k1 * rho);
	float w = k2*gamma + k3*delta;

	controlMotors2(v, -w, false, 0);//motores trocados
	distance_to_target = rho;
	return v;
}

void Trekking::cartesianControl(Position* q_desired, float dT) {
	//constants to handle errors
	const int k1 = 1;
	const int k2 = 2;
	// getting gap between configurations
	Position gap = current_position.calculateGap(*q_desired);

	//Calculating v e w
	float v = k1*(gap.getX()*cos(current_position.getTheta()) + gap.getY()*sin(current_position.getTheta()));
	float w = k2*(atan2(gap.getY(), gap.getX()) - current_position.getTheta());
	controlMotors2(v, -w, false, dT);
}


/*----|Private: Position update related functions|---------------------------*/
void Trekking::updatePosition(float dT){
	//Get the current orientation and speeds
	// readMPU();
	updateSpeeds();
	// float angular_speed = getAngularSpeed();
	//float theta = angular_speed*dT;//euler_radians[2] - initial_euler_radians; //in rad
	float theta = euler_radians[2]; //in rad
	float linear_speed = getLinearSpeed();

	//Calculating new position
	float x = current_position.getX() + linear_speed*cos(theta)*dT;
	float y = current_position.getY() + linear_speed*sin(theta)*dT;

	//Update values
	current_position.set(x, y, theta);

	// last_update_time = control_clk.getElapsedTime();
}

void Trekking::resetPosition(Position new_position){
	l_rotations_per_sec = 0;
	r_rotations_per_sec = 0;

	current_position = new_position;
	elapsed_time = 0;
	// control_clk.reset();
}

void Trekking::readMPU(){
	if(MPU.read()) {
		// accel[1] =  MPU.m_calAccel[0]/G_FACTOR;// - accel_offset[0];
		// accel[0] = -MPU.m_calAccel[1]/G_FACTOR;// - accel_offset[1];
		// accel[2] =  MPU.m_calAccel[2]/G_FACTOR;// - accel_offset[2];
		// Wb[1] = MPU.m_rawGyro[0];
		// Wb[0] = MPU.m_rawGyro[1];
		// Wb[2] = MPU.m_rawGyro[2];
		//
		euler_radians[1] = MPU.m_dmpEulerPose[0];
		euler_radians[0] = MPU.m_dmpEulerPose[1];
		euler_radians[2] = MPU.m_dmpEulerPose[2] - initial_euler_radians;
	}
}

bool Trekking::readColors(){
	bool white[3];
	white[0] = left_color.isWhite();
	white[1] = center_color.isWhite();
	white[2] = right_color.isWhite();
	int count = 0;
	for (int i=0;i<3;i++){
		if (white[i]==true) count++;
		if (count>1) return true;
	}
	return false;
}

void Trekking::updateSpeeds(){
	l_rotations_per_sec = ppsToRps(driver->getLeftPPS());
	r_rotations_per_sec = ppsToRps(driver->getRightPPS());
}

//Pulses per Sec --> Rotaion Per Sec
float Trekking::ppsToRps(int32_t pps){
	// MAX_RPS = max_pps/(PULSES_PER_ROTATION*GEAR_RATE) = 8.33 rps
	return pps/(PULSES_PER_ROTATION*GEAR_RATE);
}

// return robot linear speed in m/s
float Trekking::getLinearSpeed(){
	// Linear Speed = (2piR)*(r_speed + l_speed)/2,
	// where r_speed and l_speed are in [ROTATIONS PER SEC]
	return TWO_PI_R*(r_rotations_per_sec + l_rotations_per_sec)/2;
}

// return robot angular speed in rad/s
float Trekking::getAngularSpeed(){
	// Angular Speed = (2piR)*(r_speed - l_speed)/(2L),
	// where r_speed and l_speed are in [ROTATIONS PER SEC]
	return TWO_PI_R*(r_rotations_per_sec - l_rotations_per_sec)/(2*DISTANCE_FROM_RX);
}


/*----|Private: Operations modes|--------------------------------------------*/
void Trekking::standby(float dT) {
	if(operation_mode_switch == AUTO_MODE) {
		is_manual_message_sent = false;
		if(!is_auto_message_sent){
			log.debug("mode switch", "auto ");
			is_auto_message_sent = true;
		}
		if(init_button) {
			log.debug("init button", init_button);
			reset();
			if(checkSensors()) {
				operation_mode = &Trekking::search;
				startTimers();
			} else {
				log.error("sensors", "sensors not working as expected");
			}
		}
	}	else{
		is_auto_message_sent = false;
		if(!is_manual_message_sent){
			log.debug("mode switch", "manual ");
			is_manual_message_sent = true;
		}
		if(current_command != ' ') {
			Robot::useCommand(current_command);
			log.debug("using command", current_command);
		}
		}
}

void Trekking::search(float dT) {
	if(!is_tracking) {
		log.assert("operation mode", "SEARCH");
		// tracking_regulation_timer.start();
		is_tracking = true;
		q_desired = targets.get(current_target_index);
		//current_partial_index = 1;
		//correcao = false;
		last_desired_v = 1;
		tracking_time = 0;
	}

	tracking_time += dT;

	distance_to_target = current_position.distanceFrom(q_desired);

	//Colocar a condicao de proximidade
	if(distance_to_target <= 1.0){
  // if(abs(last_desired_v) < 4 && distance_to_target <= 0.5){
	// if(distance_to_target < 0.0){
	 	is_tracking = false;
		tracking_time = 0;
			log.assert("operation mode", "REFINED SEARCH");
			operation_mode = &Trekking::refinedSearch;
		// controlMotors(0, 0, false, dT);
		// operation_mode = &Trekking::lighting;
	}
	else{
		// BLOCO PARA DESVIAR DE OBSTÁCULOS (não testado!!)
		/************************************************************************
		if(current_target_index == 2){
			if(!correcao){
				Position *qi = new Position(30, 18, PI/2);
				regulateControl(qi, dT);
				float distance_to_partial_target = current_position.distanceFrom(qi);
				if (distance_to_partial_target < 5){
						correcao = true;
				}
			}
			else{
					regulateControl(q_desired, dT);
					// cartesianControl(q_desired, dT);
				}
		}
		************************************************************************/
		if(is_testing_cartesian){ // para testar, digite 1
			cartesianControl(q_desired, dT);
		}
		else if (is_testing_trackTrajectory){ // para testar, digite 3
			trackTrajectory(q_desired, tracking_time);
		}
		else{
			last_desired_v = regulateControl(q_desired, dT);
		}

	}
	// Log for debug
	if(is_testing_search){
		log << DELIMITER << "<REAL>";
		// printTime();
		log << DELIMITER << dT;
		printVelocities();// [V e W]
		// elapsed_time += dT;
		// printAccelInfo();
		// printGyroInfo();
		printRotations(); // [L_RPS e R_RPS]
		printPosition();  // [x y theta]
		finishLogLine();
	}
}

void Trekking::refinedSearch(float deltaTime)
{
	bool found = false;
	float error = m_pSensorArray->Update();
	if(m_pSensorArray->DetectedCount == 0)
	{
		// Nada foi detectado
		for (size_t i = 0; i < m_pSensorArray->Count(); i++)
		{
			// Se qualquer um dos sensores teve como ultima leitura válida
			// Algo muito proximo dos cones (ou do mínimo que podemos ler com confiança),
			// Verifica se estamos na base branca, se sim, ativa a sirene
			Sensor *pSensor = m_pSensorArray->GetSensorData(i).pSensor;
			if(pSensor->LastValidValue < 30) // TODO - Definir constante para dizer que o Trekking chegou perto o suficiente
			{
				// Verifica se não está em cima da base branca
				if(right_color.isWhite() || center_color.isWhite() || left_color.isWhite())
				{
					controlMotors(0, 0, false, deltaTime);
					operation_mode = &Trekking::lighting;

					current_position.setX(targets.get(current_target_index)->getX());
					current_position.setY(targets.get(current_target_index)->getY());
					// current_position.set()
					/*
					last_desired_refined_v = 0;
					integral_error_v = 0;
					integral_error_w = 0;
					//*/
					m_PidController.Reset();
					found = true;
				}
				break;
			}
		}

		if(!found)
		{
			// Nesse caso não temos nenhum cone a vista
			// E como a detecção da base branca foi feita lá em cima, não é o caso de estar a < 30cm
			// Dá uma voltinha pra encontrar o cone?
			controlMotors(0, -1, 0, deltaTime);
		}
	}
	else
	{
	// Temos algum cone sendo visto, roda o PID e segue na direção dele
		float pidOutput = m_PidController.Run(error, deltaTime);
		controlMotors(MAX_LINEAR_VELOCITY, pidOutput, 0, deltaTime); // velocidade angular = saída do PID; linear = cte
		// float power = m_PidController.Run(error, deltaTime);
		// if(error == 0)
		// {
		// 	controlMotors(power, 0, 0, deltaTime);
		// }
		// else
		// {
		// 	controlMotors(0, power, 0, deltaTime);
		// }
	}

	// Print dos valores para teste
	if(is_testing_refinedSearch){
		log << DELIMITER << "<REAL>";
		// printTime();
		log << DELIMITER << deltaTime;
		printVelocities();// [V e W]
		printRotations(); // [L_RPS e R_RPS]
		printSonarInfo();  // [L R DontCare]
		printPosition();
		finishLogLine();
	}

	if(is_testing_search){
		log << DELIMITER << "<REAL>";
		// printTime();
		log << DELIMITER << deltaTime;
		printVelocities();// [V e W]
		printRotations(); // [L_RPS e R_RPS]
		printPosition();
		finishLogLine();
	}

}

void Trekking::lighting(float dT) {
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

	// Robot::stop();
	resetPosition(init_position);
	calibrateAngle();
	m_PidController.Reset();
	/*
	right_pid.reset();
	left_pid.reset();
	*/
	// stopTimers();
	// resetTimers();
	turnOffSirene();

	//Trekking variables
	desired_linear_velocity = TWO_PI_R*SAFE_RPS;
	desired_angular_velocity = 0;
	/*
	min_distance_to_enable_lights = 0;
	min_distance_to_refine_search = 0;
	//*/
	current_command = ' ';
	//is_aligned = false;
	is_tracking = false;
	is_testing_openloop = false;
	is_testing_refinedSearch = false;
	is_testing_search = false;
	//first_mpu_sample = true;
	//first_sonars_sample = true;
	current_target_index = 0;
	distance_to_target = current_position.distanceFrom(targets.get(current_target_index));
	/*
	left_vel_ref = 0;
	right_vel_ref = 0;
	integral_error_v = 0;
	integral_error_w = 0;
	//*/
	elapsed_time = 0;
	last_update_time = millis();

	log.assert("reset", "done.");
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
	digitalWrite(SIRENE_PIN,HIGH);
	sirene_is_on = true;
}

void Trekking::turnOffSirene() {
	digitalWrite(SIRENE_PIN,LOW);
	sirene_is_on = false;
}



/*----|Private: Timer functions|---------------------------------------------*/
void Trekking::startTimers() {
	// sirene_timer.start();
}

void Trekking::stopTimers() {
	sirene_timer.stop();
	// tracking_regulation_timer.stop();
}

void Trekking::resetTimers() {
	sirene_timer.reset();
	// tracking_regulation_timer.reset();
}

void Trekking::updateTimers() {
	sirene_timer.update();
	// tracking_regulation_timer.update();
	mpu_timer.update();
	calibrate_angle_timer.update();
}



/*----|Private: Auxiliar functions|------------------------------------------*/
void Trekking::loopCheck(){
	updateTimers(); //Update all the timers
	current_command = ' ';

	//Read the serial
	if(command_stream->available()) {
		current_command = command_stream->read();
		log.debug("received command", current_command);

	} else {//Read the radio
	}

	if(current_command == 'D' || emergency_button) {
		emergency(); //stop the robot
	}

	debug(); // handle other commands from serial
	readInputs(); //handle commands from buttons
}

bool Trekking::checkSensors() {return true;}

void Trekking::calibrateAngle() {
	initial_euler_radians = MPU.m_dmpEulerPose[2];
	current_position.setTheta(0);
	digitalWrite(ALERT_LED, LOW);
	log.info("mpu","Angle calibrated!");

	// Serial.print("Calibrated angle = ");
	// Serial.println(initial_euler_radians);
}

void Trekking::debug() {
	if(current_command == 's') {
		log.debug("debug command", "set to standby");
		operation_mode = &Trekking::standby;
		stopTimers();
	}
	else if(current_command == 'e') {
		log.debug("debug command", "set to search");
		operation_mode = &Trekking::search;
		// operation_mode_switch = AUTO_MODE;
		init_button = true;
		is_tracking = false;
	}
	else if(current_command == 'f') {
		log.debug("debug command", "set to refined search");
		// operation_mode_switch = AUTO_MODE;
		operation_mode = &Trekking::refinedSearch;
	}
	else if(current_command == 'l') {
		log.debug("debug command", "set to lighting");
		operation_mode = &Trekking::lighting;

	}
	else if(current_command == 'r') {
		log.debug("debug command", "reset");
		emergency();
		reset();
	}
	else if(current_command == 'W') {
		log.debug("debug command", "turn on sirene");
		turnOnSirene();
	}
	else if(current_command == 'w') {
		log.debug("debug command", "turn off sirene");
		turnOffSirene();
	}

	else if(current_command == 'n') {
		log.debug("debug command", "print encoders");
		log <<	DELIMITER << driver->getLeftEncoder();
		log <<	DELIMITER << driver->getRightEncoder();
		log << log_endl;
	}

	else if(current_command == 'p') {
		log.debug("debug command", "print position");
		log << DEBUG;
		log << DELIMITER << "x";
		log << DELIMITER << "y";
		log << DELIMITER << "theta";
		log << DELIMITER << "Vlin";
		log << DELIMITER << "Vang" << log_endl;
		printPosition();
		printVelocities();
		finishLogLine();
	}
	else if(current_command == 'm') {
		current_command = ' ';
		log.debug("debug command", "testing mpu");
		log << DEBUG;
		log << DELIMITER << "alpha";
		log << DELIMITER << "beta";
		log << DELIMITER << "theta" << log_endl;
		while (current_command != 'm'){
			if(command_stream->available()) {
				current_command = command_stream->read();
			}
			delay(20);
			if(MPU.read()) {
				euler_radians[1] = MPU.m_dmpEulerPose[0];
				euler_radians[0] = MPU.m_dmpEulerPose[1];
				euler_radians[2] = MPU.m_dmpEulerPose[2] - initial_euler_radians;
				printMPUInfo();
				finishLogLine();
			}
		}
	}
	else if(current_command == 'o') {
		current_command = ' ';
		log.debug("debug command", "testing sonars");
		log << DEBUG;
		log << DELIMITER << "left";
		log << DELIMITER << "right";
		log << DELIMITER << "h" << log_endl;
		while (current_command != 'o'){
			if(command_stream->available()) {
				current_command = command_stream->read();
			}
				m_pSensorArray->Update();
				//readSonars();
				printSonarInfo();

				/* TODO - Rever como debugar esses valores. Talvez precise de modificações em SensorArray
				float sonarDistance = 6.5/100; // distance between the two ultrassound sensors

				float l_sonar = sonars[0]/100;
				float r_sonar = sonars[1]/100;

				float cos0 = (pow(sonarDistance,2) + pow(r_sonar,2) - pow(l_sonar,2))/(2*sonarDistance*r_sonar);
				float sin20 = 1 - pow(cos0,2);
				float h = 0;

				if(sin20 < 0){
					h = (l_sonar + r_sonar)/2;
				}
				else{
					float sin0 = sqrt(sin20);
					h = r_sonar*sin0;
				}


				log << DELIMITER << h;
				log << DELIMITER << DELIMITER << cos0;
				//*/
				finishLogLine();
		}
	}
	else if(current_command == 'x') {
		current_command = ' ';
		log.debug("debug command", "testing encoders");
		log << DEBUG;
		log << DELIMITER << "l_pps";
		log << DELIMITER << "r_pps";
		log << DELIMITER << "l_rps";
		log << DELIMITER << "r_rps";
		while (current_command != 'x'){
			if(command_stream->available()) {
				current_command = command_stream->read();
			}
				log << DELIMITER << driver->getLeftPPS();
				log << DELIMITER << driver->getRightPPS();
				log << DELIMITER;
				log << DELIMITER << ppsToRps(driver->getLeftPPS());
				log << DELIMITER << ppsToRps(driver->getRightPPS());
				finishLogLine();
		}
	}
	else if(current_command == 'a') {
		log.debug("debug command", "printing accels");
		log << DEBUG;
		log << DELIMITER << "x";
		log << DELIMITER << "y";
		log << DELIMITER << "z" << log_endl;
		printAccelInfo();
		finishLogLine();
	}
	else if(current_command == 'b') {
		current_command = ' ';
		log.debug("debug command", "testing buttons");
		log << DEBUG;
		log << DELIMITER << "init";
		log << DELIMITER << "emergency";
		log << DELIMITER << "mode" << log_endl;

		while (current_command != 'b'){
			if(command_stream->available()) {
				current_command = command_stream->read();
			}
			log << DELIMITER << digitalRead(INIT_BUTTON_PIN);
			log << DELIMITER << digitalRead(EMERGENCY_BUTTON_PIN);
			log << DELIMITER << digitalRead(OPERATION_MODE_SWITCH_PIN);
			log << log_endl;
		}
	}

	else if(current_command == 'c') {
		current_command = ' ';
		log.debug("debug command", "testing colors");
		log << DEBUG;
		log << DELIMITER << "L";
		log << DELIMITER << "C";
		log << DELIMITER << "R" << log_endl;

		while (current_command != 'c'){
			if(command_stream->available()) {
				current_command = command_stream->read();
			}
			printColorsInfo();
			finishLogLine();
		}
	}

	else if(current_command == 'y') {
		if(!is_testing_openloop){
			reset();
			log.debug("debug command", "testing open motors in open loop: angular vel");
		}
		is_testing_openloop = !is_testing_openloop;

		driver->setLeftPPS(-is_testing_openloop*tested_pps);
		driver->setRightPPS(is_testing_openloop*tested_pps);

		elapsed_time = 0;
		log << '\\' <<  log_endl;
	}

	else if(current_command == '1') {// TESTANDO CARTESIANO
		if(!is_testing_search){
			reset();
			log.debug("debug command", "testing search [CARTESIAN]");
			is_testing_search = true;
			is_testing_cartesian = true;
			operation_mode = &Trekking::search;
			log << '\\' << log_endl;
		} else{
			is_testing_cartesian = false;
			reset();
		}
		elapsed_time = 0;
	}

	else if(current_command == '2') {// TESTANDO POLAR
		if(!is_testing_search){
			reset();
			log.debug("debug command", "testing search");
			is_testing_search = true;
			operation_mode = &Trekking::search;
			log << '\\' << log_endl;
		} else{
			reset();
		}
		elapsed_time = 0;
	}

	else if(current_command == '3') { // TESTANDO TRAJECTORY
		if(!is_testing_search){
			reset();
			log.debug("debug command", "testing search [TRACK TRAJECTORY]");
			is_testing_search = true;
			is_testing_trackTrajectory = true;
			operation_mode = &Trekking::search;
			log << '\\' << log_endl;
		} else{
			is_testing_trackTrajectory = false;
			reset();
		}
		elapsed_time = 0;
	}
	else if(current_command == 'T') {
		if(!is_testing_refinedSearch){
			reset();
			log.debug("debug command", "testing refined search");
			is_testing_refinedSearch = true;
			operation_mode = &Trekking::refinedSearch;
			log << log_endl;
		} else {
			reset();
		}
		elapsed_time = 0;
	}
}

void Trekking::printSonarInfo()
{
	m_pSensorArray->PrintLog(log, DELIMITER);
}

void Trekking::printEncodersInfo()
{
	log << DELIMITER << driver->getLeftEncoder();
	log << DELIMITER << driver->getRightEncoder();
	log << DELIMITER;
}

void Trekking::printRotations()
{
	log << DELIMITER << ppsToRps(driver->getLeftPPS());
	log << DELIMITER << ppsToRps(driver->getRightPPS());
	log << DELIMITER;
}

void Trekking::printMPUInfo()
{
	log << DELIMITER << euler_radians[0]*180/PI;
	log << DELIMITER << euler_radians[1]*180/PI;
	log << DELIMITER << euler_radians[2]*180/PI;
	log << DELIMITER;

}

void Trekking::printAccelInfo()
{
	log << DELIMITER << accel[0];
	log << DELIMITER << accel[1];
	log << DELIMITER << accel[2];
	log << DELIMITER;
}

void Trekking::printGyroInfo()
{
	log << DELIMITER << Wb[0];
	log << DELIMITER << Wb[1];
	log << DELIMITER << Wb[2];
	log << DELIMITER;
}

void Trekking::printPosition(){
	log << DELIMITER << current_position.getX();
	log << DELIMITER << current_position.getY();
	log << DELIMITER << current_position.getTheta()*180/PI;
	log << DELIMITER;
}

void Trekking::printVelocities()
{
	log << DELIMITER << getLinearSpeed();
	log << DELIMITER << getAngularSpeed();
	log << DELIMITER;

}

void Trekking::finishLogLine()
{
	log << log_endl;
}

void Trekking::printTime()
{
	log << DELIMITER << elapsed_time << DELIMITER;
	// log << DELIMITER << control_clk.getElapsedTime() << DELIMITER;
}

void Trekking::printColorsInfo(){
	log << DELIMITER << left_color.getWhite(); //usamdo
	 //log << DELIMITER << left_color.getRed();
	//log << DELIMITER << left_color.getGreen();
	//log << DELIMITER << left_color.getBlue();
	log << DELIMITER << center_color.getWhite(); //usando

	log << DELIMITER << center_color.isWhite(); //usando
	log << DELIMITER << center_color.isWhite(); //usando
	// log << DELIMITER << right_color.getWhite();
	log << DELIMITER;
}

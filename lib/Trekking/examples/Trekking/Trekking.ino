#include "trekking.h"
#include "trekkingmath.h"
#include "sonarlist.h"
#include "encoderlist.h"
#include "timer.h"
#include "position.h"
#include "trekkingpins.h"
#include "PIDControler.h"
// #include "XLMaxSonarEZ.h"
#include "log.h"

#include "robot.h"
#include "autocontrolboard.h"
#include "sensor.h"
#include "sensorboard.h"
#include "ultrasonic.h"
#include "simpleencoder.h"
#include "lightsensor.h"

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU9150Lib.h"
#include "CalLib.h"
#include <dmpKey.h>
#include <dmpmap.h>
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
#include <EEPROM.h>

float max_linear_vel = 2.0;
float max_angular_vel = 2.0;

Trekking trekking(max_linear_vel, max_angular_vel);
Position *cone_1 = new Position(40,0,0);
Position *cone_2 = new Position(0,0,0);
Position *cone_3 = new Position(0,0,0);

void setup() {
	pinMode(ALERT_LED, OUTPUT);
	// pinMode(SIRENE_PIN, OUTPUT);
	digitalWrite(ALERT_LED, HIGH); //states that there is a problem.
	//If the arduino could start the
	//wire i2c communication, the trekking will put it to low
	Serial.begin(9600);
	Serial1.begin(9600);
	Serial2.begin(57600);
	
	Wire.begin();
	trekking.addTarget(cone_1);
	trekking.addTarget(cone_2);
	trekking.addTarget(cone_3);
	trekking.start();
	
}

void loop() {
	trekking.update();
}

// WHITE_VALUE
// GEAR_RATE
// RPM_MAX
// cone1 = (10, -5, 0)

#include "trekking.h"
#include "trekkingmath.h"
#include "sonarlist.h"
#include "encoderlist.h"
#include "timer.h"
#include "position.h"
#include "trekkingpins.h"
#include "PIDControler.h"
#include "XLMaxSonarEZ.h"
#include "log.h"
#include "robot.h"

#include "sensor.h"
#include "ultrasonic.h"
#include "simpleencoder.h"
#include "lightsensor.h"
#include "RoboClaw.h"
#include "BMSerial.h"


#include <Wire.h>
#include "I2Cdev.h"
#include "MPU9150Lib.h"
#include "CalLib.h"
#include <dmpKey.h>
#include <dmpmap.h>
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
#include <EEPROM.h>


// float safety_factor = 2*0.2547; // v_linear = 1 m/s
float safety_factor = 0.5; // v_linear = 1 m/s
// float safety_factor = 2*0.2547; // v_linear = 1 m/s
// float safety_factor = 4*0.055;    // v_ang = 2pi rad/s


DuoDriver* driver = new DuoDriver(TX_MOTOR_PIN,
                                  RX_MOTOR_PIN,
                                  ROBOCLAW_TIMEOUT,
                                  ROBOCLAW_ADDRESS);

Trekking trekking(safety_factor, driver);
// Position *cone_1 = new Position(13, 3, 0);
// Position *cone_1 = new Position(33, -17, 0);
Position *cone_1 = new Position(10, 0, 0);
// Position *cone_1 = new Position(40, 20, 0);
// Position *cone_2 = new Position(30, 2, 0);
// Position *cone_3 = new Position(6, 18, 0);


void setup() {
  pinMode(ALERT_LED, OUTPUT);

  digitalWrite(ALERT_LED, HIGH); //states that there is a problem.
  //If the arduino could start the
  //wire i2c communication, the trekking will put it to low
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(57600);

  Wire.begin();
  trekking.addTarget(cone_1);
  // trekking.addTarget(cone_2);
  // trekking.addTarget(cone_3);
  trekking.start();

  driver->roboclaw.begin(38400);
}

void loop() {
  // driver->
  // driver->setAllLeftPWM(255);
  // driver->setARightPWM(255);
  trekking.update();
}

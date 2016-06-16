
#include <EEPROM.h>

#include <DuoDriver.h>
#include <position.h>
#include <Trekking.h>
#include <trekkingpins.h>
#include <BMSerial.h>
#include <RoboClaw.h>

float max_linear_vel = 2.0;
float max_angular_vel = 2.0;

DuoDriver* driver = new DuoDriver(TX_MOTOR_PIN,
                                  RX_MOTOR_PIN,
                                  ROBOCLAW_TIMEOUT,
                                  ROBOCLAW_ADDRESS);

Trekking trekking(max_linear_vel, max_angular_vel, driver);
Position *cone_1 = new Position(10, 0, 0);
Position *cone_2 = new Position(0, 0, 0);
Position *cone_3 = new Position(0, 0, 0);

void setup() {
  pinMode(ALERT_LED, OUTPUT);
  digitalWrite(ALERT_LED, HIGH); //states that there is a problem.
  //If the arduino could start the
  //wire i2c communication, the trekking will put it to low

  trekking.addTarget(cone_1);
  trekking.addTarget(cone_2);
  trekking.addTarget(cone_3);

  trekking.start(); //starting serial comunications
}

void loop() {
  trekking.update();
  // delay(1000);
}

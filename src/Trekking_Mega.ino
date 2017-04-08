// WHITE_VALUE
// GEAR_RATE
// RPM_MAX
// cone1 = (10, -5, 0)

#include <EEPROM.h>
#include "Trekking.h"
#include "Ultrasonic.h"


// float safety_factor = 2*0.2547; // v_linear = 1 m/s
float safety_factor = 0.80; // v_linear = 1 m/s
// float safety_factor = 2*0.2547; // v_linear = 1 m/s
// float safety_factor = 4*0.055;    // v_ang = 2pi rad/s


DuoDriver* driver = new DuoDriver(TX_MOTOR_PIN,
                                  RX_MOTOR_PIN,
                                  ROBOCLAW_TIMEOUT,
                                  ROBOCLAW_ADDRESS);

SensorArray sensorArray;
PID pidController(0, 0.4f, 0.02f, 0.3f);
Trekking trekking(safety_factor, driver, pidController, &sensorArray);
// Position *cone_1 = new Position(13, 3, 0);
// Position *cone_1 = new Position(33, -17, 0);
// Position *cone_1 = new Position(10, -5, 0);

Position *cone_1 = new Position(18, -4.5, 0);
Position *cone_2 = new Position(13, -9, 0);
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

  sensorArray.AddSensor(&Ultrasonic(LEFT_SONAR_TX_PIN, LEFT_SONAR_RX_PIN), -1.5f, 200.0);
  sensorArray.AddSensor(&Ultrasonic(CENTER_SONAR_TX_PIN, CENTER_SONAR_RX_PIN), 0.0f, 150.0);
  sensorArray.AddSensor(&Ultrasonic(RIGHT_SONAR_TX_PIN, RIGHT_SONAR_RX_PIN), 1.5f, 150.0);

  trekking.addTarget(cone_1);
  trekking.addTarget(cone_2);
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

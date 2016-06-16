
// #include <EEPROM.h>
//
// #include <DuoDriver.h>
// #include <position.h>
// #include <trekking.h>
// #include <trekkingpins.h>
// #include <BMSerial.h>
// #include <RoboClaw.h>

#include <SoftwareSerial.h>


#define BT_SERIAL_TX 1
#define BT_SERIAL_RX 0


float max_linear_vel = 2.0;
float max_angular_vel = 2.0;
char current_command;
float t = 0;


// DuoDriver* driver = new DuoDriver(TX_MOTOR_PIN,
//                                   RX_MOTOR_PIN,
//                                   ROBOCLAW_TIMEOUT,
//                                   ROBOCLAW_ADDRESS);
//
// Trekking trekking(max_linear_vel, max_angular_vel, driver);
SoftwareSerial bluetoothSerial(BT_SERIAL_RX, BT_SERIAL_TX);


void setup() {
// trekking.start(); //starting serial comunications
Serial.begin(9600);
// bluetoothSerial.begin(9600);
// Serial.begin(9600);
}

float last_update_time = 0;
int desired_pps = 6000;

void loop() {
  float delta_t = millis() - last_update_time;
  last_update_time = millis();
  float dT = delta_t/1000.00;

  // while (bluetoothSerial.available()) {
  //   current_command = bluetoothSerial.read();
  // }
  // if(current_command == 's'){
    t += dT;

    Serial.print('x');

    // bluetoothSerial.print(dT);
    // bluetoothSerial.print("; T:");
    // bluetoothSerial.print(t);
    // bluetoothSerial.print(";" );
    // // Serial.print(driver->getLeftPPS());
    // bluetoothSerial.print(";");
    // // Serial.print(driver->getRightPPS());
    // bluetoothSerial.print(";");
    // bluetoothSerial.print(desired_pps);
    // bluetoothSerial.print("\n");


    delay(1000);
  // }
  // else{
    // t = 0;
    // driver->setLeftPPS(0);
    // driver->setRightPPS(0);
  // }
}

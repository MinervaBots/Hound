#ifndef TREKKING_PINS_H
#define TREKKING_PINS_H

#include <Arduino.h>
/*
	Buttons and switches
*/
const byte INIT_BUTTON_PIN = 3;
const byte EMERGENCY_BUTTON_PIN  = 4;
const byte OPERATION_MODE_SWITCH_PIN = 2;

const bool MANUAL_MODE = 0;
const bool AUTO_MODE = 1;

/*
	Motors
*/
// Driver with Sabertooth Style
const byte RX_MOTOR_PIN = 15;
const byte TX_MOTOR_PIN = 14;
const int ROBOCLAW_ADDRESS = 0x80;
const int ROBOCLAW_TIMEOUT = 10000;


/*
	Sonars
*/
const byte LEFT_SONAR_RX_PIN = A12;
const byte LEFT_SONAR_TX_PIN = A11;
const byte CENTER_SONAR_RX_PIN = 54;
const byte CENTER_SONAR_TX_PIN = A10;
const byte RIGHT_SONAR_RX_PIN = 54;
const byte RIGHT_SONAR_TX_PIN = A7;

/*
	Colors
*/
const byte LEFT_COLOR_OUTPUT = A13;
const byte LEFT_COLOR_S0 = 53;
const byte LEFT_COLOR_S1 = 52;
const byte LEFT_COLOR_S2 = 50;
const byte LEFT_COLOR_S3 = 48;
const byte CENTER_COLOR_OUTPUT = A14;
const byte CENTER_COLOR_S0 = 46;
const byte CENTER_COLOR_S1 = 44;
const byte CENTER_COLOR_S2 = 42;
const byte CENTER_COLOR_S3 = 40;
const byte RIGHT_COLOR_OUTPUT = A15;
const byte RIGHT_COLOR_S0 = 38;
const byte RIGHT_COLOR_S1 = 36;
const byte RIGHT_COLOR_S2 = 34;
const byte RIGHT_COLOR_S3 = 32;


/*
	Sirene
*/
const byte SIRENE_PIN = A0;

/*
	Alert led
*/

const byte ALERT_LED = 13;


#endif //TREKKING_PINS_H

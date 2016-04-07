/*
	Buttons and switches
*/
const byte INIT_BUTTON_PIN = 3;
const byte EMERGENCY_BUTTON_PIN  = 2;
const byte OPERATION_MODE_SWITCH_PIN = A3; // Não está atualizado!

const bool MANUAL_MODE = 0;
const bool AUTO_MODE = 1;

/*
	Motors
*/
// Driver with Sabertooth Style
const byte RX_MOTOR_PIN = 10;
const byte TX_MOTOR_PIN = 11;
const int ROBOCLAW_ADDRESS = 0x80;
const int ROBOCLAW_TIMEOUT = 10000;


/*
	Sonars
*/
const byte LEFT_SONAR_RX_PIN = 17;
const byte LEFT_SONAR_TX_PIN = A11;
const byte CENTER_SONAR_RX_PIN = 0;
const byte CENTER_SONAR_TX_PIN = A10;
const byte RIGHT_SONAR_RX_PIN = 0;
const byte RIGHT_SONAR_TX_PIN = A7;


/*
	Sirene
*/
// const byte SIRENE_PIN = 12;

/*
	Alert led
*/

const byte ALERT_LED = 13;

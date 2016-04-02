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
const byte R_ENABLE_PIN = 255;
const byte R_MOTOR_1_PIN = 3;
const byte R_MOTOR_2_PIN = 4;

const byte L_ENABLE_PIN = 255;
const byte L_MOTOR_1_PIN = 10;
const byte L_MOTOR_2_PIN = 11;

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

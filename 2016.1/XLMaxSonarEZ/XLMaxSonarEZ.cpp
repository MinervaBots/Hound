#include "XLMaxSonarEZ.h"

XLMaxSonarEZ::XLMaxSonarEZ(byte tx, byte rx) {
	this->rx = rx;
	this->tx = tx;
	distance = 0;

	//If the rx pin is used, set it to output
    if(rx != UNUSED){
  	 	pinMode(this->rx, OUTPUT);
    }
}

XLMaxSonarEZ::~XLMaxSonarEZ() {

}

void XLMaxSonarEZ::trigger() {
    if(rx != UNUSED){
    	digitalWrite(rx, HIGH);
    	delayMicroseconds(20);
    	digitalWrite(rx, LOW);
    }
}

float XLMaxSonarEZ::read() {
	//Read the sensor and convert to a unit of length
	distance = analogRead(tx) * SONAR_TO_CM;

	return distance;
}

float XLMaxSonarEZ::getDistance() {
	return distance;
}

byte XLMaxSonarEZ::getRX() {
	return rx;
}

byte XLMaxSonarEZ::getTX() {
	return tx;
}

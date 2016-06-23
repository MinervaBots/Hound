/*
	This is an example of XLMaxSonarEZ. More information at 
	https://github.com/brunocalou/XLMaxSonarEZ

	Created by Bruno Calou Alves, March, 2015 - brunocaloualves@gmail.com
*/

#include "XLMaxSonarEZ.h"
#include "LinkedList.h"


#define RX 9	//Trigger pin
#define TX A0 	//Read pin

//Create the object
XLMaxSonarEZ sonar(TX, RX);

void setup() {
	Serial.begin(9600);
}

void loop(){
	//Trigger the sensor
	sonar.trigger();

	//Print the read value
	Serial.println(sonar.read());

	//Delay just to give us time to read the data on the serial monitor
	delay(300);
}

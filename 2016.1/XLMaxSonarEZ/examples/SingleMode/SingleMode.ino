/*
	This is an example of XLMaxSonarEZ. More information at 
	https://github.com/brunocalou/XLMaxSonarEZ

	Created by Bruno Calou Alves, March, 2015 - brunocaloualves@gmail.com
*/

#include "sonarlist.h"
#include "LinkedList.h"

#define RX 9	//Trigger pin
#define TX A0 	//Read pin

#define RX_2 10
#define TX_2 A1

#define RX_3 11
#define TX_3 A2

//Create objects
XLMaxSonarEZ sonar1(TX, RX);
XLMaxSonarEZ sonar2(TX_2,RX_2);
XLMaxSonarEZ sonar3(TX_3,RX_3);

//Create the list on the Single mode
SonarList sonar_list(Sonar::SINGLE);

void setup() {

	Serial.begin(9600);

	//Add all the sonars to the list. Note that
	//the order that the sensors are added is
	//not important, as long as the addFirst
	//method is used.
	sonar_list.addFirst(&sonar1);
	sonar_list.addSonar(&sonar2);
	sonar_list.addSonar(&sonar3);
}

void loop(){
	//Read all the sensors
	sonar_list.read();

	//Print all the values
	Serial.print(sonar1.getDistance());
	Serial.print(" - ");
	Serial.print(sonar2.getDistance());
	Serial.print(" - ");
	Serial.println(sonar3.getDistance());

	//Delay just to give us time to read the data on the serial monitor
	delay(300);
}

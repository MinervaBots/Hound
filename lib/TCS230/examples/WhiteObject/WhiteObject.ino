/*
  White objects detector
  
  The code bellow is to detect if the surface the object is pointing is white or not,
  using the TCS230 library.
  This example code is in the public domain.

  modified 05 July 2014
  by Edilson Fernandes
*/

 
#include "TCS230.h"
#define OUTPUT_ENEABLE 7 //define output eneable pin of the sensor


TCS230 new_sensor(8,9,12,11,10,50); //inicialize a new sensor with the given pins


//the setup function runs once when you press reset or power the board
void setup() {
	pinMode(OUTPUT_ENEABLE, INPUT);
	digitalWrite(OUTPUT_ENEABLE, HIGH);
  Serial.begin(9600);
}

//the loop function runs over and over again forever
void loop() {
  if(new_sensor.isWhite()) Serial.println("The surface it is indeed white!");
  else Serial.println("Not white Bro!");
	delay(500);  
}

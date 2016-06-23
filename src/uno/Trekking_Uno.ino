/*
  Read the encoders and send it to the other arduino
*/
#include "encoderlist.h"
#include "simpleencoder.h"
#include "SoftwareSerial.h"
#include "LinkedList.h"

#define SIRENE_PIN  9
#define R_ENC_1 6
#define R_ENC_2 7
#define L_ENC_1 10
#define L_ENC_2 11


SoftwareSerial my_serial(10,11);
Stream *current_stream = &Serial;

void setup() {
  Serial.begin(57600);
  my_serial.begin(57600);
}


void loop() {
  delay(2);

 //Read the serial port
 if (current_stream->available()) {
   char incoming_message = (char) current_stream->read();
   if(incoming_message == LIGHT_ON) {
     digitalWrite(SIRENE_PIN,HIGH);
   } else if(incoming_message == LIGHT_OFF) {
     digitalWrite(SIRENE_PIN,LOW);
   }
 }
}

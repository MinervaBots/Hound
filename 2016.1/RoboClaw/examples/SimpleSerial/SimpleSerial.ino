//Roboclaw simple serial example.  Set mode to 6.  Option to 4(38400 bps)
#include "BMSerial.h"

//Arduino Due currently does not support SoftwareSerial. Only hardware uarts can be used, pins 0/1, 14/15, 16/17 or 18/19.

//Note: Most Arduinos do not support higher baudrates rates than 115200.  Also the arduino hardware uarts generate 57600 and 115200 with a
//relatively large error which can cause communications problems.


BMSerial mySerial(10,11);

void setup() {
  mySerial.begin(38400);
}

void loop() {
  mySerial.write(1);
  mySerial.write(-127);
  delay(2000);
  mySerial.write(64);
  delay(1000);
  mySerial.write(127);
  mySerial.write(-1);
  delay(2000);
  mySerial.write(-64);
  delay(1000);
  mySerial.write(1);
  mySerial.write(-1);
  delay(2000);
  mySerial.write(0);
  delay(1000);
  mySerial.write(127);
  mySerial.write(-127);
  delay(2000);
  mySerial.write(0);
  delay(1000);
}

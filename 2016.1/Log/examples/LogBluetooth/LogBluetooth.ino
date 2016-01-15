/*
  Created by Bruno Calou Alves, May, 2015.
  Read LICENSE for more information.
*/

#include "log.h"
#include "SoftwareSerial.h"

Log my_log;
SoftwareSerial mySerial(10, 11); // RX, TX

void setup()
{
  //Begin the serial communication
  mySerial.begin(9600);

  //Set the log target (it's serial by default)
  my_log.setTarget(&mySerial);
  
  //Set the priority to be printed. The VERBOSE
  //mode will print everything. Comment / uncomment
  //to see the changes.

  my_log.setPriority(INFO);
  //my_log.setPriority(ASSERT);
  //my_log.setPriority(WARN);
  //my_log.setPriority(DEBUG);
  //my_log.setPriority(WARN);
  //my_log.setPriority(VERBOSE);
}

void loop()
{
  //Print whatever you like
  my_log.info("info id", "message");
  my_log.assert("assert id", 6);
  my_log.warn("warn id", 5.9);
  my_log.debug("debug id", 1533L);
  my_log.error("error id", '6');
  my_log.verbose("verbose id", 42);

  //You can use the println method as well
  my_log.println(INFO, "cool info", "Printing");
  my_log.println(INFO, "cool info", "Cool");
  my_log.println(INFO, "cool info", "Info");
}

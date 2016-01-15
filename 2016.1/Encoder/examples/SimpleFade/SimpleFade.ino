#include "simpleencoder.h"

#define ENCODER 	0
#define LED 		6

SimpleEncoder encoder;

void add()
{
  encoder.add();
}

void setup()
{
  attachInterrupt(ENCODER,add,RISING);
}

void loop()
{
  analogWrite(LED,(int)encoder.getRotation() % 255);
}

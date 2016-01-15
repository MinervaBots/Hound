#include "simpleencoder.h"
#define ENCODER_PIN		0
#define LED 			6

volatile int last_pwm = 0;
bool up = true;
SimpleEncoder encoder(10);

void update()
{
    encoder.add();
    int pwm = ((int)encoder.getRotation() * 20) % 255;

    if(last_pwm > pwm)
    {
      up = !up;
    }
    last_pwm = pwm;

    if(up)
    {
      analogWrite(LED, pwm);
    } else {
      analogWrite(LED, 255 - pwm);
    }
}
void setup()
{
    attachInterrupt(ENCODER_PIN, update, RISING);
}

void loop()
{
}

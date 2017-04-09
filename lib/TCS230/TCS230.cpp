/*
	TCS230.h - TCS230 implements trigger and digital read methods for RGB
 *      color detection, implements a particular method that says if an object
 *      is white or not.

	The pulse in method of the Arduino library allows to the sensor to use any digital pins of any arduino board model, what is not possible
	using the frequency counter(I'd have trouble myself trying to do that).
	Created by Edilson Fernandes(Dilss), July, 2016.
	Read LICENSE for more information.
*/


#include "TCS230.h"

//This method start a new instance of the class;
TCS230::TCS230(int s0_pin, int s1_pin, int s2_pin, int s3_pin, int out_pin, int WHITE_VALUE){
    this->s0_pin = s0_pin;
    this->s1_pin = s1_pin;
    this->s2_pin = s2_pin;
    this->s3_pin = s3_pin;
    this->out_pin = out_pin;
    pinMode(this->s0_pin, OUTPUT);
    pinMode(this->s1_pin, OUTPUT);
    pinMode(this->s2_pin, OUTPUT);
    pinMode(this->s3_pin, OUTPUT);
    pinMode(this->out_pin, INPUT);
    digitalWrite(this->s0_pin, HIGH);
    digitalWrite(this->s1_pin, LOW);
    setWhiteValue(WHITE_VALUE);
}

// This 4 methods triggers the outputs end reads the input signal on OUT_PIN;
float TCS230::getRed(){
    digitalWrite(this->s2_pin, LOW);
    digitalWrite(this->s3_pin, LOW);
    float red = pulseIn(this->out_pin, digitalRead(this->out_pin) == HIGH ? LOW : HIGH);
    return red;
}

float TCS230::getBlue(){
    digitalWrite(this->s2_pin, LOW);
    digitalWrite(this->s3_pin, HIGH);
    float blue = pulseIn(this->out_pin, digitalRead(this->out_pin) == HIGH ? LOW : HIGH);
    return blue;
}

float TCS230::getGreen(){
    digitalWrite(this->s3_pin, HIGH);
    digitalWrite(this->s2_pin, HIGH);
    float green = pulseIn(this->out_pin, digitalRead(this->out_pin) == HIGH ? LOW : HIGH);
    return green;
}

float TCS230::getWhite(){
    delay(100);
    digitalWrite(s2_pin, HIGH);
    digitalWrite(s3_pin, LOW);
    float white = pulseIn(this->out_pin, digitalRead(this->out_pin) == HIGH ? LOW : HIGH);
    return white;
}

// Method used to set the parameter of white to compare with the corrent sensor reading;
void TCS230::setWhiteValue(int value){
    white_value = value;
}

double TCS230::getRawValue()
{
  return getWhite();
  //return (getRed() + getGreen() + getBlue()) / 3;
}

//This method returns if an object is white or not;
bool TCS230::isWhite(){
    int color = getWhite();
    return (color <= white_value);
}

void TCS230::printRGBW(float r, float g, float b, float w){
  Serial.println();
  Serial.print("RED: ");
  Serial.print(r);
  Serial.print("     GREEN: ");
  Serial.print(g);
  Serial.print("     BLUE: ");
  Serial.print(b);
  Serial.print("     WHITE: ");
  Serial.print(w);
}

TCS230::~TCS230() {
}

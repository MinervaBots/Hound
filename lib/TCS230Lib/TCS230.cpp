/*
	TCS230.h - TCS230 implements trigger and digital read methods for RGB
 *  color detection, implements a particular method that says if an object
 *  is the color you've set or not.

	The pulse in method of the Arduino library allows to the sensor to use any digital pins of any arduino board model, what is not possible
	using the frequency counter(I'd have trouble myself trying to do that).
	Created by Edilson Fernandes(Dilss), July, 2016.
	Read LICENSE for more information.
*/


#include "TCS230.h"

//This method start a new instance of the class;
TCS230::TCS230(int s0_pin, int s1_pin, int s2_pin, int s3_pin, int out_pin, int white):
 		white_value(white),
		k(1000000),
		scale(1)
{
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
}

// This method triggers the outputs end reads the input signal on OUT_PIN;
void TCS230::color(){
	digitalWrite(this->s2_pin, LOW);
	digitalWrite(this->s3_pin, LOW);
	red = pulseIn(this->out_pin, digitalRead(this->out_pin) == HIGH ? LOW : HIGH);
	digitalWrite(this->s3_pin, HIGH);
	blue = pulseIn(this->out_pin, digitalRead(this->out_pin) == HIGH ? LOW : HIGH);
	digitalWrite(this->s2_pin, HIGH);
	green = pulseIn(this->out_pin, digitalRead(this->out_pin) == HIGH ? LOW : HIGH);
}

//This method is to set the values of red, green and white when the sensor is reading the color you want to detect;
//it does 20 readings and keeps the maximum and the minimum value of all readings for each color;
void TCS230::calibrate(char t_color[10]){
	int i;
	max_red = 0; max_green = 0; max_blue = 0;
	min_red = 180; min_green = 200; min_blue = 140;

	for (i = 1; i <= 20; i++){
		color();
        if (max_red < red){
            max_red = red;
        }
        if (max_green < green){
            max_green = green;
        }
        if (max_blue < blue){
            max_blue = blue;
        }
        if (min_red > red){
            min_red = red;
        }
        if (min_green > green){
            min_green = green;
        }
        if (min_blue > blue){
          min_blue = blue;
        }
        delay(200);
    }
	my_color[10] = t_color[10];
	Serial.begin(9600);
    Serial.println("Calibration done!");
    Serial.print("********************* ");
    Serial.print(my_color);
    Serial.print(" *********************");
    Serial.println();
    Serial.print(min_red, DEC);
    Serial.print(" < Vermelho <  ");
    Serial.print(max_red, DEC);
    Serial.println();
    Serial.print(min_green, DEC);
    Serial.print(" < Verde < ");
    Serial.print(max_green, DEC);
    Serial.println();
    Serial.print(min_blue, DEC);
    Serial.print(" < Azul < ");
    Serial.print(max_blue, DEC);
    Serial.println();

}

//This method prints the red, green and blue values the sensor is reading at the moment;
void TCS230::printRGB(){
	color();
	Serial.begin(9600);
	Serial.print("RED : ");
    Serial.print(red, DEC);
    Serial.print("  Green : ");
    Serial.print(green, DEC);
    Serial.print("  BLUE : ");
    Serial.print(blue, DEC);
    Serial.println();
    delay(2000);
}

//This method return if the values got on current reading are the same as the values got on the calibrate method, returns true or false;
bool TCS230::isTheColor(){
    bool bool_red = false;
    bool bool_green = false;
    bool bool_blue = false;
	if ((red > min_red) & (red < max_red)){
		bool_red = true;
	 };
    if ((green > min_green) & (green < max_green)){
        bool_green = true;
    };
    if ((blue > min_blue) & (blue < max_blue)){
        bool_blue = true;
    };
	if (bool_red & bool_green & bool_blue){
        return true;
    }
    else{
        return false;
    }
}

TCS230::~TCS230() {
}

bool TCS230::isWhite(){
	digitalWrite(s2_pin, HIGH); digitalWrite(s3_pin, LOW);
  float t_white = 2*pulseIn(out_pin, HIGH)/k;
  float f_white = 1/t_white;
  int white = 0.001*f_white/scale;
	return (white >= white_value);
}

void TCS230::setWhiteValue(int value){
	white_value = value;
}

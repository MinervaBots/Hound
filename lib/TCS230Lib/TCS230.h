/*
	TCS230.h - TCS230 implements trigger and digital read methods for RGB
 *      color detection, implements a particular method that says if an object
 *      is white or not.
	Created by Edilson Fernandes, July, 2016.
	Read LICENSE for more information.
*/


#ifndef TCS230_H
#define TCS230_H


#include "Arduino.h"

#ifndef NULL
#define NULL 0
#endif

class TCS230 {
public:
    TCS230(int s0_pin, int s1_pin, int s2_pin, int s3_pin, int out_pin, int white);



    // This method reads the primary colors value(proportional);
    void color();

    //This method is to set the values of red, green and white when the sensor is reading the color you want to detect;
    //White is the default, to calibrate you need to put the sensor next to the surface you want to detect the color for, and
    //call the method giving, as argument, the name of the color you want to set.
    void calibrate(char t_color[10]);

	//This metod prints 3 numbers each one of them associated to one of primary colors;
	  void printRGB();

	//This method tells if the color read is the same that you've calibrated the sensor ;
    bool isTheColor();

    bool isWhite();
    void setWhiteValue(int value);

    virtual ~TCS230();
private:
    //Variables that stores the values returned by each digital pin
    float red;
    float green;
    float blue;
    //Variables used to store the minimum and the maximum values, between 20 readings, for each color;
  	float max_red;
    float max_green;
    float max_blue;
    float min_red;
    float min_green;
    float min_blue;
    char my_color[10];
	//Variables that keep the pins values you've choose;
    int s0_pin;
	  int s1_pin;
	  int s2_pin;
    int s3_pin;
    int out_pin;

    int white_value;
    const int k;
    const int scale;
};

#endif /* TCS230_H */

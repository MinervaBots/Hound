#include "position.h"
#include "Arduino.h"

//construtores
Position::Position(){
	Position:Position(0,0,0);
	is_degrees = false;
}

Position::Position(float x, float y, float theta){
	set(x,y,theta);
}
	
//getters
float Position::getX(){
	return x;
}

float Position::getY(){
	return y;
}

float Position::getTheta(){
	return theta;
}

//setters
void Position::set(float x, float y, float theta){
	setX(x);
	setY(y);
	setTheta(theta);
}


void Position::setX(float x){
	this->x = x;
}

void Position::setY(float y){
	this->y = y;
}

void Position::setTheta(float theta){
	// if(is_degrees) {
	// 	if(theta >= 360) {
	// 		theta -= 360;
	// 	}
	// 	if(theta < 0) {
	// 		theta += 360;
	// 	}
	// } else {
	// 	if(theta >= TWO_PI) {
	// 		theta -= TWO_PI;
	// 	}
	// 	if(theta < 0) {
	// 		theta += TWO_PI;
	// 	}
	// }
	
	this->theta = theta;
}

float Position::distanceFrom(Position *position) {
	return sqrt( sq((x - position->getX())) + sq(y - position->getY()) );
}

Position Position::calculateGap(Position pos){
	float gap_x = pos.getX() - x;
	float gap_y = pos.getY() - y;
	float gap_theta = pos.getTheta() - theta;
	return Position(gap_x, gap_y, gap_theta);
}
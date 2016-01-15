#ifndef POSITION_H
#define POSITION_H

class Position{
private:
	float x;
	float y;
	float theta;

public:
	//construtores
	Position();
	Position(float x, float y, float theta);

	//getters
	float getX();
	float getY();
	float getTheta();

	//setters
	void set(float x, float y, float theta);
	void setX(float x);
	void setY(float y);
	void setTheta(float theta);

	float distanceFrom(Position *position);
	Position calculateGap(Position pos);

	bool is_degrees;
};

#endif
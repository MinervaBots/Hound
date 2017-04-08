
/*
	trekkingmath.h - Generic math functions for the
	Trekking project
	Created by Bruno Calou Alves, April, 2015.
*/

#ifndef TREKKINGMATH_H
#define TREKKINGMATH_H

/*
	Absolute value of a number
*/
template <typename T>
T module(T x) {
	if (x < 0){
		return -x;
	}
	return x;
}

/*
	Compares if two numbers are equals, inside a margin of error
*/
template <typename T, typename U, typename V>
bool nearEquals(T x, U y, V error)
{
	return module(x - y) <= module(error);
}

/*
	Clamps a value between two others
*/
template <typename T>
T clamp(const T& value, const T& low, const T& high)
{
  return value < low ? low : (value > high ? high : value);
}

#endif // TREKKINGMATH_H

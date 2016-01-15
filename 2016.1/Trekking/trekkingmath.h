
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
bool nearEquals(T x, U y, V error) {

	if (module(x - y) <= module(error)) {
		return true;
	}

	return false;

}

#endif // TREKKINGMATH_H
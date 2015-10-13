#pragma once

//This header file contains useful constants and macros.

#include <math.h>
#include <float.h>

/**
	The epsilon value is used for all kinds of numerical computations. For instance, when checking to see if two points are equal, we will check to
	see if they are equal, within epsilon.
*/
#define EPSILON 0.0000000001

#define INFINITY DBL_MAX

#define TINY_NUMBER 0.000000001
#define TINY TINY_NUMBER

#define PI 3.14159265

/**
	This macro checks to see if the value of x is zero within epsilon: -epsilon<x<epsilon
*/
#define ZERO_WITHIN_EPSILON(x) ((x)>-EPSILON && (x)<EPSILON)


/**
	This macro checks to see if the value of x is less than y, within epsilon
*/
#define LESS_THAN_WITHIN_EPSILON(x,y) ((x)<(y)+EPSILON)


/**
	This macro checks to see if the value of x is greater than y, within epsilon
*/
#define GREATER_THAN_WITHIN_EPSILON(x,y) ((x)>(y)-EPSILON)


/**
	This macro checks to see if the values of x and y are equal within epsilon
*/
#define EQUAL_TO_WITHIN_EPSILON(x,y) ZERO_WITHIN_EPSILON((x)-(y))

/**
	and some shortcuts to the macros above
*/
#define IS_ZERO(x)			ZERO_WITHIN_EPSILON(x)
#define LESS_THAN(x,y)		LESS_THAN_WITHIN_EPSILON(x,y)
#define GREATER_THAN(x,y)	GREATER_THAN_WITHIN_EPSILON(x,y)
#define EQUAL_TO(x,y)		EQUAL_TO_WITHIN_EPSILON(x,y)


/**
	Computes the value of x in radians
*/
#define RAD(x) (((x) * PI)/180.0)

/**
	And this computes the value of x in degrees
*/
#define DEG(x) ((x * 180)/PI)

/**
	macros for max and mins
*/

#define MAX(x, y) (((x)<(y))?(y):(x))
#define MIN(x, y) (((x)>(y))?(y):(x))

#define SQR(x) ((x)*(x))

#define SGN(x) (((x)<0)?(-1):(1))

inline double safeACOS(double val){
	if (val<-1)
		return PI;
	if (val>1)
		return 0;
	return acos(val);
}

inline void boundToRange(double* v, double min, double max){
	if (*v < min)
		*v = min;
	if (*v > max)
		*v = max;
}

//this method returns the index of the current value in the range min-min+range that has been discretized in dim parts.
//the value returned will be in the range 0-(dim-1), and val should be included in the closed interval [min; max]
inline int getIndex(double min, double range, int dim, double val){
	///gracefully handle wrong input
	if (val >= min + range) return dim-1;
	if (val <= min) return 0;

	//and now see in which of the dim bins val resides in
	return (int)(((val - min) / range) * dim);
}

//if we have a 1-D line segment, that starts at min, is range long and discretized in dim parts, then this method can be used to
//retrieve the mid point of the ith segment
inline double getMidSegment(double min, double range, int dim, int i){
	return (i+0.5) * range / dim + min;
}

inline double linearlyInterpolate(double v1, double v2, double t1, double t2, double t){
	if (v1 == v2)
		return v2;
	return (t-t1)/(t2-t1) * v2 + (t2-t)/(t2-t1) * v1;
}



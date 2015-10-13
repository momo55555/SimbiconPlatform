#pragma once

#include <stdio.h>

#include "MathLib.h"



class ThreeTuple
{
	
//these variables are declared public because they provide faster access
public:
	double x;
	double y;
	double z;

public:
	/**
		some useful constructors 
	*/
	ThreeTuple();
	ThreeTuple(ThreeTuple&);
	ThreeTuple(double x, double y, double z);
	ThreeTuple(double x, double y);
	ThreeTuple(double*);
	~ThreeTuple();

	/**
		setters and getters
	*/
	inline double getX() const{
		return this->x;
	}
	inline double getY() const{
		return this->y;
	}
	inline double getZ() const{
		return this->z;
	}

	inline void setX(double x){
		this->x = x;
	}
	inline void setY(double y){
		this->y = y;
	}
	inline void setZ(double z){
		this->z = z;
	}

	inline void setValues(double x, double y, double z){
		this->x = x;
		this->y = y;
		this->z = z;
	}

	inline void setValues(ThreeTuple& p){
		this->x = p.x;
		this->y = p.y;
		this->z = p.z;
	}

	/**
		this is an equality operator.
	*/
	inline bool operator == (const ThreeTuple& p) const{
		double dx = this->x-p.x;
		double dy = this->y-p.y;
		double dz = this->z-p.z;
		return (ZERO_WITHIN_EPSILON(dx) && ZERO_WITHIN_EPSILON(dy) && ZERO_WITHIN_EPSILON(dz));
	}

	/**
		this is an equality operator.
	*/
	inline bool operator != (const ThreeTuple& p) const{
		return !(*this == p);
	}
	/**
		This method is used for debugging purposes. It prints the x, y and z components.
	*/
	inline void printTuple(){
/*		tprintf("(%lf, %lf, %lf)\n", x, y, z);*/
	}


	/**
		and a copy operator.
	*/
	inline ThreeTuple& operator = (ThreeTuple& p){
		this->x = p.x;
		this->y = p.y;
		this->z = p.z;
		return *this;
	}

	/**
		creates an exact copy of the current threetuple and returns a pointer to it.
	*/
//	virtual ThreeTuple* createCopy();

};


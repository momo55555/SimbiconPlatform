#include "stdafx.h"

#include "ThreeTuple.h"

/*constructors*/
ThreeTuple::ThreeTuple(double x,double y,double z){
	this->x = x;
	this->y = y;
	this->z = z;
}

ThreeTuple::ThreeTuple(double x,double y){
	this->x = x;
	this->y = y;
	this->z = 0;
}

ThreeTuple::ThreeTuple(ThreeTuple& p){
	this->x = p.x;
	this->y = p.y;
	this->z = p.z;
}

ThreeTuple::ThreeTuple(double* values){
	this->x = values[0];
	this->y = values[1];
	this->z = values[2];
}

ThreeTuple::ThreeTuple(){
	this->x = 0.0;
	this->y = 0.0;
	this->z = 0.0;
}

/*destructor*/
ThreeTuple::~ThreeTuple(){
}

/**
	creates an exact copy of the current threetuple and returns a pointer to it.
*/
/*
ThreeTuple* ThreeTuple::createCopy(){
	return new ThreeTuple(this->x, this->y, this->z);
}
*/

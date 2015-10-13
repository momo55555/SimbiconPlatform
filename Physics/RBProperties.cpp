#include "stdafx.h"

#include "RBProperties.h"

/**
	default constructor.
*/
RBProperties::RBProperties()
{
	//initialize with some safe values.
	this->mass = 1;
	this->invMass = 1;
	//set some good values for the coefficients of friction and restitution
	this->epsilon = 0.8;
	this->mu = 1.5;

	MOI_local = Vector3d(1,1,1);
	invMOI_local = Vector3d(1,1,1);
	isLocked = false;
	isPlanar = false;
	groundSoftness = 0.00001;
	groundPenalty = 0.2;
}

/**
	default destructor.
*/
RBProperties::~RBProperties(){
	//nothing to do here...
}

/**
	sets the mass of the rigid body
*/
void RBProperties::setMass(double m){
	this->mass = m;
	this->invMass = 1/m;
	if (isLocked)
		invMass = 0;
}

/**
	set the moment of inertia of the rigid body. The three variables represent to the x, y and z principal moments.
*/
void RBProperties::setMOI(double xM, double yM, double zM)
{
	this->MOI_local.setValues(xM, yM,zM);
	this->invMOI_local.setValues(1/xM, 1/yM, 1/zM);
	if (isLocked)
		this->invMOI_local.setValues(0,0,0);
}

/**
	This method sets the rigid body's state as fixed.
*/
void RBProperties::lockBody(bool isLocked)
{
	this->isLocked = isLocked;
	if(isLocked)
    {
		this->invMOI_local.setValues(0,0,0);
		this->invMass = 0;
	}
}

/**
	This method sets the rigid body's state as not fixed.
*/
void RBProperties::releaseBody()
{
	isLocked = false;
	this->invMOI_local.setValues(1/MOI_local.x, 1/MOI_local.y, 1/MOI_local.z);
	this->invMass = 1/this->mass;
}

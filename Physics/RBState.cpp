#include ".\rbstate.h"

/**
	Default constructor - populate the data members using safe values..
*/
RBState::RBState(void){
	//guess some default values if this constructor is used...
	this->position = Point3d(0,0,0);
	this->orientation = Quaternion(1,Vector3d(0,0,0));
	this->velocity = Vector3d(0,0,0);
	this->angularVelocity = Vector3d(0,0,0);
}

/**
	A copy constructor.
*/
RBState::RBState(const RBState& other){
	this->position = other.position;
	this->orientation = other.orientation;
	this->velocity = other.velocity;
	this->angularVelocity = other.angularVelocity;
}

/**
	and a copy operator	
*/
RBState& RBState::operator = (const RBState& other){
	this->position = other.position;
	this->orientation = other.orientation;
	this->velocity = other.velocity;
	this->angularVelocity = other.angularVelocity;
	return *this;
}

RBState::~RBState(void){
		
}


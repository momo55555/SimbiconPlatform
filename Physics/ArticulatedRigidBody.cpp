#include "stdafx.h"

#include "ArticulatedRigidBody.h"
#include "Joint.h"


ArticulatedRigidBody::ArticulatedRigidBody(void){
	this->pJoint = NULL;
	this->AFParent = NULL;
}

ArticulatedRigidBody::~ArticulatedRigidBody(void){
}

/**
	This method draws the current rigid body.
*/



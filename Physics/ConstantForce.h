#pragma once

#include "Force.h"

class ConstantForce : public Force{
protected:
	Vector3d f;
public:
	ConstantForce(RigidBody *bdy, const Point3d &localActingPoint, Vector3d value) : Force(bdy, localActingPoint){
		this->f = value;
	}

	~ConstantForce(void){

	}

	/**
		this method evaluates the value of the force. NOTE: the value returned by this method is the absolute value of the force, in world coordinates.
	*/
	virtual Vector3d evaluateForce(){
		return f;
	}

};

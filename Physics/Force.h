#pragma once

#include <Point3d.h>
#include <Vector3d.h>

#include "RigidBody.h"

/*=======================================================================================================================================================================*
 | This class implements an interface for forces that are acting on physical objects. Different forces, such ellastic forces, friction with the air, gravitational forces|
 | etc, can be obtained by implementing this interface.                                                                                                                  |
 *=======================================================================================================================================================================*/
class Force 
{
protected:
	//this is the physical object that the force is acting on.
	RigidBody* body;
	//and this is the point, in the body's local coordinate system, where the force is acting.
	Point3d	localActingPoint;
public:
	/**
		This is the constructor that should be used for forces that are not acting at the origin of the body.
	*/
	Force(RigidBody* body, const Point3d &localActingPoint){
		this->body = body;
		this->localActingPoint = localActingPoint;
	}

	/**
		and this is the constructor that should be used for forces that are acting at the origin of the body.
	*/
	Force(RigidBody* body){
		this->body = body;
		this->localActingPoint = Point3d(0,0,0);
	}

	/**
		The virtual destructor.
	*/
	virtual ~Force(void){
	}

	/**
		this method evaluates the value of the force. NOTE: the value returned by this method is the absolute value of the force, in world coordinates.
	*/
	virtual Vector3d evaluateForce() = 0;

	/**
		this method evaluates the torque that the force is generating about the center of mass of the body.
		 NOTE: the value returned by this method is the absolute value of the torque, in world coordinates.
	*/
	virtual Vector3d evaluateCorrespondingTorque(){
		return body->getWorldCoordinates(Vector3d(localActingPoint)).crossProductWith(evaluateForce());
	}

	/**
		return the force acting point, in local coordinates
	*/
	Point3d getLocalPoint(){
		return this->localActingPoint;
	}
};

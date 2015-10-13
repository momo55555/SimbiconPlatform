#pragma once

#include "ConstantForce.h"

class TimedConstantForce : public ConstantForce{
private:
	//this keeps track of when the force started acting
	double startTime;
	//and this keeps track of the duration of the force
	double duration;
public:
	TimedConstantForce(RigidBody *bdy, Point3d &localActingPoint, Vector3d value) : ConstantForce(bdy, localActingPoint, value){
		startTime = 0;
		duration = 0;
	}

	TimedConstantForce() : ConstantForce(NULL, Point3d(), Vector3d()){
		startTime = 0;
		duration = 0;
	}

	inline void setRigidBody(RigidBody* b){
		body = b;
	}

	inline void setStartingTime(double st){
		startTime = st;
	}

	inline void setDuration(double l){
		duration = l;
	}

	inline RigidBody* getBody(){
		return body;
	}

	inline double getDuration(){
		return duration;
	}

	inline Vector3d getForce(){
		return f;
	}


	inline void setForce(const Vector3d& fNew){
		this->f = fNew;
	}

	inline void setActingPoint(const Point3d& pNew){
		this->localActingPoint = pNew;
	}

	/**
		this method evaluates the value of the force. NOTE: the value returned by this method is the absolute value of the force, in world coordinates.
	*/
	Vector3d evaluateForceAtTime(double t){
		if (t>startTime && t<startTime+duration)
			return f;
		return Vector3d(0,0,0);
	}

	~TimedConstantForce(void){
	}
};

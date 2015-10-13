#include "stdafx.h"

#include "PlaneCDP.h"
#include "SphereCDP.h"
#include "CapsuleCDP.h"
#include "RigidBody.h"

PlaneCDP::~PlaneCDP(void){
}

	
/**
	Draw an outline of the capsule
*/
void PlaneCDP::draw(){
	//we won't draw the plane...
}


void PlaneCDP::updateToWorldPrimitive(){
//	bdy->state.orientation.fastRotate(p.n, &wP.n);
	wP.n = bdy->getWorldCoordinates(p.n);
	wP.p = bdy->getWorldCoordinates(p.p);
}

int PlaneCDP::computeCollisionsWithSphereCDP(SphereCDP* sp,  DynamicArray<ContactPoint> *cps){
	return getContactPoints(&this->wP, &sp->wS, cps);
}

int PlaneCDP::computeCollisionsWithCapsuleCDP(CapsuleCDP* c,  DynamicArray<ContactPoint> *cps){
	return getContactPoints(&this->wP, &c->wC, cps);
}


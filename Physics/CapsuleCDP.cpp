#include "stdafx.h"

#include "CapsuleCDP.h"
#include "SphereCDP.h"
#include "PlaneCDP.h"
#include "Segment.h"
#include "RigidBody.h"


CapsuleCDP::~CapsuleCDP(void){
}


/**
	draw an outline of the capsule
*/

void CapsuleCDP::updateToWorldPrimitive(){
	wC.p1 = bdy->getWorldCoordinates(c.p1);
	wC.p2 = bdy->getWorldCoordinates(c.p2);
	wC.radius = c.radius;
}

int CapsuleCDP::computeCollisionsWithSphereCDP(SphereCDP* sp,  DynamicArray<ContactPoint> *cps){
	return getContactPoints(&this->wC, &sp->wS, cps);
}

int CapsuleCDP::computeCollisionsWithPlaneCDP(PlaneCDP* p,  DynamicArray<ContactPoint> *cps){
	return getContactPoints(&this->wC, &p->wP, cps);
}


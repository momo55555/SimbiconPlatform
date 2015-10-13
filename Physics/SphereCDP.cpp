#include "stdafx.h"

#include "SphereCDP.h"
#include <Segment.h>
#include "CapsuleCDP.h"
#include "PlaneCDP.h"
#include "RigidBody.h"

SphereCDP::~SphereCDP(void){
}

/**
	updates the world sphere.
*/
void SphereCDP::updateToWorldPrimitive(){
	wS.pos = bdy->getWorldCoordinates(s.pos);
	wS.radius = s.radius;
}

int SphereCDP::computeCollisionsWithPlaneCDP(PlaneCDP* p,  DynamicArray<ContactPoint> *cps){
	return getContactPoints(&this->wS, &p->wP, cps);
}

int SphereCDP::computeCollisionsWithCapsuleCDP(CapsuleCDP* c,  DynamicArray<ContactPoint> *cps){
	return getContactPoints(&this->wS, &c->wC, cps);
}


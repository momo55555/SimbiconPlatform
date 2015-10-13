#pragma once

#include <PUtils.h>

#include <Point3d.h>
#include <Vector3d.h>
#include <Plane.h>

#include "CollisionDetectionPrimitive.h"


/*========================================================================================================================================================================*
 * This class implements a plane class that will be used as a collision detection primitive.                                                                              *
 * A plane is represented by the position of one point on the plane, as well as the normal (unit vector) to the plane. We will store these two quantities both in local   *
 * coordinates, and in world coordinates which will be used for the collision detection. NOTE: we will not be evaluating the collision between different planes because   *
 * I will assume that they are all part of fixed objects only.
 *========================================================================================================================================================================*/
class PlaneCDP : public CollisionDetectionPrimitive{
	friend class SphereCDP;
	friend class CapsuleCDP;
private:
	//this is the plane, expressed in the local coordinates of the rigid body that owns it
	Plane p;
	//and this is the plane expressed in world coordinates
	Plane wP;
	
public:
	PlaneCDP(const Vector3d& normal, const Point3d& origin, RigidBody* theBody = NULL ) :
	  CollisionDetectionPrimitive( PLANE_CDP, theBody )
	{
		p.n = normal;
		p.n.toUnit();
		p.p = origin;
	}
	PlaneCDP(RigidBody* theBody = NULL ) :
		CollisionDetectionPrimitive( PLANE_CDP, theBody ) {}
	~PlaneCDP(void);

	virtual char* save() { return "PlaneCDP"; }

	virtual void updateToWorldPrimitive();

	virtual void draw();
	const Vector3d& getNormal() const {return p.n;}
	inline void setNormal( const Vector3d& normal ) { p.n = normal; }
	const Point3d& getOrigin() const {return p.p;};
	inline void setOrigin( const Point3d& origin ) { p.p = origin; }

	virtual int computeCollisionsWith(CollisionDetectionPrimitive* other,  DynamicArray<ContactPoint> *cps){
		//we don't know what the other collision detection primitive is, but we know this one is a plane, so make
		//other compute the contact points with this plane
		int oldContactCount = cps->size();
		int nContacts = other->computeCollisionsWithPlaneCDP(this, cps);

		//make sure the normals still point from other to this...
		for (uint i=oldContactCount;i<cps->size();i++)
			((*cps)[i]).n *= -1;
		return nContacts;
	}

	virtual int computeCollisionsWithSphereCDP(SphereCDP* sp,  DynamicArray<ContactPoint> *cps);

	virtual int computeCollisionsWithPlaneCDP(PlaneCDP* sp,  DynamicArray<ContactPoint> *cps){return 0;}
	virtual int computeCollisionsWithCapsuleCDP(CapsuleCDP* c,  DynamicArray<ContactPoint> *cps);
	virtual int computeCollisionsWithBoxCDP(BoxCDP* sp,  DynamicArray<ContactPoint> *cps){return 0;}

};



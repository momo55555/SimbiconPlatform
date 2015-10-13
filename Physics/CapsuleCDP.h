#pragma once

#include <PUtils.h>

#include <Point3d.h>
#include <TransformationMatrix.h>
#include <Capsule.h>

#include "CollisionDetectionPrimitive.h"
#include "collisionLibrary.h"

/*========================================================================================================================================================================*
 * This class implements a capsule class that will be used as a collision detection primitive.                                                                            *
 * A capsule is represented by the position of the two end points and the radius length. We will also store a temp position for the world coordinates of the endppoints   * 
 * of the capsule. This will be used when evaluating the contact points with other primitives, and it needs to be updated any time the world position of the object that  *                      
 * owns this capsule changes.                                                                                                                                             *
 *========================================================================================================================================================================*/
class CapsuleCDP : public CollisionDetectionPrimitive{
friend class SphereCDP;
friend class PlaneCDP;

private:
	//a capsule is really just an infinite number of spheres that have the center along a segment. Therefore, to define the capsule we need the
	//two end points and the radius
	Capsule c;
	Capsule wC;
public:
	CapsuleCDP(Point3d& point1, Point3d& point2, double radius, RigidBody* theBody = NULL) :
	  CollisionDetectionPrimitive( CAPSULE_CDP, theBody ) {
		c.p1 = point1;
		c.p2 = point2;
		c.radius = radius; 
	}
	CapsuleCDP(RigidBody* theBody = NULL) :
		CollisionDetectionPrimitive( CAPSULE_CDP, theBody ) {}
	~CapsuleCDP(void);
	
	virtual char* save() { return "CapsuleCDP"; }

	virtual void updateToWorldPrimitive();

	/**
		return the radius of the sphere
	*/
	inline double getRadius(){
		return c.radius;
	}

	inline void setRadius( double radius ) {
		c.radius = radius;
	}

	/**
		return the position of the first endpoint.
	*/
	inline const Point3d& getPoint1(){
		return c.p1;
	}

	inline void setPoint1( const Point3d& point1 ) {
		c.p1 = point1;
	}

	/**
		return the position of the first endpoint.
	*/
	inline const Point3d& getPoint2(){
		return c.p2;
	}

	inline void setPoint2( const Point3d& point2 ) {
		c.p2 = point2;
	}

	virtual int computeCollisionsWith(CollisionDetectionPrimitive* other,  DynamicArray<ContactPoint> *cps){
		//we don't know what the other collision detection primitive is, but we know this one is a capsule, so make
		//other compute the contact points with this capsule
		int oldContactCount = cps->size();
		int nContacts = other->computeCollisionsWithCapsuleCDP(this, cps);

		//make sure the normals still point from other to this...
		for (uint i=oldContactCount;i<cps->size();i++)
			((*cps)[i]).n *= -1;
		return nContacts;
	}

	virtual int computeCollisionsWithSphereCDP(SphereCDP* sp,  DynamicArray<ContactPoint> *cps);

	virtual int computeCollisionsWithPlaneCDP(PlaneCDP* p,  DynamicArray<ContactPoint> *cps);

	virtual int computeCollisionsWithCapsuleCDP(CapsuleCDP* c,  DynamicArray<ContactPoint> *cps){
		return getContactPoints(&this->wC, &c->wC, cps);
	}

	virtual int computeCollisionsWithBoxCDP(BoxCDP* sp,  DynamicArray<ContactPoint> *cps){return 0;}

};

#pragma once

#include <PUtils.h>

#include <Point3d.h>
#include <Sphere.h>
#include <TransformationMatrix.h>

#include "CollisionDetectionPrimitive.h"
#include "collisionLibrary.h"

/*========================================================================================================================================================================*
 * This class implements a sphere class that will be used as a collision detection primitive.                                                                             *
 * A sphere is represented by the position of the center and the radius length. We will also store a temp position for the world coordinates of the center of the sphere. *
 * This will be used when evaluating the contact points with other primitives, and it will be automatically 
 *========================================================================================================================================================================*/
class SphereCDP : public CollisionDetectionPrimitive{
friend class CapsuleCDP;
friend class PlaneCDP;
friend class RigidBody;
public:
	//keep track of the local-coordinates sphere used by this collision detection primitive
	Sphere s;
	//and this is the sphere, expressed in world coordinates
	Sphere wS;
public:
	SphereCDP(Point3d& center, double radius, RigidBody* theBody = NULL ) :
	  CollisionDetectionPrimitive( SPHERE_CDP, theBody ) {
		s.pos = center;
		s.radius = radius;
	  }

	SphereCDP( RigidBody* theBody = NULL ) :
	  CollisionDetectionPrimitive( SPHERE_CDP, theBody ) {}
	
	virtual ~SphereCDP(void);

	virtual char* save() { return "SphereCDP"; }

	virtual void updateToWorldPrimitive();

	/**
		return the radius of the sphere
	*/
	inline double getRadius(){
		return s.radius;
	}

	inline void setRadius( double radius ) {
		s.radius = radius;
	}


	/**
		return the center of the sphere, expressed in local coordinates
	*/
	inline const Point3d& getCenter(){
		return s.pos;
	}

	inline void setCenter( const Point3d& center ) {
		s.pos = center;
	}


	virtual int computeCollisionsWith(CollisionDetectionPrimitive* other,  DynamicArray<ContactPoint> *cps){
		//we don't know what the other collision detection primitive is, but we know this one is a sphere, so make
		//other compute the contact points with this sphere
		int oldContactCount = cps->size();
		int nContacts = other->computeCollisionsWithSphereCDP(this, cps);

		//make sure the normals still point from other to this...
		for (uint i=oldContactCount;i<cps->size();i++)
			((*cps)[i]).n *= -1;
		return nContacts;
	}

	virtual int computeCollisionsWithSphereCDP(SphereCDP* sp,  DynamicArray<ContactPoint> *cps){
		return getContactPoints(&this->wS, &sp->wS, cps);
	}
	
	virtual int computeCollisionsWithPlaneCDP(PlaneCDP* p,  DynamicArray<ContactPoint> *cps);
	virtual int computeCollisionsWithCapsuleCDP(CapsuleCDP* sp,  DynamicArray<ContactPoint> *cps);
	virtual int computeCollisionsWithBoxCDP(BoxCDP* sp,  DynamicArray<ContactPoint> *cps){return 0;}

};

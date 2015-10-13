#pragma once

#include <typeinfo>

#include <TransformationMatrix.h>

#include "ContactPoint.h"
#include "PUtils.h"

/*========================================================================================================================================================================*
 * This class implements an interface for collision detection primitives such as spheres, capsules and so on.                                                             *
 *========================================================================================================================================================================*/

class SphereCDP;
class CapsuleCDP;
class PlaneCDP;
class RigidBody;
class BoxCDP;

#define UNKNOWN_CDP 0
#define SPHERE_CDP 1
#define CAPSULE_CDP 2
#define PLANE_CDP 3
#define BOX_CDP 4

class CollisionDetectionPrimitive{
protected:
	//keep track of the rigid body that this collision detection primitive belongs to - useful to update world coordinates, etc
	int type;
	RigidBody* bdy;

public:
	CollisionDetectionPrimitive(int type, RigidBody* theBody = NULL ) :
	  type(type), bdy( theBody ) {}
	virtual ~CollisionDetectionPrimitive(void);

	const char* typeName(){
		return typeid(*this).name();
	}

	virtual char* save() = 0;

	void attachBody( RigidBody* body ) {
		bdy = body;
	}

	virtual void updateToWorldPrimitive() = 0;

	/**
		draw an outline of the primitive...
	*/

	/**
		returns the type of this collision detection primitive.
	*/
	inline int getType(){return type;}

	virtual int computeCollisionsWith(CollisionDetectionPrimitive* other,  DynamicArray<ContactPoint> *cps) = 0;
	virtual int computeCollisionsWithSphereCDP(SphereCDP* sp,  DynamicArray<ContactPoint> *cps) = 0;
	virtual int computeCollisionsWithPlaneCDP(PlaneCDP* sp,  DynamicArray<ContactPoint> *cps) = 0;
	virtual int computeCollisionsWithCapsuleCDP(CapsuleCDP* sp,  DynamicArray<ContactPoint> *cps) = 0;
	virtual int computeCollisionsWithBoxCDP(BoxCDP* sp,  DynamicArray<ContactPoint> *cps) = 0;

};

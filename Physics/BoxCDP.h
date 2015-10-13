#pragma once

#include <Point3d.h>
#include <TransformationMatrix.h>

#include "CollisionDetectionPrimitive.h"

/*========================================================================================================================================================================*
 * This class implements a rectangular box class that will be used as a collision detection primitive.                                                                    *
 * A box is represented by the position of two opposite corners.                                                                                                          *
 *========================================================================================================================================================================*/
class BoxCDP : public CollisionDetectionPrimitive{
private:
	//these are the two corners of the box, expressed in local coordinates.
	Point3d p1, p2;

public:
	BoxCDP(const Point3d& point1, const Point3d& point2, RigidBody* theBody = NULL) :
		CollisionDetectionPrimitive( BOX_CDP, theBody ),
		p1(point1),
		p2(point2) {}
	BoxCDP(RigidBody* theBody = NULL ) :
		CollisionDetectionPrimitive( BOX_CDP, theBody ) {}
	virtual ~BoxCDP(void);

	virtual char* save() { return "BoxCDP"; }

	virtual void updateToWorldPrimitive(){}


	const Point3d& getPoint1() const { return p1; }
	inline void setPoint1( const Point3d& point1 ) { p1 = point1; }
	const Point3d& getPoint2() const { return p2; }
	inline void setPoint2( const Point3d& point2 ) { p2 = point2; }

	/**
		return the center of the box, expressed in local coordinates
	*/
	inline Point3d getCenter(){
		return Point3d((p1.x+p2.x)/2, (p1.y+p2.y)/2, (p1.z+p2.z)/2);
	}

	/**
		returns the length in the x-direction
	*/
	inline double getXLen(){
		return (fabs(p1.x-p2.x));
	}

	/**
		returns the length in the y-direction
	*/
	inline double getYLen(){
		return (fabs(p1.y-p2.y));
	}

	/**
		returns the length in the z-direction
	*/
	inline double getZLen(){
		return (fabs(p1.z-p2.z));
	}

	virtual int computeCollisionsWith(CollisionDetectionPrimitive* other,  DynamicArray<ContactPoint> *cps){
/*
		//we don't know what the other collision detection primitive is, but we know this one is a box, so make
		//other compute the contact points with this box
		int oldContactCount = cps->size();
		int nContacts = other->computeCollisionsWithBoxCDP(this, cps);

		//make sure the normals still point from other to this...
		for (uint i=oldContactCount;i<cps->size();i++)
			((*cps)[i]).n *= -1;
		return nContacts;
*/

		//NOT YET IMPLEMENTED!!!
		return 0;

	}

	virtual int computeCollisionsWithSphereCDP(SphereCDP* sp,  DynamicArray<ContactPoint> *cps){return 0;}
	virtual int computeCollisionsWithPlaneCDP(PlaneCDP* sp,  DynamicArray<ContactPoint> *cps){return 0;}
	virtual int computeCollisionsWithCapsuleCDP(CapsuleCDP* sp,  DynamicArray<ContactPoint> *cps){return 0;}
	virtual int computeCollisionsWithBoxCDP(BoxCDP* sp,  DynamicArray<ContactPoint> *cps){return 0;}

};
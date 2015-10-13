/**
	A bunch of useful collision detection methods for different primitives are implemented here
*/
#pragma once

#include <mathLib.h>
#include <Sphere.h>
#include <Plane.h>
#include <Capsule.h>
#include <Segment.h>

#include "ContactPoint.h"

#define RETURN_CONTACTS_REVERSE_ORDER(a, b, cps)	{					\
	int start = cps->size();											\
	int n = getContactPoints(b, a, cps);								\
	for (int i=0;i<n;i++)												\
		((*cps)[start+i]).n *= -1;										\
	return n;															\
}																		\



/**
	collision between two spheres
*/
inline int getContactPoints(Sphere* a, Sphere* b, DynamicArray<ContactPoint> *cps)
{
	//compute the distance between the origins of the two spheres. If they are closer than a.r + b.r, then they've collided
	double distSquared = SQR(b->pos.x - a->pos.x) + SQR(b->pos.y - a->pos.y) + SQR(b->pos.z - a->pos.z);
	if (distSquared <= SQR(a->radius + b->radius)){
		cps->push_back(ContactPoint());
		ContactPoint* lastPoint = &(*cps)[cps->size()-1];
		double dist = sqrt(distSquared);
		//set the penetration depth
		lastPoint->d = (a->radius + b->radius - dist);
		//set the normal (pointing from b into a)
		lastPoint->n.setToVectorBetween(b->pos, a->pos); lastPoint->n /= dist;
		lastPoint->cp.setToOffsetFromPoint(b->pos, lastPoint->n, b->radius - lastPoint->d/2.0);
		return 1;
	}
	return 0;
}

/**
	collision between a sphere and a plane
*/
inline int getContactPoints(Sphere* a, Plane* b, DynamicArray<ContactPoint> *cps){
	//compute the distance between the plane and the sphere's origin
	Vector3d v;
	v.setToVectorBetween(b->p, a->pos);
	double dist = v.dotProductWith(b->n);
	if (dist <= a->radius){
		cps->push_back(ContactPoint());
		ContactPoint* lastPoint = &(*cps)[cps->size()-1];
		//set the penetration depth
		lastPoint->d = (a->radius - dist);
		//set the normal (pointing from b into a, and the normal of b must points towards a)
		lastPoint->n = b->n;
		lastPoint->cp.setToOffsetFromPoint(a->pos, lastPoint->n, -a->radius);
		return 1;
	}
	return 0;
}

/**
	collision between a plane and a sphere
*/
inline int getContactPoints(Plane* a, Sphere* b, DynamicArray<ContactPoint> *cps){
	RETURN_CONTACTS_REVERSE_ORDER(a, b, cps);
}

/**
	collision between a capsule and a plane
*/
inline int getContactPoints(Capsule* a, Plane* b, DynamicArray<ContactPoint> *cps){
	//we'll look at the spheres that start at the end points of the capsule, and see if they interesect the plane
	Sphere tmpSphere(a->p1, a->radius);
	int n = getContactPoints(&tmpSphere, b, cps);
	tmpSphere.pos = a->p2;
	n += getContactPoints(&tmpSphere, b, cps);
	return n;
}

/**
	collision between a plane and a capsule
*/
inline int getContactPoints(Plane* a, Capsule* b, DynamicArray<ContactPoint> *cps){
	RETURN_CONTACTS_REVERSE_ORDER(a, b, cps);
}

/**
	collision between a capsule and a sphere
*/
inline int getContactPoints(Capsule* a, Sphere* b, DynamicArray<ContactPoint> *cps){
	//a capsule is really an infinite number of spheres, whose centers lie on a segment.
	//so we need to find the point on the segment that is closest to the center of the sphere,
	//and then we'll use the two spheres that 
	Segment s(a->p1, a->p2);
	Sphere tmpSphere;
	s.getClosestPointTo(b->pos, &tmpSphere.pos);
	tmpSphere.radius = a->radius;
	return getContactPoints(&tmpSphere, b, cps);
}

/**
	collision between a sphere and a capsule
*/
inline int getContactPoints(Sphere* a, Capsule* b, DynamicArray<ContactPoint> *cps){
	RETURN_CONTACTS_REVERSE_ORDER(a, b, cps);
}

/**
	collision between two capsules
*/
inline int getContactPoints(Capsule* a, Capsule* b, DynamicArray<ContactPoint> *cps){
	//a capsule is really an infinite number of spheres, whose centers lie on a segment.
	//so we need to find the closest segment between the segments of the two capsules,
	//then we'll place some spheres at the end points of that segment, and get the collisions
	//of that the 
	Segment s;

	Segment(a->p1, a->p2).getShortestSegmentTo(Segment(b->p1, b->p2), &s);
	
	return getContactPoints(&Sphere(s.a, a->radius), &Sphere(s.b, b->radius), cps);
}


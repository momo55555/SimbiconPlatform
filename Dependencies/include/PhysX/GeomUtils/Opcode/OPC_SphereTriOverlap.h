// This code contains NVIDIA Confidential Information and is disclosed to you 
// under a form of NVIDIA software license agreement provided separately to you.
//
// Notice
// NVIDIA Corporation and its licensors retain all intellectual property and
// proprietary rights in and to this software and related documentation and 
// any modifications thereto. Any use, reproduction, disclosure, or 
// distribution of this software and related documentation without an express 
// license agreement from NVIDIA Corporation is strictly prohibited.
// 
// ALL NVIDIA DESIGN SPECIFICATIONS, CODE ARE PROVIDED "AS IS.". NVIDIA MAKES
// NO WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ALL IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE.
//
// Information and code furnished is believed to be accurate and reliable.
// However, NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2008-2011 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  


#ifndef OPC_SPHERE_TRI_OVERLAP_H
#define OPC_SPHERE_TRI_OVERLAP_H

#include "CmSimd.h"
#include "Opcode.h"

namespace physx
{
namespace Ice
{

// This is collision detection. If you do another distance test for collision *response*,
// if might be useful to simply *skip* the test below completely, and report a collision.
// - if sphere-triangle overlap, result is ok
// - if they don't, we'll discard them during collision response with a similar test anyway
// Overall this approach should run faster.

// Original code by David Eberly in Magic.
PX_FORCE_INLINE Ps::IntBool SphereCollider::SphereTriOverlap(const PxVec3& vert0, const PxVec3& vert1, const PxVec3& vert2)
{
	// Early exit if one of the vertices is inside the sphere
	PxVec3 kDiff = vert2 - mCenter;
	float fC = kDiff.magnitudeSquared();
	if(fC <= mRadius2)	return Ps::IntTrue;

	kDiff = vert1 - mCenter;
	fC = kDiff.magnitudeSquared();
	if(fC <= mRadius2)	return Ps::IntTrue;

	kDiff = vert0 - mCenter;
	fC = kDiff.magnitudeSquared();
	if(fC <= mRadius2)	return Ps::IntTrue;

	// Else do the full distance test
	const PxVec3 TriEdge0	= vert1 - vert0;
	const PxVec3 TriEdge1	= vert2 - vert0;

//Point kDiff	= vert0 - mCenter;
	const float fA00	= TriEdge0.magnitudeSquared();
	const float fA01	= TriEdge0.dot(TriEdge1);
	const float fA11	= TriEdge1.magnitudeSquared();
	const float fB0		= kDiff.dot(TriEdge0);
	const float fB1		= kDiff.dot(TriEdge1);
//float fC	= kDiff.SquareMagnitude();
	const float fDet	= PxAbs(fA00*fA11 - fA01*fA01);
	float u		= fA01*fB1-fA11*fB0;
	float v		= fA01*fB0-fA00*fB1;
	float SqrDist;

	if(u + v <= fDet)
	{
		if(u < 0.0f)
		{
			if(v < 0.0f)  // region 4
			{
				if(fB0 < 0.0f)
				{
//					v = 0.0f;
					if(-fB0>=fA00)			{ /*u = 1.0f;*/		SqrDist = fA00+2.0f*fB0+fC;	}
					else					{ u = -fB0/fA00;	SqrDist = fB0*u+fC;			}
				}
				else
				{
//					u = 0.0f;
					if(fB1>=0.0f)			{ /*v = 0.0f;*/		SqrDist = fC;				}
					else if(-fB1>=fA11)		{ /*v = 1.0f;*/		SqrDist = fA11+2.0f*fB1+fC;	}
					else					{ v = -fB1/fA11;	SqrDist = fB1*v+fC;			}
				}
			}
			else  // region 3
			{
//				u = 0.0f;
				if(fB1>=0.0f)				{ /*v = 0.0f;*/		SqrDist = fC;				}
				else if(-fB1>=fA11)			{ /*v = 1.0f;*/		SqrDist = fA11+2.0f*fB1+fC;	}
				else						{ v = -fB1/fA11;	SqrDist = fB1*v+fC;			}
			}
		}
		else if(v < 0.0f)  // region 5
		{
//			v = 0.0f;
			if(fB0>=0.0f)					{ /*u = 0.0f;*/		SqrDist = fC;				}
			else if(-fB0>=fA00)				{ /*u = 1.0f;*/		SqrDist = fA00+2.0f*fB0+fC;	}
			else							{ u = -fB0/fA00;	SqrDist = fB0*u+fC;			}
		}
		else  // region 0
		{
			// minimum at interior point
			if(fDet==0.0f)
			{
//				u = 0.0f;
//				v = 0.0f;
				SqrDist = PX_MAX_F32;
			}
			else
			{
				float fInvDet = 1.0f/fDet;
				u *= fInvDet;
				v *= fInvDet;
				SqrDist = u*(fA00*u+fA01*v+2.0f*fB0) + v*(fA01*u+fA11*v+2.0f*fB1)+fC;
			}
		}
	}
	else
	{
		float fTmp0, fTmp1, fNumer, fDenom;

		if(u < 0.0f)  // region 2
		{
			fTmp0 = fA01 + fB0;
			fTmp1 = fA11 + fB1;
			if(fTmp1 > fTmp0)
			{
				fNumer = fTmp1 - fTmp0;
				fDenom = fA00-2.0f*fA01+fA11;
				if(fNumer >= fDenom)
				{
//					u = 1.0f;
//					v = 0.0f;
					SqrDist = fA00+2.0f*fB0+fC;
				}
				else
				{
					u = fNumer/fDenom;
					v = 1.0f - u;
					SqrDist = u*(fA00*u+fA01*v+2.0f*fB0) + v*(fA01*u+fA11*v+2.0f*fB1)+fC;
				}
			}
			else
			{
//				u = 0.0f;
				if(fTmp1 <= 0.0f)		{ /*v = 1.0f;*/		SqrDist = fA11+2.0f*fB1+fC;	}
				else if(fB1 >= 0.0f)	{ /*v = 0.0f;*/		SqrDist = fC;				}
				else					{ v = -fB1/fA11;	SqrDist = fB1*v+fC;			}
			}
		}
		else if(v < 0.0f)  // region 6
		{
			fTmp0 = fA01 + fB1;
			fTmp1 = fA00 + fB0;
			if(fTmp1 > fTmp0)
			{
				fNumer = fTmp1 - fTmp0;
				fDenom = fA00-2.0f*fA01+fA11;
				if(fNumer >= fDenom)
				{
//					v = 1.0f;
//					u = 0.0f;
					SqrDist = fA11+2.0f*fB1+fC;
				}
				else
				{
					v = fNumer/fDenom;
					u = 1.0f - v;
					SqrDist = u*(fA00*u+fA01*v+2.0f*fB0) + v*(fA01*u+fA11*v+2.0f*fB1)+fC;
				}
			}
			else
			{
//				v = 0.0f;
				if(fTmp1 <= 0.0f)		{ /*u = 1.0f;*/		SqrDist = fA00+2.0f*fB0+fC;	}
				else if(fB0 >= 0.0f)	{ /*u = 0.0f;*/		SqrDist = fC;				}
				else					{ u = -fB0/fA00;	SqrDist = fB0*u+fC;			}
			}
		}
		else  // region 1
		{
			fNumer = fA11 + fB1 - fA01 - fB0;
			if(fNumer <= 0.0f)
			{
//				u = 0.0f;
//				v = 1.0f;
				SqrDist = fA11+2.0f*fB1+fC;
			}
			else
			{
				fDenom = fA00-2.0f*fA01+fA11;
				if(fNumer >= fDenom)
				{
//					u = 1.0f;
//					v = 0.0f;
					SqrDist = fA00+2.0f*fB0+fC;
				}
				else
				{
					u = fNumer/fDenom;
					v = 1.0f - u;
					SqrDist = u*(fA00*u+fA01*v+2.0f*fB0) + v*(fA01*u+fA11*v+2.0f*fB1)+fC;
				}
			}
		}
	}

	return PxAbs(SqrDist) < mRadius2;
}

#ifndef OPC_SUPPORT_VMX128

// Do a partial AABB against triangle overlap test.
PX_FORCE_INLINE	 Ps::IntBool SphereCollider::LooseSphereTriOverlap(const PxVec3& vert0, const PxVec3& vert1, const PxVec3& vert2)
{
	const PxVec3 sphereMin(mCenter.x - mRadius, mCenter.y - mRadius, mCenter.z - mRadius);
	const PxVec3 sphereMax(mCenter.x + mRadius, mCenter.y + mRadius, mCenter.z + mRadius);

	// Test triangle AABB
	const PxVec3 triMin = PxVec3(PxMin(vert0.x,PxMin(vert1.x, vert2.x)), PxMin(vert0.y,PxMin(vert1.y, vert2.y)), PxMin(vert0.z,PxMin(vert1.z, vert2.z)));
	const PxVec3 triMax = PxVec3(PxMax(vert0.x,PxMax(vert1.x, vert2.x)), PxMax(vert0.y,PxMax(vert1.y, vert2.y)), PxMax(vert0.z,PxMax(vert1.z, vert2.z)));

	if((triMin.x > sphereMax.x) || (triMax.x < sphereMin.x)) return Ps::IntFalse;
	if((triMin.y > sphereMax.y) || (triMax.y < sphereMin.y)) return Ps::IntFalse;
	if((triMin.z > sphereMax.z) || (triMax.z < sphereMin.z)) return Ps::IntFalse;

	// Test the triangle plane.
	PxVec3 normal = (vert1 - vert0).cross(vert2 - vert0);
	float dist = normal.dot(vert0);

	// find the minimum maximum on normal.
	PxVec3 vMin, vMax;

	if(normal.x > 0) 
	{
		vMin.x = sphereMin.x;
		vMax.x = sphereMax.x;
	}
	else
	{
		vMin.x = sphereMax.x;
		vMax.x = sphereMin.x;
	}

	if(normal.y > 0) 
	{
		vMin.y = sphereMin.y;
		vMax.y = sphereMax.y;
	}
	else
	{
		vMin.y = sphereMax.y;
		vMax.y = sphereMin.y;
	}


	if(normal.z > 0) 
	{
		vMin.z = sphereMin.z;
		vMax.z = sphereMax.z;
	}
	else
	{
		vMin.z = sphereMax.z;
		vMax.z = sphereMin.z;
	}

	// are they disjoint?

	float minDist = vMin.dot(normal);
	float maxDist = vMax.dot(normal);

	if((minDist > dist) || (maxDist < dist))
		return Ps::IntFalse;

	// Test edge axes.

	PxVec3 axis;
	float p0, p1, p2, pMin, pMax, axisRadius;

	const PxVec3 v0 = vert0 - mCenter;
	const PxVec3 v1 = vert1 - mCenter;
	const PxVec3 v2 = vert2 - mCenter;

	const PxVec3 edge0 = v1 - v0;
	const PxVec3 edge1 = v2 - v1;
	const PxVec3 edge2 = v0 - v2;

	PxVec3 radius(mRadius, mRadius, mRadius);

	/*
		y*other.z - z*other.y,
		z*other.x - x*other.z,
		x*other.y - y*other.x

		0 - 0
		0 - e0.z
		e0.y - 0
	*/


	// axis == [1,0,0] x e0 == [0, -e0.z, e0.y]
	// x, y, z, w,    x, y, z, w
	// 0, 1, 2, 3,    4, 5, 6, 7

	

	axis = Ps::cross100(edge0);
	p0 = axis.dot(v0);
	p2 = axis.dot(v2);
	pMin = PxMin(p0, p2);
	pMax = PxMax(p0, p2);
	axisRadius = radius.dot(PxVec3(PxAbs(axis.x), PxAbs(axis.y), PxAbs(axis.z)));
	// we test against the sphere radius, not the box radius on the axis...(so this isnt a standard AABB test).
	if((pMin > axisRadius) || (pMax < -axisRadius))
		return Ps::IntFalse;

	// axis == [1,0,0] x e1 == [0, -e1.z, e1.y]
	axis = Ps::cross100(edge1);
	p0 = axis.dot(v0);
	p1 = axis.dot(v1);
	pMin = PxMin(p0, p1);
	pMax = PxMax(p0, p1);
	axisRadius = radius.dot(PxVec3(PxAbs(axis.x), PxAbs(axis.y), PxAbs(axis.z)));
	// we test against the sphere radius, not the box radius on the axis...(so this isnt a standard AABB test).
	if((pMin > axisRadius) || (pMax < -axisRadius))
		return Ps::IntFalse;

	// axis == [1,0,0] x e2 == [0, -e2.z, e2.y]
	axis = Ps::cross100(edge2);
	p0 = axis.dot(v0);
	p1 = axis.dot(v1);
	pMin = PxMin(p0, p1);
	pMax = PxMax(p0, p1);
	axisRadius = radius.dot(PxVec3(PxAbs(axis.x), PxAbs(axis.y), PxAbs(axis.z)));
	// we test against the sphere radius, not the box radius on the axis...(so this isnt a standard AABB test).
	if((pMin > axisRadius) || (pMax < -axisRadius))
		return Ps::IntFalse;

	/*
		y*other.z - z*other.y,
		z*other.x - x*other.z,
		x*other.y - y*other.x

		e0.z - 0
		0 - 0
		0 - e0.x
	*/
	// axis == [0,1,0] x e0 == [e0.z, 0, -e0.x]
	// x, y, z, w,    x, y, z, w
	// 0, 1, 2, 3,    4, 5, 6, 7

	axis = Ps::cross010(edge0);
	p0 = axis.dot(v0);
	p2 = axis.dot(v2);
	pMin = PxMin(p0, p2);
	pMax = PxMax(p0, p2);
	axisRadius = radius.dot(PxVec3(PxAbs(axis.x), PxAbs(axis.y), PxAbs(axis.z)));
	// we test against the sphere radius, not the box radius on the axis...(so this isnt a standard AABB test).
	if((pMin > axisRadius) || (pMax < -axisRadius))
		return Ps::IntFalse;


	// axis == [0,1,0] x e1 == [e1.z, 0, -e1.x]
	axis = Ps::cross010(edge1);
	p0 = axis.dot(v0);
	p1 = axis.dot(v1);
	pMin = PxMin(p0, p1);
	pMax = PxMax(p0, p1);
	axisRadius = radius.dot(PxVec3(PxAbs(axis.x), PxAbs(axis.y), PxAbs(axis.z)));
	// we test against the sphere radius, not the box radius on the axis...(so this isnt a standard AABB test).
	if((pMin > axisRadius) || (pMax < -axisRadius))
		return Ps::IntFalse;

	// axis == [0, 1, 0] x e2 == [e2.z, 0, -e2.x]
	axis = Ps::cross010(edge2);
	p0 = axis.dot(v0);
	p1 = axis.dot(v1);
	pMin = PxMin(p0, p1);
	pMax = PxMax(p0, p1);
	axisRadius = radius.dot(PxVec3(PxAbs(axis.x), PxAbs(axis.y), PxAbs(axis.z)));
	// we test against the sphere radius, not the box radius on the axis...(so this isnt a standard AABB test).
	if((pMin > axisRadius) || (pMax < -axisRadius))
		return Ps::IntFalse;


	/*
		y*other.z - z*other.y,
		z*other.x - x*other.z,
		x*other.y - y*other.x

		0 - e0.y
		e0.x - 0
		0 - 0
	*/

	// axis == [0, 0, 1] x e0 == [-e0.y, e0.x, 0]
	// x, y, z, w,    x, y, z, w
	// 0, 1, 2, 3,    4, 5, 6, 7

	// axis == [0, 1, 0] x e2 == [e2.z, 0, -e2.x]
	axis = Ps::cross001(edge0);
	p0 = axis.dot(v0);
	p2 = axis.dot(v2);
	pMin = PxMin(p0, p2);
	pMax = PxMax(p0, p2);
	axisRadius = radius.dot(PxVec3(PxAbs(axis.x), PxAbs(axis.y), PxAbs(axis.z)));
	// we test against the sphere radius, not the box radius on the axis...(so this isnt a standard AABB test).
	if((pMin > axisRadius) || (pMax < -axisRadius))
		return Ps::IntFalse;


	// axis == [0, 0, 1] x e1 == [-e1.y, e1.x, 0]

	axis = Ps::cross001(edge1);
	p0 = axis.dot(v0);
	p1 = axis.dot(v1);
	pMin = PxMin(p0, p1);
	pMax = PxMax(p0, p1);
	axisRadius = radius.dot(PxVec3(PxAbs(axis.x), PxAbs(axis.y), PxAbs(axis.z)));
	// we test against the sphere radius, not the box radius on the axis...(so this isnt a standard AABB test).
	if((pMin > axisRadius) || (pMax < -axisRadius))
		return Ps::IntFalse;

	// axis == [0, 0, 1] x e2 == [-e2.y, e2.x, 0]

	axis = Ps::cross001(edge2);
	p0 = axis.dot(v0);
	p1 = axis.dot(v1);
	pMin = PxMin(p0, p1);
	pMax = PxMax(p0, p1);
	axisRadius = radius.dot(PxVec3(PxAbs(axis.x), PxAbs(axis.y), PxAbs(axis.z)));
	// we test against the sphere radius, not the box radius on the axis...(so this isnt a standard AABB test).
	if((pMin > axisRadius) || (pMax < -axisRadius))
		return Ps::IntFalse;

	return Ps::IntTrue;
}
#else

PX_FORCE_INLINE Ps::IntBool SphereCollider::LooseSphereTriOverlap(const PxVec3& vert0, const PxVec3& vert1, const PxVec3& vert2)
{
	Cm::PxSimd::Vector4 center = Cm::PxSimd::load(mCenter);
	Cm::PxSimd::Vector4 radius = Cm::PxSimd::splatX(Cm::PxSimd::load(mRadius));
	Cm::PxSimd::Vector4 zero = Cm::PxSimd::zero();

	Cm::PxSimd::Vector4 sphereMin = Cm::PxSimd::subtract(center, radius);
	Cm::PxSimd::Vector4 sphereMax = Cm::PxSimd::add(center, radius);

	return LooseSphereTriOverlap(vert0, vert1, vert2,
								center, radius,
								sphereMin, sphereMax,
								zero);
}

PX_FORCE_INLINE	Ps::IntBool SphereCollider::LooseSphereTriOverlap(
	const PxVec3& vert0, const PxVec3& vert1, const PxVec3& vert2,
	const Cm::PxSimd::Vector4 &center, const Cm::PxSimd::Vector4 &radius,
	const Cm::PxSimd::Vector4 &sphereMin, const Cm::PxSimd::Vector4 &sphereMax,
	const Cm::PxSimd::Vector4 &zero)
{

	//We could optimize further, but most of the time is spent in these first few instructions.

	//3 unaligned loads are painful:-(
	Cm::PxSimd::Vector4 v0 = Cm::PxSimd::load(vert0);
	Cm::PxSimd::Vector4 v1 = Cm::PxSimd::load(vert1);
	Cm::PxSimd::Vector4 v2 = Cm::PxSimd::load(vert2);

	// Test triangle AABB
	Cm::PxSimd::Vector4 triMin = Cm::PxSimd::minimum(v0, Cm::PxSimd::minimum(v1, v2));
	Cm::PxSimd::Vector4 triMax = Cm::PxSimd::maximum(v0, Cm::PxSimd::maximum(v1, v2));

	Cm::PxSimd::Vector4 mask = Cm::PxSimd::or4(Cm::PxSimd::greater(triMin, sphereMax), Cm::PxSimd::greater(sphereMin, triMax));
	mask = Cm::PxSimd::permuteXYZX(mask); //slightly more efficient than two swizzles.

	if(Cm::PxSimd::intNotEqual4Bool(mask, zero))
		return Ps::IntFalse;


	// Test the triangle plane.

	Cm::PxSimd::Vector4 normal = Cm::PxSimd::cross(Cm::PxSimd::subtract(v1, v0), Cm::PxSimd::subtract(v2, v0));
	Cm::PxSimd::Vector4 dist = Cm::PxSimd::dot(normal, v0);

	
	Cm::PxSimd::Vector4 vMask = Cm::PxSimd::greater(normal, Cm::PxSimd::zero());
	Cm::PxSimd::Vector4 vMin = Cm::PxSimd::select(sphereMax, sphereMin, vMask);
	Cm::PxSimd::Vector4 vMax = Cm::PxSimd::select(sphereMin, sphereMax, vMask);

	Cm::PxSimd::Vector4 minDist = Cm::PxSimd::dot(vMin, normal);
	Cm::PxSimd::Vector4 maxDist = Cm::PxSimd::dot(vMax, normal);

	mask = Cm::PxSimd::or4(Cm::PxSimd::greater(minDist, dist), Cm::PxSimd::less(maxDist, dist));
	if(Cm::PxSimd::intNotEqualBool(mask, Cm::PxSimd::zero()))
		return Ps::IntFalse;

	//////// test edge axis
	//transform the triangle.

	v0 = Cm::PxSimd::subtract(v0, center);
	v1 = Cm::PxSimd::subtract(v1, center);
	v2 = Cm::PxSimd::subtract(v2, center);

	Cm::PxSimd::Vector4 edge0 = Cm::PxSimd::subtract(v1, v0);
	Cm::PxSimd::Vector4 edge1 = Cm::PxSimd::subtract(v2, v1);
	Cm::PxSimd::Vector4 edge2 = Cm::PxSimd::subtract(v0, v2);

	// zero out w
	edge0 = Cm::PxSimd::and4(edge0, Cm::PxSimd::xyzMask());
	edge1 = Cm::PxSimd::and4(edge1, Cm::PxSimd::xyzMask());
	edge2 = Cm::PxSimd::and4(edge2, Cm::PxSimd::xyzMask());
	
	Cm::PxSimd::Vector4 minusEdge0 = Cm::PxSimd::subtract(Cm::PxSimd::zero(), edge0);
	Cm::PxSimd::Vector4 minusEdge1 = Cm::PxSimd::subtract(Cm::PxSimd::zero(), edge1);
	Cm::PxSimd::Vector4 minusEdge2 = Cm::PxSimd::subtract(Cm::PxSimd::zero(), edge2);

	Cm::PxSimd::Vector4 minusRadius = Cm::PxSimd::subtract(Cm::PxSimd::zero(), radius);


	Cm::PxSimd::Vector4 axis, p0, p1, p2, pMin, pMax, axisRadius, minusAxisRadius;

	/*
		y*other.z - z*other.y,
		z*other.x - x*other.z,
		x*other.y - y*other.x

		0 - 0
		0 - e0.z
		e0.y - 0
	*/


	// axis == [1,0,0] x e0 == [0, -e0.z, e0.y]
	// x, y, z, w,    x, y, z, w
	// 0, 1, 2, 3,    4, 5, 6, 7

	OPC_SIMD_PERMUTE(PERMUTE_0W1Z0Y0W, OPC_SIMD_0W, OPC_SIMD_1Z, OPC_SIMD_0Y, OPC_SIMD_0W);

	axis = Cm::PxSimd::permute(edge0, minusEdge0, PERMUTE_0W1Z0Y0W );
	p0 = Cm::PxSimd::dot(v0, axis);
	p2 = Cm::PxSimd::dot(v2, axis);
	pMin = Cm::PxSimd::minimum(p0, p2);
	pMax = Cm::PxSimd::maximum(p0, p2);
	axisRadius = Cm::PxSimd::dot(radius, Cm::PxSimd::abs(axis));
	minusAxisRadius = Cm::PxSimd::subtract(Cm::PxSimd::zero(), axisRadius);
	// we test against the sphere radius, not the box radius on the axis...(so this isnt a standard AABB test).
	if(Cm::PxSimd::greaterXBool(pMin, axisRadius) || Cm::PxSimd::lessXBool(pMax, minusAxisRadius))
		return Ps::IntFalse;

	// axis == [1,0,0] x e1 == [0, -e1.z, e1.y]
	axis = Cm::PxSimd::permute(edge1, minusEdge1, PERMUTE_0W1Z0Y0W );
	p0 = Cm::PxSimd::dot(v0, axis);
	p1 = Cm::PxSimd::dot(v1, axis);
	pMin = Cm::PxSimd::minimum(p0, p1);
	pMax = Cm::PxSimd::maximum(p0, p1);
	axisRadius = Cm::PxSimd::dot(radius, Cm::PxSimd::abs(axis));
	minusAxisRadius = Cm::PxSimd::subtract(Cm::PxSimd::zero(), axisRadius);
	if(Cm::PxSimd::greaterXBool(pMin, axisRadius) || Cm::PxSimd::lessXBool(pMax, minusAxisRadius))
		return Ps::IntFalse;


	// axis == [1,0,0] x e2 == [0, -e2.z, e2.y]
	axis = Cm::PxSimd::permute(edge2, minusEdge2, PERMUTE_0W1Z0Y0W);
	p0 = Cm::PxSimd::dot(v0, axis);
	p1 = Cm::PxSimd::dot(v1, axis);
	pMin = Cm::PxSimd::minimum(p0, p1);
	pMax = Cm::PxSimd::maximum(p0, p1);
	axisRadius = Cm::PxSimd::dot(radius, Cm::PxSimd::abs(axis));
	minusAxisRadius = Cm::PxSimd::subtract(Cm::PxSimd::zero(), axisRadius);
	if(Cm::PxSimd::greaterXBool(pMin, axisRadius) || Cm::PxSimd::lessXBool(pMax, minusAxisRadius))
		return Ps::IntFalse;

	/*
		y*other.z - z*other.y,
		z*other.x - x*other.z,
		x*other.y - y*other.x

		e0.z - 0
		0 - 0
		0 - e0.x
	*/
	// axis == [0,1,0] x e0 == [e0.z, 0, -e0.x]
	// x, y, z, w,    x, y, z, w
	// 0, 1, 2, 3,    4, 5, 6, 7

	OPC_SIMD_PERMUTE(PERMUTE_0Z0W1X0W, OPC_SIMD_0Z, OPC_SIMD_0W, OPC_SIMD_1X, OPC_SIMD_0W);

	axis = Cm::PxSimd::permute(edge0, minusEdge0, PERMUTE_0Z0W1X0W);
	p0 = Cm::PxSimd::dot(v0, axis);
	p2 = Cm::PxSimd::dot(v2, axis);
	pMin = Cm::PxSimd::minimum(p0, p2);
	pMax = Cm::PxSimd::maximum(p0, p2);
	axisRadius = Cm::PxSimd::dot(radius, Cm::PxSimd::abs(axis));
	minusAxisRadius = Cm::PxSimd::subtract(Cm::PxSimd::zero(), axisRadius);
	if(Cm::PxSimd::greaterXBool(pMin, axisRadius) || Cm::PxSimd::lessXBool(pMax, minusAxisRadius))
		return Ps::IntFalse;

	// axis == [0,1,0] x e1 == [e1.z, 0, -e1.x]
	axis = Cm::PxSimd::permute(edge1, minusEdge1, PERMUTE_0Z0W1X0W);
	p0 = Cm::PxSimd::dot(v0, axis);
	p1 = Cm::PxSimd::dot(v1, axis);
	pMin = Cm::PxSimd::minimum(p0, p1);
	pMax = Cm::PxSimd::maximum(p0, p1);
	axisRadius = Cm::PxSimd::dot(radius, Cm::PxSimd::abs(axis));
	minusAxisRadius = Cm::PxSimd::subtract(Cm::PxSimd::zero(), axisRadius);
	if(Cm::PxSimd::greaterXBool(pMin, axisRadius) || Cm::PxSimd::lessXBool(pMax, minusAxisRadius))
		return Ps::IntFalse;

	// axis == [0, 1, 0] x e2 == [e2.z, 0, -e2.x]
	axis = Cm::PxSimd::permute(edge2, minusEdge2, PERMUTE_0Z0W1X0W);
	p0 = Cm::PxSimd::dot(v0, axis);
	p1 = Cm::PxSimd::dot(v1, axis);
	pMin = Cm::PxSimd::minimum(p0, p1);
	pMax = Cm::PxSimd::maximum(p0, p1);
	axisRadius = Cm::PxSimd::dot(radius, Cm::PxSimd::abs(axis));
	minusAxisRadius = Cm::PxSimd::subtract(Cm::PxSimd::zero(), axisRadius);
	if(Cm::PxSimd::greaterXBool(pMin, axisRadius) || Cm::PxSimd::lessXBool(pMax, minusAxisRadius))
		return Ps::IntFalse;

	/*
		y*other.z - z*other.y,
		z*other.x - x*other.z,
		x*other.y - y*other.x

		0 - e0.y
		e0.x - 0
		0 - 0
	*/

	// axis == [0, 0, 1] x e0 == [-e0.y, e0.x, 0]
	// x, y, z, w,    x, y, z, w
	// 0, 1, 2, 3,    4, 5, 6, 7

	OPC_SIMD_PERMUTE(PERMUTE_1Y0X0W0W, OPC_SIMD_1Y, OPC_SIMD_0X, OPC_SIMD_0W, OPC_SIMD_0W);

	axis = Cm::PxSimd::permute(edge0, minusEdge0, PERMUTE_1Y0X0W0W);
	p0 = Cm::PxSimd::dot(v0, axis);
	p2 = Cm::PxSimd::dot(v2, axis);
	pMin = Cm::PxSimd::minimum(p0, p2);
	pMax = Cm::PxSimd::maximum(p0, p2);
	axisRadius = Cm::PxSimd::dot(radius, Cm::PxSimd::abs(axis));
	minusAxisRadius = Cm::PxSimd::subtract(Cm::PxSimd::zero(), axisRadius);
	if(Cm::PxSimd::greaterXBool(pMin, axisRadius) || Cm::PxSimd::lessXBool(pMax, minusAxisRadius))
		return Ps::IntFalse;

	// axis == [0, 0, 1] x e1 == [-e1.y, e1.x, 0]
	axis = Cm::PxSimd::permute(edge1, minusEdge1, PERMUTE_1Y0X0W0W);
	p0 = Cm::PxSimd::dot(v0, axis);
	p1 = Cm::PxSimd::dot(v1, axis);
	pMin = Cm::PxSimd::minimum(p0, p1);
	pMax = Cm::PxSimd::maximum(p0, p1);
	axisRadius = Cm::PxSimd::dot(radius, Cm::PxSimd::abs(axis));
	minusAxisRadius = Cm::PxSimd::subtract(Cm::PxSimd::zero(), axisRadius);
	if(Cm::PxSimd::greaterXBool(pMin, axisRadius) || Cm::PxSimd::lessXBool(pMax, minusAxisRadius))
		return Ps::IntFalse;

	// axis == [0, 0, 1] x e2 == [-e2.y, e2.x, 0]
	axis = Cm::PxSimd::permute(edge2, minusEdge2, PERMUTE_1Y0X0W0W);
	p0 = Cm::PxSimd::dot(v0, axis);
	p1 = Cm::PxSimd::dot(v1, axis);
	pMin = Cm::PxSimd::minimum(p0, p1);
	pMax = Cm::PxSimd::maximum(p0, p1);
	axisRadius = Cm::PxSimd::dot(radius, Cm::PxSimd::abs(axis));
	minusAxisRadius = Cm::PxSimd::subtract(Cm::PxSimd::zero(), axisRadius);
	if(Cm::PxSimd::greaterXBool(pMin, axisRadius) || Cm::PxSimd::lessXBool(pMax, minusAxisRadius))
		return Ps::IntFalse;

	return Ps::IntTrue;
}

#endif

} // namespace Ice

}

#endif
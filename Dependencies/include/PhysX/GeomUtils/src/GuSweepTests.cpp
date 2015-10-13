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


#include "PsIntrinsics.h"
#include "PxIntrinsics.h"
#include "GuSweepTests.h"
#include "PxSceneQueryReport.h"

#include "GuGJKObjectSupport.h"
#include "GuHeightFieldUtil.h"
#include "GuEntityReport.h"
#include "CmScaling.h"
#include "PsArray.h"
#include "PsUtilities.h"
#include "GuGeometryQuery.h"

#include "PxCapsuleGeometry.h"
#include "PxSphereGeometry.h"
#include "PxBoxGeometry.h"
#include "PxPlaneGeometry.h"
#include "PxConvexMeshGeometry.h"
#include "PxTriangleMeshGeometry.h"

#include "GuDistanceSegmentTriangle.h"
#include "GuDistanceSegmentBox.h"
#include "GuDistanceSegmentSegment.h"
#include "GuDistancePointBox.h"
#include "GuDistancePointTriangle.h"
#include "GuIntersectionRayPlane.h"
#include "GuIntersectionRayCapsule.h"
#include "GuIntersectionRaySphere.h"
#include "GuIntersectionRayBox.h"
#include "GuIntersectionBoxBox.h"
#include "GuDistancePointSegment.h"
#include "GuIntersectionEdgeEdge.h"
#include "GuIntersectionTriangleBox.h"
#include "GuOverlapTests.h"

#include "GJKSweep.h"

#include "GuCapsule.h"
#include "GuPlane.h"

#include "PsAlloca.h"
#include "./Ice/IceUtils.h"
#include "IceSupport.h"
#include "GuBoxConversion.h"
#include "GuGeomUtilsInternal.h"
#include "GuConvexUtilsInternal.h"
#include "GuTriangleMesh.h"

//#define LOCAL_EPSILON 0.000001f
#define LOCAL_EPSILON 0.00001f	// PT: this value makes the 'basicAngleTest' pass. Fails because of a ray almost parallel to a triangle
// SD: USE_NEW_SWEEP_TEST is currently buggy: See DE120 
#define USE_NEW_SWEEP_TEST
#define NEW_SWEEP_CAPSULE_MESH	// PT: test to extrude the mesh on the fly

using namespace physx;

static const PxReal gGJKEpsilon = 0.005f;				//TODO: try simply using gEpsilon here.  Make sure that doesn't break behavior.
static const PxReal gEpsilon = .01f;

static const PxReal gFatBoxEdgeCoeff = 0.01f;
static const PxReal gFatTriangleCoeff = 0.02f;

static const float gNearPlaneNormal[] = 
{
	1.0f, 0.0f, 0.0f,
	0.0f, 1.0f, 0.0f,
	0.0f, 0.0f, 1.0f,
	-1.0f, 0.0f, 0.0f,
	0.0f, -1.0f, 0.0f,
	0.0f, 0.0f, -1.0f
};

class SE_Vector2
{
public:
	//! Constructor
	PX_FORCE_INLINE				SE_Vector2()										{}
	PX_FORCE_INLINE				SE_Vector2(PxReal x_, PxReal y_) : x(x_), y(y_)		{}
	PX_FORCE_INLINE				SE_Vector2(const SE_Vector2& v) : x(v.x), y(v.y)	{}
	//! Destructor
	PX_FORCE_INLINE				~SE_Vector2()										{}

	PX_FORCE_INLINE	SE_Vector2	operator + (const SE_Vector2& inOther)		const	{ return SE_Vector2(x + inOther.x, y + inOther.y);		}
	PX_FORCE_INLINE	SE_Vector2	operator - (const SE_Vector2& inOther)		const	{ return SE_Vector2(x - inOther.x, y - inOther.y);		}
	PX_FORCE_INLINE	SE_Vector2	operator * (PxReal inConstant)				const	{ return SE_Vector2(x * inConstant, y * inConstant);	}
	PX_FORCE_INLINE	PxReal		Dot(const SE_Vector2& inOther)				const	{ return x * inOther.x + y * inOther.y;					}
	PX_FORCE_INLINE	PxReal		GetLengthSquared()							const	{ return x*x + y*y;										}

	PxReal		x, y;
};

class SE_Vector3
{
public:
	//! Constructor
	PX_FORCE_INLINE				SE_Vector3(PxReal inX = 0.0f, PxReal inY = 0.0f, PxReal inZ = 0.0f) : x(inX), y(inY), z(inZ)	{}
	PX_FORCE_INLINE				SE_Vector3(const SE_Vector2& inOther) : x(inOther.x), y(inOther.y), z(0.0f)					{}

	PX_FORCE_INLINE	SE_Vector3	operator + (const SE_Vector3 &inOther)	const	{ return SE_Vector3(x + inOther.x, y + inOther.y, z + inOther.z);		}
	PX_FORCE_INLINE	SE_Vector3	operator - (const SE_Vector3 &inOther)	const	{ return SE_Vector3(x - inOther.x, y - inOther.y, z - inOther.z);		}

	PX_FORCE_INLINE	SE_Vector3	operator * (PxReal inConstant)			const	{ return SE_Vector3(x * inConstant, y * inConstant, z * inConstant);	}

	PX_FORCE_INLINE	SE_Vector3	operator / (PxReal inConstant)			const	{ return SE_Vector3(x / inConstant, y / inConstant, z / inConstant);	}

	PX_FORCE_INLINE	PxReal		Dot(const SE_Vector3 &inOther)			const	{ return x * inOther.x + y * inOther.y + z * inOther.z;					}

	PX_FORCE_INLINE	PxReal		GetLengthSquared()						const	{ return x*x + y*y + z*z;												}

	PX_FORCE_INLINE	PxReal		GetLength()								const	{ return PxSqrt(GetLengthSquared());								}

	PX_FORCE_INLINE	SE_Vector3	Cross(const SE_Vector3 &inOther)		const
	{
		return SE_Vector3(y * inOther.z - z * inOther.y,
			z * inOther.x - x * inOther.z,
			x * inOther.y - y * inOther.x);
	}

	PX_FORCE_INLINE	SE_Vector3	GetPerpendicular() const
	{
		if (PxAbs(x) > PxAbs(y))
		{
			const PxReal len = PxRecipSqrt(x*x + z*z);		
			return SE_Vector3(z * len, 0.0f, -x * len);
		}
		else
		{
			const PxReal len = PxRecipSqrt(y*y + z*z);		
			return SE_Vector3(0.0f, z * len, -y * len);
		}
	}

	// Data
	PxReal			x, y, z;
};


// Simple plane
class SE_Plane
{
public:
	// Constructor
	PX_FORCE_INLINE SE_Plane() :
	  mConstant(0.0f)
	  {
	  }

	  // Get signed distance to inPoint
	  PX_FORCE_INLINE	PxReal GetSignedDistance(const SE_Vector3 &inPoint) const
	  {
		  return inPoint.Dot(mNormal) + mConstant;
	  }

	  // Get two vectors that together with mNormal form a basis for the plane
	  PX_FORCE_INLINE	void GetBasisVectors(SE_Vector3 &outU, SE_Vector3 &outV) const
	  { 
		  outU = mNormal.GetPerpendicular();
		  outV = mNormal.Cross(outU); 
	  } 

	  // Convert a point from plane space to world space (2D -> 3D)
	  PX_FORCE_INLINE	SE_Vector3 ConvertPlaneToWorld(const SE_Vector3 &inU, const SE_Vector3 &inV, const SE_Vector2 &inPoint) const
	  {
		  return SE_Vector3(inU * inPoint.x + inV * inPoint.y - mNormal * mConstant);
	  }

	  // Plane equation: mNormal.Dot(point) + mConstant == 0
	  SE_Vector3	mNormal;
	  PxReal		mConstant;
};


// Convert a point from world space to plane space (3D -> 2D)
static PX_FORCE_INLINE SE_Vector2 ConvertWorldToPlane(const SE_Vector3& inU, const SE_Vector3& inV, const SE_Vector3& inPoint)
{
	return SE_Vector2(inU.Dot(inPoint), inV.Dot(inPoint));
}

// Adapted from Gamasutra (Gomez article)
// Return true if r1 and r2 are real
static PX_FORCE_INLINE bool QuadraticFormula(const PxReal a, const PxReal b, const PxReal c, PxReal& r1, PxReal& r2)
{
	const PxReal q = b*b - 4*a*c; 
	if(q>=0.0f)
	{
		PX_ASSERT(a!=0.0f);
		const PxReal sq = PxSqrt(q);
		const PxReal d = 1.0f / (2.0f*a);
		r1 = (-b + sq) * d;
		r2 = (-b - sq) * d;
		return true;//real roots
	}
	else
	{
		return false;//complex roots
	}
}

static bool SphereSphereSweep(	const PxReal ra, //radius of sphere A
								const PxVec3& A0, //previous position of sphere A
								const PxVec3& A1, //current position of sphere A
								const PxReal rb, //radius of sphere B
								const PxVec3& B0, //previous position of sphere B
								const PxVec3& B1, //current position of sphere B
								PxReal& u0, //normalized time of first collision
								PxReal& u1 //normalized time of second collision
								)
{
	const PxVec3 va = A1 - A0;
	const PxVec3 vb = B1 - B0;
	const PxVec3 AB = B0 - A0;
	const PxVec3 vab = vb - va;	// relative velocity (in normalized time)
	const PxReal rab = ra + rb;

	const PxReal a = vab.dot(vab);		//u*u coefficient
	const PxReal b = 2.0f*(vab.dot(AB));	//u coefficient

	const PxReal c = (AB.dot(AB)) - rab*rab;	//constant term

	//check if they're currently overlapping
//	if((AB.dot(AB))<= rab*rab)
	if(c<=0.0f || a==0.0f)
	{
		u0 = 0.0f;
		u1 = 0.0f;
		return true;
	}

	//check if they hit each other during the frame
	if(QuadraticFormula(a, b, c, u0, u1))
	{
		if(u0>u1)
			TSwap(u0, u1);

		// u0<u1
		//		if(u0<0.0f || u1>1.0f)	return false;
		if(u1<0.0f || u0>1.0f)	return false;

		return true;
	}
	return false;
}

static bool SweptSphereIntersect(const PxVec3& center0, PxReal radius0, const PxVec3& center1, PxReal radius1, const PxVec3& motion, PxReal& d, PxVec3& nrm)
{
	const PxVec3 movedCenter = center1 + motion;

	PxReal tmp;
	if(!SphereSphereSweep(radius0, center0, center0, radius1, center1, movedCenter, d, tmp))
		return false;

	// Compute normal
	// PT: if spheres initially overlap, the convention is that returned normal = -sweep direction
	if(d==0.0f)
		nrm = -motion;
	else
		nrm = (center1 + d * motion) - center0;
	nrm.normalize();
	return true;
}

static bool SweepSphereCapsule(const Gu::Sphere& sphere, const Gu::Capsule& lss, const PxVec3& dir, PxReal length, PxReal& d, PxVec3& ip, PxVec3& nrm, PxSceneQueryFlags hintFlags)
{
	if(lss.p0 == lss.p1)
	{
		// Sphere vs. sphere
		if(SweptSphereIntersect(sphere.center, sphere.radius, lss.p0, lss.radius, -dir*length, d, nrm))
		{
			d*=length;
//				if(hintFlags & PxSceneQueryFlag::eIMPACT)	// PT: TODO
				ip = sphere.center + nrm * sphere.radius;
			return true;
		}
		return false;
	}

	const PxReal radiusSum = lss.radius + sphere.radius;

	if(hintFlags & PxSceneQueryFlag::eINITIAL_OVERLAP)
	{
		// PT: test if shapes initially overlap
		if(Gu::distancePointSegmentSquared(lss.p0, lss.p1, sphere.center)<radiusSum*radiusSum)
		{
			d	= 0.0f;
			nrm	= -dir;
			ip	= sphere.center;	// PT: this is arbitrary
			return true;
		}
	}

	// Create inflated capsule
	Gu::Capsule Inflated;
	Inflated.p0		= lss.p0;
	Inflated.p1		= lss.p1;
	Inflated.radius	= radiusSum;

	// Raycast against it
	PxReal s[2];
	PxU32 n = Gu::intersectRayCapsule(sphere.center, dir, Inflated, s);
	if(n)
	{
		PxReal t;
		if (n == 1)
			t = s[0];
		else
			t = (s[0] < s[1]) ? s[0]:s[1];

		if(t>=0.0f && t<length)
		{
			d = t;

// PT: TODO:
//				const Ps::IntBool needsImpactPoint = hintFlags & PxSceneQueryFlag::eIMPACT;
//				if(needsImpactPoint || hintFlags & PxSceneQueryFlag::eNORMAL)
			{
				// Move capsule against sphere
				const PxVec3 tdir = t*dir;
				Inflated.p0 -= tdir;
				Inflated.p1 -= tdir;

				// Compute closest point between moved capsule & sphere
				Gu::distancePointSegmentSquared(Inflated, sphere.center, &t);
				Inflated.computePoint(ip, t);

				// Normal
				nrm = (ip - sphere.center);
				nrm.normalize();

//					if(needsImpactPoint)	// PT: TODO
					ip -= nrm * lss.radius;
			}
			return true;
		}
	}
	return false;
}

static void EdgeEdgeDist(PxVec3& x, PxVec3& y,	// closest points
				 const PxVec3& p, const PxVec3& a,		// seg 1 origin, vector
				 const PxVec3& q, const PxVec3& b)		// seg 2 origin, vector
{
	const PxVec3 T = q - p;
	const PxReal ADotA = a.dot(a);
	const PxReal BDotB = b.dot(b);
	const PxReal ADotB = a.dot(b);
	const PxReal ADotT = a.dot(T);
	const PxReal BDotT = b.dot(T);

	// t parameterizes ray (p, a)
	// u parameterizes ray (q, b)

	// Compute t for the closest point on ray (p, a) to ray (q, b)
	const PxReal Denom = ADotA*BDotB - ADotB*ADotB;

	PxReal t;
	if(Denom!=0.0f)	
	{
		t = (ADotT*BDotB - BDotT*ADotB) / Denom;

		// Clamp result so t is on the segment (p, a)
				if(t<0.0f)	t = 0.0f;
		else	if(t>1.0f)	t = 1.0f;
	}
	else
	{
		t = 0.0f;
	}

	// find u for point on ray (q, b) closest to point at t
	PxReal u;
	if(BDotB!=0.0f)
	{
		u = (t*ADotB - BDotT) / BDotB;

		// if u is on segment (q, b), t and u correspond to closest points, otherwise, clamp u, recompute and clamp t
		if(u<0.0f)
		{
			u = 0.0f;
			if(ADotA!=0.0f)
			{
				t = ADotT / ADotA;

						if(t<0.0f)	t = 0.0f;
				else	if(t>1.0f)	t = 1.0f;
			}
			else
			{
				t = 0.0f;
			}
		}
		else if(u > 1.0f)
		{
			u = 1.0f;
			if(ADotA!=0.0f)
			{
				t = (ADotB + ADotT) / ADotA;

						if(t<0.0f)	t = 0.0f;
				else	if(t>1.0f)	t = 1.0f;
			}
			else
			{
				t = 0.0f;
			}
		}
	}
	else
	{
		u = 0.0f;

		if(ADotA!=0.0f)
		{
			t = ADotT / ADotA;

					if(t<0.0f)	t = 0.0f;
			else	if(t>1.0f)	t = 1.0f;
		}
		else
		{
			t = 0.0f;
		}
	}

	x = p + a * t;
	y = q + b * u;
}

static bool RayQuad(const PxVec3& orig, const PxVec3& dir, const PxVec3& vert0, const PxVec3& vert1, const PxVec3& vert2, PxReal& t, PxReal& u, PxReal& v, bool cull)
{
	// Find vectors for two edges sharing vert0
	const PxVec3 edge1 = vert1 - vert0;
	const PxVec3 edge2 = vert2 - vert0;

	// Begin calculating determinant - also used to calculate U parameter
	const PxVec3 pvec = dir.cross(edge2);

	// If determinant is near zero, ray lies in plane of triangle
	const PxReal det = edge1.dot(pvec);

	if(cull)
	{
		if(det<LOCAL_EPSILON)						return false;

		// Calculate distance from vert0 to ray origin
		const PxVec3 tvec = orig - vert0;

		// Calculate U parameter and test bounds
		u = tvec.dot(pvec);
		if(u<0.0f || u>det)							return false;

		// Prepare to test V parameter
		const PxVec3 qvec = tvec.cross(edge1);

		// Calculate V parameter and test bounds
		v = dir.dot(qvec);
		if(v<0.0f || v>det)							return false;

		// Calculate t, scale parameters, ray intersects triangle
		t = edge2.dot(qvec);
		const PxReal oneOverDet = 1.0f / det;
		t *= oneOverDet;
		u *= oneOverDet;
		v *= oneOverDet;
	}
	else
	{
		// the non-culling branch
		if(det>-LOCAL_EPSILON && det<LOCAL_EPSILON)	return false;
		const PxReal oneOverDet = 1.0f / det;

		// Calculate distance from vert0 to ray origin
		const PxVec3 tvec = orig - vert0;

		// Calculate U parameter and test bounds
		u = (tvec.dot(pvec)) * oneOverDet;
		if(u<0.0f || u>1.0f)						return false;

		// prepare to test V parameter
		const PxVec3 qvec = tvec.cross(edge1);

		// Calculate V parameter and test bounds
		v = (dir.dot(qvec)) * oneOverDet;
		if(v<0.0f || v>1.0f)						return false;

		// Calculate t, ray intersects triangle
		t = (edge2.dot(qvec)) * oneOverDet;
	}
	return true;
}

static bool SweepCapsuleCapsule(const Gu::Capsule& lss0, const Gu::Capsule& lss1, const PxVec3& dir, PxReal length, PxReal& min_dist, PxVec3& ip, PxVec3& normal, PxU32 inHintFlags, PxU32& outHintFlags)
{
	const PxReal radiusSum = lss0.radius + lss1.radius;
	const PxVec3 center = lss1.computeCenter();

	if(inHintFlags & PxSceneQueryFlag::eINITIAL_OVERLAP)
	{
		// PT: test if shapes initially overlap

		// PT: using the same codepath for spheres and capsules was a terrible idea. The segment-segment distance
		// function doesn't work for degenerate capsules so we need to test all combinations here anyway. Sigh.
		bool initialOverlapStatus;
		if(lss0.p0==lss0.p1)
			initialOverlapStatus = Gu::distancePointSegmentSquared(lss1, lss0.p0)<radiusSum*radiusSum;
		else if(lss1.p0==lss1.p1)
			initialOverlapStatus = Gu::distancePointSegmentSquared(lss0, lss1.p0)<radiusSum*radiusSum;
		else
			initialOverlapStatus = Gu::distanceSegmentSegmentSquared(lss0, lss1)<radiusSum*radiusSum;
			
		if(initialOverlapStatus)
		{
			min_dist	= 0.0f;
			normal		= -dir;
			ip			= center;	// PT: this is arbitrary
			outHintFlags = PxSceneQueryFlag::eDISTANCE | PxSceneQueryFlag::eIMPACT | PxSceneQueryFlag::eNORMAL;
			return true;
		}
	}

	// 1. Extrude lss0 by lss1's length
	// 2. Inflate extruded shape by lss1's radius
	// 3. Raycast against resulting shape

	// Extrusion dir = capsule segment
	const PxVec3 D = (lss1.p1 - lss1.p0)*0.5f;

	const PxVec3 p0 = lss0.p0 - D;
	const PxVec3 p1 = lss0.p1 - D;
	const PxVec3 p0b = lss0.p0 + D;
	const PxVec3 p1b = lss0.p1 + D;

	Gu::Triangle T(p0b, p1b, p1);
	PxVec3 Normal;
	T.normal(Normal);
	Normal *= radiusSum;

	//	min_dist = length;
	PxReal MinDist = length;
	bool Status = false;

	PxVec3 pa,pb,pc;
	if((Normal.dot(dir)) >= 0)  // Same direction
	{
		pc = p0 - Normal;
		pa = p1 - Normal;
		pb = p1b - Normal;
	}
	else
	{
		pb = p0 + Normal;
		pa = p1 + Normal;
		pc = p1b + Normal;
	}
	PxReal t, u, v;
	if(RayQuad(center, dir, pa, pb, pc, t, u, v, true) && t>=0.0f && t<MinDist)
	{
		MinDist = t;
		Status = true;
	}

	// PT: optimization: if we hit one of the quad we can't possibly get a better hit, so let's skip all
	// the remaining tests!
	//	if(min_dist==length)
	if(!Status)
	{
		Gu::Capsule Caps[4];
		Caps[0] = Gu::Capsule(Gu::Segment(p0, p1), radiusSum);
		Caps[1] = Gu::Capsule(Gu::Segment(p1, p1b), radiusSum);
		Caps[2] = Gu::Capsule(Gu::Segment(p1b, p0b), radiusSum);
		Caps[3] = Gu::Capsule(Gu::Segment(p0, p0b), radiusSum);

		// ### a lot of ray-sphere tests could be factored out of the ray-capsule tests...
		for(PxU32 i=0;i<4;i++)
		{
			PxReal s[2];
			PxU32 n = Gu::intersectRayCapsule(center, dir, Caps[i], s);
			if(n)
			{
				PxReal t;
				if (n == 1)
					t = s[0];
				else
					t = (s[0] < s[1]) ? s[0]:s[1];

				if(t>=0.0f && t<MinDist)
				{
					MinDist = t;
					Status = true;
				}
			}
		}
	}

	if(Status)
	{
		outHintFlags = PxSceneQueryFlag::eDISTANCE;
		if(inHintFlags & (PxU32)(PxSceneQueryFlag::eIMPACT|PxSceneQueryFlag::eNORMAL))
		{
			if(1)
			{
				const PxVec3 p00 = lss0.p0 - MinDist * dir;
				const PxVec3 p01 = lss0.p1 - MinDist * dir;
				const PxVec3 p10 = lss1.p0 - MinDist * dir;
				const PxVec3 p11 = lss1.p1 - MinDist * dir;

				PxVec3 x, y;
				EdgeEdgeDist(x, y, p00, p01-p00, lss1.p0, lss1.p1-lss1.p0);

				if(inHintFlags & PxSceneQueryFlag::eNORMAL)
				{
					normal = (x - y);
					normal.normalize();
					outHintFlags |= PxSceneQueryFlag::eNORMAL;
				}

				if(inHintFlags & PxSceneQueryFlag::eIMPACT)
				{
					ip = (lss1.radius*x + lss0.radius*y)/(lss0.radius+lss1.radius);
					outHintFlags |= PxSceneQueryFlag::eIMPACT;
				}
			}
			else
			{
				// Old CCT code
				PxVec3 x, y;
				EdgeEdgeDist(x, y, lss0.p0, lss0.p1-lss0.p0, lss1.p0, lss1.p1-lss1.p0);

				if(inHintFlags & PxSceneQueryFlag::eNORMAL)
				{
					normal = (x - y);
					normal.normalize();
					outHintFlags |= PxSceneQueryFlag::eNORMAL;
				}

				if(inHintFlags & PxSceneQueryFlag::eIMPACT)
				{
					ip = (x+y)*0.5f;
					outHintFlags |= PxSceneQueryFlag::eIMPACT;
				}
			}
		}
		min_dist = MinDist;
	}
	return Status;
}

#define OUTPUT_TRI(t, p0, p1, p2){	\
t->verts[0] = p0;					\
t->verts[1] = p1;					\
t->verts[2] = p2;					\
t++;}

#define OUTPUT_TRI2(t, p0, p1, p2, d){		\
t->verts[0] = p0;							\
t->verts[1] = p1;							\
t->verts[2] = p2;							\
t->denormalizedNormal(DenormalizedNormal);	\
if((DenormalizedNormal.dot(d))>0.0f) {		\
PxVec3 Tmp = t->verts[1];					\
t->verts[1] = t->verts[2];					\
t->verts[2] = Tmp;							\
}											\
t++; *ids++ = i; }

static PxU32 ExtrudeMesh(	PxU32 nb_tris, const Gu::Triangle* triangles, const PxU32* edge_flags,
					const PxVec3& extrusion_dir, Gu::Triangle* tris, PxU32* ids, const PxVec3& dir, 
					const Gu::Box* sweptBounds)
{
	const PxU32* Base = ids;

	//PxU32 CurrentFlags =	Gu::TriangleCollisionFlag::eACTIVE_EDGE01 | 
	//						Gu::TriangleCollisionFlag::eACTIVE_EDGE12 |
	//						Gu::TriangleCollisionFlag::eACTIVE_EDGE20;

	for(PxU32 i=0; i<nb_tris; i++)
	{
		const Gu::Triangle& CurrentTriangle = triangles[i];
		//if (edge_flags)
		//	CurrentFlags = edge_flags[i];
		
		// Create triangle normal
		PxVec3 DenormalizedNormal;
		CurrentTriangle.denormalizedNormal(DenormalizedNormal);

		// Backface culling
		// bool DoCulling = (CurrentFlags & Gu::TriangleCollisionFlag::eDOUBLE_SIDED)==0;
		// bool Culled = (DoCulling && (DenormalizedNormal|dir) > 0.0f);
		const bool Culled = (DenormalizedNormal.dot(dir)) > 0.0f;
		if(Culled)	continue;

		if (sweptBounds)
		{
			PxVec3 tmp[3];
			tmp[0] = sweptBounds->rotateInv(CurrentTriangle.verts[0] - sweptBounds->center);
			tmp[1] = sweptBounds->rotateInv(CurrentTriangle.verts[1] - sweptBounds->center);
			tmp[2] = sweptBounds->rotateInv(CurrentTriangle.verts[2] - sweptBounds->center);
			const PxVec3 center(0.0f);
			if(!Gu::intersectTriangleBox(center, sweptBounds->extents, tmp[0], tmp[1], tmp[2]))
				continue;
		}

		PxVec3 p0 = CurrentTriangle.verts[0];
		PxVec3 p1 = CurrentTriangle.verts[1];
		PxVec3 p2 = CurrentTriangle.verts[2];

		PxVec3 p0b = p0 + extrusion_dir;
		PxVec3 p1b = p1 + extrusion_dir;
		PxVec3 p2b = p2 + extrusion_dir;

		p0 -= extrusion_dir;
		p1 -= extrusion_dir;
		p2 -= extrusion_dir;

		if(DenormalizedNormal.dot(extrusion_dir) >= 0.0f)	OUTPUT_TRI(tris, p0b, p1b, p2b)
		else												OUTPUT_TRI(tris, p0, p1, p2)
		*ids++ = i;

		// ### it's probably useless to extrude all the shared edges !!!!!
		//if(CurrentFlags & Gu::TriangleCollisionFlag::eACTIVE_EDGE12)
		{
			OUTPUT_TRI2(tris, p1, p1b, p2b, dir)
			OUTPUT_TRI2(tris, p1, p2b, p2, dir)
		}
		//if(CurrentFlags & Gu::TriangleCollisionFlag::eACTIVE_EDGE20)
		{
			OUTPUT_TRI2(tris, p0, p2, p2b, dir)
			OUTPUT_TRI2(tris, p0, p2b, p0b, dir)
		}
		//if(CurrentFlags & Gu::TriangleCollisionFlag::eACTIVE_EDGE01)
		{
			OUTPUT_TRI2(tris, p0b, p1b, p1, dir)
			OUTPUT_TRI2(tris, p0b, p1, p0, dir)
		}
	}
	return PxU32(ids-Base);
}

static PxU32 ExtrudeBox(const PxBounds3& local_box, const PxTransform* world, const PxVec3& extrusion_dir, Gu::Triangle* tris, const PxVec3& dir)
{
	// Handle the box as a mesh

	Gu::Triangle BoxTris[12];

	PxVec3 p[8];
	Gu::computeBoxPoints(local_box, p);

	const PxU8* PX_RESTRICT Indices = Gu::getBoxTriangles();

	for(PxU32 i=0; i<12; i++)
	{
		const PxU8 VRef0 = Indices[i*3+0];
		const PxU8 VRef1 = Indices[i*3+1];
		const PxU8 VRef2 = Indices[i*3+2];

		PxVec3 p0 = p[VRef0];
		PxVec3 p1 = p[VRef1];
		PxVec3 p2 = p[VRef2];
		if(world)
		{
			p0 = world->transform(p0);
			p1 = world->transform(p1);
			p2 = world->transform(p2);
		}

		BoxTris[i].verts[0] = p0;
		BoxTris[i].verts[1] = p1;
		BoxTris[i].verts[2] = p2;
	}
	PxU32 FakeIDs[12*7];
	return ExtrudeMesh(12, BoxTris, NULL, extrusion_dir, tris, FakeIDs, dir, NULL);
}

//
//                     point
//                      o
//                   __/|
//                __/ / |
//             __/   /  |(B)
//          __/  (A)/   |
//       __/       /    |                dir
//  p0 o/---------o---------------o--    -->
//                t (t<=fT)       t (t>fT)
//                return (A)^2    return (B)^2
//
//     |<-------------->|
//             fT
//
//
static PX_FORCE_INLINE PxReal SquareDistance(const PxVec3& p0, const PxVec3& dir, PxReal t, const PxVec3& point)
{
	PxVec3 Diff = point - p0;
/*	const PxReal fT = (Diff.dot(dir));
	if(fT>0.0f)
	{
		if(fT>=t)
			Diff -= dir*t;	// Take travel distance of point p0 into account (shortens the distance)
		else
			Diff -= fT*dir;
	}*/

	PxReal fT = (Diff.dot(dir));
	fT = physx::intrinsics::selectMax(fT, 0.0f);
	fT = physx::intrinsics::selectMin(fT, t);
	Diff -= fT*dir;

	return Diff.magnitudeSquared();
}

static PX_FORCE_INLINE bool CoarseCulling(const PxVec3& center, const PxVec3& dir, PxReal t, PxReal radius, const Gu::Triangle& tri)
{
	// ### could be precomputed
	const PxVec3 TriCenter = (tri.verts[0] + tri.verts[1] + tri.verts[2]) * (1.0f/3.0f);

	// PT: distance between the triangle center and the swept path (an LSS)
	// Same as: Gu::distancePointSegmentSquared(center, center+dir*t, TriCenter);
	PxReal d = PxSqrt(SquareDistance(center, dir, t, TriCenter)) - radius - 0.0001f;

	if (d < 0.0f)	// The triangle center lies inside the swept sphere
		return true;

	d*=d;

	// ### distances could be precomputed
/*	if(d <= (TriCenter-tri.verts[0]).magnitudeSquared())
	return true;
	if(d <= (TriCenter-tri.verts[1]).magnitudeSquared())
		return true;
	if(d <= (TriCenter-tri.verts[2]).magnitudeSquared())
		return true;
	return false;*/
	const PxReal d0 = (TriCenter-tri.verts[0]).magnitudeSquared();
	const PxReal d1 = (TriCenter-tri.verts[1]).magnitudeSquared();
	const PxReal d2 = (TriCenter-tri.verts[2]).magnitudeSquared();
	PxReal triRadius = physx::intrinsics::selectMax(d0, d1);
	triRadius = physx::intrinsics::selectMax(triRadius, d2);
	if(d <= triRadius)
		return true;
	return false;
}

// PT: returning a float is the fastest on Xbox!
#ifdef _XBOX
static PX_FORCE_INLINE float CullTriangle(const Gu::Triangle& CurrentTri, const PxVec3& dir, PxReal radius, PxReal t, const PxReal dpc0)
#else
static PX_FORCE_INLINE bool CullTriangle(const Gu::Triangle& CurrentTri, const PxVec3& dir, PxReal radius, PxReal t, const PxReal dpc0)
#endif
{
	const PxReal dp0 = CurrentTri.verts[0].dot(dir);
	const PxReal dp1 = CurrentTri.verts[1].dot(dir);
	const PxReal dp2 = CurrentTri.verts[2].dot(dir);

#ifdef _XBOX
	// PT: we have 3 ways to write that on Xbox:
	// - with the original code: suffers from a lot of FCMPs
	// - with the cndt stuff below, cast to an int: it's faster, but suffers from a very bad LHS from the float-to-int
	// - with the cndt stuff not cast to an int: we get only one FCMP instead of many, the LHS is gone, that's the fastest solution. Even though it looks awkward.
	// AP: new correct implementation
	PxReal dp = dp0;
	dp = physx::intrinsics::selectMin(dp, dp1);
	dp = physx::intrinsics::selectMin(dp, dp2);

	using physx::intrinsics::fsel;

	//if(dp>dpc0 + t + radius) return false;
	const float cndt0 = fsel(dp - (dpc0 + t + radius), 0.0f, 1.0f);

	//if(dp0<dpc0 && dp1<dpc0 && dp2<dpc0) return false; <=>
	//if(dp0>=dpc0 || dp1>=dpc0 || dp2>=dpc0) return true;
	const float cndt1 = fsel(dp0-dpc0, 1.0f, 0.0f) + fsel(dp1-dpc0, 1.0f, 0.0f) + fsel(dp2-dpc0, 1.0f, 0.0f);

	return cndt0*cndt1;
	//PxReal resx = cndt0*cndt1;
#else
	PxReal dp = dp0;
	dp = physx::intrinsics::selectMin(dp, dp1);
	dp = physx::intrinsics::selectMin(dp, dp2);

	if(dp>dpc0 + t + radius)
	{
		//PX_ASSERT(resx == 0.0f);
		return false;
	}

	// ExperimentalCulling
	if(dp0<dpc0 && dp1<dpc0 && dp2<dpc0)
	{
		//PX_ASSERT(resx == 0.0f);
		return false;
	}

	//PX_ASSERT(resx != 0.0f);
	return true;
#endif
}

static PX_FORCE_INLINE void ComputeTriData(const Gu::Triangle& tri, PxVec3& normal, PxReal& magnitude, PxReal& area)
{
	tri.denormalizedNormal(normal);

	magnitude = normal.magnitude();

	area = magnitude * 0.5f;
}

//#ifdef USE_NEW_SWEEP_TEST
// PT: special version computing (u,v) even when the ray misses the tri
static PX_FORCE_INLINE PxU32 RayTriSpecial(const PxVec3& orig, const PxVec3& dir, const PxVec3& vert0, const PxVec3& vert1, const PxVec3& vert2, PxReal& t, PxReal& u, PxReal& v)
{
	// Find vectors for two edges sharing vert0
	const PxVec3 edge1 = vert1 - vert0;
	const PxVec3 edge2 = vert2 - vert0;

	// Begin calculating determinant - also used to calculate U parameter
	const PxVec3 pvec = dir.cross(edge2);

	// If determinant is near zero, ray lies in plane of triangle
	const PxReal det = edge1.dot(pvec);

	// the non-culling branch
	if(det>-LOCAL_EPSILON && det<LOCAL_EPSILON)	return 0;
	const PxReal OneOverDet = 1.0f / det;

	// Calculate distance from vert0 to ray origin
	const PxVec3 tvec = orig - vert0;

	// Calculate U parameter
	u = (tvec.dot(pvec)) * OneOverDet;

	// prepare to test V parameter
	const PxVec3 qvec = tvec.cross(edge1);

	// Calculate V parameter
	v = (dir.dot(qvec)) * OneOverDet;

	if(u<0.0f || u>1.0f)	return 1;
	if(v<0.0f || u+v>1.0f)	return 1;

	// Calculate t, ray intersects triangle
	t = (edge2.dot(qvec)) * OneOverDet;

	return 2;
}

// Returns true if sphere can be tested against triangle vertex, false if edge test should be performed
//
// Uses a conservative approach to work for "sliver triangles" (long & thin) as well.
static PX_FORCE_INLINE bool EdgeOrVertexTest(const PxVec3& planeIntersectPoint, const Gu::Triangle& tri, PxU32 vertIntersectCandidate, PxU32 vert0, PxU32 vert1, PxU32& secondEdgeVert)
{
	const PxVec3 edge0 = tri.verts[vertIntersectCandidate] - tri.verts[vert0];
	const PxReal edge0LengthSqr = edge0.dot(edge0);

	PxVec3 diff = planeIntersectPoint - tri.verts[vert0];

	if (edge0.dot(diff) < edge0LengthSqr)  // If the squared edge length is used for comparison, the edge vector does not need to be normalized
	{
		secondEdgeVert = vert0;
		return false;
	}

	const PxVec3 edge1 = tri.verts[vertIntersectCandidate] - tri.verts[vert1];
	const PxReal edge1LengthSqr = edge1.dot(edge1);

	diff = planeIntersectPoint - tri.verts[vert1];

	if (edge1.dot(diff) < edge1LengthSqr)
	{
		secondEdgeVert = vert1;
		return false;
	}

	return true;
}

static bool SweepTriSphere(const Gu::Triangle& tri, const PxVec3& normal, const PxVec3& center, PxReal radius, const PxVec3& dir, PxReal& min_dist)
{
	// Ok, this new version is now faster than the original code. Needs more testing though.

	#define INTERSECT_POINT (tri.verts[1]*u) + (tri.verts[2]*v) + (tri.verts[0] * (1.0f-u-v))

	PxReal u,v;
	if(1)
	{
		PxVec3 R = normal * radius;
		if(dir.dot(R) >= 0.0f)
			R = -R;

		// The first point of the sphere to hit the triangle plane is the point of the sphere nearest to
		// the triangle plane. Hence, we use center - (normal*radius) below.

		// PT: casting against the extruded triangle in direction R is the same as casting from a ray moved by -R
		PxReal t;
		//		int r = RayTriSpecial(center, dir, tri.mVerts[0]+R, tri.mVerts[1]+R, tri.mVerts[2]+R, t, u, v);
		int r = RayTriSpecial(center-R, dir, tri.verts[0], tri.verts[1], tri.verts[2], t, u, v);
		if(!r)	return false;
		if(r==2)
		{
			if(t<0.0f)	return false;
			min_dist = t;
			return true;
		}
	}

	//
	// Let's do some art!
	//
	// The triangle gets divided into the following areas (based on the barycentric coordinates (u,v)):
	//
	//               \   A0    /
	//                 \      /
	//                   \   /
	//                     \/ 0
	//            A02      *      A01
	//   u /              /   \          \ v
	//    *              /      \         *
	//                  /         \						.
	//               2 /            \ 1
	//          ------*--------------*-------
	//               /                 \				.
	//        A2    /        A12         \   A1
	//
	//
	// Based on the area where the computed triangle plane intersection point lies in, a different sweep test will be applied.
	//
	// A) A01, A02, A12  : Test sphere against the corresponding edge
	// B) A0, A1, A2     : Test sphere against the corresponding vertex
	//
	// Unfortunately, B) does not work for long, thin triangles. Hence there is some extra code which does a conservative check and
	// switches to edge tests if necessary.
	//

	bool TestSphere;
	PxU32 e0,e1;
	if(u<0.0f)
	{
		if(v<0.0f)
		{
			// 0 or 0-1 or 0-2
			e0 = 0;
			PxVec3 intersectPoint = INTERSECT_POINT;
			TestSphere = EdgeOrVertexTest(intersectPoint, tri, 0, 1, 2, e1);
		}
		else if(u+v>1.0f)
		{
			// 2 or 2-0 or 2-1
			e0 = 2;
			PxVec3 intersectPoint = INTERSECT_POINT;
			TestSphere = EdgeOrVertexTest(intersectPoint, tri, 2, 0, 1, e1);
		}
		else
		{
			// 0-2
			TestSphere = false;
			e0 = 0;
			e1 = 2;
		}
	}
	else
	{
		if(v<0.0f)
		{
			if(u+v>1.0f)
			{
				// 1 or 1-0 or 1-2
				e0 = 1;
				PxVec3 intersectPoint = INTERSECT_POINT;
				TestSphere = EdgeOrVertexTest(intersectPoint, tri, 1, 0, 2, e1);
			}
			else
			{
				// 0-1
				TestSphere = false;
				e0 = 0;
				e1 = 1;
			}
		}
		else
		{
			PX_ASSERT(u+v>=1.0f);	// Else hit triangle
			// 1-2
			TestSphere = false;
			e0 = 1;
			e1 = 2;
		}
	}

	if(TestSphere)
	{
		PxReal t;
//			if(Gu::intersectRaySphere(center, dir, min_dist*2.0f, tri.verts[e0], radius, t))
		if(Gu::intersectRaySphere(center, dir, PX_MAX_F32, tri.verts[e0], radius, t))
		{
			min_dist = t;
			return true;
		}
	}
	else
	{
		const Gu::Capsule capsule(Gu::Segment(tri.verts[e0], tri.verts[e1]), radius);

		PxReal s[2];
		PxU32 n = Gu::intersectRayCapsule(center, dir, capsule, s);
		if(n)
		{
			PxReal t;
			if (n == 1)	t = s[0];
			else t = (s[0] < s[1]) ? s[0]:s[1];

			if(t>=0.0f/* && t<MinDist*/)
			{
				min_dist = t;
				return true;
			}
		}
	}
	return false;
}
//#endif

// Test intersection between a plane inPlane and a swept sphere with radius inRadius moving from inBegin to inBegin + inDelta
// If there is an intersection the function returns true and the intersection range is from 
// inBegin + outT1 * inDelta to inBegin + outT2 * inDelta
// PT: this function is only used once so we'd better inline it
static PX_FORCE_INLINE bool PlaneSweptSphereIntersect(const SE_Plane& inPlane, const SE_Vector3& inBegin, const SE_Vector3& inDelta, PxReal inRadius, PxReal& outT1, PxReal& outT2)
{
	// If the center of the sphere moves like: center = inBegin + t * inDelta for t e [0, 1]
	// then the sphere intersects the plane if: -R <= distance plane to center <= R
	const PxReal n_dot_d = inPlane.mNormal.Dot(inDelta);
	const PxReal dist_to_b = inPlane.GetSignedDistance(inBegin);
	if (n_dot_d == 0.0f)
	{
		// The sphere is moving nearly parallel to the plane, check if the distance
		// is smaller than the radius
		if (PxAbs(dist_to_b) > inRadius)
			return false;

		// Intersection on the entire range
		outT1 = 0.0f;
		outT2 = 1.0f;
	}
	else
	{
		// Determine interval of intersection
		const PxReal over = 1.0f / n_dot_d;
		outT1 = (inRadius - dist_to_b) * over;
		outT2 = (-inRadius - dist_to_b) * over;

		// Order the results
		if (outT1 > outT2)
		{
			PxReal tmp = outT1;
			outT1 = outT2;
			outT2 = tmp;
		}

		// Early out if no hit possible
		if (outT1 > 1.0f || outT2 < 0.0f)
			return false;

		// Clamp it to the range [0, 1], the range of the swept sphere
		if (outT1 < 0.0f) outT1 = 0.0f;
		if (outT2 > 1.0f) outT2 = 1.0f;
	}
	return true;
}

// Check if a polygon contains inPoint, returns true when it does
// PT: this function is only used once so we'd better inline it
static PX_FORCE_INLINE bool PolygonContains(const SE_Vector2* inVertices, PxU32 inNumVertices, const SE_Vector2& inPoint)
{
	// Loop through edges
	for (const SE_Vector2 *v1 = inVertices, *v2 = inVertices + inNumVertices - 1; v1 < inVertices + inNumVertices; v2 = v1, ++v1)
	{
		// If the point is outside this edge, the point is outside the polygon
		SE_Vector2 v1_v2 = *v2 - *v1;
		SE_Vector2 v1_point = inPoint - *v1;
		if (v1_v2.x * v1_point.y - v1_point.x * v1_v2.y > 0.0f)
			return false;
	}
	return true;
}

// Check if circle at inCenter with radius^2 = inRadiusSq intersects with a polygon.
// Function returns true when it does and the intersection point is in outPoint
// PT: this function is only used once so we'd better inline it
static PX_FORCE_INLINE bool PolygonCircleIntersect(const SE_Vector2* inVertices, PxU32 inNumVertices, const SE_Vector2& inCenter, PxReal inRadiusSq, SE_Vector2& outPoint)
{
	// Check if the center is inside the polygon
	if(PolygonContains(inVertices, inNumVertices, inCenter))
	{
		outPoint = inCenter;
		return true;
	}

	// Loop through edges
	bool collision = false;
	for(const SE_Vector2 *v1 = inVertices, *v2 = inVertices + inNumVertices - 1; v1 < inVertices + inNumVertices; v2 = v1, ++v1)
	{
		// Get fraction where the closest point to this edge occurs
		SE_Vector2 v1_v2 = *v2 - *v1;
		SE_Vector2 v1_center = inCenter - *v1;
		const PxReal fraction = v1_center.Dot(v1_v2);
		if (fraction < 0.0f)
		{
			// Closest point is v1
			const PxReal dist_sq = v1_center.GetLengthSquared();
			if (dist_sq <= inRadiusSq)
			{
				collision = true;
				outPoint = *v1;
				inRadiusSq = dist_sq;
			}
		}
		else 
		{
			const PxReal v1_v2_len_sq = v1_v2.GetLengthSquared();
			if (fraction <= v1_v2_len_sq)
			{
				// Closest point is on line segment
				const SE_Vector2 point = *v1 + v1_v2 * (fraction / v1_v2_len_sq);
				const PxReal dist_sq = (point - inCenter).GetLengthSquared();
				if (dist_sq <= inRadiusSq)
				{
					collision = true;
					outPoint = point;
					inRadiusSq = dist_sq;
				}
			}
		}
	}
	return collision;
}

// Solve the equation inA * x^2 + inB * x + inC == 0 for the lowest x in [0, inUpperBound].
// Returns true if there is such a solution and returns the solution in outX
// SDS: Fixed implementation for xbox, since assumptions on NAN/INF compares failed.
static PX_FORCE_INLINE bool FindLowestRootInInterval(PxReal inA, PxReal inB, PxReal inC, PxReal inUpperBound, PxReal& outX)
{
	// Check if a solution exists
	const PxReal determinant = inB * inB - 4.0f * inA * inC;
	if (determinant < 0.0f)
		return false;

	PxReal x;
	if (determinant != 0.0f && inA != 0.0f)
	{
		// The standard way of doing this is by computing: x = (-b +/- Sqrt(b^2 - 4 a c)) / 2 a 
		// is not numerically stable when a is close to zero. 
		// Solve the equation according to "Numerical Recipies in C" paragraph 5.6
		const PxReal q = -0.5f * (inB + (inB < 0.0f? -1.0f : 1.0f) * PxSqrt(determinant));

		// Order the results
		x = PxMin(q / inA, inC / q);
	}
	else if (inA == 0.0f)
	{
		if(inB!=0.0f)
			x = -inC / inB;
		else 
			return false; //in case inC == 0, outX is undefined.
	}
	else if (determinant == 0.0f)
	{
		x = -inB / (2.0f * inA);
	}

	// Check if x1 is a solution
	if (x >= 0.0f && x <= inUpperBound)
	{
		outX = x;
		return true;
	}

	return false;
}

// Checks intersection between a polygon an moving circle at inBegin + t * inDelta with radius^2 = inA * t^2 + inB * t + inC, t in [0, 1]
// Returns true when it does and returns the intersection position in outPoint and the intersection fraction (value for t) in outFraction
static bool SweptCircleEdgeVertexIntersect(const SE_Vector2* inVertices, int inNumVertices, const SE_Vector2& inBegin, const SE_Vector2& inDelta, PxReal inA, PxReal inB, PxReal inC, SE_Vector2* outPoint, PxReal* outFraction)
{
	// Loop through edges
	PxReal upper_bound = 1.0f;
	bool collision = false;
	for (const SE_Vector2 *v1 = inVertices, *v2 = inVertices + inNumVertices - 1; v1 < inVertices + inNumVertices; v2 = v1, ++v1)
	{
		PxReal t;

		// Check if circle hits the vertex
		const SE_Vector2 bv1 = *v1 - inBegin;
		const PxReal a1 = inA - inDelta.GetLengthSquared();
		const PxReal b1 = inB + 2.0f * inDelta.Dot(bv1);
		const PxReal c1 = inC - bv1.GetLengthSquared();
		if(FindLowestRootInInterval(a1, b1, c1, upper_bound, t))
			//		if(FindLowestRootInInterval(a1, b1, c1, upper_bound, t) && t<=1.0f)
			//		if(FindLowestRootInInterval2(a1, b1, c1, upper_bound, t))
		{
			// We have a collision
			collision = true;
			upper_bound = t;
			if(outPoint)	*outPoint = *v1;
		}

		// Check if circle hits the edge
		const SE_Vector2 v1v2 = *v2 - *v1;
		const PxReal v1v2_dot_delta = v1v2.Dot(inDelta);
		const PxReal v1v2_dot_bv1 = v1v2.Dot(bv1);
		const PxReal v1v2_len_sq = v1v2.GetLengthSquared();
		const PxReal a2 = v1v2_len_sq * a1 + v1v2_dot_delta * v1v2_dot_delta;
		const PxReal b2 = v1v2_len_sq * b1 - 2.0f * v1v2_dot_bv1 * v1v2_dot_delta;
		const PxReal c2 = v1v2_len_sq * c1 + v1v2_dot_bv1 * v1v2_dot_bv1;
		if(FindLowestRootInInterval(a2, b2, c2, upper_bound, t))
			//		if(FindLowestRootInInterval(a2, b2, c2, upper_bound, t) && t<=1.0f)
			//		if(FindLowestRootInInterval2(a2, b2, c2, upper_bound, t))
		{
			// Check if the intersection point is on the edge
			const PxReal f = t * v1v2_dot_delta - v1v2_dot_bv1;
			if (f >= 0.0f && f <= v1v2_len_sq)
			{
				// We have a collision
				collision = true;
				upper_bound = t;
				if(outPoint)	*outPoint = *v1 + v1v2 * (f / v1v2_len_sq);
			}
		}
	}

	// Check if we had a collision
	if (!collision)
		return false;
	if(outFraction)
	{
		PX_ASSERT(upper_bound>=0.0f && upper_bound<=1.0f);
		*outFraction = upper_bound;
	}
	return true;
}

// Test between a polygon and a swept sphere with radius inRadius moving from inBegin to inBegin + inDelta
// If there is an intersection the intersection position is returned in outPoint and the center of the
// sphere is at inBegin + outFraction * inDelta when it collides
static bool PolygonSweptSphereIntersect(const SE_Plane& inPlane, const SE_Vector3& u, const SE_Vector3& v, const SE_Vector2* inVertices, PxU32 inNumVertices, const SE_Vector3& inBegin, const SE_Vector3& inDelta, PxReal inRadius, SE_Vector3* outPoint, PxReal* outFraction)
{
	// Determine the range over which the sphere intersects the plane
	PxReal t1, t2;
	if(!PlaneSweptSphereIntersect(inPlane, inBegin, inDelta, inRadius, t1, t2))
		return false;

	// The radius of the circle is defined as: radius^2 = (sphere radius)^2 - (distance plane to center)^2
	// this can be written as: radius^2 = a * t^2 + b * t + c
	const PxReal n_dot_d = inPlane.mNormal.Dot(inDelta);
	const PxReal dist_to_b = inPlane.GetSignedDistance(inBegin);
	const PxReal a = -n_dot_d * n_dot_d;
	const PxReal b = -2.0f * n_dot_d * dist_to_b;
	const PxReal c = inRadius * inRadius - dist_to_b * dist_to_b;

	// Get begin and delta in plane space
	const SE_Vector2 begin = ConvertWorldToPlane(u, v, inBegin);
	const SE_Vector2 delta = ConvertWorldToPlane(u, v, inDelta);

	// Test if sphere intersects at t1
	SE_Vector2 p(0.0f, 0.0f);
	if(PolygonCircleIntersect(inVertices, inNumVertices, begin + delta * t1, a * t1 * t1 + b * t1 + c, p))
	{
		if(outFraction)	*outFraction = t1;
		if(outPoint)	*outPoint = inPlane.ConvertPlaneToWorld(u, v, p);
		return true;
	}

	// Test if sphere intersects with one of the edges or vertices
	if(SweptCircleEdgeVertexIntersect(inVertices, inNumVertices, begin, delta, a, b, c, &p, outFraction))
	{
		if(outPoint)	*outPoint = inPlane.ConvertPlaneToWorld(u, v, p);
		return true;
	}
	return false;
}

static bool LSSTriangleOverlap(const Gu::Triangle& triangle, const PxVec3& p0, const PxVec3& dir, PxReal radius, PxVec3* hit_point, PxReal* t)
{
	const Gu::Plane plane(triangle.verts[0], triangle.verts[1], triangle.verts[2]);

	SE_Plane SEP;
	SEP.mConstant = plane.d;
	SEP.mNormal.x = plane.normal.x;
	SEP.mNormal.y = plane.normal.y;
	SEP.mNormal.z = plane.normal.z;

	// Get basis
	SE_Vector3 u, v;
	SEP.GetBasisVectors(u, v);

	SE_Vector2 Proj[3];
	for(PxU32 i=0; i<3; i++)
	{
		Proj[i] = ConvertWorldToPlane(u, v, (const SE_Vector3&)triangle.verts[i]);
	}
	return PolygonSweptSphereIntersect(SEP, u, v, Proj, 3, (const SE_Vector3&)p0, (const SE_Vector3&)dir, radius, (SE_Vector3*)hit_point, t);
}

static bool SweepSphereTriangles(PxU32 nb_tris, const Gu::Triangle* PX_RESTRICT triangles,
							const PxVec3& center, const PxReal radius,
							const PxVec3& dir, PxReal length, bool enableShrinking, const PxU32* PX_RESTRICT cachedIndex,
							PxVec3& hit, PxVec3& normal, PxReal& t, PxU32& index)
{
	// The reason why there is an option for shrinking was the following comment:
	// CA: Don't shrink. Fixes jitter issue reported by GRIN

	if(!nb_tris) return false;

	index = PX_INVALID_U32;
	PxU32 initIndex = 0;
	if (cachedIndex)
	{
		PX_ASSERT(*cachedIndex < nb_tris);
		initIndex = *cachedIndex;
	}

	PxReal curT = length;
	const PxVec3 D = dir*length;	// ### shrink D for each triangle

	const PxReal dpc0 = center.dot(dir);

	const Gu::Triangle* TriangleBase = triangles;

	for(PxU32 ii=0; ii<nb_tris; ii++)	// We need i for returned triangle index
	{
		PxU32 i;
		if(ii==0)				i=initIndex;
		else if(ii==initIndex)	i=0;
		else					i=ii;

		const Gu::Triangle& CurrentTri = triangles[i];

		if(!CoarseCulling(center, dir, curT, radius, CurrentTri))
			continue;
#ifdef _XBOX
		if(CullTriangle(CurrentTri, dir, radius, enableShrinking ? curT : length, dpc0)==0.0f)
			continue;
#else
		if(!CullTriangle(CurrentTri, dir, radius, enableShrinking ? curT : length, dpc0))
			continue;
#endif
		// FIXES BUG BUT TODO BETTER
		PxVec3 TriNormal;
		PxReal Magnitude;
		PxReal Area;
		ComputeTriData(CurrentTri, TriNormal, Magnitude, Area);
		if(Area==0.0f) continue;

		// Backface culling
		const bool Culled = (TriNormal.dot(dir)) > 0.0f;
		if(Culled) continue;

		// Sweep against current triangle
		PxReal CurrentDistance;

#ifdef USE_NEW_SWEEP_TEST
		TriNormal /= Magnitude;
		CurrentDistance = 10000.0f;
		if(SweepTriSphere(CurrentTri, TriNormal, center, radius, dir, CurrentDistance))
		{
#else

//float tmpDist = 10000.0f;
//bool ss = SweepTriSphere(CurrentTri, TriNormal/Magnitude, center, radius, dir, tmpDist);


		PxVec3 tmp;
		if(LSSTriangleOverlap(CurrentTri, center, D, radius, &tmp, &CurrentDistance))
		{
			CurrentDistance *= length;
			TriNormal /= Magnitude;
#endif
			CurrentDistance += ( TriNormal.dot(dir) ) * gEpsilon;

			if(CurrentDistance<curT)
			{
#ifndef USE_NEW_SWEEP_TEST
				hit = tmp;
#endif
				curT = CurrentDistance;
				index = i;	// WARNING, only index in limited set of returned triangles
			}
		}
	}
	if(index==PX_INVALID_U32)
		return false;	// We didn't touch any triangle
	else
		t = curT;

#ifdef USE_NEW_SWEEP_TEST
	// We need the impact point, not computed by the new code
	{
		const Gu::Triangle& CurrentTri = TriangleBase[index];
		if(!LSSTriangleOverlap(CurrentTri, center, D, radius, &hit, NULL))
		{
			return false;
		}
	}
#endif

	// Compute impact normal only in the end
	// Using tri normal
	if(0)
	{
		const Gu::Triangle& CurrentTri = triangles[index];
		CurrentTri.normal(normal);
	}

	if(1)
	{
		// This is responsible for the cap-vs-box stuck while jumping. However it's needed to slide on box corners!
		PxVec3 NewCenter = center + dir*t;
		normal = (NewCenter - hit);
		normal.normalize();
	}

	if(0)
	{
//			const PxTriangle& CurrentTri = triangles[index];
		const Gu::Triangle& CurrentTri = TriangleBase[index];

		// Another test... best middle ground so far
		PxVec3 NewCenter = center + dir*t;
		PxReal u,v;
		float d = Gu::distancePointTriangleSquared(NewCenter,	CurrentTri.verts[0],
																CurrentTri.verts[1] - CurrentTri.verts[0],
																CurrentTri.verts[2] - CurrentTri.verts[0],
																&u, &v);

		PxVec3 PointOnTri = Ps::computeBarycentricPoint(CurrentTri.verts[0], CurrentTri.verts[1], CurrentTri.verts[2], u, v);
		normal = (NewCenter - PointOnTri);
		normal.normalize();
	}

	if(1)
	{
		const Gu::Triangle& CurrentTri = TriangleBase[index];

		// Put back the real dist :
		PxVec3 TriRealNormal;
		CurrentTri.normal( TriRealNormal );// PT: TODO: don't recompute that one!
		t -= ( TriRealNormal.dot(dir) ) * gEpsilon;
	}
	return true;
}

//
// The problem of testing a swept capsule against a box is transformed into sweeping a sphere (lying at the center
// of the capsule) against the extruded triangles of the box. The box triangles are extruded along the
// capsule segment axis.
//
static bool SweepCapsuleBox(const Gu::Capsule& capsule, const PxTransform& boxWorldPose, const PxVec3& boxDim, const PxVec3& dir, PxReal length, PxVec3& hit, PxReal& min_dist, PxVec3& normal, PxSceneQueryFlags hintFlags)
{
	if(hintFlags & PxSceneQueryFlag::eINITIAL_OVERLAP)
	{
		// PT: test if shapes initially overlap
		if(Gu::distanceSegmentBoxSquared(capsule.p0, capsule.p1, boxWorldPose.p, boxDim, PxMat33(boxWorldPose.q)) < capsule.radius*capsule.radius)
		{
			min_dist	= 0.0f;
			normal		= -dir;
			hit			= boxWorldPose.p;	// PT: this is arbitrary
			return true;
		}
	}

	// Extrusion dir = capsule segment
	const PxVec3 ExtrusionDir = (capsule.p1 - capsule.p0)*0.5f;

	// Extrude box
	PxReal MinDist = length;
	bool Status = false;
	{
		const PxBounds3 aabb(-boxDim, boxDim);

		PX_ALLOCA(triangles, Gu::Triangle, 12*7);
		PxU32 NbTris = ExtrudeBox(aabb, &boxWorldPose, ExtrusionDir, triangles, dir);
		PX_ASSERT(NbTris<=12*7);

		// Sweep sphere vs extruded box
		PxVec3 n;
		PxReal md;
		PxU32 trash;
		
		if(SweepSphereTriangles(NbTris, triangles, capsule.computeCenter(), capsule.radius, dir, length, false, NULL, hit, n, md, trash))
		{
			MinDist = md;
			normal = n;
			Status = true;
		}
	}

	min_dist = MinDist;
	return Status;
}

static PX_FORCE_INLINE void computeSweptBox(const PxVec3& extents, const PxVec3& center, const PxMat33& rot, const PxVec3& unitDir, const PxReal distance, Gu::Box& box)
{
	PxVec3 R1, R2;
	Gu::computeBasis(unitDir, R1, R2);

	PxReal dd[3];
	dd[0] = PxAbs(rot.column0.dot(unitDir));
	dd[1] = PxAbs(rot.column1.dot(unitDir));
	dd[2] = PxAbs(rot.column2.dot(unitDir));
	PxReal dmax = dd[0];
	PxU32 ax0=1;
	PxU32 ax1=2;
	if(dd[1]>dmax)
	{
		dmax=dd[1];
		ax0=0;
		ax1=2;
	}
	if(dd[2]>dmax)
	{
		dmax=dd[2];
		ax0=0;
		ax1=1;
	}
	if(dd[ax1]<dd[ax0])
	{
		PxU32 swap = ax0;
		ax0 = ax1;
		ax1 = swap;
	}

	R1 = rot[ax0];
	R1 -= (R1.dot(unitDir))*unitDir;	// Project to plane whose normal is dir
	R1.normalize();
	R2 = unitDir.cross(R1);

	box.setAxes(unitDir, R1, R2);

	PxReal Offset[3];
	Offset[0] = distance;
	Offset[1] = distance*(unitDir.dot(R1));
	Offset[2] = distance*(unitDir.dot(R2));

	for(PxU32 r=0; r<3; r++)
	{
		const PxVec3& R = box.rot[r];
		box.extents[r] = Offset[r]*0.5f + PxAbs(rot.column0.dot(R))*extents.x + PxAbs(rot.column1.dot(R))*extents.y + PxAbs(rot.column2.dot(R))*extents.z;
	}

	box.center = center + unitDir*distance*0.5f;
}

static bool sweepCapsuleTriangles(PxU32 nbTris, const Gu::Triangle* triangles, const Gu::Capsule& capsule, const PxVec3& unitDir, const PxReal distance,
							PxF32& t, PxVec3& normal, PxVec3& hit, PxU32& hitIndex, PxSceneQueryFlags hintFlags)
{
	if(!nbTris)
		return false;

	const PxVec3 capsuleCenter = capsule.computeCenter();

	if(hintFlags & PxSceneQueryFlag::eINITIAL_OVERLAP)
	{
		// PT: test if shapes initially overlap
		const PxVec3 segmentExtent = capsule.p1 - capsule.p0;
		const PxReal r2 = capsule.radius*capsule.radius;
		for(PxU32 i=0;i<nbTris;i++)
		{
			const PxVec3& p0 = triangles[i].verts[0];
			const PxVec3& p1 = triangles[i].verts[1];
			const PxVec3& p2 = triangles[i].verts[2];
			if(Gu::distanceSegmentTriangleSquared(capsule.p0, segmentExtent, p0, p1 - p0, p2 - p0)<r2)
			{
				hitIndex	= i;
				t			= 0.0f;
				normal		= -unitDir;
				hit			= capsuleCenter;	// PT: this is arbitrary
				return true;
			}
		}
	}

	// Extrusion dir = capsule segment
	const PxVec3 extrusionDir = (capsule.p0 - capsule.p1)*0.5f;

	// The nice thing with this approach is that we "just" fallback to already existing code
	bool Status;
	if(extrusionDir.isZero())
	{
		Status = SweepSphereTriangles(nbTris, triangles, capsuleCenter, capsule.radius, unitDir, distance, false, NULL, hit, normal, t, hitIndex);
	}
	else
	{
#ifdef NEW_SWEEP_CAPSULE_MESH
		if(1)
		{
			// - extrude on the fly
			// - re-enable shrinking
			// - don't recompute tri normal in the end
			// - remove "ubi soft fix" (and check that's effectively what it is)

/*			PX_ALLOCA(Extruded, Gu::Triangle, nbTris*7);
		PX_ALLOCA(Ids, PxU32, nbTris*7);

		PxU32 NbExtruded = ExtrudeMesh(nbTris, triangles, NULL, ExtrusionDir, Extruded, Ids, unitDir * distance, NULL);

		Status = SweepSphereTriangles(NbExtruded, Extruded, capsuleCenter, capsule.radius, unitDir, distance, false, NULL, hit, normal, t, hitIndex);
		if(Status && hitIndex!=PX_INVALID_U32)*/


			// PT: extrude mesh on the fly. This is a modified copy of SweepSphereTriangles, unfortunately
/*				static bool SweepSphereTriangles(PxU32 nb_tris, const Gu::Triangle* PX_RESTRICT triangles,
							const PxVec3& center, const PxReal radius,
							const PxVec3& dir, PxReal length, bool enableShrinking, const PxU32* PX_RESTRICT cachedIndex,
							PxVec3& hit, PxVec3& normal, PxReal& t, PxU32& index)*/

			// The reason why there is an option for shrinking was the following comment:
			// CA: Don't shrink. Fixes jitter issue reported by GRIN

				Gu::Triangle extrudedTris[7];
				PxVec3 extrudedTrisNormals[7];	// Not normalized

			hitIndex = PX_INVALID_U32;
			PxU32 initIndex = 0;
/*				if(cachedIndex)
			{
				PX_ASSERT(*cachedIndex < nb_tris);
				initIndex = *cachedIndex;
			}*/

//				const bool enableShrinking = false;	// PT: WHAT?!
			const bool enableShrinking = true;	// PT: WHAT?!
			const PxReal radius = capsule.radius;
			PxReal curT = distance;
			const PxVec3 D = unitDir*distance;	// ### shrink D for each triangle
//				const PxVec3 dir = unitDir * distance;

			const PxReal dpc0 = capsuleCenter.dot(unitDir);

//				const Gu::Triangle* triangleBase = triangles;

			Gu::Triangle bestTri;

			for(PxU32 ii=0; ii<nbTris; ii++)	// We need i for returned triangle index
			{
				PxU32 i;
				if(ii==0)				i=initIndex;
				else if(ii==initIndex)	i=0;
				else					i=ii;

				const Gu::Triangle& _currentTri = triangles[i];
/////////////
				// Create triangle normal
				PxVec3 denormalizedNormal;
				_currentTri.denormalizedNormal(denormalizedNormal);

				// Backface culling
				// bool DoCulling = (CurrentFlags & Gu::TriangleCollisionFlag::eDOUBLE_SIDED)==0;
				// bool Culled = (DoCulling && (DenormalizedNormal|dir) > 0.0f);
				const bool culled = (denormalizedNormal.dot(unitDir)) > 0.0f;
				if(culled)	continue;

				// Extrude mesh on the fly
				PxU32 nbExtrudedTris=0;

/*					if (sweptBounds)
				{
					PxVec3 tmp[3];
					tmp[0] = sweptBounds->rotateInv(CurrentTriangle.verts[0] - sweptBounds->center);
					tmp[1] = sweptBounds->rotateInv(CurrentTriangle.verts[1] - sweptBounds->center);
					tmp[2] = sweptBounds->rotateInv(CurrentTriangle.verts[2] - sweptBounds->center);
					const PxVec3 center(0.0f);
					if(!Gu::intersectTriangleBox(center, sweptBounds->extents, tmp[0], tmp[1], tmp[2]))
						continue;
				}*/

				PxVec3 p0 = _currentTri.verts[0];
				PxVec3 p1 = _currentTri.verts[1];
				PxVec3 p2 = _currentTri.verts[2];

				PxVec3 p0b = p0 + extrusionDir;
				PxVec3 p1b = p1 + extrusionDir;
				PxVec3 p2b = p2 + extrusionDir;

				p0 -= extrusionDir;
				p1 -= extrusionDir;
				p2 -= extrusionDir;

#define _OUTPUT_TRI(p0, p1, p2){														\
extrudedTris[nbExtrudedTris].verts[0] = p0;												\
extrudedTris[nbExtrudedTris].verts[1] = p1;												\
extrudedTris[nbExtrudedTris].verts[2] = p2;												\
extrudedTris[nbExtrudedTris].denormalizedNormal(extrudedTrisNormals[nbExtrudedTris]);	\
nbExtrudedTris++;}

#define _OUTPUT_TRI2(p0, p1, p2, d){			\
Gu::Triangle& t = extrudedTris[nbExtrudedTris];	\
t.verts[0] = p0;								\
t.verts[1] = p1;								\
t.verts[2] = p2;								\
PxVec3 nrm;										\
t.denormalizedNormal(nrm);						\
if(nrm.dot(d)>0.0f) {							\
PxVec3 tmp = t.verts[1];						\
t.verts[1] = t.verts[2];						\
t.verts[2] = tmp;								\
nrm = -nrm;										\
}												\
extrudedTrisNormals[nbExtrudedTris] = nrm;		\
nbExtrudedTris++; }

				if(denormalizedNormal.dot(extrusionDir) >= 0.0f)	_OUTPUT_TRI(p0b, p1b, p2b)
				else												_OUTPUT_TRI(p0, p1, p2)

				// ### it's probably useless to extrude all the shared edges !!!!!
				//if(CurrentFlags & Gu::TriangleCollisionFlag::eACTIVE_EDGE12)
				{
					_OUTPUT_TRI2(p1, p1b, p2b, unitDir)
					_OUTPUT_TRI2(p1, p2b, p2, unitDir)
				}
				//if(CurrentFlags & Gu::TriangleCollisionFlag::eACTIVE_EDGE20)
				{
					_OUTPUT_TRI2(p0, p2, p2b, unitDir)
					_OUTPUT_TRI2(p0, p2b, p0b, unitDir)
				}
				//if(CurrentFlags & Gu::TriangleCollisionFlag::eACTIVE_EDGE01)
				{
					_OUTPUT_TRI2(p0b, p1b, p1, unitDir)
					_OUTPUT_TRI2(p0b, p1, p0, unitDir)
				}
/////////////

				for(PxU32 j=0;j<nbExtrudedTris;j++)
				{
					const Gu::Triangle& currentTri = extrudedTris[j];

					PxVec3& triNormal = extrudedTrisNormals[j];
					// Backface culling
					const bool culled = (triNormal.dot(unitDir)) > 0.0f;
					if(culled)
						continue;

					// PT: beware, culling is only ok on the sphere I think
					if(!CoarseCulling(capsuleCenter, unitDir, curT, radius, currentTri))
						continue;
#ifdef _XBOX
					if(CullTriangle(currentTri, unitDir, radius, enableShrinking ? curT : distance, dpc0)==0.0f)
						continue;
#else
					if(!CullTriangle(currentTri, unitDir, radius, enableShrinking ? curT : distance, dpc0))
						continue;
#endif
					// FIXES BUG BUT TODO BETTER
//						PxVec3 triNormal;
//						PxReal magnitude;
//						PxReal area;
//						ComputeTriData(currentTri, triNormal, magnitude, area);
//						if(area==0.0f)
//							continue;

//						PxVec3 triNormal;
//						currentTri.denormalizedNormal(triNormal);
					PxReal magnitude = triNormal.magnitude();
					if(magnitude==0.0f)
						continue;

					// Backface culling
//						const bool culled = (triNormal.dot(unitDir)) > 0.0f;
//						if(culled)
//							continue;

					// Sweep against current triangle
					PxReal currentDistance;

#ifdef USE_NEW_SWEEP_TEST
					triNormal /= magnitude;
					currentDistance = 10000.0f;
					if(SweepTriSphere(currentTri, triNormal, capsuleCenter, radius, unitDir, currentDistance))
					{
#else
					PxVec3 tmp;
					if(LSSTriangleOverlap(currentTri, capsuleCenter, D, radius, &tmp, &currentDistance))
					{
						currentDistance *= distance;
						triNormal /= magnitude;
#endif
						// PT: I think this is useless now, because the impact normal is not computed with the tri normal
//							currentDistance += ( triNormal.dot(unitDir) ) * gEpsilon;

						if(currentDistance<curT)
						{
#ifdef USE_NEW_SWEEP_TEST
							bestTri = currentTri;
#else
							hit = tmp;
#endif
							curT = currentDistance;
							hitIndex = i;	// WARNING, only index in limited set of returned triangles
						}
					}
				}
			}

			if(hitIndex==PX_INVALID_U32)
				Status = false;	// We didn't touch any triangle
			else
			{
				t = curT;

#ifdef USE_NEW_SWEEP_TEST
				// We need the impact point, not computed by the new code
				{
/*						const Gu::Triangle& touchedTri = triangles[hitIndex];
					// PT: this isn't good because it can fail
					if(!LSSTriangleOverlap(touchedTri, capsuleCenter, D, radius, &hit, NULL))
					{
//							return false;
					}*/

					// Move sphere
					const PxVec3 newSphereCenter = capsuleCenter + unitDir * t;

					PxReal u, v;
					const PxReal s = Gu::distancePointTriangleSquared(newSphereCenter, bestTri.verts[0], bestTri.verts[1]-bestTri.verts[0], bestTri.verts[2]-bestTri.verts[0], &u, &v);

					hit = Ps::computeBarycentricPoint(bestTri.verts[0], bestTri.verts[1], bestTri.verts[2], u, v);
				}
#endif
				// Compute impact normal only in the end
				// PT: this one is also dubious since the capsule center can be far away from the hit point when the radius is big!
				if(1)
				{
					// This is responsible for the cap-vs-box stuck while jumping. However it's needed to slide on box corners!
					PxVec3 newCenter = capsuleCenter + unitDir*t;
					normal = (newCenter - hit);
					normal.normalize();
				}

/*					if(1)
				{
					const Gu::Triangle& currentTri = triangles[hitIndex];

					// Put back the real dist :
					PxVec3 triRealNormal;
					currentTri.normal( triRealNormal );	// PT: TODO: don't recompute that one!
					t -= ( triRealNormal.dot(unitDir) ) * gEpsilon;
				}*/
				Status = true;
			}
		}
		else
#endif
		{
			PX_ALLOCA(Extruded, Gu::Triangle, nbTris*7);
			PX_ALLOCA(Ids, PxU32, nbTris*7);

			PxU32 NbExtruded = ExtrudeMesh(nbTris, triangles, NULL, extrusionDir, Extruded, Ids, unitDir * distance, NULL);

			Status = SweepSphereTriangles(NbExtruded, Extruded, capsuleCenter, capsule.radius, unitDir, distance, false, NULL, hit, normal, t, hitIndex);
			if(Status && hitIndex!=PX_INVALID_U32)
				hitIndex = Ids[hitIndex];
		}
		if(Status && hitIndex!=PX_INVALID_U32)
		{
//				hitIndex = Ids[hitIndex];

			// PT: deadline in a few hours. No time. Should be cleaned later or re-thought.
			// PT: we need to recompute a hit here because the hit between the *capsule* and the source mesh can be very
			// different from the hit between the *sphere* and the extruded mesh.
			{
				// Move capsule
				const Gu::Segment Moved(
					PxVec3(capsule.p0 + (unitDir * t)),
					PxVec3(capsule.p1 + (unitDir * t)));

				// Touched tri
				const PxVec3& p0 = triangles[hitIndex].verts[0];
				const PxVec3& p1 = triangles[hitIndex].verts[1];
				const PxVec3& p2 = triangles[hitIndex].verts[2];

				PxReal Alpha, Beta, Gamma;
				const PxReal s = Gu::distanceSegmentTriangleSquared(Moved, p0, p1-p0, p2-p0, &Alpha, &Beta, &Gamma);

				hit = Ps::computeBarycentricPoint(p0, p1, p2, Beta, Gamma);

#ifdef NEW_SWEEP_CAPSULE_MESH
/*					if(1)
				{
					// This is responsible for the cap-vs-box stuck while jumping. However it's needed to slide on box corners!
					PxVec3 newCenter = capsuleCenter + unitDir*t;
					normal = (newCenter - hit);
					normal.normalize();
				}*/
#endif
			}
		}
	}
	return Status;
}

static bool SweepBoxSphere(const Gu::Box& box, PxReal sphereRadius, const PxVec3& spherePos, const PxVec3& dir, PxReal length, PxReal& min_dist, PxVec3& normal, PxSceneQueryFlags hintFlags)
{
	if(hintFlags & PxSceneQueryFlag::eINITIAL_OVERLAP)
	{
		// PT: test if shapes initially overlap
		if(Gu::intersectSphereBox(Gu::Sphere(spherePos, sphereRadius), box))
		{
			// Overlap
			min_dist	= 0.0f;
			normal		= -dir;
//			return false;
			return true;	// PT: TODO: was false, changed to true. Check the consequence on the kinematic CCT.
		}
	}

	PxVec3 WP[8];
	box.computeBoxPoints(WP);
	const PxU8* PX_RESTRICT Edges = Gu::getBoxEdges();
	PxReal MinDist = length;
	bool Status = false;
	for(PxU32 i=0; i<12; i++)
	{
		const PxU8 e0 = *Edges++;
		const PxU8 e1 = *Edges++;
		const Gu::Segment segment(WP[e0], WP[e1]);
		const Gu::Capsule Capsule(segment, sphereRadius);

		PxReal s[2];
		PxU32 n = Gu::intersectRayCapsule(spherePos, dir, Capsule, s);
		if(n)
		{
			PxReal t;
			if (n == 1)	t = s[0];
			else t = (s[0] < s[1]) ? s[0]:s[1];

			if(t>=0.0f && t<MinDist)
			{
				MinDist = t;

				const PxVec3 ip = spherePos + t*dir;
				Gu::distancePointSegmentSquared(Capsule, ip, &t);

				PxVec3 ip2;
				Capsule.computePoint(ip2, t);

				normal = (ip2 - ip);
				normal.normalize();
				Status = true;
			}
		}
	}

	const PxMat34Legacy M = buildMatrixFromBox(box);
	const PxVec3 LocalDir = box.rotateInv(dir);
	PxVec3 LocalPt;
	M.multiplyByInverseRT(spherePos, LocalPt);

	Gu::Box WorldBox0 = box;
	Gu::Box WorldBox1 = box;
	Gu::Box WorldBox2 = box;
	WorldBox0.extents.x += sphereRadius;
	WorldBox1.extents.y += sphereRadius;
	WorldBox2.extents.z += sphereRadius;

	const PxVec3* BoxNormals = (const PxVec3*)gNearPlaneNormal;

	PxReal tnear, tfar;
	int plane = Gu::intersectRayAABB(-WorldBox0.extents, WorldBox0.extents, LocalPt, LocalDir, tnear, tfar);

	if(plane!=-1 && tnear>=0.0f && tnear < MinDist)
	{
		MinDist = tnear;
		normal = box.rotate(BoxNormals[plane]);
		Status = true;
	}

	plane = Gu::intersectRayAABB(-WorldBox1.extents, WorldBox1.extents, LocalPt, LocalDir, tnear, tfar);

	if(plane!=-1 && tnear>=0.0f && tnear < MinDist)
	{
		MinDist = tnear;
		normal = box.rotate(BoxNormals[plane]);
		Status = true;
	}

	plane = Gu::intersectRayAABB(-WorldBox2.extents, WorldBox2.extents, LocalPt, LocalDir, tnear, tfar);

	if(plane!=-1 && tnear>=0.0f && tnear < MinDist)
	{
		MinDist = tnear;
		normal = box.rotate(BoxNormals[plane]);
		Status = true;
	}

	min_dist = MinDist;

	return Status;
}

static PX_FORCE_INLINE void computeWorldToBoxMatrix(Cm::Matrix34& worldToBox, const Gu::Box& box)
{
	Cm::Matrix34 boxToWorld;
	buildMatrixFromBox(boxToWorld, box);
	worldToBox = boxToWorld.getInverseRT();
}

// ### optimize! and refactor. And optimize for aabbs
static bool SweepBoxBox(const Gu::Box& box0, const Gu::Box& box1, const PxVec3& dir, PxReal length, PxVec3& hit, PxVec3& normal, PxReal& t, PxSceneQueryFlags hintFlags)
{
	if(hintFlags & PxSceneQueryFlag::eINITIAL_OVERLAP)
	{
		// PT: test if shapes initially overlap

		// ### isn't this dangerous ? It prevents back motions...
		// => It is indeed dangerous. We need to change this for overlap recovery.
		// Overlap *can* sometimes be allowed for gameplay reason (e.g. character-through-glass effect)
		if(Gu::intersectOBBOBB(box0.extents, box0.center, box0.rot, box1.extents, box1.center, box1.rot, true))
		{
			t		= 0.0f;
			normal	= -dir;
			hit		= box0.center;	// PT: this is arbitrary
//			return false;
			return true;	// PT: TODO: this was false, changed it to true. Make sure it still works for the kinematic CCT.
		}
	}

	PxVec3 boxVertices0[8];	box0.computeBoxPoints(boxVertices0);
	PxVec3 boxVertices1[8];	box1.computeBoxPoints(boxVertices1);

	//	float MinDist = PX_MAX_F32;
	PxReal MinDist = length;
	int col = -1;

	// In following VF tests:
	// - the direction is FW/BK since we project one box onto the other *and vice versa*
	// - the normal reaction is FW/BK for the same reason

	// Vertices1 against Box0
	if(1)
	{
		// We need:

		// - Box0 in local space
		const PxVec3 Min0 = -box0.extents;
		const PxVec3 Max0 = box0.extents;

		// - Vertices1 in Box0 space
		Cm::Matrix34 WorldToBox0;
		computeWorldToBoxMatrix(WorldToBox0, box0);

		// - the dir in Box0 space
		const PxVec3 LocalDir0 = WorldToBox0.rotate(dir);

		const PxVec3* BoxNormals0 = (const PxVec3*)gNearPlaneNormal;

		for(PxU32 i=0; i<8; i++)
		{
			PxReal tnear, tfar;
			const int plane = Gu::intersectRayAABB(Min0, Max0, WorldToBox0.transform(boxVertices1[i]), -LocalDir0, tnear, tfar);

			if(plane==-1 || tnear<0.0f)	continue;

			if(tnear < MinDist)
			{
				MinDist = tnear;
				normal = box0.rotate(BoxNormals0[plane]);
				hit = boxVertices1[i];
				col = 0;
			}
		}
	}

	// Vertices0 against Box1
	if(1)
	{
		// We need:

		// - Box1 in local space
		const PxVec3 Min1 = -box1.extents;
		const PxVec3 Max1 = box1.extents;

		// - Vertices0 in Box1 space
		Cm::Matrix34 WorldToBox1;
		computeWorldToBoxMatrix(WorldToBox1, box1);

		// - the dir in Box1 space
		const PxVec3 LocalDir1 = WorldToBox1.rotate(dir);

		const PxVec3* BoxNormals1 = (const PxVec3*)gNearPlaneNormal;

		for(PxU32 i=0; i<8; i++)
		{
			PxReal tnear, tfar;
			const int plane = Gu::intersectRayAABB(Min1, Max1, WorldToBox1.transform(boxVertices0[i]), LocalDir1, tnear, tfar);

			if(plane==-1 || tnear<0.0f)	continue;

			if(tnear < MinDist)
			{
				MinDist = tnear;
				normal = box1.rotate(-BoxNormals1[plane]);
				hit = boxVertices0[i] + tnear * dir;
				col = 1;
			}
		}
	}

	if(1)
	{
		const PxU8* PX_RESTRICT Edges0 = Gu::getBoxEdges();
		const PxU8* PX_RESTRICT Edges1 = Gu::getBoxEdges();

		PxVec3 EdgeNormals0[12];
		PxVec3 EdgeNormals1[12];
		for(PxU32 i=0; i<12; i++)	Gu::computeBoxWorldEdgeNormal(box0, i, EdgeNormals0[i]);
		for(PxU32 i=0; i<12; i++)	Gu::computeBoxWorldEdgeNormal(box1, i, EdgeNormals1[i]);

		// Loop through box edges
		for(PxU32 i=0; i<12; i++)	// 12 edges
		{
			if(!(EdgeNormals0[i].dot(dir) >= 0.0f)) continue;

			// Catch current box edge // ### one vertex already known using line-strips

			// Make it fat ###
			PxVec3 p1 = boxVertices0[Edges0[i*2+0]];
			PxVec3 p2 = boxVertices0[Edges0[i*2+1]];
			Ps::makeFatEdge(p1, p2, gFatBoxEdgeCoeff);

			// Loop through box edges
			for(PxU32 j=0;j<12;j++)
			{
				if(EdgeNormals1[j].dot(dir) >= 0.0f) continue;

				// Orientation culling
				// PT: this was commented for some reason, but it fixes the "stuck" bug reported by Ubi.
				// So I put it back. We'll have to see whether it produces Bad Things in particular cases.
				if(EdgeNormals0[i].dot(EdgeNormals1[j]) >= 0.0f)	continue;

				// Catch current box edge

				// Make it fat ###
				PxVec3 p3 = boxVertices1[Edges1[j*2+0]];
				PxVec3 p4 = boxVertices1[Edges1[j*2+1]];
				Ps::makeFatEdge(p3, p4, gFatBoxEdgeCoeff);

				PxReal Dist;
				PxVec3 ip;
				if(Gu::intersectEdgeEdge(p1, p2, dir, p3, p4, Dist, ip))
				{
					if(Dist<MinDist)
					{
						hit = ip + Dist * dir;

						normal = (p1-p2).cross(p3-p4);
						normal.normalize();
						if((normal.dot(dir)) > 0.0f) normal = -normal;

						col = 2;
						MinDist = Dist;
					}
				}
			}
		}
	}

	if(col==-1)	return false;

	t = MinDist;

	return true;
}

static Gu::Triangle InflateTriangle(const Gu::Triangle& triangle, PxReal fat_coeff)
{
	Gu::Triangle fatTri = triangle;

	// Compute triangle center
	const PxVec3& p0 = triangle.verts[0];
	const PxVec3& p1 = triangle.verts[1];
	const PxVec3& p2 = triangle.verts[2];
	const PxVec3 center = (p0 + p1 + p2)*0.333333333f;

	// Don't normalize?
	// Normalize => add a constant border, regardless of triangle size
	// Don't => add more to big triangles
	for(PxU32 i=0;i<3;i++)
	{
		const PxVec3 v = fatTri.verts[i] - center;
		fatTri.verts[i] += v * fat_coeff;
	}

	return fatTri;
}

// PT: special version to fire N parallel rays against the same tri
static PX_FORCE_INLINE Ps::IntBool RayTriPrecaCull(const PxVec3& orig, const PxVec3& dir, const PxVec3& vert0, const PxVec3& edge1, const PxVec3& edge2, const PxVec3& pvec,
						PxReal det, PxReal oneOverDet, PxReal& t)
{
	// Calculate distance from vert0 to ray origin
	const PxVec3 tvec = orig - vert0;

	// Calculate U parameter and test bounds
	PxReal u = tvec.dot(pvec);
	if((u < 0.0f) || u>det)				return 0;

	// Prepare to test V parameter
	const PxVec3 qvec = tvec.cross(edge1);

	// Calculate V parameter and test bounds
	PxReal v = dir.dot(qvec);
	if((v < 0.0f) || u+v>det)			return 0;

	// Calculate t, scale parameters, ray intersects triangle
	t = edge2.dot(qvec);
	t *= oneOverDet;
	return 1;
}

static PX_FORCE_INLINE Ps::IntBool RayTriPrecaNoCull(const PxVec3& orig, const PxVec3& dir, const PxVec3& vert0, const PxVec3& edge1, const PxVec3& edge2, const PxVec3& pvec,
						PxReal det, PxReal oneOverDet, PxReal& t)
{
	// Calculate distance from vert0 to ray origin
	const PxVec3 tvec = orig - vert0;

	// Calculate U parameter and test bounds
	PxReal u = (tvec.dot(pvec)) * oneOverDet;
	if((u < 0.0f) || u>1.0f)			return 0;

	// prepare to test V parameter
	const PxVec3 qvec = tvec.cross(edge1);

	// Calculate V parameter and test bounds
	PxReal v = (dir.dot(qvec)) * oneOverDet;
	if((v < 0.0f) || u+v>1.0f)			return 0;

	// Calculate t, ray intersects triangle
	t = (edge2.dot(qvec)) * oneOverDet;
	return 1;
}

static PX_FORCE_INLINE void closestAxis2(const PxVec3& v, PxU32& j, PxU32& k)
{
	// find largest 2D plane projection
	const PxF32 absPx = physx::intrinsics::abs(v.x);
	const PxF32 absPy = physx::intrinsics::abs(v.y);
	const PxF32 absPz = physx::intrinsics::abs(v.z);
#ifdef _XBOX
	const float delta = absPx - absPy;

	float max = physx::intrinsics::fsel(delta, absPx, absPy);
//		float m = physx::intrinsics::fsel(delta, 0.0f, 1.0f);
	float m = physx::intrinsics::fsel(delta, 1.0f, 2.0f);

	const float delta2 = max - absPz;
//		max = physx::intrinsics::fsel(delta2, max, absPz);
//		m = physx::intrinsics::fsel(delta2, m, 2.0f);
	m = physx::intrinsics::fsel(delta2, m, 0.0f);

	j = PxU32(m);
	k=j+1;
	if(k>2)
		k=0;

//		j = Ps::getNextIndex3(i);
//		k = Ps::getNextIndex3(j);

//		return i;
#else
	//PxU32 m = 0;	//x biggest axis
	j = 1;
	k = 2;
	if( absPy > absPx && absPy > absPz)
	{
		//y biggest
		j = 2;
		k = 0;
		//m = 1;
	}
	else if(absPz > absPx)
	{
		//z biggest
		j = 0;
		k = 1;
		//m = 2;
	}
//		return m;
#endif
}

static PX_FORCE_INLINE	void getScaledTriangle(const PxTriangleMeshGeometry& triGeom, const Cm::Matrix34& vertex2worldSkew, Gu::Triangle& triangle, PxTriangleID triangleIndex)
{
	Gu::TriangleMesh* tm = static_cast<Gu::TriangleMesh*>(triGeom.triangleMesh);
	tm->computeWorldTriangle(triangle, triangleIndex, vertex2worldSkew);
}

#define LAZY_NORMALIZE
#define PRECOMPUTE_FAT_BOX	// PT: TODO: clean this up
#define PRECOMPUTE_FAT_BOX_MORE

#ifdef PRECOMPUTE_FAT_BOX

/*
static PX_FORCE_INLINE void computeFatEdges(const PxVec3* PX_RESTRICT boxVertices, PxVec3* PX_RESTRICT fatEdges)
{
	const PxU8* PX_RESTRICT Edges = Gu::getBoxEdges();
	// Loop through box edges
	for(PxU32 i=0;i<12;i++)	// 12 edges
	{
		fatEdges[i*2  ] = boxVertices[*Edges++];
		fatEdges[i*2+1] = boxVertices[*Edges++];
		Ps::makeFatEdge(fatEdges[i*2], fatEdges[i*2+1], gFatBoxEdgeCoeff);
	}
}
*/

#ifdef PRECOMPUTE_FAT_BOX_MORE
struct FatEdgeData
{
	PX_FORCE_INLINE	FatEdgeData(){}
	PX_FORCE_INLINE	~FatEdgeData(){}
	Gu::Plane	plane;
	PxVec3		p1;
	PxVec3		p2;
	PxVec3		v1;
	PxU32		ii;
	PxU32		jj;
	PxReal		coeff;
	union
	{
		PxReal		edgeNormalDp;
		int			edgeNormalDpBin;
	};
};

static PX_FORCE_INLINE void computeFatEdges(const PxVec3* PX_RESTRICT boxVertices, FatEdgeData* PX_RESTRICT fatEdges, const PxVec3& motion)
{
	const PxU8* PX_RESTRICT Edges = Gu::getBoxEdges();
	const PxVec3* PX_RESTRICT EdgeNormals = Gu::getBoxLocalEdgeNormals();
	// Loop through box edges
	for(PxU32 i=0;i<12;i++)	// 12 edges
	{
		PxVec3 p1 = boxVertices[*Edges++];
		PxVec3 p2 = boxVertices[*Edges++];
		Ps::makeFatEdge(p1, p2, gFatBoxEdgeCoeff);

		fatEdges[i].edgeNormalDp = EdgeNormals[i].dot(motion);

		// While we're at it, precompute some more data for EE tests
		const PxVec3 v1 = p2 - p1;

		// Build plane P based on edge (p1, p2) and direction (dir)
		fatEdges[i].plane.normal = v1.cross(motion);
		fatEdges[i].plane.d = -(fatEdges[i].plane.normal.dot(p1));

		// find largest 2D plane projection
		PxU32 ii,jj;
	//	Ps::closestAxis(plane.normal, ii, jj);
		closestAxis2(fatEdges[i].plane.normal, ii, jj);

		fatEdges[i].coeff = 1.0f / (v1[ii]*motion[jj] - v1[jj]*motion[ii]);

		fatEdges[i].p1 = p1;
		fatEdges[i].p2 = p2;
		fatEdges[i].v1 = v1;
		fatEdges[i].ii = ii;
		fatEdges[i].jj = jj;
	}
}
#endif
#endif


// PT: specialized version where oneOverDir is available
// PT: why did we change the initial epsilon value?
#define LOCAL_EPSILON_RAY_BOX PX_EPS_F32
//#define LOCAL_EPSILON_RAY_BOX 0.0001f
static PX_FORCE_INLINE int intersectRayAABB2(const PxVec3& minimum, const PxVec3& maximum,
							const PxVec3& ro, const PxVec3& rd, const PxVec3& oneOverDir,
							float& tnear, float& tfar,
							bool fbx, bool fby, bool fbz)
{
	// PT: this unrolled loop is a lot faster on Xbox

	if(fbx)
		if(ro.x<minimum.x || ro.x>maximum.x)
		{
//			tnear = FLT_MAX;
			return -1;
		}
	if(fby)
		if(ro.y<minimum.y || ro.y>maximum.y)
		{
//			tnear = FLT_MAX;
			return -1;
		}
	if(fbz)
		if(ro.z<minimum.z || ro.z>maximum.z)
		{
//			tnear = FLT_MAX;
			return -1;
		}

#ifdef _XBOX
	const PxReal t1x_candidate = (minimum.x - ro.x) * oneOverDir.x;
	const PxReal t2x_candidate = (maximum.x - ro.x) * oneOverDir.x;
	const PxReal t1y_candidate = (minimum.y - ro.y) * oneOverDir.y;
	const PxReal t2y_candidate = (maximum.y - ro.y) * oneOverDir.y;
	const PxReal t1z_candidate = (minimum.z - ro.z) * oneOverDir.z;
	const PxReal t2z_candidate = (maximum.z - ro.z) * oneOverDir.z;

	const float deltax = t1x_candidate - t2x_candidate;
	const float deltay = t1y_candidate - t2y_candidate;
	const float deltaz = t1z_candidate - t2z_candidate;

	const float t1x = physx::intrinsics::fsel(deltax, t2x_candidate, t1x_candidate);
	const float t1y = physx::intrinsics::fsel(deltay, t2y_candidate, t1y_candidate);
	const float t1z = physx::intrinsics::fsel(deltaz, t2z_candidate, t1z_candidate);

	const float t2x = physx::intrinsics::fsel(deltax, t1x_candidate, t2x_candidate);
	const float t2y = physx::intrinsics::fsel(deltay, t1y_candidate, t2y_candidate);
	const float t2z = physx::intrinsics::fsel(deltaz, t1z_candidate, t2z_candidate);

	const float bxf = physx::intrinsics::fsel(deltax, 3.0f, 0.0f);
	const float byf = physx::intrinsics::fsel(deltay, 4.0f, 1.0f);
	const float bzf = physx::intrinsics::fsel(deltaz, 5.0f, 2.0f);

	tnear = t1x;
	tfar = t2x;
	float ret = bxf;

	const float delta = t1y - tnear;
	tnear = physx::intrinsics::fsel(delta, t1y, tnear);
	ret = physx::intrinsics::fsel(delta, byf, ret);
	tfar = physx::intrinsics::selectMin(tfar, t2y);

	const float delta2 = t1z - tnear;
	tnear = physx::intrinsics::fsel(delta2, t1z, tnear);
	ret = physx::intrinsics::fsel(delta2, bzf, ret);

	tfar = physx::intrinsics::selectMin(tfar, t2z);

	// PT: this fcmp seems cheaper than the alternative LHS below
	// TODO: return a "float bool" here as well, unify with other ray-box code
	if(tnear>tfar || tfar<LOCAL_EPSILON_RAY_BOX)
	{
//		tnear = FLT_MAX;
		return -1;
	}
//	ret = physx::intrinsics::fsel(tfar - tnear, ret, -1.0f);
///	ret = physx::intrinsics::fsel(LOCAL_EPSILON_RAY_BOX - tfar, -1.0f, ret);

	return int(ret);
#else
	PxReal t1x = (minimum.x - ro.x) * oneOverDir.x;
	PxReal t2x = (maximum.x - ro.x) * oneOverDir.x;
	PxReal t1y = (minimum.y - ro.y) * oneOverDir.y;
	PxReal t2y = (maximum.y - ro.y) * oneOverDir.y;
	PxReal t1z = (minimum.z - ro.z) * oneOverDir.z;
	PxReal t2z = (maximum.z - ro.z) * oneOverDir.z;

	int bx;
	int by;
	int bz;

	if(t1x>t2x)
	{
		PxReal t=t1x; t1x=t2x; t2x=t;
		bx = 3;
	}
	else
	{
		bx = 0;
	}

	if(t1y>t2y)
	{
		PxReal t=t1y; t1y=t2y; t2y=t;
		by = 4;
	}
	else
	{
		by = 1;
	}

	if(t1z>t2z)
	{
		PxReal t=t1z; t1z=t2z; t2z=t;
		bz = 5;
	}
	else
	{
		bz = 2;
	}

	int ret;
//	if(t1x>tnear)	// PT: no need to test for the first value
	{
		tnear = t1x;
		ret = bx;
	}
//	tfar = Px::intrinsics::selectMin(tfar, t2x);
	tfar = t2x;		// PT: no need to test for the first value

	if(t1y>tnear)
	{
		tnear = t1y;
		ret = by;
	}
	tfar = physx::intrinsics::selectMin(tfar, t2y);

	if(t1z>tnear)
	{
		tnear = t1z;
		ret = bz;
	}
	tfar = physx::intrinsics::selectMin(tfar, t2z);

	if(tnear>tfar || tfar<LOCAL_EPSILON_RAY_BOX)
		return -1;

	return ret;
#endif
}


#ifdef TOSEE	// PT: TODO: vectorize this properly

static PX_FORCE_INLINE int intersectRayAABB3(const PxVec3& minimum, const PxVec3& maximum,
									  const Gu::Triangle& tri, const PxVec3& rd, const PxVec3& oneOverDir,
									float& tnear, int& hit,
									bool fbx, bool fby, bool fbz)
{
/*	if(fbx)
		if(ro.x<minimum.x || ro.x>maximum.x)
		{
//			return -1;
		}
	if(fby)
		if(ro.y<minimum.y || ro.y>maximum.y)
		{
//			return -1;
		}
	if(fbz)
		if(ro.z<minimum.z || ro.z>maximum.z)
		{
//			return -1;
		}
*/

	const PxReal t1x_candidate0 = (minimum.x - tri.verts[0].x) * oneOverDir.x;
	const PxReal t2x_candidate0 = (maximum.x - tri.verts[0].x) * oneOverDir.x;
	const PxReal t1x_candidate1 = (minimum.x - tri.verts[1].x) * oneOverDir.x;
	const PxReal t2x_candidate1 = (maximum.x - tri.verts[1].x) * oneOverDir.x;
	const PxReal t1x_candidate2 = (minimum.x - tri.verts[2].x) * oneOverDir.x;
	const PxReal t2x_candidate2 = (maximum.x - tri.verts[2].x) * oneOverDir.x;

	const PxReal t1y_candidate0 = (minimum.y - tri.verts[0].y) * oneOverDir.y;
	const PxReal t2y_candidate0 = (maximum.y - tri.verts[0].y) * oneOverDir.y;
	const PxReal t1y_candidate1 = (minimum.y - tri.verts[1].y) * oneOverDir.y;
	const PxReal t2y_candidate1 = (maximum.y - tri.verts[1].y) * oneOverDir.y;
	const PxReal t1y_candidate2 = (minimum.y - tri.verts[2].y) * oneOverDir.y;
	const PxReal t2y_candidate2 = (maximum.y - tri.verts[2].y) * oneOverDir.y;

	const PxReal t1z_candidate0 = (minimum.z - tri.verts[0].z) * oneOverDir.z;
	const PxReal t2z_candidate0 = (maximum.z - tri.verts[0].z) * oneOverDir.z;
	const PxReal t1z_candidate1 = (minimum.z - tri.verts[1].z) * oneOverDir.z;
	const PxReal t2z_candidate1 = (maximum.z - tri.verts[1].z) * oneOverDir.z;
	const PxReal t1z_candidate2 = (minimum.z - tri.verts[2].z) * oneOverDir.z;
	const PxReal t2z_candidate2 = (maximum.z - tri.verts[2].z) * oneOverDir.z;

	const float deltax0 = t1x_candidate0 - t2x_candidate0;
	const float deltax1 = t1x_candidate1 - t2x_candidate1;
	const float deltax2 = t1x_candidate2 - t2x_candidate2;

	const float deltay0 = t1y_candidate0 - t2y_candidate0;
	const float deltay1 = t1y_candidate1 - t2y_candidate1;
	const float deltay2 = t1y_candidate2 - t2y_candidate2;

	const float deltaz0 = t1z_candidate0 - t2z_candidate0;
	const float deltaz1 = t1z_candidate1 - t2z_candidate1;
	const float deltaz2 = t1z_candidate2 - t2z_candidate2;

	const float t1x0 = physx::intrinsics::fsel(deltax0, t2x_candidate0, t1x_candidate0);
	const float t1x1 = physx::intrinsics::fsel(deltax1, t2x_candidate1, t1x_candidate1);
	const float t1x2 = physx::intrinsics::fsel(deltax2, t2x_candidate2, t1x_candidate2);

	const float t1y0 = physx::intrinsics::fsel(deltay0, t2y_candidate0, t1y_candidate0);
	const float t1y1 = physx::intrinsics::fsel(deltay1, t2y_candidate1, t1y_candidate1);
	const float t1y2 = physx::intrinsics::fsel(deltay2, t2y_candidate2, t1y_candidate2);

	const float t1z0 = physx::intrinsics::fsel(deltaz0, t2z_candidate0, t1z_candidate0);
	const float t1z1 = physx::intrinsics::fsel(deltaz1, t2z_candidate1, t1z_candidate1);
	const float t1z2 = physx::intrinsics::fsel(deltaz2, t2z_candidate2, t1z_candidate2);

	const float t2x0 = physx::intrinsics::fsel(deltax0, t1x_candidate0, t2x_candidate0);
	const float t2x1 = physx::intrinsics::fsel(deltax1, t1x_candidate1, t2x_candidate1);
	const float t2x2 = physx::intrinsics::fsel(deltax2, t1x_candidate2, t2x_candidate2);

	const float t2y0 = physx::intrinsics::fsel(deltay0, t1y_candidate0, t2y_candidate0);
	const float t2y1 = physx::intrinsics::fsel(deltay1, t1y_candidate1, t2y_candidate1);
	const float t2y2 = physx::intrinsics::fsel(deltay2, t1y_candidate2, t2y_candidate2);

	const float t2z0 = physx::intrinsics::fsel(deltaz0, t1z_candidate0, t2z_candidate0);
	const float t2z1 = physx::intrinsics::fsel(deltaz1, t1z_candidate1, t2z_candidate1);
	const float t2z2 = physx::intrinsics::fsel(deltaz2, t1z_candidate2, t2z_candidate2);

	const float bxf0 = physx::intrinsics::fsel(deltax0, 3.0f, 0.0f);
	const float bxf1 = physx::intrinsics::fsel(deltax1, 3.0f, 0.0f);
	const float bxf2 = physx::intrinsics::fsel(deltax2, 3.0f, 0.0f);

	const float byf0 = physx::intrinsics::fsel(deltay0, 4.0f, 1.0f);
	const float byf1 = physx::intrinsics::fsel(deltay1, 4.0f, 1.0f);
	const float byf2 = physx::intrinsics::fsel(deltay2, 4.0f, 1.0f);

	const float bzf0 = physx::intrinsics::fsel(deltaz0, 5.0f, 2.0f);
	const float bzf1 = physx::intrinsics::fsel(deltaz1, 5.0f, 2.0f);
	const float bzf2 = physx::intrinsics::fsel(deltaz2, 5.0f, 2.0f);

	float tnear0, tfar0;
	float tnear1, tfar1;
	float tnear2, tfar2;

	tnear0 = t1x0;
	tnear1 = t1x1;
	tnear2 = t1x2;

	tfar0 = t2x0;
	tfar1 = t2x1;
	tfar2 = t2x2;

	float ret0 = bxf0;
	float ret1 = bxf1;
	float ret2 = bxf2;

	const float delta10 = t1y0 - tnear0;
	const float delta11 = t1y1 - tnear1;
	const float delta12 = t1y2 - tnear2;

	tnear0 = physx::intrinsics::fsel(delta10, t1y0, tnear0);
	tnear1 = physx::intrinsics::fsel(delta11, t1y1, tnear1);
	tnear2 = physx::intrinsics::fsel(delta12, t1y2, tnear2);

	ret0 = physx::intrinsics::fsel(delta10, byf0, ret0);
	ret1 = physx::intrinsics::fsel(delta11, byf1, ret1);
	ret2 = physx::intrinsics::fsel(delta12, byf2, ret2);

	tfar0 = physx::intrinsics::selectMin(tfar0, t2y0);
	tfar1 = physx::intrinsics::selectMin(tfar1, t2y1);
	tfar2 = physx::intrinsics::selectMin(tfar2, t2y2);

	const float delta20 = t1z0 - tnear0;
	const float delta21 = t1z1 - tnear1;
	const float delta22 = t1z2 - tnear2;

	tnear0 = physx::intrinsics::fsel(delta20, t1z0, tnear0);
	tnear1 = physx::intrinsics::fsel(delta21, t1z1, tnear1);
	tnear2 = physx::intrinsics::fsel(delta22, t1z2, tnear2);

	ret0 = physx::intrinsics::fsel(delta20, bzf0, ret0);
	ret1 = physx::intrinsics::fsel(delta21, bzf1, ret1);
	ret2 = physx::intrinsics::fsel(delta22, bzf2, ret2);

	tfar0 = physx::intrinsics::selectMin(tfar0, t2z0);
	tfar1 = physx::intrinsics::selectMin(tfar1, t2z1);
	tfar2 = physx::intrinsics::selectMin(tfar2, t2z2);

	// PT: this fcmp seems cheaper than the alternative LHS below
	if(tnear0<0.0f || tnear0>tfar0 || tfar0<LOCAL_EPSILON_RAY_BOX)
	{
		ret0 = -1;
	}
	if(tnear1<0.0f || tnear1>tfar1 || tfar1<LOCAL_EPSILON_RAY_BOX)
	{
		ret1 = -1;
	}
	if(tnear2<0.0f || tnear2>tfar2 || tfar2<LOCAL_EPSILON_RAY_BOX)
	{
		ret2 = -1;
	}

	float ret = ret0;
	tnear = tnear0;
	hit = 0;

	if(ret1!=-1 && tnear1<tnear0)
	{
		ret = ret1;
		tnear = tnear1;
		hit = 1;
	}
	if(ret2!=-1 && tnear2<tnear0)
	{
		ret = ret2;
		tnear = tnear2;
		hit = 2;
	}

//	ret = physx::intrinsics::fsel(tfar - tnear, ret, -1.0f);
//	ret = physx::intrinsics::fsel(LOCAL_EPSILON_RAY_BOX - tfar, -1.0f, ret);

	return int(ret);
}

#endif


// PT: force-inlining this saved 500.000 cycles in the benchmark. Ok to inline, only used once anyway.
static PX_FORCE_INLINE bool intersectEdgeEdge3(const Gu::Plane& plane, const PxVec3& p1, const PxVec3& p2, const PxVec3& dir, const PxVec3& v1,
						const PxVec3& p3, const PxVec3& p4,
						PxReal& dist, PxVec3& ip, PxU32 i, PxU32 j, const PxReal coeff)
{
	// if colliding edge (p3,p4) does not cross plane return no collision
	// same as if p3 and p4 on same side of plane return 0
	//
	// Derivation:
	// d3 = d(p3, P) = (p3 | plane.n) - plane.d;		Reversed sign compared to Plane::Distance() because plane.d is negated.
	// d4 = d(p4, P) = (p4 | plane.n) - plane.d;		Reversed sign compared to Plane::Distance() because plane.d is negated.
	// if d3 and d4 have the same sign, they're on the same side of the plane => no collision
	// We test both sides at the same time by only testing Sign(d3 * d4).
	// ### put that in the Plane class
	// ### also check that code in the triangle class that might be similar
	const PxReal d3 = plane.distance(p3);

	const PxReal temp = d3 * plane.distance(p4);
	if(temp>0.0f)	return false;
//const float cndt0 = physx::intrinsics::fsel(temp, 0.0f, 1.0f);

	// if colliding edge (p3,p4) and plane are parallel return no collision
	const PxVec3 v2 = p4 - p3;

	const PxReal temp2 = plane.normal.dot(v2);
	if(temp2==0.0f)	return false;	// ### epsilon would be better

	// compute intersection point of plane and colliding edge (p3,p4)
	ip = p3-v2*(d3/temp2);

	// compute distance of intersection from line (ip, -dir) to line (p1,p2)
	dist =	(v1[i]*(ip[j]-p1[j])-v1[j]*(ip[i]-p1[i])) * coeff;
	if(dist<0.0f)	return false;
//const float cndt1 = physx::intrinsics::fsel(dist, 1.0f, 0.0f);

	// compute intersection point on edge (p1,p2) line
	ip -= dist*dir;

	// check if intersection point (ip) is between edge (p1,p2) vertices
	const PxReal temp3 = (p1.x-ip.x)*(p2.x-ip.x)+(p1.y-ip.y)*(p2.y-ip.y)+(p1.z-ip.z)*(p2.z-ip.z);
//	if(temp3<0.0f)	return true;	// collision found
//	return false;	// no collision
	return temp3<0.0f;

/*const float cndt2 = physx::intrinsics::fsel(temp3, 0.0f, 1.0f);
	const int ret = int(cndt0 * cndt1 * cndt2);
	return ret!=0;*/
}

// ### stamps
// ### replace fat tris with epsilon stuff
// Do closest tris first ?
// Use support vertices to cull entire tris?
static bool SweepBoxTriangle(const Gu::Triangle& tri, const Gu::Triangle* PX_RESTRICT edge_triangle, PxU32 edge_flags,
					 const PxBounds3& box, const PxVec3* PX_RESTRICT box_vertices,
#ifdef PRECOMPUTE_FAT_BOX
#ifdef PRECOMPUTE_FAT_BOX_MORE
					 const FatEdgeData* PX_RESTRICT fatEdges,
#else
					 const PxVec3* PX_RESTRICT fatEdges,
#endif
#endif
					 const PxVec3& motion, const PxVec3& oneOverMotion,
					 PxVec3& hit, PxVec3& normal, PxReal& d)
{
	// Create triangle normal
	PxVec3 TriNormal;
	tri.denormalizedNormal(TriNormal);

	// Backface culling
	const bool DoCulling = (edge_flags & Gu::TriangleCollisionFlag::eDOUBLE_SIDED)==0;
	if(DoCulling && (TriNormal.dot(motion)) >= 0.0f)	return false;		// ">=" is important !

	//	TriNormal.Normalize();

	// Make fat triangle
	const Gu::Triangle FatTri = InflateTriangle(tri, gFatTriangleCoeff);

	PxReal MinDist = d;	// Initialize with current best distance
	int col = -1;

	const PxVec3 negMotion = -motion;
	const PxVec3 negInvMotion = -oneOverMotion;

	if(1)
	{
		// ### cull using box-plane distance ?

		const PxVec3 Edge1 = FatTri.verts[1] - FatTri.verts[0];
		const PxVec3 Edge2 = FatTri.verts[2] - FatTri.verts[0];
		const PxVec3 PVec = motion.cross(Edge2);
		const PxReal Det = Edge1.dot(PVec);

		// Box vertices VS triangle
		// We can't use stamps here since we can still find a better TOI for a given vertex,
		// even if that vertex has already been tested successfully against another triangle.
		const PxVec3* VN = (const PxVec3*)Gu::getBoxVertexNormals();

		const PxReal OneOverDet = Det!=0.0f ? 1.0f / Det : 0.0f;

		PxU32 hitIndex;
		if(DoCulling)
		{
			if(Det>=LOCAL_EPSILON)
			{
				for(PxU32 i=0;i<8;i++)
				{
					// Orientation culling
					if((VN[i].dot(TriNormal) >= 0.0f))	// Can't rely on triangle normal for double-sided faces
						continue;

					// ### test this
					// ### ok, this causes the bug in level3's v-shaped desk. Not really a real "bug", it just happens
					// that this VF test fixes this case, so it's a bad idea to cull it. Oh, well.
					// If we use a penetration-depth code to fixup bad cases, we can enable this culling again. (also
					// if we find a better way to handle that desk)
					// Discard back vertices
//					if(VN[i].dot(motion)<0.0f)
//						continue;

					// Shoot a ray from vertex against triangle, in direction "motion"
					PxReal t;
					if(!RayTriPrecaCull(box_vertices[i], motion, FatTri.verts[0], Edge1, Edge2, PVec, Det, OneOverDet, t))
						continue;

					//if(t<=OffsetLength)	t=0.0f;
					// Only consider positive distances, closer than current best
					// ### we could test that first on tri vertices & discard complete tri if it's further than current best (or equal!)
					if(t < 0.0f || t >= MinDist)
						continue;

					MinDist = t;
					col = 0;
//					hit = box_vertices[i] + t * motion;
					hitIndex = i;
				}
			}
		}
		else
		{
			if(Det<=-LOCAL_EPSILON || Det>=LOCAL_EPSILON)
			{
				for(PxU32 i=0;i<8;i++)
				{
					// ### test this
					// ### ok, this causes the bug in level3's v-shaped desk. Not really a real "bug", it just happens
					// that this VF test fixes this case, so it's a bad idea to cull it. Oh, well.
					// If we use a penetration-depth code to fixup bad cases, we can enable this culling again. (also
					// if we find a better way to handle that desk)
					// Discard back vertices
					//			if(!VN[i].SameDirection(motion))
					//				continue;

					// Shoot a ray from vertex against triangle, in direction "motion"
					PxReal t;
					if(!RayTriPrecaNoCull(box_vertices[i], motion, FatTri.verts[0], Edge1, Edge2, PVec, Det, OneOverDet, t))
						continue;

					//if(t<=OffsetLength)	t=0.0f;
					// Only consider positive distances, closer than current best
					// ### we could test that first on tri vertices & discard complete tri if it's further than current best (or equal!)
					if(t < 0.0f || t >= MinDist)
						continue;

					MinDist = t;
					col = 0;
//					hit = box_vertices[i] + t * motion;
					hitIndex = i;
				}
			}
		}

		// Only copy this once, if needed
		if(col==0)
		{
			hit = box_vertices[hitIndex] + MinDist * motion;
			normal = TriNormal;
#ifndef LAZY_NORMALIZE
			normal.normalize();
#endif
		}
	}

	if(1)
	{
		// PT: precompute fabs-test for ray-box
		// - doing this outside of the ray-box function gets rid of 3 fabs/fcmp per call
		// - doing this with integer code removes the 3 remaining fabs/fcmps totally
		// - doing this outside reduces the LHS
#ifdef _XBOX
		const float epsilon = LOCAL_EPSILON_RAY_BOX;
		const int fabsx = (int&)(negMotion.x) & 0x7fffffff;
		const int fabsy = (int&)(negMotion.y) & 0x7fffffff;
		const int fabsz = (int&)(negMotion.z) & 0x7fffffff;
		const int intEps = (int&)epsilon;
		const bool b0 = fabsx<intEps;
		const bool b1 = fabsy<intEps;
		const bool b2 = fabsz<intEps;
#else
		const bool b0 = physx::intrinsics::abs(negMotion.x)<LOCAL_EPSILON_RAY_BOX;
		const bool b1 = physx::intrinsics::abs(negMotion.y)<LOCAL_EPSILON_RAY_BOX;
		const bool b2 = physx::intrinsics::abs(negMotion.z)<LOCAL_EPSILON_RAY_BOX;
#endif

		// ### have this as a param ?
		const PxVec3& Min = box.minimum;
		const PxVec3& Max = box.maximum;

		const PxVec3* BoxNormals = (const PxVec3*)gNearPlaneNormal;

		// Triangle vertices VS box
		// ### use stamps not to shoot shared vertices multiple times
		// ### discard non-convex verts
		for(PxU32 i=0;i<3;i++)
		{
			PxReal tnear, tfar;
			const int plane = intersectRayAABB2(Min, Max, tri.verts[i], negMotion, negInvMotion, tnear, tfar, b0, b1, b2);

			// The following works as well but we need to call "intersectRayAABB" to get a plane index compatible with BoxNormals.
			// We could fix this by unifying the plane indices returned by the different ray-aabb functions...
			//PxVec3 coord;
			//PxReal t;
			//PxU32 status = Gu::rayAABBIntersect2(Min, Max, tri.verts[i], -motion, coord, t);

			// ### don't test -1 ?
			if(plane==-1 || tnear<0.0f)	continue;
//				if(tnear<0.0f)	continue;

			if(tnear < MinDist)
			{
				MinDist = tnear;	// ### warning, tnear => flips normals
				normal = BoxNormals[plane];
				col = 1;

				hit = tri.verts[i];
			}
		}
/*
		int h;
		PxReal tnear;
		const int plane = intersectRayAABB3(Min, Max, tri, negMotion, negInvMotion, tnear, h, b0, b1, b2);

		if(plane!=-1 && tnear < MinDist)
		{
			MinDist = tnear;	// ### warning, tnear => flips normals
			normal = BoxNormals[plane];
			col = 1;

			hit = tri.verts[h];
		}*/

	}


	if(1)	// Responsible for blocking character in "corners" + jittering when walking on boxes
	{
#ifndef PRECOMPUTE_FAT_BOX_MORE2
		const PxVec3* EdgeNormals = Gu::getBoxLocalEdgeNormals();
#endif

#ifndef PRECOMPUTE_FAT_BOX
		const PxU8* PX_RESTRICT Edges = Gu::getBoxEdges();
#endif
//			PxVec3 vec0, vec1;

		// Precompute edges
/*			PxVec3 triEdges[3];
		for(PxU32 j=0; j<3; j++)
		{
			PxU32 k = j+1;
			if(k==3)	k=0;
			triEdges[j] = tri.verts[j] - tri.verts[k];
		}*/

		// Loop through box edges
		for(PxU32 i=0;i<12;i++)	// 12 edges
		{
			// Makes unwrap scene fail
//				if((EdgeNormals[i]|motion)<-INVSQRT2)	continue;

			// Catch current box edge // ### one vertex already known using line-strips

#ifndef PRECOMPUTE_FAT_BOX
			// Make it fat ###
			PxVec3 p1 = box_vertices[*Edges++];
			PxVec3 p2 = box_vertices[*Edges++];
#endif

			// ### let this one *after* *Edges++ !
#ifdef PRECOMPUTE_FAT_BOX_MORE
			if(fatEdges[i].edgeNormalDpBin < 0)
				continue;
#else
			if(!(EdgeNormals[i].dot(motion) >= 0.0f)) continue;
#endif

#ifdef PRECOMPUTE_FAT_BOX
#ifdef PRECOMPUTE_FAT_BOX_MORE
			PxVec3 p1 = fatEdges[i].p1;
			PxVec3 p2 = fatEdges[i].p2;
#else
			PxVec3 p1 = fatEdges[i*2];
			PxVec3 p2 = fatEdges[i*2+1];
#endif
#else
			Ps::makeFatEdge(p1, p2, gFatBoxEdgeCoeff);
#endif
//				const PxVec3 p2_p1 = p2 - p1;

#ifndef PRECOMPUTE_FAT_BOX_MORE
			// Precompute data for EE tests
			const PxVec3 v1 = p2 - p1;

			// Build plane P based on edge (p1, p2) and direction (dir)
			Gu::Plane plane;
			plane.normal = v1^motion;
			//	plane.normal.normalize();
			plane.d = -(plane.normal|p1);

			// find largest 2D plane projection
			PxU32 ii,jj;
			//	Ps::closestAxis(plane.normal, ii, jj);
			closestAxis2(plane.normal, ii, jj);

			const PxReal coeff = 1.0f / (v1[ii]*motion[jj] - v1[jj]*motion[ii]);
#endif

			// Loop through triangle edges
			for(PxU32 j=0; j<3; j++)
			{
				// Discard non-convex edges
//					if(!edge_flags.mRef[j])	continue;
				if(!(edge_flags & (1<<j)))	continue;

//					if(EdgeNormals1[j].SameDirection(motion))	continue;
				if(edge_triangle && !edge_triangle->verts[j].isZero() && (edge_triangle->verts[j].dot(motion) >= 0.0f))	continue;

				// Catch current triangle edge
				// j=0 => 0-1
				// j=1 => 1-2
				// j=2 => 2-0
				// => this is compatible with EdgeList
				PxU32 k = j+1;
				if(k==3)	k=0;

				const PxVec3& p3 = tri.verts[j];
				const PxVec3& p4 = tri.verts[k];

//					const PxVec3 p3_p4 = p3-p4;

				PxReal Dist;
				PxVec3 ip;

#ifdef PRECOMPUTE_FAT_BOX_MORE
				bool b1 = intersectEdgeEdge3(fatEdges[i].plane, p1, p2, motion, fatEdges[i].v1, p3, p4, Dist, ip, fatEdges[i].ii, fatEdges[i].jj, fatEdges[i].coeff);
#else
				bool b1 = intersectEdgeEdge3(plane, p1, p2, motion, v1, p3, p4, Dist, ip, ii, jj, coeff);
#endif
//					bool b1 = Gu::intersectEdgeEdge(p1, p2, motion, p3, p4, Dist, ip);
//					bool b1 = Gu::intersectEdgeEdgeNEW(p1, p2, motion, p3, p4, Dist, ip);

/*					PxReal Dist2;
				PxVec3 ip2;
				bool b2 = Gu::intersectEdgeEdgeNEW(p1, p2, motion, p3, p4, Dist2, ip2);
				assert(b1==b2);*/

				// PT: for safety checks. Remove that eventually.

//					PxReal Dist;
//					PxVec3 ip;			
//					bool b1 = intersectEdgeEdge2(p3, p4, triEdges[j], negMotion, p1, p2, p2_p1, Dist, ip);

/*					PxReal Dist2;
				PxVec3 ip2;			
				bool b2 = Gu::intersectEdgeEdge(p1, p2, motion, p3, p4, Dist2, ip2);
				assert(b1==b2);*/

/*					PxReal Dist2;
				PxVec3 ip2;
				bool b2 = intersectEdgeEdge2(p3, p4, -motion, p1, p2, Dist2, ip2);
				assert(b1==b2);*/

				if(b1)	// TODO: PT: refactor this change to CCT code
				{
					if(Dist<MinDist)
					{
						// PT: skip the cross product for now
//							normal = (p1_p2)^(p3-p4);
//							vec0 = p2_p1;
//							vec1 = triEdges[j];

/*#ifdef PRECOMPUTE_FAT_BOX_MORE
						vec0 = fatEdges[i].v1;
#else
						vec0 = v1;
#endif
						vec1 = p3-p4;
*/

#ifdef PRECOMPUTE_FAT_BOX_MORE
normal = fatEdges[i].v1.cross(p3-p4);
#else
normal = v1^(p3-p4);
#endif

#ifndef LAZY_NORMALIZE
						normal.normalize();
						if((normal|motion)>0.0f)
							normal = -normal;
#endif
						col = 2;
						MinDist = Dist;

//							hit = ip;
						hit = ip + motion*Dist;	// For v3
					}
				}
			}
		}
//			if(col==2)
//				normal = vec0^vec1;
	}

	if(col==-1)	return false;
	d = MinDist;
	return true;
}

///////////////////////////////////////////

// We have separation if one of those conditions is true:
//     -BoxExt > TriMax (box strictly to the right of the triangle)
//      BoxExt < TriMin (box strictly to the left of the triangle
// <=>  d0 = -BoxExt - TriMax > 0
//      d1 = BoxExt - TriMin < 0
// Hence we have overlap if d0 <= 0 and d1 >= 0
// overlap = (d0<=0.0f && d1>=0.0f)
#ifdef _XBOX
	#define TEST_OVERLAP									\
		const float d0 = -BoxExt - TriMax;					\
		const float d1 = BoxExt - TriMin;					\
		const float cndt0i = physx::intrinsics::fsel(d0, 0.0f, 1.0f);		\
		const float cndt1i = physx::intrinsics::fsel(d1, 1.0f, 0.0f);		\
		const float bIntersect = cndt0i * cndt1i;			\
		bValidMTD *= bIntersect;
#else
	#define TEST_OVERLAP									\
		const float d0 = -BoxExt - TriMax;					\
		const float d1 = BoxExt - TriMin;					\
		const bool bIntersect = (d0<=0.0f && d1>=0.0f);		\
		bValidMTD &= bIntersect;
#endif

// PT: inlining this one is important. Returning floats looks bad but is faster on Xbox.
#ifdef _XBOX
static PX_FORCE_INLINE float TestAxis(	const Gu::Triangle& tri, const PxVec3& extents,
						const PxVec3& dir, const PxVec3& axis,
//						bool& bValidMTD,
						float& bValidMTD,
						float& tfirst, float& tlast)
#else
static PX_FORCE_INLINE int TestAxis(	const Gu::Triangle& tri, const PxVec3& extents,
						const PxVec3& dir, const PxVec3& axis,
						bool& bValidMTD, float& tfirst, float& tlast)
#endif
{
	const float d0t = tri.verts[0].dot(axis);
	const float d1t = tri.verts[1].dot(axis);
	const float d2t = tri.verts[2].dot(axis);

	float TriMin = physx::intrinsics::selectMin(d0t, d1t);
	float TriMax = physx::intrinsics::selectMax(d0t, d1t);
	TriMin = physx::intrinsics::selectMin(TriMin, d2t);
	TriMax = physx::intrinsics::selectMax(TriMax, d2t);

	////////

	const float BoxExt = physx::intrinsics::abs(axis.x)*extents.x + physx::intrinsics::abs(axis.y)*extents.y + physx::intrinsics::abs(axis.z)*extents.z;
	TEST_OVERLAP

	const float v = dir.dot(axis);
	if(physx::intrinsics::abs(v) < 1.0E-6f)
#ifdef _XBOX
//		return float(bIntersect);
		return bIntersect;
#else
		return bIntersect;
#endif
	const float OneOverV = -1.0f / v;

//	float t0 = d0 * OneOverV;
//	float t1 = d1 * OneOverV;
//	if(t0 > t1)	TSwap(t0, t1);
	const float t0_ = d0 * OneOverV;
	const float t1_ = d1 * OneOverV;
	float t0 = physx::intrinsics::selectMin(t0_, t1_);
	float t1 = physx::intrinsics::selectMax(t0_, t1_);

#ifdef _XBOX
	const float cndt0 = physx::intrinsics::fsel(tlast - t0, 1.0f, 0.0f);
	const float cndt1 = physx::intrinsics::fsel(t1 - tfirst, 1.0f, 0.0f);
#else
	if(t0 > tlast)	return false;
	if(t1 < tfirst)	return false;
#endif

//	if(t1 < tlast)	tlast = t1;
	tlast = physx::intrinsics::selectMin(t1, tlast);

//	if(t0 > tfirst)	tfirst = t0;
	tfirst = physx::intrinsics::selectMax(t0, tfirst);

#ifdef _XBOX
//	return int(cndt0*cndt1);
	return cndt0*cndt1;
#else
	return true;
#endif
}

#ifdef _XBOX
static PX_FORCE_INLINE float TestAxis100(	const Gu::Triangle& tri, const PxVec3& extents,
						const PxVec3& dir, float oneOverDir,
						float& bValidMTD,
						float& tfirst, float& tlast)
#else
static PX_FORCE_INLINE int TestAxis100(	const Gu::Triangle& tri, const PxVec3& extents,
						const PxVec3& dir, float oneOverDir,
						bool& bValidMTD, float& tfirst, float& tlast)
#endif
{
	const float d0t = tri.verts[0].x;
	const float d1t = tri.verts[1].x;
	const float d2t = tri.verts[2].x;

	float TriMin = physx::intrinsics::selectMin(d0t, d1t);
	float TriMax = physx::intrinsics::selectMax(d0t, d1t);
	TriMin = physx::intrinsics::selectMin(TriMin, d2t);
	TriMax = physx::intrinsics::selectMax(TriMax, d2t);

	////////

	const float BoxExt = extents.x;
	TEST_OVERLAP

	const float v = dir.x;
	if(physx::intrinsics::abs(v) < 1.0E-6f)
#ifdef _XBOX
//		return float(bIntersect);
		return bIntersect;
#else
		return bIntersect;
#endif
	const float OneOverV = -1.0f * oneOverDir;

//	float t0 = d0 * OneOverV;
//	float t1 = d1 * OneOverV;
//	if(t0 > t1)	TSwap(t0, t1);
	const float t0_ = d0 * OneOverV;
	const float t1_ = d1 * OneOverV;
	float t0 = physx::intrinsics::selectMin(t0_, t1_);
	float t1 = physx::intrinsics::selectMax(t0_, t1_);

#ifdef _XBOX
	const float cndt0 = physx::intrinsics::fsel(tlast - t0, 1.0f, 0.0f);
	const float cndt1 = physx::intrinsics::fsel(t1 - tfirst, 1.0f, 0.0f);
#else
	if(t0 > tlast)	return false;
	if(t1 < tfirst)	return false;
#endif

//	if(t1 < tlast)	tlast = t1;
	tlast = physx::intrinsics::selectMin(t1, tlast);

//	if(t0 > tfirst)	tfirst = t0;
	tfirst = physx::intrinsics::selectMax(t0, tfirst);

#ifdef _XBOX
	return cndt0*cndt1;
//	return int(cndt0*cndt1);
#else
	return true;
#endif
}

#ifdef _XBOX
static PX_FORCE_INLINE float TestAxis010(	const Gu::Triangle& tri, const PxVec3& extents,
						const PxVec3& dir, float oneOverDir,
						float& bValidMTD,
						float& tfirst, float& tlast)
#else
static PX_FORCE_INLINE int TestAxis010(	const Gu::Triangle& tri, const PxVec3& extents,
						const PxVec3& dir, float oneOverDir,
						bool& bValidMTD, float& tfirst, float& tlast)
#endif
{
	const float d0t = tri.verts[0].y;
	const float d1t = tri.verts[1].y;
	const float d2t = tri.verts[2].y;

	float TriMin = physx::intrinsics::selectMin(d0t, d1t);
	float TriMax = physx::intrinsics::selectMax(d0t, d1t);
	TriMin = physx::intrinsics::selectMin(TriMin, d2t);
	TriMax = physx::intrinsics::selectMax(TriMax, d2t);

	////////

	const float BoxExt = extents.y;
	TEST_OVERLAP

	const float v = dir.y;
	if(physx::intrinsics::abs(v) < 1.0E-6f)
#ifdef _XBOX
//		return float(bIntersect);
		return bIntersect;
#else
		return bIntersect;
#endif
	const float OneOverV = -1.0f * oneOverDir;

//	float t0 = d0 * OneOverV;
//	float t1 = d1 * OneOverV;
//	if(t0 > t1)	TSwap(t0, t1);
	const float t0_ = d0 * OneOverV;
	const float t1_ = d1 * OneOverV;
	float t0 = physx::intrinsics::selectMin(t0_, t1_);
	float t1 = physx::intrinsics::selectMax(t0_, t1_);

#ifdef _XBOX
	const float cndt0 = physx::intrinsics::fsel(tlast - t0, 1.0f, 0.0f);
	const float cndt1 = physx::intrinsics::fsel(t1 - tfirst, 1.0f, 0.0f);
#else
	if(t0 > tlast)	return false;
	if(t1 < tfirst)	return false;
#endif

//	if(t1 < tlast)	tlast = t1;
	tlast = physx::intrinsics::selectMin(t1, tlast);

//	if(t0 > tfirst)	tfirst = t0;
	tfirst = physx::intrinsics::selectMax(t0, tfirst);

#ifdef _XBOX
	return cndt0*cndt1;
//	return int(cndt0*cndt1);
#else
	return true;
#endif
}

#ifdef _XBOX
static PX_FORCE_INLINE float TestAxis001(	const Gu::Triangle& tri, const PxVec3& extents,
						const PxVec3& dir, float oneOverDir,
						float& bValidMTD,
						float& tfirst, float& tlast)
#else
static PX_FORCE_INLINE int TestAxis001(	const Gu::Triangle& tri, const PxVec3& extents,
						const PxVec3& dir, float oneOverDir,
						bool& bValidMTD, float& tfirst, float& tlast)
#endif
{
	const float d0t = tri.verts[0].z;
	const float d1t = tri.verts[1].z;
	const float d2t = tri.verts[2].z;

	float TriMin = physx::intrinsics::selectMin(d0t, d1t);
	float TriMax = physx::intrinsics::selectMax(d0t, d1t);
	TriMin = physx::intrinsics::selectMin(TriMin, d2t);
	TriMax = physx::intrinsics::selectMax(TriMax, d2t);

	////////

	const float BoxExt = extents.z;
	TEST_OVERLAP

	const float v = dir.z;
	if(physx::intrinsics::abs(v) < 1.0E-6f)
#ifdef _XBOX
//		return float(bIntersect);
		return bIntersect;
#else
		return bIntersect;
#endif
	const float OneOverV = -1.0f * oneOverDir;

//	float t0 = d0 * OneOverV;
//	float t1 = d1 * OneOverV;
//	if(t0 > t1)	TSwap(t0, t1);
	const float t0_ = d0 * OneOverV;
	const float t1_ = d1 * OneOverV;
	float t0 = physx::intrinsics::selectMin(t0_, t1_);
	float t1 = physx::intrinsics::selectMax(t0_, t1_);

#ifdef _XBOX
	const float cndt0 = physx::intrinsics::fsel(tlast - t0, 1.0f, 0.0f);
	const float cndt1 = physx::intrinsics::fsel(t1 - tfirst, 1.0f, 0.0f);
#else
	if(t0 > tlast)	return false;
	if(t1 < tfirst)	return false;
#endif

//	if(t1 < tlast)	tlast = t1;
	tlast = physx::intrinsics::selectMin(t1, tlast);

//	if(t0 > tfirst)	tfirst = t0;
	tfirst = physx::intrinsics::selectMax(t0, tfirst);

#ifdef _XBOX
	return cndt0*cndt1;
//	return int(cndt0*cndt1);
#else
	return true;
#endif
}

static PX_FORCE_INLINE int TestSeparationAxes(	const Gu::Triangle& tri, const PxVec3& extents,
								const PxVec3& normal, const PxVec3& dir, const PxVec3& oneOverDir, float tmax, float& tcoll)
{
#ifdef _XBOX
	float bValidMTD = 1.0f;
#else
	bool bValidMTD = true;
#endif
	tcoll = tmax;
	float tfirst = -FLT_MAX;
	float tlast  = FLT_MAX;

	// Triangle normal
#ifdef _XBOX
	if(TestAxis(tri, extents, dir, normal, bValidMTD, tfirst, tlast)==0.0f)
#else
	if(!TestAxis(tri, extents, dir, normal, bValidMTD, tfirst, tlast))
#endif
		return 0;

	// Box normals
#ifdef _XBOX
	if(TestAxis100(tri, extents, dir, oneOverDir.x, bValidMTD, tfirst, tlast)==0.0f)
		return 0;
	if(TestAxis010(tri, extents, dir, oneOverDir.y, bValidMTD, tfirst, tlast)==0.0f)
		return 0;
	if(TestAxis001(tri, extents, dir, oneOverDir.z, bValidMTD, tfirst, tlast)==0.0f)
		return 0;
#else
	if(!TestAxis100(tri, extents, dir, oneOverDir.x, bValidMTD, tfirst, tlast))
		return 0;
	if(!TestAxis010(tri, extents, dir, oneOverDir.y, bValidMTD, tfirst, tlast))
		return 0;
	if(!TestAxis001(tri, extents, dir, oneOverDir.z, bValidMTD, tfirst, tlast))
		return 0;
#endif
	// Edges
	for(PxU32 i=0; i<3; i++)
	{
		int ip1 = i+1;
		if(i>=2)	ip1 = 0;
		const PxVec3 TriEdge = tri.verts[ip1] - tri.verts[i];

#ifdef _XBOX
		{
			const PxVec3 Sep = Ps::cross100(TriEdge);
			if((Sep.dot(Sep))>=1.0E-6f && TestAxis(tri, extents, dir, Sep, bValidMTD, tfirst, tlast)==0.0f)
				return 0;
		}
		{
			const PxVec3 Sep = Ps::cross010(TriEdge);
			if((Sep.dot(Sep))>=1.0E-6f && TestAxis(tri, extents, dir, Sep, bValidMTD, tfirst, tlast)==0.0f)
				return 0;
		}
		{
			const PxVec3 Sep = Ps::cross001(TriEdge);
			if((Sep.dot(Sep))>=1.0E-6f && TestAxis(tri, extents, dir, Sep, bValidMTD, tfirst, tlast)==0.0f)
				return 0;
		}
#else
		{
			const PxVec3 Sep = Ps::cross100(TriEdge);
			if((Sep.dot(Sep))>=1.0E-6f && !TestAxis(tri, extents, dir, Sep, bValidMTD, tfirst, tlast))
				return 0;
		}
		{
			const PxVec3 Sep = Ps::cross010(TriEdge);
			if((Sep.dot(Sep))>=1.0E-6f && !TestAxis(tri, extents, dir, Sep, bValidMTD, tfirst, tlast))
				return 0;
		}
		{
			const PxVec3 Sep = Ps::cross001(TriEdge);
			if((Sep.dot(Sep))>=1.0E-6f && !TestAxis(tri, extents, dir, Sep, bValidMTD, tfirst, tlast))
				return 0;
		}
#endif
	}

	if(tfirst > tmax || tlast < 0.0f)	return 0;
	if(tfirst <= 0.0f)
	{
#ifdef _XBOX
		if(bValidMTD==0.0f)	return 0;
#else
		if(!bValidMTD)	return 0;
#endif
		tcoll = 0.0f;
	}
	else tcoll = tfirst;

	return 1;
}

// PT: SAT-based version, in box space
static PX_FORCE_INLINE int TriBoxSweepTestBoxSpace(const Gu::Triangle& tri, const PxVec3& extents,
									 const PxVec3& dir, const PxVec3& oneOverDir, float tmax, float& toi)
{
	// Create triangle normal
	PxVec3 TriNormal;
	tri.denormalizedNormal(TriNormal);

	// Backface culling
	const bool DoCulling = true;//(edge_flags & Gu::TriangleCollisionFlag::eDOUBLE_SIDED)==0;
	if(DoCulling && (TriNormal.dot(dir)) >= 0.0f)	return 0;		// ">=" is important !

	return TestSeparationAxes(tri, extents, TriNormal, dir, oneOverDir, tmax, toi);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// functions for geometry type based jump tables:

// PT: TODO: the extra indirections in those functions is really useless. We should handle this the way we
// handled the original collision-map for contact generation

/////// CAPSULE/SPHERE SWEEP ///////
static bool sweepCapsule_sphere(const PxGeometry& sphereGeom, const PxTransform& pose, const Gu::Capsule& worldCapsule, const PxVec3& unitDir, const PxReal distance,
						PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	PX_ASSERT(sphereGeom.getType() == PxGeometryType::eSPHERE);
	return sweepCapsule(static_cast<const PxSphereGeometry&>(sphereGeom), pose, worldCapsule, unitDir, distance, sweepHit, hintFlags);
}

static bool sweepCapsule_plane(const PxGeometry& planeGeom, const PxTransform& pose, const Gu::Capsule& worldCapsule, const PxVec3& unitDir, const PxReal distance,
						PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	PX_ASSERT(planeGeom.getType() == PxGeometryType::ePLANE);
	return sweepCapsule(static_cast<const PxPlaneGeometry&>(planeGeom), pose, worldCapsule, unitDir, distance, sweepHit, hintFlags);
}

static bool sweepCapsule_capsule(const PxGeometry& capsuleGeom, const PxTransform& pose, const Gu::Capsule& worldCapsule, const PxVec3& unitDir, const PxReal distance,
							PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	PX_ASSERT(capsuleGeom.getType() == PxGeometryType::eCAPSULE);
	return sweepCapsule(static_cast<const PxCapsuleGeometry&>(capsuleGeom), pose, worldCapsule, unitDir, distance, sweepHit, hintFlags);
}

static bool sweepCapsule_box(const PxGeometry& boxGeom, const PxTransform& pose, const Gu::Capsule& worldCapsule, const PxVec3& unitDir, const PxReal distance,
						PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	PX_ASSERT(boxGeom.getType() == PxGeometryType::eBOX);
	return sweepCapsule(static_cast<const PxBoxGeometry&>(boxGeom), pose, worldCapsule, unitDir, distance, sweepHit, hintFlags);
}

static bool sweepCapsule_convexMesh(const PxGeometry& convexMeshGeom, const PxTransform& pose, const Gu::Capsule& worldCapsule, const PxVec3& unitDir, const PxReal distance, 
								PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	PX_ASSERT(convexMeshGeom.getType() == PxGeometryType::eCONVEXMESH);
	return sweepCapsule(static_cast<const PxConvexMeshGeometry&>(convexMeshGeom), pose, worldCapsule, unitDir, distance, sweepHit, hintFlags);
}

static bool sweepCapsule_triangleMesh(const PxGeometry& triangleMeshGeom, const PxTransform& pose, const Gu::Capsule& worldCapsule, const PxVec3& unitDir, const PxReal distance,
								PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	PX_ASSERT(triangleMeshGeom.getType() == PxGeometryType::eTRIANGLEMESH);
	return sweepCapsule(static_cast<const PxTriangleMeshGeometry&>(triangleMeshGeom), pose, worldCapsule, unitDir, distance, sweepHit, hintFlags);
}

static bool sweepCapsule_heightField(const PxGeometry& heightFieldGeom, const PxTransform& pose, const Gu::Capsule& worldCapsule, const PxVec3& unitDir, const PxReal distance, 
								PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	PX_ASSERT(heightFieldGeom.getType() == PxGeometryType::eHEIGHTFIELD);
	return sweepCapsule(static_cast<const PxHeightFieldGeometry&>(heightFieldGeom), pose, worldCapsule, unitDir, distance, sweepHit, hintFlags);
}

/////// BOX SWEEP ///////
static bool sweepBox_sphere(const PxGeometry& sphereGeom, const PxTransform& pose, const Gu::Box& worldBox, const PxVec3& unitDir, const PxReal distance,
						PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	PX_ASSERT(sphereGeom.getType() == PxGeometryType::eSPHERE);
	return sweepBox(static_cast<const PxSphereGeometry&>(sphereGeom), pose, worldBox, unitDir, distance, sweepHit, hintFlags);
}

static bool sweepBox_plane(const PxGeometry& planeGeom, const PxTransform& pose, const Gu::Box& worldBox, const PxVec3& unitDir, const PxReal distance,
						PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	PX_ASSERT(planeGeom.getType() == PxGeometryType::ePLANE);
	return sweepBox(static_cast<const PxPlaneGeometry&>(planeGeom), pose, worldBox, unitDir, distance, sweepHit, hintFlags);
}

static bool sweepBox_capsule(const PxGeometry& capsuleGeom, const PxTransform& pose, const Gu::Box& worldBox, const PxVec3& unitDir, const PxReal distance,
							PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	PX_ASSERT(capsuleGeom.getType() == PxGeometryType::eCAPSULE);
	return sweepBox(static_cast<const PxCapsuleGeometry&>(capsuleGeom), pose, worldBox, unitDir, distance, sweepHit, hintFlags);
}

static bool sweepBox_box(const PxGeometry& boxGeom, const PxTransform& pose, const Gu::Box& worldBox, const PxVec3& unitDir, const PxReal distance,
						PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	PX_ASSERT(boxGeom.getType() == PxGeometryType::eBOX);
	return sweepBox(static_cast<const PxBoxGeometry&>(boxGeom), pose, worldBox, unitDir, distance, sweepHit, hintFlags);
}

static bool sweepBox_convexMesh(const PxGeometry& convexMeshGeom, const PxTransform& pose, const Gu::Box& worldBox, const PxVec3& unitDir, const PxReal distance, 
								PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	PX_ASSERT(convexMeshGeom.getType() == PxGeometryType::eCONVEXMESH);
	return sweepBox(static_cast<const PxConvexMeshGeometry&>(convexMeshGeom), pose, worldBox, unitDir, distance, sweepHit, hintFlags);
}

static bool sweepBox_triangleMesh(const PxGeometry& triangleMeshGeom, const PxTransform& pose, const Gu::Box& worldBox, const PxVec3& unitDir, const PxReal distance,
								PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	PX_ASSERT(triangleMeshGeom.getType() == PxGeometryType::eTRIANGLEMESH);
	return sweepBox(static_cast<const PxTriangleMeshGeometry&>(triangleMeshGeom), pose, worldBox, unitDir, distance, sweepHit, hintFlags);
}

static bool sweepBox_heightField(const PxGeometry& heightFieldGeom, const PxTransform& pose, const Gu::Box& worldBox, const PxVec3& unitDir, const PxReal distance, 
								PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	PX_ASSERT(heightFieldGeom.getType() == PxGeometryType::eHEIGHTFIELD);
	return sweepBox(static_cast<const PxHeightFieldGeometry&>(heightFieldGeom), pose, worldBox, unitDir, distance, sweepHit, hintFlags);
}

static bool sweepConvexVsShape(const PxConvexMeshGeometry& convexGeom, const PxTransform& pose,
							   const GJKConvexInterface& convexShape, const PxTransform& shapePose,
							   const PxVec3& unitDir, const PxReal distance, PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	const Gu::ConvexMesh& convexMesh = *static_cast<Gu::ConvexMesh*>(convexGeom.convexMesh);
	
	Gu::GJKConvexSupport convexSupport(convexMesh, convexGeom.scale);

	PxReal toi;
	PxVec3 destNormal;
	PxVec3 destWorldPointA;

	sweepHit.faceIndex = 0;	//TODO: we need to return hit params including polygon feature!!!  To do that we'd have to extend our support query to look at trigs, not just vertices.

	bool ok = convexConvexLinearSweep(
		convexSupport, convexShape,
		pose, pose.p,
		shapePose, shapePose.p + (unitDir * distance),
		gGJKEpsilon,
		destNormal, destWorldPointA, toi);

	if (ok)
	{
		sweepHit.flags = PxSceneQueryFlag::eDISTANCE | PxSceneQueryFlag::eIMPACT | PxSceneQueryFlag::eNORMAL;

//		if(hintFlags & PxSceneQueryFlag::eINITIAL_OVERLAP)
		{
			// PT: test if shapes initially overlap
			if(toi<=0.0f)
			{
				sweepHit.distance	= 0.0f;
				sweepHit.normal		= -unitDir;
				sweepHit.impact		= destWorldPointA;	// PT: this is arbitrary
				return true;
			}
		}

		sweepHit.distance = distance * toi;	//hope this has same scale
		sweepHit.normal = destNormal;
		sweepHit.normal.normalize();
		sweepHit.impact = destWorldPointA;
	}

	return ok;
}

/////////////////////////////////////////////////  sweepCapsule/Sphere  //////////////////////////////////////////////////////

bool Gu::sweepCapsule(const PxSphereGeometry& sphereGeom, const PxTransform& pose, const Gu::Capsule& lss, const PxVec3& unitDir, const PxReal distance, PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	const Gu::Sphere sphere(pose.p, sphereGeom.radius);

	if(!SweepSphereCapsule(sphere, lss, -unitDir, distance, sweepHit.distance, sweepHit.impact, sweepHit.normal, hintFlags))
		return false;

	sweepHit.flags = PxSceneQueryFlag::eDISTANCE | PxSceneQueryFlag::eIMPACT | PxSceneQueryFlag::eNORMAL;

	return true;
}

bool Gu::sweepCapsule(const PxPlaneGeometry& planeGeom, const PxTransform& pose, const Gu::Capsule& lss, const PxVec3& unitDir, const PxReal distance, PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	const Gu::Plane& worldPlane = Gu::getPlane(pose);

	PxU32 index = 0;
	PxVec3 pts[2];

	if (lss.p0 == lss.p1)
	{
		// We have a sphere
		pts[0] = lss.p0;
	}
	else
	{
		// Find extreme point on the capsule
		pts[0] = lss.p0;
		pts[1] = lss.p1;
		PxReal minDp = PX_MAX_REAL;
		for(PxU32 i=0; i<2; i++)
		{
			const PxReal dp = pts[i].dot(worldPlane.normal);
			if(dp<minDp)
			{
				minDp = dp;
				index = i;
			}
		}
	}

	const PxVec3 ptOnCapsule = pts[index] - worldPlane.normal*lss.radius;

	// Raycast extreme vertex against plane
	bool hitPlane = Gu::intersectRayPlane(ptOnCapsule, unitDir, worldPlane, sweepHit.distance, &sweepHit.impact);
	if(hitPlane && sweepHit.distance > 0 && sweepHit.distance < distance)
	{
		sweepHit.normal = worldPlane.normal;
		sweepHit.flags = PxSceneQueryFlag::eDISTANCE | PxSceneQueryFlag::eIMPACT | PxSceneQueryFlag::eNORMAL;
		return true;
	}
	return false;
}

bool Gu::sweepCapsule(const PxCapsuleGeometry& capsuleGeom, const PxTransform& pose, const Gu::Capsule& lss, const PxVec3& unitDir, const PxReal distance, PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	Gu::Capsule staticCapsule;
	Gu::getWorldCapsule(staticCapsule, capsuleGeom, pose);

	PxU32 outFlags;
	if(SweepCapsuleCapsule(lss, staticCapsule, -unitDir, distance, sweepHit.distance, sweepHit.impact, sweepHit.normal, hintFlags, outFlags))
	{
		sweepHit.flags = PxSceneQueryFlags(outFlags);
		return true;
	}

	return false;
}

bool Gu::sweepCapsule(const PxBoxGeometry& boxGeom, const PxTransform& pose, const Gu::Capsule& lss, const PxVec3& unitDir, const PxReal distance, PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	if (lss.p0 == lss.p1)  // The capsule is actually a sphere
	{
		//TODO: Check if this is really faster than using a "sphere-aware" version of SweepCapsuleBox

		Gu::Box box;	buildFrom(box, pose.p, boxGeom.halfExtents, pose.q);
		if(!SweepBoxSphere(box, lss.radius, lss.p0, unitDir, distance, sweepHit.distance, sweepHit.normal, hintFlags))
			return false;

		sweepHit.normal = -sweepHit.normal;
		sweepHit.flags = PxSceneQueryFlag::eDISTANCE | PxSceneQueryFlag::eNORMAL;
		
		if(hintFlags & PxSceneQueryFlag::eIMPACT)
		{
			// The sweep test doesn't compute the impact point automatically, so we have to do it here.
			const PxVec3 NewSphereCenter = lss.p0 + unitDir * sweepHit.distance;
			PxVec3 Closest;
			const PxReal d = Gu::distancePointBoxSquared(NewSphereCenter, box.center, box.extents, box.rot, &Closest);
			// Compute point on the box, after sweep
			Closest = box.rotate(Closest);
			sweepHit.impact = Closest + box.center;
			sweepHit.flags |= PxSceneQueryFlag::eIMPACT;
		}
	}
	else
	{
		if(!SweepCapsuleBox(lss, pose, boxGeom.halfExtents, unitDir, distance, sweepHit.impact, sweepHit.distance, sweepHit.normal, hintFlags))
			return false;

		sweepHit.flags = PxSceneQueryFlag::eDISTANCE | PxSceneQueryFlag::eNORMAL;
		
		if(hintFlags & PxSceneQueryFlag::eIMPACT)
		{
			// The sweep test doesn't compute the impact point automatically, so we have to do it here.
			Gu::Capsule movedCaps = lss;
			movedCaps.p0 += unitDir * sweepHit.distance;
			movedCaps.p1 += unitDir * sweepHit.distance;
			Gu::Box box;	buildFrom(box, pose.p, boxGeom.halfExtents, pose.q);
			PxVec3 closest;
			const PxReal d = Gu::distanceSegmentBoxSquared(movedCaps, box, NULL, &closest);
			// Compute point on the box, after sweep
			closest = pose.q.rotate(closest);
			sweepHit.impact = closest + pose.p;
			sweepHit.flags |= PxSceneQueryFlag::eIMPACT;
		}
	}

	return true;
}

bool Gu::sweepCapsule(const PxConvexMeshGeometry& convexGeom, const PxTransform& pose, const Gu::Capsule& lss, const PxVec3& unitDir, const PxReal distance, PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	PxReal capsuleHalfHeight = 0.0f;
	const PxTransform capsuleTransform = getWorldTransform(lss, capsuleHalfHeight);
	GJKCapsuleSupport capsuleSupport(capsuleHalfHeight, lss.radius);

	return sweepConvexVsShape(convexGeom, pose, capsuleSupport, capsuleTransform, unitDir, distance, sweepHit, hintFlags);
}

bool Gu::sweepCapsule(const PxTriangleMeshGeometry& triMeshGeom, const PxTransform& pose, const Gu::Capsule& lss, const PxVec3& unitDir, const PxReal distance, 
					PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	// optimize this
	// use the obb sweep collider directly ?
	const Gu::TriangleMesh& tm = *static_cast<Gu::TriangleMesh*>(triMeshGeom.triangleMesh);

	const Ice::HybridModel& collisionModel = tm.getOpcodeModel();

	Ice::HybridOBBCollider boxCollider;
	boxCollider.SetPrimitiveTests(true);	// ############

	// Compute swept box
	Box capsuleBox;
	computeBoxAroundCapsule(lss, capsuleBox);

	Box sweptBox;
	computeSweptBox(capsuleBox.extents, capsuleBox.center, capsuleBox.rot, unitDir, distance, sweptBox);

	const Cm::Matrix34 vertexToWorldSkew = pose * triMeshGeom.scale;

	Gu::Box vertexSpaceBox;
	computeVertexSpaceOBB(vertexSpaceBox, sweptBox, pose, triMeshGeom.scale);

	// Collide OBB against current mesh
	HybridModelData hmd;	// PT: I suppose doing the "conversion" at runtime is fine
	collisionModel.getHybridModelData(hmd);

//	Ice::Container tempContainer;	// PT: TODO: get rid of dynamic allocations in that one
	LocalContainer(tempContainer, 128);
	VolumeColliderContainerCallback callback(tempContainer);
	boxCollider.Collide(vertexSpaceBox, hmd, &callback, NULL, NULL);

	// Get results
	PxU32 nb = tempContainer.GetNbEntries();
	if(!nb)
		return false;
	const PxU32* PX_RESTRICT indices = tempContainer.GetEntries();

	Triangle* PX_RESTRICT tmpt = (Triangle*)PX_ALLOC_TEMP(sizeof(Triangle)*nb);
	for(PxU32 i=0; i<nb; i++)
	{
		const PxU32 triangleIndex = *indices++;
		::getScaledTriangle(triMeshGeom, vertexToWorldSkew, tmpt[i], triangleIndex);	// ### move to local space...
	}

	bool status = sweepCapsuleTriangles(nb, tmpt, lss, unitDir, distance, sweepHit.distance, sweepHit.normal, sweepHit.impact, sweepHit.faceIndex, hintFlags);
	if(status)
	{
		const PxU32* indices = tempContainer.GetEntries();
		sweepHit.faceIndex = indices[sweepHit.faceIndex];
		sweepHit.flags = PxSceneQueryFlag::eDISTANCE | PxSceneQueryFlag::eIMPACT | PxSceneQueryFlag::eNORMAL;
	}
	PX_FREE_AND_RESET(tmpt);
	return status;
}

bool Gu::sweepCapsule(const PxHeightFieldGeometry& heightFieldGeom, const PxTransform& pose, const Gu::Capsule& lss, const PxVec3& unitDir, const PxReal distance, 
					PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	// Compute swept box
	Gu::Box capsuleBox;
	Gu::computeBoxAroundCapsule(lss, capsuleBox);

	Gu::Box sweptBox;
	computeSweptBox(capsuleBox.extents, capsuleBox.center, capsuleBox.rot, unitDir, distance, sweptBox);

	//### Temp hack until we can directly collide the OBB against the HF
	const PxTransform sweptBoxTR = sweptBox.getTransform();
	const PxBounds3 bounds = PxBounds3::poseExtent(sweptBoxTR, sweptBox.extents);

	const PxU32 flags = PxQueryFlags::eWORLD_SPACE;

	struct LocalReport : Gu::EntityReport<PxU32>
	{
		LocalReport()
			: tmpTris(PX_DEBUG_EXP("localReportTmpTris")),
			tmpIndices(PX_DEBUG_EXP("localReportTmpIndices"))
		{

		}
		virtual bool onEvent(PxU32 nb, PxU32* indices)
		{
			for(PxU32 i=0; i<nb; i++)
			{
				PxU32 triangleIndex = indices[i];

				Gu::Triangle tmpTri;
				hfUtil->getTriangle(*pose, tmpTri, NULL, NULL, NULL, triangleIndex, true);
				tmpTris.pushBack(tmpTri);
				tmpIndices.pushBack(triangleIndex);
			}

			return true;
		}

		Ps::Array<Gu::Triangle> tmpTris;
		Ps::Array<PxU32> tmpIndices;
		const PxTransform* pose;
		Gu::HeightFieldUtil* hfUtil;
	} myReport;

	Gu::HeightFieldUtil hfUtil(heightFieldGeom);

	//dynamic allocs are bad, but this code was already doing so before...
	myReport.tmpTris.reserve(128);
	myReport.tmpIndices.reserve(128);
	myReport.pose = &pose;
	myReport.hfUtil = &hfUtil;

	hfUtil.overlapAABBTriangles(pose, bounds, flags, &myReport);

	if(myReport.tmpTris.size() <= 0)
		return false;

	bool status = sweepCapsuleTriangles(myReport.tmpTris.size(), &(myReport.tmpTris[0]), lss, unitDir, distance, sweepHit.distance, sweepHit.normal, sweepHit.impact, sweepHit.faceIndex, hintFlags);
	if(status)
	{
		sweepHit.faceIndex = myReport.tmpIndices[sweepHit.faceIndex];
		sweepHit.flags = PxSceneQueryFlag::eDISTANCE | PxSceneQueryFlag::eIMPACT | PxSceneQueryFlag::eNORMAL;
	}

	return status;
}

/////////////////////////////////////////////////  sweepBox  //////////////////////////////////////////////////////

bool Gu::sweepBox(const PxSphereGeometry& sphereGeom, const PxTransform& pose_, const Gu::Box& box_, const PxVec3& unitDir, const PxReal distance, PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	// PT: move to relative space
	const Gu::Box box(box_.center - pose_.p, box_.extents, box_.rot);

	if(!SweepBoxSphere(box, sphereGeom.radius, PxVec3(0), -unitDir, distance, sweepHit.distance, sweepHit.normal, hintFlags))
		return false;

	sweepHit.flags = PxSceneQueryFlag::eDISTANCE | PxSceneQueryFlag::eNORMAL;

	if(hintFlags & PxSceneQueryFlag::eIMPACT)
	{
		// The sweep test doesn't compute the impact point automatically, so we have to do it here.
		const PxVec3 motion = sweepHit.distance * unitDir;
		const PxVec3 newSphereCenter = - motion;
		PxVec3 closest;
		const PxReal d = Gu::distancePointBoxSquared(newSphereCenter, box.center, box.extents, box.rot, &closest);
		// Compute point on the box, after sweep
		sweepHit.impact = box.rotate(closest) + box_.center + motion;	// PT: undo move to local space here
		sweepHit.flags |= PxSceneQueryFlag::eIMPACT;
	}
	return true;
}

bool Gu::sweepBox(const PxPlaneGeometry& planeGeom, const PxTransform& pose, const Gu::Box& box, const PxVec3& unitDir, const PxReal distance, PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	const Gu::Plane& worldPlane = Gu::getPlane(pose);

	// Find extreme point on the box
	PxVec3 boxPts[8];
	box.computeBoxPoints(boxPts);
	PxU32 index = 0;
	PxReal minDp = PX_MAX_REAL;
	for(PxU32 i=0;i<8;i++)
	{
		PxReal dp = boxPts[i].dot(worldPlane.normal);
		if(dp<minDp)
		{
			minDp = dp;
			index = i;
		}
	}

	// Raycast extreme vertex against plane
	bool hitPlane = Gu::intersectRayPlane(boxPts[index], unitDir, worldPlane, sweepHit.distance, &sweepHit.impact);
	if(hitPlane && sweepHit.distance > 0 && sweepHit.distance < distance)
	{
		sweepHit.normal = worldPlane.normal;
		sweepHit.flags = PxSceneQueryFlag::eDISTANCE | PxSceneQueryFlag::eIMPACT | PxSceneQueryFlag::eNORMAL;
		return true;
	}
	return false;
}

bool Gu::sweepBox(const PxCapsuleGeometry& capsuleGeom, const PxTransform& pose_, const Gu::Box& box_, const PxVec3& unitDir, const PxReal distance, PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	// PT: move to relative space
	const PxTransform pose(PxVec3(0), pose_.q);
	Gu::Box box(box_.center - pose_.p, box_.extents, box_.rot);

	PxVec3 n;
	const PxVec3 negDir = -unitDir;

	Gu::Capsule lss;
	Gu::getWorldCapsule(lss, capsuleGeom, pose);

	const PxTransform boxWorldPose = box.getTransform();

	if(!SweepCapsuleBox(lss, boxWorldPose, box.extents, negDir, distance, sweepHit.impact, sweepHit.distance, n, hintFlags))
		return false;

	sweepHit.normal = -n;
	sweepHit.flags = PxSceneQueryFlag::eDISTANCE | PxSceneQueryFlag::eNORMAL;

	if(hintFlags & PxSceneQueryFlag::eIMPACT)
	{
		// The sweep test doesn't compute the impact point automatically, so we have to do it here.
		box.center += (unitDir * sweepHit.distance);
		PxVec3 closest;
		const PxReal d = Gu::distanceSegmentBoxSquared(lss, box, NULL, &closest);
		// Compute point on the box, after sweep
		sweepHit.impact = box.transform(closest) + pose_.p;	// PT: undo move to local space here
		sweepHit.flags |= PxSceneQueryFlag::eIMPACT;
	}
	return true;
}

bool Gu::sweepBox(const PxBoxGeometry& boxGeom, const PxTransform& pose_, const Gu::Box& box_, const PxVec3& unitDir, const PxReal distance, PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	// PT: move to local space
	const Gu::Box box(box_.center - pose_.p, box_.extents, box_.rot);
	Gu::Box staticBox;	buildFrom(staticBox, PxVec3(0), boxGeom.halfExtents, pose_.q);

	if(SweepBoxBox(box, staticBox, unitDir, distance, sweepHit.impact, sweepHit.normal, sweepHit.distance, hintFlags))
	{
		sweepHit.impact += pose_.p;	// PT: undo move to local space
		sweepHit.flags = PxSceneQueryFlag::eDISTANCE | PxSceneQueryFlag::eIMPACT | PxSceneQueryFlag::eNORMAL;
		return true;
	}
	return false;
}

bool Gu::sweepBox(const PxConvexMeshGeometry& convexGeom, const PxTransform& pose, const Gu::Box& box, const PxVec3& unitDir, const PxReal distance, PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	const PxTransform boxTransform = box.getTransform();
	GJKBoxSupport boxSupport(box.extents);

	return sweepConvexVsShape(convexGeom, pose, boxSupport, boxTransform, unitDir, distance, sweepHit, hintFlags);
}

// PT: those two are not working well yet
//#define SORT_TRIS
//#define BEST_TRI_FIRST
#define PREFETCH_TRI

#define USE_SAT_VERSION	// PT: uses SAT-based sweep test instead of feature-based.

#ifdef PREFETCH_TRI
static PX_FORCE_INLINE void prefetchTriangle(const PxTriangleMeshGeometry& triGeom, PxTriangleID triangleIndex)
{
	const Gu::TriangleMesh& tm = *static_cast<Gu::TriangleMesh*>(triGeom.triangleMesh);
	const PxVec3* PX_RESTRICT vertices = tm.getVerticesFast();
	if(tm.mesh.has16BitIndices())
	{
		const Gu::TriangleT<PxU16>& T = ((const Gu::TriangleT<PxU16>*)tm.getTrianglesFast())[triangleIndex];
		Ps::prefetch128(vertices + T.v[0]);
		Ps::prefetch128(vertices + T.v[1]);
		Ps::prefetch128(vertices + T.v[2]);
	}
	else
	{
		const Gu::TriangleT<PxU32>& T = ((const Gu::TriangleT<PxU32>*)tm.getTrianglesFast())[triangleIndex];
		Ps::prefetch128(vertices + T.v[0]);
		Ps::prefetch128(vertices + T.v[1]);
		Ps::prefetch128(vertices + T.v[2]);
	}
}
#endif

// PT: not inlining this rarely-run function makes the benchmark ~500.000 cycles faster...
// PT: using this version all the time makes the benchmark ~300.000 cycles slower. So we just use it as a backup.
static void runBackupProcedure(PxSweepHit& sweepHit, const PxVec3& localDir, const Gu::Box& box, const Gu::Triangle& currentTriangle, const PxVec3* PX_RESTRICT boxVertices)
{
	// PT: if we hit this, the SAT & feature-based versions disagree
	// So we run a GJK-based backup procedure in this case.
	class MyGJKBox : public GJKConvexInterface
	{
		public:
		MyGJKBox(const PxVec3* v) : mBoxVertices(v)	{}

		virtual void	getBounds(PxBounds3& bounds) const
		{
			PX_ASSERT(0);
		}

		virtual PxVec3	projectHullMax(const PxVec3& localDirArg, GJKConvexInterfaceCache&) const
		{
			PxVec3 localDir = localDirArg.getNormalized();
			PxReal maximum = -FLT_MAX;
			PxU32 candidate;
			for(PxU32 i=0;i<8;i++)
			{
				const PxReal dp = localDir.dot(mBoxVertices[i]);
				if(dp>maximum)
				{
					maximum = dp;
					candidate = i;
				}
			}
			return mBoxVertices[candidate];
		}

		virtual PxVec3 inverseSupportMapping(const PxVec3& pointOnSurface, int& isFace) const
		{
			PX_ASSERT(0 && "Not implemented.");
			return PxVec3(0);
		}

		const PxVec3*	mBoxVertices;
	};

	const PxVec3 delta = localDir*sweepHit.distance - localDir*0.1f;
	const PxVec3 mp0 = currentTriangle.verts[0] - delta;
	const PxVec3 mp1 = currentTriangle.verts[1] - delta;
	const PxVec3 mp2 = currentTriangle.verts[2] - delta;
	const PxVec3 triCenter = (mp0+mp1+mp2)/3.0f;

	MyGJKBox myBox(boxVertices);
	Gu::GJKTriangleSupport myTri(mp0, mp1, mp2);
	PxTransform shape2worldA = PxTransform::createIdentity();
	PxTransform shape2worldB = PxTransform::createIdentity();
	PxVec3 sepAxisGuessInOut = box.center - triCenter;
	PxVec3 destWorldNormalOnB;
	PxVec3 destWorldPointA;
	PxVec3 destWorldPointB;
	PxReal destDistance;
	GJKConvexInterfaceCache cache;

	bool status = convexConvexDistance(
			myTri, myBox,
			shape2worldA, shape2worldB,
			sepAxisGuessInOut,
			destWorldNormalOnB, destWorldPointA, destWorldPointB, destDistance, cache);
	if(status)
	{
		sweepHit.impact = destWorldPointB + localDir*sweepHit.distance;
		sweepHit.normal = destWorldNormalOnB;
	}
	else
	{
		// PT: if the backup procedure fails, we give up
		sweepHit.impact = box.center;
		sweepHit.normal = -localDir;
	}
}

bool Gu::sweepBox(const PxTriangleMeshGeometry& triMeshGeom, const PxTransform& pose, const Gu::Box& box, const PxVec3& unitDir, const PxReal distance, PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	//- optimize this
	//- use the obb sweep collider directly ?
	const Gu::TriangleMesh& tm = *static_cast<Gu::TriangleMesh*>(triMeshGeom.triangleMesh);
	const Ice::HybridModel& collisionModel = tm.getOpcodeModel();

	Ice::HybridOBBCollider boxCollider;
//#ifdef OPC_SUPPORT_VMX128
//	boxCollider.SetPrimitiveTests(true);	// ############
//#else
	boxCollider.SetPrimitiveTests(false);	// PT: this is usually faster
//#endif

	// Compute swept box
	Box sweptBox;
	computeSweptBox(box.extents, box.center, box.rot, unitDir, distance, sweptBox);

	const Cm::Matrix34 vertexToWorldSkew = pose * triMeshGeom.scale;

	Gu::Box vertexSpaceBox;
	computeVertexSpaceOBB(vertexSpaceBox, sweptBox, pose, triMeshGeom.scale);

//	Ice::Container tempContainer;
//	tempContainer.Reserve(256); // AP: scaffold, todo: eliminate dynamic allocations here
//	tempContainer.ForceSize(0);
	LocalContainer(tempContainer, 128);
	VolumeColliderContainerCallback callback(tempContainer);

	// Collide OBB against current mesh
	HybridModelData hmd;	// PT: I suppose doing the "conversion" at runtime is fine
	collisionModel.getHybridModelData(hmd);
	boxCollider.Collide(vertexSpaceBox, hmd, &callback, NULL, NULL, false);

	// Get results
	PxU32 nb = tempContainer.GetNbEntries();
	if(!nb)
		return false;

	const PxU32* PX_RESTRICT indices = tempContainer.GetEntries();

#ifdef SORT_TRIS
	float* keys = (float*)PxAlloca(nb*sizeof(float));
	for(PxU32 i=0;i<nb;i++)
	{
		const PxU32 triangleIndex = indices[i];

		Triangle currentTriangle;	// in world space
		::getScaledTriangle(triMeshGeom, vertexToWorldSkew, currentTriangle, triangleIndex);

		float dp0 = currentTriangle.verts[0]|motion;
		float dp1 = currentTriangle.verts[1]|motion;
		float dp2 = currentTriangle.verts[2]|motion;
		float mindp = physx::intrinsics::selectMin(dp0, dp1);
		mindp = physx::intrinsics::selectMin(mindp, dp2);
		keys[i] = mindp;
	}

	PxU32* ranks0 = (PxU32*)PxAlloca(nb*sizeof(PxU32));
	PxU32* ranks1 = (PxU32*)PxAlloca(nb*sizeof(PxU32));
	StackRadixSort(RS, ranks0, ranks1);
	const PxU32* sorted = RS.Sort(keys, nb).GetRanks();
#endif

#ifdef BEST_TRI_FIRST
	float best = FLT_MAX;
	float bestTri;
	for(PxU32 i=0;i<nb;i++)
	{
		const PxU32 triangleIndex = indices[i];

		Triangle currentTriangle;	// in world space
		::getScaledTriangle(triMeshGeom, vertexToWorldSkew, currentTriangle, triangleIndex);

		const float dp0 = currentTriangle.verts[0]|motion;
		const float dp1 = currentTriangle.verts[1]|motion;
		const float dp2 = currentTriangle.verts[2]|motion;
		float mindp = physx::intrinsics::selectMin(dp0, dp1);
		mindp = physx::intrinsics::selectMin(mindp, dp2);

		const float delta = mindp - best;
		best = physx::intrinsics::fsel(delta, mindp, best);
		bestTri = physx::intrinsics::fsel(delta, float(i), bestTri);
/*		if(mindp<best)
		{
			mindp=best;
			bestTri=i;
		}*/
	}
	int ibest = int(bestTri);
	PxU32* indices_ = const_cast<PxU32*>(indices);
	const PxU32 tmp = indices_[ibest];
	indices_[ibest] = indices_[0];
	indices_[0] = tmp;
#endif

	// Move to AABB space
	Cm::Matrix34 WorldToBox;
	computeWorldToBoxMatrix(WorldToBox, box);

	const PxVec3 localDir = WorldToBox.rotate(unitDir);
	const PxVec3 localMotion = localDir * distance;

#ifndef USE_SAT_VERSION
	PxBounds3 aabb;
	aabb.maximum = box.extents;
	aabb.minimum = -box.extents;
	PxVec3 boxVertices[8];
	Gu::computeBoxPoints(aabb, boxVertices);

#ifdef PRECOMPUTE_FAT_BOX
	#ifdef PRECOMPUTE_FAT_BOX_MORE
	FatEdgeData fatEdges[12];
	computeFatEdges(boxVertices, fatEdges, localDir * distance);
	#else
	PxVec3 fatEdges[24];
	computeFatEdges(boxVertices, fatEdges);
	#endif
#endif
#endif

	bool status = false;
	sweepHit.distance = distance;	//was PX_MAX_F32, but that may trigger an assert in the caller!

	const PxVec3 oneOverMotion(
		localDir.x!=0.0f ? 1.0f/(localDir.x * distance) : 0.0f,
		localDir.y!=0.0f ? 1.0f/(localDir.y * distance) : 0.0f,
		localDir.z!=0.0f ? 1.0f/(localDir.z * distance) : 0.0f);

	const PxU32 edgeFlags =	Gu::TriangleCollisionFlag::eACTIVE_EDGE01 | 
							Gu::TriangleCollisionFlag::eACTIVE_EDGE12 |
							Gu::TriangleCollisionFlag::eACTIVE_EDGE20;

// PT: experimental code, don't clean up before I test it more and validate it

// Project box
/*float boxRadius0 =
			PxAbs(dir.x) * box.extents.x
		+	PxAbs(dir.y) * box.extents.y
		+	PxAbs(dir.z) * box.extents.z;*/

float boxRadius =
			PxAbs(localDir.x) * box.extents.x
		+	PxAbs(localDir.y) * box.extents.y
		+	PxAbs(localDir.z) * box.extents.z;

if(0)	// PT: run this to check the box radius is correctly computed
{
	PxVec3 boxVertices2[8];
	box.computeBoxPoints(boxVertices2);
	float dpmin = FLT_MAX;
	float dpmax = -FLT_MAX;
	for(int i=0;i<8;i++)
	{
		const float dp = boxVertices2[i].dot(unitDir);
		if(dp<dpmin)	dpmin = dp;
		if(dp>dpmax)	dpmax = dp;
	}
	const float goodRadius = (dpmax-dpmin)/2.0f;
	PX_UNUSED(goodRadius);
}

const float dpc0 = box.center.dot(unitDir);
float localMinDist = 1.0f;
#ifdef _DEBUG
	PxU32 totalTestsExpected = nb;
	PxU32 totalTestsReal = 0;
	PX_UNUSED(totalTestsExpected);
	PX_UNUSED(totalTestsReal);
#endif

//	const bool has16BitIndices = tm.mesh.has16BitIndices();

	Triangle currentTriangle;	// in world space
#ifdef PREFETCH_TRI
//	PxU32 counter = 0;
#endif
	while(nb--)
	{
#ifdef PREFETCH_TRI
/*		if(!counter)
		{
			counter=8;

			PxU32 nbToGo = nb+1;
			if(nbToGo>counter)	nbToGo=counter;

			for(PxU32 np=0;np<nbToGo;np++)
				prefetchTriangle(triMeshGeom, indices[np]);
		}*/
#endif

#ifdef SORT_TRIS
		const PxU32 triangleIndex = indices[*sorted++];
#else
		const PxU32 triangleIndex = *indices++;
#endif
#ifdef PREFETCH_TRI
		if(nb)
			prefetchTriangle(triMeshGeom, *indices);
#endif

		// ### try prefetching here
		::getScaledTriangle(triMeshGeom, vertexToWorldSkew, currentTriangle, triangleIndex); // ### move to local space...
//		::getScaledTriangle(tm, vertexToWorldSkew, currentTriangle, triangleIndex, has16BitIndices);

#ifdef _XBOX
		if(CullTriangle(currentTriangle, unitDir, boxRadius, localMinDist*distance, dpc0)==0.0f)
			continue;
#else
		if(!CullTriangle(currentTriangle, unitDir, boxRadius, localMinDist*distance, dpc0))
			continue;
#endif

#ifdef _DEBUG
		totalTestsReal++;
#endif
		// Move to box space
		currentTriangle.verts[0] = WorldToBox.transform(currentTriangle.verts[0]);
		currentTriangle.verts[1] = WorldToBox.transform(currentTriangle.verts[1]);
		currentTriangle.verts[2] = WorldToBox.transform(currentTriangle.verts[2]);

#ifdef USE_SAT_VERSION
		PxF32 t = PX_MAX_F32;		// could be better!
		if(TriBoxSweepTestBoxSpace(currentTriangle, box.extents, localMotion, oneOverMotion, localMinDist, t))
		{
			if(t < localMinDist)
			{
//				if(hintFlags & PxSceneQueryFlag::eINITIAL_OVERLAP)
				{
					// PT: test if shapes initially overlap
					if(t==0.0f)
					{
						sweepHit.flags		= PxSceneQueryFlag::eDISTANCE|PxSceneQueryFlag::eNORMAL|PxSceneQueryFlag::eIMPACT;
						sweepHit.distance	= 0.0f;
						sweepHit.faceIndex	= triangleIndex;
						sweepHit.impact		= box.center;	// PT: this is arbitrary
						sweepHit.normal		= -unitDir;
						return true;
					}
				}

				localMinDist			= t;
				sweepHit.distance		= t * distance;
				sweepHit.faceIndex		= triangleIndex;
				status					= true;
			}
		}
#else
		PxVec3 normal;
		PxVec3 ip;

		float dd = localMinDist;	// Initialize with current best distance

		if(SweepBoxTriangle(currentTriangle, NULL, edgeFlags, aabb, boxVertices,
#ifdef PRECOMPUTE_FAT_BOX
			fatEdges,
#endif
			localMotion, oneOverMotion, ip, normal, dd))
		{
			if(dd < localMinDist)
			{
				localMinDist			= dd;
				sweepHit.distance		= dd * distance;
				sweepHit.normal			= normal;
				sweepHit.impact			= ip;
				sweepHit.faceIndex		= triangleIndex;
				status					= true;
			}
		}
#endif
	}


	if(status)
	{
		sweepHit.flags = PxSceneQueryFlag::eDISTANCE;

#ifdef USE_SAT_VERSION
		if(hintFlags & (PxSceneQueryFlag::eNORMAL|PxSceneQueryFlag::eIMPACT))
		{
			// PT: compute impact point/normal in a second pass. Here we simply re-sweep the box against the best triangle,
			// using the feature-based code (which computes impact point and normal). This is not great because:
			// - we know there's an impact so why do all tests again?
			// - the SAT test & the feature-based tests could return different results because of FPU accuracy.
			//   The assert below captures this, but it won't do anything good the day we find a bad case.
			// It would be more robust to move the box to impact position, then compute the closest points between the box and the triangle.
			// But we don't have a ready-to-use box-triangle distance routine, and using GJK just for this seems overkill.
			PxBounds3 aabb;
			aabb.maximum = box.extents;
			aabb.minimum = -box.extents;
			PxVec3 boxVertices[8];
			Gu::computeBoxPoints(aabb, boxVertices);

#ifdef PRECOMPUTE_FAT_BOX
	#ifdef PRECOMPUTE_FAT_BOX_MORE
			FatEdgeData fatEdges[12];
			computeFatEdges(boxVertices, fatEdges, localMotion);
	#else
			PxVec3 fatEdges[24];
			computeFatEdges(boxVertices, fatEdges);
	#endif
#endif
			Triangle currentTriangle;	// in world space
			::getScaledTriangle(triMeshGeom, vertexToWorldSkew, currentTriangle, sweepHit.faceIndex); // ### move to local space...

			// Move to box space
			currentTriangle.verts[0] = WorldToBox.transform(currentTriangle.verts[0]);
			currentTriangle.verts[1] = WorldToBox.transform(currentTriangle.verts[1]);
			currentTriangle.verts[2] = WorldToBox.transform(currentTriangle.verts[2]);

			float t = PX_MAX_F32;
			bool ret = SweepBoxTriangle(currentTriangle, NULL, edgeFlags, aabb, boxVertices,
			#ifdef PRECOMPUTE_FAT_BOX
				fatEdges,
			#endif
				localMotion, oneOverMotion, sweepHit.impact, sweepHit.normal, t);
			//AP: scaffold, the backup procedure itself fails and asserts preventing TestSuiteSdk from running
			if(!ret)
				runBackupProcedure(sweepHit, localDir, box, currentTriangle, boxVertices);
		}
#endif

		if(hintFlags & PxSceneQueryFlag::eNORMAL)
		{
#ifdef LAZY_NORMALIZE
			sweepHit.normal.normalize();
			if((sweepHit.normal.dot(localMotion))>0.0f)
				sweepHit.normal = -sweepHit.normal;
#endif
			sweepHit.normal = box.rotate(sweepHit.normal);
			sweepHit.flags |= PxSceneQueryFlag::eNORMAL;
		}
		if(hintFlags & PxSceneQueryFlag::eIMPACT)
		{
			sweepHit.impact = box.rotate(sweepHit.impact) + box.center;
			sweepHit.flags |= PxSceneQueryFlag::eIMPACT;
		}
	}
	return status;
}

bool Gu::sweepBox(const PxHeightFieldGeometry& heightFieldGeom, const PxTransform& pose, const Gu::Box& box, const PxVec3& unitDir, const PxReal distance, PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	// Compute swept box
	Gu::Box sweptBox;
	computeSweptBox(box.extents, box.center, box.rot, unitDir, distance, sweptBox);

	//### Temp hack until we can directly collide the OBB against the HF
	const PxTransform sweptBoxTR = sweptBox.getTransform();
	const PxBounds3 bounds = PxBounds3::poseExtent(sweptBoxTR, sweptBox.extents);

	const PxU32 flags = PxQueryFlags::eWORLD_SPACE;

	// Move to AABB space
	const PxTransform BoxToWorld = box.getTransform();
	const PxTransform WorldToBox = BoxToWorld.getInverse();

	const PxVec3 motion = unitDir * distance;
	const PxVec3 localMotion = WorldToBox.rotate(motion);

	PxBounds3 aabb;
	aabb.maximum = box.extents;
	aabb.minimum = -box.extents;
	PxVec3 boxVertices[8];
	Gu::computeBoxPoints(aabb, boxVertices);
#ifdef PRECOMPUTE_FAT_BOX
	#ifdef PRECOMPUTE_FAT_BOX_MORE
	FatEdgeData fatEdges[12];
	computeFatEdges(boxVertices, fatEdges, motion);
	#else
	PxVec3 fatEdges[24];
	computeFatEdges(boxVertices, fatEdges);
	#endif
#endif

	sweepHit.distance = PX_MAX_F32;

	struct LocalReport : Gu::EntityReport<PxU32>
	{
		virtual bool onEvent(PxU32 nb, PxU32* indices)
		{
			const PxU32 edgeFlags =	Gu::TriangleCollisionFlag::eACTIVE_EDGE01 | 
									Gu::TriangleCollisionFlag::eACTIVE_EDGE12 |
									Gu::TriangleCollisionFlag::eACTIVE_EDGE20;

			for(PxU32 i=0; i<nb; i++)
			{
				PxU32 triangleIndex = indices[i];

				Gu::Triangle currentTriangle;	// in world space
				hfUtil->getTriangle(*pose, currentTriangle, NULL, NULL, NULL, triangleIndex, true, true);
			
				// Move to box space
				currentTriangle.verts[0] = WorldToBox.transform(currentTriangle.verts[0]);
				currentTriangle.verts[1] = WorldToBox.transform(currentTriangle.verts[1]);
				currentTriangle.verts[2] = WorldToBox.transform(currentTriangle.verts[2]);

				PxF32 t = PX_MAX_F32;
				PxVec3 normal;
				PxVec3 ip;

				if(mInitialOverlapTests)
				{
					// PT: test if shapes initially overlap
					if(Gu::intersectTriangleBox(PxVec3(0), aabb.maximum, currentTriangle.verts[0], currentTriangle.verts[1], currentTriangle.verts[2]))
					{
						mInitialOverlap	= true;
						hit->faceIndex	= triangleIndex;
						return true;
					}
				}

				// PT: TODO: use the SAT version here!!!
				if(SweepBoxTriangle(currentTriangle, NULL, edgeFlags, aabb, boxVertices,
#ifdef PRECOMPUTE_FAT_BOX
				   fatEdges,
#endif
					localMotion, oneOverMotion, ip, normal, t))
				{
					if(t < hit->distance)
					{
						hit->distance		= t;
						hit->normal			= normal;
						hit->impact			= ip;
						hit->faceIndex		= triangleIndex;
						status				= true;
					}
				}
			}
#ifdef LAZY_NORMALIZE
			if(status)
			{
				hit->normal.normalize();
				if((hit->normal.dot(localMotion))>0.0f)
					hit->normal = -hit->normal;
			}
#endif
			return true;
		}

		PxTransform WorldToBox;
		const PxTransform* pose;
		Gu::HeightFieldUtil* hfUtil;
		PxBounds3 aabb;
		PxVec3* boxVertices;
#ifdef PRECOMPUTE_FAT_BOX
	#ifdef PRECOMPUTE_FAT_BOX_MORE
		FatEdgeData* fatEdges;
	#else
		PxVec3* fatEdges;
	#endif
#endif
		PxVec3 localMotion;
		PxVec3 oneOverMotion;
		PxSweepHit* hit;
		bool status;
		bool mInitialOverlap;
		bool mInitialOverlapTests;
	} myReport;

	Gu::HeightFieldUtil hfUtil(heightFieldGeom);

	myReport.WorldToBox = WorldToBox;
	myReport.status = false;
	myReport.mInitialOverlap = false;
	myReport.mInitialOverlapTests = (hintFlags & PxSceneQueryFlag::eINITIAL_OVERLAP)!=(PxSceneQueryFlags)0;
	myReport.pose = &pose;
	myReport.hfUtil = &hfUtil;
	myReport.aabb = aabb;
	myReport.boxVertices = boxVertices;
#ifdef PRECOMPUTE_FAT_BOX
	myReport.fatEdges = fatEdges;
#endif
	myReport.localMotion = localMotion;
	myReport.hit = &sweepHit;
	myReport.oneOverMotion = PxVec3(
		localMotion.x!=0.0f ? 1.0f/localMotion.x : 0.0f,
		localMotion.y!=0.0f ? 1.0f/localMotion.y : 0.0f,
		localMotion.z!=0.0f ? 1.0f/localMotion.z : 0.0f);

	hfUtil.overlapAABBTriangles(pose, bounds, flags, &myReport);

	if(myReport.mInitialOverlap)
	{
		sweepHit.distance	= 0.0f;
		sweepHit.normal		= -unitDir;
		sweepHit.impact		= box.center;	// PT: this is arbitrary
		sweepHit.flags		= PxSceneQueryFlag::eDISTANCE | PxSceneQueryFlag::eIMPACT | PxSceneQueryFlag::eNORMAL;
		return true;
	}

	if(myReport.status)
	{
		sweepHit.distance *= distance;  // stored as toi [0,1] during computation -> scale
		sweepHit.normal = BoxToWorld.rotate(sweepHit.normal);
		sweepHit.impact = BoxToWorld.transform(sweepHit.impact);
		sweepHit.flags = PxSceneQueryFlag::eDISTANCE | PxSceneQueryFlag::eIMPACT | PxSceneQueryFlag::eNORMAL;
	}
	return myReport.status;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Gu::SweepCapsuleTriangles(PxU32 nb_tris, const Gu::Triangle* triangles, const PxU32* edge_flags,
							const PxCapsuleGeometry& capsuleGeom, const PxTransform& capsulePose,
							const PxVec3& dir, const PxReal length, const PxU32* cachedIndex,
							PxVec3& hit, PxVec3& normal, PxReal& d, PxU32& index)
{
	PxVec3 Hit, Normal;
	PxReal t;

	// We skip the extrusion for the case of the sweep direction matching the capsule axis
	// or
	// if the capsule is in fact a sphere

	PxVec3 capsuleAxis = capsulePose.q.getBasisVector0();
	capsuleAxis.normalize();
	PxReal colinearity = PxAbs(capsuleAxis.dot(dir));
	if ((colinearity < (1.0f - LOCAL_EPSILON)) && (capsuleGeom.halfHeight > 0.0f))
	{
		// Extrusion dir = capsule segment
		const PxVec3 ExtrusionDir = capsuleAxis * capsuleGeom.halfHeight;

		if(1)
		{
			PX_ALLOCA(Extruded, Gu::Triangle, nb_tris*7);
			PX_ALLOCA(Ids, PxU32, nb_tris*7);

			// Compute swept box
			Gu::Box capsuleBox;
			Gu::computeBoxAroundCapsule(capsuleGeom, capsulePose, capsuleBox);

			Gu::Box sweptBounds;
			computeSweptBox(capsuleBox.extents, capsuleBox.center, capsuleBox.rot, dir, length, sweptBounds);

			PxU32 NbExtruded = ExtrudeMesh(nb_tris, triangles, edge_flags, ExtrusionDir, Extruded, Ids, dir, &sweptBounds);

			// The nice thing with this approach is that we "just" fallback to already existing code
			if(!SweepSphereTriangles(NbExtruded, Extruded, capsulePose.p, capsuleGeom.radius, dir, length, false, NULL, Hit, Normal, t, index))
			{
				return false;
			}

			PX_ASSERT(index!=PX_INVALID_U32);
			index = Ids[index];
		}
	}
	else
	{
		const PxVec3 SphereCenter = capsulePose.p + dir * capsuleGeom.halfHeight;

		if(!SweepSphereTriangles(nb_tris, triangles, SphereCenter, capsuleGeom.radius, dir, length, true, cachedIndex, Hit, Normal, t, index))
			return false;

		PX_ASSERT(index!=PX_INVALID_U32);
	}

	d		= t;
	normal	= Normal;
	hit		= Hit;
	return true;
}

#define USE_SAT_VERSION_CCT	// PT: the SAT-based sweep for this function, almost exclusively used by the CCT

bool Gu::SweepBoxTriangles(PxU32 nb_tris, const Gu::Triangle* triangles, const Gu::Triangle* edge_triangles, const PxU32* edge_flags, 
						const PxBoxGeometry& boxGeom, const PxTransform& boxPose, const PxVec3& dir, const PxReal length, PxVec3& _hit,
						PxVec3& _normal, float& _d, PxU32& _index, const PxU32* cachedIndex)
{
	PxU32 idx = 0;
	if(cachedIndex)
		idx = *cachedIndex;

	PxBounds3 boxBounds(-boxGeom.halfExtents, boxGeom.halfExtents);
	PxVec3 boxVertices[8];
	Gu::computeBoxPoints(boxBounds, boxVertices);	// Precompute
#ifdef PRECOMPUTE_FAT_BOX
	#ifdef PRECOMPUTE_FAT_BOX_MORE
	FatEdgeData fatEdges[12];
	computeFatEdges(boxVertices, fatEdges, dir);
	#else
	PxVec3 fatEdges[24];
	computeFatEdges(boxVertices, fatEdges);
	#endif
#endif

	//------- early exit --------
	PxVec3 boxCenter = boxPose.p;

	// Project box
	const float boxRadius =
				PxAbs(dir.x) * boxGeom.halfExtents.x
			+	PxAbs(dir.y) * boxGeom.halfExtents.y
			+	PxAbs(dir.z) * boxGeom.halfExtents.z;

	const float dpc0 = boxCenter.dot(dir);
	//---------------------------

	bool Status = false;
	float localMinDist = length;	// Initialize with current best distance
	PxVec3 localHit;
	PxVec3 localNormal;
	PxU32 localIndex;
#ifdef USE_SAT_VERSION_CCT
	Gu::Triangle savedTri;
#endif

	bool hasRotation = (boxPose.q.x != 0.0f || boxPose.q.y != 0.0f || boxPose.q.z != 0.0f || boxPose.q.w != 1.0f);
	// Not sure it is worth it to avoid the rotation in this cases

	const PxVec3 oneOverDir(
		dir.x!=0.0f ? 1.0f/dir.x : 0.0f,
		dir.y!=0.0f ? 1.0f/dir.y : 0.0f,
		dir.z!=0.0f ? 1.0f/dir.z : 0.0f);

	for(PxU32 ii=0; ii<nb_tris; ii++)
	{
		PxU32 i;
		if(ii==0)
			i = idx;
		else if(ii==idx)
			i = 0;
		else
			i = ii;

#ifdef _XBOX
		if(CullTriangle(triangles[i], dir, boxRadius, localMinDist, dpc0)==0.0f)
			continue;
#else
		if(!CullTriangle(triangles[i], dir, boxRadius, localMinDist, dpc0))
			continue;
#endif

		Gu::Triangle CurrentTri = triangles[i];
		CurrentTri.verts[0] -= boxPose.p;
		CurrentTri.verts[1] -= boxPose.p;
		CurrentTri.verts[2] -= boxPose.p;
		if(hasRotation)
		{
			CurrentTri.verts[0] = boxPose.rotateInv(CurrentTri.verts[0]);
			CurrentTri.verts[1] = boxPose.rotateInv(CurrentTri.verts[1]);
			CurrentTri.verts[2] = boxPose.rotateInv(CurrentTri.verts[2]);
		}

#ifdef USE_SAT_VERSION_CCT
		PxF32 dd = PX_MAX_F32;	// could be better!
		bool b1 = TriBoxSweepTestBoxSpace(CurrentTri, boxGeom.halfExtents, dir, oneOverDir, localMinDist, dd)!=0;
#else
		PxU32 CurrentFlags;
		if (edge_flags)
			CurrentFlags = edge_flags[i];
		else
			CurrentFlags =	Gu::TriangleCollisionFlag::eACTIVE_EDGE01 | 
							Gu::TriangleCollisionFlag::eACTIVE_EDGE12 |
							Gu::TriangleCollisionFlag::eACTIVE_EDGE20;

		Gu::Triangle currentEdgeTriangle;
		const Gu::Triangle* currentEdgeTrianglePtr = NULL;
		if (edge_triangles)
		{
			currentEdgeTriangle = *(edge_triangles + i);
			currentEdgeTrianglePtr = &currentEdgeTriangle;

			if (hasRotation)
			{
				currentEdgeTriangle.verts[0] = boxPose.rotateInv(currentEdgeTriangle.verts[0]);
				currentEdgeTriangle.verts[1] = boxPose.rotateInv(currentEdgeTriangle.verts[1]);
				currentEdgeTriangle.verts[2] = boxPose.rotateInv(currentEdgeTriangle.verts[2]);
			}
		}

		PxVec3 Hit, Normal;
		float dd = localMinDist;	// Initialize with current best distance
		bool b1 = SweepBoxTriangle(CurrentTri, currentEdgeTrianglePtr, CurrentFlags, boxBounds, boxVertices,
	#ifdef PRECOMPUTE_FAT_BOX
			fatEdges,
	#endif
			dir, oneOverDir, Hit, Normal, dd);
#endif
		if(b1)
		{
			if(dd < localMinDist)
			{
				localMinDist	= dd;
#ifdef USE_SAT_VERSION_CCT
				savedTri		= CurrentTri;
#else
				localNormal		= Normal;
				localHit		= Hit;
#endif
				localIndex		= i;
				Status			= true;
			}
		}
	}

	if(Status)
	{
#ifdef USE_SAT_VERSION_CCT
		PxU32 CurrentFlags;
		if (edge_flags)
			CurrentFlags = edge_flags[localIndex];
		else
			CurrentFlags =	Gu::TriangleCollisionFlag::eACTIVE_EDGE01 | 
							Gu::TriangleCollisionFlag::eACTIVE_EDGE12 |
							Gu::TriangleCollisionFlag::eACTIVE_EDGE20;

		Gu::Triangle currentEdgeTriangle;
		const Gu::Triangle* currentEdgeTrianglePtr = NULL;
		if (edge_triangles)
		{
			currentEdgeTriangle = *(edge_triangles + localIndex);
			currentEdgeTrianglePtr = &currentEdgeTriangle;

			if (hasRotation)
			{
				currentEdgeTriangle.verts[0] = boxPose.rotateInv(currentEdgeTriangle.verts[0]);
				currentEdgeTriangle.verts[1] = boxPose.rotateInv(currentEdgeTriangle.verts[1]);
				currentEdgeTriangle.verts[2] = boxPose.rotateInv(currentEdgeTriangle.verts[2]);
			}
		}

		float dd = localMinDist*2.0f;
		bool b = SweepBoxTriangle(savedTri, currentEdgeTrianglePtr, CurrentFlags, boxBounds, boxVertices,
	#ifdef PRECOMPUTE_FAT_BOX
			fatEdges,
	#endif
			dir, oneOverDir, localHit, localNormal, dd);
		if(!b)
		{
			Gu::Box box;	buildFrom(box, boxPose.p, boxGeom.halfExtents, boxPose.q);
			PxSweepHit sweepHit;
			sweepHit.distance = localMinDist;
			runBackupProcedure(sweepHit, dir, box, savedTri, boxVertices);
			localNormal = sweepHit.normal;
			localHit = sweepHit.impact;
		}
#endif

#ifdef LAZY_NORMALIZE
		localNormal.normalize();
		if((localNormal.dot(dir))>0.0f)
			localNormal = -localNormal;
#endif

		_d		= localMinDist;
		_index	= localIndex;
		_normal	= localNormal;
		_hit	= localHit;

		if (hasRotation)
		{
			_normal	= boxPose.rotate(_normal);
			_hit	= boxPose.rotate(_hit);
		}

		_hit += boxPose.p;
	}
	return Status;
}

// and finally the jump tables
// (MUST be in order of PxGeometryType)

const Gu::SweepCapsuleFunc Gu::gSweepCapsuleMap[7] = 
{
	sweepCapsule_sphere,
	sweepCapsule_plane,
	sweepCapsule_capsule,
	sweepCapsule_box,
	sweepCapsule_convexMesh,
	sweepCapsule_triangleMesh,
	sweepCapsule_heightField
};

const Gu::SweepBoxFunc Gu::gSweepBoxMap[7] = 
{
	sweepBox_sphere,
	sweepBox_plane,
	sweepBox_capsule,
	sweepBox_box,
	sweepBox_convexMesh,
	sweepBox_triangleMesh,
	sweepBox_heightField
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

static bool sweepConvex_sphere(const PxGeometry& geom, const PxTransform& pose, const PxConvexMeshGeometry& convexGeom, const PxTransform& convexPose,
							   const PxVec3& unitDir, const PxReal distance, PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eSPHERE);
	const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom);

	const Gu::Capsule capsule(Gu::Segment(pose.p, pose.p), sphereGeom.radius);

	if(sweepCapsule(convexGeom, convexPose, capsule, -unitDir, distance, sweepHit, hintFlags))
	{
		sweepHit.impact += unitDir * sweepHit.distance;
		sweepHit.normal = -sweepHit.normal;
		return true;
	}
	return false;
}

static bool sweepConvex_plane(const PxGeometry& geom, const PxTransform& pose, const PxConvexMeshGeometry& convexGeom, const PxTransform& convexPose,
							   const PxVec3& unitDir, const PxReal distance, PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	PX_ASSERT(geom.getType() == PxGeometryType::ePLANE);
//	const PxPlaneGeometry& planeGeom = static_cast<const PxPlaneGeometry&>(geom);

	const Gu::ConvexMesh& convexMesh = *static_cast<Gu::ConvexMesh*>(convexGeom.convexMesh);

	const PxVec3* PX_RESTRICT hullVertices = convexMesh.getHull().getHullVertices();
	PxU32 numHullVertices = convexMesh.getHull().mNbHullVertices;

	const Cm::FastVertex2ShapeScaling convexScaling(convexGeom.scale);

	const Gu::Plane plane = Gu::getPlane(pose);

	sweepHit.distance	= distance;
	sweepHit.faceIndex	= 0;
	bool status = false;
	while(numHullVertices--)
	{
		const PxVec3& vertex = *hullVertices++;
		const PxVec3 worldPt = convexPose.transform(convexScaling * vertex);
		float t;
		PxVec3 pointOnPlane;
		if(intersectRayPlane(worldPt, unitDir, plane, t, &pointOnPlane))
		{
			if(t<=0.0f)
			{
				// Convex touches plane
				sweepHit.distance	= 0.0f;
				sweepHit.flags		= PxSceneQueryFlag::eDISTANCE | PxSceneQueryFlag::eIMPACT | PxSceneQueryFlag::eNORMAL | PxSceneQueryFlag::eINITIAL_OVERLAP;
				sweepHit.impact		= worldPt;
				sweepHit.normal		= -unitDir;
				return true;
			}

			if(t < sweepHit.distance)
			{
				sweepHit.distance	= t;
				sweepHit.flags		= PxSceneQueryFlag::eDISTANCE | PxSceneQueryFlag::eIMPACT | PxSceneQueryFlag::eNORMAL;
				sweepHit.impact		= pointOnPlane;
				sweepHit.normal		= plane.normal;
				status				= true;
			}
		}
	}
	return status;
}

static bool sweepConvex_capsule(const PxGeometry& geom, const PxTransform& pose, const PxConvexMeshGeometry& convexGeom, const PxTransform& convexPose,
							   const PxVec3& unitDir, const PxReal distance, PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eCAPSULE);
	const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom);

	Gu::Capsule capsule;
	Gu::getWorldCapsule(capsule, capsuleGeom, pose);

	if(sweepCapsule(convexGeom, convexPose, capsule, -unitDir, distance, sweepHit, hintFlags))
	{
		sweepHit.impact += unitDir * sweepHit.distance;
		sweepHit.normal = -sweepHit.normal;
		return true;
	}
	return false;
}

static bool sweepConvex_box(const PxGeometry& geom, const PxTransform& pose, const PxConvexMeshGeometry& convexGeom, const PxTransform& convexPose,
							   const PxVec3& unitDir, const PxReal distance, PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eBOX);
	const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom);

	Gu::Box box;
	buildFrom(box, pose.p, boxGeom.halfExtents, pose.q);

	if(sweepBox(convexGeom, convexPose, box, -unitDir, distance, sweepHit, hintFlags))
	{
		sweepHit.impact += unitDir * sweepHit.distance;
		sweepHit.normal = -sweepHit.normal;
		return true;
	}
	return false;
}

static bool sweepConvex_convexMesh(const PxGeometry& geom, const PxTransform& pose, const PxConvexMeshGeometry& convexGeom, const PxTransform& convexPose,
							   const PxVec3& unitDir, const PxReal distance, PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eCONVEXMESH);
	const PxConvexMeshGeometry& otherConvexGeom = static_cast<const PxConvexMeshGeometry&>(geom);

	const Gu::ConvexMesh& convexMesh = *static_cast<Gu::ConvexMesh*>(otherConvexGeom.convexMesh);
	Gu::GJKConvexSupport convexSupport(convexMesh, otherConvexGeom.scale);

	if(sweepConvexVsShape(convexGeom, convexPose, convexSupport, pose, -unitDir, distance, sweepHit, hintFlags))
	{
		sweepHit.impact += unitDir * sweepHit.distance;
		sweepHit.normal = -sweepHit.normal;
		return true;
	}
	return false;
}

	struct ConvexVsMeshSweepCallback : VolumeColliderTrigCallback
	{
		ConvexVsMeshSweepCallback(	const Gu::ConvexMesh& cm, const PxMeshScale& convexScale, const Cm::FastVertex2ShapeScaling& meshScale,
									const PxTransform& tr0, const PxTransform& tr1,
									const PxVec3& unitDir, const PxReal distance, PxSceneQueryFlags hintFlags) :
				mConvexSupport	(cm, convexScale),
				mMeshScale		(meshScale),
				mTransform0		(tr0),
				mTransform1		(tr1),
				mUnitDir		(unitDir),
				mDistance		(distance),
				mHintFlags		(hintFlags),
				mAnyHit			(false)
		{
			mHit.faceIndex = 0;
			mHit.distance = FLT_MAX;
		}
		virtual ~ConvexVsMeshSweepCallback()	{}

		// PT: TODO: optimize this
		virtual bool processResults(PxU32 count, const PxVec3* verts, const PxU32*)
		{
			while(count--)
			{
				const PxVec3 v0 = mMeshScale * verts[0];
				const PxVec3 v1 = mMeshScale * verts[1];
				const PxVec3 v2 = mMeshScale * verts[2];
				verts += 3;

				Gu::GJKTriangleSupport triangleSupport(v0, v1, v2);

				PxReal toi;
				PxVec3 destNormal;
				PxVec3 destWorldPointA;

				const bool ok = convexConvexLinearSweep(
					mConvexSupport, triangleSupport,
					mTransform0, mTransform0.p,
					mTransform1, mTransform1.p + (mUnitDir * mDistance),
					gGJKEpsilon,
					destNormal, destWorldPointA, toi);

				if(ok)
				{
					mHit.flags = PxSceneQueryFlag::eDISTANCE | PxSceneQueryFlag::eIMPACT | PxSceneQueryFlag::eNORMAL;

					mAnyHit = true;

					if(mHintFlags & PxSceneQueryFlag::eINITIAL_OVERLAP)
					{
						// PT: test if shapes initially overlap
						if(toi<=0.0f)
						{
							mHit.distance	= 0.0f;
							mHit.normal		= -mUnitDir;
							mHit.impact		= destWorldPointA;	// PT: this is arbitrary
							mAnyHit			= true;
							return false;
						}
					}

					const float minDist = toi * mDistance;

					if(minDist<mHit.distance)
					{
						mHit.distance	= minDist;
						mHit.normal		= destNormal;
						mHit.impact		= destWorldPointA;
						mAnyHit			= true;
					}
				}
			}
			return true;
		}
		Gu::GJKConvexSupport				mConvexSupport;
		const Cm::FastVertex2ShapeScaling&	mMeshScale;
		const PxTransform&					mTransform0;
		const PxTransform&					mTransform1;
		PxSweepHit							mHit;
		PxVec3								mUnitDir;
		PxReal								mDistance;
		PxU32								mHintFlags;
		bool								mAnyHit;
	};

static bool sweepConvex_triangleMesh(const PxGeometry& geom, const PxTransform& pose, const PxConvexMeshGeometry& convexGeom, const PxTransform& convexPose,
							   const PxVec3& unitDir, const PxReal distance, PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eTRIANGLEMESH);
	const PxTriangleMeshGeometry& meshGeom = static_cast<const PxTriangleMeshGeometry&>(geom);

	// PT: TODO: this part similar to convex-vs-overlap test, refactor
	const Cm::Matrix34 convexTM(convexPose);
	const Cm::Matrix34 meshTM(pose);

	const Gu::ConvexMesh* cm = static_cast<const Gu::ConvexMesh*>(convexGeom.convexMesh);
	const Gu::TriangleMesh* tm = static_cast<Gu::TriangleMesh*>(meshGeom.triangleMesh);

	const bool idtScaleConvex = convexGeom.scale.isIdentity();
	const bool idtScaleMesh = meshGeom.scale.isIdentity();

	Cm::FastVertex2ShapeScaling convexScaling;
	if(!idtScaleConvex)
		convexScaling.init(convexGeom.scale);

	Cm::FastVertex2ShapeScaling meshScaling;
	if(!idtScaleMesh)
		meshScaling.init(meshGeom.scale);

	PxBounds3 hullAABB;
	Gu::transformNoEmptyTest(hullAABB, convexScaling.getVertex2ShapeSkew(), cm->getLocalBounds());

	Gu::Box hullOBB;
	computeHullOBB(hullOBB, hullAABB, 0.0f, convexPose, convexTM, meshTM, meshScaling, idtScaleMesh);
	//~PT: TODO: this part similar to convex-vs-overlap test, refactor

	// Now create temporal bounds
	Gu::Box querySweptBox;
	CreateOBB(querySweptBox, hullOBB, pose.rotateInv(unitDir), distance);

	// PT: TODO: this part similar to convex-vs-overlap test, refactor
	Ice::HybridModelData hmd;	// PT: I suppose doing the "conversion" at runtime is fine
	tm->mesh.mData.mOpcodeModel.getHybridModelData(hmd);

	Ice::HybridOBBCollider collider;
	collider.SetFullBoxBoxTest(false);	// PT: usually faster
	collider.SetPrimitiveTests(true);
	collider.SetLoosePrimitiveTests(false);
	//~PT: TODO: this part similar to convex-vs-overlap test, refactor

	ConvexVsMeshSweepCallback cb(*cm, convexGeom.scale, meshScaling, convexPose, pose, -unitDir, distance, hintFlags);
	collider.Collide(querySweptBox, hmd, &cb, NULL, NULL, true);
	if(cb.mAnyHit)
	{
		sweepHit = cb.mHit;
		sweepHit.impact += unitDir * sweepHit.distance;
		sweepHit.normal = -sweepHit.normal;
		sweepHit.normal.normalize();
		return true;
	}
	return false;
}

	class ConvexVsHeightfieldSweep : public Gu::EntityReport<PxU32>
	{
	public:
		ConvexVsHeightfieldSweep(
			Gu::HeightFieldUtil& hfUtil,
			const Gu::ConvexMesh& cm,
			const PxMeshScale& convexScale,
			const PxTransform& tr0,
			const PxTransform& tr1,
			const PxVec3& unitDir, const PxReal distance, PxSceneQueryFlags hintFlags) :
			mHfUtil			(hfUtil),
			mConvexSupport	(cm, convexScale),
			mTransform0		(tr0),
			mTransform1		(tr1),
			mUnitDir		(unitDir),
			mDistance		(distance),
			mHintFlags		(hintFlags),
			mAnyHit			(false)
		{
			mHit.faceIndex = 0;
			mHit.distance = FLT_MAX;
		}

		bool testTriangle(PxU32 i)
		{
			Gu::Triangle tri;
			mHfUtil.getTriangle(PxTransform::createIdentity(), tri, NULL, NULL, NULL, i, false, false);  // First parameter not needed if local space triangle is enough

			Gu::GJKTriangleSupport triangleSupport(tri.verts[0], tri.verts[1], tri.verts[2]);

			PxReal toi;
			PxVec3 destNormal;
			PxVec3 destWorldPointA;

			const bool ok = convexConvexLinearSweep(
				mConvexSupport, triangleSupport,
				mTransform0, mTransform0.p,
				mTransform1, mTransform1.p + (mUnitDir * mDistance),
				gGJKEpsilon,
				destNormal, destWorldPointA, toi);

			if(ok)
			{
				mHit.flags = PxSceneQueryFlag::eDISTANCE | PxSceneQueryFlag::eIMPACT | PxSceneQueryFlag::eNORMAL;

				mAnyHit = true;

				if(mHintFlags & PxSceneQueryFlag::eINITIAL_OVERLAP)
				{
					// PT: test if shapes initially overlap
					if(toi<=0.0f)
					{
						mHit.distance	= 0.0f;
						mHit.normal		= -mUnitDir;
						mHit.impact		= destWorldPointA;	// PT: this is arbitrary
						mAnyHit			= true;
						return false;
					}
				}

				const float minDist = toi * mDistance;

				if(minDist<mHit.distance)
				{
					mHit.distance	= minDist;
					mHit.normal		= destNormal;
					mHit.impact		= destWorldPointA;
					mAnyHit			= true;
				}
			}
			return true;
		}

/*		bool testAll()
		{
			const Gu::HeightField& heightfield = mHfUtil.getHeightField();

			const PxU32 nbRows = heightfield.getNbRowsFast();
			const PxU32 nbColumns = heightfield.getNbColumnsFast();
			const PxU32 nbVerts = nbRows * nbColumns;
			const PxU32 nbTriangles = 2 * nbVerts;

			for(PxU32 i=0; i<nbTriangles; i++)
			{
				if(heightfield.isValidTriangle(i))
					if(!testTriangle(i))
						return false;
			}
			return true;
		}*/

		virtual bool onEvent(PxU32 nbEntities, PxU32* entities)
		{
//			static int gCount=0;
//			printf("Results%d\n", gCount++);
			for(PxU32 i=0; i < nbEntities; i++)
			{
				if(!testTriangle(entities[i]))
					return false;
			}
			return true;
		}

		Gu::HeightFieldUtil&	mHfUtil;
		Gu::GJKConvexSupport	mConvexSupport;
		const PxTransform&		mTransform0;
		const PxTransform&		mTransform1;
		PxSweepHit				mHit;
		PxVec3					mUnitDir;
		PxReal					mDistance;
		PxU32					mHintFlags;
		bool					mAnyHit;
	};

static bool sweepConvex_heightField(const PxGeometry& geom, const PxTransform& pose, const PxConvexMeshGeometry& convexGeom, const PxTransform& convexPose,
							   const PxVec3& unitDir, const PxReal distance, PxSweepHit& sweepHit, PxSceneQueryFlags hintFlags)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eHEIGHTFIELD);
	const PxHeightFieldGeometry& hfGeom = static_cast<const PxHeightFieldGeometry&>(geom);

	const Cm::Matrix34 convexTM(convexPose);
	const Cm::Matrix34 meshTM(pose);

	const Gu::ConvexMesh* cm = static_cast<const Gu::ConvexMesh*>(convexGeom.convexMesh);
//	const Gu::TriangleMesh* tm = static_cast<Gu::TriangleMesh*>(meshGeom.triangleMesh);

	const bool idtScaleConvex = convexGeom.scale.isIdentity();
//	const bool idtScaleMesh = meshGeom.scale.isIdentity();
	const bool idtScaleMesh = true;

	Cm::FastVertex2ShapeScaling convexScaling;
	if(!idtScaleConvex)
		convexScaling.init(convexGeom.scale);

	Cm::FastVertex2ShapeScaling meshScaling;
//	if(!idtScaleMesh)
//		meshScaling.init(meshGeom.scale);

	PxBounds3 hullAABB;
	Gu::transformNoEmptyTest(hullAABB, convexScaling.getVertex2ShapeSkew(), cm->getLocalBounds());

	Gu::Box hullOBB;
	computeHullOBB(hullOBB, hullAABB, 0.0f, convexPose, convexTM, meshTM, meshScaling, idtScaleMesh);

	// Now create temporal bounds
	Gu::Box querySweptBox;
	CreateOBB(querySweptBox, hullOBB, pose.rotateInv(unitDir), distance);

	// from MeshQuery::findOverlapHeightField
	const PxBounds3 bounds = PxBounds3::basisExtent(querySweptBox.center, querySweptBox.rot, querySweptBox.extents);

	Gu::HeightFieldUtil hfUtil(hfGeom);
	ConvexVsHeightfieldSweep entityReport(hfUtil, *cm, convexGeom.scale, convexPose, pose, -unitDir, distance, hintFlags);

	hfUtil.overlapAABBTriangles(pose, bounds, 0, &entityReport);
//	entityReport.testAll();

	if(entityReport.mAnyHit)
	{
		sweepHit = entityReport.mHit;
		sweepHit.impact += unitDir * sweepHit.distance;
		sweepHit.normal = -sweepHit.normal;
		sweepHit.normal.normalize();
		return true;
	}
	return false;
}

const Gu::SweepConvexFunc Gu::gSweepConvexMap[7] = 
{
	sweepConvex_sphere,
	sweepConvex_plane,
	sweepConvex_capsule,
	sweepConvex_box,
	sweepConvex_convexMesh,
	sweepConvex_triangleMesh,
	sweepConvex_heightField
};

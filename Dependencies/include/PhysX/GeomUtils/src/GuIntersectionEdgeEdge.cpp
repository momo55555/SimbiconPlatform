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


#include "GuPlane.h"
#include "GuIntersectionEdgeEdge.h"
#include "PsMathUtils.h"
#include "CmPhysXCommon.h"

using namespace physx;

bool Gu::intersectEdgeEdge(const PxVec3& p1, const PxVec3& p2, const PxVec3& dir, const PxVec3& p3, const PxVec3& p4, PxReal& dist, PxVec3& ip)
{
	const PxVec3 v1 = p2 - p1;

	// Build plane P based on edge (p1, p2) and direction (dir)
	Gu::Plane plane;
	plane.normal = v1.cross(dir);
	plane.d = -(plane.normal.dot(p1));

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
	PxReal temp = d3 * plane.distance(p4);
	if(temp>0.0f)	return false;

	// if colliding edge (p3,p4) and plane are parallel return no collision
	PxVec3 v2 = p4 - p3;

	temp = plane.normal.dot(v2);
	if(temp==0.0f)	return false;	// ### epsilon would be better

	// compute intersection point of plane and colliding edge (p3,p4)
	ip = p3-v2*(d3/temp);

	// find largest 2D plane projection
	PxU32 i,j;
	Ps::closestAxis(plane.normal, i, j);

	// compute distance of intersection from line (ip, -dir) to line (p1,p2)
	dist =	(v1[i]*(ip[j]-p1[j])-v1[j]*(ip[i]-p1[i]))/(v1[i]*dir[j]-v1[j]*dir[i]);
	if(dist<0.0f)	return false;

	// compute intersection point on edge (p1,p2) line
	ip -= dist*dir;

	// check if intersection point (ip) is between edge (p1,p2) vertices
	temp = (p1.x-ip.x)*(p2.x-ip.x)+(p1.y-ip.y)*(p2.y-ip.y)+(p1.z-ip.z)*(p2.z-ip.z);
	if(temp<0.0f)	return true;	// collision found

	return false;	// no collision
}


bool Gu::intersectEdgeEdge(const Ps::aos::Vec3VArg p1, const Ps::aos::Vec3VArg p2, const Ps::aos::Vec3VArg dir, const Ps::aos::Vec3VArg p3, const Ps::aos::Vec3VArg p4, Ps::aos::FloatV& dist, Ps::aos::Vec3V& ip)
{
	using namespace Ps::aos;

	const FloatV zero = FZero();
	const Vec3V ndir = V3Neg(dir);
	
	const Vec3V v1 = V3Sub(p2, p1);
	const Vec3V v2 = V3Sub(p4, p3);
	const Vec3V n = V3Cross(v1, dir);
	const FloatV d = FNeg(V3Dot(n, p1));

	const FloatV d3 = FAdd(V3Dot(n, p3), d);
	const FloatV d4 = FAdd(V3Dot(n, p4), d);
	
	const FloatV temp0 = FMul(d3, d4);
	const BoolV bSameSide = FIsGrtr(temp0, zero);

	//compute intersection point of plane and colliding edge (p3, p4)
	const FloatV nom = FAdd(V3Dot(p3, n), d);
	const FloatV denom = V3Dot(n, v2);
	const BoolV bParalle = FIsEq(denom, zero);

	const FloatV t = FNeg(FDiv(nom, denom));
	const Vec3V ipPlane = V3MulAdd(v2, t, p3);

	const Vec3V norm = V3Cross(n, v1);
	
	const Vec3V qip = V3Sub(ipPlane, p1);
	const FloatV tValue = FNeg(FDiv(V3Dot(norm, qip),V3Dot(norm, ndir)));

	const Vec3V tip = V3MulAdd(ndir, tValue, ipPlane);

	// check if intersection point (ip) is between edge (p1,p2) vertices
	const Vec3V vw = V3Mul(V3Sub(p1, tip), V3Sub(p2, tip));
	const FloatV temp = FAdd(V3GetX(vw), FAdd(V3GetY(vw),V3GetZ(vw)));
	const BoolV bNotInRange = FIsGrtr(temp, zero);
	
	dist = tValue;
	ip = tip;

	//const BoolV bIntersect = BNot(BOr(bSameSide, BOr(bParalle, bNotInRange)));
	return BAllEq(BOr(bSameSide, BOr(bParalle, bNotInRange)), BFFFF())==1;
}

Ps::aos::BoolV Gu::intersectEdgeEdge4(const Ps::aos::Vec3VArg p1, const Ps::aos::Vec3VArg p2, const Ps::aos::Vec3VArg dir, 
							const Ps::aos::Vec3VArg a0, const Ps::aos::Vec3VArg b0, 
							const Ps::aos::Vec3VArg a1, const Ps::aos::Vec3VArg b1,
							const Ps::aos::Vec3VArg a2, const Ps::aos::Vec3VArg b2,
							const Ps::aos::Vec3VArg a3, const Ps::aos::Vec3VArg b3,
							Ps::aos::Vec4V& dist, Ps::aos::Vec3V* ip)
{
	using namespace Ps::aos;

	const Vec4V zero = V4Zero();
	const Vec3V ndir = V3Neg(dir);
	
	const Vec3V v1 = V3Sub(p2, p1);
	const Vec3V n = V3Cross(v1, dir);
	const FloatV d = FNeg(V3Dot(n, p1));

	const Vec3V ab0 = V3Sub(b0, a0);
	const Vec3V ab1 = V3Sub(b1, a1);
	const Vec3V ab2 = V3Sub(b2, a2);
	const Vec3V ab3 = V3Sub(b3, a3);

	FloatV d0[4], d1[4];
	d0[0] = FAdd(V3Dot(n, a0), d);
	d0[1] = FAdd(V3Dot(n, a1), d);
	d0[2] = FAdd(V3Dot(n, a2), d);
	d0[3] = FAdd(V3Dot(n, a3), d);

	d1[0] = FAdd(V3Dot(n, b0), d);
	d1[1] = FAdd(V3Dot(n, b1), d);
	d1[2] = FAdd(V3Dot(n, b2), d);
	d1[3] = FAdd(V3Dot(n, b3), d);

	const Vec4V signDist0 = V4Merge(d0);
	const Vec4V signDist1 = V4Merge(d1);

	const Vec4V sign = V4Mul(signDist0, signDist1);
	
	const BoolV bSameSide = V4IsGrtr(sign, zero);

	FloatV nab[4];
	nab[0] = V3Dot(n, ab0);
	nab[1] = V3Dot(n, ab1);
	nab[2] = V3Dot(n, ab2);
	nab[3] = V3Dot(n, ab3);

	//compute intersection point of plane and colliding edge (p3, p4)
	const Vec4V nom = signDist0;
	const Vec4V denom = V4Merge(nab);
	const BoolV bParalle = V4IsEq(denom, zero);

	const Vec4V t = V4Neg(V4Div(nom, denom));
	const Vec3V ipPlane0 = V3MulAdd(ab0, V4GetX(t), a0);
	const Vec3V ipPlane1 = V3MulAdd(ab1, V4GetY(t), a1);
	const Vec3V ipPlane2 = V3MulAdd(ab2, V4GetZ(t), a2);
	const Vec3V ipPlane3 = V3MulAdd(ab3, V4GetW(t), a3);

	const Vec3V norm = V3Cross(n, v1);

	const Vec3V qip0 = V3Sub(ipPlane0, p1);
	const Vec3V qip1 = V3Sub(ipPlane1, p1);
	const Vec3V qip2 = V3Sub(ipPlane2, p1);
	const Vec3V qip3 = V3Sub(ipPlane3, p1);

	FloatV tmp0[4];
	tmp0[0] = V3Dot(norm, qip0);
	tmp0[1] = V3Dot(norm, qip1);
	tmp0[2] = V3Dot(norm, qip2);
	tmp0[3] = V3Dot(norm, qip3);

	const Vec4V nom1 = V4Merge(tmp0);
	const FloatV denom1 = FRecip(V3Dot(norm, ndir));
	const Vec4V tValue = V4Neg(V4Mul(nom1, denom1));

	//const FloatV tValue = FNeg(FDiv(V3Dot(norm, qip),V3Dot(norm, ndir)));

	const Vec3V tip0 = V3MulAdd(ndir, V4GetX(tValue), ipPlane0);
	const Vec3V tip1 = V3MulAdd(ndir, V4GetY(tValue), ipPlane1);
	const Vec3V tip2 = V3MulAdd(ndir, V4GetZ(tValue), ipPlane2);
	const Vec3V tip3 = V3MulAdd(ndir, V4GetW(tValue), ipPlane3);

	const Vec3V vw0 = V3Mul(V3Sub(p1, tip0), V3Sub(p2, tip0));
	const Vec3V vw1 = V3Mul(V3Sub(p1, tip1), V3Sub(p2, tip1));
	const Vec3V vw2 = V3Mul(V3Sub(p1, tip2), V3Sub(p2, tip2));
	const Vec3V vw3 = V3Mul(V3Sub(p1, tip3), V3Sub(p2, tip3));

	// check if intersection point (ip) is between edge (p1,p2) vertices
	FloatV temp[4];
	temp[0] = FAdd(V3GetX(vw0), FAdd(V3GetY(vw0),V3GetZ(vw0)));
	temp[1] = FAdd(V3GetX(vw1), FAdd(V3GetY(vw1),V3GetZ(vw1)));
	temp[2] = FAdd(V3GetX(vw2), FAdd(V3GetY(vw2),V3GetZ(vw2)));
	temp[3] = FAdd(V3GetX(vw3), FAdd(V3GetY(vw3),V3GetZ(vw3)));

	const Vec4V vv = V4Merge(temp);
	const BoolV bNotInRange = V4IsGrtr(vv, zero);

	dist = tValue;
	ip[0] = tip0;
	ip[1] = tip1;
	ip[2] = tip2;
	ip[3] = tip3;
	
	//const BoolV bIntersect = BNot(BOr(bSameSide, BOr(bParalle, bNotInRange)));
	return BNot(BOr(bSameSide, BOr(bParalle, bNotInRange)));
}

bool Gu::intersectEdgeEdgeNEW(const PxVec3& p1, const PxVec3& p2, const PxVec3& dir, const PxVec3& p3, const PxVec3& p4, PxReal& dist, PxVec3& ip)
{
	// Build plane P based on edge (p1, p2) and direction (dir)
	const PxVec3 v12 = p2-p1;

	Gu::Plane plane;
	plane.normal = v12.cross(dir);
	plane.d = -(plane.normal.dot(p1));

	PxReal d3 = plane.distance(p3);
	PxReal d4 = plane.distance(p4);

	// line doesn't intersect plane if both have same sign or both are 0.
	if(d3*d4>0 || d3==d4) 
		return false;

	// vector from p1 to intersection point of plane and edge (p3,p4)
	PxVec3 v1i = (d3*p4 - d4*p3)/(d3-d4) - p1;

	// find largest 2D plane projection
	PxU32 i,j;
	Ps::closestAxis(plane.normal, i, j);

	// compute distance of v1i to intersection of line (v1i, v1i-dir) and line (0,v12)
	PxReal d = (v12[i]*v1i[j]-v12[j]*v1i[i])/(v12[i]*dir[j]-v12[j]*dir[i]);
	if(d<0.0f)
		return false;

	// vector from p1 to intersection point of two lines above
	v1i -= d*dir;

	// we are allowed to write invalid output
	dist = d;
	ip = p1 + v1i;

	// return if intersection point is on sweep side and between p1 and p2
	return v1i.dot(v1i-v12)<0.0f;
}


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


#include "GuIntersectionRayCapsule.h"
#include "GuCapsule.h"
#include "GuVecCapsule.h"

using namespace physx;

PxU32 Gu::intersectRayCapsule(const PxVec3& origin, const PxVec3& dir, const Gu::Capsule& capsule, PxReal s[2])
{
	// set up quadratic Q(t) = a*t^2 + 2*b*t + c
	PxVec3 capsDir;
	capsule.computeDirection(capsDir);

	PxVec3 kW = capsDir;
	const PxReal fWLength = kW.normalize();

	// generate orthonormal basis

	PxVec3 kU;
	PxReal fInvLength;
	if ( PxAbs(kW.x) >= PxAbs(kW.y) )
	{
		// W.x or W.z is the largest magnitude component, swap them
		fInvLength = PxRecipSqrt(kW.x*kW.x + kW.z*kW.z);
		kU.x = -kW.z*fInvLength;
		kU.y = 0.0f;
		kU.z = kW.x*fInvLength;
	}
	else
	{
		// W.y or W.z is the largest magnitude component, swap them
		fInvLength = PxRecipSqrt(kW.y*kW.y + kW.z*kW.z);
		kU.x = 0.0f;
		kU.y = kW.z*fInvLength;
		kU.z = -kW.y*fInvLength;
	}

	PxVec3 kV = kW.cross(kU);
	kV.normalize();	// PT: fixed november, 24, 2004. This is a bug in Magic.

	// compute intersection

	PxVec3 kD(kU.dot(dir), kV.dot(dir), kW.dot(dir));
	const PxReal fDLength = kD.normalize();

	const PxReal fInvDLength = 1.0f/fDLength;
	const PxVec3 kDiff = origin - capsule.p0;
	const PxVec3 kP(kU.dot(kDiff), kV.dot(kDiff), kW.dot(kDiff));
	const PxReal fRadiusSqr = capsule.radius*capsule.radius;

	// Is the velocity parallel to the capsule direction? (or zero)
	if ( PxAbs(kD.z) >= 1.0f - PX_EPS_REAL || fDLength < PX_EPS_REAL )
	{
		const PxReal fAxisDir = dir.dot(capsDir);

		const PxReal fDiscr = fRadiusSqr - kP.x*kP.x - kP.y*kP.y;
		if ( fAxisDir < 0 && fDiscr >= 0.0f )
		{
			// Velocity anti-parallel to the capsule direction
			const PxReal fRoot = PxSqrt(fDiscr);
			s[0] = (kP.z + fRoot)*fInvDLength;
			s[1] = -(fWLength - kP.z + fRoot)*fInvDLength;
			return 2;
		}
		else if ( fAxisDir > 0  && fDiscr >= 0.0f )
		{
			// Velocity parallel to the capsule direction
			const PxReal fRoot = PxSqrt(fDiscr);
			s[0] = -(kP.z + fRoot)*fInvDLength;
			s[1] = (fWLength - kP.z + fRoot)*fInvDLength;
			return 2;
		}
		else
		{
			// sphere heading wrong direction, or no velocity at all
			return 0;
		}   
	}

	// test intersection with infinite cylinder
	PxReal fA = kD.x*kD.x + kD.y*kD.y;
	PxReal fB = kP.x*kD.x + kP.y*kD.y;
	PxReal fC = kP.x*kP.x + kP.y*kP.y - fRadiusSqr;
	PxReal fDiscr = fB*fB - fA*fC;
	if ( fDiscr < 0.0f )
	{
		// line does not intersect infinite cylinder
		return 0;
	}

	int iQuantity = 0;

	if ( fDiscr > 0.0f )
	{
		// line intersects infinite cylinder in two places
		const PxReal fRoot = PxSqrt(fDiscr);
		const PxReal fInv = 1.0f/fA;
		PxReal fT = (-fB - fRoot)*fInv;
		PxReal fTmp = kP.z + fT*kD.z;
		if ( 0.0f <= fTmp && fTmp <= fWLength )
			s[iQuantity++] = fT*fInvDLength;

		fT = (-fB + fRoot)*fInv;
		fTmp = kP.z + fT*kD.z;
		if ( 0.0f <= fTmp && fTmp <= fWLength )
			s[iQuantity++] = fT*fInvDLength;

		if ( iQuantity == 2 )
		{
			// line intersects capsule wall in two places
			return 2;
		}
	}
	else
	{
		// line is tangent to infinite cylinder
		const PxReal fT = -fB/fA;
		const PxReal fTmp = kP.z + fT*kD.z;
		if ( 0.0f <= fTmp && fTmp <= fWLength )
		{
			s[0] = fT*fInvDLength;
			return 1;
		}
	}

	// test intersection with bottom hemisphere
	// fA = 1
	fB += kP.z*kD.z;
	fC += kP.z*kP.z;
	fDiscr = fB*fB - fC;
	if ( fDiscr > 0.0f )
	{
		const PxReal fRoot = PxSqrt(fDiscr);
		PxReal fT = -fB - fRoot;
		PxReal fTmp = kP.z + fT*kD.z;
		if ( fTmp <= 0.0f )
		{
			s[iQuantity++] = fT*fInvDLength;
			if ( iQuantity == 2 )
				return 2;
		}

		fT = -fB + fRoot;
		fTmp = kP.z + fT*kD.z;
		if ( fTmp <= 0.0f )
		{
			s[iQuantity++] = fT*fInvDLength;
			if ( iQuantity == 2 )
				return 2;
		}
	}
	else if ( fDiscr == 0.0f )
	{
		const PxReal fT = -fB;
		const PxReal fTmp = kP.z + fT*kD.z;
		if ( fTmp <= 0.0f )
		{
			s[iQuantity++] = fT*fInvDLength;
			if ( iQuantity == 2 )
				return 2;
		}
	}

	// test intersection with top hemisphere
	// fA = 1
	fB -= kD.z*fWLength;
	fC += fWLength*(fWLength - 2.0f*kP.z);

	fDiscr = fB*fB - fC;
	if ( fDiscr > 0.0f )
	{
		const PxReal fRoot = PxSqrt(fDiscr);
		PxReal fT = -fB - fRoot;
		PxReal fTmp = kP.z + fT*kD.z;
		if ( fTmp >= fWLength )
		{
			s[iQuantity++] = fT*fInvDLength;
			if ( iQuantity == 2 )
				return 2;
		}

		fT = -fB + fRoot;
		fTmp = kP.z + fT*kD.z;
		if ( fTmp >= fWLength )
		{
			s[iQuantity++] = fT*fInvDLength;
			if ( iQuantity == 2 )
				return 2;
		}
	}
	else if ( fDiscr == 0.0f )
	{
		const PxReal fT = -fB;
		const PxReal fTmp = kP.z + fT*kD.z;
		if ( fTmp >= fWLength )
		{
			s[iQuantity++] = fT*fInvDLength;
			if ( iQuantity == 2 )
				return 2;
		}
	}

	return iQuantity;
}


//bool Gu::intersectRayCapsule(const Ps::aos::Vec3VArg origin, const Ps::aos::Vec3VArg dir, const Gu::CapsuleV& capsule, Ps::aos::FloatV& s)
//{
//	using namespace Ps::aos;
//	// set up quadratic Q(t) = a*t^2 + 2*b*t + c
//
//	//check for infinite cyliner
//	const FloatV zero = FZero();
//	const FloatV r = capsule.getRadius();
//	const Vec3V d = capsule.computeDirection();
//	const Vec3V dLength = V3Length(d);
//	const Vec3V m = V3Sub(origin, capsule.p0);
//	const FloatV n = dir;
//
//	const FloatV mm = V3Dot(m, m);
//	const FloatV mn = V3Dot(m, n);
//	const FloatV md = V3Dot(m, d);
//	const FloatV nn = V3Dot(n, n);
//	const FloatV nd = V3Dot(n, d);
//	const FloatV dd = V3Dot(d, d);
//	const FloatV rr = FMul(r, r);
//
//	const FloatV a = FSub(FMul(dd, nn), FMul(nd, nd));
//	const FloatV b = FSub(FMul(dd,mn), FMul(nd, md));
//	const FloatV c = FSub(FMul(dd, FSub(mm, rr)), FMul(md, md));
//	
//
//	const FloatV discriminant = FSub(FMul(b, b), FMul(a, c));
//	if(FAllGrtr(zero, discriminant))
//		return false; //no intersect
//
//	const FloatV nb = FNeg(b);
//	const FloatV sqrtDisc = FSqrt(discriminant);
//	const FloatV nom0 = FSub(nb, sqrtDisc);
//	const FloatV nom1 = FAdd(nb, sqrtDisc);
//	const FloatV denom = FRecip(a);
//	
//	const FloatV t0 = FMul(nom0, denom);
//	const FloatV t1 = FMul(nom1, denom);
//	
//	if((FAllGrtr(zero, t0) == 1) & (FAllGrtr(zero, t1) == 1))//no intersect
//		return false;
//
//	s = t0;
//	//test whether the intersect point is outside of the two end plane
//	
//	const FloatV k = FDiv(FMulAdd(nd, t0, md), dLength);
//
//	const PxU32 con0 = FAllGrtr(k, dLength);
//	const PxU32 con1 = FAllGrtr(zero, k);
//
//	if((con0 == 0) && (con1 == 0))
//	{
//		s = FSel(FIsGrtr(t0, zero), t0, t1);
//		return true;
//	}
//
//	
//	const BoolV con00 = FIsGrtr(zero, k);
//	const Vec3V p = V3Sel(con00, capsule.p0, capsule.p1);
//
//	const Vec3V qp = V3Sub(origin, p);
//	const FloatV a1 = nn;
//	const FloatV b1 = V3Dot(qp, dir);
//	const FloatV c1 = FSub(V3Dot(qp, qp), rr);
//	const FloatV discriminant1 = FSub(FMul(b1, b1),  FMul(a1, c1));
//	if(FAllGrtr(zero, discriminant1))
//		return false;
//
//	const FloatV tmp = FDiv(FSub(FNeg(b1), FSqrt(discriminant1)), a1);
//
//	//if t<0, which means the ray is inside the capsule, in that case, the intersect point will be the other intersect point
//	s = FSel(FIsGrtr(tmp, zero), tmp, t1);
//	return true;
//}

bool Gu::intersectRayCapsule(const Ps::aos::Vec3VArg origin, const Ps::aos::Vec3VArg dir, const Gu::CapsuleV& capsule, Ps::aos::FloatV& s)
{
	using namespace Ps::aos;
	// set up quadratic Q(t) = a*t^2 + 2*b*t + c

	//check for infinite cyliner
	const FloatV zero = FZero();
	//const FloatV one = FSub(FOne(), FEps());
	const FloatV r = capsule.getRadius();
	const Vec3V d = capsule.computeDirection();
	const FloatV dLength = V3Length(d);
	const Vec3V m = V3Sub(origin, capsule.p0);
	const Vec3V n = dir;

	const FloatV mm = V3Dot(m, m);
	const FloatV mn = V3Dot(m, n);
	const FloatV md = V3Dot(m, d);
	const FloatV nn = V3Dot(n, n);
	const FloatV nd = V3Dot(n, d);
	const FloatV dd = V3Dot(d, d);
	const FloatV rr = FMul(r, r);

	//const FloatV v = FDiv(nd, dLength);

	if(FAllGrtrOrEq(FAbs(nd), FSub(dLength, FEps())))
	{
		const Vec3V qp0 = V3Sub(origin, capsule.p0);
		const FloatV b0 = V3Dot(qp0, dir);
		const FloatV c0 = FSub(V3Dot(qp0, qp0), rr);
		const FloatV discriminant0 = FSub(FMul(b0, b0),  FMul(nn, c0));
		const FloatV t0 = FDiv(FSub(FNeg(b0), FSqrt(discriminant0)), nn);

		const Vec3V qp1 = V3Sub(origin, capsule.p1);
		const FloatV b1 = V3Dot(qp1, dir);
		const FloatV c1 = FSub(V3Dot(qp1, qp1), rr);
		const FloatV discriminant1 = FSub(FMul(b1, b1),  FMul(nn, c1));
		const FloatV t1 = FDiv(FSub(FNeg(b1), FSqrt(discriminant1)), nn);

		const PxU32 result = FAllGrtr(zero, discriminant0) & FAllGrtr(zero, discriminant1);

		const FloatV t = FMin(t0, t1);
		s = FSel(FIsGrtr(t, zero), t, zero);
	
		return (result == 0);
	}

	const FloatV a = FSub(FMul(dd, nn), FMul(nd, nd));
	const FloatV b = FSub(FMul(dd,mn), FMul(nd, md));
	const FloatV c = FSub(FMul(dd, FSub(mm, rr)), FMul(md, md));
	
	const FloatV discriminant = FSub(FMul(b, b), FMul(a, c));
	

	if(FAllGrtr(zero, discriminant))
		return false;
	
	const FloatV nb = FNeg(b);
	const FloatV denom = FRecip(a);
	const FloatV sqrtDisc = FSqrt(discriminant);
	const FloatV nom0 = FSub(nb, sqrtDisc);
	const FloatV nom1 = FAdd(nb, sqrtDisc);
	const FloatV t0 = FMul(nom0, denom);
	const FloatV t1 = FMul(nom1, denom);
	
	if(FAllGrtr(zero, t1))//no intersect
		return false;

	s = t0;
	//test whether the intersect point is outside of the two end plane
	
	//const FloatV k = FDiv(FMulAdd(nd, t0, md), dLength);
	const FloatV k = FMulAdd(nd, t0, md);

	const BoolV con0 = FIsGrtr(dd, k);
	const BoolV con1 = FIsGrtr(k, zero);

	const BoolV con = BAnd(con0, con1);
	if(BAllEq(con, BTTTT()))
	{
		s = FSel(FIsGrtr(t0, zero), t0, zero);
		return true;
	}

	const Vec3V p = V3Sel(con1, capsule.p1, capsule.p0);

	const Vec3V qp = V3Sub(origin, p);
	const FloatV a1 = nn;
	const FloatV b1 = V3Dot(qp, dir);
	const FloatV c1 = FSub(V3Dot(qp, qp), rr);
	const FloatV discriminant1 = FSub(FMul(b1, b1),  FMul(a1, c1));
	const FloatV tmp = FDiv(FSub(FNeg(b1), FSqrt(discriminant1)), a1);

	//if t<0, which means the ray is inside the capsule, in that case, the intersect point will be the other intersect point
	s = FSel(FIsGrtr(tmp, zero), tmp, zero);
	return (FAllGrtr(zero, discriminant1) == 0);
	
}


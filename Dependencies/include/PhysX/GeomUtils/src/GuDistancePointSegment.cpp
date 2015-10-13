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


#include "GuDistancePointSegment.h"

using namespace physx;

/**
A segment is defined by S(t) = mP0 * (1 - t) + mP1 * t, with 0 <= t <= 1
Alternatively, a segment is S(t) = Origin + t * Direction for 0 <= t <= 1.
Direction is not necessarily unit length. The end points are Origin = mP0 and Origin + Direction = mP1.
*/
PxReal Gu::distancePointSegmentSquared(const PxVec3& p0, const PxVec3& p1, const PxVec3& point, PxReal* param)
{
	PxVec3 Diff = point - p0;
	const PxVec3 Dir = p1 - p0;
	PxReal fT = Diff.dot(Dir);

	if(fT<=0.0f)
	{
		fT = 0.0f;
	}
	else
	{
		const PxReal SqrLen = Dir.magnitudeSquared();
		if(fT>=SqrLen)
		{
			fT = 1.0f;
			Diff -= Dir;
		}
		else
		{
			fT /= SqrLen;
			Diff -= fT*Dir;
		}
	}

	if(param)	*param = fT;

	return Diff.magnitudeSquared();
}

Ps::aos::FloatV Gu::distancePointSegmentSquared(const Ps::aos::Vec3VArg a, const Ps::aos::Vec3VArg b, const Ps::aos::Vec3VArg p, Ps::aos::FloatV& param)
{
	using namespace Ps::aos;
	const FloatV zero = FZero();
	const FloatV one = FOne();

	const Vec3V ap = V3Sub(p, a);
	const Vec3V ab = V3Sub(b, a);
	const FloatV nom = V3Dot(ap, ab);
	
	const FloatV denom = V3Dot(ab, ab);
	const FloatV tValue = FClamp(FMul(nom, FRecip(denom)), zero, one);

	const FloatV t = FSel(FIsEq(denom, zero), zero, tValue);
	const Vec3V v = V3NegMulSub(ab, t, ap);
	param = t;
	return V3Dot(v, v);
}

//Ps::aos::FloatV Gu::distancePointSegmentTValue(const Ps::aos::Vec3VArg a, const Ps::aos::Vec3VArg b, const Ps::aos::Vec3VArg p)
//{
//	using namespace Ps::aos;
//	const FloatV zero = FZero();
//	const Vec3V ap = V3Sub(p, a);
//	const Vec3V ab = V3Sub(b, a);
//	const FloatV nom = V3Dot(ap, ab);
//	
//	const FloatV denom = V3Dot(ab, ab);
//	const FloatV tValue = FMul(nom, FRecip(denom));
//	return FSel(FIsEq(denom, zero), zero, tValue);
//}

////Calculates the distance (a0,b0) -> p0, (a0,b0) -> p1, (a1,b1) ->p0, (a1,b1) -> p1 and returns as 
////elements x,y,z,w in return result respectively
//Ps::aos::Vec4V Gu::distancePointSegmentTValue22(const Ps::aos::Vec3VArg a0, const Ps::aos::Vec3VArg b0, 
//												 const Ps::aos::Vec3VArg a1, const Ps::aos::Vec3VArg b1,
//												 const Ps::aos::Vec3VArg p0, const Ps::aos::Vec3VArg p1)
//{
//	using namespace Ps::aos;
//	const Vec4V zero = V4Zero();
//	const Vec3V ap00 = V3Sub(p0, a0);
//	const Vec3V ap10 = V3Sub(p1, a0);
//	const Vec3V ap01 = V3Sub(p0, a1);
//	const Vec3V ap11 = V3Sub(p1, a1);
//
//	const Vec3V ab00 = V3Sub(b0, a0);
//	const Vec3V ab10 = V3Sub(b1, a0);
//	const Vec3V ab01 = V3Sub(b0, a1);
//	const Vec3V ab11 = V3Sub(b1, a1);
//
//	const FloatV nom00 = V3Dot(ap00, ab00);
//	const FloatV nom10 = V3Dot(ap10, ab10);
//	const FloatV nom01 = V3Dot(ap01, ab01);
//	const FloatV nom11 = V3Dot(ap11, ab11);
//	
//	const FloatV denom00 = V3Dot(ab00, ab00);
//	const FloatV denom10 = V3Dot(ab10, ab10);
//	const FloatV denom01 = V3Dot(ab01, ab01);
//	const FloatV denom11 = V3Dot(ab11, ab11);
//
//	const Vec4V nom = V4Merge(nom00, nom10, nom01, nom11);
//	const Vec4V denom = V4Merge(denom00, denom10, denom01, denom11);
//
//	const Vec4V recip = V4Recip(denom);
//	const Vec4V tValue = VMul(nom, recip);
//	return V4Sel(V4IsEq(denom, zero), zero, tValue);
//}

Ps::aos::Vec4V Gu::distancePointSegmentSquared(const Ps::aos::Vec3VArg a, const Ps::aos::Vec3VArg b, 
												const Ps::aos::Vec3VArg p0, const Ps::aos::Vec3VArg p1,
												const Ps::aos::Vec3VArg p2, const Ps::aos::Vec3VArg p3,
												Ps::aos::Vec4V& param)
{
	using namespace Ps::aos;
	const Vec4V zero = V4Zero();
	const Vec4V one = V4One();
	const Vec3V ab = V3Sub(b, a);
	const FloatV denom = V3Dot(ab, ab);

	const Vec3V ap0 = V3Sub(p0, a);
	const Vec3V ap1 = V3Sub(p1, a);
	const Vec3V ap2 = V3Sub(p2, a);
	const Vec3V ap3 = V3Sub(p3, a);

	const Mat44V m0(Vec4V_From_Vec3V(ap0), Vec4V_From_Vec3V(ap1), Vec4V_From_Vec3V(ap2), Vec4V_From_Vec3V(ap3));
	const Mat44V m0T(M44Trnsps(m0));
	const Vec4V nom0(V4Mul(m0T.col0, V3GetX(ab)));
	const Vec4V nom1(V4MulAdd(m0T.col1, V3GetY(ab), nom0));
	const Vec4V nom(V4MulAdd(m0T.col2, V3GetZ(ab), nom1));
	
	const Vec4V tValue = V4Clamp(V4Mul(nom, V4Recip(denom)), zero, one);

	const Vec4V t = V4Sel(V4IsEq(denom, zero), zero, tValue);
	const Vec3V v0 =V3NegMulSub(ab, V4GetX(t), ap0);
	const Vec3V v1 =V3NegMulSub(ab, V4GetY(t), ap1);
	const Vec3V v2 =V3NegMulSub(ab, V4GetZ(t), ap2);
	const Vec3V v3 =V3NegMulSub(ab, V4GetW(t), ap3);
	param = t;
	const FloatV sqDis[4] = { V3Dot(v0, v0), V3Dot(v1, v1), V3Dot(v2, v2), V3Dot(v3, v3) };
	return V4Merge(sqDis);
}




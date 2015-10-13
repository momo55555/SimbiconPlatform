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


#ifndef OPC_SPHERE_AABB_OVERLAP_H
#define OPC_SPHERE_AABB_OVERLAP_H

#include "CmSimd.h"
#include "Opcode.h"

namespace physx
{
namespace Ice
{

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Sphere-AABB overlap test, based on Jim Arvo's code.
 *	\param		center		[in] box center
 *	\param		extents		[in] box extents
 *	\return		Ps::IntTrue on overlap
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef OPC_SUPPORT_VMX128
PX_FORCE_INLINE Ps::IntBool SphereCollider::SphereAABBOverlap(const PxVec3& center, const PxVec3& extents)
{ 
	// Stats
	mNbVolumeBVTests++;

	float d = 0.0f;

	//find the square of the distance
	//from the sphere to the box
#ifdef OLDIES
	for(udword i=0;i<3;i++)
	{
		float tmp = mCenter[i] - center[i];
		float s = tmp + extents[i];

		if(s<0.0f)	d += s*s;
		else
		{
			s = tmp - extents[i];
			if(s>0.0f)	d += s*s;
		}
	}
#endif

//#ifdef NEW_TEST

//	float tmp = mCenter.x - center.x;
//	float s = tmp + extents.x;

	float tmp,s;

	tmp = mCenter.x - center.x;
	s = tmp + extents.x;

	if(s<0.0f)
	{
		d += s*s;
		if(d>mRadius2)	return Ps::IntFalse;
	}
	else
	{
		s = tmp - extents.x;
		if(s>0.0f)
		{
			d += s*s;
			if(d>mRadius2)	return Ps::IntFalse;
		}
	}

	tmp = mCenter.y - center.y;
	s = tmp + extents.y;

	if(s<0.0f)
	{
		d += s*s;
		if(d>mRadius2)	return Ps::IntFalse;
	}
	else
	{
		s = tmp - extents.y;
		if(s>0.0f)
		{
			d += s*s;
			if(d>mRadius2)	return Ps::IntFalse;
		}
	}

	tmp = mCenter.z - center.z;
	s = tmp + extents.z;

	if(s<0.0f)
	{
		d += s*s;
		if(d>mRadius2)	return Ps::IntFalse;
	}
	else
	{
		s = tmp - extents.z;
		if(s>0.0f)
		{
			d += s*s;
			if(d>mRadius2)	return Ps::IntFalse;
		}
	}
//#endif

#ifdef OLDIES
//	Point Min = center - extents;
//	Point Max = center + extents;

	float d = 0.0f;

	//find the square of the distance
	//from the sphere to the box
	for(udword i=0;i<3;i++)
	{
float Min = center[i] - extents[i];

//		if(mCenter[i]<Min[i])
		if(mCenter[i]<Min)
		{
//			float s = mCenter[i] - Min[i];
			float s = mCenter[i] - Min;
			d += s*s;
		}
		else
		{
float Max = center[i] + extents[i];

//			if(mCenter[i]>Max[i])
			if(mCenter[i]>Max)
			{
				float s = mCenter[i] - Max;
				d += s*s;
			}
		}
	}
#endif
	return d <= mRadius2;
}
#else

PX_FORCE_INLINE Ps::IntBool SphereCollider::SphereAABBOverlap(const PxVec3& center, const PxVec3& extents)
{
	Cm::PxSimd::Vector4 center4 = Cm::PxSimd::load(center);
	Cm::PxSimd::Vector4 extents4 = Cm::PxSimd::load(extents);

	return SphereAABBOverlap(center4, extents4);
}

PX_FORCE_INLINE Ps::IntBool SphereCollider::SphereAABBOverlap(const Cm::PxSimd::Vector4& center4, const Cm::PxSimd::Vector4& extents4)
{
	// calculate the squared distance between the point and the AABB

	Cm::PxSimd::Vector4 sphereCen = Cm::PxSimd::load(mCenter);
	Cm::PxSimd::Vector4 sphereRadius2 = Cm::PxSimd::splatX(Cm::PxSimd::load(mRadius2));

	Cm::PxSimd::Vector4 bbMin = Cm::PxSimd::subtract(center4, extents4);
	Cm::PxSimd::Vector4 bbMax = Cm::PxSimd::add(center4, extents4);

	Cm::PxSimd::Vector4 minMask = Cm::PxSimd::less(sphereCen, bbMin);
	Cm::PxSimd::Vector4 maxMask = Cm::PxSimd::greater(sphereCen, bbMax);

	Cm::PxSimd::Vector4 sqDist = Cm::PxSimd::zero();

	Cm::PxSimd::Vector4 tmp = Cm::PxSimd::subtract(bbMin, sphereCen);
	Cm::PxSimd::Vector4 sqDistA = Cm::PxSimd::multiplyAdd(tmp, tmp, sqDist);
	sqDist = Cm::PxSimd::select(sqDist, sqDistA, minMask);

	Cm::PxSimd::Vector4 tmp2 = Cm::PxSimd::subtract(sphereCen, bbMax);
	Cm::PxSimd::Vector4 sqDistB = Cm::PxSimd::multiplyAdd(tmp2, tmp2, sqDist);
	sqDist = Cm::PxSimd::select(sqDist, sqDistB, maxMask);

	// horizontal add(3 component)
	sqDist = Cm::PxSimd::dot(sqDist, Cm::PxSimd::one());

	//compare to radius squared
	return Cm::PxSimd::lessEqual4Bool(sqDist, sphereRadius2);
}

#endif

} // namespace Ice

}

#endif
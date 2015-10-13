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

namespace physx
{

#define OPC_RAY_TEST_LOCAL_EPSILON 0.000001f

namespace Ice
{

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes a ray-triangle intersection test.
 *	Original code from Tomas Moeller's "Fast Minimum Storage Ray-Triangle Intersection".
 *	It's been optimized a bit with integer code, and modified to return a non-intersection if distance from
 *	ray origin to triangle is negative.
 *
 *	\param		vert0	[in] triangle vertex
 *	\param		vert1	[in] triangle vertex
 *	\param		vert2	[in] triangle vertex
 *	\return		true on overlap. mStabbedFace is filled with relevant info.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PX_FORCE_INLINE Ps::IntBool RayCollider::RayTriOverlap(const PxVec3& vert0, const PxVec3& vert1, const PxVec3& vert2)
{
	// Find vectors for two edges sharing vert0
	const PxVec3 edge1 = vert1 - vert0;
	const PxVec3 edge2 = vert2 - vert0;

	// Begin calculating determinant - also used to calculate U parameter
	const PxVec3 pvec = mDir.cross(edge2);

	// If determinant is near zero, ray lies in plane of triangle
	const float det = edge1.dot(pvec);

	if(mCulling)
	{
#ifndef _XBOX___
		if(det<OPC_RAY_TEST_LOCAL_EPSILON)
			return Ps::IntFalse;
#endif
		// From here, det is > 0. So we can use integer cmp.

		// Calculate distance from vert0 to ray origin
		const PxVec3 tvec = mOrigin - vert0;

		// Calculate U parameter and test bounds
		mStabbedFace.mU = tvec.dot(pvec);
//		if(IR(u)&0x80000000 || u>det)					return Ps::IntFalse;
#ifndef _XBOX___
	#ifdef OPC_GEOM_EPSILON
		if(mStabbedFace.mU < -mGeomEpsilon || mStabbedFace.mU > det + mGeomEpsilon)	return Ps::IntFalse;
	#else
		if(IS_NEGATIVE_FLOAT(mStabbedFace.mU) || IR(mStabbedFace.mU)>IR(det))		return Ps::IntFalse;
	#endif
#endif

		// Prepare to test V parameter
		const PxVec3 qvec = tvec.cross(edge1);

		// Calculate V parameter and test bounds
		mStabbedFace.mV = mDir.dot(qvec);
#ifndef _XBOX___
	#ifdef OPC_GEOM_EPSILON
		if(mStabbedFace.mV < -mGeomEpsilon || (mStabbedFace.mU + mStabbedFace.mV) > det + mGeomEpsilon)	return Ps::IntFalse;
	#else
		if(IS_NEGATIVE_FLOAT(mStabbedFace.mV) || mStabbedFace.mU+mStabbedFace.mV>det)	return Ps::IntFalse;
	#endif
#endif
		// Calculate t, scale parameters, ray intersects triangle
		mStabbedFace.mDistance = edge2.dot(qvec);
		// Det > 0 so we can early exit here
		// Intersection point is valid if distance is positive (else it can just be a face behind the orig point)
#ifndef _XBOX___
		if(mStabbedFace.mDistance<=OPC_RAY_TEST_LOCAL_EPSILON)
			return Ps::IntFalse;
#endif


#ifdef _XBOX___
		const float cndt0 = physx::intrinsics::fsel(det - OPC_RAY_TEST_LOCAL_EPSILON, 1.0f, 0.0f);
//		if(det<OPC_RAY_TEST_LOCAL_EPSILON)
//			return Ps::IntFalse;

	#ifdef OPC_GEOM_EPSILON
		const float cndt1 = physx::intrinsics::fsel(mStabbedFace.mU + mGeomEpsilon, 1.0f, 0.0f);
		const float cndt2 = physx::intrinsics::fsel(det + mGeomEpsilon - mStabbedFace.mU, 1.0f, 0.0f);
//		if(mStabbedFace.mU < -mGeomEpsilon || mStabbedFace.mU > det + mGeomEpsilon)	return Ps::IntFalse;
	#else
		if(IS_NEGATIVE_FLOAT(mStabbedFace.mU) || IR(mStabbedFace.mU)>IR(det))		return Ps::IntFalse;
	#endif

	#ifdef OPC_GEOM_EPSILON
		const float cndt3 = physx::intrinsics::fsel(mStabbedFace.mV + mGeomEpsilon, 1.0f, 0.0f);
		const float cndt4 = physx::intrinsics::fsel(det + mGeomEpsilon - mStabbedFace.mU - mStabbedFace.mV, 1.0f, 0.0f);
//		if(mStabbedFace.mV < -mGeomEpsilon || (mStabbedFace.mU + mStabbedFace.mV) > det + mGeomEpsilon)	return Ps::IntFalse;
	#else
		if(IS_NEGATIVE_FLOAT(mStabbedFace.mV) || mStabbedFace.mU+mStabbedFace.mV>det)	return Ps::IntFalse;
	#endif

		const float cndt5 = physx::intrinsics::fsel(mStabbedFace.mDistance, 1.0f, 0.0f);
//		if(mStabbedFace.mDistance<0.0f)
//			return Ps::IntFalse;

		const float cndt = cndt0*cndt1*cndt2*cndt3*cndt4*cndt5;
		if(cndt==0.0f)
			return Ps::IntFalse;
#endif


		// Else go on
		const float OneOverDet = 1.0f / det;
		mStabbedFace.mDistance *= OneOverDet;
		mStabbedFace.mU *= OneOverDet;
		mStabbedFace.mV *= OneOverDet;
	}
	else
	{
		// the non-culling branch
		if(det>-OPC_RAY_TEST_LOCAL_EPSILON && det<OPC_RAY_TEST_LOCAL_EPSILON)
			return Ps::IntFalse;
		const float OneOverDet = 1.0f / det;

		// Calculate distance from vert0 to ray origin
		const PxVec3 tvec = mOrigin - vert0;

		// Calculate U parameter and test bounds
		mStabbedFace.mU = (tvec.dot(pvec)) * OneOverDet;
		if(mStabbedFace.mU<0.0f || mStabbedFace.mU>1.0f)
			return Ps::IntFalse;

		// prepare to test V parameter
		const PxVec3 qvec = tvec.cross(edge1);

		// Calculate V parameter and test bounds
		mStabbedFace.mV = (mDir.dot(qvec)) * OneOverDet;
		if(mStabbedFace.mV<0.0f || mStabbedFace.mU+mStabbedFace.mV>1.0f)
			return Ps::IntFalse;

		// Calculate t, ray intersects triangle
		mStabbedFace.mDistance = (edge2.dot(qvec)) * OneOverDet;
		// Intersection point is valid if distance is positive (else it can just be a face behind the orig point)
		if(mStabbedFace.mDistance<=OPC_RAY_TEST_LOCAL_EPSILON)
			return Ps::IntFalse;
	}
	return Ps::IntTrue;
}

} // namespace Ice

}

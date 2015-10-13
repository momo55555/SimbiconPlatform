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


// Opcode 1.1: ray-AABB overlap tests based on Woo's code
// Opcode 1.2: ray-AABB overlap tests based on the separating axis theorem
//
// The point of intersection is not computed anymore. The distance to impact is not needed anymore
// since we now have two different queries for segments or rays.

namespace physx
{
namespace Ice
{

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes a segment-AABB overlap test using the separating axis theorem. Segment is cached within the class.
 *	\param		center	[in] AABB center
 *	\param		extents	[in] AABB extents
 *	\return		true on overlap
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PX_FORCE_INLINE Ps::IntBool RayCollider::SegmentAABBOverlap(const PxVec3& center, const PxVec3& extents)
{
	const float Dx = mData2.x - center.x;		if(PxAbs(Dx) > extents.x + mFDir.x)	return Ps::IntFalse;
	const float Dy = mData2.y - center.y;		if(PxAbs(Dy) > extents.y + mFDir.y)	return Ps::IntFalse;
	const float Dz = mData2.z - center.z;		if(PxAbs(Dz) > extents.z + mFDir.z)	return Ps::IntFalse;

	float f;
	f = mData.y * Dz - mData.z * Dy;	if(PxAbs(f) > extents.y*mFDir.z + extents.z*mFDir.y)	return Ps::IntFalse;
	f = mData.z * Dx - mData.x * Dz;	if(PxAbs(f) > extents.x*mFDir.z + extents.z*mFDir.x)	return Ps::IntFalse;
	f = mData.x * Dy - mData.y * Dx;	if(PxAbs(f) > extents.x*mFDir.y + extents.y*mFDir.x)	return Ps::IntFalse;

	return Ps::IntTrue;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes a ray-AABB overlap test using the separating axis theorem. Ray is cached within the class.
 *	\param		center	[in] AABB center
 *	\param		extents	[in] AABB extents
 *	\return		true on overlap
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef OPC_SUPPORT_VMX128

PX_FORCE_INLINE Ps::IntBool RayCollider::RayAABBOverlap(const PxVec3& center, const PxVec3& extents)
{
//	float Dx = mOrigin.x - center.x;	if(fabsf(Dx) > extents.x && Dx*mDir.x>=0.0f)	return Ps::IntFalse;
//	float Dy = mOrigin.y - center.y;	if(fabsf(Dy) > extents.y && Dy*mDir.y>=0.0f)	return Ps::IntFalse;
//	float Dz = mOrigin.z - center.z;	if(fabsf(Dz) > extents.z && Dz*mDir.z>=0.0f)	return Ps::IntFalse;

	const float Dx = mOrigin.x - center.x;	if(GREATER(Dx, extents.x) && Dx*mDir.x>=0.0f)	return Ps::IntFalse;
	const float Dy = mOrigin.y - center.y;	if(GREATER(Dy, extents.y) && Dy*mDir.y>=0.0f)	return Ps::IntFalse;
	const float Dz = mOrigin.z - center.z;	if(GREATER(Dz, extents.z) && Dz*mDir.z>=0.0f)	return Ps::IntFalse;

//	float Dx = mOrigin.x - center.x;	if(GREATER(Dx, extents.x) && ((SIR(Dx)-1)^SIR(mDir.x))>=0.0f)	return Ps::IntFalse;
//	float Dy = mOrigin.y - center.y;	if(GREATER(Dy, extents.y) && ((SIR(Dy)-1)^SIR(mDir.y))>=0.0f)	return Ps::IntFalse;
//	float Dz = mOrigin.z - center.z;	if(GREATER(Dz, extents.z) && ((SIR(Dz)-1)^SIR(mDir.z))>=0.0f)	return Ps::IntFalse;

	float f;
	f = mDir.y * Dz - mDir.z * Dy;		if(PxAbs(f) > extents.y*mFDir.z + extents.z*mFDir.y)	return Ps::IntFalse;
	f = mDir.z * Dx - mDir.x * Dz;		if(PxAbs(f) > extents.x*mFDir.z + extents.z*mFDir.x)	return Ps::IntFalse;
	f = mDir.x * Dy - mDir.y * Dx;		if(PxAbs(f) > extents.x*mFDir.y + extents.y*mFDir.x)	return Ps::IntFalse;

	return Ps::IntTrue;
}
#else

PX_FORCE_INLINE Ps::IntBool RayCollider::RayAABBOverlap(const PxVec3& center, const PxVec3& extents)
{
	Cm::PxSimd::Vector4 c = Cm::PxSimd::load(center);
	Cm::PxSimd::Vector4 e = Cm::PxSimd::load(extents);

	return RayAABBOverlap(c, e);
}

PX_FORCE_INLINE Ps::IntBool RayCollider::RayAABBOverlap(const Cm::PxSimd::Vector4& center, const Cm::PxSimd::Vector4& extents)
{
	Cm::PxSimd::Vector4 origin = Cm::PxSimd::load(mOrigin);
	Cm::PxSimd::Vector4 dir = Cm::PxSimd::load(mDir);

	Cm::PxSimd::Vector4 dirYZX = Cm::PxSimd::permuteYZX(dir);

	Cm::PxSimd::Vector4 fDir = Cm::PxSimd::load(mFDir);
	Cm::PxSimd::Vector4 fDirYZZ = Cm::PxSimd::permuteYZZ(fDir); 
	Cm::PxSimd::Vector4 fDirXYX = Cm::PxSimd::permuteXYX(fDir);

	Cm::PxSimd::Vector4 zero = Cm::PxSimd::zero();

	return RayAABBOverlap(center, extents,
		origin, dir,
		dirYZX, fDir,
		fDirYZZ, fDirXYX,
		zero);

}

PX_FORCE_INLINE Ps::IntBool RayCollider::RayAABBOverlap(const Cm::PxSimd::Vector4& center, const Cm::PxSimd::Vector4& extents,
										 const Cm::PxSimd::Vector4& origin, const Cm::PxSimd::Vector4& dir,
										 const Cm::PxSimd::Vector4& dirYZX, const Cm::PxSimd::Vector4& fDir,
										 const Cm::PxSimd::Vector4& fDirYZZ, const Cm::PxSimd::Vector4& fDirXYX,
										 const Cm::PxSimd::Vector4& zero
										 )
{
	Cm::PxSimd::Vector4 D = Cm::PxSimd::subtract(origin, center);

	Cm::PxSimd::Vector4 maskA = Cm::PxSimd::greater(Cm::PxSimd::abs(D), extents);
	Cm::PxSimd::Vector4 maskB = Cm::PxSimd::greaterEqual(Cm::PxSimd::multiply(D, dir), zero);

	Cm::PxSimd::Vector4 DYZX = Cm::PxSimd::permuteYZX(D);
	
	Cm::PxSimd::Vector4 f = Cm::PxSimd::subtract(Cm::PxSimd::multiply(dir, DYZX), Cm::PxSimd::multiply(dirYZX, D));
	Cm::PxSimd::Vector4 absF = Cm::PxSimd::abs(f);		

	Cm::PxSimd::Vector4 extentsXYX = Cm::PxSimd::permuteXYX(extents);
	Cm::PxSimd::Vector4 extentsYZZ = Cm::PxSimd::permuteYZZ(extents);
	
	Cm::PxSimd::Vector4 tmp = Cm::PxSimd::add(Cm::PxSimd::multiply(extentsXYX, fDirYZZ), Cm::PxSimd::multiply(extentsYZZ, fDirXYX));
	Cm::PxSimd::Vector4 maskC = Cm::PxSimd::greater(absF, tmp);

	Cm::PxSimd::Vector4 mask = Cm::PxSimd::or4(maskC, Cm::PxSimd::and4(maskA, maskB));

	if(Cm::PxSimd::intNotEqualBool(mask, zero))
		return Ps::IntFalse;
	else
		return Ps::IntTrue;

}

#endif

} // namespace Ice

}

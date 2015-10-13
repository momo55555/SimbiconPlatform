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



#include "CmSimd.h"
#include "Opcode.h"

// Following code from Magic-Software (http://www.magic-software.com/)
// A bit modified for Opcode

// New version in Opcode 1.4

namespace physx
{
namespace Ice
{

#ifndef OPC_SUPPORT_VMX128
PX_FORCE_INLINE Ps::IntBool LSSCollider::LSSAABBOverlap(const PxVec3& center, const PxVec3& extents)
{
	// Stats
	mNbVolumeBVTests++;

	const float dcx = mSCen.x - center.x;
	const float ex = extents.x + mRadius;
	if(PxAbs(dcx)>ex + mFDir.x)	return Ps::IntFalse;

	const float dcy = mSCen.y - center.y;
	const float ey = extents.y + mRadius;
	if(PxAbs(dcy)>ey + mFDir.y)	return Ps::IntFalse;

	const float dcz = mSCen.z - center.z;
	const float ez = extents.z + mRadius;
	if(PxAbs(dcz)>ez + mFDir.z)	return Ps::IntFalse;

	if(PxAbs(mSDir.y * dcz - mSDir.z * dcy) > ey*mFDir.z + ez*mFDir.y)	return Ps::IntFalse;
	if(PxAbs(mSDir.z * dcx - mSDir.x * dcz) > ex*mFDir.z + ez*mFDir.x)	return Ps::IntFalse;
	if(PxAbs(mSDir.x * dcy - mSDir.y * dcx) > ex*mFDir.y + ey*mFDir.x)	return Ps::IntFalse;

	return Ps::IntTrue;
}
#else
PX_FORCE_INLINE Ps::IntBool LSSCollider::LSSAABBOverlap(const PxVec3& center, const PxVec3& extents)
{
	Cm::PxSimd::Vector4 center4 = Cm::PxSimd::load(center);
	Cm::PxSimd::Vector4 extents4 = Cm::PxSimd::load(extents);

	return LSSAABBOverlap(center4, extents4);
}

PX_FORCE_INLINE Ps::IntBool LSSCollider::LSSAABBOverlap(const Cm::PxSimd::Vector4& center, const Cm::PxSimd::Vector4& extents)
{
	Cm::PxSimd::Vector4 sCen = Cm::PxSimd::load(mSCen);
	Cm::PxSimd::Vector4 radius = Cm::PxSimd::splatX(Cm::PxSimd::load(mRadius));
	Cm::PxSimd::Vector4 fDir = Cm::PxSimd::load(mFDir);
	Cm::PxSimd::Vector4 sDir = Cm::PxSimd::load(mSDir);//sDir appears to equal sCen can we remove.


	Cm::PxSimd::Vector4 dc = Cm::PxSimd::subtract(sCen, center);
	Cm::PxSimd::Vector4 e = Cm::PxSimd::add(extents, radius);
	Cm::PxSimd::Vector4 absDc = Cm::PxSimd::abs(dc);
	Cm::PxSimd::Vector4 ePlusFDir = Cm::PxSimd::add(e, fDir);

	Cm::PxSimd::Vector4 mask = Cm::PxSimd::greater(absDc, ePlusFDir);
	if(Cm::PxSimd::intNotEqualBool(mask, Cm::PxSimd::zero()))//could avoid this with any true variant of greater.
		return Ps::IntFalse;

	/*
	if(fabsf(mSDir.x * dcy - mSDir.y * dcx) > ex*mFDir.y + ey*mFDir.x)	return Ps::IntFalse;
	if(fabsf(mSDir.y * dcz - mSDir.z * dcy) > ey*mFDir.z + ez*mFDir.y)	return Ps::IntFalse;
	if(fabsf(mSDir.z * dcx - mSDir.x * dcz) > ex*mFDir.z + ez*mFDir.x)	return Ps::IntFalse;
	*/

	Cm::PxSimd::Vector4 dcYZX = Cm::PxSimd::permuteYZX(dc);
	Cm::PxSimd::Vector4 sDirYZX = Cm::PxSimd::permuteYZX(sDir);
	Cm::PxSimd::Vector4 eXYX = Cm::PxSimd::permuteXYX(e);
	Cm::PxSimd::Vector4 fDirYZZ = Cm::PxSimd::permuteYZZ(fDir);
	Cm::PxSimd::Vector4 eYZZ = Cm::PxSimd::permuteYZZ(e);
	Cm::PxSimd::Vector4 fDirXYX = Cm::PxSimd::permuteXYX(fDir);

	Cm::PxSimd::Vector4 a = Cm::PxSimd::subtract(Cm::PxSimd::multiply(sDir, dcYZX), Cm::PxSimd::multiply(sDirYZX, dc));
	a = Cm::PxSimd::abs(a);

	Cm::PxSimd::Vector4 b = Cm::PxSimd::multiplyAdd(eXYX, fDirYZZ, Cm::PxSimd::multiply(eYZZ, fDirXYX));

	mask = Cm::PxSimd::greater(a, b);
	if(Cm::PxSimd::intNotEqualBool(mask, Cm::PxSimd::zero()))//could avoid this with any true variant of greater.
		return Ps::IntFalse;

	return Ps::IntTrue;
}
#endif //OPC_SUPPORT_VMX128

} // namespace Ice

}

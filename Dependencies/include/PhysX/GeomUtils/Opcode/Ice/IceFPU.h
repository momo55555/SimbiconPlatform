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


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef ICEFPU_H
#define ICEFPU_H

#include "CmPhysXCommon.h"

namespace physx
{

#define	SIGN_BITMASK			0x80000000

//! Integer representation of a floating-point value.
#define IR(x)					((PxU32&)(x))
//! Signed integer representation of a floating-point value.
#define SIR(x)					((PxI32&)(x))

//! Floating-point representation of an integer value.
#define FR(x)					((float&)(x))

namespace Ice
{

	//! Checks 2 values have different signs
	PX_FORCE_INLINE Ps::IntBool DifferentSign(float f0, float f1)
	{
		union { PxU32 u; float f; } u1, u2;
		u1.f = f0;
		u2.f = f1;
		return (u1.u^u2.u)&SIGN_BITMASK;
	}

	//! Is the float valid ?
	PX_FORCE_INLINE bool IsNAN(float value)
	{
		union { PxU32 u; float f; } u;
		u.f = value;
		return (u.u & 0x7f800000) == 0x7f800000;
	}
	PX_FORCE_INLINE bool IsIndeterminate(float value)
	{
		union { PxU32 u; float f; } u;
		u.f = value;
		return u.u == 0xffc00000;
	}
	PX_FORCE_INLINE bool IsPlusInf(float value)
	{
		union { PxU32 u; float f; } u;
		u.f = value;
		return u.u == 0x7f800000;
	}
	PX_FORCE_INLINE bool IsMinusInf(float value)
	{
		union { PxU32 u; float f; } u;
		u.f = value;
		return u.u == 0xff800000;
	}

	PX_FORCE_INLINE	bool IsValidFloat(float value)
	{
		if(IsNAN(value))
			return false;
		if(IsIndeterminate(value))
			return false;
		if(IsPlusInf(value))
			return false;
		if(IsMinusInf(value))
			return false;
		return true;
	}
};

}

#endif // ICEFPU_H

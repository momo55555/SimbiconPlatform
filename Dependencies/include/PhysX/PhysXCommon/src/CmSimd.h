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


#ifndef PX_PHYSICS_COMMON_SIMD
#define PX_PHYSICS_COMMON_SIMD

#include "Px.h"
#include "PxVec3.h"
#include "PxMath.h"
#include "PsMathUtils.h"
#include "CmPhysXCommon.h"
#include "GuPlane.h"
#include <float.h>

#if defined(PX_WINDOWS) && 0 // PX_SUPPORT_SSE doesn't exist
#include "windows/CmSimd_WIN.h"
#elif (defined(PX_LINUX) || defined(PX_APPLE)) && 0 // PX_SUPPORT_SSE doesn't exist
#include "linux/CmSimd_LINUX.h"
#elif defined(PX_X360)
#include "xbox360/CmSimd_XBOX.h"
#elif defined(PX_WII)
#include "wii/CmSimd_WII.h"
#else
#define NX_USE_PLAIN_SIMD	//Define this to always force usage of C++ implementation
#endif

namespace physx
{
namespace Cm
{

/*!
PxSimd provides an abstraction of platform specific SIMD functionality. The aim is to
take advantage on SIMD on other platforms eventually, with less effort. (and aid testing)

Currently there exists three implementations of it
1. C++ implementation using normal FPU instructions, only for testing.
2. SSE implementation for X86
3. VMX128 implementation for Xbox 360


Notice on aligned functions:
Functions ending with A is operating on aligned data. They expect the input parameters
to be 16-byte aligned. PxVec3s will be read/written as 4 component vectors!

NOTE: PxSimd mirrors PxcSimd. In future we should probably use the same source somehow.
*/

//! Plain C++ version
#ifdef NX_USE_PLAIN_SIMD
class PxSimd
#else
class PxSimdVerify
#endif
{
public:
	struct Vector4
	{
		Vector4()
		{}

		Vector4(PxReal x, PxReal y, PxReal z, PxReal w)
			: x(x), y(y), z(z), w(w)
		{}

		Vector4(PxU32 x, PxU32 y, PxU32 z, PxU32 w)
			: ux(x), uy(y), uz(z), uw(w)
		{}

		union
		{
			struct 
			{
				PxReal x, y, z, w;
			};
			struct 
			{
				PxU32 ux, uy, uz, uw;
			};
			PxReal f[4];
			PxU32	u[4];
		};
		
	};

	/* simd support */
	static PX_INLINE bool isSupported()
	{
		return true;
	}

	/* Setup any rounding modes etc */
	static PX_INLINE PxU32 setup()
	{
		return 0;
	}

	/* Reset any setup rounding modes etc*/
	static PX_INLINE void reset(PxU32 fromSetup)
	{
	}

	/* loads */	
	static PX_INLINE Vector4 load(const PxVec3& pxcVec)
	{
		return Vector4(pxcVec.x, pxcVec.y, pxcVec.z, 0);
	}

	static PX_INLINE Vector4 loadW1(const PxVec3& pxcVec)
	{
		return Vector4(pxcVec.x, pxcVec.y, pxcVec.z, 1.0f);
	}

	static PX_INLINE Vector4 load(const Gu::Plane& pxcPlane)
	{
		return Vector4(pxcPlane.normal.x, pxcPlane.normal.y, pxcPlane.normal.z, pxcPlane.d);
	}

	//A = aligned
	static PX_INLINE Vector4 loadA(const PxVec3& pxcVec)
	{
		return Vector4(pxcVec.x, pxcVec.y, pxcVec.z, 0);
	}

	static PX_INLINE Vector4 load(const PxReal& a)
	{
		return Vector4(a, 0, 0, 0);
	}

	static PX_INLINE Vector4 load(const PxU32& a)
	{
		return Vector4(a, 0, 0, 0);
	}

	static PX_INLINE Vector4 load(const PxU8& a)
	{
		return Vector4((PxU32)a, 0, 0, 0);
	}

	static PX_INLINE Vector4 load3(const PxReal* a)
	{
		return Vector4(a[0], a[1], a[2], 0.0f);
	}

	static PX_INLINE Vector4 load4(const PxReal* a)
	{
		return Vector4(a[0], a[1], a[2], a[3]);
	}

	/* stores */
	static PX_INLINE void store(PxVec3& pxcVec, const Vector4& simdVec)
	{
		pxcVec.x = simdVec.x;
		pxcVec.y = simdVec.y;
		pxcVec.z = simdVec.z;
	}

	//A = aligned
	static PX_INLINE void storeA(PxVec3& pxcVec, const Vector4& simdVec)
	{
		pxcVec.x = simdVec.x;
		pxcVec.y = simdVec.y;
		pxcVec.z = simdVec.z;
	}

	static PX_INLINE void storeA4(PxVec3& nxVec, const Vector4& simdVec)
	{
		PxReal *fPtr = (PxReal *)&nxVec;
		fPtr[0] = simdVec.x;
		fPtr[1] = simdVec.y;
		fPtr[2] = simdVec.z;
		fPtr[3] = simdVec.w;
	}

	static PX_INLINE void store(PxReal& a, const Vector4& simdVec)
	{
		a = simdVec.x;
	}

	static PX_INLINE void store(PxU32& a, const Vector4& simdVec)
	{
		a = simdVec.ux;
	}

	static PX_INLINE void store(PxU8& a, const Vector4& simdVec)
	{
		a = simdVec.ux;
	}

	/*
	Conversion operators
	*/

	static PX_INLINE Vector4 uintToFloat(const Vector4& a)
	{
		return Vector4((PxReal)a.ux,(PxReal)a.uy,(PxReal)a.uz,(PxReal)a.uw);
	}

	/* basic math */
	static PX_INLINE Vector4 add(const Vector4& a, const Vector4& b)
	{
		return Vector4(a.x+b.x, a.y+b.y, a.z+b.z, a.w+b.w);
	}

	static PX_INLINE Vector4 subtract(const Vector4& a, const Vector4& b)
	{
		return Vector4(a.x-b.x, a.y-b.y, a.z-b.z, a.w-b.w);
	}

	static PX_INLINE Vector4 multiply(const Vector4& a, const Vector4& b)
	{
		return Vector4(a.x*b.x, a.y*b.y, a.z*b.z, a.w*b.w);
	}

	static PX_INLINE Vector4 multiplyAdd(const Vector4& a, const Vector4& b, const Vector4& c)
	{
		return add(multiply(a, b), c);
	}

	static PX_INLINE Vector4 reciprocal(const Vector4& a)
	{
		return Vector4(1.0f/a.x, 1.0f/a.y, 1.0f/a.z, 1.0f/a.w);
	}

	static PX_INLINE Vector4 reciprocalEst(const Vector4& a)
	{
		return reciprocal(a);
	}

	static PX_INLINE Vector4 reciprocalSafe(const Vector4& a)
	{
		Vector4 mask = notEqual(a, zero());
		Vector4 result = reciprocal(a);
		return and4(result, mask);
	}

	static PX_INLINE Vector4 reciprocalSqrt(const Vector4& a)
	{
		return reciprocal(sqrt(a));
	}

	static PX_INLINE Vector4 pow(const Vector4& a, const Vector4& b)
	{
		return Vector4(Ps::pow(a.x,b.x), Ps::pow(a.y,b.y), Ps::pow(a.z,b.z), Ps::pow(a.w,b.w));
	}

	static PX_INLINE Vector4 sqrt(const Vector4& a)
	{
		return Vector4(PxSqrt(a.x),PxSqrt(a.y),PxSqrt(a.z),PxSqrt(a.w));
	}

	static PX_INLINE Vector4 sqrtSafe(const Vector4& a)
	{
		return sqrt(a);
	}

	static PX_INLINE Vector4 acos(const Vector4& a)
	{
		return Vector4(PxAcos(a.x), PxAcos(a.y), PxAcos(a.z), PxAcos(a.w));
	}

	// 3 component dot product
	static PX_INLINE Vector4 dot(const Vector4& a, const Vector4& b)
	{
		PxReal d = a.x*b.x + a.y*b.y + a.z*b.z;
		return Vector4(d,d,d,d);
	}

	static PX_INLINE Vector4 dot4(const Vector4& a, const Vector4& b)
	{
		PxReal d = a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w;
		return Vector4(d,d,d,d);
	}

	static PX_INLINE Vector4 cross(const Vector4& a, const Vector4& b)
	{
		return Vector4( a.y*b.z - a.z*b.y,
						a.z*b.x - a.x*b.z,
						a.x*b.y - a.y*b.x,
						0);
	}

	static PX_INLINE Vector4 abs(const Vector4& a)
	{
		return Vector4(PxAbs(a.x), PxAbs(a.y), PxAbs(a.z), PxAbs(a.w));
	}

	static PX_INLINE Vector4 magnitude(const Vector4& a)
	{
		return sqrt(dot(a, a));
	}

	// magnitude with safe zero magnitude handling.
	static PX_INLINE Vector4 magnitudeSafe(const Vector4& a)
	{
		Vector4 d = dot(a,a);
		Vector4 mask = notEqual(a, zero());
		Vector4 m = sqrt(d);

		return and4(mask, m);
	}

	static PX_INLINE Vector4 normalize(const Vector4& a)
	{
		Vector4 scale = reciprocalSqrt(dot(a,a));
		return multiply(a, scale);
	}

	static PX_INLINE Vector4 normalizeSafe(const Vector4& n, Vector4& nLength)
	{
		nLength = magnitudeSafe(n);
		return normalizeSafe(n);
	}

	static PX_INLINE Vector4 normalizeRecip(const Vector4& n, Vector4& rcpLength)
	{
		rcpLength = reciprocal(magnitude(n));
		return normalize(n);
	}

	static PX_INLINE Vector4 normalizeSafe(const Vector4& a)
	{
		Vector4 d = dot(a,a);
		Vector4 mask = notEqual(a, zero());
		
		Vector4 scale = reciprocalSqrt(d);
		Vector4 result = multiply(a, scale);

		return and4(mask, result);
	}

	static PX_INLINE Vector4 magnitudeSafe(const Vector4& n, Vector4& oneH)
	{
		Vector4 mag = magnitudeSafe(n);
		Vector4 mask = notEqual(mag, zero());

		oneH = and4(mask, reciprocal(mag));
		return mag;
	}

	// doesnt handle infinities, zero etc.
	static PX_INLINE void sqrtAndRcpSqrt(const Vector4& a, Vector4& sq, Vector4& rsq)
	{
		sq = Vector4(PxSqrt(a.x), PxSqrt(a.y), PxSqrt(a.z), PxSqrt(a.w));
		rsq = reciprocal(sq);
	}
	
	// doesnt handle infinities, zero etc.
	static PX_INLINE void sqrtAndRcpSqrtEst(const Vector4& a, Vector4& sq, Vector4& rsq)
	{
		sqrtAndRcpSqrt(a, sq, rsq);
	}

	static PX_INLINE Vector4 clamp(const Vector4& a,const Vector4& high, const Vector4& low)
	{
		return Vector4(
			PxClamp(a.x, low.x, high.x),
			PxClamp(a.y, low.y, high.y),
			PxClamp(a.z, low.z, high.z),
			PxClamp(a.w, low.w, high.w));
	}

	static PX_INLINE Vector4 zero()
	{
		return Vector4(0.0f,0.0f,0.0f,0.0f);
	}

	static PX_INLINE Vector4 one()
	{
		return Vector4(1.0f,1.0f,1.0f,1.0f);
	}

	static PX_INLINE Vector4 minusOne()
	{
		return Vector4(-1.0f, -1.0f, -1.0f, -1.0f);
	}

	static PX_INLINE Vector4 half()
	{
		return Vector4(0.5f, 0.5f, 0.5f, 0.5f);
	}

	static PX_INLINE Vector4 allSet()
	{
		return Vector4(0xffFFffFF,0xffFFffFF,0xffFFffFF,0xffFFffFF);
	}

	static PX_INLINE Vector4 eps()
	{
		const static Vector4 e (PX_EPS_REAL, PX_EPS_REAL, PX_EPS_REAL, PX_EPS_REAL);
		return e;
	}

	static PX_INLINE Vector4 eps6()
	{
		const static Vector4 e (1e-6f, 1e-6f, 1e-6f, 1e-6f);
		return e;
	}

	static PX_INLINE Vector4 floatMin()
	{
		return Vector4(-PX_MAX_REAL, -PX_MAX_REAL, -PX_MAX_REAL, -PX_MAX_REAL);
	}

	static PX_INLINE Vector4 floatMax()
	{
		return Vector4(PX_MAX_REAL, PX_MAX_REAL, PX_MAX_REAL, PX_MAX_REAL);
	}

	static PX_INLINE Vector4 signMask()
	{
		return Vector4(0x80000000, 0x80000000, 0x80000000, 0x80000000);
	}

	static PX_INLINE Vector4 rotate(const Vector4& basis0, const Vector4& basis1, const Vector4& basis2, const Vector4& other)
	{
		Vector4 X = splatX(other);
		Vector4 Y = splatY(other);
		Vector4 Z = splatZ(other);

		Vector4 result = multiply(basis0, X);
		result = multiplyAdd(basis1, Y, result);
		result = multiplyAdd(basis2, Z, result);

		return result;
	}

	static PX_INLINE Vector4 rotateInv(const Vector4& basis0, const Vector4& basis1, const Vector4& basis2, const Vector4& other)
	{
		return mergeXYZ(dot(basis0, other),
						dot(basis1, other),
						dot(basis2, other));
	}

	/* Min/max, per component */
	static PX_INLINE Vector4 maximum(const Vector4& a, const Vector4& b)
	{
		return Vector4( (a.x > b.x) ? a.x : b.x,
						(a.y > b.y) ? a.y : b.y,
						(a.z > b.z) ? a.z : b.z,
						(a.w > b.w) ? a.w : b.w);
	}

	static PX_INLINE Vector4 minimum(const Vector4& a, const Vector4& b)
	{
		return Vector4( (a.x < b.x) ? a.x : b.x,
						(a.y < b.y) ? a.y : b.y,
						(a.z < b.z) ? a.z : b.z,
						(a.w < b.w) ? a.w : b.w);
	}

	/* splatting */
	static PX_INLINE Vector4 splatX(const Vector4& a)
	{
		return Vector4(a.x, a.x, a.x, a.x);
	}

	static PX_INLINE Vector4 splatY(const Vector4& a)
	{
		return Vector4(a.y, a.y, a.y, a.y);
	}

	static PX_INLINE Vector4 splatZ(const Vector4& a)
	{
		return Vector4(a.z, a.z, a.z, a.z);
	}

	static PX_INLINE Vector4 splatW(const Vector4& a)
	{
		return Vector4(a.w, a.w, a.w, a.w);
	}

	/* permutations and selects */
	static PX_INLINE Vector4 mergeXYZ(const Vector4& vX, const Vector4& vY, const Vector4& vZ)
	{
		return Vector4(vX.x, vY.y, vZ.z, 0);
	}

	/* create a Vector4 from the x elements of 3 vectors(w undefined) */
	static PX_INLINE Vector4 columnX(const Vector4& vX,const Vector4& vY,const Vector4& vZ)
	{
		return Vector4(vX.x, vY.x, vZ.x, 0);
	}

	static PX_INLINE Vector4 columnY(const Vector4& vX,const Vector4& vY,const Vector4& vZ)
	{
		return Vector4(vX.y, vY.y, vZ.y, 0);
	}

	static PX_INLINE Vector4 columnZ(const Vector4& vX,const Vector4& vY,const Vector4& vZ)
	{
		return Vector4(vX.z, vY.z, vZ.z, 0);
	}


	static PX_INLINE Vector4 permute(const Vector4& a, const Vector4& b, 
		const PxU32 e0,const PxU32 e1,const PxU32 e2,const PxU32 e3)
	{
		//TODO RETHINK
		Vector4 result(0.0f,0.0f,0.0f,0.0f);
		return result;
	}

	static PX_INLINE Vector4 select(const Vector4& a, const Vector4& b, const Vector4& control)
	{
		return Vector4( (~control.ux & a.ux) | (control.ux & b.ux),
						(~control.uy & a.uy) | (control.uy & b.uy),
						(~control.uz & a.uz) | (control.uz & b.uz),
						(~control.uw & a.uw) | (control.uw & b.uw));
	}


	/* float comparisons (bitmasks)*/
	static PX_INLINE Vector4 equal(const Vector4& a, const Vector4& b)
	{
		return Vector4( (a.x == b.x) ? 0xFFFFFFFF : 0,
						(a.y == b.y) ? 0xFFFFFFFF : 0,
						(a.z == b.z) ? 0xFFFFFFFF : 0,
						(a.w == b.w) ? 0xFFFFFFFF : 0);
	}

	static PX_INLINE Vector4 notEqual(const Vector4& a, const Vector4& b)
	{
		return Vector4( (a.x != b.x) ? 0xFFFFFFFF : 0,
						(a.y != b.y) ? 0xFFFFFFFF : 0,
						(a.z != b.z) ? 0xFFFFFFFF : 0,
						(a.w != b.w) ? 0xFFFFFFFF : 0);
	}

	static PX_INLINE Vector4 less(const Vector4& a, const Vector4& b)
	{
		return Vector4( (a.x < b.x) ? 0xFFFFFFFF : 0,
						(a.y < b.y) ? 0xFFFFFFFF : 0,
						(a.z < b.z) ? 0xFFFFFFFF : 0,
						(a.w < b.w) ? 0xFFFFFFFF : 0);
	}

	static PX_INLINE Vector4 greater(const Vector4& a, const Vector4& b)
	{
		return Vector4( (a.x > b.x) ? 0xFFFFFFFF : 0,
						(a.y > b.y) ? 0xFFFFFFFF : 0,
						(a.z > b.z) ? 0xFFFFFFFF : 0,
						(a.w > b.w) ? 0xFFFFFFFF : 0);
	}

	static PX_INLINE Vector4 lessEqual(const Vector4& a, const Vector4& b)
	{
		return Vector4( (a.x <= b.x) ? 0xFFFFFFFF : 0,
						(a.y <= b.y) ? 0xFFFFFFFF : 0,
						(a.z <= b.z) ? 0xFFFFFFFF : 0,
						(a.w <= b.w) ? 0xFFFFFFFF : 0);
	}

	static PX_INLINE Vector4 greaterEqual(const Vector4& a, const Vector4& b)
	{
		return Vector4( (a.x >= b.x) ? 0xFFFFFFFF : 0,
						(a.y >= b.y) ? 0xFFFFFFFF : 0,
						(a.z >= b.z) ? 0xFFFFFFFF : 0,
						(a.w >= b.w) ? 0xFFFFFFFF : 0);
	}

	/* float comparisons (bool, for jumps)- first 3 components */
	/* Tests if _all_ components(3) are equal etc */
	static PX_INLINE Ps::IntBool equalBool(const Vector4& a, const Vector4& b)
	{
		return (a.x == b.x) && (a.y == b.y) && (a.z == b.z);
	}

	static PX_INLINE Ps::IntBool notEqualBool(const Vector4& a, const Vector4& b)
	{
		return (a.x != b.x) && (a.y != b.y) && (a.z != b.z);
	}

	static PX_INLINE Ps::IntBool lessBool(const Vector4& a, const Vector4& b)
	{
		return (a.x < b.x) && (a.y < b.y) && (a.z < b.z);
	}

	static PX_INLINE Ps::IntBool greaterBool(const Vector4& a, const Vector4& b)
	{
		return (a.x > b.x) && (a.y > b.y) && (a.z > b.z);
	}

	static PX_INLINE Ps::IntBool lessEqualBool(const Vector4& a, const Vector4& b)
	{
		return (a.x <= b.x) && (a.y <= b.y) && (a.z <= b.z);
	}

	static PX_INLINE Ps::IntBool greaterEqualBool(const Vector4& a, const Vector4& b)
	{
		return (a.x >= b.x) && (a.y >= b.y) && (a.z >= b.z);
	}
	/* float comparisons, bool x component only */

	static PX_INLINE Ps::IntBool equalXBool(const Vector4& a, const Vector4& b)
	{
		return (a.x == b.x);
	}

	static PX_INLINE Ps::IntBool notEqualXBool(const Vector4& a, const Vector4& b)
	{
		return (a.x != b.x);
	}

	static PX_INLINE Ps::IntBool lessXBool(const Vector4& a, const Vector4& b)
	{
		return (a.x < b.x);
	}

	static PX_INLINE Ps::IntBool greaterXBool(const Vector4& a, const Vector4& b)
	{
		return (a.x > b.x);
	}

	static PX_INLINE Ps::IntBool lessEqualXBool(const Vector4& a, const Vector4& b)
	{
		return (a.x <= b.x);
	}

	static PX_INLINE Ps::IntBool greaterEqualXBool(const Vector4& a, const Vector4& b)
	{
		return (a.x >= b.x);
	}

	/* int comparisons (bitmasks) */
	static PX_INLINE Vector4 intNotEqual(const Vector4& a, const Vector4& b)
	{
		return Vector4( (a.ux != b.ux) ? 0xFFFFFFFF : 0,
						(a.uy != b.uy) ? 0xFFFFFFFF : 0,
						(a.uz != b.uz) ? 0xFFFFFFFF : 0,
						(a.uw != b.uw) ? 0xFFFFFFFF : 0);
	}

	/* int comparisons bool (first 3 components)*/
	static PX_INLINE Ps::IntBool intNotEqualBool(const Vector4& a, const Vector4& b)
	{
		return (a.ux != b.ux) || (a.uy != b.uy) || (a.uz != b.uz);
	}

	/* bitwise logical operations */
	static PX_INLINE Vector4 and4(const Vector4& a, const Vector4& b)
	{
		return Vector4( a.ux & b.ux,
						a.uy & b.uy,
						a.uz & b.uz,
						a.uw & b.uw);
	}

	static PX_INLINE Vector4 or4(const Vector4& a, const Vector4& b)
	{
		return Vector4( a.ux | b.ux,
						a.uy | b.uy,
						a.uz | b.uz,
						a.uw | b.uw);
	}

	static PX_INLINE Vector4 xor4(const Vector4& a, const Vector4& b)
	{
		return Vector4(a.ux ^ b.ux, a.uy ^ b.uy, a.uz ^ b.uz, a.uw ^ b.uw);
	}


	static PX_INLINE Vector4 andNot(const Vector4& a, const Vector4& b)
	{
		return Vector4( a.ux & ~b.ux,
						a.uy & ~b.uy,
						a.uz & ~b.uz,
						a.uw & ~b.uw);
	}

private:
#ifdef NX_USE_PLAIN_SIMD
	PxSimd()
#else
	PxSimdVerify()
#endif
	{}
};


} // namespace Cm

}

#endif

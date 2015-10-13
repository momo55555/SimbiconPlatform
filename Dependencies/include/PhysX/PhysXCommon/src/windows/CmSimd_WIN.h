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
#ifndef PX_SIMD_WIN_H
#define PX_SIMD_WIN_H

namespace physx
{
namespace shdfnd3
{

class PxSimd
{
public:
	typedef __m128 Vector4;

	static PX_INLINE bool isSupported()
	{
		//static bool sseSupport = PxcCPUSupportSSE();
		//return sseSupport;

		PX_ASSERT(!"CPU detection not supported by PxSimd");
		return true;
	}

	/* Setup any rounding modes etc */
	static PX_INLINE PxU32 setup()
	{
		PxU32 oldMXCSR = _mm_getcsr();
		_mm_setcsr(oldMXCSR | 0x8040); // set DAZ and FZ bits
		return oldMXCSR;
	}

	/* Reset any setup rounding modes etc*/
	static PX_INLINE void reset(PxU32 fromSetup)
	{
		_mm_setcsr(fromSetup);
	}

	/* loads */	
	static PX_INLINE Vector4 load(const PxVec3 &pxcVec)
	{
		return _mm_loadu_ps(&pxcVec.x);
	}

	//A = aligned
	static PX_INLINE Vector4 loadA(const PxVec3 &pxcVec)
	{
		return _mm_load_ps(&pxcVec.x);
	}

	static PX_INLINE Vector4 load(const PxReal &a)
	{
		return _mm_load_ss(&a);
	}

	static PX_INLINE Vector4 load(const PxU32 &a)
	{
		return _mm_load_ss((PxReal*)&a);
	}

	static PX_INLINE Vector4 load(const PxU8 &a)
	{
		PxU32 u = (PxU32)a;
		return _mm_load_ss((PxReal*)&u);
	}

	/* stores */
	static PX_INLINE void store(PxVec3 &pxcVec, const Vector4 &simdVec)
	{
		//Ugly, cannot write outside pxcVec
		_mm_store_ss(&pxcVec.x, simdVec); //store x
		_mm_store_ss(&pxcVec.y, _mm_shuffle_ps(simdVec, simdVec, _MM_SHUFFLE(1,1,1,1))); //store z
		_mm_store_ss(&pxcVec.z, _mm_shuffle_ps(simdVec, simdVec, _MM_SHUFFLE(2,2,2,2))); //store z
	}

	//A = aligned
	static PX_INLINE void storeA(PxVec3 &pxcVec, const Vector4 &simdVec)
	{
		_mm_store_ps(&pxcVec.x, simdVec);
	}

	static PX_INLINE void store(PxReal &a, const Vector4 &simdVec)
	{
		_mm_store_ss(&a, simdVec);
	}

	static PX_INLINE void store(PxU32 &a, const Vector4 &simdVec)
	{
		_mm_store_ss((PxReal*)&a, simdVec);
	}

	static PX_INLINE void store(PxU8 &a, const Vector4 &simdVec)
	{
		PxU32 u;
		_mm_store_ss((PxReal*)&u, simdVec);
		a = u;
	}

	/* basic math */
	static PX_INLINE Vector4 add(const Vector4 &a, const Vector4 &b)
	{
		return _mm_add_ps(a, b);
	}

	static PX_INLINE Vector4 subtract(const Vector4 &a, const Vector4 &b)
	{
		return _mm_sub_ps(a, b);
	}

	static PX_INLINE Vector4 multiply(const Vector4 &a, const Vector4 &b)
	{
		return _mm_mul_ps(a, b);
	}

	static PX_INLINE Vector4 reciprocal(const Vector4 &a)
	{
		return _mm_rcp_ps(a);
	}

	static PX_INLINE Vector4 reciprocalSqrt(const Vector4 &a)
	{
		return _mm_rsqrt_ps(a);
	}

	// 3 component dot product
	static PX_INLINE Vector4 dot(const Vector4 &a, const Vector4 &b)
	{
		__m128 dot1 = _mm_mul_ps(a, b);
		__m128 shuf1 = _mm_shuffle_ps(dot1, dot1, _MM_SHUFFLE(3,0,2,1));
		__m128 shuf2 = _mm_shuffle_ps(dot1, dot1, _MM_SHUFFLE(3,1,0,2));

		return _mm_add_ps(shuf2, _mm_add_ps(dot1,shuf1));
	}

	static PX_INLINE Vector4 dot4(const Vector4 &a, const Vector4 &b)
	{
		__m128 dot1 = _mm_mul_ps(a, b);										//x,y,z,w
		__m128 shuf1 = _mm_shuffle_ps(dot1, dot1, _MM_SHUFFLE(2,1,0,3));	//w,x,y,z
		__m128 shuf2 = _mm_shuffle_ps(dot1, dot1, _MM_SHUFFLE(1,0,3,2));	//z,w,x,y
		__m128 shuf3 = _mm_shuffle_ps(dot1, dot1, _MM_SHUFFLE(0,3,2,1));	//y,z,w,x

		return _mm_add_ps(_mm_add_ps(shuf2, shuf3), _mm_add_ps(dot1,shuf1));
	}

	static PX_INLINE Vector4 cross(const Vector4 &a, const Vector4 &b)
	{
		__m128 l1 = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 2, 1)); //y,z,x,w
		__m128 l2 = _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 1, 0, 2)); //z,x,y,w

		__m128 r1 = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 1, 0, 2)); //z,x,y,w
		__m128 r2 = _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 0, 2, 1)); //y,z,x,w

		return _mm_sub_ps(_mm_mul_ps(l1, l2), _mm_mul_ps(r1,r2));
	}

	static PX_INLINE Vector4 abs(const Vector4 &a)
	{
		__declspec(align(16)) const static PxU32 absMask[4] = {0x7fFFffFF, 0x7fFFffFF, 0x7fFFffFF, 0x7fFFffFF};
		return _mm_and_ps(a, _mm_load_ps((PxReal*)absMask));
		//return _mm_max_ps(a, _mm_sub_ps(_mm_setzero_ps(), a));
	}

	static PX_INLINE Vector4 zero()
	{
		return _mm_setzero_ps();
	}

	static PX_INLINE Vector4 one()
	{
		return _mm_set_ps1(1.0f);
	}

	static PX_INLINE Vector4 allSet()
	{
		Vector4 a = zero();
		return equal(a,a);
	}

	static PX_INLINE Vector4 eps()
	{
		return _mm_set_ps1(PX_EPS_REAL);
	}

	static PX_INLINE Vector4 eps6()
	{
		return _mm_set_ps1(1e-6f);
	}

	/* Min/max, per component */
	static PX_INLINE Vector4 maximum(const Vector4 &a, const Vector4 &b)
	{
		return _mm_max_ps(a, b);
	}

	static PX_INLINE Vector4 minimum(const Vector4 &a, const Vector4 &b)
	{
		return _mm_min_ps(a, b);
	}


	/* splatting */
	static PX_INLINE Vector4 splatX(const Vector4 &a)
	{
		return _mm_shuffle_ps(a, a, _MM_SHUFFLE(0,0,0,0));
	}

	static PX_INLINE Vector4 splatY(const Vector4 &a)
	{
		return _mm_shuffle_ps(a, a, _MM_SHUFFLE(1,1,1,1));
	}

	static PX_INLINE Vector4 splatZ(const Vector4 &a)
	{
		return _mm_shuffle_ps(a, a, _MM_SHUFFLE(2,2,2,2));
	}

	static PX_INLINE Vector4 splatW(const Vector4 &a)
	{
		return _mm_shuffle_ps(a, a, _MM_SHUFFLE(3,3,3,3));
	}

	/* permutations and selects */
	static PX_INLINE Vector4 mergeXYZ(const Vector4 &vX, const Vector4 &vY, const Vector4 &vZ)
	{
		__m128 vxy = _mm_move_ss(vY, vX);							//vX.x , vY.y, vY.z, vY.w;
		return _mm_shuffle_ps(vxy, vZ, _MM_SHUFFLE(3,2,1,0));		//vxy.X, vxy.Y, vZ.z, vZ.w
	}

	/* create a Vector4 from the x elements of 3 vectors(w undefined) */
	static PX_INLINE Vector4 columnX(const Vector4 &vX,const Vector4 &vY,const Vector4 &vZ)
	{
		__m128 vxy1 = _mm_shuffle_ps(vX, vY, _MM_SHUFFLE(0,0,0,0));		//vX.x, vX.x, vY.x, vY.x
		__m128 vxy = _mm_shuffle_ps(vxy1, vZ, _MM_SHUFFLE(0,0,2,0));	//vX.x, vY.x, vZ.x, vZ.x
		return vxy;
	}

	static PX_INLINE Vector4 columnY(const Vector4 &vX,const Vector4 &vY,const Vector4 &vZ)
	{
		__m128 vxy1 = _mm_shuffle_ps(vX, vY, _MM_SHUFFLE(1,1,1,1));		//vX.y, vX.y, vY.y, vY.y
		__m128 vxy = _mm_shuffle_ps(vxy1, vZ, _MM_SHUFFLE(1,1,2,0));	//vX.y, vY.y, vZ.y, vZ.y
		return vxy;
	}

	static PX_INLINE Vector4 columnZ(const Vector4 &vX,const Vector4 &vY,const Vector4 &vZ)
	{
		__m128 vxy1 = _mm_shuffle_ps(vX, vY, _MM_SHUFFLE(2,2,2,2));		//vX.z, vX.z, vY.z, vY.z
		__m128 vxy = _mm_shuffle_ps(vxy1, vZ, _MM_SHUFFLE(2,2,2,0));	//vX.z, vY.z, vZ.z, vZ.z
		return vxy;
	}


	static PX_INLINE Vector4 permute(const Vector4 &a, const Vector4 &b, 
		const PxU32 e0,const PxU32 e1,const PxU32 e2,const PxU32 e3)
	{
		//TODO RETHINK
		ASSERT(!"Not implemented!");	
		Vector4 result;
		return result;
	}

	static PX_INLINE Vector4 select(const Vector4 &a, const Vector4 &b, const Vector4 &control)
	{
		return _mm_or_ps(_mm_andnot_ps(control, a), _mm_and_ps(control, b));
	}


	/* float comparisons (bitmasks)*/
	static PX_INLINE Vector4 equal(const Vector4 &a, const Vector4 &b)
	{
		return _mm_cmpeq_ps(a, b);
	}

	static PX_INLINE Vector4 notEqual(const Vector4 &a, const Vector4 &b)
	{
		return _mm_cmpneq_ps(a, b);
	}

	static PX_INLINE Vector4 less(const Vector4 &a, const Vector4 &b)
	{
		return _mm_cmplt_ps(a,b);
	}

	static PX_INLINE Vector4 greater(const Vector4 &a, const Vector4 &b)
	{
		return _mm_cmpgt_ps(a,b);
	}

	static PX_INLINE Vector4 lessEqual(const Vector4 &a, const Vector4 &b)
	{
		return _mm_cmple_ps(a, b);
	}

	static PX_INLINE Vector4 greaterEqual(const Vector4 &a, const Vector4 &b)
	{
		return _mm_cmpge_ps(a,b);
	}

	/* float comparisons (bool, for jumps)- first 3 components */
	/* Tests if _all_ components(3) are equal etc */
	static PX_INLINE Ps::IntBool equalBool(const Vector4 &a, const Vector4 &b)
	{	
		__m128 mask = equal(a,b);
		return allBitsSet(mask);
	}

	static PX_INLINE Ps::IntBool notEqualBool(const Vector4 &a, const Vector4 &b)
	{
		__m128 mask = notEqual(a,b);
		return allBitsSet(mask);
	}

	static PX_INLINE Ps::IntBool lessBool(const Vector4 &a, const Vector4 &b)
	{
		__m128 mask = less(a,b);
		return allBitsSet(mask);
	}

	static PX_INLINE Ps::IntBool greaterBool(const Vector4 &a, const Vector4 &b)
	{
		__m128 mask = greater(a,b);
		return allBitsSet(mask);
	}

	static PX_INLINE Ps::IntBool lessEqualBool(const Vector4 &a, const Vector4 &b)
	{
		__m128 mask = lessEqual(a,b);
		return allBitsSet(mask);
	}

	static PX_INLINE Ps::IntBool greaterEqualBool(const Vector4 &a, const Vector4 &b)
	{
		__m128 mask = greaterEqual(a,b);
		return allBitsSet(mask);
	}
	/* float comparisons bool, X component only */
	static PX_INLINE Ps::IntBool equalXBool(const Vector4 &a, const Vector4 &b)
	{
		return _mm_comieq_ss(a, b);
	}

	static PX_INLINE Ps::IntBool notEqualXBool(const Vector4 &a, const Vector4 &b)
	{
		return _mm_comineq_ss(a, b);
	}

	static PX_INLINE Ps::IntBool lessXBool(const Vector4 &a, const Vector4 &b)
	{
		return _mm_comilt_ss(a, b);
	}

	static PX_INLINE Ps::IntBool greaterXBool(const Vector4 &a, const Vector4 &b)
	{
		return _mm_comigt_ss(a, b);
	}

	static PX_INLINE Ps::IntBool lessEqualXBool(const Vector4 &a, const Vector4 &b)
	{
		return _mm_comile_ss(a, b);
	}

	static PX_INLINE Ps::IntBool greaterEqualXBool(const Vector4 &a, const Vector4 &b)
	{
		return _mm_comige_ss(a, b);
	}

	/* int comparisons (bitmasks) */
	static PX_INLINE Vector4 intNotEqual(const Vector4 &a, const Vector4 &b)
	{
		//TODO: This have to be done in integer registers for now.. find a solution!
		__m128 mask = _mm_andnot_ps(a, b);
		__declspec(align(16)) PxU32 intRegs[4];
		_mm_store_ps((PxReal*)&intRegs[0], mask);

		if(intRegs[0] != 0) intRegs[0] = 0xFFffFFff;
		if(intRegs[1] != 0) intRegs[1] = 0xFFffFFff;
		if(intRegs[2] != 0) intRegs[2] = 0xFFffFFff;
		if(intRegs[3] != 0) intRegs[3] = 0xFFffFFff;
		
		return _mm_load_ps((PxReal*)&intRegs[0]);
	}

	/* int comparisons bool (first 3 components)*/
	static PX_INLINE Ps::IntBool intNotEqualBool(const Vector4 &a, const Vector4 &b)
	{
		__m128 mask = intNotEqual(a,b);
		return allBitsSet(mask);//This is wrong! (should be anyBitsSet)
	}

	/* bitwise logical operations */
	static PX_INLINE Vector4 and4(const Vector4 &a, const Vector4 &b)
	{
		return _mm_and_ps(a, b);
	}

	static PX_INLINE Vector4 or4(const Vector4 &a, const Vector4 &b)
	{
		return _mm_or_ps(a, b);
	}

	static PX_INLINE Vector4 andNot(const Vector4 &a, const Vector4 &b)
	{
		return _mm_andnot_ps(b, a);
	}

private:

	//Helper
	static PX_INLINE Ps::IntBool allBitsSet(const Vector4& mask)
	{
		//and them together
		__m128 shuf1 = _mm_shuffle_ps(mask, mask, _MM_SHUFFLE(3,0,2,1));	//y, z, x, w
		__m128 shuf2 = _mm_shuffle_ps(mask, mask, _MM_SHUFFLE(3,1,0,2));	//z, x, y, w

		__m128 finalMask = _mm_and_ps(mask, _mm_and_ps(shuf1, shuf2));
		__m128 eq = _mm_andnot_ps(finalMask, _mm_set_ps1(1.0f));

		return _mm_ucomieq_ss(eq, _mm_setzero_ps());
	}

	PxSimd(){}
};

} // namespace shdfnd3
} // namespace physx

#endif
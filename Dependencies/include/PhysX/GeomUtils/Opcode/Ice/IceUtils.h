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
#ifndef ICEUTILS_H
#define ICEUTILS_H

#include "PxSimpleTypes.h"
#include "PxPhysXCommon.h"
#include "PxAssert.h"

namespace physx
{
namespace Ice
{

	//! Even faster?
	PX_INLINE PxU32	CountBits2(PxU32 bits)
	{
		bits = bits - ((bits >> 1) & 0x55555555);
		bits = ((bits >> 2) & 0x33333333) + (bits & 0x33333333);
		bits = ((bits >> 4) + bits) & 0x0F0F0F0F;
		return (bits * 0x01010101) >> 24;
	}

	//! Test to see if a number is an exact power of two (from Steve Baker's Cute Code Collection)
	PX_INLINE bool	IsPowerOfTwo(PxU32 n)				{ return ((n&(n-1))==0);					}

	//! Classic XOR swap (from Steve Baker's Cute Code Collection)
	//! x ^= y;		/* x' = (x^y) */
	//! y ^= x;		/* y' = (y^(x^y)) = x */
	//! x ^= y;		/* x' = (x^y)^x = y */
	PX_INLINE void	Swap(PxU32& x, PxU32& y)			{ x ^= y; y ^= x; x ^= y;					}

	//! Little/Big endian (from Steve Baker's Cute Code Collection)
	//!
	//! Extra comments by Kenny Hoff:
	//! Determines the byte-ordering of the current machine (little or big endian)
	//! by setting an integer value to 1 (so least significant bit is now 1); take
	//! the address of the int and cast to a byte pointer (treat integer as an
	//! array of four bytes); check the value of the first byte (must be 0 or 1).
	//! If the value is 1, then the first byte least significant byte and this
	//! implies LITTLE endian. If the value is 0, the first byte is the most
	//! significant byte, BIG endian. Examples:
	//!      integer 1 on BIG endian: 00000000 00000000 00000000 00000001
	//!   integer 1 on LITTLE endian: 00000001 00000000 00000000 00000000
	//!---------------------------------------------------------------------------
	//! int IsLittleEndian()	{ int x=1;	return ( ((char*)(&x))[0] );	}
	PX_INLINE char	LittleEndian()						{ int i = 1; return *((char*)&i);			}

	//!< Alternative abs function
	PX_INLINE PxU32	abs_(PxI32 x)						{ PxI32 y= x >> 31;	return (x^y)-y;		}

	// "Integer Minimum or Maximum
	// Given 2's complement integer values x and y, the minimum can be computed without any branches as
	// x+(((y-x)>>(WORDBITS-1))&(y-x)).
	// Logically, this works because the shift by (WORDBITS-1) replicates the sign bit to create a mask
	// -- be aware, however, that the C language does not require that shifts are signed even if their
	// operands are signed, so there is a potential portability problem. Additionally, one might think
	// that a shift by any number greater than or equal to WORDBITS would have the same effect, but many
	// instruction sets have shifts that behave strangely when such shift distances are specified. 
	// Of course, maximum can be computed using the same trick:
	// x-(((x-y)>>(WORDBITS-1))&(x-y))."

	//!< Alternative minimum function
	PX_INLINE PxI32	min_(PxI32 a, PxI32 b)			{ PxI32 delta = b-a;	return a + (delta&(delta>>31));	}
	//!< Alternative maximum function
	PX_INLINE PxI32	max_(PxI32 a, PxI32 b)			{ PxI32 delta = a-b;	return a - (delta&(delta>>31));	}

	/*
	"Just call it repeatedly with various input values and always with the same variable as "memory".
	The sharpness determines the degree of filtering, where 0 completely filters out the input, and 1
	does no filtering at all.

	I seem to recall from college that this is called an IIR (Infinite Impulse Response) filter. As opposed
	to the more typical FIR (Finite Impulse Response).

	Also, I'd say that you can make more intelligent and interesting filters than this, for example filters
	that remove wrong responses from the mouse because it's being moved too fast. You'd want such a filter
	to be applied before this one, of course."

	(JCAB on Flipcode)
	*/
	PX_INLINE float	FeedbackFilter(float val, float& memory, float sharpness)
	{
		PX_ASSERT(sharpness>=0.0f && sharpness<=1.0f && "Invalid sharpness value in feedback filter");
				if(sharpness<0.0f)	sharpness = 0.0f;
		else	if(sharpness>1.0f)	sharpness = 1.0f;
		return memory = val * sharpness + memory * (1.0f - sharpness);
	}

	// Generic functions
	template<class Type> PX_INLINE void TSwap(Type& a, Type& b)								{ const Type c = a; a = b; b = c;			}
	template<class Type> PX_INLINE Type TClamp(const Type& x, const Type& lo, const Type& hi)	{ return ((x<lo) ? lo : (x>hi) ? hi : x);	}

	template<class Type> PX_INLINE void TSort(Type& a, Type& b)
	{
		if(a>b)	TSwap(a, b);
	}

	template<class Type> PX_INLINE void TSort(Type& a, Type& b, Type& c)
	{
		if(a>b)	TSwap(a, b);
		if(b>c)	TSwap(b, c);
		if(a>b)	TSwap(a, b);
		if(b>c)	TSwap(b, c);
	}

	// Prevent nasty user-manipulations (strategy borrowed from Charles Bloom)
//	#define PREVENT_COPY(curclass)	void operator = (const curclass& object)	{	ASSERT(!"Bad use of operator =");	}
	// ... actually this is better !
	#define PREVENT_COPY(cur_class)	private: cur_class(const cur_class& object);	cur_class& operator=(const cur_class& object);

	// Compute implicit coords from an index:
	// The idea is to get back 2D coords from a 1D index.
	// For example:
	//
	// 0		1		2	...	nbu-1
	// nbu		nbu+1	i	...
	//
	// We have i, we're looking for the equivalent (u=2, v=1) location.
	//		i = u + v*nbu
	// <=>	i/nbu = u/nbu + v
	// Since 0 <= u < nbu, u/nbu = 0 (integer)
	// Hence: v = i/nbu
	// Then we simply put it back in the original equation to compute u = i - v*nbu
	PX_INLINE void Compute2DCoords(PxU32& u, PxU32& v, PxU32 i, PxU32 nbu)
	{
		v = i / nbu;
		u = i - (v * nbu);
	}

	// "Next Largest Power of 2
	// Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
	// that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
	// the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
	// largest power of 2. For a 32-bit value:"
	PX_FORCE_INLINE PxU32	NextPowerOfTwo(PxU32 x)
	{
		x |= (x >> 1);
		x |= (x >> 2);
		x |= (x >> 4);
		x |= (x >> 8);
		x |= (x >> 16);
		return x+1;
	}

};

}

#endif // ICEUTILS_H

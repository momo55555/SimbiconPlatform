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


#ifndef PX_FOUNDATION_PSUTILITIES_H
#define PX_FOUNDATION_PSUTILITIES_H

#include "PxVec3.h"
#include "Ps.h"
#include "PsIntrinsics.h"

namespace physx
{
namespace shdfnd3
	{

	// PT: checked casts
	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxU16 to16(PxU32 value)
	{
		PX_ASSERT(value<=0xffff);
		return PxU16(value);
	}
	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxU8 to8(PxU16 value)
	{
		PX_ASSERT(value<=0xff);
		return PxU8(value);
	}
	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxU8 to8(PxU32 value)
	{
		PX_ASSERT(value<=0xff);
		return PxU8(value);
	}

	template<class T>
	PX_CUDA_CALLABLE static PX_INLINE void swap(T& x, T& y)
	{
		T tmp = x;
		x = y;
		y = tmp;
	}


/*!
Get number of elements in array
*/
template <typename T, size_t N>
char (&ArraySizeHelper(T (&array)[N]))[N];
#define PX_ARRAY_SIZE(_array) (sizeof(physx::shdfnd3::ArraySizeHelper(_array)))

/*!
Sort two elements using operator<

On return x will be the smaller of the two
*/
template<class T>
PX_CUDA_CALLABLE static PX_FORCE_INLINE void order(T& x, T& y)
{
	if(y < x)
		swap(x, y);
}

// most architectures can do predication on real comparisons, and on VMX, it matters

PX_CUDA_CALLABLE static PX_FORCE_INLINE void order(PxReal& x, PxReal& y)
{
	PxReal newX = PxMin(x, y);
	PxReal newY = PxMax(x, y);
	x=newX;
	y=newY;
}



	/*!
	Sort two elements using operator< and also keep order
	of any extra data
	*/
	template<class T, class E1>
	PX_CUDA_CALLABLE static PX_FORCE_INLINE void order(T& x, T& y, E1& xe1, E1& ye1)
	{
		if(y < x)
		{
			swap(x, y);
			swap(xe1, ye1);
		}
	}



	PX_INLINE void debugBreak()
	{
#if defined PX_WINDOWS
			__debugbreak();
#elif defined PX_LINUX
			asm ("int $3");
#elif defined PX_GNUC
		__builtin_trap();
#else
		PX_ASSERT(false);
#endif
	}

	bool checkValid(const float&);
	bool checkValid(const PxVec3&);
	bool checkValid(const PxQuat&);
	bool checkValid(const PxMat33&);
	bool checkValid(const PxTransform&);
	bool checkValid(const char*);

	PX_CUDA_CALLABLE static PX_INLINE PxI32 getPadding2(size_t value, PxU32 alignment)
	{
		const PxI32 mask = alignment-1;
		const PxI32 overhead = PxI32(value) & mask;
		return (alignment - overhead) & mask;
	}

	// PT: "After doing a dcbz128, there is a delay of about 140 cycles before writes to that cache line can proceed without stalling.
	// This is much faster than an L2 cache miss, but for ideal performance, it is best to avoid this stall by doing the cache-line
	// zeroing a few cache lines ahead of where you are writing."
	PX_FORCE_INLINE void invalidateCache(void* PX_RESTRICT voidPtr, PxI32 size)
	{
#ifdef PX_X360
		PxU8* PX_RESTRICT ptr = reinterpret_cast<PxU8*>(voidPtr);
		const PxI32 padding = getPadding2(size_t(ptr), 128);
		const PxI32 sizeToCover = size - padding;
		if(sizeToCover>=128)
		{
			PxU8* ptr128 = ptr + padding;
			PxU32 nb128 = sizeToCover/128;
			while(nb128--)
			{
//				Ps::memZero128(ptr128);
				physx::shdfnd3::memZero128(ptr128);
				ptr128 += 128;
			}
		}
#else
		(void)voidPtr;
		(void)size;
#endif
	}


} // namespace shdfnd3
} // namespace physx

#define PX_STRINGIZE_HELPER(X)				#X
#define PX_STRINGIZE(X)						PX_STRINGIZE_HELPER(X)

#define PX_CONCAT_HELPER(X,Y)				X##Y
#define PX_CONCAT(X,Y)						PX_CONCAT_HELPER(X,Y)

#ifdef PX_CHECKED
#define PX_CHECK_MSG(exp, msg)				(!!(exp) || (physx::shdfnd3::getFoundation().error(physx::PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, msg), 0) )
#else	
#define PX_CHECK_MSG(exp, msg)				((void)0)
#endif

#define PX_CHECK(exp)						PX_CHECK_MSG(exp, #exp)
#define PX_CHECK_AND_RETURN(exp,msg)		{ if(!(exp)) { PX_CHECK_MSG(exp, msg); return; } }
#define PX_CHECK_AND_RETURN_NULL(exp,msg)	{ if(!(exp)) { PX_CHECK_MSG(exp, msg); return 0; } }
#define PX_CHECK_AND_RETURN_VAL(exp,msg,r)	{ if(!(exp)) { PX_CHECK_MSG(exp, msg); return r; } }

#ifdef PX_VC
// VC compiler defines __FUNCTION__ as a string literal so it is possible to concatenate it with another string
// Example: #define PX_CHECK_VALID(x)		PX_CHECK_MSG(physx::shdfnd3::checkValid(x), __FUNCTION__ ": parameter invalid!")
#define PX_CHECK_VALID(x)					PX_CHECK_MSG(physx::shdfnd3::checkValid(x), __FUNCTION__)
#elif defined PX_GNUC
// GCC compiler defines __FUNCTION__ as a variable, hence, it is NOT possible concatenate an additional string to it
// In GCC, __FUNCTION__ only returns the function name, using __PRETTY_FUNCTION__ will return the full function definition
#define PX_CHECK_VALID(x)					PX_CHECK_MSG(physx::shdfnd3::checkValid(x), __PRETTY_FUNCTION__)
#else
// Generic macro for other compilers
#define PX_CHECK_VALID(x)					PX_CHECK_MSG(physx::shdfnd3::checkValid(x), __FUNCTION__)
#endif

#endif

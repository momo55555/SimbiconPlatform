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


#ifndef PX_FOUNDATION_PSFASTMEMORY_H
#define PX_FOUNDATION_PSFASTMEMORY_H

#include "Ps.h"

namespace physx
{
namespace shdfnd3
{
	PX_DEPRECATED PX_INLINE void fastMemzero(void* addr, size_t size)		
	{ 
		memset(addr, 0, size);	
	}

	PX_DEPRECATED PX_INLINE void* fastMemset(void* dest, int c, size_t count)
	{
		return memset(dest,c,count);
	}

	PX_DEPRECATED PX_INLINE void* fastMemcpy(void* dest, const void* src, size_t count)
	{
		return memcpy(dest,src,count);
	}

	PX_DEPRECATED PX_INLINE void* fastMemmove(void* dest, const void* src, size_t count)
	{
		return memmove(dest,src,count);
	}
	
	PX_DEPRECATED PX_INLINE void gatherStrided(const void* src, void* dst, PxU32 nbElem, PxU32 elemSize, PxU32 stride)
	{
		const PxU8* s = (const PxU8*)src;
		PxU8* d = (PxU8*)dst;
		while(nbElem--)
		{
			memcpy(d, s, elemSize);
			d += elemSize;
			s += stride;
		}
	}
} // namespace shdfnd3
} // namespace physx

#endif


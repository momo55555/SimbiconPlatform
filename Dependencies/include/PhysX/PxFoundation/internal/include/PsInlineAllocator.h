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


#ifndef PX_FOUNDATION_PSINLINEALLOCATOR_H
#define PX_FOUNDATION_PSINLINEALLOCATOR_H

#include "PsUserAllocated.h"

namespace physx
{
namespace shdfnd3
{
	// this is used by the array class to allocate some space for a small number
	// of objects along with the metadata
	template<PxU32 N, typename BaseAllocator>
	class InlineAllocator : private BaseAllocator
	{
	public:
		InlineAllocator(const PxEmpty& v) : BaseAllocator(v)	{}

		InlineAllocator(const BaseAllocator& alloc = BaseAllocator())
			: BaseAllocator(alloc), mBufferUsed(false)
		{}

		void* allocate(PxU32 size, const char* filename, int line)
		{
			if(!mBufferUsed && size<=N) 
			{
				mBufferUsed = true;
				return mBuffer;
			}
			return BaseAllocator::allocate(size, filename, line);
		}

		void deallocate(void* ptr)
		{
			if(ptr == mBuffer)
				mBufferUsed = false;
			else
				BaseAllocator::deallocate(ptr);
		}

		PX_FORCE_INLINE	PxU8*	getInlineBuffer()		{ return mBuffer;	  }
		PX_FORCE_INLINE	bool	isBufferUsed()	const	{ return mBufferUsed; }

	protected:
		PxU8 mBuffer[N];
		bool mBufferUsed;
	};
} // namespace shdfnd3
} // namespace physx

#endif

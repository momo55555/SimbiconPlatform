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


#include "PsTempAllocator.h"

#include "PsArray.h"
#include "PsMutex.h"
#include "PsAtomic.h"
#include "PxMath.h"
#include "PsIntrinsics.h"
#include "PsFoundation.h"

#pragma warning( disable : 4706 ) // assignment within conditional expression

namespace physx
{
namespace shdfnd3
{
	namespace 
	{
		typedef TempAllocatorChunk Chunk;
		typedef Array<Chunk*, Allocator> AllocFreeTable;

		PX_INLINE AllocFreeTable& getFreeTable() { return getFoundation().getTempAllocFreeTable(); }
		PX_INLINE Mutex& getMutex() { return getFoundation().getTempAllocMutex();	}

		const PxU32 sMinIndex = 8; // 256B min
		const PxU32 sMaxIndex = 17; // 128kB max
	}


	void* TempAllocator::allocate(size_t size, const char* filename, int line)
	{
		if(!size)
			return 0;

		PxU32 index = PxMax(highestSetBit(PxU32(size) + sizeof(Chunk) - 1), sMinIndex);

		Chunk* ret = 0;
		if(index < sMaxIndex)
		{
			Mutex::ScopedLock lock(getMutex());

			// find chunk up to 16x bigger than necessary
			Chunk **it=getFreeTable().begin() + index - sMinIndex;
			Chunk **end=PxMin(it+3, getFreeTable().end());
			while(it<end && !(ret = *it))
				++it;

			if(ret)
				// pop top off freelist
				*it = ret->mNext, index = PxU32(it - getFreeTable().begin() + sMinIndex);
			else
				// create new chunk
				ret = (Chunk*)Allocator::allocate(2 << index, filename, line);

		} else {
			// too big for temp allocation, forward to base allocator
			ret = (Chunk*)Allocator::allocate(size + sizeof(Chunk), filename, line);
		}


		ret->mIndex = index; 
		return ret + 1;
	}

	void TempAllocator::deallocate(void* ptr)
	{
		if(!ptr)
			return;

		Chunk* chunk = reinterpret_cast<Chunk*>(ptr) - 1;
		PxU32 index = chunk->mIndex;

		if(index >= sMaxIndex)
			return Allocator::deallocate(chunk);

		Mutex::ScopedLock lock(getMutex());

		index -= sMinIndex;
		if(getFreeTable().size() <= index)
			getFreeTable().resize(index+1);

		chunk->mNext = getFreeTable()[index];
		getFreeTable()[index] = chunk;
	}

} // namespace shdfnd3
} // namespace physx

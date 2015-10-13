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


#ifndef PX_PHYSICS_COMMON_FLUSHPOOL
#define PX_PHYSICS_COMMON_FLUSHPOOL

#include "Px.h"
#include "PsUserAllocated.h"
#include "CmPhysXCommon.h"
#include "PsMutex.h"
#include "PsArray.h"

/*
Pool used to allocate variable sized tasks. It's intended to be cleared after a short period (time step).
*/

namespace physx
{
namespace Cm
{

	class FlushPool
	{
	public:
		FlushPool(PxU32 chunkSize) : mChunkIndex(0), mOffset(0), mChunkSize(chunkSize)
		{
			mChunks.pushBack(static_cast<PxU8*>(PX_ALLOC(mChunkSize)));
		}

		~FlushPool()
		{
			for (PxU32 i = 0; i < mChunks.size(); ++i)
				PX_FREE(mChunks[i]);
		}

		void* allocate(PxU32 size)
		{
			PX_ASSERT(size <= mChunkSize && !mChunks.empty());
			Ps::Mutex::ScopedLock lock(mMutex);
			if (mOffset + size > mChunkSize)
			{
				mChunkIndex++;
				mOffset = 0;
				if (mChunkIndex >= mChunks.size())
					mChunks.pushBack(static_cast<PxU8*>(PX_ALLOC(mChunkSize)));
			}

			void* ptr = mChunks[mChunkIndex] + mOffset;
			mOffset += size;
			return ptr;
		}

		void clear()
		{
			Ps::Mutex::ScopedLock lock(mMutex);
			//release memory not used previously
			PxU32 targetSize = mChunkIndex+2;
			while (mChunks.size() > targetSize)
				PX_FREE(mChunks.popBack());

			mChunkIndex = 0;
			mOffset = 0;
		}

	private:
		Ps::Mutex mMutex;
		Ps::Array<PxU8*> mChunks;
		PxU32 mChunkIndex;
		PxU32 mOffset;
		PxU32 mChunkSize;
	};

	
} // namespace Cm

}

#endif

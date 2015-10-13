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

#ifndef PX_SPU_TASK_H
#define PX_SPU_TASK_H

#ifdef PX_PS3

#include "PxTask.h"
#include "PxSpuDispatcher.h"
#include "PsAtomic.h"

namespace physx
{
	namespace pxtask
	{

		class SpuTask : public LightCpuTask
		{
		public:

			static const uint32_t kMaxSpus = 6;
			static const uint32_t kArgsPerSpu = 4;

			// Each instance of an SpuTask may create multiple SPURS tasks (or 
			// the appropriate workload type as implemented by the SpuDispatcher)  
			// by specifying the number of SPUs to use and arguments for each

			SpuTask(const void* elfStart, uint32_t numSpus=1, const uint32_t* args=NULL) 
				: mElfStart(elfStart), mNumSpusToRun(numSpus), mNumSpusFinished(0) 
			{
				if (args)
				{
					memcpy(mArgs, args, mNumSpusToRun*kArgsPerSpu*sizeof(uint32_t));
				}		
			}

			virtual ~SpuTask() {}

			PX_INLINE const uint32_t getSpuCount() const
			{
				return mNumSpusToRun; 
			}

			PX_INLINE void setSpuCount(uint32_t numSpusToRun)  
			{ 
				mNumSpusToRun = numSpusToRun; 
			}

			PX_INLINE const uint32_t* getArgs(uint32_t spuIndex) const 
			{ 
				PX_ASSERT(spuIndex < kMaxSpus); 
				return mArgs[spuIndex]; 
			}

			PX_INLINE void setArgs(uint32_t spuIndex, uint32_t arg1, uint32_t arg2, uint32_t arg3, uint32_t arg4)
			{
				PX_ASSERT(spuIndex < kMaxSpus);
				uint32_t* arguments = mArgs[spuIndex];
				arguments[0]=arg1;
				arguments[1]=arg2;
				arguments[2]=arg3;
				arguments[3]=arg4;
			}

			PX_INLINE const void* getElfStart() const
			{
				return mElfStart; 
			}

			PX_INLINE void notifySpuFinish()
			{
				++mNumSpusFinished;

				// if all SPU tasks have finished clean-up and release
				if (mNumSpusFinished == mNumSpusToRun)
				{
					mNumSpusFinished = 0;
					release();
				}
			}

			// modifies LightCpuTask's behavior by submitting to the SpuDispatcher
			virtual void removeReference()
			{
				if( !physx::shdfnd3::atomicDecrement( &mRefCount ) )
				{
					SpuDispatcher* dispatcher = mTm->getSpuDispatcher();
					PX_ASSERT( dispatcher );
					if( dispatcher )
						dispatcher->submitTask( *this );
					else
						release();
				}
			}

			// this will be called by the SpuDispatcher from whichever thread calls
			// submitTask(); the task will be scheduled to SPURS immediately 
			// following this function returning
			virtual void run() {}

		protected:

			const void* mElfStart;
			uint32_t mNumSpusToRun;
			uint32_t mNumSpusFinished;
			uint32_t mArgs[kMaxSpus][kArgsPerSpu];
		};

	} // end pxtask namespace
} // end physx namespace

#endif	// PX_PS3
#endif

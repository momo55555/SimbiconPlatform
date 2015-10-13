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


#ifndef PX_PHYSICS_EXTENSIONS_NP_DEFAULT_CPU_DISPATCHER_H
#define PX_PHYSICS_EXTENSIONS_NP_DEFAULT_CPU_DISPATCHER_H

#include "CmPhysXCommon.h"
#include "PsUserAllocated.h"
#include "PsSync.h"
#include "PsSList.h"
#include "PxDefaultCpuDispatcher.h"
#include "ExtSharedQueueEntryPool.h"


namespace physx
{
	namespace pxtask
	{
		class BaseTask;
	}
}

namespace physx
{
namespace Ext
{
	class CpuWorkerThread;

#pragma warning(push)
#pragma warning(disable:4324)	// Padding was added at the end of a structure because of a __declspec(align) value.
								// Because of the SList member I assume

	class DefaultCpuDispatcher : public PxDefaultCpuDispatcher, public Ps::UserAllocated
	{
		friend class TaskQueueHelper;

	private:
		DefaultCpuDispatcher() : mQueueEntryPool(0) {}
		~DefaultCpuDispatcher();

	public:
		DefaultCpuDispatcher(PxU32 numThreads, PxU32* affinityMasks);

		//---------------------------------------------------------------------------------
		// physx::pxtask::CpuDispatcher implementation
		//---------------------------------------------------------------------------------
		virtual void submitTask(pxtask::BaseTask& task);

		//---------------------------------------------------------------------------------
		// PxDefaultCpuDispatcher implementation
		//---------------------------------------------------------------------------------
		virtual void release();

		//---------------------------------------------------------------------------------
		// DefaultCpuDispatcher
		//---------------------------------------------------------------------------------
		pxtask::BaseTask*		getJob();
		pxtask::BaseTask*		stealJob();
        void					waitForWork() { mWorkReady.wait(); }
        void					resetWakeSignal();

		static PxU32			getAffinityMask(PxU32 affinityMask);


	protected:
				CpuWorkerThread*				mWorkerThreads;
				SharedQueueEntryPool<>			mQueueEntryPool;
				Ps::SList						mJobList;
				Ps::Sync						mWorkReady;
				PxU32							mNumThreads;
				bool							mShuttingDown;
	};

#pragma warning(pop)

} // namespace Ext
}

#endif

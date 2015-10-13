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


#ifndef PX_PHYSICS_SCENEQUERY
#define PX_PHYSICS_SCENEQUERY
/** \addtogroup physics 
@{ */

#include "PsUserAllocated.h"
#include "PxBatchQuery.h"
#include "SqBatchQueryStream.h"
#include "SqPrunable.h"

namespace physx
{

class NpBatchQuery;
class BatchQueryStream;

namespace Sq
{
	class SceneQueryManager;

	class BatchQuery : public Ps::UserAllocated
	{
	public:
														BatchQuery(SceneQueryManager& owner, const PxBatchQueryDesc& d);
														~BatchQuery();

						void							execute();
						void							release();

		PX_FORCE_INLINE	const void*						getFilterShaderData()			const	{ return mDesc.filterShaderData;		}
		PX_FORCE_INLINE	PxU32							getFilterShaderDataSize()		const	{ return mDesc.filterShaderDataSize;	}
		PX_FORCE_INLINE	PxBatchQueryPreFilterShader		getPreFilterShader()			const	{ return mDesc.preFilterShader;			}
		PX_FORCE_INLINE	PxBatchQueryPostFilterShader	getPostFilterShader()			const	{ return mDesc.postFilterShader;		}
	public:
		// internal methods
		PX_FORCE_INLINE	BatchQueryStream&				getBatchQueryStream()			const	{ return mBatchQueryStream;				}
		PX_FORCE_INLINE	const PxBatchQueryDesc&			getDesc()						const	{ return mDesc;							}

		//For spu raycast
#if defined(PX_PS3)
						bool							mIsRaycastPS3SupportTaskStarted;	
#endif

	private:
		// stream containing all batched queries
		mutable			BatchQueryStream				mBatchQueryStream;
						SceneQueryManager&				mSceneQueryManager;
						PxBatchQueryDesc				mDesc;

						PxU32							mRaycastHitOffset;
						PxU32							mSweepHitOffset;
						PxU32							mOverlapHitOffset;
	};

} // namespace Sq

}

/** @} */
#endif

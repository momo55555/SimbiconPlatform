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
#ifndef OPC_VOLUMECOLLIDER_H
#define OPC_VOLUMECOLLIDER_H

#include "./Ice/IceContainer.h"
#include "OPC_Collider.h"

namespace physx
{
namespace Ice
{
	struct BaseModelData;

	class VolumeCollider : public Collider
	{
		public:
											VolumeCollider(): mNbVolumeBVTests(0)	{}
											~VolumeCollider()						{}

								PxU32		mNbVolumeBVTests;		//!< Number of Volume-BV tests

		PX_FORCE_INLINE			void		InitQuery()
											{
												// Reset stats & contact status
												mNbVolumeBVTests	= 0;
											}
	};

	struct VolumeColliderTrigCallback
	{
		// return false for early out
		virtual bool processResults(PxU32 maxTrigs, const PxVec3* trigVertexTriples, const PxU32* trigIndices) = 0;
	};

	struct VolumeColliderContainerCallback : VolumeColliderTrigCallback
	{
		Ice::Container& container;
		VolumeColliderContainerCallback(Ice::Container& container) : container(container)
		{
			container.Reset();
		}
		virtual ~VolumeColliderContainerCallback() {}

		virtual bool processResults(PxU32 count, const PxVec3*, const PxU32* buf)
		{
			container.Add(buf, count);
			return true;
		}
	};

	struct VolumeColliderTrigAccumulationCallback : VolumeColliderTrigCallback
	{
		// PT: those guys are more efficient than Ps::Array (no LHS on the size member)
		Ice::Container	vertTriples;
		Ice::Container	indices;

		PX_FORCE_INLINE	PxU32			getNbIndices()	const	{ return indices.GetNbEntries();					}
		PX_FORCE_INLINE	const PxVec3*	getAccVerts()	const	{ return (const PxVec3*)vertTriples.GetEntries();	}
		PX_FORCE_INLINE	const PxU32*	getAccIndices()	const	{ return (const PxU32*)indices.GetEntries();		}

		virtual ~VolumeColliderTrigAccumulationCallback() {}

		virtual bool processResults(PxU32 aCount, const PxVec3* aVertTriples, const PxU32* aIndices)
		{
			PxVec3* reservedVerts = (PxVec3*)vertTriples.Reserve(aCount*3*sizeof(PxVec3)/sizeof(PxU32));
			memCopy(reservedVerts, aVertTriples, aCount*3*sizeof(PxVec3));
			PxU32* reservedIndices = (PxU32*)indices.Reserve(aCount);
			memCopy(reservedIndices, aIndices, aCount*sizeof(PxU32));
			return true;
		}
	};

	struct VolumeColliderAnyHitCallback : VolumeColliderTrigCallback
	{
		bool anyHits;
		VolumeColliderAnyHitCallback() : anyHits(false)	{}
		virtual ~VolumeColliderAnyHitCallback()			{}

		virtual bool processResults(PxU32 count, const PxVec3*, const PxU32* buf)
		{
			PX_ASSERT(count > 0);
			anyHits = true;
			return false;
		}
	};

} // namespace Ice

}

#endif // OPC_VOLUMECOLLIDER_H

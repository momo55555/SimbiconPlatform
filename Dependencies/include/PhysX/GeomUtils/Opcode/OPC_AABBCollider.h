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
#ifndef OPC_AABBCOLLIDER_H
#define OPC_AABBCOLLIDER_H

#include "OPC_VolumeCollider.h"
#include "OPC_ModelData.h"

namespace physx
{
namespace Ice
{
	class AABBCollider : public VolumeCollider
	{
		public:
		PX_FORCE_INLINE						AABBCollider()	{}
		PX_FORCE_INLINE						~AABBCollider()	{}

							CollisionAABB	mBox;			//!< Query box in (center, extents) form
							PxVec3			mMin;			//!< Query box minimum point
							PxVec3			mMax;			//!< Query box maximum point
		// Leaf description
							PxVec3			mLeafVerts[3];	//!< Triangle vertices
			// Overlap tests
		PX_INLINE			Ps::IntBool		AABBAABBOverlap(const PxVec3& b, const PxVec3& Pb);
		PX_INLINE			Ps::IntBool		AABBAABBOverlapMinMax(const PxVec3& bbMin, const PxVec3& bbMax);
#ifdef OPC_SUPPORT_VMX128
		PX_INLINE			Ps::IntBool		AABBAABBOverlap(const Cm::PxSimd::Vector4& extents, const Cm::PxSimd::Vector4& center);
		PX_INLINE			Ps::IntBool		AABBAABBOverlapMinMax(const Cm::PxSimd::Vector4& bbMin, const Cm::PxSimd::Vector4& bbMax);
		PX_INLINE			Ps::IntBool		TriBoxOverlap(const Cm::PxSimd::Vector4& leafVerts0,const Cm::PxSimd::Vector4& leafVerts1,const Cm::PxSimd::Vector4& leafVerts2,
														const Cm::PxSimd::Vector4& center,const Cm::PxSimd::Vector4& extents);

#endif
		PX_INLINE			Ps::IntBool		TriBoxOverlap(const PxVec3& v0, const PxVec3& v1, const PxVec3& v2);
	};

	class HybridAABBCollider : public AABBCollider
	{
		public:
		void Collide(
			const CollisionAABB& box,
			const HybridModelData& model, bool primTests,
			VolumeColliderTrigCallback* resultsCallback);
	};

} // namespace Ice

}

#include "OPC_AABBColliderOverlap.h"

#endif // OPC_AABBCOLLIDER_H

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


#ifndef PX_PHYSICS_SQFILTERING
#define PX_PHYSICS_SQFILTERING
/** \addtogroup physics 
@{ */

#include "PxFiltering.h"
#include "NpShape.h"
#include "SqUtilities.h"
#include "NpRigidDynamic.h"

namespace physx
{
namespace Sq
{

	PX_INLINE void setFilterObjectAttributeType(PxFilterObjectAttributes& attr, PxFilterObjectType::Enum type)
	{
		PX_ASSERT((attr & (PxFilterObjectType::eMAX_TYPE_COUNT-1)) == 0);
		attr |= type;
	}

	PX_INLINE PxFilterObjectAttributes getFilterAttributesFast(const NpShape& shape)
	{
		PxFilterObjectAttributes attr = 0;

		if(shape.getFlagsInternal() & PxShapeFlag::eTRIGGER_SHAPE)
			attr |= PxFilterObjectFlags::eTRIGGER;

		PxRigidActor& shapeActor = shape.getActorFast();

		const PxType serialType = shapeActor.getSerialType();
		if(serialType==PxSerialType::eRIGID_DYNAMIC)
		{
			PxRigidDynamic* rb = static_cast<PxRigidDynamic*>(&shapeActor);
			if (rb->getRigidDynamicFlags() & PxRigidDynamicFlag::eKINEMATIC)	// ### VIRTUAL CALL
				attr |= PxFilterObjectFlags::eKINEMATIC;

			setFilterObjectAttributeType(attr, PxFilterObjectType::eRIGID_DYNAMIC);
		}
		else if(serialType == PxSerialType::eARTICULATION_LINK)
		{
			setFilterObjectAttributeType(attr, PxFilterObjectType::eARTICULATION);
		}
		else
		{
			setFilterObjectAttributeType(attr, PxFilterObjectType::eRIGID_STATIC);
		}

		return attr;
	}

	// PT: specialized version when the actor type is already known
	PX_FORCE_INLINE PxFilterObjectAttributes getFilterAttributesFast(const NpShape& shape, const NpRigidStatic& npRigidStatic)
	{
		PxFilterObjectAttributes attr;
		if(shape.getFlagsInternal() & PxShapeFlag::eTRIGGER_SHAPE)
			attr = PxFilterObjectFlags::eTRIGGER;
		else
			attr = 0;

		setFilterObjectAttributeType(attr, PxFilterObjectType::eRIGID_STATIC);

		return attr;
	}

	// PT: specialized version when the actor type is already known
	PX_FORCE_INLINE PxFilterObjectAttributes getFilterAttributesFast(const NpShape& shape, const NpRigidDynamic& npRigidDynamic)
	{
		PxFilterObjectAttributes attr;
		if(shape.getFlagsInternal() & PxShapeFlag::eTRIGGER_SHAPE)
			attr = PxFilterObjectFlags::eTRIGGER;
		else
			attr = 0;

		if (npRigidDynamic.getRigidDynamicFlagsFast() & PxRigidDynamicFlag::eKINEMATIC)
			attr |= PxFilterObjectFlags::eKINEMATIC;

		setFilterObjectAttributeType(attr, PxFilterObjectType::eRIGID_DYNAMIC);

		return attr;
	}

	PX_INLINE bool filterShape(SceneQueryShapeData& shape, PxSimulationFilterShader shader, const PxFilterData* queryData, const void* constBlock, PxU32 constBlockSize, PxClientID queryClient, bool queryClientTakesForeignShapes)
	{
		//multiclient filtering:
		if (
			 (shape.actorClientID != queryClient)
			 && (
				 !queryClientTakesForeignShapes ||
				 !(shape.actorClientBehaviorBits & PxActorClientBehaviorBit::eREPORT_TO_FOREIGN_CLIENTS_SCENE_QUERY)
				 )
			 )
			 return true;

		if(queryData && shader)
		{
			PxFilterObjectAttributes queryAttr = 0;
			PxFilterObjectAttributes shapeAttr = shape.attr;
			PxPairFlags pairFlags;
			// ### PT: we should pass the shape pointer to the callback and NOT compute "shapeAttr" ourselves
			const PxFilterFlags filterFlags = shader(queryAttr, *queryData, shapeAttr, shape.queryFilterData, pairFlags, constBlock, constBlockSize);
			return (filterFlags & (PxFilterFlag::eKILL | PxFilterFlag::eSUPPRESS));
		}
		return false;
	}

	static PX_FORCE_INLINE bool applyBatchedPreFilterPreTest(PxBatchQueryPreFilterShader preFilter, const PxFilterData& filterData, 
															const void* constBlock, PxU32 constBlockSize,
															PxU32 inFilterFlags, const Sq::SceneQueryShapeData& shapeData,
															PxSceneQueryFilterFlags& outFilterFlags, PxSceneQueryHitType::Enum& hitType)
	{
		outFilterFlags = PxSceneQueryFilterFlags(inFilterFlags);

		//const PxFilterData& rayFd = filterData;
		//if (rayFd.word0 != 0 || rayFd.word1 != 0 || rayFd.word2 != 0 || rayFd.word3 != 0)
		//{
		//	const PxFilterData& objFd = shapeData.queryFilterData;
		//	PxU32 keep = (rayFd.word0 & objFd.word0) | (rayFd.word1 & objFd.word1) | (rayFd.word2 & objFd.word2) | (rayFd.word3 & objFd.word3);
		//	if (!keep)
		//		return true;
		//}

		if (preFilter && (inFilterFlags & PxSceneQueryFilterFlag::ePREFILTER))
		{
			hitType = preFilter(filterData, shapeData.queryFilterData, constBlock, constBlockSize, outFilterFlags);

			// Adopt changes to eBACKFACE and eMESH_MULTIPLE
			outFilterFlags = (PxSceneQueryFilterFlags(inFilterFlags) & ~(PxSceneQueryFilterFlag::eBACKFACE | PxSceneQueryFilterFlag::eMESH_MULTIPLE)) | (outFilterFlags & (PxSceneQueryFilterFlag::eBACKFACE | PxSceneQueryFilterFlag::eMESH_MULTIPLE));
		}
		return false;
	}

	static PX_FORCE_INLINE PxSceneQueryHitType::Enum  applyBatchedPostFilterTest(PxBatchQueryPostFilterShader postFilter, const PxFilterData& filterData, 
															const void* constBlock, PxU32 constBlockSize,
															PxSceneQueryFilterFlags filterFlags, const Sq::SceneQueryShapeData& shapeData,															
															const PxSceneQueryHit& hit, PxSceneQueryHitType::Enum hitType)
	{
		if (postFilter && (filterFlags & PxSceneQueryFilterFlag::ePOSTFILTER))
		{
			hitType = postFilter(filterData, shapeData.queryFilterData, constBlock, constBlockSize, hit);
		}
		return hitType;	
	}


	//
	// filterFlag    Initialized to the PxSceneQueryFilter::filterFlag and potentially overwritten by preFilter() callback
	// hitType       Not initialized, set if the preFilter() callback runs
	//
	static PX_FORCE_INLINE bool applyNonBatchedFilterPreTest(PxSceneQueryFilterCallback* filterCall, const PxFilterData& queryFd, 
															PxU32 inFilterFlags, const Sq::SceneQueryShapeData& shapeData,
															PxSceneQueryFilterFlags& outFilterFlags, PxSceneQueryHitType::Enum& hitType)
	{
		// if the filterData field is non-zero, and the bitwise-AND value of filterData AND the shape's
		// queryFilterData is zero, the shape is skipped.

		outFilterFlags = PxSceneQueryFilterFlags(inFilterFlags);

		if (queryFd.word0 != 0 || queryFd.word1 != 0 || queryFd.word2 != 0 || queryFd.word3 != 0)
		{
			const PxFilterData& objFd = shapeData.queryFilterData;
			PxU32 keep = (queryFd.word0 & objFd.word0) | (queryFd.word1 & objFd.word1) | (queryFd.word2 & objFd.word2) | (queryFd.word3 & objFd.word3);
			if (!keep)
				return true;
		}

		if (filterCall && (inFilterFlags & PxSceneQueryFilterFlag::ePREFILTER))
		{
			hitType = filterCall->preFilter(queryFd, shapeData.shape, outFilterFlags);

			// Adopt changes to eBACKFACE and eMESH_MULTIPLE
			outFilterFlags = (PxSceneQueryFilterFlags(inFilterFlags) & ~(PxSceneQueryFilterFlag::eBACKFACE | PxSceneQueryFilterFlag::eMESH_MULTIPLE)) | (outFilterFlags & (PxSceneQueryFilterFlag::eBACKFACE | PxSceneQueryFilterFlag::eMESH_MULTIPLE));
		}

		return false;
	}


	static PX_FORCE_INLINE PxSceneQueryHitType::Enum applyNonBatchedFilterPostTest(PxSceneQueryFilterCallback* filterCall, PxSceneQueryFilterFlags filterFlags,
																					const PxFilterData& queryFd, const PxSceneQueryHit& hit, PxSceneQueryHitType::Enum hitType
																					)
	{
		if (filterCall && (filterFlags & PxSceneQueryFilterFlag::ePOSTFILTER))
			return filterCall->postFilter(queryFd, hit);

		return hitType;
	}

	PX_FORCE_INLINE bool applyClientFilter(PxClientID queryClient, bool passForeignShapes, const SceneQueryShapeData& shape)
	{
		bool reportToForeignClients = (shape.actorClientBehaviorBits & PxActorClientBehaviorBit::eREPORT_TO_FOREIGN_CLIENTS_SCENE_QUERY) != 0;
		return (queryClient != shape.actorClientID) && !(passForeignShapes && reportToForeignClients);
	}
} // namespace Sq

}

/** @} */
#endif

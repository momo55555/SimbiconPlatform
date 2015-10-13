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
#include "PsIntrinsics.h"
#include "SqPruningPool.h"

using namespace physx;
using namespace Sq;

Prunable::Prunable() :
	mPrevious	(NULL),
	mNext		(NULL),
	mOwner		(NULL),

	mPRNFlags	(0),
	mHandle		(INVALID_PRUNING_HANDLE)
{
}

Prunable::~Prunable()
{
	// Auto remove ? => need access to pruner
	PX_ASSERT(mHandle==INVALID_PRUNING_HANDLE && "WARNING: deleting an object still in use! Call PruningEngine::RemoveObject() before!");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "SqUtilities.h"
void Prunable::GetWorldAABB(PxBounds3& box) const
{
	SceneQueryShapeData* s = const_cast<SceneQueryShapeData*>(static_cast<const SceneQueryShapeData*>(this));

	// PT: it would make sense to use NpShape::getWorldBoundsFast() here but I have no idea if it actually returns the same stuff
	// (doesn't access the geometry the same way, doesn't compute the global pose the same way either). So until we know more...

	const PxTransform globalPose = Sq::getGlobalPose(*s->shape);

	// SA: picking is broken without the next line, need to revisit SceneQueryShapeData
	s->sqGlobalPose = globalPose;

	PxVec3 c,e;
	s->shapeGeometryEA->computeBounds(globalPose, NULL, c, e);
	box = PxBounds3(c-e, c+e);
}

void Prunable::ComputeWorldAABB_Special(PxBounds3& box) const
{
	SceneQueryShapeData* s = const_cast<SceneQueryShapeData*>(static_cast<const SceneQueryShapeData*>(this));
	PxVec3 c,e;
	s->shapeGeometryEA->computeBounds(s->sqGlobalPose, NULL, c, e);
	box = PxBounds3(c-e, c+e);
}

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

#include "GuConvexUtilsInternal.h"
#include "PxBounds3.h"
#include "CmScaling.h"
#include "GuBoxConversion.h"

using namespace physx;

void Gu::computeHullOBB(Gu::Box& hullOBB, const PxBounds3& hullAABB, float offset, const PxTransform& transform0, const Cm::Matrix34& world0, const Cm::Matrix34& world1, const Cm::FastVertex2ShapeScaling& meshScaling, bool idtScaleMesh)
{
	//query OBB in world space
	PxVec3 center = transform0.transform((hullAABB.minimum + hullAABB.maximum) * 0.5f);
	PxVec3 extents = (hullAABB.maximum - hullAABB.minimum) * 0.5f;
	extents += PxVec3(offset);	//fatten query region for dist based.
	//orientation is world0.M

	//transform to mesh shape space:
	center = world1.transformTranspose(center);
	PxMat33 obbBasis = PxMat33(world1.base0, world1.base1, world1.base2).getTranspose() * PxMat33(world0.base0, world0.base1, world0.base2);
	//OBB orientation is world0

	if(!idtScaleMesh)
		meshScaling.transformQueryBounds(center, extents, obbBasis);

	// Setup an ICE OBB of the convex hull in vertex space
	hullOBB.center	= center;
	hullOBB.extents	= extents;
	hullOBB.rot		= obbBasis;
}

void Gu::computeVertexSpaceOBB(Gu::Box& dst, const Gu::Box& src, const PxTransform& meshPose, const PxMeshScale& meshScale)
{
	// PT: somehow extracted from the overlap tests. This stuff needs a huge refactoring.
	const PxMat34Legacy absPose(meshPose);
	PxMat34Legacy vertexToWorldSkew(absPose.M * meshScale.toMat33(), absPose.t);

	PxMat34Legacy worldToVertexSkew(false);
	vertexToWorldSkew.getInverse(worldToVertexSkew);

	//make vertex space OBB
	dst = transform(worldToVertexSkew, src);
}
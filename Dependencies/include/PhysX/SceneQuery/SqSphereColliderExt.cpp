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

#include "SqSphereColliderExt.h"

using namespace physx;
using namespace Sq;
using namespace Ice;

#include "GuSphere.h"

void SphereColliderExt::initQuery(Prunable** objectsBase, ReportPrunablesCallback callback, void* userData, const Gu::Sphere& sphere)
{
	VolumeCollider::InitQuery();

	// Compute sphere in model space:
	// - Precompute R^2
	mRadius2 = sphere.radius * sphere.radius;
	mRadius = sphere.radius;
	// - Compute center position
	mCenter = sphere.center;

	// Setup destination pointer
	mObjectsBase	= objectsBase;
	mCallback		= callback;
	mUserData		= userData;
	PX_ASSERT(callback);
}

bool SphereColliderExt::collideExt(Prunable** objectsBase, ReportPrunablesCallback callback, void* userData, const Gu::Sphere& sphere, const AABBTree* tree)
{
	if(!tree)
		return false;

	initQuery(objectsBase, callback, userData, sphere);

	// Perform collision query
	_collideExt(tree->GetNodes());

	return true;
}

void SphereColliderExt::_collideExt(const AABBTreeNode* PX_RESTRICT node)
{
	if(mFlags & OPC_ABORT_QUERY)
		return;

	// Perform Sphere-AABB overlap test
	if(!SphereAABBOverlap(node->GetAABB().getCenter(), node->GetAABB().getExtents()))
		return;

	if(node->IsLeaf())
	{
		mFlags |= OPC_CONTACT;
		if(!reportTouchedObjects(node, mObjectsBase, mCallback, mUserData))
		{
			mFlags |= OPC_ABORT_QUERY;
			return;
		}
	}
	else
	{
		_collideExt(node->GetPos());
		_collideExt(node->GetNeg());
	}
}

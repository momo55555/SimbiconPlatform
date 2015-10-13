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

#include "SqRayColliderExt.h"
#include "OPC_AABBTree.h"
#include "StabCodes.h"

using namespace physx;
using namespace Sq;
using namespace Ice;

bool RayColliderExt::collideExt(const PxVec3& orig, const PxVec3& dir, const AABBTree* tree, BoxHitCallback callback, void* user_data)
{
	if(!tree)
		return false;

	// Init collision query
	// Basically this is only called to initialize precomputed data
	if(InitQuery(orig, dir))
		return true;

	mExtCallback	= callback;
	mExtUserData	= user_data;

	// Perform stabbing query
	if(mMaxDist!=PX_MAX_F32)
		_segmentStabExt(tree->GetNodes());
	else
		_rayStabExt(tree->GetNodes());

	return true;
}

bool RayColliderExt::closestHitExt(const PxVec3& orig, const PxVec3& dir, const AABBTree* tree, BoxHitCallback callback, void* user_data)
{
	if(!tree)
		return false;

	// Init collision query
	// Basically this is only called to initialize precomputed data
	if(InitQuery(orig, dir))
		return true;

	mExtCallback	= callback;
	mExtUserData	= user_data;

	// Perform stabbing query
	if(mMaxDist!=PX_MAX_F32)
		_segmentClosestStabExt(tree->GetNodes());

	return true;
}

void RayColliderExt::_segmentStabExt(const AABBTreeNode* PX_RESTRICT node)
{
	if(mFlags & OPC_CONTACT)
		return;

	// Test the box against the segment
	const PxVec3 center = node->GetAABB().getCenter();
	const PxVec3 extents = node->GetAABB().getExtents();
	if(!SegmentAABBOverlap(center, extents))	return;

	if(node->IsLeaf())
	{
		PxU32 Status = (mExtCallback)(node->GetPrimitives(), node->GetNbPrimitives(), mMaxDist, mExtUserData);
		if(Status & STAB_STOP)
		{
			mFlags |= OPC_CONTACT;
			return;
		}
	}
	else
	{
		_segmentStabExt(node->GetPos());
		_segmentStabExt(node->GetNeg());
	}
}

void RayColliderExt::_segmentClosestStabExt(const AABBTreeNode* PX_RESTRICT node)
{
	if(mFlags & OPC_CONTACT)
		return;

	// Test the box against the segment
	const PxVec3 center = node->GetAABB().getCenter();
	const PxVec3 extents = node->GetAABB().getExtents();
	if(!SegmentAABBOverlap(center, extents))	return;

	if(node->IsLeaf())
	{
		// ### missing filtering => done outside
		PxU32 Status = (mExtCallback)(node->GetPrimitives(), node->GetNbPrimitives(), mMaxDist, mExtUserData);
		if(Status & STAB_STOP)
		{
			mFlags |= OPC_CONTACT;
			return;
		}
		if(Status & STAB_UPDATE_MAX_DIST)
		{
			SetupSegment();
		}
	}
	else
	{
//		_SegmentClosestStab(node->GetPos(), callback, user_data);
//		_SegmentClosestStab(node->GetNeg(), callback, user_data);

		// ### use c/e for vanilla trees as well
		const PxVec3 PCenter = node->GetPos()->GetAABB().getCenter();
		const PxVec3 NCenter =	node->GetNeg()->GetAABB().getCenter();
		const float val = (PCenter - NCenter).dot(mDir);

		if(val > 0.0f)	// Do smaller rank first
		{
			_segmentClosestStabExt(node->GetNeg());
			_segmentClosestStabExt(node->GetPos());
		}
		else
		{
			_segmentClosestStabExt(node->GetPos());
			_segmentClosestStabExt(node->GetNeg());
		}
	}
}

#ifndef OPC_SUPPORT_VMX128

void RayColliderExt::_rayStabExt(const AABBTreeNode* PX_RESTRICT node)
{
	if(mFlags & OPC_CONTACT)
		return;

	// Test the box against the ray
	const PxVec3 center = node->GetAABB().getCenter();
	const PxVec3 extents = node->GetAABB().getExtents();
	if(!RayAABBOverlap(center, extents))	return;

	if(node->IsLeaf())
	{
		PxU32 Status = (mExtCallback)(node->GetPrimitives(), node->GetNbPrimitives(), mMaxDist, mExtUserData);
		if(Status & STAB_STOP)
		{
			mFlags |= OPC_CONTACT;
			return;
		}
	}
	else
	{
		_rayStabExt(node->GetPos());
		_rayStabExt(node->GetNeg());
	}
}

#else

void RayColliderExt::_rayStabExt(const AABBTreeNode* PX_RESTRICT node)
{
	if(mFlags & OPC_CONTACT)
		return;

	// Test the box against the ray
	/*Point Center, Extents;
	node->GetAABB().GetCenter(Center);
	node->GetAABB().GetExtents(Extents);*/

	Cm::PxSimd::Vector4 boxMin = Cm::PxSimd::load(node->GetAABB().minimum);
	Cm::PxSimd::Vector4 boxMax = Cm::PxSimd::load(node->GetAABB().maximum);

	//we could probably divide by 2 faster...
	Cm::PxSimd::Vector4 Center = Cm::PxSimd::multiply(Cm::PxSimd::add(boxMin, boxMax), Cm::PxSimd::half());
	Cm::PxSimd::Vector4 Extents = Cm::PxSimd::multiply(Cm::PxSimd::subtract(boxMax, boxMin), Cm::PxSimd::half());

	if(!RayAABBOverlap(Center, Extents))	return;

	if(node->IsLeaf())
	{
		PxU32 Status = (mExtCallback)(node->GetPrimitives(), node->GetNbPrimitives(), mMaxDist, mExtUserData);
		if(Status & STAB_STOP)
		{
			mFlags |= OPC_CONTACT;
			return;
		}
	}
	else
	{
		_rayStabExt(node->GetPos());
		_rayStabExt(node->GetNeg());
	}
}

#endif

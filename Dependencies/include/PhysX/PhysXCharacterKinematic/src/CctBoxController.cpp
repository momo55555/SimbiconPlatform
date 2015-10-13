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

#include "PxController.h"
#include "CctBoxController.h"
#include "PxBoxGeometry.h"
#include "PxRigidDynamic.h"
#include "PxShape.h"

using namespace physx;
using namespace Cct;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

BoxController::BoxController(const PxControllerDesc& desc, PxPhysics& sdk, PxScene* s) : Controller(desc, s)
{
	mType = PxControllerShapeType::eBOX;

	const PxBoxControllerDesc& bc = (const PxBoxControllerDesc&)desc;

	mExtents	= bc.extents;

	// Create kinematic actor under the hood
	if(1)
	{
		PxBoxGeometry boxGeom;
		boxGeom.halfExtents		= mExtents*0.8f;
//		boxGeom.halfExtents		= mExtents*0.9f;
//		boxGeom.halfExtents		= mExtents*1.1f;
//		boxGeom.halfExtents		= mExtents;
//		boxGeom.halfExtents.y	+= stepOffset*1.1f;

		// PT: we don't disable raycasting or CD because:
		// - raycasting is needed for visibility queries (the SDK otherwise doesn't know about the CCTS)
		// - collision is needed because the only reason we create actors there is to handle collisions with dynamic shapes
		// So it's actually wrong to disable any of those, and we'll have to filter out the CCT shapes using the 'CCTS' mark.
		const PxReal density = 10.0f;	// ### expose this ?

		// LOSS OF ACCURACY
		PxTransform globalPose;
		globalPose.p.x  = (float)mPosition.x;
		globalPose.p.y  = (float)mPosition.y;
		globalPose.p.z  = (float)mPosition.z;
		globalPose.q = PxQuat::createIdentity();

		createProxyActor(sdk, globalPose, boxGeom, *desc.material, density);
	}
}

BoxController::~BoxController()
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void BoxController::reportSceneChanged()
{
	mCctModule.voidTestCache();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool BoxController::getWorldBox(PxExtendedBounds3& box) const
{
	setCenterExtents(box, mPosition, mExtents);
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const PxVec3& BoxController::getExtents() const
{
	return mExtents;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool BoxController::setExtents(const PxVec3& e)
{
	mExtents	= e;
	if(mKineActor)
	{
		PxShape* shape = NULL;
		mKineActor->getShapes(&shape, 1);

		PX_ASSERT(shape->getGeometryType() == PxGeometryType::eBOX);
		PxBoxGeometry bg;
		shape->getBoxGeometry(bg);

		PxReal oldHeight = bg.halfExtents[mUserParams.mUpDirection];

		bg.halfExtents = e * 0.8f;
		shape->setGeometry(bg);

		PxExtendedVec3 pos = getPosition();
		pos[mUserParams.mUpDirection] += (bg.halfExtents[mUserParams.mUpDirection]-oldHeight)*(1/.8f);
		setPosition(pos);
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


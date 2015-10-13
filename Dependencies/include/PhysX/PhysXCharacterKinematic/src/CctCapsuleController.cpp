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
#include "CctCapsuleController.h"
#include "PxCapsuleGeometry.h"
#include "PxRigidDynamic.h"
#include "PxShape.h"

using namespace physx;
using namespace Cct;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CapsuleController::CapsuleController(const PxControllerDesc& desc, PxPhysics& sdk, PxScene* s) : Controller(desc, s)
{
	mType = PxControllerShapeType::eCAPSULE;

	const PxCapsuleControllerDesc& cc = (const PxCapsuleControllerDesc&)desc;

	mRadius			= cc.radius;
	mHeight			= cc.height;
	mClimbingMode	= cc.climbingMode;

	// Create kinematic actor under the hood
	if(1)
	{
		PxCapsuleGeometry capsGeom;
		capsGeom.radius		= mRadius * 0.8f;
		capsGeom.halfHeight	= 0.5f * mHeight * 0.8f;

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
		if(desc.upDirection==PxCCTUpAxis::eY)
			globalPose.q = PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f));
		else if(desc.upDirection==PxCCTUpAxis::eZ)
			globalPose.q = PxQuat(-PxHalfPi, PxVec3(0.0f, 1.0f, 0.0f));
		else
			globalPose.q = PxQuat::createIdentity();

		createProxyActor(sdk, globalPose, capsGeom, *desc.material, density);
	}
}

CapsuleController::~CapsuleController()
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CapsuleController::reportSceneChanged()
{
	mCctModule.voidTestCache();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool CapsuleController::getWorldBox(PxExtendedBounds3& box) const
{
	setCenterExtents(box, mPosition, PxVec3(mRadius, mRadius+mHeight*0.5f, mRadius));
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool CapsuleController::setRadius(PxF32 r)
{
	mRadius = r;
	if(mKineActor)
	{
		PxShape* shape = NULL;
		mKineActor->getShapes(&shape, 1);

		PX_ASSERT(shape->getGeometryType() == PxGeometryType::eCAPSULE);
		PxCapsuleGeometry cg;
		shape->getCapsuleGeometry(cg);
		cg.radius = r * 0.8f;
		shape->setGeometry(cg);
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool CapsuleController::setHeight(PxF32 h)
{
	mHeight = h;
	if(mKineActor)
	{
		PxShape* shape = NULL;
		mKineActor->getShapes(&shape, 1);

		PX_ASSERT(shape->getGeometryType() == PxGeometryType::eCAPSULE);
		PxCapsuleGeometry cg;
		shape->getCapsuleGeometry(cg);

		PxReal oldHeight = cg.halfHeight;

		cg.halfHeight = 0.5f * h * 0.8f;
		shape->setGeometry(cg);

		PxExtendedVec3 pos = getPosition();
		pos[mUserParams.mUpDirection] += (cg.halfHeight-oldHeight)*(1/.8f);
		setPosition(pos);
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxCapsuleClimbingMode::Enum CapsuleController::getClimbingMode() const
{
	return mClimbingMode;
}

bool CapsuleController::setClimbingMode(PxCapsuleClimbingMode::Enum mode)
{
	if(mode>=PxCapsuleClimbingMode::eLAST)
		return false;
	mClimbingMode = mode;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

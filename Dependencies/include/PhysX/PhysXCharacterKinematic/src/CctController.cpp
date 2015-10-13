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
#include "CctController.h"
#include "CctBoxController.h"
#include "CctCharacterControllerManager.h"
#include "PxScene.h"
#include "PxRigidDynamic.h"
#include "PxShape.h"
#include "PxExtensionsAPI.h"

using namespace physx;
using namespace Cct;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Controller::Controller(const PxControllerDesc& desc, PxScene* s) : mManager(NULL), mScene(s)
{
	mType				= PxControllerShapeType::eFORCE_DWORD;
	mInteractionMode	= desc.interactionMode;

	mUserParams.mUpDirection			= desc.upDirection;
	mUserParams.mSlopeLimit				= desc.slopeLimit;
	mUserParams.mContactOffset			= desc.contactOffset;
	mUserParams.mStepOffset				= desc.stepOffset;
	mUserParams.mInvisibleWallHeight	= desc.invisibleWallHeight;
	mUserParams.mMaxJumpHeight			= desc.maxJumpHeight;
	mUserParams.mHandleSlope			= desc.slopeLimit!=0.0f;

	mCallback			= desc.callback;
	mUserData			= desc.userData;

	mKineActor			= NULL;
	mPosition			= desc.position;
	mFilteredPosition	= desc.position;
	mExposedPosition	= desc.position;
	mMemory				= desc.position[desc.upDirection];
}

Controller::~Controller()
{
	if(mScene && mKineActor)
		mKineActor->release();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Controller::releaseInternal()
{
	mManager->releaseController(*getNxController());
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Controller::setPos(const PxExtendedVec3& pos)
{
	mPosition = mFilteredPosition = mExposedPosition = pos;
	mMemory = pos[mUserParams.mUpDirection];

	// Update kinematic actor
	if(mKineActor)
	{
		PxTransform targetPose = mKineActor->getGlobalPose();
		targetPose.p = PxVec3(PxReal(mPosition.x), PxReal(mPosition.y), PxReal(mPosition.z));  // LOSS OF ACCURACY
		mKineActor->moveKinematic(targetPose);	
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Controller::createProxyActor(PxPhysics& sdk, const PxTransform& transform, const PxGeometry& geometry, const PxMaterial& material, PxF32 density)
{
	mKineActor = sdk.createRigidDynamic(transform);
	if(!mKineActor)
		return false;

	PxShape* shape = mKineActor->createShape(geometry, material);
	shape->userData = (void*)PX_MAKEFOURCC('C','C','T','S'); // Mark as a "CCT Shape"
	mKineActor->setRigidDynamicFlag(PxRigidDynamicFlag::eKINEMATIC, true);

	PxRigidBodyExt::updateMassAndInertia(*mKineActor, density);
	mScene->addActor(*mKineActor);
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

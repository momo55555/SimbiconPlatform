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
#include "SqPruner.h"

using namespace physx;
using namespace Ice;
using namespace Sq;

// pruner == acceleration structure

PRUNERCREATE::PRUNERCREATE()
{
	mExpectedWorldBox.setEmpty();
	mUpAxis				= 2;
	mSubdivisionLevel	= 7;

	mNbStaticObjects	= 0;
	mNbDynamicObjects	= 0;
	mStaticType			= PxPruningStructure::eLAST;
	mDynamicType		= PxPruningStructure::eLAST;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static HandleManager* gHM = NULL;

void ReleaseSignatureManager()
{
	if(gHM && gHM->GetNbObjects())
	{
		PX_ASSERT(!"The signature manager still contains objects! Please release everything before closing the lib.");
	}
	PX_DELETE_AND_RESET(gHM);
}

Signature::Signature() : mTimestamp(0)
{
	// Lazy create the signature manager. If no signatures are ever used, the ram won't be wasted.
	if(!gHM)	gHM = PX_NEW(HandleManager);
	PX_ASSERT(gHM && "Signature::Signature: handle manager not created!");
	if(gHM)	mStructureHandle = gHM->Add(this);
}

Signature::~Signature()
{
	PX_ASSERT(gHM && "Signature::~Signature: handle manager not created!");
	if(gHM)	gHM->Remove(mStructureHandle);

	// We can't automatically release the signature manager here! Else it gets deleted when the last
	// signature is deleted, which means all counters will be reset for next signature, possibly
	// mapping some previous signature marks around...
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Pruner::Pruner() :
	mCullFunc			(NULL),
	mStabFunc			(NULL),
	mOverlapSphereFunc	(NULL),
	mOverlapAABBFunc	(NULL),
	mOverlapOBBFunc		(NULL),
	mOverlapCapsuleFunc	(NULL)
{
}

#include "SqUtilities.h"
// PT: this is the generic code
void Pruner::addShapes(PxU32 nbShapes, SceneQueryShapeData*const* PX_RESTRICT shapes)
{
	SceneQueryShapeData* nextSqData = shapes[0];
	for(PxU32 i=0; i<nbShapes; i++)
	{
		SceneQueryShapeData* sqData = nextSqData;
		PX_ASSERT(sqData);

		if(i!=nbShapes-1)
		{
			nextSqData = shapes[i+1];
			Ps::prefetch128(nextSqData);
		}

		// PT: same as "AddObject", optimized
		PX_ASSERT(sqData->mHandle==INVALID_PRUNING_HANDLE);

		// PT: TODO: sigh... this stuff is so broken. In the original version doing "AddObject" immediately followed by "UpdateObject" was useless.
		// We need to figure out if it changed, and why.
		AddObject(*sqData);
		UpdateObject(*sqData);
	}
}

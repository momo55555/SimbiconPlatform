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

// WARNING: limited to 64K objects

// ### Hmmm, the handle can actually be implicit !
// Or at least, we know it's always sorted in this class, i.e. Array[object.handle] = object.

PruningPool::PruningPool() : 
	mNbObjects		(0),
	mMaxNbObjects	(0),
	mWorldBoxes		(NULL),
	mObjects		(NULL)
{
}

PruningPool::~PruningPool()
{
	PX_FREE_AND_RESET(mWorldBoxes);
	PX_FREE_AND_RESET(mObjects);
}

bool PruningPool::Init(PxU32 nb_objects)
{
	if(!nb_objects)	return false;	// Use dynamic arrays

	PX_FREE_AND_RESET(mWorldBoxes);
	PX_FREE_AND_RESET(mObjects);

	mNbObjects		= 0;
	mMaxNbObjects	= nb_objects;

	mWorldBoxes	= (PxBounds3*)	PX_ALLOC(sizeof(PxBounds3)*nb_objects);
	mObjects	= (Prunable**)	PX_ALLOC(sizeof(Prunable*)*nb_objects);

	return true;
}

bool PruningPool::Resize()
{
	// Check we need resizing
	if(mNbObjects==mMaxNbObjects)
	{
		if(mMaxNbObjects==0xffff)	return false;
		PxU32 MaxNbObjects = mMaxNbObjects;
		if(!MaxNbObjects)	MaxNbObjects = 4;
		else				MaxNbObjects <<= 1;
		if(MaxNbObjects>0xffff)	MaxNbObjects = 0xffff;
		mMaxNbObjects = MaxNbObjects;

		PxBounds3*	NewBoxes	= (PxBounds3*)	PX_ALLOC(sizeof(PxBounds3)*mMaxNbObjects);
		Prunable**	NewPrunes	= (Prunable**)	PX_ALLOC(sizeof(Prunable*)*mMaxNbObjects);

		if(mWorldBoxes)	Ps::memCopy(NewBoxes, mWorldBoxes, mNbObjects*sizeof(PxBounds3));
		if(mObjects)	Ps::memCopy(NewPrunes, mObjects, mNbObjects*sizeof(Prunable*));

		PX_FREE_AND_RESET(mWorldBoxes);
		PX_FREE_AND_RESET(mObjects);

		mWorldBoxes	= NewBoxes;
		mObjects	= NewPrunes;
	}
	return true;
}

bool PruningPool::AddObject(Prunable& object, SwapCallback callback, void* user_data)
{
	// Resize if needed
	if(mNbObjects==mMaxNbObjects && !Resize())
		return false;

	const PxU32 NbObjects = mNbObjects++;
	mWorldBoxes[NbObjects].setEmpty();
	mObjects[NbObjects] = &object;
	object.mHandle = NbObjects;
	if(callback)
		(callback)(PX_INVALID_U32, NbObjects, user_data);

	return true;
}

void PruningPool::RemoveObject(Prunable& object, SwapCallback callback, void* user_data)
{
	if(callback)
		(callback)(object.mHandle, PX_INVALID_U32, user_data);

	PX_ASSERT(mNbObjects);
	const PxU32 LastM = mNbObjects - 1;
	if(LastM!=object.mHandle)
	{
		mWorldBoxes[object.mHandle]	= mWorldBoxes[LastM];
		mObjects[object.mHandle]	= mObjects[LastM];
		mObjects[LastM]->mHandle	= object.mHandle;
		if(callback)
			(callback)(LastM, object.mHandle, user_data);
	}

	mNbObjects = LastM;

	// Mark as removed
//	if(callback)	(callback)(object.mHandle, PX_INVALID_U32, user_data);
	object.mHandle = INVALID_PRUNING_HANDLE;
}

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
#include "IceHandleManager.h"

using namespace physx;
using namespace Ice;

//! Initial list size
#define DEFAULT_HANDLEMANAGER_SIZE	2

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Constructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HandleManager::HandleManager() : mCurrentNbObjects(0), mNbFreeIndices(0)
{
	mMaxNbObjects	= DEFAULT_HANDLEMANAGER_SIZE;
	mObjects		= (void**)PX_ALLOC(sizeof(void*)*mMaxNbObjects);
	mOutToIn		= (PxU16*)PX_ALLOC(sizeof(PxU16)*mMaxNbObjects);
	mInToOut		= (PxU16*)PX_ALLOC(sizeof(PxU16)*mMaxNbObjects);
	mStamps			= (PxU16*)PX_ALLOC(sizeof(PxU16)*mMaxNbObjects);
	Ps::memSet(mOutToIn, 0xff, mMaxNbObjects*sizeof(PxU16));
	Ps::memSet(mInToOut, 0xff, mMaxNbObjects*sizeof(PxU16));
	Ps::memZero(mStamps, mMaxNbObjects*sizeof(PxU16));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Destructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HandleManager::~HandleManager()
{
	SetupLists();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Releases old arrays and assign new ones.
 *	\param		objects	[in] new objects address
 *	\param		oti		[in] new out-to-in address
 *	\param		ito		[in] new in-to-out address
 *	\param		stamps	[in] new stamps address
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool HandleManager::SetupLists(void** objects, PxU16* oti, PxU16* ito, PxU16* stamps)
{
	// Release old data
	PX_FREE_AND_RESET(mStamps);
	PX_FREE_AND_RESET(mInToOut);
	PX_FREE_AND_RESET(mOutToIn);
	PX_FREE_AND_RESET(mObjects);
	// Assign new data
	mObjects	= objects;
	mOutToIn	= oti;
	mInToOut	= ito;
	mStamps		= stamps;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Adds an object to the manager.
 *	\param		object	[in] the new object to be added.
 *	\return		The object's handle. Use it as further reference for the remove method.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Handle HandleManager::Add(void* object)
{
	// Are there any free indices I should recycle?
	if(mNbFreeIndices)
	{
		PxU16 FreeIndex = mInToOut[mCurrentNbObjects];		// Get the recycled virtual index
		mObjects[mCurrentNbObjects]	= object;				// The physical location is always at the end of the list (it never has holes).
		mOutToIn[FreeIndex]			= mCurrentNbObjects++;	// Update virtual-to-physical remapping table.
		mNbFreeIndices--;
		return (mStamps[FreeIndex]<<16)|FreeIndex;			// Return virtual index (handle) to the client app
	}
	else
	{
		PX_ASSERT(mCurrentNbObjects<0xffff && "Internal error - 64K objects in HandleManager!");

		// Is the array large enough for another entry?
		if(mCurrentNbObjects==mMaxNbObjects)
		{
			// Nope! Resize all arrays (could be avoided with linked lists... one day)
			mMaxNbObjects<<=1;													// The more you eat, the more you're given
			if(mMaxNbObjects>0xffff)	mMaxNbObjects = 0xffff;					// Clamp to 64K
			void** NewList		= (void**)PX_ALLOC(sizeof(void*)*mMaxNbObjects);		// New physical list
			PxU16* NewOTI		= (PxU16*)PX_ALLOC(sizeof(PxU16)*mMaxNbObjects);		// New remapping table
			PxU16* NewITO		= (PxU16*)PX_ALLOC(sizeof(PxU16)*mMaxNbObjects);		// New remapping table
			PxU16* NewStamps	= (PxU16*)PX_ALLOC(sizeof(PxU16)*mMaxNbObjects);		// New stamps
			Ps::memCopy(NewList, mObjects,	mCurrentNbObjects*sizeof(void*));	// Copy old data
			Ps::memCopy(NewOTI, mOutToIn,	mCurrentNbObjects*sizeof(PxU16));	// Copy old data
			Ps::memCopy(NewITO, mInToOut,	mCurrentNbObjects*sizeof(PxU16));	// Copy old data
			Ps::memCopy(NewStamps, mStamps,	mCurrentNbObjects*sizeof(PxU16));	// Copy old data
			Ps::memSet(NewOTI+mCurrentNbObjects, 0xff, (mMaxNbObjects-mCurrentNbObjects)*sizeof(PxU16));
			Ps::memSet(NewITO+mCurrentNbObjects, 0xff, (mMaxNbObjects-mCurrentNbObjects)*sizeof(PxU16));
			Ps::memZero(NewStamps+mCurrentNbObjects, (mMaxNbObjects-mCurrentNbObjects)*sizeof(PxU16));
			SetupLists(NewList, NewOTI, NewITO, NewStamps);
		}

		mObjects[mCurrentNbObjects]	= object;				// Store object at mCurrentNbObjects = physical index = virtual index
		mOutToIn[mCurrentNbObjects]	= mCurrentNbObjects;	// Update virtual-to-physical remapping table.
		mInToOut[mCurrentNbObjects]	= mCurrentNbObjects;	// Update physical-to-virtual remapping table.
		PxU32 tmp = mCurrentNbObjects++;
		return (mStamps[tmp]<<16)|tmp;						// Return virtual index (handle) to the client app
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Removes an object from the manager.
 *	\param		handle	[in] the handle returned from the Add() method.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void HandleManager::Remove(Handle handle)
{
    PxU16 VirtualIndex = PxU16(handle);
	if(VirtualIndex>=mMaxNbObjects)		return;		// Invalid handle
	PxU16 PhysicalIndex = mOutToIn[VirtualIndex];	// Get the physical index
	if(PhysicalIndex==0xffff)			return;		// Value has already been deleted
	if(PhysicalIndex>=mMaxNbObjects)	return;		// Invalid index

	// There must be at least one valid entry.
	if(mCurrentNbObjects)
	{
        if(mStamps[VirtualIndex]!=handle>>16)	return;							// Stamp mismatch => index has been recycled

		// Update list so that there's no hole
		mObjects[PhysicalIndex]					= mObjects[--mCurrentNbObjects];// Move the real object so that the array has no holes.
		mOutToIn[mInToOut[mCurrentNbObjects]]	= PhysicalIndex;				// Update virtual-to-physical remapping table.
		mInToOut[PhysicalIndex]					= mInToOut[mCurrentNbObjects];	// Update physical-to-virtual remapping table.
		// Keep track of the recyclable virtual index
		mInToOut[mCurrentNbObjects]				= VirtualIndex;					// Store the free virtual index/handle at the end of mInToOut
		mOutToIn[VirtualIndex]					= 0xffff;						// Invalidate the entry
		mNbFreeIndices++;														// One more free index
		mStamps[VirtualIndex]++;												// Update stamp
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Gets an object from the list. Returns real pointer according to handle.
 *	\param		handle	[in] the handle returned from the Add() method.
 *	\return		the corresponding object
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void* HandleManager::GetObject(Handle handle) const
{
	PxU16 VirtualIndex = PxU16(handle);
	if(VirtualIndex>=mMaxNbObjects)			return NULL;	// Invalid handle
	PxU16 PhysicalIndex = mOutToIn[VirtualIndex];			// Get physical index
	if(PhysicalIndex==0xffff)				return NULL;	// Object has been deleted
	if(PhysicalIndex>=mMaxNbObjects)		return NULL;	// Index is invalid
	if(mStamps[VirtualIndex]!=handle>>16)	return NULL;	// Index has been recycled
	return mObjects[PhysicalIndex];							// Returns stored pointer
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Remaps the inner array in an app-friendly order. Of course all handles remain valid.
 *	\param		ranks	[in] remapping table => ranks[i] = index of new object i in old list
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool HandleManager::Remap(const PxU32* ranks)
{
	// Checking
	if(!ranks)	return false;

	// Get some bytes
	void** NewList	= (void**)PX_ALLOC(sizeof(void*)*mMaxNbObjects);
	PxU16* NewOTI	= (PxU16*)PX_ALLOC(sizeof(PxU16)*mMaxNbObjects);
	PxU16* NewITO	= (PxU16*)PX_ALLOC(sizeof(PxU16)*mMaxNbObjects);
	Ps::memSet(NewOTI,	0xff, mMaxNbObjects*sizeof(PxU16));
	Ps::memSet(NewITO,	0xff, mMaxNbObjects*sizeof(PxU16));
	// Stamps are not modified since they're handle-indexed

	// Rebuild internal lists
	for(PxU32 i=0;i<mCurrentNbObjects;i++)
	{
		// Current physical index in mList is i. New index is ranks[i].
		PxU32 NewIndex = ranks[i];

		// Check 0 <= NewIndex < mCurrentNbObjects
		if(NewIndex>=mCurrentNbObjects)
		{
			PX_FREE_AND_RESET(NewList);
			PX_FREE_AND_RESET(NewOTI);
			PX_FREE_AND_RESET(NewITO);
			return false;
		}

		NewList[i] = mObjects[NewIndex];	// Update new list
		PxU16 Handle = mInToOut[NewIndex];	// Catch current handle (which is not modified by the remapping process)
		NewITO[i] = Handle;					// Update ITO internal remapping table
		NewOTI[Handle] = i;					// Update OTI internal remapping table
	}

	// Fix free indices
	for(PxU32 i=0;i<mNbFreeIndices;i++)
	{
		// Copy the recycled handle (still the same) in the new list
		NewITO[mCurrentNbObjects+i] = mInToOut[mCurrentNbObjects+i];
	}

	// Swap lists

	// Release old data
	PX_FREE_AND_RESET(mInToOut);
	PX_FREE_AND_RESET(mOutToIn);
	PX_FREE_AND_RESET(mObjects);
	// Assign new data
	mObjects	= NewList;
	mOutToIn	= NewOTI;
	mInToOut	= NewITO;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Gets used ram.
 *	\return		number of bytes used
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PxU32 HandleManager::GetUsedRam() const
{
	PxU32 Ram = sizeof(HandleManager);
	Ram += mMaxNbObjects * sizeof(PxU16);	// mOutToIn
	Ram += mMaxNbObjects * sizeof(PxU16);	// mInToOut
	Ram += mMaxNbObjects * sizeof(PxU16);	// mStamps
	Ram += mMaxNbObjects * sizeof(PxU32);	// mList
	return Ram;
}

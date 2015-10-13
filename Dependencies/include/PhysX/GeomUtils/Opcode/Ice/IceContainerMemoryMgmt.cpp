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
#include "IceContainer.h"

using namespace physx;
using namespace Ice;

#define INVALID_ID 0xffffffff

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Initializes the container so that it uses an external memory buffer. The container doesn't own the memory, resizing is disabled.
 *	\param		max_entries		[in] max number of entries in the container
 *	\param		entries			[in] external memory buffer
 *	\param		allow_resize	[in] false to prevent resizing the array (forced fixed length), true to enable it. In that last case the
 *									initial memory is NOT released, it's up to the caller. That way the initial memory can be on the stack.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Container::InitSharedBuffers(PxU32 max_entries, PxU32* entries, bool allow_resize)
{
	// PT: copy the code to avoid an LHS....
//	Empty();	// Make sure everything has been released
	if(IR(mGrowthFactor)!=INVALID_ID && SIR(mGrowthFactor)>=0)	// PT: avoid very bad FCMP
		PX_DELETE_POD(mEntries);	// Release memory if we own it

	mCurNbEntries = 0;
	mMaxNbEntries	= max_entries;
	mEntries		= entries;
	if(allow_resize)
		mGrowthFactor = -mGrowthFactor;	// Negative growth ==> we don't own the memory, resize is enabled
	else
		IR(mGrowthFactor) = INVALID_ID;	// Invalid growth ==> we don't own the memory, resize is disabled
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Resizes the container.
 *	\param		needed	[in] assume the container can be added at least "needed" values
 *	\return		true if success.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Container::Resize(PxU32 needed)
{
	// Check growth is allowed
	if(IR(mGrowthFactor)==INVALID_ID)
	{
		PX_ASSERT(!"Invalid operation - trying to resize a static buffer!");
		return false;
	}
	// else we're going to resize the array. If the growth is negative, we don't own the current memory so we shouldn't release it
	const float GrowthFactor = fabsf(mGrowthFactor);
	const bool ReleaseMemory = mGrowthFactor>0.0f;

	// Get more entries
	mMaxNbEntries = mMaxNbEntries ? PxU32(float(mMaxNbEntries)*GrowthFactor) : 2;	// Default nb Entries = 2
	if(mMaxNbEntries<mCurNbEntries + needed)	mMaxNbEntries = mCurNbEntries + needed;

	// Get some bytes for new entries
	PxU32*	NewEntries = PX_NEW(PxU32)[mMaxNbEntries];

	// Copy old data if needed
	if(mCurNbEntries)	Ps::memCopy(NewEntries, mEntries, mCurNbEntries*sizeof(PxU32));

	// Delete old data if needed
	if(ReleaseMemory)
		PX_DELETE_POD(mEntries);

	// Assign new pointer
	mEntries = NewEntries;
	// We now own the memory => make growth factor positive
	mGrowthFactor = GrowthFactor;

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Clears the container. All stored values are deleted, and it frees used ram.
 *	\see		Reset()
 *	\return		Self-Reference
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Container& Container::Empty()
{
	if(IR(mGrowthFactor)!=INVALID_ID && SIR(mGrowthFactor)>=0)	// PT: avoid very bad FCMP
	{
		PX_DELETE_POD(mEntries);	// Release memory if we own it
		mMaxNbEntries = 0;
	}
	mCurNbEntries = 0;
	return *this;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Sets the initial size of the container. If it already contains something, it's discarded.
 *	\param		nb		[in] Number of entries
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Container::SetSize(PxU32 nb)
{
	// Make sure it's empty
	Empty();

	// Checkings
	if(!nb)	return false;

	// Initialize for nb entries
	mMaxNbEntries = nb;

	// Get some bytes for new entries
	mEntries = PX_NEW(PxU32)[mMaxNbEntries];

	return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Refits the container and get rid of unused bytes.
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Container::Refit()
{
	// Check refit is allowed
	if(IR(mGrowthFactor)==INVALID_ID || SIR(mGrowthFactor)<0)	// PT: avoid very bad FCMP
	{
		PX_ASSERT(!"Invalid operation - trying to refit a static buffer!");
		return false;
	}

	// Get just enough entries
	mMaxNbEntries = mCurNbEntries;
	if(!mMaxNbEntries)	return false;

	// Get just enough bytes
	PxU32*	NewEntries = PX_NEW(PxU32)[mMaxNbEntries];

	// Copy old data
	Ps::memCopy(NewEntries, mEntries, mCurNbEntries*sizeof(PxU32));

	// Delete old data
	PX_DELETE_POD(mEntries);

	// Assign new pointer
	mEntries = NewEntries;

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Clears the container. All stored values are deleted, and it frees used ram.
 *	\see		Reset()
 *	\return		Self-Reference
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ContainerSizeT& ContainerSizeT::Empty()
{
	if(mGrowthFactor>=0.0f)
		PX_DELETE_POD(mEntries); // Release memory if we own it
	mCurNbEntries = mMaxNbEntries = 0;
	return *this;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Resizes the container.
 *	\param		needed	[in] assume the container can be added at least "needed" values
 *	\return		true if success.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool ContainerSizeT::Resize(PxU32 needed)
{
	// Check growth is allowed
	if(mGrowthFactor<=0.0f)
	{
		PX_ASSERT(!"Invalid operation - trying to resize a static buffer!");
		return false;
	}

	// Get more entries
	mMaxNbEntries = mMaxNbEntries ? PxU32(float(mMaxNbEntries)*mGrowthFactor) : 2;	// Default nb Entries = 2
	if(mMaxNbEntries<mCurNbEntries + needed)	mMaxNbEntries = mCurNbEntries + needed;

	// Get some bytes for new entries
	size_t*	NewEntries = PX_NEW(size_t)[mMaxNbEntries];

	// Copy old data if needed
	if(mCurNbEntries)	Ps::memCopy(NewEntries, mEntries, mCurNbEntries*sizeof(size_t));

	// Delete old data
	PX_DELETE_POD(mEntries);

	// Assign new pointer
	mEntries = NewEntries;

	return true;
}

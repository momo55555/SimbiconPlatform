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


#include "PsFoundation.h"

#include "PxErrorCallback.h"
#include "PxQuat.h"
#include "PsThread.h"
#include "PsUtilities.h"
#include "PsTempAllocator.h"
#include "PsPAEventSrc.h"
#include "PsString.h"

#include <stdio.h>

#pragma warning(disable:4996) // intentionally suppressing this warning message

#if defined(PX_WINDOWS) && !defined(PX_VC9)
#	define vsnprintf _vsnprintf
#endif

namespace physx
{
namespace shdfnd3
{

Foundation::Foundation(PxErrorCallback& errc, PxAllocatorCallback& alloc):
	mErrorCallback(errc),
	mAllocator(alloc),
	mErrorMask(PxErrorCode::Enum(~0))
{
}

Foundation::~Foundation()
{
	// deallocate temp buffer allocations
	Allocator alloc;
	for(PxU32 i=0; i<mTempAllocFreeTable.size(); ++i)
	{
		for(TempAllocatorChunk* ptr = mTempAllocFreeTable[i]; ptr; )
		{
			TempAllocatorChunk* next = ptr->mNext;
			alloc.deallocate(ptr);
			ptr = next;
		}
	}
	mTempAllocFreeTable.reset();
}

PxAllocatorCallback& Foundation::getAllocatorCallback() const
{
	return mAllocator.getBaseAllocator();
}

void Foundation::error(PxErrorCode::Enum c, const char* file, int line, const char* messageFmt, ...)
{
	va_list va;
	va_start(va, messageFmt);
	errorImpl(c, file, line, messageFmt, va);
	va_end(va);
}

void Foundation::errorImpl(PxErrorCode::Enum e, const char* file, int line, const char* messageFmt, va_list va)
{
	PX_ASSERT(messageFmt);
	if (e & mErrorMask)
	{
		//this function is reentrant but user's error callback may not be, so...
		Mutex::ScopedLock lock(mErrorMutex);	

		//create a string from the format strings and the optional params
		// We draw the line at a 1MB string.
		const int maxSize = 1000000;

		// If the string is less than 161 characters,
		// allocate it on the stack because this saves
		// the malloc/free time.
		const int stackBufSize = 161;
		int bufSize = stackBufSize;
		char stackBuffer[stackBufSize+1];
		char* stringBuffer = stackBuffer;
		char *heapBuffer = 0;

		while(0 > vsnprintf(stringBuffer, bufSize, messageFmt, va) && bufSize < maxSize)
		{
			PX_FREE(heapBuffer);
			bufSize *= 2;
			heapBuffer = PX_NEW(char)[bufSize+1];
			stringBuffer = heapBuffer;
		}

		mErrorCallback.reportError(e, stringBuffer, file, line);

		PX_FREE(heapBuffer);
	}
}

bool Foundation::createInstance(PxU32 version, PxErrorCallback& errc, PxAllocatorCallback& alloc)
{
	if (version != PX_PUBLIC_FOUNDATION_VERSION)
	{
		char* buffer = new char[256];
		physx::string::sprintf_s(buffer,256, "Wrong version: foundation version is %d, tried to create %d", PX_PUBLIC_FOUNDATION_VERSION, version);
		errc.reportError(PxErrorCode::eINVALID_PARAMETER, buffer, __FILE__, __LINE__);
		return 0;
	}

	if ( mInstance == NULL )
	{
		// if we don't assign this here, the Foundation object can't create member
		// subobjects which require the allocator

		mInstance = reinterpret_cast<Foundation*>( alloc.allocate( 
			sizeof(Foundation), "Foundation", __FILE__,__LINE__));

		PX_PLACEMENT_NEW(mInstance, Foundation)(errc, alloc);

		return true;
	}

	return false;
}

void Foundation::destroyInstance()
{
	PX_ASSERT(mInstance != NULL);
	PxAllocatorCallback& alloc = mInstance->mAllocator.getBaseAllocator();
	mInstance->~Foundation();
	alloc.deallocate(mInstance);
	mInstance = 0;
}

void Foundation::setInstance(Foundation* ptr) 
{
	mInstance = ptr;
}

PxErrorCallback& Foundation::getErrorCallback() const
{
	return mErrorCallback;
}

void Foundation::setErrorLevel(PxErrorCode::Enum mask)
{
	mErrorMask = mask;
}

PxErrorCode::Enum Foundation::getErrorLevel() const
{
	return mErrorMask;
}

PxAllocatorCallback& getAllocator()
{ 
	return getFoundation().getCheckedAllocator();
}

void *Foundation::AlignCheckAllocator::allocate(size_t size, const char* typeName, const char* filename, int line)
{
	void *addr = mAllocator.allocate(size, typeName, filename, line);
	if(!(reinterpret_cast<size_t>(addr)&15))
	{
		//Same comment as before in the allocation system.
		//We don't lock the listener array mutex because of an assumption
		//where the listener array is rarely changing.
		PxU32 theCount = mListenerCount;
		for( PxU32 idx = 0; idx < theCount; ++idx )
			mListeners[idx]->onAllocation( size, typeName, filename, line, addr );
		return addr;
	}

	getFoundation().getErrorCallback().reportError(PxErrorCode::eOUT_OF_MEMORY, "Allocations for PhysX must be 16-byte aligned.", __FILE__, __LINE__);
	return 0;
}


Foundation* Foundation::mInstance = NULL;


} // namespace shdfnd3
} // namespace physx

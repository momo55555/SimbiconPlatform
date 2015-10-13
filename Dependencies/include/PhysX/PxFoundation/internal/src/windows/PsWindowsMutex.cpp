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


#include "windows/PsWindowsInclude.h"
#include "PsUserAllocated.h"
#include "PsMutex.h"
#include "PxAssert.h"

namespace physx
{
namespace shdfnd3
{

namespace 
{
	CRITICAL_SECTION* getMutex(MutexImpl* impl)
	{
		return reinterpret_cast<CRITICAL_SECTION*>(impl);
	}
}

MutexImpl::MutexImpl() 
{ 
	InitializeCriticalSection(getMutex(this)); 
}

MutexImpl::~MutexImpl() 
{ 
	DeleteCriticalSection(getMutex(this)); 
}

bool MutexImpl::lock()
{
	EnterCriticalSection(getMutex(this));
	return true;
}

bool MutexImpl::trylock()
{
	return TryEnterCriticalSection(getMutex(this)) != 0;
}

bool MutexImpl::unlock()
{
	LeaveCriticalSection(getMutex(this));
	return true;
}

const PxU32 MutexImpl::size = sizeof(CRITICAL_SECTION);

class ReadWriteLockImpl
{
public:
    HANDLE				hReaderEvent;
    HANDLE				hMutex;
    CRITICAL_SECTION	writerMutex;
    LONG				counter;	//count the number of readers in the lock.
    LONG				recursionCounter;	//handle recursive writer locking
};

ReadWriteLock::ReadWriteLock()
{
    mImpl = reinterpret_cast<ReadWriteLockImpl*>(PX_ALLOC(sizeof(ReadWriteLockImpl)));

    mImpl->hReaderEvent = CreateEvent(NULL,TRUE,FALSE,NULL);
    PX_ASSERT( mImpl->hReaderEvent != NULL );

    mImpl->hMutex = CreateEvent( NULL, FALSE, TRUE, NULL );
    PX_ASSERT( mImpl->hMutex != NULL );
    
    InitializeCriticalSection( &mImpl->writerMutex );
    mImpl->counter = -1;
    mImpl->recursionCounter = 0;
}

ReadWriteLock::~ReadWriteLock()
{
    if( mImpl->hReaderEvent != NULL )
    {
        CloseHandle(mImpl->hReaderEvent);
    }

    if( mImpl->hMutex != NULL )
    {
        CloseHandle( mImpl->hMutex );
    }

    DeleteCriticalSection( &mImpl->writerMutex );

    PX_FREE( mImpl );
}

void ReadWriteLock::lockReader()
{
    if( InterlockedIncrement( &mImpl->counter ) == 0 )
    {
        WaitForSingleObject( mImpl->hMutex, INFINITE );
        SetEvent( mImpl->hReaderEvent );
    }
    
    WaitForSingleObject( mImpl->hReaderEvent, INFINITE );
}

void ReadWriteLock::lockWriter()
{
    EnterCriticalSection( &mImpl->writerMutex );

    //we may already have the global mutex(really an event so we have to handle recursion ourselves)
    if( ++mImpl->recursionCounter == 1 )
    {
        WaitForSingleObject( mImpl->hMutex, INFINITE );
    }
}

void ReadWriteLock::unlockReader()
{
    if( InterlockedDecrement( &mImpl->counter ) < 0 )
    {
        ResetEvent( mImpl->hReaderEvent );
        SetEvent( mImpl->hMutex );
    }
}

void ReadWriteLock::unlockWriter()
{
    if( --mImpl->recursionCounter == 0 )
    {
        SetEvent( mImpl->hMutex );
    }

    LeaveCriticalSection( &mImpl->writerMutex );
}

} // namespace shdfnd3
} // namespace physx


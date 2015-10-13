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
#include "PsFoundation.h"
#include "PsThread.h"
#include "PxAssert.h"

// an exception for setting the thread name in microsoft debuggers
#define PS_MS_VC_EXCEPTION 0x406D1388

namespace physx
{
namespace shdfnd3
{

	// struct for naming a thread in the debugger
#pragma pack(push, 8)

	typedef struct tagTHREADNAME_INFO
	{
	   DWORD dwType;		// Must be 0x1000.
	   LPCSTR szName;		// Pointer to name (in user addr space).
	   DWORD dwThreadID;	// Thread ID (-1=caller thread).
	   DWORD dwFlags;		// Reserved for future use, must be zero.
	} THREADNAME_INFO;

#pragma pack(pop)

namespace {

	DWORD WINAPI PxThreadStart(LPVOID arg)
	{
		((Thread *) arg)->execute();
		return 0;
	}
}

class ThreadImpl
{
public:
	enum State
	{
		NotStarted,
		Started,
		Stopped
	};

	HANDLE				thread;
	LONG				quitNow;				//Should be 32bit aligned on SMP systems.
	State				state;
	DWORD				threadID;

	Thread::ExecuteFn	fn;
	void *				arg;
};

Thread::Id Thread::getId()
{
	return static_cast<Id>(GetCurrentThreadId());
}

Thread::Thread()
{
	mImpl = (ThreadImpl *)PX_ALLOC(sizeof(ThreadImpl));
	mImpl->thread = NULL;
	mImpl->state = ThreadImpl::NotStarted;
	mImpl->quitNow = 0;
}

Thread::Thread(ExecuteFn fn, void *arg)
{
	mImpl = (ThreadImpl *)PX_ALLOC(sizeof(ThreadImpl));
	mImpl->thread = NULL;
	mImpl->state = ThreadImpl::NotStarted;
	mImpl->quitNow = 0;
	mImpl->fn = fn;
	mImpl->arg = arg;

	start(0);
}


Thread::~Thread()
{
	if(mImpl->state == ThreadImpl::Started)
		kill();
	CloseHandle(mImpl->thread);
	PX_FREE(mImpl);
}

void Thread::start(PxU32 stackSize)
{
	if(mImpl->state != ThreadImpl::NotStarted)
		return;
	mImpl->state = ThreadImpl::Started;

	mImpl->thread = CreateThread(NULL, stackSize, PxThreadStart, (LPVOID) this, 0, &mImpl->threadID);
}

void Thread::signalQuit()
{
	InterlockedIncrement(&(mImpl->quitNow));
}

bool Thread::waitForQuit()
{
	if(mImpl->state==ThreadImpl::NotStarted)
		return false;

	WaitForSingleObject(mImpl->thread,INFINITE);
	return true;
}


bool Thread::quitIsSignalled()
{
	return InterlockedCompareExchange(&(mImpl->quitNow),0,0)!=0;
}

void Thread::quit()
{
	mImpl->state = ThreadImpl::Stopped;
	ExitThread(0);
}

void Thread::kill()
{
	if (mImpl->state==ThreadImpl::Started)
		TerminateThread(mImpl->thread, 0);
	mImpl->state = ThreadImpl::Stopped;
}

void Thread::sleep(PxU32 ms)
{
	Sleep(ms);
}

void Thread::yield()
{
	SwitchToThread();
}

void Thread::execute(void)
{
	(*mImpl->fn)(mImpl->arg);
}

PxU32 Thread::setAffinityMask(PxU32 mask)
{
	return mask ? (PxU32)SetThreadAffinityMask(mImpl->thread, mask) : 0;
}

void Thread::setName(const char *name)
{
	THREADNAME_INFO info;
	info.dwType		= 0x1000;
	info.szName		= name;
	info.dwThreadID	= mImpl->threadID;
	info.dwFlags	= 0;

	// C++ Exceptions are disabled for this project, but SEH is not (and cannot be)
	// http://stackoverflow.com/questions/943087/what-exactly-will-happen-if-i-disable-c-exceptions-in-a-project
	__try
	{
		RaiseException( PS_MS_VC_EXCEPTION, 0, sizeof(info)/sizeof(ULONG_PTR), (ULONG_PTR*)&info );
	}
	__except(EXCEPTION_EXECUTE_HANDLER)
	{
		// this runs if not attached to a debugger (thus not really naming the thread)
	}
}

void Thread::setPriority(ThreadPriority::Enum prio)
{
	switch(prio)
	{
	case ThreadPriority::eHIGH:
		SetThreadPriority(mImpl->thread,THREAD_PRIORITY_HIGHEST);
		break;
	case ThreadPriority::eABOVE_NORMAL:
		SetThreadPriority(mImpl->thread,THREAD_PRIORITY_ABOVE_NORMAL);
		break;
	case ThreadPriority::eNORMAL:
		SetThreadPriority(mImpl->thread,THREAD_PRIORITY_NORMAL);
		break;
	case ThreadPriority::eBELOW_NORMAL:
		SetThreadPriority(mImpl->thread,THREAD_PRIORITY_BELOW_NORMAL);
		break;
	case ThreadPriority::eLOW:
		SetThreadPriority(mImpl->thread,THREAD_PRIORITY_LOWEST);
		break;
	default:
		break;
	}
}

ThreadPriority::Enum Thread::getPriority( Id threadId )
{
	ThreadPriority::Enum retval = ThreadPriority::eLOW;
	int priority = GetThreadPriority( (HANDLE) threadId );
	PX_COMPILE_TIME_ASSERT( THREAD_PRIORITY_HIGHEST > THREAD_PRIORITY_ABOVE_NORMAL );
	if ( priority >= THREAD_PRIORITY_HIGHEST )
		retval = ThreadPriority::eHIGH;
	else if ( priority >= THREAD_PRIORITY_ABOVE_NORMAL )
		retval = ThreadPriority::eABOVE_NORMAL;
	else if ( priority >= THREAD_PRIORITY_NORMAL )
		retval = ThreadPriority::eNORMAL;
	else if ( priority >= THREAD_PRIORITY_BELOW_NORMAL )
		retval = ThreadPriority::eBELOW_NORMAL;
	return retval;
}


PxU32 TlsAlloc()
{
	DWORD rv=::TlsAlloc();
	PX_ASSERT(rv!=TLS_OUT_OF_INDEXES);
	return (PxU32)rv;
}

void TlsFree(PxU32 index)
{
	::TlsFree(index);
}

void *TlsGet(PxU32 index)
{
	return ::TlsGetValue(index);
}

PxU32 TlsSet(PxU32 index,void *value)
{
	return ::TlsSetValue(index,value);
}

const PxU32 Thread::DEFAULT_STACK_SIZE  = 1048576;

} // namespace shdfnd3
} // namespace physx

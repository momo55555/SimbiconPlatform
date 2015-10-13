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


#include "PsPAEventSrc.h"
#include "PsUserAllocated.h"
#include "windows/PsWindowsInclude.h"

namespace physx
{
namespace shdfnd3
{

typedef void *PmHandle;					 // handle for connection to data collector

typedef PmHandle	(PmCreateSourceConnection_FUNC)(PxU32 version);
typedef bool        (PmDestroySourceConnection_FUNC)(PmHandle hconn);
typedef EventID		(PmRegisterEvent_FUNC)(PmHandle hconn, const char *name);
typedef bool		(PmSubmitEvent_FUNC)(PmHandle hconn, EventID id, PxU32 data0, PxU32 data1, PxU8 data2);
typedef bool		(PmSubmitEventWithTimestamp_FUNC)(PmHandle hconn, EventID id, PxU32 data0, PxU32 data1, PxU8 data2, PxU64 timestamp);
typedef bool		(PmEventEnabled_FUNC)(PmHandle hconn, EventID id);
typedef bool		(PmEventLoggingEnabled_FUNC)(PmHandle hconn);
typedef bool        (PmSubmitEventWTFUnsafe_FUNC)(PmHandle hconn, EventID id, PxU32 data0, PxU32 data1, PxU8 data2, PxU64 timestamp);
typedef bool        (PmSubmitEventLock_FUNC)(PmHandle hconn);
typedef bool        (PmSubmitEventUnlock_FUNC)(PmHandle hconn);

typedef struct _THREAD_BASIC_INFORMATION {
    PxU32               ExitStatus;
    void               *TebBaseAddress;
    PxU32               ClientId;
    PxU32               reserved;
    PxU32               AffinityMask;
    PxU32               Priority;
    PxU32               BasePriority;
} THREAD_BASIC_INFORMATION, *PTHREAD_BASIC_INFORMATION;

typedef PxU32 (__stdcall tinfo_FUNC)(void *ThreadHandle,
    PxU32 ThreadInformationClass,
    PTHREAD_BASIC_INFORMATION ThreadInformation,
    PxU32 ThreadInformationLength,
    PxU32 *ReturnLength );

#define PERFMON_VERSION                  0x06121900

class ConnImpl : public UserAllocated 
{
public:
	ConnImpl();
	~ConnImpl();

	PX_INLINE EventID	registerEvent(const char *name);
	PX_INLINE bool	    submitEvent(EventID id, PxU32 data0, PxU32 data1, PxU8 data2);
	PX_INLINE bool	    submitEventWithTimestamp(EventID id, PxU32 data0, PxU32 data1, PxU8 data2, PxU64 timestamp);
	PX_INLINE bool	    eventEnabled(EventID id);
	PX_INLINE bool	    eventLoggingEnabled();
	PX_INLINE PxU32	    GetThreadPriority();
	PX_INLINE PxU8	    GetProcID();

	PmSubmitEventWTFUnsafe_FUNC     *submitEventWTFUnsafe;
	PmSubmitEventLock_FUNC          *submitEventLock;
	PmSubmitEventUnlock_FUNC        *submitEventUnLock;
	PmHandle						 mConnHandle;

private:
	PmCreateSourceConnection_FUNC	*createFunc;
	PmDestroySourceConnection_FUNC	*destroyFunc;
	PmRegisterEvent_FUNC			*registerEventFunc;
	PmSubmitEvent_FUNC				*submitEventFunc;
	PmSubmitEventWithTimestamp_FUNC *submitEventWithTimestampFunc;
	PmEventEnabled_FUNC				*eventEnabledFunc;
	PmEventLoggingEnabled_FUNC		*eventLoggingEnabledFunc;

	HMODULE		mNtDll;
	HMODULE		mPmDll;
	tinfo_FUNC *mNtQueryInfo;
};

/* PAUtils */


PAUtils::PAUtils()
{
    mImpl = NULL;
}

PAUtils::~PAUtils()
{
    if( mImpl )
        PX_DELETE(mImpl);
}

EventID PAUtils::registerEvent(const char *name)
{
    if( !mImpl )
       mImpl = PX_NEW(ConnImpl);
    return mImpl->registerEvent( name );
}

bool    PAUtils::isEventEnabled( EventID id )
{
    if( mImpl )
        return mImpl->eventEnabled( id );
    else
        return false;
}

bool    PAUtils::isEnabled()
{
    if( mImpl )
        return mImpl->eventLoggingEnabled();
    else
        return false;
}

void    PAUtils::rawEvent( EventID id, PxU32 data0, PxU32 data1, PxU8 data2 )
{
    if( mImpl )
        mImpl->submitEvent( id, data0, data1, data2 );
}

void PAUtils::startEvent( EventID id, PxU16 data )
{
    if( mImpl && mImpl->eventLoggingEnabled() )
    {
        PxU32 threadCpuData = 0;
        ((PxU8*)(&threadCpuData))[0] = (PxU8) mImpl->GetThreadPriority();
        ((PxU8*)(&threadCpuData))[1] = mImpl->GetProcID();
        ((PxU16*)(&threadCpuData))[1] = data;
        mImpl->submitEvent( id, GetCurrentThreadId(), threadCpuData, PsEventTypes::eSTART );
    }
}

void PAUtils::stopEvent( EventID id, PxU16 data )
{
    if( mImpl && mImpl->eventLoggingEnabled() )
    {
        PxU32 threadCpuData = 0;
        ((PxU8*)(&threadCpuData))[0] = (PxU8) mImpl->GetThreadPriority();
        ((PxU8*)(&threadCpuData))[1] = mImpl->GetProcID();
        ((PxU16*)(&threadCpuData))[1] = data;
        mImpl->submitEvent( id, GetCurrentThreadId(), threadCpuData, PsEventTypes::eSTOP );
    }
}

void PAUtils::statEvent(EventID id, PxU32 stat)
{
    if( mImpl )
        mImpl->submitEvent( id, stat, GetCurrentThreadId(), PsEventTypes::eSTAT );
}

void PAUtils::statEvent(EventID id, PxU32 stat, PxU32 ident)
{
    if( mImpl )
        mImpl->submitEvent( id, stat, ident, PsEventTypes::eSTAT );
}

void PAUtils::debugEvent(EventID id, PxU32 data0, PxU32 data1)
{
    if( mImpl )
        mImpl->submitEvent( id, data0, data1, PsEventTypes::eDEBUG );
}

bool PAUtils::lock()
{
    if( !mImpl || !mImpl->submitEventLock )
    {
        static bool reportOnce /* = false */;
        if( !reportOnce )
			getFoundation().error(PX_INFO, "The AGPerfMON DLL does not contain a lock() method, GPU profiling disabled.");
        reportOnce = true;
        return false;
    }
    return mImpl->submitEventLock( mImpl->mConnHandle );
}

bool PAUtils::rawEventWithTimestamp( EventID id, PxU64 timestamp, PxU32 data0, PxU32 data1, PxU8 data2 )
{
    // lock must be acquired
    return mImpl->submitEventWTFUnsafe( mImpl->mConnHandle, id, data0, data1, data2, timestamp );
}

bool PAUtils::unlock()
{
    if( mImpl )
        return mImpl->submitEventUnLock( mImpl->mConnHandle );
    else
        return false;
}

ConnImpl::ConnImpl()
	: mNtDll( 0 )
	, mPmDll( 0 )
	, mConnHandle( 0 )
	, mNtQueryInfo( 0 )
{
#ifdef UNICODE
#if defined(PX_X64)
#   define PERFMON_LIB_NAME				L"AgPerfMon_x64.dll"
#else
#   define PERFMON_LIB_NAME				L"AgPerfMon_x86.dll"
#endif
#   define NTDLL_LIB_NAME               L"NTDLL.DLL"
#else
#if defined(PX_X64)
#   define PERFMON_LIB_NAME				"AgPerfMon_x64.dll"
#else
#   define PERFMON_LIB_NAME				"AgPerfMon_x86.dll"
#endif
#   define NTDLL_LIB_NAME               "NTDLL.DLL"
#endif
	mNtDll = LoadLibrary( NTDLL_LIB_NAME );
	if( mNtDll )
	{
		mNtQueryInfo = (tinfo_FUNC*) GetProcAddress( mNtDll, "NtQueryInformationThread");
	}

    mPmDll = LoadLibrary( PERFMON_LIB_NAME );
    if( !mPmDll )
    {
		//shdfnd3::getFoundation().error(PX_INFO, "Unable to find AgPerfMon.dll");
        return;
    }

    /** get the function pointers to the event src api */
    createFunc              = (PmCreateSourceConnection_FUNC*)GetProcAddress(mPmDll, "AgPmCreateSourceConnection");
    destroyFunc             = (PmDestroySourceConnection_FUNC*)GetProcAddress(mPmDll, "AgPmDestroySourceConnection");
    registerEventFunc       = (PmRegisterEvent_FUNC*)GetProcAddress(mPmDll, "AgPmRegisterEvent");
    submitEventFunc         = (PmSubmitEvent_FUNC*)GetProcAddress(mPmDll, "AgPmSubmitEvent");
	submitEventWithTimestampFunc = (PmSubmitEventWithTimestamp_FUNC*)GetProcAddress(mPmDll, "AgPmSubmitEventWithTimestamp");
    submitEventWTFUnsafe    = (PmSubmitEventWTFUnsafe_FUNC*)GetProcAddress(mPmDll, "AgPmSubmitEventWithTimestampUnsafe");
    submitEventLock         = (PmSubmitEventLock_FUNC*)GetProcAddress(mPmDll, "AgPmSubmitEventLock");
    submitEventUnLock       = (PmSubmitEventUnlock_FUNC*)GetProcAddress(mPmDll, "AgPmSubmitEventUnlock");
    eventEnabledFunc        = (PmEventEnabled_FUNC*)GetProcAddress(mPmDll, "AgPmEventEnabled");
    eventLoggingEnabledFunc = (PmEventLoggingEnabled_FUNC*)GetProcAddress(mPmDll, "AgPmEventLoggingEnabled");

    if(!createFunc          ||
       !destroyFunc         ||
       !registerEventFunc   ||
       !submitEventFunc     ||
       !eventEnabledFunc    ||
	   !submitEventWithTimestampFunc ||
       !eventLoggingEnabledFunc)
    {
		shdfnd3::getFoundation().error(PX_INFO,
            "The AGPerfMON DLL does not contain all required entrypoints, PerfMON is disabled.  Install the latest AgPerfMon.dll.");
        FreeLibrary( mPmDll );
        mPmDll = 0;
        return;
    }

    mConnHandle = createFunc(PERFMON_VERSION);

	if (!mConnHandle)
	{
		shdfnd3::getFoundation().error(PX_INFO,
			"The AGPerfMON DLL was not able to load properly, PerfMON is disabled.");
		FreeLibrary( mPmDll );
		mPmDll = 0;
	}
}

ConnImpl::~ConnImpl()
{
	if( mConnHandle )
	{
		destroyFunc( mConnHandle );
		mConnHandle = 0;
	}

    if( mNtDll )
	{
        FreeLibrary( mNtDll );
	}

	if( mPmDll )
	{
		FreeLibrary( mPmDll );
	}
}

PxU32 ConnImpl::GetThreadPriority()
{
    PxU32 retVal = 0;
    
    if( mNtQueryInfo )
    {
        /** call into the Native API to get the dynamic thread priority */
        PxU32 len;
        THREAD_BASIC_INFORMATION tbi;
		PxU32 status = mNtQueryInfo(GetCurrentThread(), 0, &tbi, sizeof(tbi), &len); //740ns
        if( !status )
            retVal = tbi.Priority;
    }
    
    return retVal;
}

#ifdef PX_X64
#include <intrin.h>
PxU8 ConnImpl::GetProcID()
{
   int CPUInfo[4];
   int InfoType = 0;
    __cpuid(CPUInfo, InfoType);

    return (PxU8) (CPUInfo[1] >> 24); // APIC Physical ID
}
#else
PxU8 ConnImpl::GetProcID()
{
    unsigned int reg_ebx = 0;
    __asm 
    {
        mov  eax, 1         // call cpuid with eax = 1
        cpuid
        mov  reg_ebx, ebx   // Has APIC ID info
    }
    return (PxU8)(reg_ebx >> 24); 
}
#endif

EventID ConnImpl::registerEvent( const char *name )
{
    if( 0 == mConnHandle )
        return INVALID_EVENT_ID;

    return registerEventFunc(mConnHandle, name);
}

bool ConnImpl::submitEvent( EventID id, PxU32 data0, PxU32 data1, PxU8 data2 )
{
    if( 0 == mConnHandle )
        return false;

    return submitEventFunc( mConnHandle, id, data0, data1, data2 );
}

#if 0  // unreferenced (unused by Windows PAUtils)
bool ConnImpl::PmSubmitEventWithTimestamp( EventID id, PxU32 data0, PxU32 data1, PxU8 data2, PxU64 timestamp )
{
    if( 0 == mConnHandle )
        return false;

    return submitEventWithTimestampFunc( mConnHandle, id, data0, data1, data2, timestamp );
}
#endif

bool ConnImpl::eventEnabled(EventID id)
{
    if( 0 == mConnHandle )
        return false;

    return eventEnabledFunc(mConnHandle, id);
}

bool ConnImpl::eventLoggingEnabled()
{
    if( 0 == mConnHandle )
        return false;

    return eventLoggingEnabledFunc(mConnHandle);
}

} // end shdfnd3 namespace
} // end physx namespace

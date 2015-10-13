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


#ifndef PVD_CONNECTION_MANAGER_IMPL_H
#define PVD_CONNECTION_MANAGER_IMPL_H

#include "PvdConnectionManager.h"
#include "PsArray.h"
#include "PsMutex.h"
#include "PsUserAllocated.h"
#include "PvdConnectionTraits.h"
#include "PxProfileMemoryEventTypes.h"
#include "PxProfileZoneManager.h"
#include "PvdCommFileStream.h"

namespace PVD
{
	inline char* AllocateMemory( size_t inSize, const char* typeName, const char* file, PxI32 line)
	{
		if ( file == NULL )
			file = "";
		return (char*) physx::shdfnd::getFoundation().getAllocator().allocate(inSize, typeName, file, line);
	}
	
	inline char* UntrackedAllocateMemory( size_t inSize, const char* typeName, const char* file, PxI32 line)
	{
		if ( file == NULL )
			file = "";
		return (char*) physx::shdfnd::getFoundation().getAllocatorCallback().allocate(inSize, typeName, file, line);
	}

	//bypass the memory tracking system.  We need this for the data stream that sends information
	//from the data tracking system.
	inline void DeallocateMemory( char* inMem )
	{
		if ( inMem )
			physx::shdfnd::getFoundation().getAllocator().deallocate(inMem);
	}
	
	inline void UntrackedDeallocateMemory( char* inMem )
	{
		if ( inMem )
			physx::shdfnd::getFoundation().getAllocatorCallback().deallocate(inMem);
	}

	template<typename TDataType>
	inline void DeallocateMemory( TDataType* inMem ) { DeallocateMemory( reinterpret_cast< char* >( inMem ) ); }

#include "ClientDllAllocator.h"
	
	typedef ClientDllAllocator<char> TAllocatorType;
	typedef ClientDllUntrackedAllocator<char> UntrackedAllocatorType;
	typedef physx::shdfnd::MutexT<TAllocatorType> TMutexType;
	typedef TMutexType::ScopedLock TScopedLockType;
	typedef PvdConnectionTraits< TMutexType,TScopedLockType,TAllocatorType,UntrackedAllocatorType> TPhysxConnectionTraits; 

	
	class PvdConnectionManagerImpl : public UserAllocated
											, public PvdConnectionManager
											, public PvdConnectionHandler
											, public PxAllocationListener
	{
		typedef TPhysxConnectionTraits		TTraitsType;
		typedef void (PvdConnectionManagerImpl::*TAllocationHandler)( size_t size, const char* typeName, const char* filename, int line, void* allocatedMemory );
		typedef void (PvdConnectionManagerImpl::*TDeallocationHandler)( void* allocatedMemory );
		

		Array<PvdConnectionHandler*>		mHandlers;
		PvdConnectionDataProvider*			mDataProvider;
		InternalPvdConnection*				mConnection;
		TMutexType							mMutex;
		TMutexType							mAllocationMutex;
		PxProfileMemoryEventRecorder*		mRecorder;
		TAllocationHandler					mAllocationHandler;
		TDeallocationHandler				mDeallocationHandler;
		physx::PxProfileZoneManager*		mProfileSDKManager;

		


	public:
		PvdConnectionManagerImpl( bool inRecordMemoryEvents = true )
			: mDataProvider( NULL )
			, mConnection( NULL )
			, mRecorder( NULL )
			, mProfileSDKManager( NULL )
		{
			if ( inRecordMemoryEvents )
			{
				mRecorder = &PxProfileMemoryEventRecorder::createRecorder( &Foundation::getInstance() );
				mAllocationHandler = &PvdConnectionManagerImpl::recordAllocationHandler;
				mDeallocationHandler = &PvdConnectionManagerImpl::recordDeallocationHandler;
			}
			else
			{
				mAllocationHandler = &PvdConnectionManagerImpl::nullAllocationHandler;
				mDeallocationHandler = &PvdConnectionManagerImpl::nullDeallocationHandler;
			}
			Foundation::getInstance().getAllocator().registerAllocationListener( *this );
		}

		virtual ~PvdConnectionManagerImpl()
		{
			Foundation::getInstance().getAllocator().deregisterAllocationListener( *this );
			disconnect();
			if ( mRecorder != NULL )
				mRecorder->release();
			mRecorder = NULL;
			mHandlers.clear();
		}
			
		
		virtual void onPvdConnected( PvdConnection* inFactory )
		{
			disconnect();
			PX_ASSERT( mConnection == NULL );
			TScopedLockType lock( mMutex );
			{
				//Have to be very careful with the mutex order; allocation mutex needs to be
				//unlocked before the rest of the system hears the connect message.
				mConnection = static_cast<InternalPvdConnection*>( inFactory );
				if ( mConnection != NULL )
					mConnection->addRef();
				if ( mProfileSDKManager != NULL )
					mProfileSDKManager->addProfileZoneHandler( *mConnection );

				bool isMemoryConnection = inFactory->getConnectionType() & PvdConnectionType::Memory;
				if ( isMemoryConnection )
				{
					TScopedLockType lock( mAllocationMutex );
					if ( mRecorder != NULL )
						mRecorder->setListener( mConnection );
					else
					{
						mAllocationHandler = &PvdConnectionManagerImpl::connectionAllocationHandler;
						mDeallocationHandler = &PvdConnectionManagerImpl::connectionDeallocationHandler;
					}
				}
			}
			for ( PxU32 idx = 0; idx < mHandlers.size(); ++idx )
				mHandlers[idx]->onPvdConnected( mConnection );
		}
		virtual void disableConnectionMemoryEvents()
		{	
			TScopedLockType lock( mAllocationMutex );
			if ( mRecorder )
				mRecorder->setListener( NULL );
			else
			{
				mAllocationHandler = &PvdConnectionManagerImpl::nullAllocationHandler;
				mDeallocationHandler = &PvdConnectionManagerImpl::nullDeallocationHandler;
			}
		}
		virtual void onPvdDisconnected( PvdConnection* inFactory ) 
		{
			if ( mConnection != NULL )
			{
				TScopedLockType lock( mMutex );
				for ( PxU32 idx = 0; idx < mHandlers.size(); ++idx )
					mHandlers[idx]->onPvdDisconnected( inFactory );

				{
					disableConnectionMemoryEvents();
					//Disconnect the profiler from the system.
					if ( mProfileSDKManager != NULL )
						mProfileSDKManager->removeProfileZoneHandler( *mConnection );
					mConnection->release();
					mConnection = NULL;
				}
			}
		}

		void setDataProvider( PvdConnectionDataProvider* inProvider )
		{
			TScopedLockType lock( mMutex );
			mDataProvider = inProvider;
		}

		virtual void setProfileZoneManager( physx::PxProfileZoneManager& inManager )
		{
			mProfileSDKManager = &inManager;
		}
		
		virtual void addHandler( PvdConnectionHandler* inHandler )
		{
			TScopedLockType lock( mMutex );
			//Do a push_front operation.
			PxU32 numHandlers = mHandlers.size();
			mHandlers.resize( numHandlers + 1 );
			memMove( mHandlers.begin() + 1, mHandlers.begin(), numHandlers * sizeof( PvdConnectionHandler* ) );
			mHandlers[0] = inHandler;
			if ( mConnection != NULL )
				inHandler->onPvdConnected( mConnection );
		}

		virtual void removeHandler( PvdConnectionHandler* inHandler )
		{
			TScopedLockType lock( mMutex );
			for ( PxU32 idx = 0; idx < mHandlers.size(); ++idx )
			{
				if ( mHandlers[idx] == inHandler )
				{
					mHandlers.replaceWithLast( idx );
					if ( mConnection != NULL )
						inHandler->onPvdDisconnected( mConnection );
				}
			}
		}

		virtual PvdConnection* connect( PvdCommInStream* inInStream
												, PvdCommOutStream* inOutStream
												, bool inUseErrorCheckingForDebugStream
												, TConnectionFlagsType inConnectionType
												, bool inUseDoubleBufferedOutput = true )
		{
			TScopedLockType lock( mMutex );
			return TTraitsType::createDebuggerConnectionFactory( inInStream, inOutStream, inUseErrorCheckingForDebugStream, inConnectionType, this, mDataProvider, inUseDoubleBufferedOutput );
		}
		
		virtual PvdConnection* connect( const char* inFilename
										, bool inCheckAPI
										, TConnectionFlagsType inConnectionType )
		{
			FStreamCommOutStream* theStream = PX_NEW( FStreamCommOutStream ) ();
			if ( theStream->open( inFilename ) )
				return connect( NULL, theStream, inCheckAPI, inConnectionType );
			else
				PX_DELETE( theStream );
			return NULL;
		}
		
		virtual PvdConnection* getCurrentConnection()
		{
			return mConnection;
		}
		
		virtual void disconnect()
		{
			TScopedLockType lock( mMutex );
			disableConnectionMemoryEvents();
			if ( mConnection != NULL )
				mConnection->disconnect();
		}

		virtual void release() 
		{ 
			PX_DELETE( this ); 
		}

		template<typename TNetworkStreamType>
		PvdConnection* connect( const char* inHost
										, int inPort
										, unsigned int inTimeoutInMilliseconds
										, bool inCheckAPI
										, TConnectionFlagsType inConnectionType )
		{
			PvdCommInStream* theInStream;
			PvdCommOutStream* theOutStream;
			if ( TNetworkStreamType().connect( inHost, inPort, inTimeoutInMilliseconds, theInStream, theOutStream ) )
				return connect( theInStream, theOutStream, inCheckAPI, inConnectionType );
			return NULL;
		}
		
		virtual void flushMemoryEvents()
		{
			if ( mConnection != NULL )
			{
				TScopedLockType lock( mAllocationMutex );
				mConnection->flushMemoryEvents();
			}
		}

		void onAllocation( size_t size, const char* typeName, const char* filename, int line, void* allocatedMemory )
		{
			(this->*mAllocationHandler)( size, typeName, filename, line, allocatedMemory );	
		}

		void onDeallocation( void* addr )
		{
			(this->*mDeallocationHandler)(addr);	
		}
		

		void recordAllocationHandler( size_t size, const char* typeName, const char* filename, int line, void* allocatedMemory )
		{
			TScopedLockType lock( mAllocationMutex );
			mRecorder->onAllocation( size, typeName, filename, line, allocatedMemory );
		}

		void recordDeallocationHandler( void* addr )
		{
			TScopedLockType lock( mAllocationMutex );
			mRecorder->onDeallocation( addr );
		}

		void connectionAllocationHandler( size_t size, const char* typeName, const char* filename, int line, void* allocatedMemory )
		{
			TScopedLockType lock( mAllocationMutex );
			mConnection->onAllocation( size, typeName, filename, line, allocatedMemory );
		}
		
		void connectionDeallocationHandler( void* addr )
		{
			TScopedLockType lock( mAllocationMutex );
			mConnection->onDeallocation( addr );
		}
		
		void nullAllocationHandler( size_t , const char* , const char* , int , void* )
		{
		}

		void nullDeallocationHandler( void* )
		{
		}
	};
}

#endif
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

#ifndef PVD_PVDCONNECTIONFACTORYIMPL_H
#define PVD_PVDCONNECTIONFACTORYIMPL_H
#include "PvdConnection.h"
#include "PVDCommByteStream.h"
#include "PvdConnectionPhysXProfilingClient.h"
#include "PvdConnectionManager.h"
#include "PvdConnectionPropertyHandler.h"
#include "PvdConnectionDataProvider.h"
#include "PVDConnectionStreamDebuggerMessageReader.h"
#include "PvdConnectionPhysXMemoryEventClient.h"
#include "PvdDoubleBufferedOutStream.h"

namespace PVD
{
	struct StreamOwnerDisconnectionType
	{
		virtual void signalDisconnect(){}
	};
	//Implements the stream interfaces w/o explicitly
	//implementing the interfaces.  Allows multiple threads
	//to handle the streams.
	template<typename TMutexType
			, typename TScopedLockType
			, typename TDeleteOperator=SDeleteOperator>
	struct PvdConnectionStreamOwner
	{
		PvdCommOutStream*	mOutStream;
		PvdCommInStream*	mInStream;
		PxU32				mRefCount;
		mutable TMutexType	mMutex;
		mutable TMutexType	mReadMutex;
		bool				mLocked;

		PvdConnectionStreamOwner( const PvdConnectionStreamOwner& inOther );
		PvdConnectionStreamOwner& operator=( const PvdConnectionStreamOwner& inOther );
	public:
		PvdConnectionStreamOwner( PvdCommOutStream*	inOutStream, PvdCommInStream* inInStream )
			: mOutStream( inOutStream )
			, mInStream( inInStream )
			, mRefCount( 0 )
			, mLocked( false )
		{
		}
		PX_INLINE void addRef()
		{
			TScopedLockType theLocker( mMutex );
			++mRefCount;
		}
		void disconnect()
		{
			if ( mOutStream )
				mOutStream->disconnect();
			//disconnect calls must be reentrant.
			if ( mInStream )
				mInStream->disconnect();

			{
				TScopedLockType theLocker( mMutex );
				PvdCommOutStream* theOutStream = mOutStream;
				mOutStream = NULL;
				if ( theOutStream ) theOutStream->destroy();
			}
			{
				TScopedLockType theReadLocker( mReadMutex );
				PvdCommInStream* theInStream = mInStream;
				mInStream = NULL;
				if ( theInStream ) theInStream->destroy();
			}
		}

		PxU32 release()
		{
			PxU32 theTempCount = 0;
			{
				TScopedLockType theLocker( mMutex );
				if ( mRefCount )
					--mRefCount;
				theTempCount = mRefCount;
				if ( !theTempCount )
					disconnect();
			}
			//Unlock the mutex.
			if ( !theTempCount )
				TDeleteOperator()( this );
			return theTempCount;
		}


		PvdCommLayerError processError( PvdCommLayerError error )
		{
			if ( error != PvdCommLayerError::None )
				disconnect();
			return error;
		}

		template<typename TStreamType>
		PvdCommLayerError processError( PvdCommLayerError error, TStreamType* inStream )
		{
			if ( error != PvdCommLayerError::None )
				inStream->disconnect();
			return error;
		}

#define CHECK_STREAM_CONNECTION( stream ) if ( !stream || !stream->isConnected() ) return PvdCommLayerError::NetworkError

		/**
		 * Out stream implementation
		 */
		PvdCommLayerError write( const PxU8* inBytes, PxU32 inLength )
		{
			PX_ASSERT( mLocked ); //Assert that begin/end send were called before write
			CHECK_STREAM_CONNECTION( mOutStream );
			//This disconnect cannot be a full disconnect *because*
			//our send lock is held during this time.
			return processError( mOutStream->write( inBytes, inLength ), mOutStream );
		}
		
		PvdCommLayerError readBytes( PxU8* outBytes, PxU32 ioRequested )
		{
			TScopedLockType theLocker( mReadMutex );
			CHECK_STREAM_CONNECTION( mInStream );
			return processError( mInStream->readBytes( outBytes, ioRequested ) );
		}

		bool isConnected() const
		{
			{
				TScopedLockType theLocker( mMutex );
				if ( mOutStream != NULL ) return mOutStream->isConnected();
			}
			{
				TScopedLockType theLocker(mReadMutex);
				if ( mInStream != NULL ) return mInStream->isConnected();
			}
			return false;
		}

		void destroy()
		{
			release();
		}

		void lock()
		{
			mMutex.lock();
			PX_ASSERT( !mLocked ); //Assert we aren't double locked on the same thread
			mLocked = true;
		}

		void unlock()
		{
			PX_ASSERT( mLocked ); //Assert we were locked in the first place
			mLocked = false;
			mMutex.unlock();
		}

		bool isLocked() { return mLocked; }
		bool checkConnected()
		{
			if ( !isConnected() )
			{
				disconnect();
				return false;
			}
			return true;
		}

		void beginSend()
		{
			checkConnected();
			lock();
		}

		void endSend()
		{
			unlock();
		}
	};

	template<typename THandlerType>
	struct DebugMessageForward
	{
		THandlerType* mHandler;
		DebugMessageForward( THandlerType* inType )
			: mHandler( inType )
		{
		}
		template<typename TDataType>
		bool operator()( const TDataType& inDt )
		{
			mHandler->handleDebugMessage( inDt );
			return true;
		}
		bool operator()() { return false; }
	};

	template<typename TMutexType
			, typename TScopedLockType
			, typename TConnectionType
			, typename TAllocatorType
			, typename TDeleteOperator
			, typename TUntrackedConnectionType
			, typename TUntrackedDeleteOperator>
	class PvdConnectionImpl : public InternalPvdConnection, public PvdConnectionDataHandler, public StreamOwnerDisconnectionType, public PvdDataStreamOwner
	{
		typedef HashMap<PxU64, PvdConnectionPropertyHandler*, Hash<PxU64>, TAllocatorType > TInstanceToHandlerMapType;
		typedef PvdConnectionImpl< TMutexType, TScopedLockType, TConnectionType, TAllocatorType, TDeleteOperator, TUntrackedConnectionType, TUntrackedDeleteOperator > TThisType;
		typedef PvdConnectionStreamOwner<TMutexType,TScopedLockType,TDeleteOperator>	TOwnerType;
		typedef typename TConnectionType::TTypeCheckerType								TTypeCheckerType;
		typedef typename TConnectionType::TWriterType									TWriter;
		typedef typename TUntrackedConnectionType::TWriterType							TUntrackedWriter;
		typedef PvdConnectionPhysXProfilingClient										TProfileClientType;
		typedef PvdCommLayerInStreamImpl<TOwnerType>									TInStreamType;
		typedef PvdDataStreamEventInStream<false,TAllocatorType>						TEventInStreamType;
		typedef PvdConnectionStreamDebuggerMessageReader< TEventInStreamType, TAllocatorType > TDebugMessageReaderType;
		typedef physx::shdfnd::HashMap<RegisterPropertyStructKey
										, RegisterPropertyStructEntry<TAllocatorType>
										, RegisterPropertyStructKeyHasher
										, TAllocatorType> TPropertyDefinitionStructHash;

		
		TOwnerType*													mStreamOwner;
		TTypeCheckerType*											mTypeChecker;
		PxU32														mRefCount;
		Array<TProfileClientType*,TAllocatorType>					mProfilingClients;
		bool														mProfileDatatypesRegistered;
		TInStreamType												mInStream;
		PvdConnectionHandler*										mFactoryHandler;
		TInstanceToHandlerMapType									mInstanceToHandlerMap;
		PvdConnectionState::Enum									mCurrentConnectionState;
		TConnectionFlagsType										mConnectionType;
		Array<PxU8, TAllocatorType>									mReadBuffer;
		TMutexType													mConnectionStateMutex;
		PvdConnectionRunningProvider*								mRunningProvider;
		PvdConnectionPhysXMemoryEventClient*						mMemoryEventClient;
		TPropertyDefinitionStructHash								mPropertyStructDefinitions;


		PvdConnectionImpl( const PvdConnectionImpl& inOther );
		PvdConnectionImpl& operator=( const PvdConnectionImpl& inOther );

	public:
		PvdConnectionImpl( PvdCommOutStream*	inOutStream, PvdCommInStream* inInStream, TConnectionFlagsType inConnectionType, bool inUseDoubleBufferedOutput )
			: mStreamOwner( NULL )
			, mRefCount( 0 )
			, mProfileDatatypesRegistered( false )
			, mInStream( NULL )
			, mFactoryHandler( NULL )
			, mCurrentConnectionState( PvdConnectionState::Recording )
			, mConnectionType( inConnectionType )
			, mRunningProvider( NULL )
			, mMemoryEventClient( NULL )
		{
			mStreamOwner = reinterpret_cast<TOwnerType*>( TAllocatorType().allocate( sizeof( TOwnerType ), __FILE__, __LINE__ ) );
	
			if ( inOutStream != NULL && inUseDoubleBufferedOutput )
			{
				PxU32 theCapacity = 0x20000; //128K buffer size, works out to two 64KB buffers
#ifdef PX_WINDOWS
				theCapacity = 0x100000; // 1MB of buffer on windows only, works out to two 512KB buffers
#endif
				inOutStream = PX_NEW( DoubleBufferedOutStream )( theCapacity, inOutStream );
			}
			new ( mStreamOwner ) TOwnerType( inOutStream, inInStream );
			mStreamOwner->addRef();
			mTypeChecker = reinterpret_cast< TTypeCheckerType* >( TAllocatorType().allocate( sizeof( TTypeCheckerType ), __FILE__, __LINE__ ) );
			new ( mTypeChecker ) TTypeCheckerType();
			mTypeChecker->addRef();
			mInStream.mStream = mStreamOwner;
		}

		virtual ~PvdConnectionImpl()
		{
			mStreamOwner->disconnect();
			if ( mRunningProvider != NULL )
				mRunningProvider->destroy(); //stop trying to read any information.
			mRunningProvider = NULL;
			mFactoryHandler = NULL;
			setCurrentConnectionState( PvdConnectionState::Monitoring );
			for( PxU32 idx = 0; idx < mProfilingClients.size(); ++idx )
			{
				TDeleteOperator()( mProfilingClients[idx] );
			}
			mProfilingClients.clear();
			if ( mMemoryEventClient != NULL )
				TDeleteOperator()( mMemoryEventClient );
			mMemoryEventClient = NULL;
			mStreamOwner->release();
			mStreamOwner = NULL;
			mTypeChecker->release();
			mTypeChecker = NULL;
		}

		void setHandler( PvdConnectionHandler* inHandler )
		{
			mFactoryHandler = inHandler;
		}

		void signalDisconnect()
		{
			//release anyone blocked on us.
			setCurrentConnectionState( PvdConnectionState::Monitoring );
			//Ensure *we* don't get destroyed right here.
			//Also ensure the memory system is released.
			if ( mFactoryHandler != NULL )
				mFactoryHandler->onPvdDisconnected( this );
		}

		void sendStreamInitialization() 
		{ 
			mStreamOwner->addRef();
			//This will call destroy, which will call release
			//hence the spurious addref.
			TConnectionType::sendStreamInitialization( mStreamOwner ); 
			
			if ( mConnectionType & PvdConnectionType::Memory )
			{
				TUntrackedConnectionType* memoryConnection( doCreateConnection<TUntrackedConnectionType>() );
				registerProfileClasses( memoryConnection );
				
				PvdConnectionPhysXMemoryEventClient* theClient = reinterpret_cast<PvdConnectionPhysXMemoryEventClient*>( TAllocatorType().allocate( sizeof( PvdConnectionPhysXMemoryEventClient ), __FILE__, __LINE__ ) );
				new (theClient) PvdConnectionPhysXMemoryEventClient( memoryConnection );
				mMemoryEventClient = theClient;
				memoryConnection->localFlush();
				memoryConnection->disableCaching();
			}
		}
		virtual void registerPropertyHandler( PxU64 inInstance, PvdConnectionPropertyHandler* inHandler )
		{
			mInstanceToHandlerMap[inInstance] = inHandler;
		}
		
		template<typename TLocalConnectionType>
		TLocalConnectionType* doCreateConnection()
		{
			mStreamOwner->addRef();
			TScopedLockType theLocker( mStreamOwner->mMutex );
			typedef typename TLocalConnectionType::TLocalWriterType TLocalWriter;
			typedef typename TLocalConnectionType::TLocalAllocatorType TLocalAllocatorType;
			TLocalWriter* theWriter = reinterpret_cast<TLocalWriter*>( TLocalAllocatorType().allocate( sizeof(TWriter), __FILE__, __LINE__ ) );
			new ( theWriter ) TLocalWriter( mStreamOwner );
			TLocalConnectionType* theConnection = reinterpret_cast<TLocalConnectionType*>( TLocalAllocatorType().allocate( sizeof( TLocalConnectionType ), __FILE__, __LINE__ ) );
			new (theConnection) TLocalConnectionType( theWriter, mTypeChecker, this );
			return theConnection;
		}
		virtual PvdConnectionState::Enum getConnectionState()
		{
			TScopedLockType theLocker( mConnectionStateMutex );
			return mCurrentConnectionState;
		}
		virtual void checkConnection()
		{
			getConnectionState();
			if ( !isConnected() )
			{
				signalDisconnect();
			}
		}
		virtual TConnectionFlagsType getConnectionType() { return mConnectionType; }

		virtual PvdDataStream* createDataStream()
		{
			return doCreateConnection<TConnectionType>();
		}

		virtual bool isConnected()
		{
			return mStreamOwner->isConnected();
		}

		virtual void disconnect()
		{
			signalDisconnect();
		}

		template<typename TLocalConnectionType>
		void registerProfileClasses( TLocalConnectionType* profilingConnection )
		{
			if ( mProfileDatatypesRegistered == false )
			{
				TProfileClientType::registerProfilingClassNames( profilingConnection );
				profilingConnection->localFlush();
				mProfileDatatypesRegistered = true;
			}
		}
		
		virtual void onZoneAdded( PxProfileZone& inSDK )
		{
			if ( mConnectionType & PvdConnectionType::Profile )
			{
				TConnectionType* profilingConnection( doCreateConnection<TConnectionType>() );
				registerProfileClasses( profilingConnection );
				TProfileClientType* theClient = reinterpret_cast<TProfileClientType*>( TAllocatorType().allocate( sizeof( TProfileClientType ), __FILE__, __LINE__ ) );
				new (theClient) TProfileClientType( profilingConnection, &inSDK );
				profilingConnection->localFlush();
				//Ensure that we write directly to the lower level socket implementation
				profilingConnection->disableCaching();
				mProfilingClients.pushBack( theClient );
			}
		}
		
		virtual void onZoneRemoved( PxProfileZone& inSDK )
		{
			for ( PxU32 idx = 0; idx < mProfilingClients.size(); ++idx )
			{
				if ( mProfilingClients[idx]->getSDK() == &inSDK )
					mProfilingClients[idx]->removeFromSDK();
			}
		}

		virtual PvdUnknown* queryInterface( const char* ) { return NULL; }
		
		virtual void addRef()
		{
			TScopedLockType theLocker( mStreamOwner->mMutex );
			++mRefCount;
		}

		virtual void release()
		{
			PxU32 theTemp;
			{
				TScopedLockType theLocker( mStreamOwner->mMutex );
				if ( mRefCount )
					--mRefCount;
				theTemp = mRefCount;
			}
			if ( !theTemp )
				TDeleteOperator()( this );
		}

		void setDataProvider( PvdConnectionDataProvider* inProvider )
		{
			if ( mStreamOwner->mInStream != NULL && inProvider != NULL )
			{
				//Take into account we are giving the provider a new stream.
				mStreamOwner->addRef();
				mRunningProvider = inProvider->beginHandlingData( this, &mInStream );
			}
		}

		virtual void handleData( const PxU8* inData, PxU32 inLength )
		{
			for ( PxU32 idx = 0; idx < inLength; ++idx )
				mReadBuffer.pushBack( inData[idx] );
			parseReadBuffer();
		}

		virtual void handleNoMoreData()
		{
			//Ensure we don't leave things in the paused state.
			//This only happens when shutting down or when PVD disconnects
			//at an odd time.
			setCurrentConnectionState( PvdConnectionState::Monitoring );
		}

		void parseReadBuffer()
		{
			
			TEventInStreamType theEventReader;
			if ( mReadBuffer.size() == 0 )
				return;
			//Attempt to read the event header.
			PxU8* start = mReadBuffer.begin();
			PxU8* end = mReadBuffer.end();

			do
			{
				TEventInStreamType theEventReader;
				PxU32 streamVersion( createStreamInitialization().mStreamVersion );
				//Assume PVD is running from latest stream version
				theEventReader.setup( start, end, streamVersion );
				SEventHeader theHeader;
				theHeader.streamify( theEventReader );
				if ( theEventReader.hasFailed() == false 
					&& theEventReader.amountLeft() >= theHeader.mSize )
				{
					const PxU8* beginPtr = theEventReader.getReadPtr();
					const PxU8* endPtr = beginPtr + theHeader.mSize;
					theEventReader.setup( beginPtr, endPtr, streamVersion );
					TDebugMessageReaderType theDebugMessageReader;
					theDebugMessageReader.setup( theHeader, &theEventReader );
					while( theEventReader.isValid() )
						theDebugMessageReader.template readNextBatchEvent<int>( DebugMessageForward<TThisType>( this ) );
					mReadBuffer.removeRange( 0, static_cast<PxU32>( endPtr - start ) );
					start = mReadBuffer.begin();
					end = mReadBuffer.end();
				}
				else
					break; //exit the loop.
			} while ( start < end );	
		}

		void setCurrentConnectionState( PvdConnectionState::Enum inState )
		{
			bool unlock = false;
			{
				TScopedLockType theLocker( mConnectionStateMutex );
				if ( inState == mCurrentConnectionState )
					return;
				unlock = mCurrentConnectionState == PvdConnectionState::Paused;
				mCurrentConnectionState = inState;
			}

			if ( mCurrentConnectionState == PvdConnectionState::Paused )
				mConnectionStateMutex.lock();
			else if ( unlock )
				mConnectionStateMutex.unlock();
		}

		void handleDebugMessage( const DebugMessagePause& )
		{
			setCurrentConnectionState( PvdConnectionState::Paused );
		}
		void handleDebugMessage( const DebugMessageRecord& )
		{
			setCurrentConnectionState( PvdConnectionState::Recording );
		}
		void handleDebugMessage( const DebugMessageMonitor& )
		{
			setCurrentConnectionState( PvdConnectionState::Monitoring );
		}
		void handleDebugMessage( const DebugMessageDisconnect& )
		{
			disconnect();
		}
		void handleDebugMessage( const DebugMessagesetPropertyValue& inValue )
		{
			const typename TInstanceToHandlerMapType::Entry* theEntry = mInstanceToHandlerMap.find( inValue.mInstanceId );
			if ( theEntry != NULL )
				theEntry->second->onPropertyChanged( inValue.mInstanceId, inValue.mProperty, PvdCommLayerValue( inValue.mDatatype, inValue.mValue ) );
		}

		virtual void flushMemoryEvents()
		{
			if ( mMemoryEventClient != NULL )
				mMemoryEventClient->mBuffer->flushProfileEvents();
		}

		virtual void onAllocation( size_t inSize, const char* inType, const char* inFile, int inLine, void* inAddr )
		{
			if ( mMemoryEventClient != NULL )
				mMemoryEventClient->mBuffer->onAllocation( inSize, inType, inFile, inLine, inAddr );
		}

		virtual void onDeallocation( void* inAddr )
		{
			if ( mMemoryEventClient != NULL )
				mMemoryEventClient->mBuffer->onDeallocation( inAddr );
		}

		virtual void onPropertyStructDefinition( PxU32 inStructKey
												, PxU32 inNamespace
												, PxU32 inClass
												, PxU32 inStructByteSize
												, const PropertyStructEntry* inEntries
												, PxU32 inEntryCount )
		{
			TScopedLockType theLocker( mStreamOwner->mMutex );
			RegisterPropertyStructKey theKey( inStructKey, inNamespace );
			const typename TPropertyDefinitionStructHash::Entry* entry = mPropertyStructDefinitions.find( theKey ); 
			if ( entry ) return;
			mPropertyStructDefinitions.insert( theKey, RegisterPropertyStructEntry<TAllocatorType>() );
			RegisterPropertyStructEntry<TAllocatorType>& theEntry( mPropertyStructDefinitions[theKey] );
			theEntry.setup( inClass, inStructByteSize, inEntries, inEntryCount );
		}

		virtual RegisterPropertyStruct getStructDefinition( PxU32 inStructKey, PxU32 inNamespace )
		{
			TScopedLockType theLocker( mStreamOwner->mMutex );
			RegisterPropertyStructKey theKey( inStructKey, inNamespace );
			const typename TPropertyDefinitionStructHash::Entry* entry = mPropertyStructDefinitions.find( theKey ); 
			if ( entry )
			{
				const RegisterPropertyStructEntry<TAllocatorType>& theEntry( entry->second );
				return createRegisterPropertyStruct( inStructKey, theEntry.mClass, theEntry.mByteSize
													, static_cast<PxU32>( theEntry.mEntries.size() )
													, const_cast<PropertyStructEntry*>( theEntry.mEntries.begin() ) );
			}
			return createRegisterPropertyStruct( 0, 0, 0, 0, 0 );
		}
	};
}

#endif

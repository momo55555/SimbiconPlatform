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

#ifndef PVD_PVDCONNECTIONSTREAMWRITER_H
#define PVD_PVDCONNECTIONSTREAMWRITER_H

#include "PVDCommByteStream.h"
#include "PvdDataStreamEventOutStream.h"
#include "PvdRenderCommands.h"
#include "PsTime.h"

namespace PVD
{
	template<typename TStreamType
			, typename TAllocator>
	struct SDatatypeHandlerManager
	{
		typedef AbstractOutStreamDatatypeHandler<TStreamType> TDatatypeHandler;

		TStreamType*										mStream;
		physx::shdfnd::Array<TDatatypeHandler*,TAllocator> mDatatypeHandlers;
		physx::shdfnd::Array<TDatatypeHandler*,TAllocator> mUserDatatypeHandlers;
		SDatatypeHandlerManager()
			: mStream( NULL )
		{
			typedef AbstractOutStreamDatatypeHandlerCreator<TStreamType, TAllocator > TCreatorType;
			for (PxU8 idx = 1; idx < PvdCommLayerDatatype::Last; ++idx )
			{
				TDatatypeHandler* theHandler = commLayerDataOperate<TDatatypeHandler*>( PvdCommLayerDatatype( idx ), TCreatorType() );
				mDatatypeHandlers.pushBack( theHandler );
			}
		}
		
		~SDatatypeHandlerManager()
		{
			for ( PxU32 idx = 0; idx < mDatatypeHandlers.size(); ++idx )
				TAllocator().deallocate( reinterpret_cast<char*>(mDatatypeHandlers[idx]), sizeof(TAllocator) );
			mDatatypeHandlers.clear();
		}
		
		TDatatypeHandler* findHandlerForType( PvdCommLayerDatatype inType )
		{
			return mDatatypeHandlers[inType.mDatatype-1];
		}

		void setUserDatatypes( const PvdCommLayerDatatype* inTypes = NULL, PxU32 inPropCount = 0 )
		{
			mUserDatatypeHandlers.clear();
			for ( PxU32 idx =0; idx < inPropCount; ++idx )
				mUserDatatypeHandlers.pushBack( findHandlerForType( inTypes[idx] ) );
			updateStream();
		}
		
		TDatatypeHandler** getUserDatatypes() 
		{
			if ( mUserDatatypeHandlers.size() )
				return &mUserDatatypeHandlers[0];
			return NULL;
		}

		void updateStream()
		{
			mStream->setDatatypeHandlers( getUserDatatypes() );
		}

		void setStream( TStreamType* inStream )
		{
			for ( PxU32 idx = 0; idx < PvdCommLayerDatatype::Last - 1; ++idx )
			{
				if ( mDatatypeHandlers[idx] )
					mDatatypeHandlers[idx]->SetStream( inStream );
			}
			mStream = inStream;
		}
	};

	template<typename TStreamType>
	inline void beginSend( TStreamType* inStream ) { if( inStream ) inStream->beginSend(); }
	inline void beginSend( PvdCommOutStream* ) {}

	template<typename TStreamType>
	inline void endSend( TStreamType* inStream ) { if( inStream ) inStream->endSend(); }
	inline void endSend( PvdCommOutStream* ) {}

	template<typename TStreamType>
	struct ScopedStreamWatcher
	{
		TStreamType* mStream;
		ScopedStreamWatcher( TStreamType* inStream )
			: mStream( inStream )
		{
			PVD::beginSend( mStream );
		}
		~ScopedStreamWatcher() { PVD::endSend( mStream ); }
	};

	/**
	 *	Handles streaming the data out of the system.
	 *	The cache byte size tells us the cutoff point
	 *	when sending the blocks of information.  We could
	 *	queue the blocks until an end event or we are storing
	 *	too much data on this object.
	 *
	 *	There is a contract enforced where once the underlying
	 *	network stream returns an error we never call their API's
	 *	again other than destroy.
	 */
	template<typename TPXU8Container, 
			typename TAllocator,
			PxU32 TMaxCacheByteSize,
			typename TDeleteOperator=SDeleteOperator,
			typename TStreamType=PvdCommOutStream>
	class PvdDataStreamWriter
	{
	public:
		typedef PvdDataStreamEventOutStream<SCommOutStreamWriter<TStreamType> > TCommOutStream;
		typedef PvdDataStreamEventOutStream<SCommByteCounterWriter> TByteCounterStream;
		typedef PvdDataStreamEventOutStream<MemoryBufferWriter > TContainerStream;
	protected:


		//An event header sent over the network always has type, streamid, timestamp and size.
		//So there are 21 bytes.  If we decide to send a Multiple or
		//Batch event, there is another 4 bytes added to the header.
		//In the case of a Multiple event, there is another byte added
		//for the type of the data as it is uniformly typed.
		PxU8							mHeaderStorage[26];
		physx::profile::MemoryBuffer<>	mEventContainer; //Events get written first to the container, then the output stream.
		TStreamType*					mOutStream;
		PvdCommLayerError				mLastError;

		PvdConnectionEventType			mEventType; //The outermost meta-event type (multiple or batch)
		PxU32							mEventCount; //The number of events in the current multiple event run.
		PvdConnectionEventType			mMultipleType; //The type of event in the current multiple run.
		bool							mCachingEnabled; //If we can even use our cache buffer

		SDatatypeHandlerManager<TCommOutStream, TAllocator> mOutStreamHandlerManager;
		SDatatypeHandlerManager<TByteCounterStream, TAllocator> mByteCounterStreamHandlerManager;
		SDatatypeHandlerManager<TContainerStream, TAllocator> mContainerStreamHandlerManager;

		
		SCommByteCounterWriter mCounter;
		PvdDataStreamEventOutStream<SCommByteCounterWriter> mCountStream;
		
		SCommOutStreamWriter<TStreamType> mStreamWriter;
		PvdDataStreamEventOutStream< SCommOutStreamWriter<TStreamType> > mOutStreamStream;

		MemoryBufferWriter mContainerWriter;
		PvdDataStreamEventOutStream<MemoryBufferWriter > mContainerStream;


		PvdDataStreamWriter( const PvdDataStreamWriter& inOther );
		PvdDataStreamWriter& operator=( const PvdDataStreamWriter& inOther );

	public:
		inline PvdDataStreamWriter( TStreamType* inOutStream )
			: mOutStream( inOutStream )
			, mLastError( PvdCommLayerError::None )
			, mEventType( PvdConnectionEventType::EventBatch )
			, mEventCount( 0 )
			, mMultipleType( PvdConnectionEventType::Unknown )
			, mCachingEnabled( true )
			, mCountStream( &mCounter )
			, mStreamWriter( inOutStream )
			, mOutStreamStream( &mStreamWriter )
			, mContainerWriter( &mEventContainer )
			, mContainerStream( &mContainerWriter )
		{
			mByteCounterStreamHandlerManager.setStream( &mCountStream );
			mOutStreamHandlerManager.setStream( &mOutStreamStream );
			mContainerStreamHandlerManager.setStream( &mContainerStream );
			writeHeaderToContainer();
		}

		inline ~PvdDataStreamWriter() 
		{
			flush();
			if ( mOutStream )
				mOutStream->destroy();
			mOutStream = NULL;
		}

		inline void destroy() { TDeleteOperator()(this); }

		inline PvdCommLayerError flush()
		{
			localFlush();
			return mLastError;
		}

		inline PvdCommLayerError localFlush()
		{
			return SendContainer();
		}


		inline bool isConnected() { return mOutStream->isConnected(); }

		
		inline PvdCommLayerError sendEvent( SStreamInitialization inInit )
		{
			ScopedStreamWatcher<TStreamType> theWatcher( mOutStream );
			inInit.streamify( mOutStreamStream );
			mLastError = mStreamWriter.mLastError;
			return mLastError;
		}

		inline PvdCommLayerError sendEvent( const SBeginPropertyBlock& inBlock )
		{
			return BeginMultiple( inBlock, PVD::PvdConnectionEventType::SendPropertyBlock );
		}

		inline PvdCommLayerError sendEvent( const SRawPropertyBlock& inBlock )
		{
			if ( inBlock.mCheckData != NULL )
			{
				SSendPropertyBlock checkBlock = createSendPropertyBlock( inBlock.mDatatypes, inBlock.mPropertyCount, inBlock.mInstanceId, inBlock.mCheckData );
				mCounter.mByteCount = 0;
				checkBlock.streamify( mCountStream );
				PxU32 expected = mCounter.mByteCount;
				PX_FORCE_PARAMETER_REFERENCE(expected);
				mCounter.mByteCount = 0;
				const_cast<SRawPropertyBlock&>(inBlock).streamify( mCountStream );
				PX_ASSERT( expected == mCounter.mByteCount );
			}
			return MaybeSendMultipleObject( inBlock );
		}
		
		inline PvdCommLayerError sendEvent( const SSendPropertyBlock& inBlock )
		{
			return MaybeSendMultipleObject( inBlock );
		}
		
		inline PvdCommLayerError sendEvent( const SEndPropertyBlock& inBlock )
		{
			return EndMultiple( inBlock );
		}
		
		inline PvdCommLayerError sendEvent( const SBeginArrayBlock& inBlock )
		{
			return BeginMultiple( inBlock, PVD::PvdConnectionEventType::ArrayObject );
		}
		
		inline PvdCommLayerError sendEvent( const SBeginArrayPropertyBlock& inBlock )
		{
			return BeginMultiple( inBlock, PVD::PvdConnectionEventType::ArrayObject );
		}
		
		inline PvdCommLayerError sendEvent( const SArrayObject& inBlock )
		{
			return MaybeSendMultipleObject( inBlock );
		}

		inline PvdCommLayerError sendEvent( const SArrayObjects& inBlock )
		{
			return MaybeSendMultipleObject( inBlock );
		}

		inline PvdCommLayerError sendEvent( const SEndArrayBlock& inBlock )
		{
			return EndMultiple( inBlock );
		}
		
		inline PvdCommLayerError sendEvent( const SEndArrayPropertyBlock& inBlock )
		{
			return EndMultiple( inBlock );
		}

		inline PvdCommLayerError sendEvent( const SSetPropertyValue& inValue )
		{
			if ( inValue.mDatatype == PvdCommLayerDatatype::Section )
			{
				SSetPropertyValue theValue( inValue );
				theValue.mValue.mSection.mTimestamp = physx::shdfnd::Time::getCurrentCounterValue();
				return doSendEvent( theValue );
			}
			return doSendEvent( inValue );
		}
		
		template<typename TDataType>
		inline PvdCommLayerError sendEvent( const TDataType& inEvent )
		{
			return doSendEvent( inEvent );
		}

		inline void setDatatypes(const PvdCommLayerDatatype* inDatatypes, PxU32 inPropertyCount )
		{
			mOutStreamHandlerManager.setUserDatatypes( inDatatypes, inPropertyCount );
			mByteCounterStreamHandlerManager.setUserDatatypes( inDatatypes, inPropertyCount );
			mContainerStreamHandlerManager.setUserDatatypes( inDatatypes, inPropertyCount );
		}

		inline void disableCaching() { mCachingEnabled = false; }

	protected:

		template<typename TDataType, typename TFuncStreamType>
		inline void SerializeToStream( TDataType inDataType, TFuncStreamType& inStream, bool inWriteType, PxU8 inEventType )
		{
			if ( inWriteType )
				inStream.streamify( inEventType );
			inDataType.streamify( inStream );
		}

		template<typename TDataType, typename TFuncStreamType>
		inline void SerializeToStream( const TDataType& inDataType, TFuncStreamType& inStream, bool inWriteType )
		{
			SerializeToStream( inDataType, inStream, inWriteType, inDataType.getEventType().mEventType );
		}

		inline PxU32 GetMaxCacheSize() const { if ( mCachingEnabled ) return TMaxCacheByteSize; return 0; }

		template<typename TDataType>
		inline void SerializeToContainer( const TDataType& inDataType, bool inWriteType, PxU8 inEventType)
		{
			if ( mLastError == PvdCommLayerError::None )
			{
				mCounter.mByteCount = 0;
				//Check the size of the data.
				SerializeToStream( inDataType, mCountStream, inWriteType, inEventType );

				PxU32 maxCacheSize( GetMaxCacheSize() );
				//If container.size
				if ( ( mEventContainer.size() + mCounter.mByteCount ) >= TMaxCacheByteSize 
						|| mCounter.mByteCount >= maxCacheSize )
					SendContainer();
				if ( mCounter.mByteCount >= maxCacheSize )
				{
					//Send a single, large event.
					mEventCount = 1;
					ScopedStreamWatcher<TStreamType> watcher( mOutStream );
					SendHeader(mCounter.mByteCount);
					SerializeToStream( inDataType, mOutStreamStream, inWriteType, inEventType );
					mLastError = mStreamWriter.mLastError;
					mEventCount = 0;
				}
				else
				{
					SerializeToStream( inDataType, mContainerStream, inWriteType, inEventType );
					++mEventCount;
				}
			}
		}
		
		template<typename TDataType>
		inline void SerializeToContainer( const TDataType& inDataType, bool inWriteType)
		{
			SerializeToContainer( inDataType, inWriteType, inDataType.getEventType().mEventType );
		}

		inline PvdCommLayerError writeBytes( const PxU8* inBytes, PxU32 inLength )
		{
			if ( inBytes && inLength && mLastError == PvdCommLayerError::None )
				mLastError = mOutStream->write( inBytes, inLength );
			return mLastError;
		}

		inline SEventHeader3 createHeader( PxU32 inPayloadSize )
		{
			SEventHeader3 theHeader = { mEventType, static_cast<PxU64>(reinterpret_cast<size_t>(this)), physx::shdfnd::Time::getCurrentCounterValue(), inPayloadSize + 4 };
			return theHeader;
		}

		inline PvdCommLayerError SendHeader( PxU32 inPayloadSize )
		{
			PX_ASSERT( mEventType == PvdConnectionEventType::EventBatch );
			//The total event size is whatever we have buffered along with
			//4 bytes for the overall event count.
			SEventHeader3 theHeader( createHeader( inPayloadSize ) );
			theHeader.toBytes( mHeaderStorage );
			writeU32( mHeaderStorage + 21, mEventCount );
			writeBytes( mHeaderStorage, 25 );
			return mLastError;
		}

		//We are switching modes or flush
		inline PvdCommLayerError SendContainer()
		{
			if ( mEventCount )
			{
				PxU32 theContainerSize = static_cast<PxU32>( mEventContainer.size() );
				PxU32 thePayloadSize = theContainerSize - 25;
				//write a new header followed by the contain size minus the header to the event container.
				mEventContainer.clear();
				SEventHeader3 theHeader = createHeader( thePayloadSize );
				theHeader.streamify( mContainerStream );
				mContainerStream.streamify( mEventCount );
				{ //hold the mutex protected the stream for as short of a time as possible.
					ScopedStreamWatcher<TStreamType> watcher( mOutStream );
					writeBytes( mEventContainer.begin(), theContainerSize );
				}
				mEventCount = 0;
				mEventContainer.clear();
				writeHeaderToContainer();
			}
			return mLastError;
		}

		inline void writeHeaderToContainer()
		{
			SEventHeader3 theHeader = { PvdConnectionEventType::Unknown, 0, 0, 0 };
			theHeader.streamify( mContainerStream );
			mContainerStream.streamify( (PxU32)0 ); //event count
		}

		template<typename TDataType>
		inline PvdCommLayerError doSendEvent( const TDataType& inEvent, PxU8 inEventType )
		{
			if ( mEventType != PvdConnectionEventType::EventBatch )
			{
				SendContainer();
				mEventType = PvdConnectionEventType::EventBatch;
			}
			SerializeToContainer( inEvent, true, inEventType );
			return mLastError;
		}

		template<typename TDataType>
		inline PvdCommLayerError doSendEvent( const TDataType& inEvent )
		{
			return doSendEvent( inEvent, PVD::getEventType( inEvent ) );
		}

		template<typename TDataType>
		inline PvdCommLayerError BeginMultiple( const TDataType& inDataType
													, PvdConnectionEventType )
		{
			PvdCommLayerError retval = doSendEvent( inDataType );
			
			PxU32 inPropertyCount = inDataType.mPropertyCount;
			const PvdCommLayerDatatype* inDatatypes = inDataType.mDatatypes;

			setDatatypes( inDatatypes, inPropertyCount );
			return retval;
		}
		
		template<typename TDataType>
		inline PvdCommLayerError MaybeSendMultipleObject( const TDataType& inDataType, PvdConnectionEventType inType )
		{
			doSendEvent( inDataType, inType.mEventType );
			return mLastError;
		}

		template<typename TDataType>
		inline PvdCommLayerError MaybeSendMultipleObject( const TDataType& inDataType )
		{
			return MaybeSendMultipleObject( inDataType, inDataType.getEventType() );
		}

		template<typename TDataType>
		inline PvdCommLayerError EndMultiple( const TDataType& inEndMessage )
		{
			return doSendEvent( inEndMessage );
		}
	};
}

#endif
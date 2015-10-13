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

#ifndef PVD_PVDCONNECTIONSTREAMREADER_H
#define PVD_PVDCONNECTIONSTREAMREADER_H

#include "PVDConnectionEvents.h"
#include "PvdDataStreamEventInStream.h"
#include "PsHashMap.h"

namespace PVD
{
	template<typename TReaderType>
	struct PvdConnectionStreamReaderOp
	{
		TReaderType* mReader;
		PvdConnectionStreamReaderOp( TReaderType* inReader )
			: mReader( inReader ) {}
		template<typename TDataType>
		inline PvdConnectionEventData operator()( const TDataType& inEvent ) { return mReader->operator()( inEvent ); }
		inline PvdConnectionEventData operator()() { return PvdConnectionEventData(); }
	};

	/**
	 *	This class is not an exact analogue of the stream writer
	 *	due to the way it is intended to be used.  Currently PVD2 
	 *	reads data from a socket in another thread than the one that
	 *	does the stream parsing.  So the thread that does the stream
	 *	parsing always has just data pointers; it never actually
	 *	reads from the network.  Also, because they have been
	 *	read from the network already this class doesn't need to
	 *	worry about an event that is only partially sent; that would
	 *	be caught and taken care of by the network thread.
	 *	
	 *	This class must be parameterized based on if at least
	 *	if the stream is big or little endian.
	 */
	template<typename TEventInStreamType
			, typename TAllocator=ClientAllocator<char>
			, typename TDeleteOperator=SDeleteOperator>
	struct PvdDataStreamReaderImpl
	{
		
		typedef AbstractInStreamDatatypeHandler<TEventInStreamType> TDatatypeHandler;

		struct StreamState
		{
			//Data needed for the multiple object sets.
			physx::shdfnd::Array<PvdCommLayerDatatype,TAllocator >	mDatatypes;
			physx::shdfnd::Array<TDatatypeHandler*,TAllocator >		mDatatypeHandlers;
			PxU32													mPropertyCount;
			PxU32													mNamespace;
			physx::shdfnd::Array<PxU32,TAllocator>					mNamespaceStack;

			StreamState() 
				: mPropertyCount( 0 ) 
			{
			}
			void pushNamespace() { mNamespaceStack.pushBack( mNamespace ); }
			void popNamespace() { mNamespace = 0; if ( mNamespaceStack.size() ) mNamespace = mNamespaceStack.back(); mNamespaceStack.popBack();  }
			void setNamespace( const char* ns ) { mNamespace = safeStrHash( ns ); }
		};

		typedef physx::shdfnd3::HashMap<PxU64, StreamState, Hash<PxU64>, TAllocator> TStreamStateHash;
		typedef physx::shdfnd3::HashMap<RegisterPropertyStructKey, RegisterPropertyStructEntry<TAllocator>, RegisterPropertyStructKeyHasher, TAllocator > TPropertyStructHash;

		TStreamStateHash					mStreamStateHash;

		//Data needed meta event types
		PvdConnectionEventType				mEventType;
		PxU32								mEventCount;
		PvdConnectionEventType				mEventSubtype;
		PxU32								mEventSizeCheck;
		PxU32								mEventReadSize;

		//Stream used to get the data. 
		TEventInStreamType					mInStream;
		PxU32								mStreamVersion;
		PxU32								mStreamFlags;
		PxU64								mCurrentStreamId;
		PxU64								mCurrentTimestamp;
		bool								mRawPropertyBlocksEnabled;
		TPropertyStructHash					mPropertyStructs;
		const typename TStreamStateHash::Entry*		mLastStreamEntry;


		PvdDataStreamReaderImpl( PxU16 inStreamVersion = SStreamInitialization::sCurrentStreamVersion, PxU16 inStreamFlags = 0 )
			: mEventCount( 0 )
			, mEventSizeCheck( 0 )
			, mEventReadSize( 0 )
			, mEventType( PvdConnectionEventType::Unknown )
			, mEventSubtype( PvdConnectionEventType::Unknown )
			, mStreamVersion( inStreamVersion )
			, mCurrentStreamId( 0 )
			, mCurrentTimestamp( 0 )
			, mRawPropertyBlocksEnabled( false )
			, mLastStreamEntry( NULL )
			, mStreamFlags( inStreamFlags )
		{
			if ( mStreamVersion == 0 && mStreamFlags )
			{
				mStreamVersion = mStreamFlags;
				mStreamFlags = 0;
			}
		}
		
		void setData( const PxU8* inBeginPtr, const PxU8* inEndPtr )
		{
			mInStream.setup( inBeginPtr, inEndPtr, mStreamVersion );
			mEventCount = 0;
			mEventSizeCheck = 0;
			mEventReadSize = 0;
		}

		void enableRawPropertyBlocks() { mRawPropertyBlocksEnabled = true; }

		bool hasMoreData() const { return mInStream.hasMoreData(); }

		void setStreamVersion( PxU16 inVersion) {}
		PxU32 getStreamVersion() const { return mStreamVersion; }
		PxU64 getCurrentStreamId() const { return mCurrentStreamId; }
		PxU32 getCurrentNamespaceId() { return GetStreamState().mNamespace; }
		bool is64BitStream() const { return mStreamFlags & StreamFlags::PointersAre64Bits; }

		PvdConnectionEventData nextEvent()
		{
			if ( mEventCount )
			{
				if ( mEventType == PvdConnectionEventType::MultipleEvent )
					return ReadNextMultipleEvent();
				else if ( mEventType == PvdConnectionEventType::EventBatch )
					return readNextBatchEvent();
			}
			//Ensure the size parameter on the previous header was correct
			PX_ASSERT( mEventSizeCheck == mEventReadSize );
			PvdConnectionEventType theEventType;
			if ( mStreamVersion == 1 )
			{
				SEventHeader theHeader;
				theHeader.streamify( mInStream );
				theEventType = theHeader.mEventType;
				mEventSizeCheck = theHeader.mSize;
			}
			else if ( mStreamVersion < 4 )
			{
				SEventHeader2 theHeader;
				theHeader.streamify( mInStream );
				theEventType = theHeader.mEventType;
				mCurrentStreamId = theHeader.mStreamId;
				mEventSizeCheck = theHeader.mSize;
			}
			else
			{
				SEventHeader3 theHeader;
				theHeader.streamify( mInStream );
				theEventType = theHeader.mEventType;
				mCurrentStreamId = theHeader.mStreamId;
				mCurrentTimestamp = theHeader.mTimestamp;
				mEventSizeCheck = theHeader.mSize;
				applyStreamState();
			}
			mEventReadSize = 0;
			if ( theEventType == PvdConnectionEventType::MultipleEvent )
			{
				mInStream.streamify( mEventCount );
				mInStream.streamify( mEventSubtype.mEventType );
				mEventReadSize += 5;
				mEventType = PvdConnectionEventType::MultipleEvent;
				return nextEvent();
			}
			else if ( theEventType == PvdConnectionEventType::EventBatch )
			{
				mInStream.streamify( mEventCount );
				mEventReadSize += 4;
				mEventType = PvdConnectionEventType::EventBatch;
				return nextEvent();
			}
			else //Who knows what happened, but we got an invalid event.
				return PvdConnectionEventData();
		}
		void destroy() { TDeleteOperator()(this); }
		

		inline bool hasFailed() { return mInStream.mFail; }

		////////////////////////////////////////////////////////////
		// Implementation.  Public because it is called from an 
		// parameterized operator().
		////////////////////////////////////////////////////////////


		inline PvdConnectionEventData deserializeNextEvent( PvdConnectionEventType inType )
		{
			PxU32 originalSize = mInStream.amountLeft();
			PvdConnectionEventData retval( EventTypeBasedOperation<PvdConnectionEventData>( inType, PvdConnectionStreamReaderOp<PvdDataStreamReaderImpl<TEventInStreamType> >( this )) );
			PX_ASSERT( mInStream.mFail == false );
			PxU32 finalSize = mInStream.amountLeft();
			mEventReadSize += originalSize - finalSize;
			return retval;
		}

		//Implementation
		inline PvdConnectionEventData ReadNextMultipleEvent()
		{
			PvdConnectionEventData retval;
			if ( mEventCount )
			{
				--mEventCount;
				return deserializeNextEvent( mEventSubtype );
			}
			return retval;
		}

		inline PvdConnectionEventData readNextBatchEvent()
		{
			PvdConnectionEventData retval;
			if ( mEventCount )
			{
				--mEventCount;
				PvdConnectionEventType theEventType;
				mInStream.streamify( theEventType.mEventType );
				mEventReadSize += 1;
				retval = deserializeNextEvent( theEventType );
			}
			return retval;
		}

		template<typename TDataType>
		inline TDataType deserialize()
		{
			TDataType theTemp;
			theTemp.streamify( mInStream );
			return theTemp;
		}
		StreamState& GetStreamState()
		{
			if ( mLastStreamEntry != NULL && mLastStreamEntry->first == mCurrentStreamId )
				return const_cast<StreamState&>(mLastStreamEntry->second);

			mLastStreamEntry = mStreamStateHash.find( mCurrentStreamId );
			if ( mLastStreamEntry == NULL )
			{
				mStreamStateHash.insert( mCurrentStreamId, StreamState() );
				mLastStreamEntry = mStreamStateHash.find( mCurrentStreamId );
				PX_ASSERT( mLastStreamEntry );
			}
			return const_cast<StreamState&>( mLastStreamEntry->second );
		}
		template<typename TDataType>
		inline void beginBlock( const TDataType& inEvent)
		{
			StreamState& theState( GetStreamState() );
			theState.mDatatypes.clear();
			theState.mDatatypeHandlers.clear();
			theState.mPropertyCount = inEvent.mPropertyCount;
			mInStream.setDatatypeHandlers( NULL );
			for ( PxU32 idx =0; idx < inEvent.mPropertyCount; ++idx )
			{
				theState.mDatatypes.pushBack( inEvent.mDatatypes[idx] );
				theState.mDatatypeHandlers.pushBack( mInStream.findHandlerForType( inEvent.mDatatypes[idx] ) );
			}
			applyStreamState();
		}
		
		void applyStreamState()
		{
			StreamState& theState( GetStreamState() );
			if ( theState.mPropertyCount )
				mInStream.setDatatypeHandlers( theState.mDatatypeHandlers.begin() );
		}


		//The blocks require special handling
		inline PvdConnectionEventData operator()( const SBeginPropertyBlock& )
		{
			SBeginPropertyBlock theBlock( deserialize<SBeginPropertyBlock>() );
			beginBlock( theBlock );
			return theBlock;
		}
		inline PvdConnectionEventData operator()( const SSendPropertyBlock& )
		{
			StreamState& theState( GetStreamState() );
			if ( mRawPropertyBlocksEnabled )
			{
				SRawPropertyBlock theBlock = createRawPropertyBlock( theState.mDatatypes.begin(), theState.mPropertyCount, 0, NULL, 0, NULL );
				theBlock.inPlaceRead( mInStream );
				PvdConnectionEventDataUnion theUnion;
				theUnion.mRawPropertyBlock = theBlock;
				return PvdConnectionEventData( PvdConnectionEventType::RawPropertyBlock, theUnion );
			}
			else
			{
				SSendPropertyBlock theBlock = createSendPropertyBlock( theState.mDatatypes.begin(), theState.mPropertyCount );
				theBlock.streamify( mInStream );
				return theBlock;
			}
		}
		inline PvdConnectionEventData operator()( const SEndPropertyBlock& )
		{
			mInStream.setDatatypeHandlers( NULL );
			return createEndPropertyBlock();
		}
		inline PvdConnectionEventData operator()( const SBeginArrayBlock& )
		{
			SBeginArrayBlock theBlock( deserialize<SBeginArrayBlock>() );
			beginBlock( theBlock);
			return theBlock;
		}
		
		inline PvdConnectionEventData operator()( const SBeginArrayPropertyBlock& )
		{
			SBeginArrayPropertyBlock theBlock( deserialize<SBeginArrayPropertyBlock>() );
			beginBlock( theBlock );
			return theBlock;
		}

		inline PvdConnectionEventData operator()( const SArrayObject&)
		{
			StreamState& theState( GetStreamState() );
			SArrayObject theObject = createArrayObject( theState.mDatatypes.begin(), theState.mPropertyCount );
			theObject.streamify( mInStream );
			return theObject;
		}
		
		inline PvdConnectionEventData operator()( const SArrayObjects&)
		{
			StreamState& theState( GetStreamState() );
			SArrayObjects theObject = createArrayObjects( theState.mDatatypes.begin(), theState.mPropertyCount );
			theObject.streamify( mInStream );
			return theObject;
		}

		inline PvdConnectionEventData operator()( const SEndArrayBlock& )
		{
			mInStream.setDatatypeHandlers( NULL );
			return createEndArrayBlock();
		}
		
		inline PvdConnectionEventData operator()( const SEndArrayPropertyBlock& )
		{
			mInStream.setDatatypeHandlers( NULL );
			return createEndArrayPropertyBlock();
		}
		
		inline PvdConnectionEventData operator()( const PushNamespace& )
		{
			StreamState& theState( GetStreamState() );
			PushNamespace theStruct( deserialize<PushNamespace>() );
			theState.pushNamespace();
			return theStruct;
		}

		inline PvdConnectionEventData operator()( const SetNamespace& )
		{
			StreamState& theState( GetStreamState() );
			SetNamespace theStruct( deserialize<SetNamespace>() );
			theState.setNamespace( theStruct.mNamespace );
			return theStruct;
		}
		
		inline PvdConnectionEventData operator()( const PopNamespace& )
		{
			StreamState& theState( GetStreamState() );
			PopNamespace theStruct( deserialize<PopNamespace>() );
			theState.popNamespace();
			return theStruct;
		}

		inline PvdConnectionEventData operator()( const RegisterPropertyStruct& )
		{
			StreamState& theState( GetStreamState() );
			RegisterPropertyStruct theStruct( deserialize<RegisterPropertyStruct>() );
			RegisterPropertyStructKey theKey( theStruct.mKey, theState.mNamespace );
			mPropertyStructs.insert( theKey, RegisterPropertyStructEntry<TAllocator>() );
			RegisterPropertyStructEntry<TAllocator>& theEntry( mPropertyStructs[theKey] );
			theEntry.setup( theStruct.mClass, theStruct.mStructByteSize, theStruct.mEntries, theStruct.mPropertyCount );
			return theStruct;
		}

		inline PvdConnectionEventData operator()( const SendPropertyStruct& )
		{
			StreamState& theState( GetStreamState() );
			SendPropertyStruct theStruct;
			theStruct.streamifyStart( mInStream );
			//Now fill in the members we should know from the struct id.
			RegisterPropertyStructKey theKey( theStruct.mStructId, theState.mNamespace );
			const typename TPropertyStructHash::Entry* entry = mPropertyStructs.find( theKey );
			PX_ASSERT( entry );
			if ( entry != NULL )
			{
				RegisterPropertyStructEntry<TAllocator>& theEntry( const_cast<RegisterPropertyStructEntry<TAllocator>& >( entry->second ) );
				theStruct.mClass = theEntry.mClass;
				theStruct.mDataLen = theEntry.mByteSize;
				theStruct.mEntries = theEntry.mEntries.begin();
				theStruct.mEntryCount = theEntry.mEntries.size();
				theStruct.mStrings = theEntry.mStringOffsets.begin();
				theStruct.mStringCount = theEntry.mStringOffsets.size();
				theStruct.readEnd( mInStream, is64BitStream() );
				return theStruct;
			}
			//we failed to parse this message.
			return PvdConnectionEventData();
		}
		
		//Most events can serialize/deserialize trivially from the connection.
		template<typename TDataType>
		inline PvdConnectionEventData operator()( const TDataType& )
		{
			return deserialize<TDataType>();
		}
	};
}

#endif
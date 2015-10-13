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

#ifndef PVD_PVDCONNECTIONEVENTINSTREAM_H
#define PVD_PVDCONNECTIONEVENTINSTREAM_H

#include "PxProfileBase.h"
#include "PvdDataStreamEventStream.h"
#include "ClientAllocator.h"
#include "PsArray.h"
#include "BlockParserFunctions.h"

namespace PVD
{

	template<typename TContainer, typename TIterator>
	inline void append( TContainer& ioArray, TIterator inStart, TIterator inEnd )
	{
		for ( ; inStart != inEnd; ++inStart )
			ioArray.pushBack(*inStart);
	}

	template<typename TStreamType>
	struct commLayerInputStreamStore
	{
		TStreamType* mStream;
		commLayerInputStreamStore( TStreamType* inStream ) : mStream( inStream ) {}

		inline PvdCommLayerData operator()( FloatBuffer inData ) { return createCommLayerData( mStream->store(inData) ); }
		inline PvdCommLayerData operator()( U32Buffer inData ) { return createCommLayerData( mStream->store(inData) ); }
		inline PvdCommLayerData operator()( String inData ) { return createCommLayerData( mStream->store(inData) ); }

		template<typename TDatatype>
		inline PvdCommLayerData operator()( TDatatype inData ) { return createCommLayerData( inData ); }
		inline PvdCommLayerData operator()() { PvdCommLayerData retval; memset( &retval, 0, sizeof( retval ) ); return retval; }
	};

	template<typename TStreamType>
	struct commLayerInputStreamFixup
	{
		TStreamType* mStream;
		commLayerInputStreamFixup( TStreamType* inStream ) : mStream( inStream ) {}

		inline PvdCommLayerData operator()( FloatBuffer inData ) { return createCommLayerData( mStream->fixup(inData) ); }
		inline PvdCommLayerData operator()( U32Buffer inData ) { return createCommLayerData( mStream->fixup(inData) ); }
		inline PvdCommLayerData operator()( String inData ) { return createCommLayerData( mStream->fixup(inData) ); }

		template<typename TDatatype>
		inline PvdCommLayerData operator()( TDatatype inData ) { return createCommLayerData( inData ); }
		inline PvdCommLayerData operator()() { PvdCommLayerData retval; memset( &retval, 0, sizeof( retval ) ); return retval; }
	};

	template<typename TDataType>
	inline bool requiresFixup( const TDataType& ) 
	{
		return false;
	}
	
	template<> inline bool requiresFixup( const FloatBuffer& ) { return true; }
	template<> inline bool requiresFixup( const U32Buffer& ) { return true; }
	template<> inline bool requiresFixup( const String& ) { return true; }

	//Use interface for at least some erasure.
	template<typename TStreamType>
	class AbstractInStreamDatatypeHandler
	{
	protected:
		TStreamType* mStream;
		commLayerInputStreamStore<TStreamType> mStoreOp;
		commLayerInputStreamFixup<TStreamType> mFixupOp;

	public:
		AbstractInStreamDatatypeHandler( TStreamType* inStream ) 
			: mStream( inStream )
			, mStoreOp( inStream )
			, mFixupOp( inStream )
		{}
		virtual ~AbstractInStreamDatatypeHandler(){}
		virtual PvdCommLayerDatatype datatype() const = 0;
		virtual PvdCommLayerData streamifyAndStore() = 0;
		virtual PvdCommLayerData fixup(PvdCommLayerData& inData) = 0;
		virtual bool requiresFixup() const = 0;
		virtual PxU8* streamify( PxU8* outMemory ) = 0;
		virtual void bulkStreamify( PxU8* outMemory, PxU32 inCount ) = 0;
		virtual PxU32 byteSize() const = 0;
	};
	
	//!!!!These objects must not be any larger than their base class!!!
	template<typename TStreamType, typename TDataType>
	class InStreamDatatypeHandler : public AbstractInStreamDatatypeHandler<TStreamType>
	{
	public:
		InStreamDatatypeHandler( TStreamType* inStream ) : AbstractInStreamDatatypeHandler<TStreamType>( inStream ) {}
		virtual PvdCommLayerDatatype datatype() const { return getDatatypeForValue( TDataType() ); }
		virtual PvdCommLayerData streamifyAndStore() 
		{ 
			TDataType theValue = TDataType(); 
			AbstractInStreamDatatypeHandler<TStreamType>::mStream->streamify( theValue ); 
			return mStoreOp( theValue );
		}

		virtual PvdCommLayerData fixup(PvdCommLayerData& inData)
		{
			return mFixupOp( getCommLayerData<TDataType>(inData) );
		}
		virtual bool requiresFixup() const { return PVD::requiresFixup( TDataType() ); }
		virtual PxU8* streamify( PxU8* outMemory )
		{
			TDataType theValue; 
			AbstractInStreamDatatypeHandler<TStreamType>::mStream->streamify( theValue ); 
			memcpy( outMemory, &theValue, sizeof( TDataType ) );
			return outMemory + sizeof( TDataType );
		}
		virtual void bulkStreamify( PxU8* outMemory, PxU32 inCount )
		{
			TDataType* theTypePtr = reinterpret_cast< TDataType* >( outMemory );
			AbstractInStreamDatatypeHandler<TStreamType>::mStream->readPtrBlock( theTypePtr, inCount );
		}
		virtual PxU32 byteSize() const { return sizeof( TDataType ); }
	};
		

	template<typename TStreamType
			, typename TAllocator>
	struct AbstractInStreamDatatypeHandlerCreator
	{
		TStreamType* mStream;
		AbstractInStreamDatatypeHandlerCreator( TStreamType* inStream ) : mStream( inStream ) {}
		template<typename TDataType>
		inline AbstractInStreamDatatypeHandler<TStreamType>* operator()( const TDataType& )
		{
			typedef InStreamDatatypeHandler<TStreamType, TDataType> THandlerType;
			THandlerType* theHandler = (THandlerType*)TAllocator().allocate( sizeof( THandlerType ), __FILE__, __LINE__ );
			new(theHandler) THandlerType( mStream );
			return theHandler;
		}
		
		inline AbstractInStreamDatatypeHandler<TStreamType>* operator()()
		{
			return NULL;
		}
	};

	/**
	 *	Read data from a byte array into actual events.
	 *	The events don't story any buffers information so 
	 *	this object needs to have several buffers for temporary
	 *	storage.  This means that you can only process one 
	 *	event at a time as reading the next event may destroy
	 *	some data pointed to be the previous event.
	 *
	 *	There is an algorithm implemented in two different places for
	 *	the case where we need to read many buffers for one return value.
	 *	We copy the data into secondary storage and then assign the pointer
	 *	the index into the secondary storage.  Then after the entire section
	 *	is read and before we return we re-assign the pointer to point into
	 *	parts of the secondary storage buffer.
	 *
	 *	Also note this is a not code that gets compiled into any game engines,
	 *	as it reads the events on into the pvd debugging engine.
	 */
	template<bool DoSwapBytes = false
			, typename TAllocator=ClientAllocator<char> >
	class PvdDataStreamEventInStream //The safest version, does not modify source data.
	{
	private:
		PvdDataStreamEventInStream( const PvdDataStreamEventInStream& inOther );
		PvdDataStreamEventInStream& operator=( const PvdDataStreamEventInStream& inOther );
	public:
		typedef AbstractInStreamDatatypeHandler<PvdDataStreamEventInStream<DoSwapBytes, TAllocator> > TDatatypeHandler;
		BlockParser<DoSwapBytes> mBlockParser;

		physx::shdfnd::Array<PxU8,TAllocator> mTempContainer;
		//For concatenating the strings together
		//when we have bitflags.
		physx::shdfnd::Array<PxU8,TAllocator> mSecondaryContainer;
		physx::shdfnd::Array<PxU32,TAllocator> mPropertyContainer;
		physx::shdfnd::Array<NamedValueDefinition,TAllocator> mBitflagContainer;
		physx::shdfnd::Array<PvdCommLayerData,TAllocator> mValueContainer;
		physx::shdfnd::Array<PvdCommLayerDatatype,TAllocator> mDatatypeContainer;
		physx::shdfnd::Array<TDatatypeHandler*,TAllocator> mDatatypeHandlers;
		TDatatypeHandler**									mUserDatatypeHandlers;
		PxU32												mStreamVersion;
		bool												mFail;

		template<typename TContainerType>
		PvdDataStreamEventInStream( const TContainerType& inContainer, PxU32 inStreamVersion )
			: mUserDatatypeHandlers( NULL )
			, mStreamVersion( inStreamVersion )
			, mFail( false )
		{
			if ( inContainer.size() )
			{
				const PxU8* theBegin( &inContainer[0] );
				
				mBlockParser = BlockParser<DoSwapBytes>( theBegin, theBegin + inContainer.size() );
			}
			initializeDatatypeHandlers();
		}

		PvdDataStreamEventInStream( const PxU8* inBegin = NULL, const PxU8* inEnd = NULL, PxU32 inStreamVersion = 0 )
			: mUserDatatypeHandlers( NULL )
			, mStreamVersion( inStreamVersion )
			, mFail( false )
		{
			mBlockParser = BlockParser<DoSwapBytes>( inBegin, inEnd );
			initializeDatatypeHandlers();
		}

		~PvdDataStreamEventInStream()
		{
			for ( PxU32 idx = 0; idx < mDatatypeHandlers.size(); ++idx )
				TAllocator().deallocate( reinterpret_cast<char*>(mDatatypeHandlers[idx]), sizeof(TAllocator) );
			mDatatypeHandlers.clear();
		}

		inline void initializeDatatypeHandlers()
		{
			typedef AbstractInStreamDatatypeHandlerCreator<PvdDataStreamEventInStream<DoSwapBytes, TAllocator>, TAllocator > TCreatorType;
			for (PxU8 idx = 1; idx < PvdCommLayerDatatype::Last; ++idx )
			{
				TDatatypeHandler* theHandler = commLayerDataOperate<TDatatypeHandler*>( PvdCommLayerDatatype( idx ), TCreatorType( this ) );
				mDatatypeHandlers.pushBack( theHandler );
			}
		}

		TDatatypeHandler* findHandlerForType( PvdCommLayerDatatype inType )
		{
			return mDatatypeHandlers[inType.mDatatype - 1];
		}

		TDatatypeHandler* findHandlerForType( PvdCommLayerDatatype inType, PxU32 inIdx )
		{
			if ( mUserDatatypeHandlers != NULL )
				return mUserDatatypeHandlers[inIdx];
			return findHandlerForType( inType );
		}

		void setDatatypeHandlers( TDatatypeHandler** inHandlers )
		{
			mUserDatatypeHandlers = inHandlers;
		}

		inline PxU32 amountLeft() const { return mBlockParser.amountLeft(); }

		const PxU8* getReadPtr() const { return mBlockParser.mBegin; }

		inline void setup( const PxU8* inBegin, const PxU8* inEnd, PxU32 inStreamVersion )
		{
			mBlockParser = BlockParser<DoSwapBytes>( inBegin, inEnd );
			mStreamVersion = inStreamVersion;
			mFail = false;
		}

		inline bool hasMoreData() const { return mBlockParser.mBegin != mBlockParser.mEnd; }
		inline bool hasFailed() const { return mFail; }
		inline bool isValid() const { return hasMoreData() && !hasFailed(); }

		inline SEventHeader readEventHeader()
		{
			SEventHeader retval;
			retval.streamify( *this );
			return retval;
		}

		inline bool checkLength( PxU32 inBytes )
		{
			return mBlockParser.checkLength( inBytes );
		}

		inline void updateFail( bool inReadSuccess )
		{
			mFail |= !inReadSuccess;
		}

		template<typename TDataType>
		inline void simpleStreamify( TDataType& outData )
		{
			if ( mFail == false )
				updateFail( mBlockParser.read(outData) );
		}
		
		inline void streamify( PxU8& outData ) { simpleStreamify( outData ); }
		inline void streamify( PxU16& outData ) { simpleStreamify( outData ); }
		inline void streamify( PxU32& outData ) { simpleStreamify( outData ); }
		inline void streamify( PxU64& outData ) { simpleStreamify( outData ); }
		inline void streamify( PxI8& outData ) { simpleStreamify( outData ); }
		inline void streamify( PxI16& outData ) { simpleStreamify( outData ); }
		inline void streamify( PxI32& outData ) { simpleStreamify( outData ); }
		inline void streamify( PxI64& outData ) { simpleStreamify( outData ); }
		inline void streamify( PxF32& outData ) { simpleStreamify( outData ); }
		inline void streamify( PxF64& outData ) { simpleStreamify( outData ); }

		//Writing never does endian conversions so this is fine.
		template<typename TDataType>
		inline void readBlock( TDataType* outData, PxU32 inLength )
		{
			if ( mFail == false )
				updateFail( mBlockParser.readBlock( outData, inLength ) );
		}

		inline void readBlock( const char* outData, PxU32 inLength )
		{
			PxU8* thePtr = reinterpret_cast< PxU8*>( outData );
			readBlock( thePtr, inLength );
		}

		template<typename TDataType>
		inline void readPtrBlock( TDataType* outData, PxU32 inLength )
		{
			for ( PxU32 idx = 0; idx < inLength; ++idx )
				streamify( outData[idx] );
		}

		template<typename TDataType>
		inline void readPtrBlock( const TDataType* outData, PxU32 inLength )
		{
			readPtrBlock( const_cast< TDataType* >( outData ), inLength );
		}
		
		inline void readPtrBlock( Float3* outData, PxU32 inLength ) { readBlock( reinterpret_cast< float* >( outData ), inLength * 3 );	}
		inline void readPtrBlock( float* outData, PxU32 inLength ) { readBlock( outData, inLength ); }
		inline void readPtrBlock( PxU32* outData, PxU32 inLength ) { readBlock( outData, inLength ); }
		inline void readPtrBlock( PxU16* outData, PxU32 inLength ) { readBlock( outData, inLength ); }
		inline void readPtrBlock( PxU8* outData, PxU32 inLength ) {	readBlock( outData, inLength );	}

		template<typename TDataType>
		inline void readTempBlock( const TDataType*& outData, PxU32 inLength )
		{
			PxU32 theSize = inLength * sizeof( TDataType );
			mTempContainer.resize( theSize );
			if ( inLength )
			{
				readBlock( reinterpret_cast< TDataType* >( mTempContainer.begin() ), inLength );
				outData = reinterpret_cast< const TDataType* >( mTempContainer.begin() );
			}
			else
				outData = NULL;
		}
		
		inline void streamify( bool& outData )
		{
			PxU8 theData = blockParserRead<PxU8>( mBlockParser );
			outData = theData ? true : false;
		}

		template<typename TDataType>
		inline void bulkStreamify( TDataType*& outData, PxU32 inDataLen )
		{
			if ( inDataLen )
			{
				outData = reinterpret_cast< TDataType* >( const_cast< PxU8* >( mBlockParser.mBegin ) );
				readPtrBlock( outData, inDataLen );
			}
			else
				outData = NULL;
		}
		
		void streamify( const char*& outString )
		{
			PxU32 inLen = 0;
			outString = NULL;
			streamify( inLen );
			//Stream versions less than 2 didn't null-terminate
			//their strings, thus forcing a copy operation.
			if ( mStreamVersion < 3 )
			{
				mTempContainer.resize(inLen);
				if ( inLen )
				{
					readBlock( mTempContainer.begin(), inLen );
					mTempContainer.pushBack( 0 );
					outString = reinterpret_cast<const char*>(mTempContainer.begin());
				}
			}
			else
			{
				if ( inLen )
				{
					outString = reinterpret_cast< const char* >( mBlockParser.mBegin );
					
					// this unuual syntax is necessary for SNC because mBlockParser 
					// is itself a template, see http://www.comeaucomputing.com/techtalk/templates/
					updateFail( mBlockParser.template readBlock<char>( inLen ) );
				}
			}
		}

		inline void inPlaceStreamify( PvdCommLayerDatatype inDatatype, PxU32 inIdx )
		{
			PxU8* theDataPtr = const_cast<PxU8*>( getReadPtr() );
			if ( inDatatype.mDatatype != PvdCommLayerDatatype::String )
			{
				TDatatypeHandler* handler = findHandlerForType( inDatatype, inIdx );
				handler->streamify( theDataPtr );
			}
			else
			{
				PxU32 leftOver = amountLeft();
				PxU32 theValue = 0;
				if ( leftOver >= 4 )
				{
					streamify( theValue );
					theValue = PxMin( theValue, amountLeft() );
					memcpy( theDataPtr, &theValue, sizeof( theValue ) );
				}
				mBlockParser.mBegin += theValue;
			}
		}

		inline void streamify( PvdCommLayerDatatype& outDatatype )
		{
			streamify( outDatatype.mDatatype );
		}
		
		template<typename TDataType>
		inline const TDataType* storePtr( const TDataType* inPtr )
		{
			return inPtr;
		}

		inline const char* storePtr( const char* inPtr )
		{
			if ( mStreamVersion < 3 )
			{
				if ( mTempContainer.size() )
				{
					inPtr = reinterpret_cast< const char* >( mSecondaryContainer.size() + 1 );
					append( mSecondaryContainer, mTempContainer.begin(), mTempContainer.end() );
				}
				else
					inPtr = NULL;
			}
			return inPtr;
		}

		template<typename TDataType>
		inline const TDataType* fixupPtr( const TDataType* inPtr )
		{
			return inPtr;
		}

		inline const char* fixupPtr( const char* inPtr )
		{
			if ( mStreamVersion < 3 )
			{
				if ( inPtr )
				{
					//In this case, the pointer really has just an index in it stored during storePtr.
					inPtr = reinterpret_cast< const char* >( &mSecondaryContainer[static_cast<PxU32>(PX_PROFILE_POINTER_TO_U64(inPtr) - 1)] );
				}
			}
			return inPtr;
		}

		void streamify( PxU32& inPropertyName, const NamedValueDefinition*& outDefinitions, PxU32& outDefinitionLength )
		{
			PxU32 theLength = 0;
			inPropertyName = NULL;
			mBitflagContainer.clear();
			mSecondaryContainer.clear();
			streamify( inPropertyName );
			streamify( theLength );
			outDefinitions = NULL;
			for ( PxU32 idx = 0; idx < theLength; ++idx )
			{
				NamedValueDefinition theDefinition;
				streamify( theDefinition.mName );
				streamify( theDefinition.mValue );
				theDefinition.mName = storePtr( theDefinition.mName );
				mBitflagContainer.pushBack( theDefinition );
			}
			if ( mStreamVersion < 3 )
			{
				for ( PxU32 idx = 0; idx < theLength; ++idx )
				{
					NamedValueDefinition& theDefinition = mBitflagContainer[idx];
					theDefinition.mName = fixupPtr( theDefinition.mName );
				}
			}
			outDefinitionLength = theLength;
			if ( outDefinitionLength )
				outDefinitions = mBitflagContainer.begin();
		}
		inline void readRaw( PxU8 inDatatype, PvdCommLayerData& outData )
		{
			outData = commLayerDataOperate<PvdCommLayerData>( PvdCommLayerDatatype( inDatatype ), CommLayerStreamOperator<PvdDataStreamEventInStream>( this ) );
		}

		inline void streamify( PxU8& outDatatype, PvdCommLayerData& outData )
		{
			readRaw( outDatatype, outData );
		}
		
		inline void readRaw( PxU8 inDatatype, PvdCommLayerMediumData& outData )
		{
			outData = commLayerMediumDataOperate<PvdCommLayerMediumData>( PvdCommLayerDatatype( inDatatype ), CommLayerMediumStreamOperator<PvdDataStreamEventInStream>( this ) );
		}

		inline void streamify( PxU8& outDatatype, PvdCommLayerMediumData& outData )
		{
			readRaw( outDatatype, outData );
		}
		
		inline void readRaw( PxU8 inDatatype, PvdCommLayerSmallData& outData )
		{
			outData = commLayerSmallDataOperate<PvdCommLayerSmallData>( PvdCommLayerDatatype( inDatatype ), CommLayerSmallStreamOperator<PvdDataStreamEventInStream>( this ) );
		}

		inline void streamify( PxU8& outDatatype, PvdCommLayerSmallData& outData )
		{
			readRaw( outDatatype, outData );
		}

		template<PxU32 TNumFloats>
		inline void streamify( TFixedFloatArray<TNumFloats>& outData )
		{
			readBlock( outData.mFloats, TNumFloats );
		}
		
		template<PxU32 TNumData>
		inline void streamify( TFixedU32Array<TNumData>& outData )
		{
			readBlock( outData.mData, TNumData );
		}

		template<typename TDataType>
		inline void streamify( GenericDatatype<TDataType>& outData )
		{
			streamify( outData.mValue );
		}

		inline void streamifySectionTimestamp( PxU64& outData )
		{
			if ( mStreamVersion < 6 )
			{
				PxU32 payload;
				streamify( payload );
				outData = payload;
			}
			else
				streamify( outData );
		}

		template<typename TDataType>
		inline void streamify( Buffer<TDataType>& outData )
		{
			streamify( outData.mLength );
			outData.mData = reinterpret_cast< TDataType* >( const_cast<PxU8*>(mBlockParser.mBegin) );
			readPtrBlock( outData.mData, outData.mLength );
		}
		
		inline void streamify( HeightFieldSample& inData )
		{
			PxU16* theData = reinterpret_cast< PxU16* >( &inData );
			streamify( theData[0] );
			PxU8* theSmallData = reinterpret_cast< PxU8* >( &theData[1] );
			streamify( theSmallData[0] );
			streamify( theSmallData[1] );
		}

		inline void streamify( Section& inData )
		{
			streamify( inData.mType );
			streamify( inData.mTimestamp );
		}

		void streamify( const PxU32*& outProperties, PxU32& outNumProps )
		{
			outProperties = NULL;
			streamify( outNumProps );
			if ( outNumProps )
			{
				outProperties = reinterpret_cast< PxU32* >( const_cast< PxU8* >( mBlockParser.mBegin ) );
				readPtrBlock( outProperties, outNumProps );
			}
		}
		
		inline void streamify( const PvdCommLayerDatatype*& outDatatypes, PxU32& outNumProps )
		{
			outDatatypes = NULL;
			streamify( outNumProps );
			if ( outNumProps )
			{
				outDatatypes = reinterpret_cast< PvdCommLayerDatatype* >( const_cast< PxU8* >( mBlockParser.mBegin ) );
				readPtrBlock( outDatatypes, outNumProps );
			}
		}
		
		void streamify( const PvdCommLayerDatatype* inDatatypes, PxU32 inNumProps, const PvdCommLayerData*& outValues )
		{
			outValues = NULL;
			PxU32 theTemp = inNumProps;	
			if ( theTemp )
			{
				mValueContainer.clear();
				mValueContainer.reserve(theTemp);
				mSecondaryContainer.clear();
				bool requiresFixup = false;
				for ( PxU32 idx =0; idx < theTemp; ++idx )
				{
					TDatatypeHandler* theHandler = findHandlerForType( inDatatypes[idx], idx );
					requiresFixup |= theHandler->requiresFixup();
					mValueContainer.pushBack( theHandler->streamifyAndStore() );
				}
				if ( mStreamVersion < 3 )
				{
					if ( requiresFixup )
					{
						for ( PxU32 idx =0; idx < theTemp; ++idx )
						{
							TDatatypeHandler* theHandler = findHandlerForType( inDatatypes[idx], idx );
							PvdCommLayerData& theValue = mValueContainer[idx];
							PvdCommLayerData theData = theValue;
							theData = theHandler->fixup( theData );
							theValue = theData;
						}
					}
				}
			}
			if ( inNumProps )
				outValues = mValueContainer.begin();
		}

		inline FloatBuffer store( FloatBuffer inData )
		{
			inData.mData = storePtr( inData.mData );
			return inData;
		}
		
		inline U32Buffer store( U32Buffer inData )
		{
			inData.mData = storePtr( inData.mData );
			return inData;
		}
		
		inline String store( String inData )
		{
			inData.mValue = storePtr( inData.mValue );
			return inData;
		}

		inline FloatBuffer fixup( FloatBuffer inData )
		{
			inData.mData = fixupPtr( inData.mData );
			return inData;
		}
		
		inline U32Buffer fixup( U32Buffer inData )
		{
			inData.mData = fixupPtr( inData.mData );
			return inData;
		}
		
		inline String fixup( String inData )
		{
			inData.mValue = fixupPtr( inData.mValue );
			return inData;
		}
		
		inline void streamify( const char*& inStr1, const char*& inStr2 )
		{
			streamify( inStr1 );
			inStr1 = storePtr( inStr1 );
			streamify( inStr2 );
			inStr2 = storePtr( inStr2 );
			inStr1 = fixupPtr( inStr1 );
			inStr2 = fixupPtr( inStr2 );
		}
		
		inline void streamify( const char*& inStr1, const char*& inStr2, const char*& inStr3 )
		{
			streamify( inStr1 );
			inStr1 = storePtr( inStr1 );
			streamify( inStr2 );
			inStr2 = storePtr( inStr2 );
			streamify( inStr3 );
			inStr3 = storePtr( inStr3 );

			inStr1 = fixupPtr( inStr1 );
			inStr2 = fixupPtr( inStr2 );
			inStr3 = fixupPtr( inStr3 );
		}
		
		inline void streamify( PxU32& outNumBlocks
									, const PvdCommLayerDatatype* inDatatypes
									, PxU32 inNumProps
									, const PvdCommLayerData*& outData )
		{
			PxU32 theNumBlocks = 0;
			outData = NULL;

			streamify( theNumBlocks );

			for ( PxU32 block =0; block < theNumBlocks; ++block )
			{
				for ( PxU32 idx = 0; idx < inNumProps; ++idx )
				{
					PvdCommLayerData theData;
					readRaw( inDatatypes[idx].mDatatype, theData );
					PvdCommLayerData tempValue = commLayerDataOperate<PvdCommLayerData>( inDatatypes[idx], theData, commLayerInputStreamStore<PvdDataStreamEventInStream>( this ) );
					mValueContainer.pushBack( tempValue );
				}
			}
			if ( mStreamVersion < 3 ) //Copy from secondary storage back to primary storage.
			{
				for ( PxU32 block =0; block < theNumBlocks; ++block )
				{
					for ( PxU32 idx = 0; idx < inNumProps; ++idx )
					{
						PxU32 totalIndex = block*inNumProps + idx;
						PvdCommLayerData& theValue = mValueContainer[totalIndex];
						theValue = commLayerDataOperate<PvdCommLayerData>( inDatatypes[idx], theValue, commLayerInputStreamFixup<PvdDataStreamEventInStream>( this ) );
					}
				}
			}
			outNumBlocks = theNumBlocks;
			if ( theNumBlocks && inNumProps )
				outData = mValueContainer.begin();
		}
		
		inline void streamify( const PvdCommLayerDatatype* inDatatypes
												, PxU32 inPropertyCount
												, PxU32& outStride
												, PxU32& outValueCount
												, const PxU8*& outValues )
		{
			outStride = 0;
			outValues = NULL;
			streamify( outValueCount );

			//Calculate the byte size of the data
			PxU32 itemSize = 0;
			for ( PxU32 idx = 0; idx < inPropertyCount; ++idx )
				itemSize += findHandlerForType( inDatatypes[idx], idx )->byteSize();

			if ( outValueCount )
			{
				PxU8* theWriter = const_cast< PxU8* >( mBlockParser.mBegin );
				outValues = theWriter;
				if ( inPropertyCount == 1 )
				{
					findHandlerForType( inDatatypes[0], 0 )->bulkStreamify( theWriter, outValueCount );
				}
				else
				{
					for ( PxU32 value = 0; value < outValueCount; ++value )
					{
						for ( PxU32 theProp = 0; theProp < inPropertyCount; ++theProp )
							theWriter = findHandlerForType( inDatatypes[theProp], theProp )->streamify( theWriter );
					}
				}
			}
		}

		inline void streamify( PxU8& outTransformType, RenderTransformData& outData )
		{
			streamify( outTransformType );
			outData = visitRenderTransform<RenderTransformData>( outTransformType, RenderTransformStreamOperator<PvdDataStreamEventInStream>( this ) );
		}

		inline void streamify( const IdentityTransform& )
		{
		}

		inline void streamify( PxU8& outTransformType, RenderTransformData& outTransformData, PxU8& outPrimitiveType, RenderPrimitiveData& outPrimitiveData )
		{
			//combine the primitive and transform types
			PxU8 streamItem;
			streamify( streamItem );
			outPrimitiveType = (streamItem >> 2) + 1;
			outTransformType = (streamItem & 0x3) + 1;
			outTransformData = visitRenderTransform<RenderTransformData>( outTransformType, RenderTransformStreamOperator<PvdDataStreamEventInStream>( this ) );
			outPrimitiveData = visitRenderPrimitive<RenderPrimitiveData>( outPrimitiveType, RenderPrimitiveStreamOperator<PvdDataStreamEventInStream>( this ) );
		}
		
		inline void streamify( const PxF32*& inPositions, PxU32& inPositionCount, const PxU32*& in32BitIndices, const PxU16*& in16BitIndices, PxU32& inTriangleCount )
		{
			streamify( inPositionCount );
			inPositions = reinterpret_cast< PxF32* >( const_cast< PxU8* >( mBlockParser.mBegin ) );
			readPtrBlock( inPositions, inPositionCount * 3 );
			streamify( inTriangleCount );
			bool is16Bit = inTriangleCount & 0x1;
			inTriangleCount = inTriangleCount >> 1;
			in32BitIndices = NULL;
			in16BitIndices = NULL;
			PxU32 theIndexCount = inTriangleCount * 3;
			PxU8* theDataPtr = const_cast< PxU8* >( mBlockParser.mBegin );
			if ( theIndexCount )
			{
				if ( is16Bit)
				{
					in16BitIndices = reinterpret_cast< PxU16* >( theDataPtr );
					readPtrBlock( in16BitIndices, theIndexCount );
				}
				else
				{
					in32BitIndices = reinterpret_cast< PxU32* >( theDataPtr );
					readPtrBlock( in32BitIndices, theIndexCount );
				}
			}
		}
		inline void streamify( PropertyStructEntry*& inEntries, PxU32& inEntryCount )
		{
			streamify( inEntryCount );
			inEntries = reinterpret_cast< PropertyStructEntry* >( const_cast< PxU8* >( mBlockParser.mBegin ) );
			if ( DoSwapBytes )
			{
				//Do an in-place byte swapping operation.
				for ( PxU32 idx =0; idx < inEntryCount; ++idx )
				{
					streamify( inEntries[idx].mProperty );
					streamify( inEntries[idx].mOffset );
					streamify( inEntries[idx].mType );
				}
			}
			else
			{
				const PxU8* temp;
				PxU32 len = inEntryCount * sizeof( PropertyStructEntry );
				bulkStreamify( temp, len );
			}
		}

		//On the flip side the offset stored in the struct
		inline void streamifyEmbeddedString( const PxU8* inData, PxU32, PxU32 inStringOffset )
		{
			PxU64 startAddr = static_cast< PxU64> ( reinterpret_cast<size_t>( inData ) );
			PxU8* destAddr = const_cast<PxU8*>( inData + inStringOffset );
			PxU32 len;
			PxU8* dataPtr = const_cast<PxU8*>( mBlockParser.mBegin );
			streamify( len );
			//write back the result to the memory address so later clients can get the string length
			//immediately before the string.
			memcpy( dataPtr, &len, sizeof( len ) );
			const char* theDataPtr = reinterpret_cast< const char* >( mBlockParser.mBegin );
			updateFail( mBlockParser.readBlock<char>( len ) );
			PxU32 offset = static_cast<PxU32>( (static_cast<PxU64>( reinterpret_cast<size_t>( theDataPtr ) ) - startAddr) );
			physx::shdfnd::memCopy( destAddr, &offset, sizeof( offset ) );
		}

		inline void endianConvert( const PxU8* inData, PropertyStructEntry* entries, PxU32 entryCount, bool inIs64Bit )
		{
			const PxU8* currentBegin = mBlockParser.mBegin;
			for ( PxU32 idx =0; idx < entryCount; ++idx )
			{
				mBlockParser.mBegin = inData + entries[idx].mOffset;
				if ( (PxU8)entries[idx].mType != PvdCommLayerDatatype::String )
				{
					PxU32 theType = (PxU8)entries[idx].mType;
					if ( theType == PvdCommLayerDatatype::Pointer )
						theType = inIs64Bit ? PvdCommLayerDatatype::U64 : PvdCommLayerDatatype::U32;
					TDatatypeHandler* theHandler( findHandlerForType( (PxU8)theType ) );
					PX_ASSERT( theHandler );
					if ( theHandler ) theHandler->streamify( const_cast<PxU8*>( mBlockParser.mBegin ) );
				}
			}
			mBlockParser.mBegin = currentBegin;
		}
	};
}

#endif

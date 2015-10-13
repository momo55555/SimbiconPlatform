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

#ifndef PVD_PVDCONNECTIONEVENTOUTSTREAM_H
#define PVD_PVDCONNECTIONEVENTOUTSTREAM_H

#include "PvdDataStreamEventStream.h"
#include "PVDCommByteStream.h"
#include "PvdRenderTypes.h"
#include "BlockParserFunctions.h"
#include "PsIntrinsics.h"
#include "PsArray.h"
#include "../src/PxProfileMemoryBuffer.h"

namespace PVD
{
	template<typename TU8ContainerType>
	struct SU8ContainerWriter
	{
		TU8ContainerType* mContainer;
		SU8ContainerWriter( TU8ContainerType* inContainer ) : mContainer( inContainer ) {}
		

		//Writing never does endian conversions so this is fine.
		inline void writeBlock( const PxU8* inData, PxU32 inItemSize, PxU32 inLength, PxU32 inStride )
		{
			PxU32 current = mContainer->size();
			PxU32 writeSize = inItemSize * inLength;
			if ( inData == NULL ) 
				writeSize = 0;
			if ( writeSize )
			{
				mContainer->resize( mContainer->size() + writeSize );
				PxU8* writePtr = &((*mContainer)[current]);
				if ( inStride == 0 || inStride == inItemSize )
					physx::shdfnd::memCopy( writePtr, inData, writeSize );
				else
				{
					PxU32 increment = inStride;
					for ( PxU32 idx =0; idx < inLength; ++idx, writePtr += inItemSize, inData += increment )
						physx::shdfnd::memCopy( writePtr, inData, inItemSize );
				}
			}
		}

		template<typename TDataType>
		inline void writeBlock( const TDataType* inData, PxU32 inLength )
		{
			writeBlock( reinterpret_cast< const PxU8*>( inData ), sizeof( TDataType ), inLength, 0 );
		}

		template<typename TDataType>
		inline void streamify( TDataType inType )
		{
			PxU8* theData = reinterpret_cast<PxU8*>( &inType );
			for( PxU32 idx = 0; idx < sizeof( inType ); ++idx )
				mContainer->pushBack( theData[idx] );
		}

		inline void streamify( PxU8 inData )
		{
			mContainer->pushBack( inData );
		}
	};

	struct MemoryBufferWriter
	{
		physx::profile::MemoryBuffer<>* mContainer;
		MemoryBufferWriter( physx::profile::MemoryBuffer<>* inBuffer )
			: mContainer( inBuffer )
		{
		}

		//Writing never does endian conversions so this is fine.
		inline void writeBlock( const PxU8* __restrict inData, PxU32 inItemSize, PxU32 inLength, PxU32 inStride )
		{
			mContainer->writeStrided( inData, inItemSize, inLength, inStride );
		}

		template<typename TDataType>
		inline void writeBlock( const TDataType* inData, PxU32 inLength )
		{
			mContainer->write( inData, inLength );
		}

		template<typename TDataType>
		inline void streamify( TDataType inType )
		{
			mContainer->write( inType );
		}

		inline void streamify( PxU8 inData )
		{
			mContainer->write( inData );
		}

	};

	template<typename TWriterType, bool TSwapBytes>
	struct SByteSwappingContainerWriter
	{
		TWriterType* mWriter;
		SByteSwappingContainerWriter( TWriterType* inWriter )
			: mWriter( inWriter )
		{
		}

		bool val() { return TSwapBytes; }
		
		template<typename TDataType>
		inline void streamify( TDataType inData )
		{
			if( val() )
			{
				PxU8* theData = reinterpret_cast<PxU8*>( &inData );
				BlockParseFunctions::swapBytes<sizeof(TDataType)>( theData );
			}
			mWriter->streamify( inData );
		}

		//This function cannot be done this way.
		inline void writeBlock( const PxU8*, PxU32, PxU32, PxU32 )
		{
			PX_ASSERT( false );
		}
		template<typename TDataType>
		inline void writeBlock( const TDataType* inData, PxU32 inLength )
		{
			if ( val() )
			{
				for( PxU32 idx = 0; idx < inLength; ++idx )
					streamify( inData[idx] );
			}
			else
				mWriter->writeBlock( inData, inLength );
		}
		inline void streamify( PxU8 inData )
		{
			mWriter->streamify( inData );
		}

	};

	template<typename TStreamType=PvdCommOutStream>
	struct SCommOutStreamWriter
	{
		TStreamType*	mStream;
		PvdCommLayerError	mLastError;
		SCommOutStreamWriter( TStreamType* inStream )
			: mStream( inStream )
			, mLastError( PvdCommLayerError::None ) {}

		inline void writeBytes( const PxU8* inBytes, PxU32 inLength )
		{
			if ( mLastError == PvdCommLayerError::None )
				mLastError = mStream->write( inBytes, inLength );
		}

		inline void streamify( PxU32 inData )
		{
			UStreamDatatypeConverter theConverter;
			theConverter.mU32 = inData;
			writeBytes( theConverter.mU8, 4 );
		}
		
		inline void streamify( PxU16 inData )
		{
			UStreamDatatypeConverter theConverter;
			theConverter.mU16 = inData;
			writeBytes( theConverter.mU8, 2 );
		}
		
		//Writing never does endian conversions so this is fine.
		inline void writeBlock( const PxU8* inData, PxU32 inItemSize, PxU32 inLength, PxU32 inStride )
		{
			PxU32 writeSize = inItemSize * inLength;
			if ( inData == NULL )
				writeSize = 0;
			if ( writeSize )
			{
				if ( inStride == 0 || inStride == inItemSize )
					writeBytes( reinterpret_cast< const PxU8* >( inData ), writeSize );
				else
				{
					PxU32 increment = inStride;
					for ( PxU32 idx =0; idx < inLength; ++idx, inData += increment )
						writeBytes( inData, inItemSize );
				}
			}
		}
		//Writing never does endian conversions so this is fine.
		template<typename TDataType>
		inline void writeBlock( const TDataType* inData, PxU32 inLength )
		{
			writeBlock( reinterpret_cast< const PxU8*>( inData ), sizeof( TDataType ), inLength, 0 );
		}
		inline void streamify( PxU8 inData )
		{
			writeBytes( &inData, 1 );
		}
	};

	//Record the number of bytes required to write the requested data.
	struct SCommByteCounterWriter
	{
		PxU32 mByteCount;
		SCommByteCounterWriter() : mByteCount( 0 ) {}

		inline void streamify( PxU32)
		{
			mByteCount += 4;
		}
		//Writing never does endian conversions so this is fine.
		inline void writeBlock( const PxU8* , PxU32 inItemSize, PxU32 inLength, PxU32 )
		{
			PxU32 writeSize = inItemSize * inLength;
			mByteCount += writeSize;
		}
		//Writing never does endian conversions so this is fine.
		template<typename TDataType>
		inline void writeBlock( const TDataType* inData, PxU32 inLength )
		{
			writeBlock( reinterpret_cast< const PxU8*>( inData ), sizeof( TDataType ), inLength, 0 );
		}
		inline void streamify( PxU8 )
		{
			mByteCount += 1;
		}
		inline void streamify( PxU16 )
		{
			mByteCount += 2;
		}
	};

	template<typename TStreamType>
	class AbstractOutStreamDatatypeHandler
	{
	protected:
		TStreamType* mOutStream;
	public:
		AbstractOutStreamDatatypeHandler( TStreamType* inOutStream )
			: mOutStream( inOutStream )
		{
		}
		virtual ~AbstractOutStreamDatatypeHandler(){}
		void SetStream( TStreamType* inStream ) { mOutStream = inStream; }
		virtual PvdCommLayerDatatype datatype() const = 0;
		virtual void streamify( const PvdCommLayerData& inData ) = 0;
		virtual PxU32 byteSize() const = 0;
	};


	template<typename TStreamType, typename TDataType>
	class OutStreamDatatypeHandler : public AbstractOutStreamDatatypeHandler<TStreamType>
	{
	public:
		OutStreamDatatypeHandler( TStreamType* inStream ) : AbstractOutStreamDatatypeHandler<TStreamType>( inStream )
		{
		}
		virtual PvdCommLayerDatatype datatype() const { return getDatatypeForValue(TDataType()); }
		virtual void streamify( const PvdCommLayerData& inData ) { AbstractOutStreamDatatypeHandler<TStreamType>::mOutStream->streamify( getCommLayerData<TDataType>( inData ) ); }
		//Incorrect for complex variable-length types such as strings or buffers.
		virtual PxU32 byteSize() const { return sizeof( TDataType ); }
	};

	template<typename TStreamType
			, typename TAllocator>
	struct AbstractOutStreamDatatypeHandlerCreator
	{
		template<typename TDataType>
		AbstractOutStreamDatatypeHandler<TStreamType>* operator()( const TDataType& )
		{
			typedef OutStreamDatatypeHandler<TStreamType, TDataType> TDatatypeHandler;
			TDatatypeHandler* theHandler = (TDatatypeHandler*)TAllocator().allocate( sizeof( TDatatypeHandler ), __FILE__, __LINE__ );
			new( theHandler ) TDatatypeHandler( NULL );
			return theHandler;
		}
		
		AbstractOutStreamDatatypeHandler<TStreamType>* operator()()
		{
			return NULL;
		}
	};

	/**
	 *	Boils down the event serialization code into just a few
	 *	functions.  Cheap to create class meant to be created where
	 *	needed.
	 */
	template<typename TWriterType>
	struct PvdDataStreamEventOutStream
	{
		typedef AbstractOutStreamDatatypeHandler<PvdDataStreamEventOutStream<TWriterType> > TDatatypeHandler;

		TWriterType* mWriter;
		TDatatypeHandler**									mUserDatatypeHandlers;


		PvdDataStreamEventOutStream( TWriterType* inWriter )
			: mWriter( inWriter )
			, mUserDatatypeHandlers( NULL )
		{
		}

		TDatatypeHandler* findHandlerForType( PxU32 inIdx )
		{
			if ( mUserDatatypeHandlers != NULL )
				return mUserDatatypeHandlers[inIdx];
			return NULL;
		}
		
		void setDatatypeHandlers( TDatatypeHandler** inHandlers )
		{
			mUserDatatypeHandlers = inHandlers;
		}
		
		//Writing never does endian conversions so this is fine.
		template<typename TDataType>
		inline void writeBlock( const TDataType* inData, PxU32 inLength )
		{
			mWriter->writeBlock( inData, inLength );
		}

		inline void streamify( bool inData )
		{
			PxU8 theData = inData ? 1 : 0;
			streamify( theData );
		}

		inline void streamify( PxU8 inData )
		{
			mWriter->streamify( inData );
		}

		inline void streamify( PxU16 inData )
		{
			mWriter->streamify( inData );
		}

		inline void streamifySectionTimestamp( PxU64 inData )
		{
			streamify( inData );
		}
		
		inline void streamify( PxU32 inData )
		{
			mWriter->streamify( inData );
		}

		inline void streamify( PxU64 inData )
		{
			UStreamDatatypeConverter theConverter;
			theConverter.mU64 = inData;
			writeBlock( theConverter.mU8, 8 );
		}

		inline void streamify( PxI8 inData )
		{
			UStreamDatatypeConverter theConverter;
			theConverter.mI8[0] = inData;
			streamify( theConverter.mU8[0] );
		}

		inline void streamify( PxI16 inData )
		{
			UStreamDatatypeConverter theConverter;
			theConverter.mI16 = inData;
			streamify( theConverter.mU16 );
		}

		inline void streamify( PxI32 inData )
		{
			UStreamDatatypeConverter theConverter;
			theConverter.mI32 = inData;
			streamify( theConverter.mU32 );
		}
		inline void streamify( PxI64 inData )
		{
			UStreamDatatypeConverter theConverter;
			theConverter.mI64 = inData;
			streamify( theConverter.mU64 );
		}

		inline void streamify( PxF32 inData )
		{
			UStreamDatatypeConverter theConverter;
			theConverter.mF32 = inData;
			streamify( theConverter.mU32 );
		}
		
		inline void streamify( PxF64 inData )
		{
			UStreamDatatypeConverter theConverter;
			theConverter.mF64 = inData;
			streamify( theConverter.mU64 );
		}

		inline void streamify( const char* inString )
		{
			PxU32 inLen = SafeStrLen( inString ) + 1;
			streamify( inLen );
			if ( inLen > 1 )
				writeBlock( inString, inLen );
			else
				streamify( (PxU8)0 );
		}

		inline void streamify( PxU32 inPropertyName, const NamedValueDefinition* inDefinitions, PxU32 inDefinitionLength )
		{
			streamify( inPropertyName );
			streamify( inDefinitionLength );
			for ( PxU32 idx = 0; idx < inDefinitionLength; ++idx )
			{
				streamify( inDefinitions[idx].mName );
				streamify( inDefinitions[idx].mValue );
			}
		}

		inline void writeRaw( PxU8 inDatatype, const PvdCommLayerData& inData )
		{
			commLayerDataOperate<PvdCommLayerData>( inDatatype, inData, CommLayerStreamOperator<PvdDataStreamEventOutStream>( this ) );
		}
		
		inline void writeRaw( PxU8 inDatatype, const PvdCommLayerMediumData& inData )
		{
			commLayerMediumDataOperate<PvdCommLayerMediumData>( inDatatype, inData, CommLayerMediumStreamOperator<PvdDataStreamEventOutStream>( this ) );
		}
		
		inline void writeRaw( PxU8 inDatatype, const PvdCommLayerSmallData& inData )
		{
			commLayerSmallDataOperate<PvdCommLayerSmallData>( inDatatype, inData, CommLayerSmallStreamOperator<PvdDataStreamEventOutStream>( this ) );
		}

		inline void streamify( PxU8 inDatatype, const PvdCommLayerData& inData )
		{
			writeRaw( inDatatype, inData );
		}
		
		inline void streamify( PxU8 inDatatype, const PvdCommLayerMediumData& inData )
		{
			writeRaw( inDatatype, inData );
		}
		
		inline void streamify( PxU8 inDatatype, const PvdCommLayerSmallData& inData )
		{
			writeRaw( inDatatype, inData );
		}

		template<PxU32 TNumFloats>
		inline void streamify( const TFixedFloatArray<TNumFloats>& inData )
		{
			writeBlock( inData.mFloats, TNumFloats );
		}
		
		template<PxU32 TNumData>
		inline void streamify( const TFixedU32Array<TNumData>& outData )
		{
			writeBlock( outData.mData, TNumData );
		}

		template<typename TDataType>
		inline void streamify( const GenericDatatype<TDataType>& inData )
		{
			streamify( inData.mValue );
		}

		template<typename TDataType>
		inline void streamify( const Buffer<TDataType>& inData )
		{
			streamify( inData.mLength );
			writeBlock( inData.mData, inData.mLength );
		}
		
		template<typename TDataType>
		inline void bulkStreamify( TDataType*& outData, PxU32 inDataLen )
		{
			writeBlock( outData, inDataLen );
		}
		
		inline void streamify( const HeightFieldSample& inData )
		{
			const PxU16* theData = reinterpret_cast< const PxU16* >( &inData );
			streamify( theData[0] );
			const PxU8* theSmallData = reinterpret_cast< const PxU8* >( &theData[1] );
			streamify( theSmallData[0] );
			streamify( theSmallData[1] );
		}
		
		inline void streamify( const Section& inData )
		{
			streamify( inData.mType );
			streamify( inData.mTimestamp );
		}

		inline void streamify( const PxU32* inProperties, PxU32 inNumProps )
		{
			streamify( inNumProps );
			writeBlock( inProperties, inNumProps );
		}
		
		inline void streamify( const PvdCommLayerDatatype* inDatatypes, PxU32 inNumProps )
		{
			streamify( inNumProps );
			for ( PxU32 idx = 0; idx < inNumProps; ++idx )
				streamify( inDatatypes[idx].mDatatype );
		}
		
		inline void streamify( const PvdCommLayerDatatype* inDatatypes, PxU32 inNumProps, const PvdCommLayerData*	inValues )
		{
			for ( PxU32 idx = 0; idx < inNumProps; ++idx )
			{
				TDatatypeHandler* theHandler = findHandlerForType( idx );
				if ( theHandler )
					theHandler->streamify( inValues[idx] );
				else
					writeRaw( inDatatypes[idx].mDatatype, inValues[idx] );
			}
		}
		
		inline void streamify( const char* inStr1, const char* inStr2 )
		{
			streamify( inStr1 );
			streamify( inStr2 );
		}
		
		inline void streamify( const char* inStr1, const char* inStr2, const char* inStr3 )
		{
			streamify( inStr1 );
			streamify( inStr2 );
			streamify( inStr3 );
		}

		inline void streamify( PxU32 inNumBlocks
									, const PvdCommLayerDatatype* inDatatypes
									, PxU32 inNumProps
									, const PvdCommLayerData* inData )
		{
			streamify( inNumBlocks );
			for ( PxU32 idx =0; idx < inNumBlocks; ++idx )
				streamify( inDatatypes, inNumProps, inData + idx * inNumProps );
		}

		inline void streamify( const PvdCommLayerDatatype* /*inDatatypes*/
												, PxU32 inPropertyCount
												, PxU32 inStride
												, PxU32 inValueCount
												, const PxU8* inValues )
		{
			//This check is required and has happened.
			if ( inValues == NULL )  
				inValueCount = 0;
			streamify( inValueCount );
			//Calculate the byte size of the data
			PxU32 itemSize = 0;
			for ( PxU32 idx = 0; idx < inPropertyCount; ++idx )
				itemSize += findHandlerForType( idx )->byteSize();
			mWriter->writeBlock( inValues, itemSize, inValueCount, inStride );
		}

		inline void streamify( PxU8 inTransformType, const RenderTransformData& inData )
		{
			streamify( inTransformType );
			visitRenderTransform<RenderTransformData>( inTransformType, inData, RenderTransformStreamOperator<PvdDataStreamEventOutStream>( this ) );
		}

		inline void streamify( const IdentityTransform& )
		{
		}

		inline void streamify( PxU8 inTransformType, const RenderTransformData& inTransformData, PxU8 inPrimitiveType, const RenderPrimitiveData& inPrimitiveData )
		{
			//combine the primitive and transform types
			PxU8 streamItem( static_cast<PxU8>(((inPrimitiveType-1) << 2)) + (inTransformType - 1) );
			streamify( streamItem );
			visitRenderTransform<RenderTransformData>( inTransformType, inTransformData, RenderTransformStreamOperator<PvdDataStreamEventOutStream>( this ) );
			visitRenderPrimitive<RenderPrimitiveData>( inPrimitiveType, inPrimitiveData, RenderPrimitiveStreamOperator<PvdDataStreamEventOutStream>( this ) );
		}

		inline void streamify( const PxF32*	inPositions, PxU32 inPositionCount, const PxU32* in32BitIndices, const PxU16* in16BitIndices, PxU32	inTriangleCount )
		{
			streamify( inPositionCount );
			writeBlock( inPositions, inPositionCount * 3 );
			PxU32 theIndexCount = inTriangleCount * 3;
			inTriangleCount = inTriangleCount << 1;
			if ( in32BitIndices != NULL )
			{
				streamify( inTriangleCount );
				writeBlock( in32BitIndices, theIndexCount );
			}
			else
			{
				//Adding one to the index count to indicate we are dealing with 16 bit indices
				inTriangleCount += 1;
				streamify( inTriangleCount );
				writeBlock( in16BitIndices, theIndexCount );
			}
		}

		inline void streamify( const PropertyStructEntry* inEntries, PxU32 inEntryCount )
		{
			streamify( inEntryCount );
			writeBlock( inEntries, inEntryCount );
		}

		inline void streamifyEmbeddedString( const PxU8* inData, PxU32 inDataLen, PxU32 inStringOffset )
		{
			if ( inStringOffset + sizeof(const char*) <= inDataLen )
			{
				const char* theStrPtr = *reinterpret_cast<const char* const *>( inData + inStringOffset );
				streamify( theStrPtr );
			}
		}
	};
}

#endif
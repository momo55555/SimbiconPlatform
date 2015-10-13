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

#ifndef PVD_PVDCONNECTIONSTREAMDEBUGGERMESSAGEWRITER_H
#define PVD_PVDCONNECTIONSTREAMDEBUGGERMESSAGEWRITER_H

#include "PVDCommByteStream.h"
#include "PvdDataStreamEventOutStream.h"

namespace PVD
{
	template<typename TAllocator=ClientAllocator<char>
			, typename TDeleteOperator=SDeleteOperator
			, bool TSwapBytes=false>
	class PvdConnectionStreamDebuggerMessageWriter
	{
		typedef PvdConnectionStreamDebuggerMessageWriter<TAllocator,TDeleteOperator,TSwapBytes> TThisType;
		typedef physx::shdfnd::Array<PxU8,TAllocator> TPXU8Container;

		PvdCommOutStream*		mOutStream;
		PvdCommLayerError		mLastError;
		TPXU8Container			mEventContainer;
		PxU32					mEventCount;
		PxU8*					mWritePointer;

		PvdConnectionStreamDebuggerMessageWriter( const PvdConnectionStreamDebuggerMessageWriter& inOther );
		PvdConnectionStreamDebuggerMessageWriter& operator=( const PvdConnectionStreamDebuggerMessageWriter& inOther );
	public:
		
		PvdConnectionStreamDebuggerMessageWriter( PvdCommOutStream* inStream )
			: mOutStream( inStream )
			, mLastError( PvdCommLayerError::None )
			, mEventCount( 0 )
		{
			//Resize to contain an 8 bit event name
			//32 bit following data size
			//32 bit following event count
			mEventContainer.resize( 9 );
		}

		void setOutStream( PvdCommOutStream* inStream ) { mOutStream = inStream; }

		inline ~PvdConnectionStreamDebuggerMessageWriter() 
		{
			if ( mOutStream )
				mOutStream->destroy();
			mOutStream = NULL;
		}
		inline PvdCommLayerError pause()
		{
			return sendEvent( DebugMessagePause() );
		}
		inline PvdCommLayerError record()
		{
			return sendEvent( DebugMessageRecord() );
		}
		inline PvdCommLayerError monitor()
		{
			return sendEvent( DebugMessageMonitor() );
		}
		inline PvdCommLayerError disconnect()
		{
			return sendEvent( DebugMessageDisconnect() );
		}
		inline PvdCommLayerError setPropertyValue( PxU64 inInstanceId, PxU32 inProperty, const PvdCommLayerValue& inValue )
		{
			return sendEvent( createDebugMessagesetPropertyValue( inInstanceId, inProperty, inValue ) );
		}

		inline void streamify( PxU32 inData, PxU8* inResultPtr )
		{
			PxU8* theDataPtr = reinterpret_cast<PxU8*>( &inData );
			for ( PxU32 idx = 0; idx < sizeof( PxU32 ); ++idx )
				inResultPtr[idx] = theDataPtr[idx];
		}

		inline void streamify( PxU32 inData ) 
		{ 
			streamify( inData, mWritePointer );
			mWritePointer += sizeof( inData );
		}
		
		inline PvdCommLayerError flush()
		{
			PxU32 theContainerSize = static_cast<PxU32>( mEventContainer.size() );
			if ( mLastError == PvdCommLayerError::None 
					&& theContainerSize > 9 )
			{
				mEventContainer[0] = static_cast<PxU8>( PvdDebugMessageType::EventBatch );
				mWritePointer = mEventContainer.begin() + 1;
				SByteSwappingContainerWriter<TThisType, TSwapBytes> theWriter(this);
				theWriter.streamify( theContainerSize - 5 );
				theWriter.streamify( mEventCount );
				mLastError = mOutStream->write( &mEventContainer[0], static_cast<PxU32>( mEventContainer.size() ) );
				mEventContainer.resize(9);
				mEventCount = 0;
			}
			return mLastError;
		}

	protected:
		
		template<typename TEventType>
		inline PvdCommLayerError sendEvent( TEventType inEvent )
		{
			mEventContainer.pushBack( getPVDDebugMessageType<TEventType>().mType );
			SU8ContainerWriter<TPXU8Container> theWriter( &mEventContainer );
			typedef SByteSwappingContainerWriter<SU8ContainerWriter<TPXU8Container>, TSwapBytes > TSwapWriterType;
			TSwapWriterType theSwapWriter( &theWriter );
			PvdDataStreamEventOutStream<TSwapWriterType> theOutStream( &theSwapWriter );
			inEvent.streamify( theOutStream );
			++mEventCount;
			return mLastError;
		}
	};
}

#endif
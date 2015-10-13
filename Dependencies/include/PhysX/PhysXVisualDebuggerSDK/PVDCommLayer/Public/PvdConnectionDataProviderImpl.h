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

#ifndef PVDCONNECTIONDATAPROVIDERIMPLH
#define PVDCONNECTIONDATAPROVIDERIMPLH
#include "PvdConnectionDataProvider.h"
#include "PVDCommByteStream.h"
#include "PsThread.h"
#include "PsMutex.h"
#include "PsUserAllocated.h"
#include "PVDConnectionEvents.h"
#include "PvdDataStreamEventInStream.h"

namespace PVD
{
	//Provides a buffering input stream.
	struct PvdCommInStreamStoringReader
	{
		Array<PxU8>			mData;
		PvdCommInStream*	mStream;
		PvdCommInStreamStoringReader( PvdCommInStream*	inStream )
			: mStream( inStream )
		{
		}
		~PvdCommInStreamStoringReader()
		{
			mStream->destroy();
		}

		PxU8* read( PxU32& inLength )
		{
			if ( inLength )
			{
				PxU32 size = mData.size();
				mData.resize( size + inLength );
				PxU8* retval = mData.begin() + size;
				PxU32 theLength = inLength;
				mStream->readBytes( retval, inLength );
				if ( theLength != inLength )
					mData.resize( size + theLength );
				return retval;
			}
			inLength = 0;
			return mData.begin();
		}

		template<typename TDataType>
		void streamify( TDataType& inData )
		{
			PxU32 theLen = sizeof( inData );
			PxU8* theNewData = read( theLen );
			if ( theLen == sizeof( inData ) )
				inData = *reinterpret_cast< TDataType* >( theNewData );
		}
	};

	/**
	 *	Default pvd connection provider.  Launches a thread that does blocking reads as long
	 *	as the reader is connected.
	 */
	class PvdConnectionRunningProviderImpl : public PvdConnectionRunningProvider, public Thread
	{
		PvdConnectionDataHandler*	mHandler;
		PvdCommInStreamStoringReader mReader;

	public:
		PvdConnectionRunningProviderImpl( PvdConnectionDataHandler* inHandler, PvdCommInStream* inStream )
			: mHandler( inHandler )
			, mReader( inStream )
		{
			mHandler = inHandler;
			if ( mHandler != NULL && inStream != NULL &&
				inStream->isConnected() )
				start( 0x10000 ); //64K of stack to allow the handler space to process the result
		}

		~PvdConnectionRunningProviderImpl()
		{
			waitForQuit();
		}

		virtual void destroy()
		{
			PX_DELETE( this );
		}

		virtual void execute()
		{
            setName("PVD::InStream");
			while( mReader.mStream->isConnected() )
			{
				SEventHeader theHeader = { 0, 0 };
				theHeader.streamify( mReader );
				PxU32 theSize = theHeader.mSize;
				if ( mReader.mStream->isConnected() )
					mReader.read( theSize );
				mHandler->handleData( mReader.mData.begin(), mReader.mData.size() );
				mReader.mData.clear();
			}
			mHandler->handleNoMoreData();
			quit();
		}
	};


	/**
	 *	Factory interface that provides a running data provider.  The running provider is the entity
	 *	that pushes new information coming from the pvd application into the SDK-side
	 *	PVD SDK system.
	 */
	class PvdConnectionDataProviderImpl : public UserAllocated, public PvdConnectionDataProvider
	{
	public:
		virtual ~PvdConnectionDataProviderImpl(){}
		virtual PvdConnectionRunningProvider* beginHandlingData( PvdConnectionDataHandler* inHandler, PvdCommInStream* inStream )
		{
			if ( inHandler && inStream )
			{
				PvdConnectionRunningProviderImpl* theImpl = PX_NEW( PvdConnectionRunningProviderImpl )( inHandler, inStream );
				return theImpl;
			}
			return NULL;
		}
	};
}

#endif

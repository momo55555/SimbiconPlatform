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

#ifndef PVD_PVDCOMMNETWORKSTREAM_H
#define PVD_PVDCOMMNETWORKSTREAM_H
#include "PVDCommByteStream.h"
#include "PsUserAllocated.h"
#include "PsSocket.h"
#include "PsMutex.h"
#include "PsThread.h"

namespace PVD
{
	
	class SocketInfo : public UserAllocated
	{
		physx::shdfnd::Socket mSocket;
		PxU32		mRefCount;

		SocketInfo( const SocketInfo& inOther );
		SocketInfo& operator=( const SocketInfo& inOther );

	public:
		SocketInfo() 
			: mSocket( false )
			, mRefCount( 0 ) {}
		physx::shdfnd::Socket& Socket() { return mSocket; }
		void addRef(){ ++mRefCount; }
		void release()
		{
			if ( mRefCount ) -- mRefCount;
			if ( !mRefCount ) PX_DELETE( this );
		}
	};

	//No caching at all here, the caching is either a layer above here
	//or a layer below.
	class SocketCommOutStream : public PvdCommOutStream, public UserAllocated
	{
		SocketInfo* mInfo;
	public:
		SocketCommOutStream( SocketInfo* inInfo ) 
			: mInfo( inInfo ) 
		{ 
			mInfo->addRef();
		}

		virtual ~SocketCommOutStream(){ mInfo->release(); mInfo=NULL; }
	
		virtual PvdCommLayerError write( const PxU8* inBytes, PxU32 inLength )
		{
			return doWriteBytes( inBytes, inLength );
		}

		PvdCommLayerError doWriteBytes( const PxU8* inBytes, PxU32 inLength )
		{
			if ( inLength == 0 )
				return PvdCommLayerError::None;

			PxU32 amountWritten = 0;
			do
			{
				//Sockets don't have to write as much as requested, so we need
				//to wrap this call in a do/while loop.
				//If they don't write any bytes then we consider them disconnected.
				amountWritten = mInfo->Socket().write( inBytes, inLength );
				inLength -= amountWritten;
				inBytes += amountWritten;
			}while( inLength && amountWritten );
			
			if ( amountWritten == 0 ) return PvdCommLayerError::NetworkError;

			return PvdCommLayerError::None;
		}

		virtual PvdCommLayerError flush() 
		{ 
			return mInfo->Socket().flush() == true ?  (PxU8)PvdCommLayerError::None : (PxU8)PvdCommLayerError::NetworkError;
		}

		virtual bool isConnected() const { return mInfo->Socket().isConnected(); }
		virtual void disconnect() { mInfo->Socket().disconnect(); }
		virtual void destroy() { PX_DELETE( this ); }
	};

	class SocketCommInStream : public PvdCommInStream, public UserAllocated
	{
		SocketInfo* mInfo;
	public:
		SocketCommInStream( SocketInfo* inInfo ) : mInfo( inInfo ) 
		{ 
			mInfo->addRef(); 
		}
		virtual ~SocketCommInStream(){ mInfo->release(); mInfo=NULL; }
		virtual PvdCommLayerError readBytes( PxU8* outBytes, PxU32 ioRequested )
		{
			if ( !isConnected() || outBytes == NULL )
				return PvdCommLayerError::NetworkError;
			if ( ioRequested == 0 )
				return PvdCommLayerError::None;
			PxU32 amountRead = 0;
			do
			{
				amountRead = mInfo->Socket().read( outBytes, ioRequested );
				ioRequested -= amountRead;
			}while( ioRequested && amountRead );

			if ( amountRead == 0 )
				return PvdCommLayerError::NetworkError;
			
			return PvdCommLayerError::None;
		}
		virtual bool isConnected() const
		{
			return mInfo->Socket().isConnected();
		}
		virtual void disconnect() { mInfo->Socket().disconnect(); }
		virtual void destroy() { PX_DELETE( this ); }
	};

	class PvdCommNetworkStreams
	{
	public:
		bool connect( const char* inHost
					, int inPort
					, unsigned int inTimeoutInMilliseconds
					, PvdCommInStream*& outInStream
					, PvdCommOutStream*& outOutStream )
		{
			outInStream = NULL;
			outOutStream = NULL;
			SocketInfo* theInfo = PX_NEW(SocketInfo)();
			if ( theInfo->Socket().connect( inHost, (PxU16)inPort, inTimeoutInMilliseconds ) )
			{
				theInfo->Socket().setBlocking( true );
				outInStream = PX_NEW( SocketCommInStream )( theInfo );
				outOutStream = PX_NEW( SocketCommOutStream )( theInfo );
				return true;
			}
			theInfo->addRef();
			theInfo->release();
			return false;
		}
	};
}

#endif

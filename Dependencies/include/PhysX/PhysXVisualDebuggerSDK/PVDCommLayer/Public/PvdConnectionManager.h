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

#ifndef PVD_PVDCONNECTIONFACTORYMANAGER_H
#define PVD_PVDCONNECTIONFACTORYMANAGER_H
#include "PvdConnection.h"

namespace physx
{
	class PxProfileZoneManager;
}

namespace PVD //PhysX Visual Debugger
{
	class PvdConnection;
	class PvdCommInStream;
	class PvdCommOutStream;
	class PvdConnectionDataProvider;

	class PvdConnectionHandler
	{
	protected:
		virtual ~PvdConnectionHandler(){}
	public:
		virtual void onPvdConnected( PvdConnection* inFactory ) = 0;
		virtual void onPvdDisconnected( PvdConnection* inFactory ) = 0;
	};

	/**
	 *	The connection factory manager provides ways of managing a single PVD connection.
	 *	Clients can be notified when the connection is created and can setup a policy
	 *	for dealing with the incoming data from the PVD application if there is any.
	 *
	 *	The default data provider uses a thread that does a block read on the incoming
	 *	connection stream.  If you would like to do something else you will need first
	 *	implement you own network abstraction as the physx networking layers don't work
	 *	in non-blocking mode on platforms other than windows (and they only partially work
	 *	in non-blocking mode on windows).  Second please pay attention to the comments in
	 *	the implementation file: PvdConnectionDataProviderImpl.h
	 */
	class PvdConnectionManager
	{
	protected:
		virtual ~PvdConnectionManager(){}
	public:
		/**
		 *	The factory manager has exactly one data provider.  The default one
		 *	spawns a read thread to handle messages coming from PVD when a new
		 *	connection is made.
		 */
		virtual void setDataProvider( PvdConnectionDataProvider* inProvider ) = 0;
		/**
		 *	Set the profile zone manager.  This takes care of ensuring that all profiling
		 *	events get forwarded to PVD.
		 */
		virtual void setProfileZoneManager( physx::PxProfileZoneManager& inManager ) = 0;
		/**
		 *	Handler will be notified every time there is a new connection.
		 */
		virtual void addHandler( PvdConnectionHandler* inHandler ) = 0;
		/**
		 *	Handler will be notified when a connection is destroyed.
		 */
		virtual void removeHandler( PvdConnectionHandler* inHandler ) = 0;
		/**
		 *	Connect to the factory.  Use error checking turns on some form of error checking
		 *	in the debugger stream.  This should never be left on in release versions but is
		 *	useful when you want to figure out why PVD isn't displaying what you think it
		 *	should.
		 *	Using double buffered output is highly recommend to avoid blocking the sending thread
		 *	on the output system.  Unfortunately, when tunnelling through a 2.8.X connection, this
		 *	option isn't available so we buffer output and send output in blocks instead
		 *
		 *	The connection type is static and can't change once the system starts.
		 */
		virtual PvdConnection* connect( PvdCommInStream* inInStream
												, PvdCommOutStream* inOutStream
												, bool inUseErrorCheckingForDebugStream
												, TConnectionFlagsType inConnectionType = defaultConnectionFlags()
												, bool inUseDoubleBufferedOutput = true ) = 0;

		virtual PvdConnection* connect( const char* inFilename
										, bool inCheckAPI
										, TConnectionFlagsType inConnectionType = defaultConnectionFlags() ) = 0;

		/**
		 *	Return the factory representing the current connection to PVD, if any.
		 */
		virtual PvdConnection* getCurrentConnection() = 0;

		/**
		 *	If there is a current connection, disconnect from the factory.
		 */
		virtual void disconnect() = 0;

		/**
		 *	If connected, then flush the entire system of profiling events, memory events
		 *	and the network socket buffer.  User-managed data streams will need to have
		 *	localFlush called on them before this happens in order for their data to be
		 *	flushed.  The reason this is here as well as the connection is that the 
		 *	connection doesn't own the mutex that protects most of the memory event
		 *	system.
		 */
		virtual void flushMemoryEvents() = 0;

		virtual void release() = 0;

		template<typename TNetworkStreamType>
		PvdConnection* connect( const char* inHost
										, int inPort
										, unsigned int inTimeoutInMilliseconds
										, bool inCheckAPI
										, TConnectionFlagsType inConnectionType = defaultConnectionFlags() )
		{
			PvdCommInStream* theInStream;
			PvdCommOutStream* theOutStream;
			if ( TNetworkStreamType().connect( inHost, inPort, inTimeoutInMilliseconds, theInStream, theOutStream ) )
				return connect( theInStream, theOutStream, inCheckAPI, inConnectionType );
			return NULL;
		}
	};
}

#endif

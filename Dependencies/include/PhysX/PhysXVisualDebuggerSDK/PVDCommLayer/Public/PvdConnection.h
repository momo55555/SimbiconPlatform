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

#ifndef PVD_PVDCONNECTIONFACTORY_H
#define PVD_PVDCONNECTIONFACTORY_H
#include "PxFlags.h"
#include "PvdDataStream.h"
#include "PxProfileZoneManager.h"
#include "PvdConnectionType.h"
#include "PxBroadcastingAllocator.h"

namespace physx
{
	class PxProfileEventBufferClientManager;
	class PxProfileNameProvider;
}

namespace PVD //PhysX Visual Debugger
{
	class PvdConnectionPropertyHandler;
	class PvdDataStream;

	struct PvdConnectionState
	{
		enum Enum
		{
			Unknown = 0,
			Recording,
			Monitoring,
			Paused,
		};
	};

	//Threadsafe class.  Connections created are not threadsafe, however.
	//They will all share the network layer's connection.
	class PvdConnection : public PvdQueryInterface
						, public PvdRefCounted
	{
	protected:
		virtual ~PvdConnection(){}
	public:
		//Register a property handler for a given instance.  This handler will be called from the read
		//thread when PVD requestes a property to be changed.
		virtual void registerPropertyHandler( PxU64 inInstance, PvdConnectionPropertyHandler* inHandler ) = 0;

		//May actively change during debugging.
		//Getting this variable may block until the read thread 
		//is disconnected or releases the connection state mutex.
		virtual PvdConnectionState::Enum getConnectionState() = 0;
		//gets the connection state which will block if the system is paused.
		//checks the connection for errors and disconnects if there are any.
		virtual void checkConnection() = 0;
		//Will currently never change during debugging
		virtual TConnectionFlagsType getConnectionType() = 0;
		virtual PvdDataStream* createDataStream() = 0;

		virtual bool isConnected() = 0;
		virtual void disconnect() = 0;
	};

	class InternalPvdConnection : public PvdConnection
								, public PxAllocationListener //acts as an event sync for memory events.
								, public physx::PxProfileZoneHandler //acts as an event sync for profile events
	{
	protected:
		virtual ~InternalPvdConnection(){}
	public:
		//Do not ever call this unless you have whatever mutex is protected the allocation stream.
		virtual void flushMemoryEvents() = 0;
	};
}
 
#endif

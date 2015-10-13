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

#ifndef PVD_CONNECTION_PHYSX_MEMORY_EVENT_CLIENT_H
#define PVD_CONNECTION_PHYSX_MEMORY_EVENT_CLIENT_H

#include "PxProfileBase.h"
#include "PxProfileEventNames.h"
#include "PxProfileEventBufferClient.h"
#include "PxProfileEventBufferClientManager.h"
#include "PxProfileMemoryEventTypes.h"
#include "PVDCommLayerValue.h"
#include "PvdDataStream.h"
#include "PvdConnectionPhysXProfilingClient.h"
#include "PsFoundation.h"

namespace PVD
{
	struct PvdConnectionPhysXMemoryEventClient
		: public physx::PxProfileEventBufferClient
	{
		PvdDataStream*								mStream;
		physx::PxProfileMemoryEventBuffer*			mBuffer;

		static const char* getProfilingNamespace() { return PvdConnectionPhysXProfilingClient::getProfilingNamespace(); }
		
		PxU64 getInstanceName() { return PX_PROFILE_POINTER_TO_U64( this ); }

		//We assume the profile client has registered our necessary classes and properties.
		PvdConnectionPhysXMemoryEventClient( PvdDataStream* inConn, PxU32 inMemoryBufferCacheSize = 0x1000 )
			: mStream( inConn )
			, mBuffer(&physx::PxProfileMemoryEventBuffer::createMemoryEventBuffer( &Foundation::getInstance(), inMemoryBufferCacheSize )) 
		{
			mStream->addRef();
			mStream->setNamespace( getProfilingNamespace() );
			mStream->createInstance( ProfileClientClassNames::MemoryEventStream,  getInstanceName(), EInstanceUIFlags::TopLevel );
			mBuffer->addClient( *this );
		}
		
		virtual ~PvdConnectionPhysXMemoryEventClient()
		{
			mStream->release();
			mStream = NULL;
			mBuffer->release();
			mBuffer = NULL;
		}

		virtual void handleBufferFlush( const PxU8* inData, PxU32 inLength )
		{
			if ( mStream != NULL )
				mStream->setPropertyValue( getInstanceName(), ProfileClientPropertyNames::MemoryEventData, PvdCommLayerValue( createStreamUpdate( inData, inLength ) ) );
		}

		virtual void handleClientRemoved()
		{
			if ( mStream != NULL )
				mStream->release();
			mStream = NULL;
		}
	};
}

#endif

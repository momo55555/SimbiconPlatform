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

#ifndef PVD_CONNECTION_PHYSX_PROFILING_CLIENT_H
#define PVD_CONNECTION_PHYSX_PROFILING_CLIENT_H
#include "PxProfileBase.h"
#include "PxProfileEventNames.h"
#include "PxProfileEventBufferClient.h"
#include "PxProfileEventBufferClientManager.h"
#include "PxProfileZone.h"
#include "PVDCommLayerValue.h"
#include "PvdDataStream.h"

namespace PVD
{
	using namespace physx;

	struct ProfileClientClassNames
	{
		enum Enum
		{
			NoClassName = 0,
			SDK,
			Event,
			MemoryEventStream,
		};
	};

	struct ProfileClientPropertyNames
	{
		enum Enum
		{
			NoPropertyName = 0,
			Name,
			Value,
			CompileTimeEnabled,
			Enabled,
			ProfileData,
			MemoryEventData,
		};
	};



	struct PvdConnectionPhysXProfilingClient : public physx::PxProfileZoneClient
	{
		PvdDataStream*						mConnection;
		PxProfileZone*						mSDK;
		PxU64								mTopLevelProfileObjectName;

		static const char* getProfilingNamespace() { return "physx.profiling"; }
		static const char* getMemoryStreamNamespace() { return "physx.profiling.memory"; }
		
		static void registerProfilingClassNames( PvdDataStream* inConnection )
		{
			inConnection->pushNamespace();
			inConnection->setNamespace( getProfilingNamespace() );
#define REGISTER_PROFILE_CLASS( name ) inConnection->createClass( #name, ProfileClientClassNames::name )
			REGISTER_PROFILE_CLASS( SDK );
			REGISTER_PROFILE_CLASS( Event );
			REGISTER_PROFILE_CLASS( MemoryEventStream );
#undef REGISTER_PROFILE_CLASS

#define REGISTER_PROFILE_ARRAY_PROPERTY( clsName, propName, arrayClsName ) \
	inConnection->defineArrayProperty( ProfileClientClassNames::clsName, #propName, ProfileClientClassNames::arrayClsName, ProfileClientPropertyNames::propName )
#define REGISTER_PROFILE_PROPERTY( clsName, propName, propType ) \
	inConnection->defineProperty( ProfileClientClassNames::clsName, #propName, "", PvdCommLayerDatatype::propType, ProfileClientPropertyNames::propName );

			REGISTER_PROFILE_PROPERTY( SDK, Name, String );
			inConnection->defineProperty( ProfileClientClassNames::SDK, "ProfileData", getProfilingNamespace(), PvdCommLayerDatatype::Stream, ProfileClientPropertyNames::ProfileData );
			REGISTER_PROFILE_PROPERTY( Event, Name, String );
			REGISTER_PROFILE_PROPERTY( Event, Value, U32 );
			REGISTER_PROFILE_PROPERTY( Event, Enabled, Boolean );
			REGISTER_PROFILE_PROPERTY( Event, CompileTimeEnabled, Boolean );

			inConnection->defineProperty( ProfileClientClassNames::MemoryEventStream, "MemoryEventData", getMemoryStreamNamespace(), PvdCommLayerDatatype::Stream, ProfileClientPropertyNames::MemoryEventData );

#undef REGISTER_PROFILE_PROPERTY
#undef REGISTER_PROFILE_ARRAY_PROPERTY
			inConnection->popNamespace();
		}

		PxU64 getInstanceName() { return PX_PROFILE_POINTER_TO_U64( this ); }
		PxU64 getInstanceName( const PxProfileEventName& inId ) { return PX_PROFILE_POINTER_TO_U64( inId.mName ); }

		PvdConnectionPhysXProfilingClient( PvdDataStream* inConnection
										, PxProfileZone* inSDK )
			: mConnection( inConnection )
			, mSDK( inSDK )
		{
			mConnection->setNamespace( getProfilingNamespace() );

			//Send over the static data the makes up the description of the profiling
			//dataset.
			PxProfileNames theNames( mSDK->getProfileNames() );
			//Use an offset from the connection for the handles to the base object.
			mTopLevelProfileObjectName = getInstanceName();
			mConnection->createInstance( ProfileClientClassNames::SDK, mTopLevelProfileObjectName, EInstanceUIFlags::TopLevel );
			mConnection->setPropertyValue( mTopLevelProfileObjectName, ProfileClientPropertyNames::Name, PvdCommLayerValue( mSDK->getName() ) );
			setupNewEvents( theNames.mEvents, theNames.mEventCount );
			sendEventInformation( theNames.mEvents, theNames.mEventCount );
			mSDK->addClient( *this );
		}

		inline void setupNewEvents( const PxProfileEventName* inNames, PxU32 inLen )
		{
			for ( PxU32 idx = 0; idx < inLen; ++idx )
			{
				const PxProfileEventName& theEventName( inNames[idx] );
				PxU64 theName( getInstanceName( theEventName ) );
				mConnection->createInstance( ProfileClientClassNames::Event, theName );
				mConnection->addChild( mTopLevelProfileObjectName, theName );
			}
		}

		inline void sendEventInformation( const PxProfileEventName* inNames, PxU32 inLen )
		{
			PxU32 eventProperties[] = { ProfileClientPropertyNames::Name, ProfileClientPropertyNames::Value, ProfileClientPropertyNames::CompileTimeEnabled, ProfileClientPropertyNames::Enabled };
			PvdCommLayerDatatype eventPropTypes[] = { PvdCommLayerDatatype::String, PvdCommLayerDatatype::U32, PvdCommLayerDatatype::Boolean, PvdCommLayerDatatype::Boolean };
			mConnection->beginPropertyBlock( ProfileClientClassNames::Event, eventProperties, eventPropTypes, sizeof( eventProperties )/ sizeof( *eventProperties ) );
			PvdCommLayerValue theValues[4];
			for ( PxU32 idx = 0; idx < inLen; ++idx )
			{
				const PxProfileEventName& theName = inNames[idx];
				theValues[0] = PvdCommLayerValue( theName.mName );
				theValues[1] = PvdCommLayerValue( (PxU32)theName.mEventId.mEventId );
				theValues[2] = PvdCommLayerValue( theName.mEventId.mCompileTimeEnabled );
				theValues[3] = PvdCommLayerValue( true );
				mConnection->sendPropertyBlock( getInstanceName( theName ), theValues );
			}
			mConnection->endPropertyBlock();
		}

		virtual ~PvdConnectionPhysXProfilingClient()
		{
			removeFromSDK();
		}

		virtual void removeFromSDK()
		{
			if ( mSDK != NULL )
				mSDK->removeClient( *this );
			mSDK = NULL;
			handleClientRemoved();
		}

		virtual void handleClientRemoved()
		{
			if ( mConnection != NULL )
				mConnection->release();
			mConnection = NULL;
			mSDK = NULL;
		}

		virtual void handleBufferFlush( const PxU8* inData, PxU32 inLength )
		{
			if ( mConnection != NULL )
				mConnection->setPropertyValue( getInstanceName(), ProfileClientPropertyNames::ProfileData, PvdCommLayerValue( createStreamUpdate( inData, inLength ) ) );
		}

		virtual void handleEventAdded( const PxProfileEventName& inName )
		{
			setupNewEvents( &inName, 1 );
			sendEventInformation( &inName, 1 );
		}

		virtual physx::PxProfileZone* getSDK() { return mSDK; }
	};
}
#endif

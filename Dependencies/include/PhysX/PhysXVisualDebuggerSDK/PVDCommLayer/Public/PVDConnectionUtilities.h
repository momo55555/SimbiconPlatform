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

#ifndef PVD_PVDCONNECTIONUTILITIES_H
#define PVD_PVDCONNECTIONUTILITIES_H

#include "PvdDataStream.h"
#include "PxVec3.h"

namespace PVD
{

	namespace PvdConnectionUtilities
	{
	//Helper functions for minimal code when implementing PVD communication bindings.

		inline PvdCommLayerError createInstance( PvdDataStream* inConnection, PxU64 inInstanceKey, const char* inTypeName, PVD::EInstanceUIFlags inFlags = PVD::EInstanceUIFlags::None )
		{
			PxU32 theClassKey = HashFunction( inTypeName ); //Use the pointer address as the key.
			PvdCommLayerError theError = inConnection->createClass( inTypeName, theClassKey );
			if ( theError == PvdCommLayerError::None )
				theError = inConnection->createInstance( theClassKey, inInstanceKey, inFlags );
			return theError;
		}

		inline PvdCommLayerError defineProperty( PvdDataStream* inConnection, PxU64 inInstanceKey, const char* inName, PvdCommLayerDatatype inDatatype )
		{
			PvdCommLayerError theError = PvdCommLayerError::None;
			PxU32 thePropertyKey = HashFunction( inName );
			return inConnection->definePropertyOnInstance( inInstanceKey, inName, NULL, inDatatype, thePropertyKey );
		}

		inline PvdCommLayerError setPropertyValue( PvdDataStream* inConnection, PxU64 inInstanceKey, const char* inName, const PvdCommLayerValue& inValue, bool inCreate = true )
		{
			PvdCommLayerError theError = PvdCommLayerError::None;
			if ( inCreate )
				theError = defineProperty( inConnection, inInstanceKey, inName, inValue.getDatatype() );
			if ( theError == PvdCommLayerError::None )
			{
				PxU32 thePropertyKey = HashFunction( inName );
				theError = inConnection->setPropertyValue( inInstanceKey, thePropertyKey, inValue );
			}
			return theError;
		}

		struct CameraClassKeys
		{
			enum Enum
			{
				Origin = 1,
				Target,
				Up,
				Name,
			};
		};

		inline void defineCamera( PvdDataStream* inConnection )
		{
			PxU32 theClassKey = HashFunction( "Camera" );
			inConnection->pushNamespace();
			inConnection->setNamespace( "" );
			inConnection->createClass( "Camera", theClassKey );

			inConnection->defineProperty( theClassKey, "Origin", "", PvdCommLayerDatatype::Float3, CameraClassKeys::Origin);
			inConnection->defineProperty( theClassKey, "Target", "", PvdCommLayerDatatype::Float3, CameraClassKeys::Target );
			inConnection->defineProperty( theClassKey, "Up", "", PvdCommLayerDatatype::Float3, CameraClassKeys::Up );
			inConnection->defineProperty( theClassKey, "Name", "", PvdCommLayerDatatype::String, CameraClassKeys::Name );
			inConnection->popNamespace();
		}

		inline void createCamera( PvdDataStream* inConnection, PxU64 inInstanceKey, const char* inName )
		{
			inConnection->pushNamespace();
			inConnection->setNamespace( "" );
			PxU32 theClassKey = HashFunction( "Camera" );
			inConnection->createInstance( theClassKey, inInstanceKey, EInstanceUIFlags::None );
			inConnection->setPropertyValue( inInstanceKey, CameraClassKeys::Name, PVD::createString( inName ) );
			inConnection->popNamespace();
		}

		inline PvdCommLayerValue ToPvd( const physx::pubfnd::PxVec3& inVec3 ) { return createFloat3( inVec3.x, inVec3.y, inVec3.z ); }

		inline void updateCamera( PvdDataStream* inConnection, physx::pubfnd::PxU64 inInstanceKey, const physx::pubfnd::PxVec3& inOrigin, const physx::pubfnd::PxVec3& inUp, const physx::pubfnd::PxVec3& inTarget )
		{
			inConnection->setPropertyValue( inInstanceKey, CameraClassKeys::Origin, ToPvd( inOrigin ) );
			inConnection->setPropertyValue( inInstanceKey, CameraClassKeys::Up, ToPvd( inUp ) );
			inConnection->setPropertyValue( inInstanceKey, CameraClassKeys::Target, ToPvd( inTarget ) );
		}
	};
}

#endif
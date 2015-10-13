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

#ifndef PVD_PVDCONNECTIONSTREAMWRITERBASE_H
#define PVD_PVDCONNECTIONSTREAMWRITERBASE_H

#include "PVDConnectionEvents.h"
#include "PvdRenderInterfaceToCommands.h"

namespace PVD
{
	template<typename TSerializer>
	struct SendEventOperator
	{
		TSerializer* mSerializer;
		SendEventOperator( TSerializer* inSerializer )
			: mSerializer( inSerializer )
		{
		}
		template<typename TDataType>
		inline PvdCommLayerError operator()( const TDataType& inType )
		{
			return mSerializer->sendEvent( inType );
		}
	};
	/**
	 *	Transforms the base connection API into a base stream of events.
	 */
	template<typename TSerializer
			, typename TDatatypeContainer>
	struct PvdDataStreamWriterBase : public PvdRenderInterfaceToCommands<PvdCommLayerError, SendEventOperator<TSerializer> > 
	{
		//Batching information
		TDatatypeContainer				mDatatypes;
		PxU32							mPropertyCount;
		TSerializer*					mSerializer;

		PvdDataStreamWriterBase( TSerializer* inType ) 
			: PvdRenderInterfaceToCommands<PvdCommLayerError, SendEventOperator<TSerializer> >( SendEventOperator<TSerializer>( inType ) )
			, mPropertyCount( 0 )
			, mSerializer( inType )
		{}

		inline PvdCommLayerError sendStreamInitialization()
		{
			return mSerializer->sendEvent( createStreamInitialization() );
		}

		inline PvdCommLayerError setNamespace( const char* inNamespace )
		{
			return mSerializer->sendEvent( createSetNamespace( inNamespace ) );
		}

		inline PvdCommLayerError pushNamespace()
		{
			return mSerializer->sendEvent( createPushNamespace() );
		}

		inline PvdCommLayerError popNamespace()
		{
			return mSerializer->sendEvent( createPopNamespace() );
		}
		
		inline PvdCommLayerError createClass( const char* inName, PxU32 inKey )
		{
			return mSerializer->sendEvent( createCreateClass( inName, inKey ) );
		}

		inline PvdCommLayerError deriveClass( PxU32 inParentKey, PxU32 inChildKey )
		{
			return mSerializer->sendEvent( createDeriveClass( inParentKey, inChildKey ) );
		}

		inline PvdCommLayerError defineProperty( PxU32 inClass
													, const char* inName
													, const char* inSemantic
													, PvdCommLayerDatatype inDatatype
													, PxU32 inKey )
		{
			return mSerializer->sendEvent( createDefineProperty( inClass, inName, inSemantic, inDatatype, inKey ) );
		}
		inline PvdCommLayerError definePropertyOnInstance( PxU64 inInstanceId
															, const char* inName
															, const char* inSemantic
															, PvdCommLayerDatatype inDatatype
															, PxU32 inKey )
		{
			return mSerializer->sendEvent( createDefinePropertyOnInstance( inInstanceId, inName, inSemantic, inDatatype, inKey ) );
		}

		inline PvdCommLayerError definePropertyStruct( PxU32 inStructKey
															, PxU32 inClass
															, PxU32 inStructByteSize
															, const PropertyStructEntry* inEntries
															, PxU32 inEntryCount )
		{
			return mSerializer->sendEvent( createRegisterPropertyStruct( inStructKey, inClass, inStructByteSize, inEntryCount, const_cast<PropertyStructEntry*>( inEntries ) ) );
		}

		inline PvdCommLayerError defineArrayProperty( PxU32 inClass
													, const char* inName
													, PxU32 inArrayClass
													, PxU32 inKey )
		{
			return mSerializer->sendEvent( createDefineArrayProperty( inClass, inKey, inArrayClass, inName ) );
		}
		inline PvdCommLayerError defineBitflagNames( PxU32 inClass
														, PxU32 inPropertyName
														, const NamedValueDefinition* inDefinitions
														, PxU32 inDefinitionLength )
		{
			return mSerializer->sendEvent( createDefineBitflagNames( inClass, inPropertyName, inDefinitions, inDefinitionLength ) );
		}
		inline PvdCommLayerError defineEnumerationNames( PxU32 inClass
														, PxU32 inPropertyName
														, const NamedValueDefinition* inDefinitions
														, PxU32 inDefinitionLength )
		{
			return mSerializer->sendEvent( createDefineEnumerationNames( inClass, inPropertyName, inDefinitions, inDefinitionLength ) );
		}
		inline PvdCommLayerError createInstance( PxU32 inClass, PxU64 inInstanceId, PxU32 inFlags )
		{
			return mSerializer->sendEvent( createCreateInstance( inClass, inInstanceId, inFlags ) );
		}
		inline PvdCommLayerError setPropertyValue( PxU64 inInstance, PxU32 inProperty, const PvdCommLayerValue& inValue )
		{
			return mSerializer->sendEvent( createSetPropertyValue( inInstance, inProperty, inValue ) );
		}
		inline PvdCommLayerError setPropertyValue( PxU64 inInstance, PxU32 inProperty, const PvdCommLayerMediumValue& inValue )
		{
			return mSerializer->sendEvent( createSetPropertyMediumValue( inInstance, inProperty, inValue ) );
		}
		
		inline PvdCommLayerError setPropertyValue( PxU64 inInstance, PxU32 inProperty, const PvdCommLayerSmallValue& inValue )
		{
			return mSerializer->sendEvent( createSetPropertySmallValue( inInstance, inProperty, inValue ) );
		}

		inline void setDatatypes( const PvdCommLayerDatatype* inDatatypes, PxU32 inPropertyCount )
		{
			mDatatypes.clear();
			for ( PxU32 idx = 0; idx < inPropertyCount; ++idx )
				mDatatypes.pushBack( inDatatypes[idx] );
			mPropertyCount = inPropertyCount;
		}
		inline PvdCommLayerError beginPropertyBlock( PxU32 inClass
														, const PxU32* inProperties
														, const PvdCommLayerDatatype*	inDatatypes
														, PxU32 inPropertyCount )
		{
			setDatatypes( inDatatypes, inPropertyCount );
			return mSerializer->sendEvent( createBeginPropertyBlock( inClass, inProperties, inDatatypes, inPropertyCount ) );
		}

		inline PvdCommLayerError PropertyBlock( PxU64 inInstance, const PvdCommLayerData* inValues )
		{
			return mSerializer->sendEvent( createSendPropertyBlock( mDatatypes.begin(), mPropertyCount, inInstance, inValues ) );
		}
		inline PvdCommLayerError PropertyBlock( PxU64 inInstance, const PxU8* inData, PxU32 inDataLen, const PvdCommLayerData* inValues  )
		{
			return mSerializer->sendEvent( createRawPropertyBlock( mDatatypes.begin(), mPropertyCount, inInstance, inData, inDataLen, inValues ) );
		}
		inline PvdCommLayerError endPropertyBlock()
		{
			return mSerializer->sendEvent( createEndPropertyBlock() );
		}
		inline PvdCommLayerError sendPropertyStruct( PxU64 inInstance
													, PxU32 inStructId
													, PxU32 inClass
													, PropertyStructEntry* inEntries
													, PxU32 inEntryCount
													, PxU32* inStringOffsets
													, PxU32 inStringOffsetCount
													, const PxU8* inData
													, PxU32 inDataLen )
		{
			return mSerializer->sendEvent( CreateSendPropertyStruct( inInstance, inStructId, inClass, inEntries, inEntryCount, inStringOffsets, inStringOffsetCount, inData, inDataLen ) );
		}
		inline PvdCommLayerError addChild( PxU64 inParent, PxU64 inChild )
		{
			return mSerializer->sendEvent( createAddChild( inParent, inChild ) );
		}
		inline PvdCommLayerError removeChild( PxU64 inParent, PxU64 inChild )
		{
			return mSerializer->sendEvent( createRemoveChild( inParent, inChild ) );
		}
		inline PvdCommLayerError removeAllChildren( PxU64 inInstanceId, PxU32 inChildClass )
		{
			return mSerializer->sendEvent( createRemoveAllChildren( inInstanceId, inChildClass ) );
		}
		inline PvdCommLayerError destroyInstance( PxU64 inInstance )
		{
			return mSerializer->sendEvent( createDestroyInstance( inInstance ) );
		}
		inline PvdCommLayerError beginArrayBlock( PxU32 inClass
														, PxU64 inInstance
														, const PxU32* inProperties
														, const PvdCommLayerDatatype*	inDatatypes
														, PxU32 inPropertyCount )
		{
			setDatatypes( inDatatypes, inPropertyCount );
			return mSerializer->sendEvent( createBeginArrayBlock( inClass, inInstance, inProperties, inDatatypes, inPropertyCount ) );
		}
		
		inline PvdCommLayerError beginArrayPropertyBlock( PxU64 inInstance
														, PxU32 inProperty
														, const PxU32* inProperties
														, const PvdCommLayerDatatype*	inDatatypes
														, PxU32 inPropertyCount )
		{
			setDatatypes( inDatatypes, inPropertyCount );
			return mSerializer->sendEvent( createBeginArrayPropertyBlock( inInstance, inProperty, inProperties, inDatatypes, inPropertyCount ) );
		}
		inline PvdCommLayerError ArrayObject( const PvdCommLayerData* inValues )
		{
			return mSerializer->sendEvent( createArrayObject( mDatatypes.begin(), inValues, mPropertyCount ) );
		}
		inline PvdCommLayerError ArrayObjects( const PxU8* inData, PxU32 inStride, PxU32 inCount )
		{
			return mSerializer->sendEvent( createArrayObjects( mDatatypes.begin(), mPropertyCount, inStride, inCount, inData ) );
		}
		inline PvdCommLayerError endArrayBlock()
		{
			return mSerializer->sendEvent( createEndArrayBlock() );
		}
		inline PvdCommLayerError endArrayPropertyBlock()
		{
			return mSerializer->sendEvent( createEndArrayPropertyBlock() );
		}
		inline PvdCommLayerError NamedEvent( const char* inName, PxU32 inPayload )
		{
			return mSerializer->sendEvent( createNamedEvent( inName, inPayload ) );
		}
		inline PvdCommLayerError Error( const char* inType, const char* inMessage, const char* inFile, PxU32 inLine )
		{
			return mSerializer->sendEvent( createError( inType, inMessage, inFile, inLine ) );
		}
		inline PvdCommLayerError Warning( const char* inType, const char* inMessage, const char* inFile, PxU32 inLine )
		{
			return mSerializer->sendEvent( createWarning( inType, inMessage, inFile, inLine ) );
		}
		inline PvdCommLayerError beginSection( const char* inType, PxU64 inPayload )
		{
			return mSerializer->sendEvent( createBeginSection( inType, inPayload ) );
		}
		inline PvdCommLayerError endSection( const char* inType, PxU64 inPayload )
		{
			return mSerializer->sendEvent( createEndSection( inType, inPayload ) );
		}
		inline PvdCommLayerError namedEventWithInstance( PxU64 inInstanceId, const char* inEventName )
		{
			return mSerializer->sendEvent( createNamedEventWithInstance( inEventName, inInstanceId ) );
		}
		inline PvdCommLayerError disconnect()
		{
			return mSerializer->sendEvent( createDisconnect() );
		}
	};
}
#endif
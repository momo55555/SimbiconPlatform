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

#ifndef PX_META_DATA_SEND_PROPERTIES_H
#define PX_META_DATA_SEND_PROPERTIES_H

#if PX_SUPPORT_VISUAL_DEBUGGER

#include "PvdMetaDataPropertyVisitor.h"
#include "../src/PxProfileMemoryBuffer.h"
#include "PvdDataStreamHelpers.h"

namespace physx
{
namespace Pvd
{
struct PropertyBlockSetup
{
	PvdBeginPropertyBlockHelper& mHelper;

	PropertyBlockSetup( PvdBeginPropertyBlockHelper& inHelper )
		: mHelper( inHelper )
	{
	}

	PropertyBlockSetup( const PropertyBlockSetup& other )
		: mHelper( other.mHelper )
	{
	}

	void pushName( const char* ) {}
	void pushBracketedName( const char* ) {}
	void popName() {}

	void addProperty( PxU32 inKey, PvdCommLayerDatatype inType )
	{
		mHelper.addProperty( inKey, inType );
	}

	template<typename TAccessorType>
	void simpleProperty( PxU32 key, TAccessorType& inProp )
	{
		typedef typename TAccessorType::prop_type TPropertyType;
		addProperty(key, getDatatypeForType<TPropertyType>());
	}
	
	template<typename TAccessorType>
	void enumProperty( PxU32 key, TAccessorType& inProp, const PxU32ToName* inConversions )
	{
		addProperty(key, PvdCommLayerDatatype::EnumerationValue);
	}

	template<typename TAccessorType>
	void flagsProperty( PxU32 key, const TAccessorType& inAccessor, const PxU32ToName* inConversions )
	{
		addProperty(key, PvdCommLayerDatatype::Bitflag);
	}

	template<typename TAccessorType, typename TInfoType>
	void complexProperty( PxU32* key, const TAccessorType& inAccessor, TInfoType& inInfo )
	{
		PxU32 theOffset = inAccessor.mOffset;
		inInfo.visitBaseProperties( makePvdPropertyFilter( *this, key, &theOffset ) );
		inInfo.visitInstanceProperties( makePvdPropertyFilter( *this, key, &theOffset ) );
	}
};

template<typename TVisitFunc>
inline void SetupPropertyBlock( PvdDataStream* inStream, PxU32 inClass, TVisitFunc visitFunc )
{
	PvdBeginPropertyBlockHelper& theHelper( inStream->getBeginPropertyBlockHelper() );
	PropertyBlockSetup theSetup( theHelper );
	visitFunc( theSetup );
	theHelper.beginPropertyBlock( inClass );
}

template<typename TSourceType>
struct PvdClassInfoSend
{
	PvdSendPropertyBlockHelper& mHelper;
	const TSourceType* mSourceType;
	PvdClassInfoSend( PvdSendPropertyBlockHelper& inHelper, const TSourceType* inSourceType )
		: mHelper( inHelper )
		, mSourceType( inSourceType )
	{
	}

	PvdClassInfoSend( const PvdClassInfoSend& inOther )
		: mHelper( inOther.mHelper )
		, mSourceType( inOther.mSourceType )
	{
	}

	void pushName( const char* ) {}
	void pushBracketedName( const char* ) {}
	void popName() {}

#define DEFINE_BASIC_ADD_PROPERTY( datatype ) void addProperty( datatype inValue ) { mHelper.addValue( inValue ); }

	DEFINE_BASIC_ADD_PROPERTY( PxU8 );
	DEFINE_BASIC_ADD_PROPERTY( PxU16 );
	DEFINE_BASIC_ADD_PROPERTY( PxU32 );
	DEFINE_BASIC_ADD_PROPERTY( PxU64 );
	DEFINE_BASIC_ADD_PROPERTY( PxI8 );
	DEFINE_BASIC_ADD_PROPERTY( PxI16 );
	DEFINE_BASIC_ADD_PROPERTY( PxI32 );
	DEFINE_BASIC_ADD_PROPERTY( PxI64 );
	DEFINE_BASIC_ADD_PROPERTY( PxF32 );
	DEFINE_BASIC_ADD_PROPERTY( PxF64 );
	DEFINE_BASIC_ADD_PROPERTY( PxVec3 );
	DEFINE_BASIC_ADD_PROPERTY( bool );
	DEFINE_BASIC_ADD_PROPERTY( const PxQuat& );
	DEFINE_BASIC_ADD_PROPERTY( const PxTransform& );
	DEFINE_BASIC_ADD_PROPERTY( const PxBounds3& );
	DEFINE_BASIC_ADD_PROPERTY( const char* );

#undef DEFINE_BASIC_ADD_PROPERTY
	
	void addProperty( const PxFilterData& inValue ) { mHelper.addValue( createFilterData( inValue.word0, inValue.word1, inValue.word2, inValue.word3 ) ); }
	void addProperty( const PxMetaDataPlane& inValue ) { mHelper.addValue( createPlane( inValue.normal.x, inValue.normal.y, inValue.normal.z, inValue.distance ) ); }
	void addProperty( const PxRigidActor* inValue ) { mHelper.addValue( createInstanceId( reinterpret_cast<PxU64>( static_cast<const PxActor*>( inValue ) ) ) ); }

	
	template<typename TAccessorType>
	void simpleProperty( PxU32 key, TAccessorType& inProp )
	{
		addProperty( inProp.get( mSourceType ) );
	}
	
	template<typename TAccessorType>
	void enumProperty( PxU32 key, TAccessorType& inProp, const PxU32ToName* inConversions )
	{
		addProperty( inProp.get( mSourceType ) );
	}

	template<typename TAccessorType>
	void flagsProperty( PxU32 key, const TAccessorType& inProp, const PxU32ToName* inConversions )
	{
		addProperty( static_cast<PxU32>( inProp.get( mSourceType ) ) );
	}

	template<typename TAccessorType, typename TInfoType>
	void complexProperty( PxU32* key, const TAccessorType& inAccessor, TInfoType& inInfo )
	{
		typedef typename TAccessorType::prop_type TPropertyType;
		TPropertyType theValue = inAccessor.get( mSourceType );
		PvdClassInfoSend<TPropertyType> theSender( mHelper, &theValue );
		PxU32 theOffset = inAccessor.mOffset;
		inInfo.visitBaseProperties( makePvdPropertyFilter( theSender, key, &theOffset ) );
		inInfo.visitInstanceProperties( makePvdPropertyFilter( theSender, key, &theOffset ) );
	}
};

template<typename TSourceType, typename TVisitFunc>
inline void SendPropertyBlock( PvdDataStream* inStream, PxU64 inInstance, const TSourceType* inSource, TVisitFunc inFunc )
{
	PvdSendPropertyBlockHelper& theHelper( inStream->getSendPropertyBlockHelper() );
	PvdClassInfoSend<TSourceType> theSender( theHelper, inSource );
	inFunc( theSender );
	theHelper.sendPropertyBlock( inInstance );
}

template<typename TDataType>
inline void SendSinglePropertyBlock( PvdDataStream* inStream, PxU32 inClassId, PxU64 inInstance, const TDataType* inSource )
{
	SetupPropertyBlock( inStream, inClassId, visitAllPvdProperties<TDataType,PropertyBlockSetup> );
	SendPropertyBlock( inStream, inInstance, inSource, visitAllPvdProperties<TDataType, PvdClassInfoSend<TDataType> > );
	inStream->endPropertyBlock();
}

}

}

#endif
#endif

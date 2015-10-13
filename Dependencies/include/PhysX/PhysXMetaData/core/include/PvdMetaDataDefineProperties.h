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

#ifndef PVD_META_DATA_DEFINE_PROPERTIES_H
#define PVD_META_DATA_DEFINE_PROPERTIES_H

#if PX_SUPPORT_VISUAL_DEBUGGER

#include "PvdMetaDataPropertyVisitor.h"
#include "PvdDataStreamHelpers.h"
#include "PvdDataStream.h"

namespace physx
{
namespace Pvd
{

struct PvdClassInfoDefine
{
	PvdPropertyDefinitionHelper& mHelper;
	PxU32 mClassKey;

	PvdClassInfoDefine( PvdPropertyDefinitionHelper& info, PxU32 inKey )
		: mHelper( info )
		, mClassKey( inKey ) { }

	PvdClassInfoDefine( const PvdClassInfoDefine& other )
		: mHelper( other.mHelper )
		, mClassKey( other.mClassKey )
	{
	}

	void defineProperty( PvdCommLayerDatatype inDtype, PxU32 inKey )
	{
		mHelper.defineProperty( mClassKey, "", inDtype, inKey ); 
	}

	void pushName( const char* inName )
	{
		mHelper.pushName( inName );
	}
	
	void pushBracketedName( const char* inName) 
	{
		mHelper.pushBracketedName( inName );
	}

	void popName()
	{
		mHelper.popName();
	}

	inline void defineNameValueDefs( const PxU32ToName* theConversions )
	{
		while( theConversions->mName != NULL )
		{
			mHelper.addNamedValueDefinition( theConversions->mName, theConversions->mValue );
			++theConversions;
		}
	}

	template<typename TAccessorType>
	void simpleProperty( PxU32 key, TAccessorType& inProp )
	{
		typedef typename TAccessorType::prop_type TPropertyType;
		defineProperty(getDatatypeForType<TPropertyType>(), key );
	}
	
	template<typename TAccessorType>
	void enumProperty( PxU32 key, TAccessorType& inProp, const PxU32ToName* inConversions )
	{
		defineProperty( PvdCommLayerDatatype::EnumerationValue, key);
		defineNameValueDefs( inConversions );
		mHelper.defineEnumerationNames( mClassKey, key );
		typedef typename TAccessorType::prop_type TPropType;
	}

	template<typename TAccessorType>
	void flagsProperty( PxU32 key, const TAccessorType& inAccessor, const PxU32ToName* inConversions )
	{
		defineProperty( PvdCommLayerDatatype::Bitflag, key);
		defineNameValueDefs( inConversions );
		mHelper.defineBitflagNames( mClassKey, key );
		typedef typename TAccessorType::prop_type TPropType;
	}

	template<typename TAccessorType, typename TInfoType>
	void complexProperty( PxU32* key, const TAccessorType& inAccessor, TInfoType& inInfo )
	{
		PxU32 theOffset = inAccessor.mOffset;
		inInfo.visitBaseProperties( makePvdPropertyFilter( *this, key, &theOffset ) );
		inInfo.visitInstanceProperties( makePvdPropertyFilter( *this, key, &theOffset ) );
	}
	
	template<PxU32 TKey, typename TObjectType, typename TPropertyType, PxU32 TEnableFlag>
	void handleBuffer( const PxBufferPropertyInfo<TKey, TObjectType, PxStrideIterator< const TPropertyType >, TEnableFlag>& inProp )
	{
		mHelper.pushName( inProp.mName );
		mHelper.defineArrayProperty( mClassKey, PvdClassForType<TPropertyType>().PvdClass, TKey );
		mHelper.popName();
	}
	
	template<PxU32 TKey, typename TObjectType, typename TEnumType, typename TStorageType, PxU32 TEnableFlag>
	void handleFlagsBuffer( const PxBufferPropertyInfo<TKey, TObjectType, PxStrideIterator<const PxFlags<TEnumType, TStorageType> >, TEnableFlag>& inProp, const PxU32ToName* inConversion )
	{
		mHelper.pushName( inProp.mName );
		mHelper.defineArrayProperty( mClassKey, PvdClassForType<PxFlags<TEnumType, TStorageType> >().PvdClass, TKey );
		mHelper.popName();
	}

	template<PxU32 TKey, typename TObjectType, typename TPropertyType, PxU32 TEnableFlag>
	void handleBuffer( const PxBufferPropertyInfo<TKey, TObjectType, const Array< TPropertyType >&, TEnableFlag>& inProp )
	{
		mHelper.pushName( inProp.mName );
		mHelper.defineArrayProperty( mClassKey, PvdClassForType<TPropertyType>().PvdClass, TKey );
		mHelper.popName();
	}

	template<PxU32 TKey, typename TObjectType, typename TCollectionType>
	void handleCollection( const PxReadOnlyCollectionPropertyInfo<TKey, TObjectType, TCollectionType>& inProp )
	{
		mHelper.pushName( inProp.mName );
		mHelper.defineArrayProperty( mClassKey, PvdClassForType<TCollectionType>().PvdClass, TKey );
		mHelper.popName();
	}
	
	template<PxU32 TKey, typename TObjectType, typename TEnumType>
	void handleCollection( const PxReadOnlyCollectionPropertyInfo<TKey, TObjectType, TEnumType>& inProp, const PxU32ToName* inConversions )
	{
		mHelper.pushName( inProp.mName );
		mHelper.defineArrayProperty( mClassKey, PvdClassForType<TEnumType>().PvdClass, TKey );
		mHelper.popName();
	}
};


struct PvdClassInfoValueStructDefine
{
	PvdPropertyDefinitionHelper& mHelper;

	PvdClassInfoValueStructDefine( PvdPropertyDefinitionHelper& info )
		: mHelper( info )
	{ }

	PvdClassInfoValueStructDefine( const PvdClassInfoValueStructDefine& other )
		: mHelper( other.mHelper )
	{
	}

	void defineValueStructOffset( PxU32 inKey, PvdCommLayerDatatype inDtype, const ValueStructOffsetRecord& inProp )
	{
		if ( inProp.mHasValidOffset )
		{
			if ( inDtype.mDatatype == PvdCommLayerDatatype::ObjectId )
				inDtype.mDatatype = PvdCommLayerDatatype::Pointer;
			mHelper.addStructPropertyEntry( inKey, inDtype, inProp.mOffset );
		}
	}

	void defineValueStructOffset( PxU32 inKey, const ValueStructOffsetRecord& inProp, PxU32 inPropSize )
	{
		PvdCommLayerDatatype type;
		switch( inPropSize )
		{
		case 8: type = PvdCommLayerDatatype::U64; break;
		case 4: type = PvdCommLayerDatatype::U32; break;
		case 2: type = PvdCommLayerDatatype::U16; break;
		default: PX_ASSERT( false ); //fallthrough intentional
		case 1: type = PvdCommLayerDatatype::U8; break;
		}
		defineValueStructOffset( inKey, type, inProp );
	}

	void pushName( const char* )
	{
	}
	
	void pushBracketedName( const char* ) 
	{
	}

	void popName()
	{
	}

	template<typename TAccessorType>
	void simpleProperty( PxU32 key, TAccessorType& inProp )
	{
		typedef typename TAccessorType::prop_type TPropertyType;
		defineValueStructOffset( key, getDatatypeForType<TPropertyType>(), inProp );
	}
	
	template<typename TAccessorType>
	void enumProperty( PxU32 key, TAccessorType& inAccessor, const PxU32ToName* inConversions )
	{
		typedef typename TAccessorType::prop_type TPropType;
		defineValueStructOffset( key, inAccessor, sizeof( TPropType ) );
	}

	template<typename TAccessorType>
	void flagsProperty( PxU32 key, const TAccessorType& inAccessor, const PxU32ToName* inConversions )
	{
		typedef typename TAccessorType::prop_type TPropType;
		defineValueStructOffset( key, inAccessor, sizeof( TPropType ) );
	}

	template<typename TAccessorType, typename TInfoType>
	void complexProperty( PxU32* key, const TAccessorType& inAccessor, TInfoType& inInfo )
	{
		PxU32 theOffset = inAccessor.mOffset;
		inInfo.visitBaseProperties( makePvdPropertyFilter( *this, key, &theOffset ) );
		inInfo.visitInstanceProperties( makePvdPropertyFilter( *this, key, &theOffset ) );
	}

	template<PxU32 TKey, typename TObjectType, typename TCollectionType>
	void handleCollection( const PxReadOnlyCollectionPropertyInfo<TKey, TObjectType, TCollectionType>& prop )
	{
	}
	
	template<PxU32 TKey, typename TObjectType, typename TEnumType>
	void handleCollection( const PxReadOnlyCollectionPropertyInfo<TKey, TObjectType, TEnumType>& prop, const PxU32ToName* inConversions )
	{
	}
};

}

}

#endif
#endif

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

#ifndef PVD_PVD_DATA_STREAM_HELPERS_IMPL_H
#define PVD_PVD_DATA_STREAM_HELPERS_IMPL_H
#include "PVDCommLayerTypes.h"
#include "PsArray.h"
#include "../src/PxProfileMemoryBuffer.h"
#include "PvdDataStream.h"
#include "PvdDataStreamHelpers.h"

namespace PVD
{
	using namespace physx::pubfnd;
	using namespace physx::shdfnd;

	
	class PvdPropertyDefinitionHelperImpl : public PvdPropertyDefinitionHelper
	{
		Array<char>							mNameBuffer;
		Array<PxU32>						mNameStack;
		Array<NamedValueDefinition>			mTempDefinitions;
		Array<PropertyStructEntry>			mPropertyStructEntries;
		PvdDataStream&						mStream;
		
	public:

		PvdPropertyDefinitionHelperImpl( PvdDataStream& inStream )
			: mStream( inStream )
		{
		}

		inline void appendStrToBuffer( const char* str )
		{
			if ( str == NULL )
				return;
			size_t strLen = strlen( str );
			size_t endBufOffset = mNameBuffer.size();
			size_t resizeLen = endBufOffset;
			//account for null
			if ( mNameBuffer.empty() )
				resizeLen += 1;
			else
				endBufOffset -=1;

			mNameBuffer.resize( static_cast<PxU32>( resizeLen + strLen ) );
			char* endPtr = mNameBuffer.begin() + endBufOffset;
			memcpy( endPtr, str, strLen );
		}

		virtual void pushName( const char* nm, const char* appender = "." ) 
		{ 
			size_t nameBufLen = mNameBuffer.size();
			mNameStack.pushBack( static_cast<PxU32>( nameBufLen ) );
			if ( mNameBuffer.empty() == false )
				appendStrToBuffer( appender );
			appendStrToBuffer( nm );
			mNameBuffer.back() = 0;
		}
		
		virtual void pushBracketedName( const char* inName, const char* leftBracket = "[", const char* rightBracket = "]" )
		{
			size_t nameBufLen = mNameBuffer.size();
			mNameStack.pushBack( static_cast<PxU32>( nameBufLen ) );
			appendStrToBuffer( leftBracket );
			appendStrToBuffer( inName );
			appendStrToBuffer( rightBracket );
			mNameBuffer.back() = 0;
		}

		virtual void popName()
		{
			if ( mNameStack.empty() )
				return;
			mNameBuffer.resize( static_cast<PxU32>( mNameStack.back() ) );
			mNameStack.popBack();
			if ( mNameBuffer.empty() == false )
				mNameBuffer.back() = 0;
		}

		virtual const char* getTopName()
		{
			if ( mNameBuffer.size() )
				return mNameBuffer.begin();
			return "";
		}

		/**
		 *	Define a property using the top of the name stack and the passed-in semantic
		 */
		virtual void defineProperty( PxU32 inClass, const char* inSemantic, PvdCommLayerDatatype inDatatype, PxU32 inPropertyKey )
		{
			mStream.defineProperty( inClass, getTopName(), inSemantic, inDatatype, inPropertyKey );
		}

		virtual void defineArrayProperty( PxU32 inClass, PxU32 inArrayClass, PxU32 inPropertyKey )
		{
			mStream.defineArrayProperty( inClass, getTopName(), inArrayClass, inPropertyKey );
		}

		virtual void defineProperty( PxU32 inClass, const char* inName, const char* inSemantic, PvdCommLayerDatatype inDatatype, PxU32 inPropertyKey ) 
		{
			if ( mNameBuffer.empty() )
				mStream.defineProperty( inClass, inName, inSemantic, inDatatype, inPropertyKey );
			else
			{
				pushName( inName );
				mStream.defineProperty( inClass, getTopName(), inSemantic, inDatatype, inPropertyKey );
				popName();
			}
		}

		virtual void addNamedValueDefinition( const char* inName, PxU32 inValue )
		{
			NamedValueDefinition theDef = { inName, inValue };
			mTempDefinitions.pushBack( theDef );
		}

		virtual void defineBitflagNames( PxU32 inClass, PxU32 inPropertyKey )
		{
			if ( mTempDefinitions.empty() == false )
			{
				mStream.defineBitflagNames( inClass, inPropertyKey, mTempDefinitions.begin(), static_cast<PxU32>( mTempDefinitions.size() ) );
			}
			mTempDefinitions.clear();
		}
		virtual void defineEnumerationNames( PxU32 inClass, PxU32 inPropertyKey ) 
		{
			if ( mTempDefinitions.empty() == false )
			{
				mStream.defineEnumerationNames( inClass, inPropertyKey, mTempDefinitions.begin(), static_cast<PxU32>( mTempDefinitions.size() ) );
			}
			mTempDefinitions.clear();
		}
		
		virtual void clearStructEntries() { mPropertyStructEntries.clear(); }
		virtual void addStructPropertyEntry( PxU32 inPropertyKey, PvdCommLayerDatatype inDatatype, PxU32 inOffset )
		{
			mPropertyStructEntries.pushBack( PropertyStructEntry( inPropertyKey, inOffset, inDatatype.mDatatype ) );
		}

		virtual void definePropertyStruct( PxU32 inStructKey, PxU32 inClass, PxU32 inStructSizeInBytes ) 
		{
			mStream.definePropertyStruct( inStructKey, inClass, inStructSizeInBytes, mPropertyStructEntries.begin(), static_cast<PxU32>( mPropertyStructEntries.size() ) );
			mPropertyStructEntries.clear();
		}

		void reset()
		{
			mNameBuffer.clear();
			mNameStack.clear();
			mTempDefinitions.clear();
			mPropertyStructEntries.clear();
		}
	};

	class PvdBeginPropertyBlockHelperImpl : public PvdBeginPropertyBlockHelper
	{
		Array<PxU32>						mBodyPropKeys;
		Array<PVD::PvdCommLayerDatatype>	mBodyPropTypes;
		PvdDataStream&						mStream;

	public:
		PvdBeginPropertyBlockHelperImpl( PvdDataStream& inStream ) : mStream( inStream ) {}
		virtual void addProperty( PxU32 inPropertyKey, PvdCommLayerDatatype inDatatype )
		{
			mBodyPropKeys.pushBack( inPropertyKey );
			mBodyPropTypes.pushBack( inDatatype );
		}

		virtual void beginPropertyBlock(PxU32 inClass ) 
		{
			mStream.beginPropertyBlock( inClass, mBodyPropKeys.begin(), mBodyPropTypes.begin(), static_cast<PxU32>( mBodyPropKeys.size() ) );
			mBodyPropKeys.clear();
			mBodyPropTypes.clear();
		}

		void reset()
		{
			mBodyPropKeys.clear();
			mBodyPropTypes.clear();
		}
	};

	class PvdSendPropertyBlockHelperImpl : public PvdSendPropertyBlockHelper
	{
		physx::profile::MemoryBuffer<>		mBuffer;
		PvdDataStream&						mStream;
	public:

		PvdSendPropertyBlockHelperImpl( PvdDataStream& inStream )
			: mStream( inStream )
		{
		}
		
		template<typename TDataType>
		inline void doAddProperty( const TDataType& inType ) 
		{ 
			mBuffer.write( inType ); 
		}
		
		virtual void addValue( PxU8 inVal ) { doAddProperty( inVal ); }
		virtual void addValue( PxU16 inVal ) { doAddProperty( inVal ); }
		virtual void addValue( PxU32 inVal ) { doAddProperty( inVal ); }
		virtual void addValue( PxU64 inVal ) { doAddProperty( inVal ); }
		virtual void addValue( PxI8 inVal ) { doAddProperty( inVal ); }
		virtual void addValue( PxI16 inVal ) { doAddProperty( inVal ); }
		virtual void addValue( PxI32 inVal ) { doAddProperty( inVal ); }
		virtual void addValue( PxI64 inVal ) { doAddProperty( inVal ); }
		virtual void addValue( PxF32 inVal ) { doAddProperty( inVal ); }
		virtual void addValue( PxF64 inVal ) { doAddProperty( inVal ); }
		virtual void addValue( PxVec3 inVal ) { doAddProperty( inVal ); }
		virtual void addBitflagValue( PxU32 inVal ) { doAddProperty( inVal ); }
		virtual void addEnumerationValue( PxU32 inVal ) { doAddProperty( inVal ); }
		virtual void addValue( bool inVal ) { doAddProperty( inVal ); }
		virtual void addValue( const PxQuat& inVal ) { doAddProperty( createQuat( inVal ) ); }
		virtual void addValue( const PxTransform& inVal ) { doAddProperty( createTransform( inVal ) ); }
		virtual void addValue( const PxBounds3& inVal ) { doAddProperty( createBounds3( inVal ) ); }
		virtual void addValue( const PVD::FilterData& inVal ) { doAddProperty( inVal ); }
		virtual void addValue( const PVD::Plane& inVal ) { doAddProperty( inVal ); }
		virtual void addValue( InstanceId inVal ) { doAddProperty( inVal ); }
		virtual void addValue( const char* inVal ) 
		{ 
			if ( inVal == NULL ) inVal = "";
			//Write the null termination to the stream so we don't have to copy
			//strings into different buffers when parsing to get valid strings.
			PxU32 strLen = static_cast<PxU32>( strlen( inVal ) ) + 1;
			mBuffer.write( strLen );
			mBuffer.write( inVal, strLen );
		}

		virtual void sendPropertyBlock( PxU64 inInstance )
		{
			PxU32 theSize = static_cast<PxU32>( mBuffer.size() );
			if ( theSize )
				mStream.sendPropertyBlock( inInstance, mBuffer.begin(), theSize, NULL );
			mBuffer.clear();
		}
		
		void reset()
		{
			mBuffer.clear();
		}
	};
}

#endif
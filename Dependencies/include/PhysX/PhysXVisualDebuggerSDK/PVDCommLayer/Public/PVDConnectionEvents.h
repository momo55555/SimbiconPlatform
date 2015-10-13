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

#ifndef PVD_PVDCONNECTIONEVENTS_H
#define PVD_PVDCONNECTIONEVENTS_H

#include "PVDCommLayerTypes.h"
#include "PvdRenderCommands.h"
#include "PsTime.h"
#include "PsHash.h"

namespace PVD
{
	/**
	 *	Define a data interface to the PVD layer.
	 *	These events define the actual bits that are sent
	 *	over the wire to the remote pvd implementation.
	 *	The individual events are meant to be constructed either
	 *	fully or trivially, so either all properties are defined
	 *	else none of the properties are defined.
	 *
	 *	Adding a new event type means a new enumeration,
	 *	bumping the stream version number, and a new structure
	 *	that defines the event type.
	 *
	 *	streamify is a two-way operator.  When reading it sets
	 *	the member variables and when writing it writes them.  This
	 *	saves code over needing to write the same sequence of operations
	 *	in multiple places.
	 */
	struct PvdConnectionEventType
	{
		enum
		{
			Unknown = 0,
			StreamInitialization,
			CreateClass,
			DeriveClass,
			DefineProperty,
			DefineBitflagNames,
			CreateInstance,
			SetPropertyValue,
			SetPropertyMediumValue,
			SetPropertySmallValue,
			BeginPropertyBlock,
			SendPropertyBlock,
			EndPropertyBlock,
			AddChild,
			RemoveChild,
			DestroyInstance,
			BeginArrayBlock,
			ArrayObject,
			EndArrayBlock,
			NamedEvent,
			Error,
			Warning,
			BeginSection,
			EndSection,
			Disconnect,
			//These are meta-events where they indicate containment of
			//other relative events.
			EventBatch, //A set of events with an event type preceding each one but no size.
			//A multiple events block.  This ended up making everything less efficient
			//so it is unused any more on the sending side.  On the receiving side we still need
			//to support it for legacy reasons
			MultipleEvent, 
			//Extension events made after version 1.0 made it into APEX
			NamedEventWithInstance,
			DefineEnumerationNames,
			DefinePropertyOnInstance,
			RemoveAllChildren,
			ArrayObjects,
			DefineArrayProperty,
			BeginArrayPropertyBlock,
			EndArrayPropertyBlock,
			SetNamespace,
			PushNamespace,
			PopNamespace,
			RegisterPropertyStruct,
			SendPropertyStruct,
			RawPropertyBlock, //this one is wedged in there in an odd fasion.
			Last,
		};
		PxU8	mEventType;
		PvdConnectionEventType( PxU8 inType=Unknown ) : mEventType( inType ) {}
		PX_INLINE bool operator==( const PvdConnectionEventType& inOther ) const { return mEventType == inOther.mEventType; }
		PX_INLINE bool operator!=( const PvdConnectionEventType& inOther ) const { return !(*this == inOther ); }
		PX_INLINE const char* toString() const
		{
			if ( mEventType & 0x80 )
			{
				RenderCommandType theType( mEventType & 0x7F );
				return theType.toString();
			}
			switch( mEventType )
			{
			case Unknown: return "Unknown";
			case StreamInitialization: return "StreamInitialization";
			case CreateClass: return "CreateClass";
			case DeriveClass: return "DeriveClass";
			case DefineProperty: return "DefineProperty";
			case DefineBitflagNames: return "DefineBitflagNames";
			case CreateInstance: return "CreateInstance";
			case SetPropertyValue: return "SetPropertyValue";
			case SetPropertyMediumValue: return "SetPropertyMediumValue";
			case SetPropertySmallValue: return "SetPropertySmallValue";
			case BeginPropertyBlock: return "BeginPropertyBlock";
			case SendPropertyBlock: return "SendPropertyBlock";
			case EndPropertyBlock: return "EndPropertyBlock";
			case AddChild: return "AddChild";
			case RemoveChild: return "RemoveChild";
			case DestroyInstance: return "DestroyInstance";
			case BeginArrayBlock: return "BeginArrayBlock";
			case ArrayObject: return "ArrayObject";
			case EndArrayBlock: return "EndArrayBlock";
			case NamedEvent: return "NamedEvent";
			case Error: return "Error";
			case Warning: return "Warning";
			case BeginSection: return "BeginSection";
			case EndSection: return "EndSection";
			case Disconnect: return "Disconnect";
			case EventBatch: return "EventBatch";
			case MultipleEvent: return "MultipleEvent";
			case NamedEventWithInstance: return "NamedEventWithInstance";
			case DefineEnumerationNames: return "DefineEnumerationNames";
			case DefinePropertyOnInstance: return "DefinePropertyOnInstance";
			case RemoveAllChildren: return "RemoveAllChildren";
			case ArrayObjects: return "ArrayObjects";
			case DefineArrayProperty: return "DefineArrayProperty";
			case BeginArrayPropertyBlock: return "BeginArrayPropertyBlock";
			case EndArrayPropertyBlock: return "EndArrayPropertyBlock";
			case SetNamespace: return "SetNamespace";
			case PushNamespace: return "PushNamespace";
			case PopNamespace: return "PopNamespace";
			case RegisterPropertyStruct: return "RegisterPropertyStruct";
			case SendPropertyStruct: return "SendPropertyStruct";
			}
			return "";
		}
	};
	
	union UStreamDatatypeConverter
	{
		PxU8  mU8[8];
		PxU16 mU16;
		PxU32 mU32;
		PxU64 mU64;
		PxI8  mI8[8];
		PxI16 mI16;
		PxI32 mI32;
		PxI64 mI64;
		PxF32 mF32;
		PxF64 mF64;
	};

	PX_INLINE void writeU32( PxU8* inDest, PxU32 inData )
	{
		UStreamDatatypeConverter theConverter;
		theConverter.mU32 = inData;
		inDest[0] = theConverter.mU8[0];
		inDest[1] = theConverter.mU8[1];
		inDest[2] = theConverter.mU8[2];
		inDest[3] = theConverter.mU8[3];
	}

	PX_INLINE void writeU64( PxU8* inDest, PxU64 inData )
	{
		UStreamDatatypeConverter theConverter;
		theConverter.mU64 = inData;
		inDest[0] = theConverter.mU8[0];
		inDest[1] = theConverter.mU8[1];
		inDest[2] = theConverter.mU8[2];
		inDest[3] = theConverter.mU8[3];
		inDest[4] = theConverter.mU8[4];
		inDest[5] = theConverter.mU8[5];
		inDest[6] = theConverter.mU8[6];
		inDest[7] = theConverter.mU8[7];
	}

	PX_INLINE void writeF64( PxU8* inDest, PxF64 inData )
	{
		UStreamDatatypeConverter theConverter;
		theConverter.mF64 = inData;
		writeU64( inDest, theConverter.mU64 );
	}
	
	struct SEventHeader
	{
		PvdConnectionEventType mEventType;
		PxU32					mSize;

		PX_INLINE void toBytes( PxU8* outData ) const
		{
			outData[0] = mEventType.mEventType;
			writeU32( outData + 1, mSize );
		}

		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& inStream )
		{
			inStream.streamify( mEventType.mEventType );
			inStream.streamify( mSize );
		}
	};

	struct SEventHeader2
	{
		PvdConnectionEventType	mEventType;
		PxU64					mStreamId;
		PxU32					mSize;

		PX_INLINE void toBytes( PxU8* outData ) const
		{
			outData[0] = mEventType.mEventType;
			writeU64( outData + 1, mStreamId );
			writeU32( outData + 9, mSize );
		}

		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& inStream )
		{
			inStream.streamify( mEventType.mEventType );
			inStream.streamify( mStreamId );
			inStream.streamify( mSize );
		}
	};

	struct SEventHeader3
	{
		PvdConnectionEventType	mEventType;
		PxU64					mStreamId;
		PxU64					mTimestamp; //Tick counts.  See PsTime.h!!
		PxU32					mSize;

		PX_INLINE void toBytes( PxU8* outData ) const
		{
			outData[0] = mEventType.mEventType; outData += 1;
			writeU64( outData, mStreamId ); outData += 8;
			writeU64( outData, mTimestamp ); outData += 8;
			writeU32( outData, mSize ); outData += 4;
		}
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& inStream )
		{
			inStream.streamify( mEventType.mEventType );
			inStream.streamify( mStreamId );
			inStream.streamify( mTimestamp );
			inStream.streamify( mSize );
		}
	};

	struct StreamFlags
	{
		enum Enum
		{
			PointersAre64Bits = 1,
		};
	};

	struct SStreamInitialization
	{
		PxU32 mMagicNumber;
		PxU16 mStreamVersion;
		PxU16 mStreamFlags;
		PxU64 mTickCountToTensOfNanosNumerator;
		PxU64 mTickCountToTensOfNanosDenominator;
		static const PxU16 sCurrentStreamVersion = 8;
		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::StreamInitialization; }
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream )
		{
			ioStream.streamify( mMagicNumber );
			ioStream.streamify( mStreamVersion );
			ioStream.streamify( mStreamFlags );
			if ( mStreamVersion >= 5 )
			{
				ioStream.streamify( mTickCountToTensOfNanosNumerator );
				ioStream.streamify( mTickCountToTensOfNanosDenominator );
			}
		}

		template<typename TOperator>
		PX_INLINE bool compare( const SStreamInitialization& inOther, TOperator inOperator ) const
		{
			return inOperator( mMagicNumber, inOther.mMagicNumber ) 
				&& inOperator( mStreamVersion, inOther.mStreamVersion )
				&& inOperator( mStreamFlags, inOther.mStreamFlags )
				&& inOperator( mTickCountToTensOfNanosNumerator, inOther.mTickCountToTensOfNanosNumerator )
				&& inOperator( mTickCountToTensOfNanosDenominator, inOther.mTickCountToTensOfNanosDenominator );
		}
	};

	PX_INLINE SStreamInitialization createStreamInitialization() 
	{ 
		using namespace physx::shdfnd;
		CounterFrequencyToTensOfNanos conversion( Time::getCounterFrequency() );
		PxU16 streamFlags = 0;
		if ( sizeof( void* ) == 8 )
			streamFlags |= StreamFlags::PointersAre64Bits;
		SStreamInitialization retval = { 0xDEADBEEF, SStreamInitialization::sCurrentStreamVersion, streamFlags, conversion.mNumerator, conversion.mDenominator };
		return retval; 
	}

	struct SCreateClass
	{
		const char* mName;
		PxU32		mKey;
		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::CreateClass; }
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream )
		{
			ioStream.streamify( mName );
			ioStream.streamify( mKey );
		}
		template<typename TOperator>
		PX_INLINE bool compare( const SCreateClass& inOther, TOperator inOperator ) const
		{
			return inOperator( mName, inOther.mName ) && inOperator( mKey, inOther.mKey );
		}
	};

	PX_INLINE SCreateClass createCreateClass( const char* inName, PxU32 inKey ) { SCreateClass retval = { inName, inKey }; return retval; }
	PX_INLINE SCreateClass createCreateClass() { return createCreateClass( 0, 0 ); }

	struct SDeriveClass
	{
		PxU32 mParentClass;
		PxU32 mChildClass;
		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::DeriveClass; }
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream )
		{
			ioStream.streamify( mParentClass );
			ioStream.streamify( mChildClass );
		}
		template<typename TOperator>
		PX_INLINE bool compare( const SDeriveClass& inOther, TOperator inOperator ) const
		{
			return inOperator( mParentClass, inOther.mParentClass ) && inOperator( mParentClass, inOther.mParentClass );
		}
	};

	PX_INLINE SDeriveClass createDeriveClass( PxU32 inParentClass, PxU32 inChildClass ) { SDeriveClass retval = { inParentClass, inChildClass }; return retval; }
	PX_INLINE SDeriveClass createDeriveClass() { return createDeriveClass( 0, 0 ); }

	struct SDefineProperty
	{
		PxU32					mClass;
		const char*				mName;
		const char*				mSemantic;
		PxU8					mDatatype;
		PxU32					mKey;	

		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::DefineProperty; }
		
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream )
		{
			ioStream.streamify( mClass );
			ioStream.streamify( mDatatype );
			ioStream.streamify( mKey );
			//Pointer types need to be deserialized and serialized together.
			//Due to the implementation strategies taken and that fact
			//that this object doesn't own the memory it points to.
			ioStream.streamify( mName, mSemantic );
		}
		template<typename TOperator>
		PX_INLINE bool compare( const SDefineProperty& inOther, TOperator inOperator ) const
		{
			return inOperator( mClass, inOther.mClass ) 
				&& inOperator( mName, inOther.mName )
				&& inOperator( mSemantic, inOther.mSemantic )
				&& inOperator( mDatatype, inOther.mDatatype )
				&& inOperator( mKey, inOther.mKey );
		}
	};
	PX_INLINE SDefineProperty createDefineProperty( PxU32 inClass, const char* inName, const char* inSemantic,
													PvdCommLayerDatatype inDatatype, PxU32 inKey )
	{
		SDefineProperty retval = { inClass, inName, inSemantic, inDatatype.mDatatype, inKey };
		return retval;
	}
	PX_INLINE SDefineProperty createDefineProperty() { return createDefineProperty( 0, NULL, NULL, PvdCommLayerDatatype(), 0 ); }

	
	struct SDefinePropertyOnInstance
	{
		PxU64					mInstance;
		const char*				mName;
		const char*				mSemantic;
		PxU8					mDatatype;
		PxU32					mKey;	

		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::DefinePropertyOnInstance; }
		
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream )
		{
			ioStream.streamify( mInstance );
			ioStream.streamify( mDatatype );
			ioStream.streamify( mKey );
			//Pointer types need to be deserialized and serialized together.
			//Due to the implementation strategies taken and that fact
			//that this object doesn't own the memory it points to.
			ioStream.streamify( mName, mSemantic );
		}
		template<typename TOperator>
		PX_INLINE bool compare( const SDefinePropertyOnInstance& inOther, TOperator inOperator ) const
		{
			return inOperator( mInstance, inOther.mInstance ) 
				&& inOperator( mName, inOther.mName )
				&& inOperator( mSemantic, inOther.mSemantic )
				&& inOperator( mDatatype, inOther.mDatatype )
				&& inOperator( mKey, inOther.mKey );
		}
	};

	PX_INLINE SDefinePropertyOnInstance createDefinePropertyOnInstance( PxU64 inInstanceId, const char* inName, const char* inSemantic,
																	PvdCommLayerDatatype inDatatype, PxU32 inKey )
	{
		SDefinePropertyOnInstance retval = { inInstanceId, inName, inSemantic, inDatatype.mDatatype, inKey };
		return retval;
	}

	PX_INLINE SDefinePropertyOnInstance createDefinePropertyOnInstance() { return createDefinePropertyOnInstance( 0, NULL, NULL, PvdCommLayerDatatype(), 0 ); }
	
	struct SDefineArrayProperty
	{
		PxU32						mClass;
		PxU32						mKey;
		PxU32						mArrayClass;
		const char*					mPropertyName;

		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::DefineArrayProperty; }

		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream )
		{
			ioStream.streamify( mClass );
			ioStream.streamify( mKey );
			ioStream.streamify( mArrayClass );
			ioStream.streamify( mPropertyName );
		}

		template<typename TOperator>
		PX_INLINE bool compare( const SDefineArrayProperty& inOther, TOperator inOperator ) const
		{
			return inOperator( mClass, inOther.mClass )
					&& inOperator( mKey, inOther.mKey )
					&& inOperator( mArrayClass, inOther.mArrayClass )
					&& inOperator( mPropertyName, inOther.mPropertyName );
		}
	};

	inline SDefineArrayProperty createDefineArrayProperty( PxU32 inClass, PxU32 inProperty, PxU32 inArrayClass, const char* inName )
	{
		SDefineArrayProperty retval = { inClass, inProperty, inArrayClass, inName };
		return retval;
	}

	struct SDefineItemNames
	{
		PxU32							mClass;
		PxU32							mProperty;
		const NamedValueDefinition*	mDefinitions;
		PxU32							mDefinitionLength;

		
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream )
		{
			ioStream.streamify( mClass );
			ioStream.streamify( mProperty, mDefinitions, mDefinitionLength );
		}
		template<typename TOperator>
		PX_INLINE bool compare( const SDefineItemNames& inOther, TOperator inOperator ) const
		{
			return inOperator( *this, inOther );
		}
		inline void Initialize( PxU32 inClass
								, PxU32 inPropertyName
								, const NamedValueDefinition* inDefinitions
								, PxU32 inDefinitionLength )
		{
			mClass = inClass; mProperty = inPropertyName; mDefinitions = inDefinitions; mDefinitionLength = inDefinitionLength;
		}
	};

	struct SDefineBitflagNames : public SDefineItemNames
	{	
		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::DefineBitflagNames; }
	};
	
	PX_INLINE SDefineBitflagNames createDefineBitflagNames( PxU32 inClass
															, PxU32 inPropertyName
															, const NamedValueDefinition* inDefinitions
															, PxU32 inDefinitionLength )
	{
		SDefineBitflagNames retval;
		retval.Initialize( inClass, inPropertyName, inDefinitions, inDefinitionLength );
		return retval;
	}

	PX_INLINE SDefineBitflagNames createDefineBitflagNames() { return createDefineBitflagNames( 0, 0, NULL, 0 ); }
	
	struct SDefineEnumerationNames : public SDefineItemNames
	{	
		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::DefineEnumerationNames; }
	};
	
	
	PX_INLINE SDefineEnumerationNames createDefineEnumerationNames( PxU32 inClass
																	, PxU32 inPropertyName
																	, const NamedValueDefinition* inDefinitions
																	, PxU32 inDefinitionLength )
	{
		SDefineEnumerationNames retval;
		retval.Initialize( inClass, inPropertyName, inDefinitions, inDefinitionLength );
		return retval;
	}

	PX_INLINE SDefineEnumerationNames createDefineEnumerationNames() { return createDefineEnumerationNames( 0, 0, NULL, 0 ); }

	struct SCreateInstance
	{
		PxU32 mClass;
		PxU64 mInstanceId;
		PxU32 mFlags;

		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::CreateInstance; }
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream )
		{
			ioStream.streamify( mClass );
			ioStream.streamify( mInstanceId );
			ioStream.streamify( mFlags );
		}
		template<typename TOperator>
		PX_INLINE bool compare( const SCreateInstance& inOther, TOperator inOperator ) const
		{
			return inOperator( mClass, inOther.mClass) &&
					inOperator( mInstanceId, inOther.mInstanceId) &&
					inOperator( mFlags, inOther.mFlags );
		}
		template<typename TStreamType>
		PX_INLINE void textStreamify( TStreamType& ioStream )
		{
			ioStream.streamify( "class", mClass );
			ioStream.streamify( "instanceId", mInstanceId );
			ioStream.streamify( "flags", mFlags );
		}
	};
	
	PX_INLINE SCreateInstance createCreateInstance( PxU32 inClass, PxU64 inInstanceId, PxU32 inFlags ) 
	{ 
		SCreateInstance retval = { inClass, inInstanceId, inFlags }; return retval; 
	}
	PX_INLINE SCreateInstance createCreateInstance() { return createCreateInstance( 0, 0, 0 ); }
	
	struct SSetPropertyValue
	{
		PxU64					mInstanceId;
		PxU32					mProperty;
		PxU8					mDatatype;
		PvdCommLayerData		mValue;

		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::SetPropertyValue; }
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream )
		{
			ioStream.streamify( mInstanceId );
			ioStream.streamify( mProperty );
			ioStream.streamify( mDatatype );
			ioStream.streamify( mDatatype, mValue );
		}
		template<typename TOperator>
		PX_INLINE bool compare( const SSetPropertyValue& inOther, TOperator inOperator ) const
		{
			return inOperator( mInstanceId, inOther.mInstanceId) &&
					inOperator( mProperty, inOther.mProperty) &&
					inOperator( PvdCommLayerValue( mDatatype, mValue ), PvdCommLayerValue( inOther.mDatatype, inOther.mValue ) );
		}
	};
	
	PX_INLINE SSetPropertyValue createSetPropertyValue( PxU64 inInstanceId, PxU32 inProperty, const PvdCommLayerValue& inValue )
	{ 
		SSetPropertyValue retval = { inInstanceId, inProperty, inValue.getDatatype().mDatatype, inValue.getData() }; 
		return retval; 
	}
	PX_INLINE SSetPropertyValue createSetPropertyValue(PvdCommLayerDatatype inDatatype) { return createSetPropertyValue( 0, 0, PvdCommLayerValue(inDatatype) ); }

	struct SSetPropertyMediumValue
	{
		PxU64						mInstanceId;
		PxU32						mProperty;
		PxU8						mDatatype;
		PvdCommLayerMediumData		mValue;

		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::SetPropertyMediumValue; }
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream )
		{
			ioStream.streamify( mInstanceId );
			ioStream.streamify( mProperty );
			ioStream.streamify( mDatatype );
			ioStream.streamify( mDatatype, mValue );
		}
		template<typename TOperator>
		PX_INLINE bool compare( const SSetPropertyMediumValue& inOther, TOperator inOperator ) const
		{
			return inOperator( mInstanceId, inOther.mInstanceId) &&
					inOperator( mProperty, inOther.mProperty) &&
					inOperator( PvdCommLayerValue( mDatatype, mValue ), PvdCommLayerValue( inOther.mDatatype, inOther.mValue ) );
		}
	};
	
	PX_INLINE SSetPropertyMediumValue createSetPropertyMediumValue( PxU64 inInstanceId, PxU32 inProperty, const PvdCommLayerMediumValue& inValue )
	{ 
		SSetPropertyMediumValue retval = { inInstanceId, inProperty, inValue.getDatatype().mDatatype, inValue.getData() }; 
		return retval; 
	}
	PX_INLINE SSetPropertyMediumValue createSetPropertyMediumValue(PvdCommLayerDatatype inDatatype) { return createSetPropertyMediumValue( 0, 0, PvdCommLayerMediumValue(inDatatype) ); }

	
	struct SSetPropertySmallValue
	{
		PxU64						mInstanceId;
		PxU32						mProperty;
		PxU8						mDatatype;
		PvdCommLayerSmallData		mValue;

		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::SetPropertySmallValue; }
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream )
		{
			ioStream.streamify( mInstanceId );
			ioStream.streamify( mProperty );
			ioStream.streamify( mDatatype );
			ioStream.streamify( mDatatype, mValue );
		}
		template<typename TOperator>
		PX_INLINE bool compare( const SSetPropertySmallValue& inOther, TOperator inOperator ) const
		{
			return inOperator( mInstanceId, inOther.mInstanceId) &&
					inOperator( mProperty, inOther.mProperty) &&
					inOperator( PvdCommLayerValue( mDatatype, mValue ), PvdCommLayerValue( inOther.mDatatype, inOther.mValue ) );
		}
	};
	
	PX_INLINE SSetPropertySmallValue createSetPropertySmallValue( PxU64 inInstanceId, PxU32 inProperty, const PvdCommLayerSmallValue& inValue )
	{ 
		SSetPropertySmallValue retval = { inInstanceId, inProperty, inValue.getDatatype().mDatatype, inValue.getData() }; 
		return retval; 
	}
	PX_INLINE SSetPropertySmallValue createSetPropertySmallValue(PvdCommLayerDatatype inDatatype) { return createSetPropertySmallValue( 0, 0, PvdCommLayerSmallValue(inDatatype) ); }

	struct SBeginPropertyBlock
	{
		PxU32							mClass;
		const PxU32*					mProperties;
		const PvdCommLayerDatatype*	mDatatypes;
		PxU32							mPropertyCount;

		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::BeginPropertyBlock; }

		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream )
		{
			ioStream.streamify( mClass );
			ioStream.streamify( mDatatypes, mPropertyCount );
			ioStream.streamify( mProperties, mPropertyCount );
		}

		template<typename TOperator>
		PX_INLINE bool compare( const SBeginPropertyBlock& inOther, TOperator inOperator ) const
		{
			return inOperator( *this, inOther);
		}
	};
	
	PX_INLINE SBeginPropertyBlock createBeginPropertyBlock( PxU32 inClass, const PxU32* inProperties, const PvdCommLayerDatatype* inDatatypes, PxU32 inPropertyCount )
	{ 
		SBeginPropertyBlock retval = { inClass, inProperties, inDatatypes, inPropertyCount };
		return retval; 
	}
	PX_INLINE SBeginPropertyBlock createBeginPropertyBlock() { return createBeginPropertyBlock( 0, NULL, NULL, 0 ); }

	struct SSendPropertyBlock
	{
		PxU64							mInstanceId;
		const PvdCommLayerData*			mValues;
		const PvdCommLayerDatatype*		mDatatypes;
		PxU32							mPropertyCount;
		
		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::SendPropertyBlock; }

		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream )
		{
			ioStream.streamify( mInstanceId );
			ioStream.streamify( mDatatypes, mPropertyCount, mValues );
		}

		template<typename TOperator>
		PX_INLINE bool compare( const SSendPropertyBlock& inOther, TOperator inOperator ) const
		{
			return inOperator( *this, inOther);
		}
	};
	
	PX_INLINE SSendPropertyBlock createSendPropertyBlock( const PvdCommLayerDatatype* inDatatypes
															, PxU32 inPropertyCount
															, PxU64 inInstance
															, const PvdCommLayerData* inData )
	{ 
		SSendPropertyBlock retval = { inInstance, inData, inDatatypes, inPropertyCount }; 
		return retval; 
	}

	PX_INLINE SSendPropertyBlock createSendPropertyBlock(const PvdCommLayerDatatype* inDatatypes, PxU32 inCounts) 
	{ 
		return createSendPropertyBlock( inDatatypes, inCounts, 0, NULL );
	}

	
	struct SRawPropertyBlock
	{
		PxU64							mInstanceId;
		const PvdCommLayerDatatype*		mDatatypes;
		PxU32							mPropertyCount;
		const PxU8*						mData;
		PxU32							mDataLen;
		const PvdCommLayerData*			mCheckData;
		
		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::SendPropertyBlock; }

		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream )
		{
			ioStream.streamify( mInstanceId );
			ioStream.bulkStreamify( mData, mDataLen );
		}

		template<typename TStreamType>
		PX_INLINE void inPlaceRead( TStreamType& ioStream )
		{
			ioStream.streamify( mInstanceId );
			mData = ioStream.getReadPtr();
			for ( PxU32 idx =0; idx < mPropertyCount; ++idx )
			{
				//Perform byte swapping and update the data ptr to point to 
				//the next location.
				ioStream.inPlaceStreamify( mDatatypes[idx], idx );
			}
			mDataLen = static_cast<PxU32>( ioStream.getReadPtr() - mData );
		}

		template<typename TOperator>
		PX_INLINE bool compare( const SSendPropertyBlock& inOther, TOperator inOperator ) const
		{
			return true;
		}
	};

	PX_INLINE SRawPropertyBlock createRawPropertyBlock( const PvdCommLayerDatatype* inDatatypes
															, PxU32 inPropertyCount
															, PxU64 inInstanceId
															, const PxU8* inData
															, PxU32 inDataLen
															, const PvdCommLayerData* inCheckData )
	{
		SRawPropertyBlock retval = { inInstanceId, inDatatypes, inPropertyCount, inData, inDataLen, inCheckData };
		return retval;
	}

	struct SEndPropertyBlock
	{
		PxU32 unused;

		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::EndPropertyBlock; }
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType&)
		{
		}

		template<typename TOperator>
		PX_INLINE bool compare( const SEndPropertyBlock& /*inOther*/, TOperator /*inOperator*/ ) const
		{
			return true;
		}
	};

	PX_INLINE SEndPropertyBlock createEndPropertyBlock() 
	{ 
		SEndPropertyBlock retval = { 0 };
		return retval;
	}


	struct SAddChild
	{
		PxU64				mParent;
		PxU64				mChild;
		
		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::AddChild; }
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream )
		{
			ioStream.streamify( mParent );
			ioStream.streamify( mChild );
		}
		template<typename TOperator>
		PX_INLINE bool compare( const SAddChild& inOther, TOperator inOperator ) const
		{
			return inOperator( mParent, inOther.mParent) &&
					inOperator( mChild, inOther.mChild);
		}
		template<typename TStreamType>
		PX_INLINE void textStreamify( TStreamType& ioStream )
		{
			ioStream.streamify( "parent", mParent );
			ioStream.streamify( "child", mChild );
		}
	};
	
	PX_INLINE SAddChild createAddChild( PxU64 inParent, PxU64 inChild ) { SAddChild retval = { inParent, inChild }; return retval; }
	PX_INLINE SAddChild createAddChild() { return createAddChild( 0, 0 ); }
	
	struct SRemoveChild
	{
		PxU64				mParent;
		PxU64				mChild;

		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::RemoveChild; }
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream )
		{
			ioStream.streamify( mParent );
			ioStream.streamify( mChild );
		}
		template<typename TOperator>
		PX_INLINE bool compare( const SRemoveChild& inOther, TOperator inOperator ) const
		{
			return inOperator( mParent, inOther.mParent) &&
					inOperator( mChild, inOther.mChild);
		}
		template<typename TStreamType>
		PX_INLINE void textStreamify( TStreamType& ioStream )
		{
			ioStream.streamify( "parent", mParent );
			ioStream.streamify( "child", mChild );
		}
	};
	
	PX_INLINE SRemoveChild createRemoveChild( PxU64 inParent, PxU64 inChild ) { SRemoveChild retval = { inParent, inChild }; return retval; }
	PX_INLINE SRemoveChild createRemoveChild() { return createRemoveChild( 0, 0 ); }

	struct SDestroyInstance
	{
		PxU64 mInstanceId;

		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::DestroyInstance; }
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream )
		{
			ioStream.streamify( mInstanceId );
		}
		template<typename TOperator>
		PX_INLINE bool compare( const SDestroyInstance& inOther, TOperator inOperator ) const
		{
			return inOperator( mInstanceId, inOther.mInstanceId);
		}
		template<typename TStreamType>
		PX_INLINE void textStreamify( TStreamType& ioStream )
		{
			ioStream.streamify( "instanceId", mInstanceId );
		}
	};
	
	PX_INLINE SDestroyInstance createDestroyInstance( PxU64 inInstanceId ) { SDestroyInstance retval = { inInstanceId }; return retval; }
	PX_INLINE SDestroyInstance createDestroyInstance() { return createDestroyInstance( 0 ); }
	
	struct SBeginArrayBlock
	{
		PxU32								mClassKey;
		PxU64								mInstanceId;
		const PxU32*						mProperties;
		const PvdCommLayerDatatype*			mDatatypes;
		PxU32								mPropertyCount;

		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::BeginArrayBlock; }
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream )
		{
			ioStream.streamify( mClassKey );
			ioStream.streamify( mInstanceId );
			ioStream.streamify( mDatatypes, mPropertyCount );
			ioStream.streamify( mProperties, mPropertyCount );
		}
		template<typename TOperator>
		PX_INLINE bool compare( const SBeginArrayBlock& inOther, TOperator inOperator ) const
		{
			return inOperator( *this, inOther );
		}
	};
	
	PX_INLINE SBeginArrayBlock createBeginArrayBlock( PxU32 inClassKey
																, PxU64 mInstanceId
																, const PxU32* inProperties
																, const PvdCommLayerDatatype* inDatatypes
																, PxU32 inNumProps )
	{ SBeginArrayBlock retval = { inClassKey, mInstanceId, inProperties, inDatatypes, inNumProps }; return retval; }
	PX_INLINE SBeginArrayBlock createBeginArrayBlock() { return createBeginArrayBlock( 0, 0, NULL, NULL, 0 ); }
	
	struct SBeginArrayPropertyBlock
	{
		PxU64								mInstanceId;
		PxU32								mInstanceProperty;
		const PxU32*						mProperties;
		const PvdCommLayerDatatype*			mDatatypes;
		PxU32								mPropertyCount;

		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::BeginArrayPropertyBlock; }
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream )
		{
			ioStream.streamify( mInstanceId );
			ioStream.streamify( mInstanceProperty );
			ioStream.streamify( mDatatypes, mPropertyCount );
			ioStream.streamify( mProperties, mPropertyCount );
		}

		template<typename TOperator>
		PX_INLINE bool compare( const SBeginArrayPropertyBlock& inOther, TOperator inOperator ) const
		{
			return inOperator( mInstanceId, inOther.mInstanceId )
					&& inOperator( mInstanceProperty, inOther.mInstanceProperty )
					&& inOperator( mPropertyCount, inOther.mPropertyCount )
					&& inOperator( mProperties, inOther.mProperties, mPropertyCount )
					&& inOperator( mDatatypes, inOther.mDatatypes, mPropertyCount );
		}
	};
	
	PX_INLINE SBeginArrayPropertyBlock createBeginArrayPropertyBlock( PxU64 inInstanceId
													, PxU32 inProperty
													, const PxU32* inProperties
													, const PvdCommLayerDatatype* inDatatypes
													, PxU32 inNumProps )
	{ SBeginArrayPropertyBlock retval = { inInstanceId, inProperty, inProperties, inDatatypes, inNumProps }; return retval; }

	struct SArrayObject
	{
		const PvdCommLayerData*		mValues;
		const PvdCommLayerDatatype*	mDatatypes;
		PxU32							mPropertyCount;

		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::ArrayObject; }
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream )
		{
			ioStream.streamify( mDatatypes, mPropertyCount, mValues );
		}
		template<typename TOperator>
		PX_INLINE bool compare( const SArrayObject& inOther, TOperator inOperator ) const
		{
			return inOperator( *this, inOther);
		}
	};
	
	PX_INLINE SArrayObject createArrayObject( const PvdCommLayerDatatype* inDatatypes, const PvdCommLayerData* inValues, PxU32 inValueCount ) 
	{ 
		SArrayObject retval = { inValues, inDatatypes, inValueCount };
		return retval;
	}
	//There is no trivial constructor of these objects because they are partial objects;
	//they don't have enough information to serialize or deserialize themselves in a default
	//state and that information isn't serialized; it is inferred from the BeginArrayObject
	//block.
	PX_INLINE SArrayObject createArrayObject(const PvdCommLayerDatatype* inDatatypes, PxU32 inValueCount ) 
	{ 
		return createArrayObject( inDatatypes, NULL, inValueCount );
	}

	struct SEndArrayBlock
	{
		PxU32 mUnused;

		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::EndArrayBlock; }
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType&)
		{
		}
		template<typename TOperator>
		PX_INLINE bool compare( const SEndArrayBlock& /*inOther*/, TOperator /*inOperator*/ ) const
		{
			return true;
		}
	};
	
	PX_INLINE SEndArrayBlock createEndArrayBlock() 
	{ 
		SEndArrayBlock retval = { 0 };
		return retval;
	}

	struct SEndArrayPropertyBlock
	{
		PxU32 mUnused;

		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::EndArrayPropertyBlock; }
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType&)
		{
		}
		template<typename TOperator>
		PX_INLINE bool compare( const SEndArrayPropertyBlock& /*inOther*/, TOperator /*inOperator*/ ) const
		{
			return true;
		}
	};
	
	PX_INLINE SEndArrayPropertyBlock createEndArrayPropertyBlock() 
	{ 
		SEndArrayPropertyBlock retval = { 0 };
		return retval;
	}

	struct SNamedEvent
	{
		const char* mName;
		PxU32		mPayload;

		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::NamedEvent; }
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream )
		{
			ioStream.streamify( mName );
			ioStream.streamify( mPayload );
		}
		template<typename TOperator>
		PX_INLINE bool compare( const SNamedEvent& inOther, TOperator inOperator ) const
		{
			return inOperator(mName, inOther.mName) && inOperator(mPayload, inOther.mPayload);
		}
	};
	
	PX_INLINE SNamedEvent createNamedEvent(const char* inName, PxU32 inPayload )
	{ 
		SNamedEvent retval = { inName, inPayload };
		return retval;
	}
	
	PX_INLINE SNamedEvent createNamedEvent() { return createNamedEvent( NULL, 0 ); }

	struct SError
	{
		const char* mType;
		const char* mMessage;
		const char* mFile;
		PxU32		mLine;

		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::Error; }
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream )
		{
			ioStream.streamify( mLine );
			ioStream.streamify( mType, mMessage, mFile );
		}
		template<typename TOperator>
		PX_INLINE bool compare( const SError& inOther, TOperator inOperator ) const
		{
			return inOperator(mType, inOther.mType) 
				&& inOperator(mMessage, inOther.mMessage)
				&& inOperator(mFile, inOther.mFile)
				&& inOperator(mLine, inOther.mLine);
		}
	};
	
	PX_INLINE SError createError( const char* inType, const char* inMessage, const char* inFile, PxU32 inLine )
	{ 
		SError retval = { inType, inMessage, inFile, inLine };
		return retval;
	}
	PX_INLINE SError createError() { return createError( NULL, NULL, NULL, 0 ); }
	
	struct SWarning
	{
		const char* mType;
		const char* mMessage;
		const char* mFile;
		PxU32		mLine;
		
		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::Warning; }
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream )
		{
			ioStream.streamify( mLine );
			ioStream.streamify( mType, mMessage, mFile );
		}
		template<typename TOperator>
		PX_INLINE bool compare( const SWarning& inOther, TOperator inOperator ) const
		{
			return inOperator(mType,	inOther.mType) 
				&& inOperator(mMessage, inOther.mMessage)
				&& inOperator(mFile,	inOther.mFile)
				&& inOperator(mLine,	inOther.mLine);
		}
	};
	
	PX_INLINE SWarning createWarning( const char* inType, const char* inMessage, const char* inFile, PxU32 inLine )
	{ 
		SWarning retval = { inType, inMessage, inFile, inLine };
		return retval;
	}
	PX_INLINE SWarning createWarning() { return createWarning( NULL, NULL, NULL, 0 ); }

	struct SBeginSection
	{
		const char*		mName;
		PxU64			mTimestamp;
		
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream )
		{
			ioStream.streamify( mName );
			ioStream.streamifySectionTimestamp( mTimestamp );
		}
		template<typename TOperator>
		PX_INLINE bool compare( const SBeginSection& inOther, TOperator inOperator ) const
		{
			return inOperator(mName,	inOther.mName) 
				&& inOperator(mTimestamp, inOther.mTimestamp);
		}
		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::BeginSection; }
	};
	
	PX_INLINE SBeginSection createBeginSection( const char* inName, PxU64 mTimestamp )
	{ 
		SBeginSection retval = { inName, mTimestamp };
		return retval;
	}
	PX_INLINE SBeginSection createBeginSection() { return createBeginSection( NULL, 0 ); }

	struct SEndSection
	{
		const char*		mName;
		PxU64			mTimestamp;
		
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream )
		{
			ioStream.streamify( mName );
			ioStream.streamifySectionTimestamp( mTimestamp );
		}
		template<typename TOperator>
		PX_INLINE bool compare( const SEndSection& inOther, TOperator inOperator ) const
		{
			return inOperator(mName,	inOther.mName) 
				&& inOperator(mTimestamp, inOther.mTimestamp);
		}
		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::EndSection; }
	};
	
	PX_INLINE SEndSection createEndSection( const char* inName, PxU64 mTimestamp )
	{ 
		SEndSection retval = { inName, mTimestamp };
		return retval;
	}
	PX_INLINE SEndSection createEndSection() { return createEndSection( NULL, 0 ); }

	struct SDisconnect
	{
		PxU8 unused;
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& )
		{
		}
		template<typename TOperator>
		PX_INLINE bool compare( const SDisconnect& , TOperator ) const
		{
			return true;
		}
		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::Disconnect; }
	};

	PX_INLINE SDisconnect createDisconnect() { return SDisconnect(); }


	struct SNamedEventWithInstance
	{
		PxU64			mInstanceId;
		const char*		mName;
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& inStream )
		{
			inStream.streamify( mInstanceId );
			inStream.streamify( mName );
		}
		template<typename TOperator>
		PX_INLINE bool compare( const SNamedEventWithInstance& inOther, TOperator inOperator ) const
		{
			return inOperator( mInstanceId, inOther.mInstanceId )
				 && inOperator( mName, inOther.mName );
		}
		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::NamedEventWithInstance; }
	};

	PX_INLINE SNamedEventWithInstance createNamedEventWithInstance(const char* inName, PxU64 inInstanceId ) 
	{ 
		SNamedEventWithInstance retval = { inInstanceId, inName };
		return retval;
	}

	PX_INLINE SNamedEventWithInstance createNamedEventWithInstance() { return createNamedEventWithInstance( NULL, 0 ); } 

	struct SRemoveAllChildren
	{
		PxU64 mInstanceId;
		PxU32 mChildClass;

		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::RemoveAllChildren; }
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& inStream )
		{
			inStream.streamify( mInstanceId );
			inStream.streamify( mChildClass );
		}
		template<typename TOperator>
		PX_INLINE bool compare( const SRemoveAllChildren& inOther, TOperator inOperator ) const
		{
			return inOperator( mInstanceId, inOther.mInstanceId )
				&& inOperator( mChildClass, inOther.mChildClass );
		}
	};
	
	PX_INLINE SRemoveAllChildren createRemoveAllChildren(PxU64 inInstanceId = 0, PxU32 inChildClass = 0 ) 
	{ 
		SRemoveAllChildren retval = { inInstanceId, inChildClass };
		return retval;
	}

	struct SArrayObjects
	{
		const PvdCommLayerDatatype*	mDatatypes;
		PxU32						mPropertyCount;
		PxU32						mStride;
		PxU32						mValueCount;
		const PxU8*					mValues;

		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::ArrayObjects; }

		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream )
		{
			ioStream.streamify( mDatatypes, mPropertyCount, mStride, mValueCount, mValues );
		}

		template<typename TOperator>
		PX_INLINE bool compare( const SArrayObjects& inOther, TOperator inOperator ) const
		{
			return inOperator( *this, inOther );
		}
	};
	
	PX_INLINE SArrayObjects createArrayObjects( const PvdCommLayerDatatype*	inDatatypes
												, PxU32 inPropertyCount
												, PxU32 inStride = 0
												, PxU32 inValueCount = 0
												, const PxU8* inValues = 0 ) 
	{ 
		SArrayObjects retval = { inDatatypes, inPropertyCount, inStride, inValueCount, inValues };
		return retval;
	}

	struct SetNamespace
	{
		const char*	mNamespace;

		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::SetNamespace; }

		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream )
		{
			ioStream.streamify( mNamespace );
		}

		template<typename TOperator>
		PX_INLINE bool compare( const SetNamespace& inOther, TOperator inOperator ) const
		{
			return inOperator( mNamespace, inOther.mNamespace );
		}
	};

	PX_INLINE SetNamespace createSetNamespace( const char* inNamespace ) { SetNamespace retval = { inNamespace }; return retval; }
	
	struct PushNamespace
	{
		PxU32 unused;
		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::PushNamespace; }
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType&)
		{
		}
		template<typename TOperator>
		PX_INLINE bool compare( const PushNamespace&, TOperator ) const
		{
			return true;
		}
	};
	PX_INLINE PushNamespace createPushNamespace() { PushNamespace retval = { 0 }; return retval; }
	
	struct PopNamespace
	{
		PxU32 unused;
		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::PopNamespace; }
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType&)
		{
		}
		template<typename TOperator>
		PX_INLINE bool compare( const PopNamespace&, TOperator ) const
		{
			return true;
		}
	};
	PX_INLINE PopNamespace createPopNamespace() { PopNamespace retval = { 0 }; return retval; }
	

	struct RegisterPropertyStruct
	{
		PxU32						mKey;
		PxU32						mClass;
		PxU32						mStructByteSize;
		PxU32						mPropertyCount;
		PropertyStructEntry*		mEntries;

		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::RegisterPropertyStruct; }

		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& stream)
		{
			stream.streamify( mKey );
			stream.streamify( mClass );
			stream.streamify( mStructByteSize );
			stream.streamify( mEntries, mPropertyCount );
		}

		template<typename TOperator>
		PX_INLINE bool compare( const RegisterPropertyStruct& inOther, TOperator inOperator ) const
		{
			bool equal = inOperator( mKey, inOther.mKey )
				&& inOperator( mClass, inOther.mClass )
				&& inOperator( mStructByteSize, inOther.mStructByteSize )
				&& inOperator( mPropertyCount, inOther.mPropertyCount );
				
			if ( equal )
			{
				for ( PxU32 idx = 0; idx < mPropertyCount && equal; ++idx )
					equal = mEntries[idx].compare( inOther.mEntries[idx], inOperator );
			}
			return equal;
		}
	};

	PX_INLINE RegisterPropertyStruct createRegisterPropertyStruct( PxU32 inKey, PxU32 classId, PxU32 structByteSize, PxU32 propCount, PropertyStructEntry* inEntries )
	{ 
		RegisterPropertyStruct retval = { inKey, classId, structByteSize, propCount, inEntries };
		return retval;
	}
	
	struct RegisterPropertyStructKey
	{
		PxU32 mStruct;
		PxU32 mNamespace;
		RegisterPropertyStructKey( PxU32 sname, PxU32 ns )
			: mStruct( sname )
			, mNamespace( ns )
		{
		}
		bool operator==( const RegisterPropertyStructKey& other ) const
		{
			return mStruct == other.mStruct
				&& mNamespace == other.mNamespace;
		}
	};

	struct RegisterPropertyStructKeyHasher
	{
		PxU32 operator()( const RegisterPropertyStructKey& key )
		{
			return physx::shdfnd::hash( key.mStruct ) ^ physx::shdfnd::hash( key.mNamespace );
		}

		bool operator()( const RegisterPropertyStructKey& lhs, const RegisterPropertyStructKey& rhs )
		{
			return lhs == rhs;
		}
	};

	template<typename TAllocator>
	struct RegisterPropertyStructEntry
	{
		PxU32 mClass;
		PxU32 mByteSize;
		Array<PropertyStructEntry,TAllocator> mEntries;
		Array<PxU32,TAllocator> mStringOffsets;

		RegisterPropertyStructEntry()
		{
		}

		void setup( PxU32 cls, PxU32 bs, const PropertyStructEntry* entries, PxU32 entryCount )
		{
			mClass = cls;
			mByteSize = bs;
			mEntries.clear();
			mStringOffsets.clear();
			//Record where the string offsets lie.  This allows us to 
			//efficiently send string properties.
			for ( PxU32 idx = 0; idx < entryCount; ++idx )
			{
				mEntries.pushBack( entries[idx] );
				if ( entries[idx].mType == PvdCommLayerDatatype::String )
					mStringOffsets.pushBack( entries[idx].mOffset );
			}
		}
	};

	struct SendPropertyStruct
	{
		PxU64					mInstanceId;
		PxU32					mStructId;
		PxU32					mClass;
		PropertyStructEntry*	mEntries;
		PxU32					mEntryCount;
		PxU32*					mStrings;
		PxU32					mStringCount;
		const PxU8*				mData;
		PxU32					mDataLen;

		PX_INLINE PvdConnectionEventType getEventType() const { return PvdConnectionEventType::SendPropertyStruct; }

		template<typename TStreamType>
		PX_INLINE void streamifyStart(TStreamType& stream )
		{
			//Only send what we absolutely have to across the wire.
			stream.streamify( mInstanceId );
			stream.streamify( mStructId );
		}
		
		template<typename TStreamType>
		PX_INLINE void readEnd(TStreamType& stream, bool inIs64Bit )
		{
			stream.bulkStreamify( mData, mDataLen );
			//Perform byte swapping and update the data ptr to point to 
			//the next location.
			stream.endianConvert( mData, mEntries, mEntryCount, inIs64Bit );
			for ( PxU32 idx = 0; idx < mStringCount; ++idx )
			{
				//The data is a ptr to a ptr to string and point the pointer result
				//at this offset.
				stream.streamifyEmbeddedString( mData, mDataLen, mStrings[idx] );
			}
		}



		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& stream)
		{
			streamifyStart( stream );
			stream.bulkStreamify( mData, mDataLen );
			for ( PxU32 idx = 0; idx < mStringCount; ++idx )
			{
				//The data is a ptr to a ptr to string and point the pointer result
				//at this offset.
				stream.streamifyEmbeddedString( mData, mDataLen, mStrings[idx] );
			}
		}

		template<typename TOperator>
		PX_INLINE bool compare( const SendPropertyStruct& inOther, TOperator inOperator ) const
		{
			bool equal = inOperator( mInstanceId, inOther.mInstanceId )
				&& inOperator( mStructId, inOther.mStructId )
				&& inOperator( mClass, inOther.mClass )
				&& inOperator( mEntryCount, inOther.mEntryCount )
				&& inOperator( mStringCount, inOther.mStringCount )
				&& inOperator( mDataLen, inOther.mDataLen );

			if ( equal )
			{
				for ( PxU32 idx = 0; idx < mEntryCount && equal; ++idx )
					equal = mEntries[idx].compare( inOther.mEntries[idx], inOperator );
				for ( PxU32 idx = 0; idx < mStringCount && equal; ++idx )
					equal = inOperator( mStrings[idx], inOther.mStrings[idx] );
			}
			return equal;
		}
	};

	inline SendPropertyStruct CreateSendPropertyStruct( PxU64 inInstanceId
												, PxU32 inStructId
												, PxU32	inClass
												, PropertyStructEntry* inEntries
												, PxU32 inEntryCount
												, PxU32* inStrings
												, PxU32	inStringCount
												, const PxU8* inData
												, PxU32	inDataLen )
	{
		SendPropertyStruct retval = { inInstanceId, inStructId, inClass, inEntries, inEntryCount, inStrings, inStringCount, inData, inDataLen };
		return retval;
	}

	template<typename TEventType>
	PX_INLINE PxU8 getEventType( const TEventType& inType ) { return inType.getEventType().mEventType; }

	template<typename TRenderCommandType>
	PX_INLINE PxU8 getRenderCommandEventType()
	{
		return 0x80 + getRenderCommandType<TRenderCommandType>().mType;
	}

	template<> PX_INLINE PxU8 getEventType( const PushRenderState& ) { return getRenderCommandEventType<PushRenderState>(); }
	template<> PX_INLINE PxU8 getEventType( const PopRenderState& ) { return getRenderCommandEventType<PopRenderState>(); }
	template<> PX_INLINE PxU8 getEventType( const SetCurrentInstance& ) { return getRenderCommandEventType<SetCurrentInstance>(); }
	template<> PX_INLINE PxU8 getEventType( const SetCurrentColor& ) { return getRenderCommandEventType<SetCurrentColor>(); }
	template<> PX_INLINE PxU8 getEventType( const SetCurrentRenderFlags& ) { return getRenderCommandEventType<SetCurrentRenderFlags>(); }
	template<> PX_INLINE PxU8 getEventType( const AddCurrentRenderFlags& ) { return getRenderCommandEventType<AddCurrentRenderFlags>(); }
	template<> PX_INLINE PxU8 getEventType( const RemoveCurrentRenderFlags& ) { return getRenderCommandEventType<RemoveCurrentRenderFlags>(); }
	template<> PX_INLINE PxU8 getEventType( const SetCurrentTextScale& ) { return getRenderCommandEventType<SetCurrentTextScale>(); }
	template<> PX_INLINE PxU8 getEventType( const PushTransform& ) { return getRenderCommandEventType<PushTransform>(); }
	template<> PX_INLINE PxU8 getEventType( const SetTransform& ) { return getRenderCommandEventType<SetTransform>(); }
	template<> PX_INLINE PxU8 getEventType( const MultiplyTransform& ) { return getRenderCommandEventType<MultiplyTransform>(); }
	template<> PX_INLINE PxU8 getEventType( const PopTransform& ) { return getRenderCommandEventType<PopTransform>(); }
	template<> PX_INLINE PxU8 getEventType( const DrawPrimitive& ) { return getRenderCommandEventType<DrawPrimitive>(); }
	template<> PX_INLINE PxU8 getEventType( const DrawRenderStatePrimitive& ) { return getRenderCommandEventType<DrawRenderStatePrimitive>(); }

	union PvdConnectionEventDataUnion
	{
		SStreamInitialization		mStreamInitialization;
		SCreateClass				mCreateClass;
		SDeriveClass				mDeriveClass;
		SDefineProperty				mDefineProperty;
		SDefineArrayProperty		mDefineArrayProperty;
		SDefinePropertyOnInstance	mDefinePropertyOnInstance;
		SDefineBitflagNames			mDefineBitflagNames;
		SDefineEnumerationNames		mDefineEnumerationNames;
		SCreateInstance				mCreateInstance;
		SSetPropertyValue			mSetPropertyValue;
		SSetPropertyMediumValue		mSetPropertyMediumValue;
		SSetPropertySmallValue		mSetPropertySmallValue;
		SBeginPropertyBlock			mBeginPropertyBlock;
		SSendPropertyBlock			mSendPropertyBlock;
		SEndPropertyBlock			mEndPropertyBlock;
		SAddChild					mAddChild;
		SRemoveChild				mRemoveChild;
		SDestroyInstance			mDestroyInstance;
		SBeginArrayBlock			mBeginArrayBlock;
		SBeginArrayPropertyBlock	mBeginArrayPropertyBlock;
		SArrayObject				mArrayObject;
		SArrayObjects				mArrayObjects;
		SEndArrayBlock				mEndArrayBlock;
		SEndArrayPropertyBlock		mEndArrayPropertyBlock;
		SNamedEvent					mNamedEvent;
		SError						mError;
		SWarning					mWarning;
		SBeginSection				mBeginSection;
		SEndSection					mEndSection;
		SDisconnect					mDisconnect;
		SNamedEventWithInstance		mNamedEventWithInstance;
		SRemoveAllChildren			mRemoveAllChildren;
		RenderCommandData			mRenderCommand;
		SetNamespace				mSetNamespace;
		PushNamespace				mPushNamespace;
		PopNamespace				mPopNamespace;
		RegisterPropertyStruct		mRegisterPropertyStruct;
		SendPropertyStruct			mSendPropertyStruct;
		SRawPropertyBlock			mRawPropertyBlock;
	};
	
	template<typename TDataType>
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const TDataType&) { PX_ASSERT(false); return PvdConnectionEventDataUnion(); }
	//Packing the data up into the union.
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SStreamInitialization& inData ) { PvdConnectionEventDataUnion retval; retval.mStreamInitialization = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SCreateClass& inData ) { PvdConnectionEventDataUnion retval; retval.mCreateClass = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SDeriveClass& inData ) { PvdConnectionEventDataUnion retval; retval.mDeriveClass = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SDefineProperty& inData ) { PvdConnectionEventDataUnion retval; retval.mDefineProperty = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SDefineArrayProperty& inData ) { PvdConnectionEventDataUnion retval; retval.mDefineArrayProperty = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SDefinePropertyOnInstance& inData ) { PvdConnectionEventDataUnion retval; retval.mDefinePropertyOnInstance = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SDefineBitflagNames& inData ) { PvdConnectionEventDataUnion retval; retval.mDefineBitflagNames = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SDefineEnumerationNames& inData ) { PvdConnectionEventDataUnion retval; retval.mDefineEnumerationNames = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SCreateInstance& inData ) { PvdConnectionEventDataUnion retval; retval.mCreateInstance = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SSetPropertyValue& inData ) { PvdConnectionEventDataUnion retval; retval.mSetPropertyValue = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SSetPropertyMediumValue& inData ) { PvdConnectionEventDataUnion retval; retval.mSetPropertyMediumValue = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SSetPropertySmallValue& inData ) { PvdConnectionEventDataUnion retval; retval.mSetPropertySmallValue = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SBeginPropertyBlock& inData ) { PvdConnectionEventDataUnion retval; retval.mBeginPropertyBlock = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SSendPropertyBlock& inData ) { PvdConnectionEventDataUnion retval; retval.mSendPropertyBlock = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SEndPropertyBlock& inData ) { PvdConnectionEventDataUnion retval; retval.mEndPropertyBlock = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SAddChild& inData ) { PvdConnectionEventDataUnion retval; retval.mAddChild = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SRemoveChild& inData ) { PvdConnectionEventDataUnion retval; retval.mRemoveChild = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SDestroyInstance& inData ) { PvdConnectionEventDataUnion retval; retval.mDestroyInstance = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SBeginArrayBlock& inData ) { PvdConnectionEventDataUnion retval; retval.mBeginArrayBlock = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SBeginArrayPropertyBlock& inData ) { PvdConnectionEventDataUnion retval; retval.mBeginArrayPropertyBlock = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SArrayObject& inData ) { PvdConnectionEventDataUnion retval; retval.mArrayObject = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SArrayObjects& inData ) { PvdConnectionEventDataUnion retval; retval.mArrayObjects = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SEndArrayBlock& inData ) { PvdConnectionEventDataUnion retval; retval.mEndArrayBlock = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SEndArrayPropertyBlock& inData ) { PvdConnectionEventDataUnion retval; retval.mEndArrayPropertyBlock = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SNamedEvent& inData ) { PvdConnectionEventDataUnion retval; retval.mNamedEvent = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SError& inData ) { PvdConnectionEventDataUnion retval; retval.mError = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SWarning& inData ) { PvdConnectionEventDataUnion retval; retval.mWarning = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SBeginSection& inData ) { PvdConnectionEventDataUnion retval; retval.mBeginSection = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SEndSection& inData ) { PvdConnectionEventDataUnion retval; retval.mEndSection = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SDisconnect& inData ) { PvdConnectionEventDataUnion retval; retval.mDisconnect = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SNamedEventWithInstance& inData ) { PvdConnectionEventDataUnion retval; retval.mNamedEventWithInstance = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SRemoveAllChildren& inData ) { PvdConnectionEventDataUnion retval; retval.mRemoveAllChildren = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const RenderCommandData& inData ) { PvdConnectionEventDataUnion retval; retval.mRenderCommand = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SetNamespace& inData ) { PvdConnectionEventDataUnion retval; retval.mSetNamespace = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const PushNamespace& inData ) { PvdConnectionEventDataUnion retval; retval.mPushNamespace = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const PopNamespace& inData ) { PvdConnectionEventDataUnion retval; retval.mPopNamespace = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const RegisterPropertyStruct& inData ) { PvdConnectionEventDataUnion retval; retval.mRegisterPropertyStruct = inData; return retval; }
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionEventData( const SendPropertyStruct& inData ) { PvdConnectionEventDataUnion retval; retval.mSendPropertyStruct = inData; return retval; }

	template<typename TDataType>
	PX_INLINE PvdConnectionEventDataUnion doCreatePVDConnectionRenderEventData( const TDataType& inData ) { PvdConnectionEventDataUnion retval; retval.mRenderCommand = toRenderCommandData( inData ); return retval; }

	template<typename TDataType>
	PX_INLINE PvdConnectionEventDataUnion createPVDConnectionEventData( const TDataType& inType )
	{
		if ( isRenderCommand<TDataType>() )
			return doCreatePVDConnectionRenderEventData( inType );
		return doCreatePVDConnectionEventData( inType );
	}
	

	//Unpacking the data from the union.
	template<typename TDataType>
	PX_INLINE TDataType doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& /*inData*/ ) 
	{  
		PX_ASSERT( false );
		return TDataType();
	}
	
	template<> PX_INLINE SStreamInitialization		doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mStreamInitialization; }
	template<> PX_INLINE SCreateClass				doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mCreateClass; }
	template<> PX_INLINE SDeriveClass				doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mDeriveClass; }
	template<> PX_INLINE SDefineProperty			doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mDefineProperty; }
	template<> PX_INLINE SDefineArrayProperty		doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mDefineArrayProperty; }
	template<> PX_INLINE SDefinePropertyOnInstance	doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mDefinePropertyOnInstance; }
	template<> PX_INLINE SDefineBitflagNames		doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mDefineBitflagNames; }
	template<> PX_INLINE SDefineEnumerationNames	doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mDefineEnumerationNames; }
	template<> PX_INLINE SCreateInstance			doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mCreateInstance; }
	template<> PX_INLINE SSetPropertyValue			doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mSetPropertyValue; }
	template<> PX_INLINE SSetPropertyMediumValue	doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mSetPropertyMediumValue; }
	template<> PX_INLINE SSetPropertySmallValue		doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mSetPropertySmallValue; }
	template<> PX_INLINE SBeginPropertyBlock		doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mBeginPropertyBlock; }
	template<> PX_INLINE SSendPropertyBlock			doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mSendPropertyBlock; }
	template<> PX_INLINE SEndPropertyBlock			doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mEndPropertyBlock; }
	template<> PX_INLINE SAddChild					doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mAddChild; }
	template<> PX_INLINE SRemoveChild				doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mRemoveChild; }
	template<> PX_INLINE SDestroyInstance			doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mDestroyInstance; }
	template<> PX_INLINE SBeginArrayBlock			doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mBeginArrayBlock; }
	template<> PX_INLINE SBeginArrayPropertyBlock	doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mBeginArrayPropertyBlock; }
	template<> PX_INLINE SArrayObject				doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mArrayObject; }
	template<> PX_INLINE SArrayObjects				doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mArrayObjects; }
	template<> PX_INLINE SEndArrayBlock				doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mEndArrayBlock; }
	template<> PX_INLINE SEndArrayPropertyBlock		doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mEndArrayPropertyBlock; }
	template<> PX_INLINE SNamedEvent				doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mNamedEvent; }
	template<> PX_INLINE SError						doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mError; }
	template<> PX_INLINE SWarning					doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mWarning; }
	template<> PX_INLINE SBeginSection				doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mBeginSection; }
	template<> PX_INLINE SEndSection				doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mEndSection; }
	template<> PX_INLINE SDisconnect				doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mDisconnect; }
	template<> PX_INLINE SNamedEventWithInstance	doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mNamedEventWithInstance; }
	template<> PX_INLINE SRemoveAllChildren			doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mRemoveAllChildren; }
	template<> PX_INLINE RenderCommandData			doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mRenderCommand; }
	template<> PX_INLINE SetNamespace				doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mSetNamespace; }
	template<> PX_INLINE PushNamespace				doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mPushNamespace; }
	template<> PX_INLINE PopNamespace				doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mPopNamespace; }
	template<> PX_INLINE RegisterPropertyStruct		doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mRegisterPropertyStruct; }
	template<> PX_INLINE SendPropertyStruct			doGetPvdConnectionEvent( const PvdConnectionEventDataUnion& inData ) { return inData.mSendPropertyStruct; }

	template<typename TDataType> PX_INLINE TDataType doGetPvdConnectionRenderEvent( const PvdConnectionEventDataUnion& inData ) { return fromRenderCommandData<TDataType>( inData.mRenderCommand ); }

	template<typename TDataType>
	PX_INLINE TDataType getPVDConnectionEvent( const PvdConnectionEventDataUnion& inData )
	{
		if ( isRenderCommand<TDataType>() )
			return doGetPvdConnectionRenderEvent<TDataType>( inData );
		return doGetPvdConnectionEvent<TDataType>( inData );
	}

	//Now that we have all the types of the event stream
	//here is a function that will take an operator and based on the datatype
	//passed in call the operation with a default instance of each type.
	//Using this instead of your own switch statement reduces the
	//chances of error when we add new event types
	template< typename TReturnType, typename TOperator >
	inline TReturnType EventTypeBasedOperation( PvdConnectionEventType inType, const PvdConnectionEventDataUnion& inData, TOperator inOperator )
	{
		if ( inType.mEventType & 0x80 )
		{
			PxU8 theRenderCommandType = inType.mEventType & 0x7F;
			return visitRenderCommand<TReturnType, TOperator>( theRenderCommandType, inData.mRenderCommand, inOperator );
		}
		switch( inType.mEventType )
		{
		case PvdConnectionEventType::CreateClass:				return inOperator(inData.mCreateClass);
		case PvdConnectionEventType::DeriveClass:				return inOperator(inData.mDeriveClass);
		case PvdConnectionEventType::DefineProperty:			return inOperator(inData.mDefineProperty);
		case PvdConnectionEventType::DefineArrayProperty:		return inOperator(inData.mDefineArrayProperty);
		case PvdConnectionEventType::DefinePropertyOnInstance:	return inOperator(inData.mDefinePropertyOnInstance);
		case PvdConnectionEventType::DefineBitflagNames:		return inOperator(inData.mDefineBitflagNames);
		case PvdConnectionEventType::DefineEnumerationNames:	return inOperator(inData.mDefineEnumerationNames);
		case PvdConnectionEventType::CreateInstance:			return inOperator(inData.mCreateInstance);
		case PvdConnectionEventType::SetPropertyValue:			return inOperator(inData.mSetPropertyValue);
		case PvdConnectionEventType::SetPropertyMediumValue:	return inOperator(inData.mSetPropertyMediumValue);
		case PvdConnectionEventType::SetPropertySmallValue:		return inOperator(inData.mSetPropertySmallValue);
		case PvdConnectionEventType::BeginPropertyBlock:		return inOperator(inData.mBeginPropertyBlock);
		case PvdConnectionEventType::SendPropertyBlock:			return inOperator(inData.mSendPropertyBlock);
		case PvdConnectionEventType::EndPropertyBlock:			return inOperator(inData.mEndPropertyBlock);
		case PvdConnectionEventType::AddChild:					return inOperator(inData.mAddChild);
		case PvdConnectionEventType::RemoveChild:				return inOperator(inData.mRemoveChild);
		case PvdConnectionEventType::DestroyInstance:			return inOperator(inData.mDestroyInstance);
		case PvdConnectionEventType::BeginArrayBlock:			return inOperator(inData.mBeginArrayBlock);
		case PvdConnectionEventType::BeginArrayPropertyBlock:	return inOperator(inData.mBeginArrayPropertyBlock);
		case PvdConnectionEventType::ArrayObject:				return inOperator(inData.mArrayObject);
		case PvdConnectionEventType::ArrayObjects:				return inOperator(inData.mArrayObjects);
		case PvdConnectionEventType::EndArrayBlock:				return inOperator(inData.mEndArrayBlock);
		case PvdConnectionEventType::EndArrayPropertyBlock:		return inOperator(inData.mEndArrayPropertyBlock);
		case PvdConnectionEventType::NamedEvent:				return inOperator(inData.mNamedEvent);
		case PvdConnectionEventType::Error:						return inOperator(inData.mError);
		case PvdConnectionEventType::Warning:					return inOperator(inData.mWarning);
		case PvdConnectionEventType::BeginSection:				return inOperator(inData.mBeginSection);
		case PvdConnectionEventType::EndSection:				return inOperator(inData.mEndSection);
		case PvdConnectionEventType::Disconnect:				return inOperator(inData.mDisconnect);
		case PvdConnectionEventType::NamedEventWithInstance:	return inOperator(inData.mNamedEventWithInstance);
		case PvdConnectionEventType::RemoveAllChildren:			return inOperator(inData.mRemoveAllChildren);
		case PvdConnectionEventType::SetNamespace:				return inOperator(inData.mSetNamespace);
		case PvdConnectionEventType::PushNamespace:				return inOperator(inData.mPushNamespace);
		case PvdConnectionEventType::PopNamespace:				return inOperator(inData.mPopNamespace);
		case PvdConnectionEventType::RegisterPropertyStruct:	return inOperator(inData.mRegisterPropertyStruct);
		case PvdConnectionEventType::SendPropertyStruct:		return inOperator(inData.mSendPropertyStruct);
		case PvdConnectionEventType::RawPropertyBlock:			return inOperator(inData.mRawPropertyBlock);
		default:
			PX_ASSERT( false ); //The event type is bad.
			return inOperator();
		}
	}

	template< typename TReturnType, typename TOperator >
	inline TReturnType EventTypeBasedOperation( PvdConnectionEventType inType, TOperator inOperator )
	{
		return EventTypeBasedOperation<TReturnType,TOperator>( inType, PvdConnectionEventDataUnion(), inOperator );
	}

	//Descriminated union implementation.
	class PvdConnectionEventData
	{
		PvdConnectionEventType mEventType;
		PvdConnectionEventDataUnion mData;
	public:
		PX_INLINE PvdConnectionEventData()
		{
		}
		PX_INLINE PvdConnectionEventData( const PvdConnectionEventData& inOther )
			: mEventType( inOther.mEventType )
			, mData( inOther.mData )
		{
		}
		PX_INLINE PvdConnectionEventData& operator=( const PvdConnectionEventData& inOther )
		{
			mEventType = inOther.mEventType;
			mData = inOther.mData;
			return *this; 
		}
		PX_INLINE PvdConnectionEventData( PvdConnectionEventType et, PvdConnectionEventDataUnion d )
			: mEventType( et )
			, mData( d )
		{
		}
		template<typename TDataType>
		PX_INLINE PvdConnectionEventData( const TDataType& inData )
			: mEventType( PVD::getEventType( inData ) )
			, mData( createPVDConnectionEventData( inData ) )
		{
		}
		PX_INLINE PvdConnectionEventType getEventType() const { return mEventType; }
		PX_INLINE PvdConnectionEventDataUnion getEventData() const { return mData; }
		template<typename TDataType> 
		PX_INLINE TDataType getEventValue() const 
		{ 
			PX_ASSERT( mEventType == PVD::getEventType( TDataType() ) ); 
			return getPVDConnectionEvent<TDataType>( mData ); 
		}
		template<typename TReturnType, typename TOperator>
		PX_INLINE TReturnType visit( TOperator inOperator ) const
		{
			return EventTypeBasedOperation<TReturnType, TOperator>( mEventType, mData, inOperator );
		}
	};
}

#endif

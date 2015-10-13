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

#ifndef PVD_PVDCOMMLAYERDATATYPES_H
#define PVD_PVDCOMMLAYERDATATYPES_H

#include "PxSimpleTypes.h"
#include "PxAssert.h"
#include "PxQuat.h"
#include "PxBounds3.h"
#include "PxVec2.h"
#include "PxVec4.h"

namespace physx { 
	struct PxFilterData;
	namespace shdfnd3{}
	namespace pubfnd3{}
}

namespace PVD
{

	using namespace physx::pubfnd3;
	using namespace physx::shdfnd3;
	//Helper functions used below.

	class StringHash
	{
	public:
		PxU32 operator()(const char *string) const
		{
			// "DJB" string hash 
			PxU32 h = 5381;
			for(const char *ptr = string; *ptr; ptr++)
				h = ((h<<5)+h)^*ptr;
			return h;
		}
		bool operator()(const char* string0, const char* string1) const
		{
			return !strcmp(string0, string1);
		}
	};
	

	//Strlen that can handle NULL;
	PX_INLINE PxU32 SafeStrLen( const char* inData ) { return inData ? static_cast< PxU32> ( strlen( inData ) ) : 0; }
	PX_INLINE PxI32 SafeStrCompare( const char* inLhs, const char* inRhs ) 
	{ 
		if ( inLhs && inRhs )
			return strcmp( inLhs, inRhs );
		if ( inLhs || inRhs )
			return 1;
		return 0;
	}
	PX_INLINE PxI32 SafeStrEqual( const char* inLhs, const char* inRhs ) 
	{
		return SafeStrCompare( inLhs, inRhs ) == 0;
	}
	
	PX_INLINE PxU32 HashFunction(const char * const input)	
	{
		StringHash theHasher;
		return theHasher(input);
	}
	
	PX_INLINE PxU32 safeStrHash( const char* const input )
	{
		const char* theData = input;
		if ( !input )
			theData = "";
		return HashFunction( theData );
	}

	/**
	 *	The datatype list is meant to be exhaustive.  When it isn't, using
	 *	a base type along with a semantic should allow us to work
	 *	around it until a newer version comes out.
	 *	
	 *	When adding datatypes to this list you cannot change the order of the types.
	 *	So you can't insert types into the middle, but you need to add them at the end
	 *	because this enumeration is sent through the data stream casted to an integer.
	 */
	struct PvdCommLayerDatatype
	{
		enum Enum
		{
			Unknown = 0,
			Boolean,
			U8,
			U16,
			U32,
			U64,
			I8,
			I16,
			I32,
			I64,
			Float,
			Double,
			Float3,
			Position,
			Velocity,
			Direction,
			Float4,
			Plane,
			Quat,
			Float6,
			Bounds3,
			Float9,
			Mat33,
			Float12,
			Frame,
			ObjectId,
			String,
			Bitflag,
			FloatBuffer,
			U32Buffer,
			EnumerationValue,
			Float7,
			Transform, //PxTransform
			U32Array4,
			FilterData, //PxFilterData
			HeightFieldSample, //PxHeightFieldSample
			Stream, //Stream property type for send endless streams of data
			//Section properties are begin/end pairs that denote time ranges. 
			//These are relatively expensive compared to profiling events, so
			//they are inappropriate for sections of code but more for scene
			//level sections of activity.
			//Finally, you can't use these datatypes in arrays; they require
			//appended information on the sending side (the timestamp).
			Section,
			Pointer, //not mapped precisely to a property but used in a property struct
			Float2,
			Last,
		};
		PxU8 mDatatype;
		PX_INLINE PvdCommLayerDatatype( PxU8 inDatatype = Unknown ) : mDatatype( inDatatype ) {}
		PX_INLINE bool operator==( const PvdCommLayerDatatype& inOther ) const { return mDatatype == inOther.mDatatype; }
		PX_INLINE bool operator!=( const PvdCommLayerDatatype& inOther ) const { return !(mDatatype == inOther.mDatatype); }
		inline const char* toString() const
		{
			switch( mDatatype )
			{
			case Unknown: return "Unknown";
			case Boolean: return "Boolean";
			case U8: return "U8";
			case U16: return "U16";
			case U32: return "U32";
			case U64: return "U64";
			case I8: return "I8";
			case I16: return "I16";
			case I32: return "I32";
			case I64: return "I64";
			case Float: return "Float";
			case Double: return "Double";
			case Float3: return "Float3";
			case Position: return "Position";
			case Velocity: return "Velocity";
			case Direction: return "Direction";
			case Float4: return "Float4";
			case Plane: return "Plane";
			case Quat: return "Quat";
			case Float6: return "Float6";
			case Bounds3: return "Bounds3";
			case Float9: return "Float9";
			case Mat33: return "Mat33";
			case Float12: return "Float12";
			case Frame: return "Frame";
			case ObjectId: return "ObjectId";
			case String: return "String";
			case Bitflag: return "Bitflag";
			case FloatBuffer: return "FloatBuffer";
			case U32Buffer: return "U32Buffer";
			case EnumerationValue: return "EnumerationValue";
			case Float7: return "Float7";
			case Transform: return "Transform";
			case U32Array4: return "U32Array4";
			case FilterData: return "FilterData";
			case Stream: return "Stream";
			case Section: return "Section";
			}
			return "";
		}
	};

	struct NamedValueDefinition
	{
		const char* mName;
		//Value must only contain the high bits.
		PxU32		mValue;
	};

	struct PropertyStructEntry
	{
		PxU32					mProperty;
		PxU32					mOffset;
		PxU32					mType;
		PropertyStructEntry( PxU32 prop, PxU32 offset, PxU32 type )
			: mProperty( prop )
			, mOffset( offset )
			, mType( type )
		{
			PX_COMPILE_TIME_ASSERT( sizeof( PropertyStructEntry ) == 3 * sizeof( PxU32 ) );
		}
		
		template<typename TOperator>
		bool compare( const PropertyStructEntry& inOther, TOperator inOperator ) const
		{
			return inOperator( mProperty, inOther.mProperty )
				&& inOperator( mOffset, inOther.mOffset )
				&& inOperator( mType, inOther.mType );
		}
	};

	///////////////////////////////////////////////////////////////////
	//	Extended types used as communication datatypes.  As these are all
	//	used in the union below, they can't have constructors nor equal
	//	operators.
	///////////////////////////////////////////////////////////////////

	/**
	 *	Array types of fixed length.
	 */
	template<PxU32 TNumFloats>
	struct TFixedFloatArray
	{
		PxF32 mFloats[TNumFloats];
		PX_INLINE void set(PxF32 inConstant = 0.f ) { for ( PxU32 idx =0; idx < TNumFloats; ++idx ) { mFloats[idx] = inConstant; } }
		PX_INLINE const PxF32& operator[](int inIdx ) const { return mFloats[inIdx]; }
		PX_INLINE PxF32& operator[](int inIdx ) { return mFloats[inIdx]; }
		PX_INLINE void batchSet( const PxF32* inSource ) { for ( PxU32 idx =0; idx < TNumFloats; ++idx ) { mFloats[idx] = inSource[idx]; } }
		PX_INLINE bool operator==( const TFixedFloatArray<TNumFloats>& inOther ) const
		{
			for ( PxU32 idx =0; idx < TNumFloats; ++idx ) 
			{
				if ( mFloats[idx] != inOther.mFloats[idx] ) return false;
			}
			return true;
		}
	};

	struct Float2 : public TFixedFloatArray<2>
	{
		PX_INLINE void set( PxF32 inX = 0.f, PxF32 inY = 0.f )
		{
			mFloats[0] = inX;
			mFloats[1] = inY;
		}
		PX_INLINE operator PxVec2 () const { return PxVec2( mFloats[0], mFloats[1] ); }
	};
	PX_INLINE Float2 createFloat2( PxF32 inX = 0.f, PxF32 inY = 0.f ) { Float2 retval; retval.set( inX, inY ); return retval; }
	PX_INLINE Float2 createFloat2( const PxVec2& inVal ) { return createFloat2( inVal.x, inVal.y ); }

	struct Float3 : public TFixedFloatArray<3>
	{
		PX_INLINE void set( PxF32 inX = 0.f, PxF32 inY = 0.f, PxF32 inZ = 0.f )
		{
			mFloats[0] = inX;
			mFloats[1] = inY;
			mFloats[2] = inZ;
		}
		PX_INLINE operator PxVec3 () const { return PxVec3( mFloats[0], mFloats[1], mFloats[2] ); }
	};
	PX_INLINE Float3 createFloat3( PxF32 inX = 0.f, PxF32 inY = 0.f, PxF32 inZ = 0.f ) { Float3 retval; retval.set( inX, inY, inZ ); return retval; }
	PX_INLINE Float3 createFloat3( const PxVec3& inVal ) { return createFloat3( inVal.x, inVal.y, inVal.z ); }
	struct Position : public Float3 {};
	PX_INLINE Position createPosition( PxF32 inX = 0.f, PxF32 inY = 0.f, PxF32 inZ = 0.f ) { Position retval; retval.set( inX, inY, inZ ); return retval; }
	PX_INLINE Position createPosition( const PxVec3& inVal ) { return createPosition( inVal.x, inVal.y, inVal.z ); }
	struct Velocity : public Float3 {};
	PX_INLINE Velocity createVelocity( PxF32 inX = 0.f, PxF32 inY = 0.f, PxF32 inZ = 0.f ) { Velocity retval; retval.set( inX, inY, inZ ); return retval; }
	PX_INLINE Velocity createVelocity( const PxVec3& inVal ) { return createVelocity( inVal.x, inVal.y, inVal.z ); }
	struct Direction : public Float3 {};
	PX_INLINE Direction createDirection( PxF32 inX = 0.f, PxF32 inY = 0.f, PxF32 inZ = 0.f ) { Direction retval; retval.set( inX, inY, inZ ); return retval; }
	PX_INLINE Direction createDirection( const PxVec3& inVal ) { return createDirection( inVal.x, inVal.y, inVal.z ); }
	struct Float4 : public TFixedFloatArray<4>
	{
		PX_INLINE void set( PxF32 inX = 0.f, PxF32 inY = 0.f, PxF32 inZ = 0.f, PxF32 inW = 0.f )
		{
			mFloats[0] = inX;
			mFloats[1] = inY;
			mFloats[2] = inZ;
			mFloats[3] = inW;
		}
		PX_INLINE operator PxVec4 () const { return PxVec4( mFloats[0], mFloats[1], mFloats[2], mFloats[3] ); }
	};
	PX_INLINE Float4 createFloat4( PxF32 inX = 0.f, PxF32 inY = 0.f, PxF32 inZ = 0.f, PxF32 inW = 0.f ) { Float4 retval; retval.set( inX, inY, inZ, inW ); return retval; }
	PX_INLINE Float4 createFloat4( const PxVec4& inVal ) { return createFloat4( inVal.x, inVal.y, inVal.z, inVal.w ); }
	struct Plane : public Float4 {};
	PX_INLINE Plane createPlane( PxF32 inX = 0.f, PxF32 inY = 0.f, PxF32 inZ = 0.f, PxF32 inW = 0.f ) { Plane retval; retval.set( inX, inY, inZ, inW ); return retval; }
	struct Quat : public Float4 
	{
		PX_INLINE operator PxQuat () const { return PxQuat( mFloats[0], mFloats[1], mFloats[2], mFloats[3] ); }
	};
	PX_INLINE Quat createQuat( PxF32 inX = 0.f, PxF32 inY = 0.f, PxF32 inZ = 0.f, PxF32 inW = 0.f ) { Quat retval; retval.set( inX, inY, inZ, inW ); return retval; }
	PX_INLINE Quat createQuat( const PxQuat& inQuat ) { return createQuat( inQuat.x, inQuat.y, inQuat.z, inQuat.w ); }

	struct Float12 : public TFixedFloatArray<12> {};
	PX_INLINE Float12 createFloat12( const PxF32* inData ) { Float12 retval; retval.batchSet( inData ); return retval; }
	struct Frame : public Float12 {};
	PX_INLINE Frame createFrame( const float* inData ) { Frame retval; retval.batchSet( inData ); return retval; }

	struct Float9 : public TFixedFloatArray<9> {};
	PX_INLINE Float9 createFloat9( const PxF32* inData ) { Float9 retval; retval.batchSet( inData ); return retval; }

	struct Mat33 : public Float9 
	{
		PX_INLINE operator PxMat33 () { return reinterpret_cast< PxMat33& >( *this ); }
		PX_INLINE operator PxMat33 () const { return reinterpret_cast< const PxMat33& >( *this ); }
	};
	PX_INLINE Mat33 createMat33( const PxF32* inData ) { Mat33 retval; retval.batchSet( inData ); return retval; }
	PX_INLINE Mat33 createMat33( const PxMat33& inData ) { return reinterpret_cast< const Mat33& >( inData ); }

	struct Float6 : public TFixedFloatArray<6> {};
	PX_INLINE Float6 createFloat6( const PxF32* inData ) { Float6 retval; retval.batchSet( inData ); return retval; }
	
	struct Bounds3 : public Float6 
	{
		PX_INLINE operator PxBounds3 () const { return PxBounds3( PxVec3( mFloats[3], mFloats[4], mFloats[5] ), PxVec3( mFloats[0], mFloats[1], mFloats[2] ) ); }
	}; //The six values represent 3 xyz min values and 3 xyz max values
	PX_INLINE Bounds3 createBounds3( const PxF32* inData ) { Bounds3 retval; retval.batchSet( inData ); return retval; }
	PX_INLINE Bounds3 createBounds3( PxF32 minX, PxF32 minY, PxF32 minZ
									, PxF32 maxX, PxF32 maxY, PxF32 maxZ ) 
	{ 
		Bounds3 retval;
		retval[0] = maxX;
		retval[1] = maxY;
		retval[2] = maxZ;
		retval[3] = minX;
		retval[4] = minY;
		retval[5] = minZ;
		return retval;
	}
	PX_INLINE Bounds3 createBounds3( const PxBounds3& inValue )
	{
		return createBounds3( inValue.minimum.x, inValue.minimum.y, inValue.minimum.z,
							  inValue.maximum.x, inValue.maximum.y, inValue.maximum.z );
	}


	struct Float7 : public TFixedFloatArray<7> {};
	PX_INLINE Float7 createFloat7( const PxF32* inData ) { Float7 retval; retval.batchSet( inData ); return retval; }
	//A transform is a quaternion followed by a vec3, the quat is xyzw, the vector is xyz
	struct Transform : public Float7 
	{
		PX_INLINE operator PxTransform () const
		{
			return PxTransform( PxVec3( mFloats[4], mFloats[5], mFloats[6] )
							, PxQuat( mFloats[0], mFloats[1], mFloats[2], mFloats[3] ) );
		}
	};

	PX_INLINE Transform createTransform( const PxF32* inData ) { Transform retval; retval.batchSet( inData ); return retval; }
	PX_INLINE Transform createTransform( PxF32 quatX, PxF32 quatY, PxF32 quatZ, PxF32 quatW
											, PxF32 vecX, PxF32 vecY, PxF32 vecZ ) 
	{ 
		Transform retval;
		retval[0] = quatX;
		retval[1] = quatY;
		retval[2] = quatZ;
		retval[3] = quatW;
		retval[4] = vecX;
		retval[5] = vecY;
		retval[6] = vecZ;
		return retval;
	}

	PX_INLINE Transform createTransform( const PxTransform& inValue )
	{
		return createTransform( inValue.q.x, inValue.q.y, inValue.q.z, inValue.q.w, inValue.p.x, inValue.p.y, inValue.p.z );
	}

	template<PxU32 TNumData>
	struct TFixedU32Array
	{
		PxU32 mData[TNumData];
		PX_INLINE void set(PxU32 inConstant = 0 ) { for ( PxU32 idx =0; idx < TNumData; ++idx ) { mData[idx] = inConstant; } }
		PX_INLINE const PxU32& operator[](int inIdx ) const { return mData[inIdx]; }
		PX_INLINE PxU32& operator[](int inIdx ) { return mData[inIdx]; }
		PX_INLINE void batchSet( const PxU32* inSource ) { for ( PxU32 idx =0; idx < TNumData; ++idx ) { mData[idx] = inSource[idx]; } }
		PX_INLINE bool operator==( const TFixedU32Array<TNumData>& inOther ) const
		{
			for ( PxU32 idx =0; idx < TNumData; ++idx ) 
			{
				if ( mData[idx] != inOther.mData[idx] ) return false;
			}
			return true;
		}
	};

	struct U32Array4 : public TFixedU32Array<4> 
	{
		PX_INLINE void set( PxU32 in0=0, PxU32 in1=0, PxU32 in2=0, PxU32 in3=0 )
		{
			mData[0] = in0;
			mData[1] = in1;
			mData[2] = in2;
			mData[3] = in3;
		}
	};
	inline U32Array4 createU32Array4( PxU32 in0=0, PxU32 in1=0, PxU32 in2=0, PxU32 in3=0 )
	{
		U32Array4 retval;
		retval.set( in0, in1, in2, in3 );
		return retval;
	}
	struct FilterData : public U32Array4 {};
	
	inline FilterData createFilterData( PxU32 in0=0, PxU32 in1=0, PxU32 in2=0, PxU32 in3=0 )
	{
		FilterData retval;
		retval.set( in0, in1, in2, in3 );
		return retval;
	}

	/**
	 *	Generic wrappers over base datatypes.
	 */
	template<typename TDataType>
	struct GenericDatatype
	{
		TDataType mValue;
		bool operator==( const GenericDatatype& inOther ) const { return mValue == inOther.mValue; }
	};
	struct InstanceId : public GenericDatatype<PxU64> {};
	PX_INLINE InstanceId createInstanceId( PxU64 inId ) { InstanceId retval; retval.mValue = inId; return retval; }

	struct Bitflag : public GenericDatatype<PxU32> {};
	PX_INLINE Bitflag createBitflag( PxU32 inId ) { Bitflag retval; retval.mValue = inId; return retval; }

	struct EnumerationValue : public GenericDatatype<PxU32> {};
	PX_INLINE EnumerationValue createEnumerationValue( PxU32 inValue ) { EnumerationValue retval; retval.mValue = inValue; return retval; }

	struct String : public GenericDatatype<const char*> {};
	PX_INLINE String createString( const char* inData = NULL ){ String retval; retval.mValue = inData; return retval; }

	/**
	 *	Buffers for unbounded arrays of the basic types.
	 *	Currently we don't support PxU16 types but if there is
	 *	a need for it we should support them directly as
	 *	opposed to using attempting to put PxU16's into
	 *	PxU32 types as it screws up endian conversions.
	 */
	template< typename TDataType>
	struct Buffer
	{
		const TDataType* mData;
		PxU32 mLength;
		bool operator==( const Buffer& inOther ) const { return mData == inOther.mData && mLength == inOther.mLength; }
	};
	struct FloatBuffer : public Buffer<PxF32> {};
	PX_INLINE FloatBuffer createBuffer( const PxF32* inData = NULL, PxU32 inLen = 0 ) { FloatBuffer retval; retval.mData = inData; retval.mLength = inLen; return retval; }
	struct U32Buffer : public Buffer<PxU32> {};
	PX_INLINE U32Buffer createBuffer( const PxU32* inData = NULL, PxU32 inLen = 0 ) { U32Buffer retval; retval.mData = inData; retval.mLength = inLen; return retval; }

	struct HeightFieldSample //PxHeightFieldSample
	{
		PxI16			mHeight			: 16;
		PxU8			mMaterialIndex0	: 7;
		PxU8			mTessFlag		: 1;
		PxU8			mMaterialIndex1	: 7;
		PxU8			mUnused			: 1;
		bool operator==( const HeightFieldSample& inOther ) const 
		{ 
			return mHeight == inOther.mHeight 
				&& mMaterialIndex0 == inOther.mMaterialIndex0
				&& mTessFlag == inOther.mTessFlag
				&& mMaterialIndex1 == inOther.mMaterialIndex1;
		}
	};
	
	PX_INLINE HeightFieldSample createHeightFieldSample( PxI16 inHeight=0, PxU8 inMaterialIndex0=0, bool inTessFlag=false, PxU8 inMaterialIndex1=0 ) 
	{ HeightFieldSample retval = { inHeight, inMaterialIndex0, inTessFlag ? 1 : 0, inMaterialIndex1, 0 }; return retval; }
	
	struct StreamUpdate : public Buffer<PxU8> {};
	
	PX_INLINE StreamUpdate createStreamUpdate(const PxU8* inData, PxU32 inLen) 
	{ StreamUpdate retval; retval.mData = inData; retval.mLength = inLen; return retval; }

	struct SectionType
	{
		enum Enum
		{
			Unknown = 0,
			Begin = 1,
			End =2,
		};
	};

	struct Section
	{
		PxU64	mTimestamp;
		PxU8	mType;
		bool operator==( const Section& inOther ) const 
		{ 
			return mTimestamp == inOther.mTimestamp 
				&& mType == inOther.mType;
		}
	};

	PX_INLINE Section createSection(SectionType::Enum inType, PxU64	inTimestamp = 0 )
	{ Section retval = { inTimestamp, static_cast<PxU8>( inType ) }; return retval; }

	template<typename TDataType>
	struct PvdTypeToEnumMap { bool cause_compile_error; };
	template<PvdCommLayerDatatype::Enum>
	struct PvdEnumToTypeMap { bool cause_compile_error; };

#define MAP_PVD_ENUM_TYPE_TO_DATATYPE( datatype, enumType )																\
	template<> struct PvdTypeToEnumMap<datatype> { enum Enum { PvdEnumType = PvdCommLayerDatatype::enumType }; };		\
	template<> struct PvdEnumToTypeMap<PvdCommLayerDatatype::enumType> { typedef datatype TPvdDatatype; };

#define ALTERNATE_PVD_ENUM_TYPE_TO_DATATYPE( datatype, enumType ) \
	template<> struct PvdTypeToEnumMap<datatype> { enum Enum { PvdEnumType = PvdCommLayerDatatype::enumType }; };


	MAP_PVD_ENUM_TYPE_TO_DATATYPE( bool, Boolean );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( PxU8, U8 );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( PxU16, U16 );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( PxU32, U32 );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( PxU64, U64 );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( PxI8, I8 );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( PxI16, I16 );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( PxI32, I32 );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( PxI64, I64 );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( PxF32, Float );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( PxF64, Double );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( Float2, Float2 );
	ALTERNATE_PVD_ENUM_TYPE_TO_DATATYPE( PxVec2, Float2 );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( Float3, Float3 );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( Position, Position );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( Velocity, Velocity );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( Direction, Direction );
	ALTERNATE_PVD_ENUM_TYPE_TO_DATATYPE( PxVec3, Float3 );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( Float4, Float4 );
	ALTERNATE_PVD_ENUM_TYPE_TO_DATATYPE( PxVec4, Float4 );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( Plane, Plane );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( Quat, Quat );
	ALTERNATE_PVD_ENUM_TYPE_TO_DATATYPE( PxQuat, Quat );

	MAP_PVD_ENUM_TYPE_TO_DATATYPE( Float6, Float6 );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( Bounds3, Bounds3 );
	ALTERNATE_PVD_ENUM_TYPE_TO_DATATYPE( PxBounds3, Bounds3 );

	MAP_PVD_ENUM_TYPE_TO_DATATYPE( Float9, Float9 );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( Mat33, Mat33 );
	ALTERNATE_PVD_ENUM_TYPE_TO_DATATYPE( PxMat33, Mat33 );

	MAP_PVD_ENUM_TYPE_TO_DATATYPE( Float12, Float12 );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( Frame, Frame );

	MAP_PVD_ENUM_TYPE_TO_DATATYPE( InstanceId, ObjectId );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( String, String );
	ALTERNATE_PVD_ENUM_TYPE_TO_DATATYPE( const char *, String );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( Bitflag, Bitflag );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( FloatBuffer, FloatBuffer );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( U32Buffer, U32Buffer );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( EnumerationValue, EnumerationValue );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( Float7, Float7 );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( Transform, Transform );
	ALTERNATE_PVD_ENUM_TYPE_TO_DATATYPE( PxTransform, Transform );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( U32Array4, U32Array4 );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( FilterData, FilterData );
	ALTERNATE_PVD_ENUM_TYPE_TO_DATATYPE( physx::PxFilterData, FilterData );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( HeightFieldSample, HeightFieldSample );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( StreamUpdate, Stream );
	MAP_PVD_ENUM_TYPE_TO_DATATYPE( Section, Section );

	template<typename TDataType>
	PX_INLINE PvdCommLayerDatatype getDatatypeForType() { return static_cast<PvdCommLayerDatatype::Enum>( PvdTypeToEnumMap<TDataType>::PvdEnumType ); }
	
	/**
	 *	Mapping compiler types to enumeration types.
	 */
	PX_INLINE PvdCommLayerDatatype getDatatypeForValue() { return PvdCommLayerDatatype::Unknown; }
	template<typename TDataType>
	PX_INLINE PvdCommLayerDatatype getDatatypeForValue(const TDataType&) { return getDatatypeForType<TDataType>(); }
	PX_INLINE PvdCommLayerDatatype getDatatypeForValue(const char*) { return getDatatypeForType<const char*>(); }
}

#endif
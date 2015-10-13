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

#ifndef PVD_PVDCOMMLAYERSMALLVALUE_H
#define PVD_PVDCOMMLAYERSMALLVALUE_H

#include "PVDCommLayerDatatypes.h"

namespace PVD
{	
	/**
	 *	Union for unifying all the communication datatypes.  This
	 *	greatly simplifies any interface using the types.
	 *	Small values are at most 4 bytes.
	 */
	union PvdCommLayerSmallData
	{
		bool				mBool;
		PxU8				mU8;
		PxU16				mU16;
		PxU32				mU32;
		PxI8				mI8;
		PxI16				mI16;
		PxI32				mI32;
		PxF32				mF32;
		Bitflag				mBitflag;
		String				mString;
		EnumerationValue	mEnumerationValue;
		HeightFieldSample	mHeightFieldSample;
	};

	template<typename TDataType>
	PX_INLINE PvdCommLayerSmallData createCommLayerSmallData( TDataType) { PX_ASSERT( false ); PvdCommLayerSmallData retval; memset( &retval, 0, sizeof( retval ) ); return retval; }

	/**
	 *	Mapping compiler types to the union's datastructures.
	 */
	template<> PX_INLINE PvdCommLayerSmallData createCommLayerSmallData( bool inValue ) { PvdCommLayerSmallData retval; retval.mBool = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerSmallData createCommLayerSmallData( PxU8 inValue ) { PvdCommLayerSmallData retval; retval.mU8 = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerSmallData createCommLayerSmallData( PxU16 inValue ) { PvdCommLayerSmallData retval; retval.mU16 = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerSmallData createCommLayerSmallData( PxU32 inValue ) { PvdCommLayerSmallData retval; retval.mU32 = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerSmallData createCommLayerSmallData( PxI8 inValue ) { PvdCommLayerSmallData retval; retval.mI8 = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerSmallData createCommLayerSmallData( PxI16 inValue ) { PvdCommLayerSmallData retval; retval.mI16 = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerSmallData createCommLayerSmallData( PxI32 inValue ) { PvdCommLayerSmallData retval; retval.mI32 = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerSmallData createCommLayerSmallData( PxF32 inValue ) { PvdCommLayerSmallData retval; retval.mF32 = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerSmallData createCommLayerSmallData( Bitflag inValue ) { PvdCommLayerSmallData retval; retval.mBitflag = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerSmallData createCommLayerSmallData( String inValue ) { PvdCommLayerSmallData retval; retval.mString = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerSmallData createCommLayerSmallData( const char* inValue ) { PvdCommLayerSmallData retval; retval.mString = createString( inValue ); return retval; }
	template<> PX_INLINE PvdCommLayerSmallData createCommLayerSmallData( EnumerationValue inValue ) { PvdCommLayerSmallData retval; retval.mEnumerationValue = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerSmallData createCommLayerSmallData( HeightFieldSample inValue ) { PvdCommLayerSmallData retval; retval.mHeightFieldSample = inValue; return retval; }

	template<typename TDataType>
	PX_INLINE TDataType getCommLayerSmallData( const PvdCommLayerSmallData& /*inData*/ ) { PX_ASSERT( false ); return TDataType(); }
	
	template<> PX_INLINE bool				getCommLayerSmallData( const PvdCommLayerSmallData& inData ) { return inData.mBool; }
	template<> PX_INLINE PxU8				getCommLayerSmallData( const PvdCommLayerSmallData& inData ) { return inData.mU8; }
	template<> PX_INLINE PxU16				getCommLayerSmallData( const PvdCommLayerSmallData& inData ) { return inData.mU16; }
	template<> PX_INLINE PxU32				getCommLayerSmallData( const PvdCommLayerSmallData& inData ) { return inData.mU32; }
	template<> PX_INLINE PxI8				getCommLayerSmallData( const PvdCommLayerSmallData& inData ) { return inData.mI8; }
	template<> PX_INLINE PxI16				getCommLayerSmallData( const PvdCommLayerSmallData& inData ) { return inData.mI16; }
	template<> PX_INLINE PxI32				getCommLayerSmallData( const PvdCommLayerSmallData& inData ) { return inData.mI32; }
	template<> PX_INLINE PxF32				getCommLayerSmallData( const PvdCommLayerSmallData& inData ) { return inData.mF32; }
	template<> PX_INLINE Bitflag			getCommLayerSmallData( const PvdCommLayerSmallData& inData ) { return inData.mBitflag; }
	template<> PX_INLINE String				getCommLayerSmallData( const PvdCommLayerSmallData& inData ) { return inData.mString; }
	template<> PX_INLINE const char*		getCommLayerSmallData( const PvdCommLayerSmallData& inData ) { return inData.mString.mValue; }
	template<> PX_INLINE EnumerationValue	getCommLayerSmallData( const PvdCommLayerSmallData& inData ) { return inData.mEnumerationValue; }
	template<> PX_INLINE HeightFieldSample	getCommLayerSmallData( const PvdCommLayerSmallData& inData ) { return inData.mHeightFieldSample; }


	/**
	 *	Operate on the values in the union in a typesafe way.  When the union is extended you will
	 *	get compile errors, as opposed to perhaps silent runtime failures.
	 */
	template<typename TReturnType, typename TOperator>
	inline TReturnType commLayerSmallDataOperate( PvdCommLayerDatatype inDatatype, const PvdCommLayerSmallData& inData, TOperator inOperator )
	{
		switch( inDatatype.mDatatype )
		{
		case PvdCommLayerDatatype::Boolean:			return inOperator( inData.mBool );
		case PvdCommLayerDatatype::U8:					return inOperator( inData.mU8 );
		case PvdCommLayerDatatype::U16:				return inOperator( inData.mU16 );
		case PvdCommLayerDatatype::U32:				return inOperator( inData.mU32 );
		case PvdCommLayerDatatype::I8:					return inOperator( inData.mI8 );
		case PvdCommLayerDatatype::I16:				return inOperator( inData.mI16 );
		case PvdCommLayerDatatype::I32:				return inOperator( inData.mI32 );
		case PvdCommLayerDatatype::Float:				return inOperator( inData.mF32 );
		case PvdCommLayerDatatype::Bitflag:			return inOperator( inData.mBitflag );
		case PvdCommLayerDatatype::String:				return inOperator( inData.mString );
		case PvdCommLayerDatatype::EnumerationValue:	return inOperator( inData.mEnumerationValue );
		case PvdCommLayerDatatype::HeightFieldSample:	return inOperator( inData.mHeightFieldSample );
		}
		return inOperator();
	}
	
	template<typename TReturnType, typename TOperator>
	PX_INLINE TReturnType commLayerSmallDataOperate( PvdCommLayerDatatype inDatatype, TOperator inOperator )
	{
		return commLayerSmallDataOperate<TReturnType, TOperator>( inDatatype, PvdCommLayerSmallData(), inOperator );
	}

	struct SUPVDCommLayerSmallDataDefaulter
	{
		PX_INLINE PvdCommLayerSmallData operator()( bool ) { return createCommLayerSmallData( false ); }
		PX_INLINE PvdCommLayerSmallData operator()( PxU8 ) { return createCommLayerSmallData( (PxU8) 0 ); }
		PX_INLINE PvdCommLayerSmallData operator()( PxU16 ) { return createCommLayerSmallData( (PxU16)0 ); }
		PX_INLINE PvdCommLayerSmallData operator()( PxU32 ) { return createCommLayerSmallData( (PxU32)0 ); }
		PX_INLINE PvdCommLayerSmallData operator()( PxI8 ) { return createCommLayerSmallData( (PxI8)0 ); }
		PX_INLINE PvdCommLayerSmallData operator()( PxI16 ) { return createCommLayerSmallData( (PxI16)0 ); }
		PX_INLINE PvdCommLayerSmallData operator()( PxI32 ) { return createCommLayerSmallData( (PxI32)0 ); }
		PX_INLINE PvdCommLayerSmallData operator()( PxF32 ) { return createCommLayerSmallData( (PxF32)0 ); }
		PX_INLINE PvdCommLayerSmallData operator()( Bitflag ) { return createCommLayerSmallData( createBitflag( 0 ) ); }
		PX_INLINE PvdCommLayerSmallData operator()( String ) { return createCommLayerSmallData( createString( NULL ) ); }
		PX_INLINE PvdCommLayerSmallData operator()( EnumerationValue ) { return createCommLayerSmallData( createEnumerationValue( 0 ) ); }
		PX_INLINE PvdCommLayerSmallData operator()( HeightFieldSample ) { return createCommLayerSmallData( createHeightFieldSample() ); }
		PX_INLINE PvdCommLayerSmallData operator()() { PvdCommLayerSmallData retval; memset( &retval, 0, sizeof( retval ) ); return retval; }
	};

	/**
	 *	Defaults means initialized to zero.
	 */
	PX_INLINE PvdCommLayerSmallData getDefaultSmallDataValue( PvdCommLayerDatatype inDatatype )
	{
		return commLayerSmallDataOperate<PvdCommLayerSmallData>( inDatatype, SUPVDCommLayerSmallDataDefaulter() );
	}


	/**
	 *	Implementation of the discriminated union to house the types.
	 */
	class PvdCommLayerSmallValue
	{
		PvdCommLayerDatatype		mDatatype;
		PvdCommLayerSmallData		mValue;

	public:
		PX_INLINE PvdCommLayerSmallValue() : mDatatype(PvdCommLayerDatatype::Unknown) {}

		PX_INLINE PvdCommLayerSmallValue( const PvdCommLayerSmallValue& inOther )
			: mDatatype(inOther.mDatatype)
			, mValue(inOther.mValue){}

		template<typename TDataType>
		PX_INLINE PvdCommLayerSmallValue( const TDataType& inValue )
			: mDatatype( getDatatypeForValue(inValue) )
			, mValue( createCommLayerSmallData(inValue) ){}

		PX_INLINE PvdCommLayerSmallValue( const PvdCommLayerDatatype& inType )
			: mDatatype( inType )
			, mValue( getDefaultSmallDataValue(inType) ){}
		
		PX_INLINE PvdCommLayerSmallValue( const PvdCommLayerDatatype& inType, const PvdCommLayerSmallData& inValue )
			: mDatatype( inType )
			, mValue( inValue ){}

		PX_INLINE PvdCommLayerSmallValue& operator=( const PvdCommLayerSmallValue& inOther )
		{
			mDatatype = inOther.mDatatype;
			mValue = inOther.mValue;
			return *this;
		}
		PX_INLINE PvdCommLayerDatatype		getDatatype() const { return mDatatype; }
		PX_INLINE PvdCommLayerSmallData	getData() const { return mValue; }

		template<typename TDataType> 
		PX_INLINE TDataType getValue() const { PX_ASSERT( mDatatype == getDatatypeForValue(TDataType()) ); return getCommLayerSmallData<TDataType>(mValue); }

		template<typename TReturnValue, typename TOperator>
		PX_INLINE TReturnValue visit(TOperator inOperator) const { return commLayerSmallDataOperate<TReturnValue, TOperator>( mDatatype, mValue, inOperator ); }
	};

}

#endif
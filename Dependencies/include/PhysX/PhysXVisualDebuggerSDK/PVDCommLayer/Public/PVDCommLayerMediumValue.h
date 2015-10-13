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

#ifndef PVD_PVDCOMMLAYERMEDIUMVALUE_H
#define PVD_PVDCOMMLAYERMEDIUMVALUE_H

#include "PVDCommLayerDatatypes.h"
#include "PxAssert.h"
#include "PVDCommLayerSmallValue.h"

namespace PVD
{
	
	/**
	 *	Union for unifying all the communication datatypes.  This
	 *	greatly simplifies any interface using the types.
	 *	Medium values are at most 16 bytes long
	 */
	union PvdCommLayerMediumData
	{
		PvdCommLayerSmallData	mSmallData;
		PxU64					mU64;
		PxI64					mI64;
		PxF64					mF64;
		InstanceId				mObjectId;
		Float3					mFloat3;		
		Position				mPosition;
		Velocity				mVelocity;
		Direction				mDirection;
		Float4					mFloat4;
		Plane					mPlane;
		Quat					mQuat;
		FloatBuffer				mFloatBuffer;
		U32Buffer				mU32Buffer;
		U32Array4				mU32Array4;
		FilterData				mFilterData;
		StreamUpdate			mStreamUpdate;
	};

	/**
	 *	Mapping compiler types to the union's datastructures.
	 */
	template<typename TDataType>
	PX_INLINE PvdCommLayerMediumData createCommLayerMediumData( const TDataType& inValue ) { PvdCommLayerMediumData retval; retval.mSmallData = createCommLayerSmallData( inValue ); return retval; }
	template<> PX_INLINE PvdCommLayerMediumData createCommLayerMediumData( const PvdCommLayerSmallData& inValue ) { PvdCommLayerMediumData retval; retval.mSmallData = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerMediumData createCommLayerMediumData( const PxU64& inValue ) { PvdCommLayerMediumData retval; retval.mU64 = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerMediumData createCommLayerMediumData( const PxI64& inValue ) { PvdCommLayerMediumData retval; retval.mI64 = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerMediumData createCommLayerMediumData( const PxF64& inValue ) { PvdCommLayerMediumData retval; retval.mF64 = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerMediumData createCommLayerMediumData( const InstanceId& inValue ) { PvdCommLayerMediumData retval; retval.mObjectId = inValue; return retval; }		
	template<> PX_INLINE PvdCommLayerMediumData createCommLayerMediumData( const Float3& inValue ) { PvdCommLayerMediumData retval; retval.mFloat3 = inValue; return retval; }		
	template<> PX_INLINE PvdCommLayerMediumData createCommLayerMediumData( const PxVec3& inValue ) { PvdCommLayerMediumData retval; retval.mFloat3 = createFloat3( inValue.x, inValue.y, inValue.z ); return retval; }
	template<> PX_INLINE PvdCommLayerMediumData createCommLayerMediumData( const Position& inValue ) { PvdCommLayerMediumData retval; retval.mPosition = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerMediumData createCommLayerMediumData( const Velocity& inValue ) { PvdCommLayerMediumData retval; retval.mVelocity = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerMediumData createCommLayerMediumData( const Direction& inValue ) { PvdCommLayerMediumData retval; retval.mDirection = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerMediumData createCommLayerMediumData( const Float4& inValue ) { PvdCommLayerMediumData retval; retval.mFloat4 = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerMediumData createCommLayerMediumData( const Plane& inValue ) { PvdCommLayerMediumData retval; retval.mPlane = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerMediumData createCommLayerMediumData( const Quat& inValue ) { PvdCommLayerMediumData retval; retval.mQuat = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerMediumData createCommLayerMediumData( const PxQuat& inValue ) { PvdCommLayerMediumData retval; retval.mQuat = createQuat( inValue.x, inValue.y, inValue.z, inValue.w ); return retval; }
	template<> PX_INLINE PvdCommLayerMediumData createCommLayerMediumData( const FloatBuffer& inValue ) { PvdCommLayerMediumData retval; retval.mFloatBuffer = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerMediumData createCommLayerMediumData( const U32Buffer& inValue ) { PvdCommLayerMediumData retval; retval.mU32Buffer = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerMediumData createCommLayerMediumData( const U32Array4& inValue ) { PvdCommLayerMediumData retval; retval.mU32Array4 = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerMediumData createCommLayerMediumData( const FilterData& inValue ) { PvdCommLayerMediumData retval; retval.mFilterData = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerMediumData createCommLayerMediumData( const StreamUpdate& inValue ) { PvdCommLayerMediumData retval; retval.mStreamUpdate = inValue; return retval; }

	template<typename TDataType>
	PX_INLINE TDataType getCommLayerMediumData( const PvdCommLayerMediumData& inData ) { return getCommLayerSmallData<TDataType>( inData.mSmallData ); }
	template<> PX_INLINE PvdCommLayerSmallData getCommLayerMediumData( const PvdCommLayerMediumData& inData ) { return inData.mSmallData;	 }
	template<> PX_INLINE PxU64			getCommLayerMediumData( const PvdCommLayerMediumData& inData ) { return inData.mU64; }
	template<> PX_INLINE PxI64			getCommLayerMediumData( const PvdCommLayerMediumData& inData ) { return inData.mI64; }
	template<> PX_INLINE PxF64			getCommLayerMediumData( const PvdCommLayerMediumData& inData ) { return inData.mF64; }
	template<> PX_INLINE InstanceId		getCommLayerMediumData( const PvdCommLayerMediumData& inData ) { return inData.mObjectId; }
	template<> PX_INLINE Float3			getCommLayerMediumData( const PvdCommLayerMediumData& inData ) { return inData.mFloat3; }
	template<> PX_INLINE PxVec3			getCommLayerMediumData( const PvdCommLayerMediumData& inData ) { return inData.mFloat3; }
	template<> PX_INLINE Position		getCommLayerMediumData( const PvdCommLayerMediumData& inData ) { return inData.mPosition; }
	template<> PX_INLINE Velocity		getCommLayerMediumData( const PvdCommLayerMediumData& inData ) { return inData.mVelocity; }
	template<> PX_INLINE Direction		getCommLayerMediumData( const PvdCommLayerMediumData& inData ) { return inData.mDirection; }
	template<> PX_INLINE Float4			getCommLayerMediumData( const PvdCommLayerMediumData& inData ) { return inData.mFloat4; }
	template<> PX_INLINE Plane			getCommLayerMediumData( const PvdCommLayerMediumData& inData ) { return inData.mPlane; }
	template<> PX_INLINE Quat			getCommLayerMediumData( const PvdCommLayerMediumData& inData ) { return inData.mQuat; }
	template<> PX_INLINE PxQuat			getCommLayerMediumData( const PvdCommLayerMediumData& inData ) { return inData.mQuat; }
	template<> PX_INLINE FloatBuffer	getCommLayerMediumData( const PvdCommLayerMediumData& inData ) { return inData.mFloatBuffer; }
	template<> PX_INLINE U32Buffer		getCommLayerMediumData( const PvdCommLayerMediumData& inData ) { return inData.mU32Buffer; }
	template<> PX_INLINE U32Array4		getCommLayerMediumData( const PvdCommLayerMediumData& inData ) { return inData.mU32Array4; }
	template<> PX_INLINE FilterData		getCommLayerMediumData( const PvdCommLayerMediumData& inData ) { return inData.mFilterData; }
	template<> PX_INLINE StreamUpdate	getCommLayerMediumData( const PvdCommLayerMediumData& inData ) { return inData.mStreamUpdate; }

	/**
	 *	Operate on the values in the union in a typesafe way.  When the union is extended you will
	 *	get compile errors, as opposed to perhaps silent runtime failures.
	 */
	template<typename TReturnType, typename TOperator>
	inline TReturnType commLayerMediumDataOperate( PvdCommLayerDatatype inDatatype, const PvdCommLayerMediumData& inData, TOperator inOperator )
	{
		switch( inDatatype.mDatatype )
		{ 
		case PvdCommLayerDatatype::U64:			return inOperator( inData.mU64 );
		case PvdCommLayerDatatype::I64:			return inOperator( inData.mI64 );
		case PvdCommLayerDatatype::Double:		return inOperator( inData.mF64 );
		case PvdCommLayerDatatype::ObjectId:	return inOperator( inData.mObjectId );
		case PvdCommLayerDatatype::Float3:		return inOperator( inData.mFloat3 );
		case PvdCommLayerDatatype::Position:	return inOperator( inData.mPosition );
		case PvdCommLayerDatatype::Velocity:	return inOperator( inData.mVelocity );
		case PvdCommLayerDatatype::Direction:	return inOperator( inData.mDirection );
		case PvdCommLayerDatatype::Float4:		return inOperator( inData.mFloat4 );
		case PvdCommLayerDatatype::Plane:		return inOperator( inData.mPlane );
		case PvdCommLayerDatatype::Quat:		return inOperator( inData.mQuat );
		case PvdCommLayerDatatype::FloatBuffer: return inOperator( inData.mFloatBuffer );
		case PvdCommLayerDatatype::U32Buffer:	return inOperator( inData.mU32Buffer );
		case PvdCommLayerDatatype::U32Array4:	return inOperator( inData.mU32Array4 );
		case PvdCommLayerDatatype::FilterData:	return inOperator( inData.mFilterData );
		case PvdCommLayerDatatype::Stream:		return inOperator( inData.mStreamUpdate );
		default: return commLayerSmallDataOperate<TReturnType, TOperator>( inDatatype, inData.mSmallData, inOperator );
		}
	}
	
	template<typename TReturnType, typename TOperator>
	PX_INLINE TReturnType commLayerMediumDataOperate( PvdCommLayerDatatype inDatatype, TOperator inOperator )
	{
		return commLayerMediumDataOperate<TReturnType, TOperator>( inDatatype, PvdCommLayerMediumData(), inOperator );
	}

	struct SUPVDCommLayerMediumDataDefaulter
	{
		template<typename TDataType> PX_INLINE PvdCommLayerMediumData operator()( const TDataType& inData ) 
		{ 
			PvdCommLayerMediumData retval; retval.mSmallData = SUPVDCommLayerSmallDataDefaulter().operator()( inData ); return retval; 
		}
		PX_INLINE PvdCommLayerMediumData operator()( const PxU64& ) { PxU64 retval = 0; return createCommLayerMediumData( retval ); }
		PX_INLINE PvdCommLayerMediumData operator()( const PxI64& ) { PxI64 retval = 0; return createCommLayerMediumData( retval ); }
		PX_INLINE PvdCommLayerMediumData operator()( const PxF64& ) { PxF64 retval = 0; return createCommLayerMediumData( retval ); }
		PX_INLINE PvdCommLayerMediumData operator()( const InstanceId& ) { InstanceId retval; retval.mValue = 0; return createCommLayerMediumData( retval ); }
		PX_INLINE PvdCommLayerMediumData operator()( const Float3& ) { Float3 retval; retval.set(); return createCommLayerMediumData( retval ); }
		PX_INLINE PvdCommLayerMediumData operator()( const Position& ) { Position retval; retval.set(); return createCommLayerMediumData( retval ); }
		PX_INLINE PvdCommLayerMediumData operator()( const Velocity& ) { Velocity retval; retval.set(); return createCommLayerMediumData( retval ); }
		PX_INLINE PvdCommLayerMediumData operator()( const Direction& ) { Direction retval; retval.set(); return createCommLayerMediumData( retval ); }
		PX_INLINE PvdCommLayerMediumData operator()( const Float4& ) { Float4 retval; retval.set(); return createCommLayerMediumData( retval ); }
		PX_INLINE PvdCommLayerMediumData operator()( const Plane& ) { Plane retval; retval.set(); return createCommLayerMediumData( retval ); }
		PX_INLINE PvdCommLayerMediumData operator()( const Quat& ) { Quat retval; retval.set(); return createCommLayerMediumData( retval ); }
		PX_INLINE PvdCommLayerMediumData operator()( const FloatBuffer& ) { FloatBuffer retval; retval.mData = NULL; retval.mLength = 0; return createCommLayerMediumData( retval ); }
		PX_INLINE PvdCommLayerMediumData operator()( const U32Buffer& ) { U32Buffer retval; retval.mData = NULL; retval.mLength = 0; return createCommLayerMediumData( retval ); }
		PX_INLINE PvdCommLayerMediumData operator()( const U32Array4& ) { U32Array4 retval; retval.set(); return createCommLayerMediumData( retval ); }
		PX_INLINE PvdCommLayerMediumData operator()( const FilterData& ) { FilterData retval; retval.set(); return createCommLayerMediumData( retval ); }
		PX_INLINE PvdCommLayerMediumData operator()( const StreamUpdate& ) { StreamUpdate retval; retval.mData = NULL; retval.mLength = 0; return createCommLayerMediumData( retval ); }
		PX_INLINE PvdCommLayerMediumData operator()() { PvdCommLayerMediumData retval; memset( &retval, 0, sizeof( retval ) ); return retval; }
	};

	/**
	 *	Defaults means initialized to zero.
	 */
	PX_INLINE PvdCommLayerMediumData getDefaultMediumDataValue( PvdCommLayerDatatype inDatatype )
	{
		return commLayerMediumDataOperate<PvdCommLayerMediumData>( inDatatype, SUPVDCommLayerMediumDataDefaulter() );
	}


	/**
	 *	Implementation of the discriminated union to house the types.
	 */
	class PvdCommLayerMediumValue
	{
		PvdCommLayerDatatype		mDatatype;
		PvdCommLayerMediumData		mValue;

	public:
		PX_INLINE PvdCommLayerMediumValue() : mDatatype(PvdCommLayerDatatype::Unknown) {}

		PX_INLINE PvdCommLayerMediumValue( const PvdCommLayerMediumValue& inOther )
			: mDatatype(inOther.mDatatype)
			, mValue(inOther.mValue){}

		template<typename TDataType>
		PX_INLINE PvdCommLayerMediumValue( const TDataType& inValue )
			: mDatatype( getDatatypeForValue(inValue) )
			, mValue( createCommLayerMediumData(inValue) ){}

		PX_INLINE PvdCommLayerMediumValue( const PvdCommLayerDatatype& inType )
			: mDatatype( inType )
			, mValue( getDefaultMediumDataValue(inType) ){}
		
		PX_INLINE PvdCommLayerMediumValue( const PvdCommLayerDatatype& inType, const PvdCommLayerMediumData& inValue )
			: mDatatype( inType )
			, mValue( inValue ){}

		PX_INLINE PvdCommLayerMediumValue& operator=( const PvdCommLayerMediumValue& inOther )
		{
			mDatatype = inOther.mDatatype;
			mValue = inOther.mValue;
			return *this;
		}
		PX_INLINE PvdCommLayerDatatype		getDatatype() const { return mDatatype; }
		PX_INLINE PvdCommLayerMediumData	getData() const { return mValue; }

		template<typename TDataType> 
		PX_INLINE TDataType getValue() const { PX_ASSERT( mDatatype == getDatatypeForValue(TDataType()) ); return getCommLayerMediumData<TDataType>(mValue); }

		template<typename TReturnValue, typename TOperator>
		PX_INLINE TReturnValue visit(TOperator inOperator) const { return commLayerMediumDataOperate<TReturnValue, TOperator>( mDatatype, mValue, inOperator ); }
	};
}

#endif
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

#ifndef PVD_COMMLAYERVALUE_H
#define PVD_COMMLAYERVALUE_H

#include "PVDCommLayerDatatypes.h"
#include "PxAssert.h"
#include "PVDCommLayerMediumValue.h"

namespace PVD
{
	/**
	 *	Union for unifying all the communication datatypes.  This
	 *	greatly simplifies any interface using the types.
	 *	Large data, anything that is larger than 16 bytes
	 *	goes into this structure.
	 */
	union PvdCommLayerData
	{
		PvdCommLayerMediumData	mMediumData;
		Float6					mFloat6;
		Bounds3					mBounds3;
		Float7					mFloat7;
		Transform				mTransform;
		Float9					mFloat9;
		Mat33					mMat33;
		Float12					mFloat12;
		Frame					mFrame;
		Section					mSection;
	};

	template<typename TDataType>
	PX_INLINE PvdCommLayerData createCommLayerData( const TDataType& inValue ) { PvdCommLayerData retval; retval.mMediumData = createCommLayerMediumData( inValue ); return retval; }
	
	template<> PX_INLINE PvdCommLayerData createCommLayerData( const PvdCommLayerMediumData& inValue ) { PvdCommLayerData retval; retval.mMediumData = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerData createCommLayerData( const Float6& inValue ) { PvdCommLayerData retval; retval.mFloat6 = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerData createCommLayerData( const Bounds3& inValue ) { PvdCommLayerData retval; retval.mBounds3 = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerData createCommLayerData( const PxBounds3& inValue ) { PvdCommLayerData retval; retval.mBounds3 = createBounds3( inValue ); return retval; }
	template<> PX_INLINE PvdCommLayerData createCommLayerData( const Float7& inValue ) { PvdCommLayerData retval; retval.mFloat7 = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerData createCommLayerData( const Transform& inValue ) { PvdCommLayerData retval; retval.mTransform = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerData createCommLayerData( const PxTransform& inValue ) {  PvdCommLayerData retval; retval.mTransform = createTransform( inValue ); return retval; }
	template<> PX_INLINE PvdCommLayerData createCommLayerData( const Float9& inValue ) { PvdCommLayerData retval; retval.mFloat9 = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerData createCommLayerData( const Mat33& inValue ) { PvdCommLayerData retval; retval.mMat33 = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerData createCommLayerData( const PxMat33& inValue ) { PvdCommLayerData retval; retval.mMat33 = reinterpret_cast<const Mat33&>(inValue); return retval; }
	template<> PX_INLINE PvdCommLayerData createCommLayerData( const Float12& inValue ) { PvdCommLayerData retval; retval.mFloat12 = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerData createCommLayerData( const Frame& inValue ) { PvdCommLayerData retval; retval.mFrame = inValue; return retval; }
	template<> PX_INLINE PvdCommLayerData createCommLayerData( const Section& inValue ) { PvdCommLayerData retval; retval.mSection = inValue; return retval; }

	
	template<typename TDataType>
	PX_INLINE TDataType					getCommLayerData( const PvdCommLayerData& inData ) { return getCommLayerMediumData<TDataType>( inData.mMediumData ); }
	template<> PX_INLINE Float6			getCommLayerData( const PvdCommLayerData& inData ) { return inData.mFloat6; }
	template<> PX_INLINE Bounds3		getCommLayerData( const PvdCommLayerData& inData ) { return inData.mBounds3; }
	template<> PX_INLINE PxBounds3		getCommLayerData( const PvdCommLayerData& inData ) { return inData.mBounds3; }
	template<> PX_INLINE Float7			getCommLayerData( const PvdCommLayerData& inData ) { return inData.mFloat7; }
	template<> PX_INLINE Transform		getCommLayerData( const PvdCommLayerData& inData ) { return inData.mTransform; }
	template<> PX_INLINE PxTransform	getCommLayerData( const PvdCommLayerData& inData ) { return inData.mTransform; }
	template<> PX_INLINE Float9			getCommLayerData( const PvdCommLayerData& inData ) { return inData.mFloat9; }
	template<> PX_INLINE Mat33			getCommLayerData( const PvdCommLayerData& inData ) { return inData.mMat33; }
	template<> PX_INLINE PxMat33		getCommLayerData( const PvdCommLayerData& inData ) { return inData.mMat33; }
	template<> PX_INLINE Float12		getCommLayerData( const PvdCommLayerData& inData ) { return inData.mFloat12; }
	template<> PX_INLINE Frame			getCommLayerData( const PvdCommLayerData& inData ) { return inData.mFrame; }
	template<> PX_INLINE Section		getCommLayerData( const PvdCommLayerData& inData ) { return inData.mSection; }

	/**
	 *	Operate on the values in the union in a typesafe way.  When the union is extended you will
	 *	get compile errors, as opposed to perhaps silent runtime failures.
	 */
	template<typename TReturnType, typename TOperator>
	inline TReturnType commLayerDataOperate( PvdCommLayerDatatype inDatatype, const PvdCommLayerData& inData, TOperator inOperator )
	{
		switch( inDatatype.mDatatype )
		{
		case PvdCommLayerDatatype::Float6:		return inOperator( inData.mFloat6 );
		case PvdCommLayerDatatype::Bounds3:		return inOperator( inData.mBounds3 );
		case PvdCommLayerDatatype::Float7:		return inOperator( inData.mFloat7 );
		case PvdCommLayerDatatype::Transform:	return inOperator( inData.mTransform );
		case PvdCommLayerDatatype::Float9:		return inOperator( inData.mFloat9 );
		case PvdCommLayerDatatype::Mat33:		return inOperator( inData.mMat33 );
		case PvdCommLayerDatatype::Float12:		return inOperator( inData.mFloat12 );
		case PvdCommLayerDatatype::Frame:		return inOperator( inData.mFrame );
		case PvdCommLayerDatatype::Section:		return inOperator( inData.mSection );
		default: return commLayerMediumDataOperate<TReturnType,TOperator>( inDatatype, inData.mMediumData, inOperator );
		}
	}
	
	template<typename TReturnType, typename TOperator>
	PX_INLINE TReturnType commLayerDataOperate( PvdCommLayerDatatype inDatatype, TOperator inOperator )
	{
		return commLayerDataOperate<TReturnType, TOperator>( inDatatype, PvdCommLayerData(), inOperator );
	}

	struct SUPVDCommLayerDataDefaulter
	{
		template<typename TDataType>
		PX_INLINE PvdCommLayerData operator()( const TDataType& inValue ) { PvdCommLayerData retval; retval.mMediumData = SUPVDCommLayerMediumDataDefaulter().operator()(inValue); return retval; }
		PX_INLINE PvdCommLayerData operator()( const Float6& ) { Float6 retval; retval.set(); return createCommLayerData( retval ); }
		PX_INLINE PvdCommLayerData operator()( const Bounds3& ) { Bounds3 retval; retval.set(); return createCommLayerData( retval ); }
		PX_INLINE PvdCommLayerData operator()( const Float7& ) { Float7 retval; retval.set(); return createCommLayerData( retval ); }
		PX_INLINE PvdCommLayerData operator()( const Transform& ) { Transform retval; retval.set(); return createCommLayerData( retval ); }
		PX_INLINE PvdCommLayerData operator()( const Float9& ) { Float9 retval; retval.set(); return createCommLayerData( retval ); }
		PX_INLINE PvdCommLayerData operator()( const Mat33& ) { Mat33 retval; retval.set(); return createCommLayerData( retval ); }
		PX_INLINE PvdCommLayerData operator()( const Float12& ) { Float12 retval; retval.set(); return createCommLayerData( retval ); }
		PX_INLINE PvdCommLayerData operator()( const Frame& ) { Frame retval; retval.set(); return createCommLayerData( retval ); }
		PX_INLINE PvdCommLayerData operator()( const Section& ) { Section retval = { 0, 0 }; return createCommLayerData( retval ); }
		PX_INLINE PvdCommLayerData operator()() { PvdCommLayerData retval; memset( &retval, 0, sizeof( retval ) ); return retval; }
	};

	/**
	 *	Defaults means initialized to zero.
	 */
	PX_INLINE PvdCommLayerData getDefaultDataValue( PvdCommLayerDatatype inDatatype )
	{
		return commLayerDataOperate<PvdCommLayerData>( inDatatype, SUPVDCommLayerDataDefaulter() );
	}


	/**
	 *	Implementation of the discriminated union to house the types.
	 */
	class PvdCommLayerValue
	{
		PvdCommLayerDatatype	mDatatype;
		PvdCommLayerData		mValue;

	public:
		PX_INLINE PvdCommLayerValue() : mDatatype(PvdCommLayerDatatype::Unknown) {}

		PX_INLINE PvdCommLayerValue( const PvdCommLayerValue& inOther )
			: mDatatype(inOther.mDatatype)
			, mValue(inOther.mValue){}

		template<typename TDataType>
		PX_INLINE PvdCommLayerValue( const TDataType& inValue )
			: mDatatype( getDatatypeForValue(inValue) )
			, mValue( createCommLayerData(inValue) ){}

		PX_INLINE PvdCommLayerValue( const PvdCommLayerDatatype& inType )
			: mDatatype( inType )
			, mValue( getDefaultDataValue(inType) ){}
		
		template<typename TDataType>
		PX_INLINE PvdCommLayerValue( const PvdCommLayerDatatype& inType, const TDataType& inValue )
			: mDatatype( inType )
			, mValue( createCommLayerData(inValue) ){}
		
		PX_INLINE PvdCommLayerValue( const PvdCommLayerDatatype& inType, const PvdCommLayerData& inValue )
				: mDatatype( inType )
				, mValue( inValue ){}
		

		PX_INLINE PvdCommLayerValue& operator=( const PvdCommLayerValue& inOther )
		{
			mDatatype = inOther.mDatatype;
			mValue = inOther.mValue;
			return *this;
		}
		PX_INLINE PvdCommLayerDatatype	getDatatype() const { return mDatatype; }
		PX_INLINE PvdCommLayerData		getData() const { return mValue; }

		template<typename TDataType>
		PX_INLINE TDataType	getValue() const { PX_ASSERT( mDatatype == getDatatypeForValue( TDataType() ) ); return getCommLayerData<TDataType>( mValue ); }

		template<typename TReturnValue, typename TOperator>
		PX_INLINE TReturnValue visit(TOperator inOperator) const { return commLayerDataOperate<TReturnValue, TOperator>( mDatatype, mValue, inOperator ); }
	};
}

#endif
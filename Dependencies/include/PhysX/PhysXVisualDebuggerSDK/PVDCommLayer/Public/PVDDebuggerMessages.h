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

#ifndef PVD_PVDDEBUGGERMESSAGE_H
#define PVD_PVDDEBUGGERMESSAGE_H

#include "PxSimpleTypes.h"

namespace PVD
{
	struct PvdDebugMessageType
	{
		enum 
		{
			Unknown = 0,
			Pause, //Stop playback of the scene
			Record, //Continue playback of the scene.
			Monitor, //Switch mode to monity
			Disconnect, //Go away.
			SetPropertyValue,
			EventBatch, //Meta event grouping sub events.
			Last,
		};
		PxU8 mType;
		PvdDebugMessageType( PxU8 inType=Unknown) : mType(inType){}
		bool operator==( const PvdDebugMessageType& inOther ) const { return mType == inOther.mType; }
		bool operator!=( const PvdDebugMessageType& inOther ) const { return mType != inOther.mType; }
	};

	struct DebugMessagePause
	{
		PxU8 unused;
		template<typename TStreamType>
		inline void streamify( TStreamType& ) {}
		template<typename TComparator>
		inline bool compare(const DebugMessagePause& /*inOther*/, TComparator /*inComparator*/) const { return true; }
	};
	
	struct DebugMessageRecord
	{
		PxU8 unused;
		template<typename TStreamType>
		inline void streamify( TStreamType& ) {}
		template<typename TComparator>
		inline bool compare(const DebugMessageRecord& /*inOther*/, TComparator /*inComparator*/) const { return true; }
	};

	struct DebugMessageMonitor
	{
		PxU8 unused;
		template<typename TStreamType>
		inline void streamify( TStreamType& ) {}
		template<typename TComparator>
		inline bool compare(const DebugMessageMonitor& /*inOther*/, TComparator /*inComparator*/) const { return true; }
	};

	inline DebugMessageRecord createDebugMessageRecord() { return DebugMessageRecord(); }
	
	struct DebugMessageDisconnect
	{
		PxU8 unused;
		template<typename TStreamType>
		inline void streamify( TStreamType& ) {}
		template<typename TComparator>
		inline bool compare(const DebugMessageDisconnect& /*inOther*/, TComparator /*inComparator*/) const { return true; }
	};
	
	inline DebugMessageDisconnect createDebugMessageDisconnect() { return DebugMessageDisconnect(); }

	struct DebugMessagesetPropertyValue
	{
		PxU64					mInstanceId;
		PxU32					mProperty;
		PxU8					mDatatype; //PvdCommLayerDatatype
		PvdCommLayerData		mValue;

		template<typename TStreamType>
		inline void streamify( TStreamType& inStream ) 
		{
			inStream.streamify( mInstanceId );
			inStream.streamify( mProperty );
			inStream.streamify( mDatatype );
			inStream.streamify( mDatatype, mValue );
		}
		template<typename TComparator>
		inline bool compare(const DebugMessagesetPropertyValue& inOther, TComparator inComparator) const
		{ 
			return inComparator( mInstanceId, inOther.mInstanceId) &&
					inComparator( mProperty, inOther.mProperty) &&
					inComparator( PvdCommLayerValue( mDatatype, mValue ), PvdCommLayerValue( inOther.mDatatype, inOther.mValue ) );
		}
	};

	inline DebugMessagesetPropertyValue createDebugMessagesetPropertyValue( PxU64 inInstance, PxU32 inProperty, const PvdCommLayerValue& inValue )
	{
		DebugMessagesetPropertyValue retval = { inInstance, inProperty, inValue.getDatatype().mDatatype, inValue.getData() };
		return retval;
	}
	
	inline DebugMessagesetPropertyValue createDebugMessagesetPropertyValue() { return createDebugMessagesetPropertyValue( 0, 0, PvdCommLayerValue() ); }

	template<typename TDataType>
	PX_INLINE PvdDebugMessageType getPVDDebugMessageType() { PX_ASSERT( false ); return PvdDebugMessageType::Unknown; }
	
	template<> PX_INLINE PvdDebugMessageType getPVDDebugMessageType<DebugMessagePause>() { return PvdDebugMessageType::Pause; }
	template<> PX_INLINE PvdDebugMessageType getPVDDebugMessageType<DebugMessageRecord>() { return PvdDebugMessageType::Record; }
	template<> PX_INLINE PvdDebugMessageType getPVDDebugMessageType<DebugMessageMonitor>() { return PvdDebugMessageType::Monitor; }
	template<> PX_INLINE PvdDebugMessageType getPVDDebugMessageType<DebugMessageDisconnect>() { return PvdDebugMessageType::Disconnect; }
	template<> PX_INLINE PvdDebugMessageType getPVDDebugMessageType<DebugMessagesetPropertyValue>() { return PvdDebugMessageType::SetPropertyValue; }

	union PvdDebugMessageData
	{
		DebugMessagePause				mPause;
		DebugMessageRecord				mRecord;
		DebugMessageMonitor				mMonitor;
		DebugMessageDisconnect			mDisconnect;
		DebugMessagesetPropertyValue	msetPropertyValue;
	};

	template< typename TDataType>
	PX_INLINE PvdDebugMessageData toPVDDebugMessageData( const TDataType& ) { PX_ASSERT( false ); return PvdDebugMessageData(); }
	template<> PX_INLINE PvdDebugMessageData toPVDDebugMessageData( const DebugMessagePause& inData )			{ PvdDebugMessageData retval; retval.mPause = inData; return retval; }
	template<> PX_INLINE PvdDebugMessageData toPVDDebugMessageData( const DebugMessageRecord& inData )			{ PvdDebugMessageData retval; retval.mRecord = inData; return retval; }
	template<> PX_INLINE PvdDebugMessageData toPVDDebugMessageData( const DebugMessageMonitor& inData )			{ PvdDebugMessageData retval; retval.mMonitor = inData; return retval; }	
	template<> PX_INLINE PvdDebugMessageData toPVDDebugMessageData( const DebugMessageDisconnect& inData )		{ PvdDebugMessageData retval; retval.mDisconnect = inData; return retval; }
	template<> PX_INLINE PvdDebugMessageData toPVDDebugMessageData( const DebugMessagesetPropertyValue& inData ) { PvdDebugMessageData retval; retval.msetPropertyValue = inData; return retval; }

	
	template< typename TDataType>
	inline TDataType fromPVDDebugMessageData( const PvdDebugMessageData& /*inData*/ ) { PX_ASSERT( false ); return TDataType(); }
	template<> PX_INLINE DebugMessagePause				fromPVDDebugMessageData<DebugMessagePause>( const PvdDebugMessageData& inData )	{ return inData.mPause; }
	template<> PX_INLINE DebugMessageRecord				fromPVDDebugMessageData<DebugMessageRecord>( const PvdDebugMessageData& inData )	{ return inData.mRecord; }
	template<> PX_INLINE DebugMessageMonitor			fromPVDDebugMessageData<DebugMessageMonitor>( const PvdDebugMessageData& inData )	{ return inData.mMonitor; }
	template<> PX_INLINE DebugMessageDisconnect			fromPVDDebugMessageData<DebugMessageDisconnect>( const PvdDebugMessageData& inData )	{ return inData.mDisconnect; }
	template<> PX_INLINE DebugMessagesetPropertyValue	fromPVDDebugMessageData<DebugMessagesetPropertyValue>( const PvdDebugMessageData& inData ) { return inData.msetPropertyValue; }

	template<typename TReturnType, typename TOperator>
	inline TReturnType visitDebugMessage( PvdDebugMessageType inType, const PvdDebugMessageData& inData, TOperator inOperator )
	{
		switch( inType.mType )
		{
		case PvdDebugMessageType::Pause: return inOperator( fromPVDDebugMessageData<DebugMessagePause>( inData ) );
		case PvdDebugMessageType::Record: return inOperator( fromPVDDebugMessageData<DebugMessageRecord>( inData ) );
		case PvdDebugMessageType::Monitor: return inOperator( fromPVDDebugMessageData<DebugMessageMonitor>( inData ) );
		case PvdDebugMessageType::Disconnect: return inOperator( fromPVDDebugMessageData<DebugMessageDisconnect>( inData ) );
		case PvdDebugMessageType::SetPropertyValue: return inOperator( fromPVDDebugMessageData<DebugMessagesetPropertyValue>( inData ) );
		}
		return inOperator();
	}
	
	template<typename TReturnType, typename TOperator>
	inline TReturnType visitDebugMessage( PvdDebugMessageType inType, TOperator inOperator )
	{
		return visitDebugMessage<TReturnType,TOperator>( inType, PvdDebugMessageData(), inOperator );
	}

	class PvdDebugMessage
	{
		PvdDebugMessageType mType;
		PvdDebugMessageData mData;
	public:
		PX_INLINE PvdDebugMessage(){}
		PX_INLINE PvdDebugMessage( PvdDebugMessageType inType ) : mType(inType){}

		PX_INLINE PvdDebugMessage( PvdDebugMessageType inType, const PvdDebugMessageData& inData ) 
				: mType(inType)
				, mData(inData) {}
		
		template<typename TDataType>
		PX_INLINE PvdDebugMessage( const TDataType& inData ) 
				: mType(getPVDDebugMessageType<TDataType>())
				, mData(toPVDDebugMessageData(inData)) {}

		PX_INLINE PvdDebugMessage( const PvdDebugMessage& inOther )
			: mType( inOther.mType ) 
			, mData( inOther.mData ) {}

		PX_INLINE PvdDebugMessage& operator=( const PvdDebugMessage& inOther )
		{
			mType = inOther.mType;
			mData = inOther.mData;
			return *this;
		}

		PX_INLINE PvdDebugMessageType	getType() const { return mType; }
		PX_INLINE PvdDebugMessageData	getData() const { return mData; }

		template<typename TReturnType, typename TOperator>
		PX_INLINE TReturnType visit( TOperator inOperator ) { return visitDebugMessage<TReturnType, TOperator>( mType, mData, inOperator ); }
		template<typename TDataType>
		PX_INLINE TDataType getValue() const { PX_ASSERT( getPVDDebugMessageType<TDataType>() == mType ); return fromPVDDebugMessageData<TDataType>( mData ); }
	};
}

#endif
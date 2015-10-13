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

#ifndef PVD_PVDRENDERCOMMANDS_H
#define PVD_PVDRENDERCOMMANDS_H

#include "PvdRenderTypes.h"

namespace PVD { 

	/**
	 *	Then render commands PVD supports
	 */
	struct RenderCommandType
	{
		enum Enum
		{
			Unknown = 0,
			PushRenderState,
			PopRenderState,
			SetCurrentInstance,
			SetCurrentColor,
			SetCurrentTextScale,
			SetCurrentRenderFlags,
			AddCurrentRenderFlags,
			RemoveCurrentRenderFlags,
			PushTransform,
			SetTransform,
			MultiplyTransform,
			PopTransform,
			DrawPrimitive,
			DrawRenderStatePrimitive,
			Last,
		};
		PxU8	mType;
		RenderCommandType( PxU8 inType=Unknown ) : mType( inType ) {}
		PX_INLINE bool operator==( const RenderCommandType& inOther ) const { return mType == inOther.mType; }
		PX_INLINE bool operator!=( const RenderCommandType& inOther ) const { return !(*this == inOther ); }
		PX_INLINE const char* toString() const
		{
			switch( mType )
			{
			case Unknown: return "Unknown";
			case PushRenderState: return "PushRenderState";
			case PopRenderState: return "PopRenderState";
			case SetCurrentInstance: return "SetCurrentInstance";
			case SetCurrentColor: return "SetCurrentColor";
			case SetCurrentRenderFlags: return "SetCurrentRenderFlags";
			case AddCurrentRenderFlags: return "AddCurrentRenderFlags";
			case RemoveCurrentRenderFlags: return "RemoveCurrentRenderFlags";
			case PushTransform: return "PushTransform";
			case SetTransform: return "SetTransform";
			case MultiplyTransform: return "MultiplyTransform";
			case PopTransform: return "PopTransform";
			case DrawPrimitive: return "DrawPrimitive";
			case DrawRenderStatePrimitive: return "DrawRenderStatePrimitive";
			}
			return "";
		}
	};

	struct PushRenderState
	{
		PxU32 unused;
		template<typename TStreamType>
		inline void streamify( TStreamType& ) {}
		template<typename TOperator>
		inline bool compare( const PushRenderState&, TOperator ) const { return true; }
	};
	PX_INLINE PushRenderState createPushRenderState() { PushRenderState retval = { 0 }; return retval; }
	
	struct PopRenderState
	{
		PxU32 unused;
		template<typename TStreamType>
		inline void streamify( TStreamType& ) {}
		template<typename TOperator>
		inline bool compare( const PopRenderState&, TOperator ) const { return true; }
	};
	PX_INLINE PopRenderState createPopRenderState() { PopRenderState retval = { 0 }; return retval; }

	struct SetCurrentInstance
	{
		PxU64 mInstance;
		template<typename TStreamType>
		inline void streamify( TStreamType& inStream ) { inStream.streamify( mInstance ); }
		template<typename TOperator>
		inline bool compare( const SetCurrentInstance& inOther, TOperator inOperator ) const { return inOperator( mInstance, inOther.mInstance ); }
	};
	PX_INLINE SetCurrentInstance createSetCurrentInstance( PxU64 inInstance = 0 )  { SetCurrentInstance retval = { inInstance }; return retval; }
	
	struct SetCurrentColor
	{
		Color mColor;
		template<typename TStreamType>
		inline void streamify( TStreamType& inStream ) { inStream.streamify( mColor.mValue ); }
		template<typename TOperator>
		inline bool compare( const SetCurrentColor& inOther, TOperator inOperator ) const { return inOperator( mColor.mValue, inOther.mColor.mValue ); }
	};
	PX_INLINE SetCurrentColor createSetCurrentColor( Color inColor )  { SetCurrentColor retval = { inColor }; return retval; }

	
	struct SetCurrentTextScale
	{
		PxF32 mScale;
		template<typename TStreamType> inline void streamify( TStreamType& inStream ) { inStream.streamify( mScale ); }
		template<typename TOperator> inline bool compare( const SetCurrentTextScale& inOther, TOperator inOperator ) const { return inOperator( mScale, inOther.mScale ); }
	};
	PX_INLINE SetCurrentTextScale createSetCurrentTextScale( PxF32 inScale )  { SetCurrentTextScale retval = { inScale }; return retval; }

	struct RenderFlagsCommandBase
	{
		PxU32 mFlags;
		template<typename TStreamType> inline void streamify( TStreamType& inStream ) { inStream.streamify( mFlags ); }
		template<typename TOperator> inline bool compare( const RenderFlagsCommandBase& inOther, TOperator inOperator ) const { return inOperator( mFlags, inOther.mFlags ); }
	};

	struct SetCurrentRenderFlags : RenderFlagsCommandBase {};
	PX_INLINE SetCurrentRenderFlags createSetCurrentRenderFlags( PxU32 inFlags ) { SetCurrentRenderFlags retval; retval.mFlags = inFlags; return retval; }

	struct AddCurrentRenderFlags : RenderFlagsCommandBase {};
	PX_INLINE AddCurrentRenderFlags createAddCurrentRenderFlags( PxU32 inFlags ) { AddCurrentRenderFlags retval; retval.mFlags = inFlags; return retval; }

	struct RemoveCurrentRenderFlags : RenderFlagsCommandBase {};
	PX_INLINE RemoveCurrentRenderFlags createRemoveCurrentRenderFlags( PxU32 inFlags ) { RemoveCurrentRenderFlags retval; retval.mFlags = inFlags; return retval; }

	struct PushTransform
	{
		PxU32 mUnused;
		template<typename TStreamType>
		inline void streamify( TStreamType&) {}
		template<typename TOperator>
		inline bool compare( const PushTransform&, TOperator) const { return true; }
	};

	PX_INLINE PushTransform createPushTransform()  { PushTransform retval = { 0 }; return retval; }

	struct SetTransform
	{
		RenderTransformData mData;
		PxU8 mType;
		template<typename TStreamType>
		inline void streamify( TStreamType& ioStream) { ioStream.streamify( mType, mData ); }
		template<typename TOperator>
		inline bool compare( const SetTransform& inOther, TOperator inOperator ) const { return inOperator( RenderTransform( mType, mData ), RenderTransform( inOther.mType, inOther.mData ) ); }
	};
	PX_INLINE SetTransform createSetTransform( RenderTransformType inType, RenderTransformData inData ) { SetTransform retval = { inData, inType.mType }; return retval; }

	
	struct MultiplyTransform
	{
		RenderTransformData mData;
		PxU8 mType;
		template<typename TStreamType>
		inline void streamify( TStreamType& ioStream) { ioStream.streamify( mType, mData ); }
		template<typename TOperator>
		inline bool compare( const MultiplyTransform& inOther, TOperator inOperator ) const { return inOperator( RenderTransform( mType, mData ), RenderTransform( inOther.mType, inOther.mData ) ); }
	};
	PX_INLINE MultiplyTransform createMultiplyTransform( RenderTransformType inType, RenderTransformData inData ) { MultiplyTransform retval = { inData, inType.mType }; return retval; }
	
	
	struct PopTransform
	{
		PxU32 mUnused;
		template<typename TStreamType>
		inline void streamify( TStreamType&) {}
		template<typename TOperator>
		inline bool compare( const PopTransform&, TOperator) const { return true; }
	};
	PX_INLINE PopTransform createPopTransform()  { PopTransform retval = { 0 }; return retval; }

	struct DrawPrimitive
	{
		RenderPrimitiveData		mPrimitiveData;
		RenderTransformData			mTransformData;
		PxU8					mPrimitiveType;
		PxU8					mTransformType;

		template<typename TStreamType>
		inline void streamify( TStreamType& ioStream) { ioStream.streamify( mTransformType, mTransformData, mPrimitiveType, mPrimitiveData ); }
		template<typename TOperator>
		inline bool compare( const DrawPrimitive& inOther, TOperator inOperator ) const
		{ 
			return inOperator( RenderTransform( mTransformType, mTransformData), RenderTransform( inOther.mTransformType, inOther.mTransformData ) )
				&& inOperator( RenderPrimitive( mPrimitiveType, mPrimitiveData), RenderPrimitive( inOther.mPrimitiveType, inOther.mPrimitiveData ) ); 
		}
	};

	PX_INLINE DrawPrimitive createDrawPrimitive( RenderTransformType inTransformType
													, RenderTransformData inTransformData
													, RenderPrimitiveType inPrimitiveType
													, RenderPrimitiveData inPrimitiveData )
	{
		DrawPrimitive retval = { inPrimitiveData , inTransformData, inPrimitiveType.mType, inTransformType.mType };
		return retval;
	}

	struct DrawRenderStatePrimitive
	{
		DrawPrimitive	mDrawPrimitive;
		RenderState		mRenderState;
		
		template<typename TStreamType>
		inline void streamify( TStreamType& ioStream) 
		{
			mDrawPrimitive.streamify( ioStream );
			mRenderState.streamify( ioStream );
		}
		template<typename TOperator>
		inline bool compare( const DrawRenderStatePrimitive& inOther, TOperator inOperator ) const
		{ 
			return mDrawPrimitive.compare( inOther.mDrawPrimitive, inOperator )
				&& mRenderState.compare( inOther.mRenderState, inOperator );
		}
	};

	PX_INLINE DrawRenderStatePrimitive createDrawRenderStatePrimitive( const DrawPrimitive& inPrimitive, const RenderState& inState )
	{
		DrawRenderStatePrimitive retval = { inPrimitive, inState };
		return retval;
	}

	template<typename TDataType> PX_INLINE RenderCommandType getRenderCommandType() { PX_ASSERT(false); return RenderCommandType(); }
	template<> PX_INLINE RenderCommandType getRenderCommandType<PushRenderState>() { return RenderCommandType::PushRenderState; }
	template<> PX_INLINE RenderCommandType getRenderCommandType<PopRenderState>() { return RenderCommandType::PopRenderState; }
	template<> PX_INLINE RenderCommandType getRenderCommandType<SetCurrentInstance>() { return RenderCommandType::SetCurrentInstance; }
	template<> PX_INLINE RenderCommandType getRenderCommandType<SetCurrentColor>() { return RenderCommandType::SetCurrentColor; }
	template<> PX_INLINE RenderCommandType getRenderCommandType<SetCurrentRenderFlags>() { return RenderCommandType::SetCurrentRenderFlags; }
	template<> PX_INLINE RenderCommandType getRenderCommandType<AddCurrentRenderFlags>() { return RenderCommandType::AddCurrentRenderFlags; }
	template<> PX_INLINE RenderCommandType getRenderCommandType<RemoveCurrentRenderFlags>() { return RenderCommandType::RemoveCurrentRenderFlags; }
	template<> PX_INLINE RenderCommandType getRenderCommandType<SetCurrentTextScale>() { return RenderCommandType::SetCurrentTextScale; }
	template<> PX_INLINE RenderCommandType getRenderCommandType<PushTransform>() { return RenderCommandType::PushTransform; }
	template<> PX_INLINE RenderCommandType getRenderCommandType<SetTransform>() { return RenderCommandType::SetTransform; }
	template<> PX_INLINE RenderCommandType getRenderCommandType<MultiplyTransform>() { return RenderCommandType::MultiplyTransform; }
	template<> PX_INLINE RenderCommandType getRenderCommandType<PopTransform>() { return RenderCommandType::PopTransform; }
	template<> PX_INLINE RenderCommandType getRenderCommandType<DrawPrimitive>() { return RenderCommandType::DrawPrimitive; }
	template<> PX_INLINE RenderCommandType getRenderCommandType<DrawRenderStatePrimitive>() { return RenderCommandType::DrawRenderStatePrimitive; }

	template<typename TDataType> PX_INLINE bool isRenderCommand() { return false; }
	template<> PX_INLINE bool isRenderCommand<PushRenderState>() { return true; }
	template<> PX_INLINE bool isRenderCommand<PopRenderState>() { return true; }
	template<> PX_INLINE bool isRenderCommand<SetCurrentInstance>() { return true; }
	template<> PX_INLINE bool isRenderCommand<SetCurrentColor>() { return true; }
	template<> PX_INLINE bool isRenderCommand<SetCurrentTextScale>() { return true; }
	template<> PX_INLINE bool isRenderCommand<SetCurrentRenderFlags>() { return true; }
	template<> PX_INLINE bool isRenderCommand<AddCurrentRenderFlags>() { return true; }
	template<> PX_INLINE bool isRenderCommand<RemoveCurrentRenderFlags>() { return true; }
	template<> PX_INLINE bool isRenderCommand<PushTransform>() { return true; }
	template<> PX_INLINE bool isRenderCommand<SetTransform>() { return true; }
	template<> PX_INLINE bool isRenderCommand<MultiplyTransform>() { return true; }
	template<> PX_INLINE bool isRenderCommand<PopTransform>() { return true; }
	template<> PX_INLINE bool isRenderCommand<DrawPrimitive>() { return true; }
	template<> PX_INLINE bool isRenderCommand<DrawRenderStatePrimitive>() { return true; }


	union RenderCommandData
	{	
		PushRenderState mPushRenderState;
		PopRenderState mPopRenderState;
		SetCurrentInstance mSetCurrentInstance;
		SetCurrentColor mSetCurrentColor;
		SetCurrentTextScale mSetCurrentTextScale;
		SetCurrentRenderFlags mSetCurrentRenderFlags;
		AddCurrentRenderFlags mAddCurrentRenderFlags;
		RemoveCurrentRenderFlags mRemoveCurrentRenderFlags;
		PushTransform mPushTransform;
		SetTransform mSetIdentityTransform;
		MultiplyTransform mMultiplyTransform;
		PopTransform mPopTransform;
		DrawPrimitive mDebugPrimitive;
		DrawRenderStatePrimitive mDrawRenderStatePrimitive;
	};
	
	template<typename TDataType> PX_INLINE RenderCommandData toRenderCommandData( const TDataType&) { PX_ASSERT(false); return RenderCommandData(); }
	PX_INLINE RenderCommandData toRenderCommandData( const PushRenderState& inItem ) { RenderCommandData retval; retval.mPushRenderState = inItem; return retval; }
	PX_INLINE RenderCommandData toRenderCommandData( const PopRenderState& inItem ) { RenderCommandData retval; retval.mPopRenderState = inItem; return retval; }
	PX_INLINE RenderCommandData toRenderCommandData( const SetCurrentInstance& inItem ) { RenderCommandData retval; retval.mSetCurrentInstance = inItem; return retval; }
	PX_INLINE RenderCommandData toRenderCommandData( const SetCurrentColor& inItem ) { RenderCommandData retval; retval.mSetCurrentColor = inItem; return retval; }
	PX_INLINE RenderCommandData toRenderCommandData( const SetCurrentTextScale& inItem ) { RenderCommandData retval; retval.mSetCurrentTextScale = inItem; return retval; }
	PX_INLINE RenderCommandData toRenderCommandData( const SetCurrentRenderFlags& inItem ) { RenderCommandData retval; retval.mSetCurrentRenderFlags = inItem; return retval; }
	PX_INLINE RenderCommandData toRenderCommandData( const AddCurrentRenderFlags& inItem ) { RenderCommandData retval; retval.mAddCurrentRenderFlags = inItem; return retval; }
	PX_INLINE RenderCommandData toRenderCommandData( const RemoveCurrentRenderFlags& inItem ) { RenderCommandData retval; retval.mRemoveCurrentRenderFlags = inItem; return retval; }
	PX_INLINE RenderCommandData toRenderCommandData( const PushTransform& inItem ) { RenderCommandData retval; retval.mPushTransform = inItem; return retval; }
	PX_INLINE RenderCommandData toRenderCommandData( const SetTransform& inItem ) { RenderCommandData retval; retval.mSetIdentityTransform = inItem; return retval; }
	PX_INLINE RenderCommandData toRenderCommandData( const MultiplyTransform& inItem ) { RenderCommandData retval; retval.mMultiplyTransform = inItem; return retval; }
	PX_INLINE RenderCommandData toRenderCommandData( const PopTransform& inItem ) { RenderCommandData retval; retval.mPopTransform = inItem; return retval; }
	PX_INLINE RenderCommandData toRenderCommandData( const DrawPrimitive& inItem ) { RenderCommandData retval; retval.mDebugPrimitive = inItem; return retval; }
	PX_INLINE RenderCommandData toRenderCommandData( const DrawRenderStatePrimitive& inItem ) { RenderCommandData retval; retval.mDrawRenderStatePrimitive = inItem; return retval; }

	template<typename TDataType> PX_INLINE TDataType fromRenderCommandData( const RenderCommandData&) { PX_ASSERT(false); return TDataType(); }
	template<> PX_INLINE PushRenderState fromRenderCommandData( const RenderCommandData& inData ) { return inData.mPushRenderState; }
	template<> PX_INLINE PopRenderState fromRenderCommandData( const RenderCommandData& inData ) { return inData.mPopRenderState; }
	template<> PX_INLINE SetCurrentInstance fromRenderCommandData( const RenderCommandData& inData ) { return inData.mSetCurrentInstance; }
	template<> PX_INLINE SetCurrentColor fromRenderCommandData( const RenderCommandData& inData ) { return inData.mSetCurrentColor; }
	template<> PX_INLINE SetCurrentTextScale fromRenderCommandData( const RenderCommandData& inData ) { return inData.mSetCurrentTextScale; }
	template<> PX_INLINE SetCurrentRenderFlags fromRenderCommandData( const RenderCommandData& inData ) { return inData.mSetCurrentRenderFlags; }
	template<> PX_INLINE AddCurrentRenderFlags fromRenderCommandData( const RenderCommandData& inData ) { return inData.mAddCurrentRenderFlags; }
	template<> PX_INLINE RemoveCurrentRenderFlags fromRenderCommandData( const RenderCommandData& inData ) { return inData.mRemoveCurrentRenderFlags; }
	template<> PX_INLINE PushTransform fromRenderCommandData( const RenderCommandData& inData ) { return inData.mPushTransform; }
	template<> PX_INLINE SetTransform fromRenderCommandData( const RenderCommandData& inData ) { return inData.mSetIdentityTransform; }
	template<> PX_INLINE MultiplyTransform fromRenderCommandData( const RenderCommandData& inData ) { return inData.mMultiplyTransform; }
	template<> PX_INLINE PopTransform fromRenderCommandData( const RenderCommandData& inData ) { return inData.mPopTransform; }
	template<> PX_INLINE DrawPrimitive fromRenderCommandData( const RenderCommandData& inData ) { return inData.mDebugPrimitive; }
	template<> PX_INLINE DrawRenderStatePrimitive fromRenderCommandData( const RenderCommandData& inData ) { return inData.mDrawRenderStatePrimitive; }

	template<typename TReturnType, typename TOperator>
	PX_INLINE TReturnType visitRenderCommand( RenderCommandType inType, const RenderCommandData& inData, TOperator inOperator )
	{
		switch( inType.mType )
		{
		case RenderCommandType::PushRenderState: return inOperator( fromRenderCommandData<PushRenderState>( inData ) );
		case RenderCommandType::PopRenderState: return inOperator( fromRenderCommandData<PopRenderState>( inData ) );
		case RenderCommandType::SetCurrentInstance: return inOperator( fromRenderCommandData<SetCurrentInstance>( inData ) );
		case RenderCommandType::SetCurrentColor: return inOperator( fromRenderCommandData<SetCurrentColor>( inData ) );
		case RenderCommandType::SetCurrentTextScale: return inOperator( fromRenderCommandData<SetCurrentTextScale>( inData ) );
		case RenderCommandType::SetCurrentRenderFlags: return inOperator( fromRenderCommandData<SetCurrentRenderFlags>( inData ) );
		case RenderCommandType::AddCurrentRenderFlags: return inOperator( fromRenderCommandData<AddCurrentRenderFlags>( inData ) );
		case RenderCommandType::RemoveCurrentRenderFlags: return inOperator( fromRenderCommandData<RemoveCurrentRenderFlags>( inData ) );
		case RenderCommandType::PushTransform: return inOperator( fromRenderCommandData<PushTransform>( inData ) );
		case RenderCommandType::SetTransform: return inOperator( fromRenderCommandData<SetTransform>( inData ) );
		case RenderCommandType::MultiplyTransform: return inOperator( fromRenderCommandData<MultiplyTransform>( inData ) );
		case RenderCommandType::PopTransform: return inOperator( fromRenderCommandData<PopTransform>( inData ) );
		case RenderCommandType::DrawPrimitive: return inOperator( fromRenderCommandData<DrawPrimitive>( inData ) );
		case RenderCommandType::DrawRenderStatePrimitive: return inOperator( fromRenderCommandData<DrawRenderStatePrimitive>( inData ) );
		default:
			PX_ASSERT(false);
			return inOperator();
		}
	}
	
	template<typename TReturnType, typename TOperator>
	PX_INLINE TReturnType visitRenderCommand( RenderCommandType inType, TOperator inOperator ) { return visitRenderCommand<TReturnType, TOperator>( inType, RenderCommandData(), inOperator ); }


	class RenderCommand
	{
		RenderCommandData mData;
		RenderCommandType mType;
	public:
		RenderCommand( RenderCommandType inType = RenderCommandType() ) : mType( inType ) {}
		RenderCommand( RenderCommandType inType, RenderCommandData inData ) : mData( inData ), mType( inType ) {}
		RenderCommand( const RenderCommand& inOther ) : mData( inOther.mData ), mType( inOther.mType ) {}
		RenderCommand& operator=( const RenderCommand& inOther ) 
		{
			mType = inOther.mType;
			mData = inOther.mData;
			return *this;
		}
		template<typename TDataType>
		RenderCommand( const TDataType& inData ) : mType( getRenderCommandType<TDataType>() ), mData( toRenderCommandData( inData ) ) {}
		RenderCommandData getData() const { return mData; }
		RenderCommandType getType() const { return mType; }

		template<typename TDataType>
		TDataType getValue() const { PX_ASSERT( mType == getRenderCommandType<TDataType>() ); return fromRenderCommandData<TDataType>( mData ); }

		template<typename TReturnType, typename TOperator>
		TReturnType visit( TOperator inOperator ) const { return visitRenderCommand<TReturnType, TOperator>( mType, mData, inOperator ); }
	};

} 

#endif
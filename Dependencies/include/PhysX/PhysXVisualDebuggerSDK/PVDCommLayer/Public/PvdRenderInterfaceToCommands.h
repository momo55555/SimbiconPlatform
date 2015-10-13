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

#ifndef PVD_PVDRENDERINTERFACETOCOMMANDS_H
#define PVD_PVDRENDERINTERFACETOCOMMANDS_H
#include "PvdRenderCommands.h"

namespace PVD
{
	//Pack up the interface functions to commands
	template<typename TReturnType,typename TCommandHandler>
	struct PvdRenderInterfaceToCommands
	{
		TCommandHandler mHandler;

		PvdRenderInterfaceToCommands( TCommandHandler inHandler ) : mHandler( inHandler ) {}

		PX_INLINE TReturnType pushRenderState() { return mHandler( createPushRenderState() ); }
		PX_INLINE TReturnType setCurrentInstance( PxU64 inInstanceId ) { return mHandler( createSetCurrentInstance( inInstanceId ) ); }
		PX_INLINE TReturnType setCurrentColor( Color inColor ) { return mHandler( createSetCurrentColor( inColor ) ); }
		PX_INLINE TReturnType setCurrentTextScale( PxF32 inTextScale ) { return mHandler( createSetCurrentTextScale( inTextScale ) ); }
		PX_INLINE TReturnType setCurrentRenderFlags( const RenderFlags& inFlags ) { return mHandler( createSetCurrentRenderFlags( inFlags.mFlags ) ); }
		PX_INLINE TReturnType addCurrentRenderFlags( const RenderFlags& inFlags ) { return mHandler( createAddCurrentRenderFlags( inFlags.mFlags ) ); }
		PX_INLINE TReturnType removeCurrentRenderFlags( const RenderFlags& inFlags ) { return mHandler( createRemoveCurrentRenderFlags( inFlags.mFlags ) ); }
		PX_INLINE TReturnType popRenderState() { return mHandler( createPopRenderState() ); }

		PX_INLINE TReturnType pushTransform() { return mHandler( createPushTransform() ); }
		PX_INLINE TReturnType setTransform(const RenderTransform& inTransform) { return mHandler( createSetTransform( inTransform.getType(), inTransform.getData() ) ); }
		PX_INLINE TReturnType multiplyTransform( const RenderTransform& inTransform ) { return mHandler( createMultiplyTransform( inTransform.getType(), inTransform.getData() ) ); }
		PX_INLINE TReturnType popTransform() { return mHandler( createPopTransform() ); }

		PX_INLINE TReturnType drawPrimitive(const RenderPrimitive& inPrimitive, const RenderTransform& inTransform) 
		{ 
			return mHandler( createDrawPrimitive( inTransform.getType(), inTransform.getData(), inPrimitive.getType(), inPrimitive.getData() ) );
		}
		PX_INLINE TReturnType drawPrimitive( const RenderPrimitive& inPrimitive, const RenderTransform& inTransform, const RenderState& inState )
		{
			return mHandler( createDrawRenderStatePrimitive( createDrawPrimitive( inTransform.getType(), inTransform.getData()
																				, inPrimitive.getType(), inPrimitive.getData() ), inState ) );
		}
	};

	//Unpack commands to interface functions.
	template<typename TReturnType, typename TInterfaceItem>
	struct PvdRenderCommandsToInterface
	{
		TInterfaceItem* mInterfaceItem;
		PvdRenderCommandsToInterface( TInterfaceItem* inItem ) : mInterfaceItem( inItem ) {}

		PX_INLINE TReturnType operator()() { return TReturnType(); }
		template<typename TDataType> PX_INLINE TReturnType operator()( const TDataType& ) { PX_ASSERT( false ); return TReturnType(); }

		PX_INLINE TReturnType operator()( const PushRenderState& ) { return mInterfaceItem->pushRenderState(); }
		PX_INLINE TReturnType operator()( const SetCurrentInstance& inItem ) { return mInterfaceItem->setCurrentInstance(inItem.mInstance); }
		PX_INLINE TReturnType operator()( const SetCurrentColor& inItem ) { return mInterfaceItem->setCurrentColor( inItem.mColor ); }
		PX_INLINE TReturnType operator()( const SetCurrentRenderFlags& inItem ) { return mInterfaceItem->setCurrentRenderFlags( inItem.mFlags ); }
		PX_INLINE TReturnType operator()( const AddCurrentRenderFlags& inItem ) { return mInterfaceItem->addCurrentRenderFlags( inItem.mFlags ); }
		PX_INLINE TReturnType operator()( const RemoveCurrentRenderFlags& inItem ) { return mInterfaceItem->removeCurrentRenderFlags( inItem.mFlags ); }
		PX_INLINE TReturnType operator()( const SetCurrentTextScale& inItem ) { return mInterfaceItem->setCurrentTextScale( inItem.mScale ); }
		PX_INLINE TReturnType operator()( const PopRenderState&) { return mInterfaceItem->popRenderState(); }

		PX_INLINE TReturnType operator()( const PushTransform&) { return mInterfaceItem->pushTransform(); }
		PX_INLINE TReturnType operator()( const SetTransform& inItem ) { return mInterfaceItem->setTransform( RenderTransform( inItem.mType, inItem.mData ) ); }
		PX_INLINE TReturnType operator()( const MultiplyTransform& inItem ) { return mInterfaceItem->multiplyTransform( RenderTransform( inItem.mType, inItem.mData ) ); }
		PX_INLINE TReturnType operator()( const PopTransform& ) { return mInterfaceItem->popTransform(); }

		PX_INLINE TReturnType operator()(const DrawPrimitive& inItem ) 
		{ 
			return mInterfaceItem->drawPrimitive( RenderPrimitive( inItem.mPrimitiveType, inItem.mPrimitiveData ), RenderTransform( inItem.mTransformType, inItem.mTransformData ) );
		}
		
		PX_INLINE TReturnType operator()(const DrawRenderStatePrimitive& inItem )
		{
			return mInterfaceItem->drawPrimitive(RenderPrimitive( inItem.mDrawPrimitive.mPrimitiveType, inItem.mDrawPrimitive.mPrimitiveData )
												, RenderTransform( inItem.mDrawPrimitive.mTransformType, inItem.mDrawPrimitive.mTransformData )
												, inItem.mRenderState );
		}
	};

	template<typename TReturnType, typename TInterfaceItem>
	inline TReturnType unpackRenderCommand( const RenderCommand& inCommand, TInterfaceItem* inInterface )
	{
		return inCommand.visit<TReturnType>( PvdRenderCommandsToInterface<TReturnType, TInterfaceItem>( inInterface ) );
	}
}
#endif
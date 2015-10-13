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

#ifndef PVD_PVDCONNECTIONEVENTSTREAM_H
#define PVD_PVDCONNECTIONEVENTSTREAM_H

#include "PVDConnectionEvents.h"
#include "PvdRenderTypes.h"

namespace PVD
{
	template<typename TStreamType>
	struct CommLayerStreamOperator
	{
		TStreamType* mStream;
		CommLayerStreamOperator( TStreamType* inStream ) : mStream( inStream ) {}
		template<typename TDatatype>
		PX_INLINE PvdCommLayerData operator()( TDatatype inData ) { mStream->streamify( inData ); return createCommLayerData( inData ); }
		PX_INLINE PvdCommLayerData operator()() { PvdCommLayerData retval; memset( &retval, 0, sizeof( retval ) ); return retval; }
	};
	
	template<typename TStreamType>
	struct CommLayerMediumStreamOperator
	{
		TStreamType* mStream;
		CommLayerMediumStreamOperator( TStreamType* inStream ) : mStream( inStream ) {}
		template<typename TDatatype>
		PX_INLINE PvdCommLayerMediumData operator()( TDatatype inData ) { mStream->streamify( inData ); return createCommLayerMediumData( inData ); }
		PX_INLINE PvdCommLayerMediumData operator()() { PvdCommLayerMediumData retval; memset( &retval, 0, sizeof( retval ) ); return retval; }
	};
	
	template<typename TStreamType>
	struct CommLayerSmallStreamOperator
	{
		TStreamType* mStream;
		CommLayerSmallStreamOperator( TStreamType* inStream ) : mStream( inStream ) {}
		template<typename TDatatype>
		PX_INLINE PvdCommLayerSmallData operator()( TDatatype inData ) { mStream->streamify( inData ); return createCommLayerSmallData( inData ); }
		PX_INLINE PvdCommLayerSmallData operator()() { PvdCommLayerSmallData retval; memset( &retval, 0, sizeof( retval ) ); return retval; }
	};

	template<typename TStreamType>
	struct RenderTransformStreamOperator
	{
		TStreamType* mStream;
		RenderTransformStreamOperator( TStreamType* inStream ) : mStream( inStream ) {}
		template<typename TDatatype>
		PX_INLINE RenderTransformData operator()( TDatatype inData ) { mStream->streamify( inData ); return toTransformData( inData ); }
		PX_INLINE RenderTransformData operator()() { RenderTransformData retval; memset( &retval, 0, sizeof( retval ) ); return retval; }
	};

	template<typename TStreamType>
	struct RenderPrimitiveStreamOperator
	{
		TStreamType* mStream;
		RenderPrimitiveStreamOperator( TStreamType* inStream ) : mStream( inStream ) {}
		template<typename TDatatype>
		PX_INLINE RenderPrimitiveData operator()( TDatatype inData ) { renderStreamify( *mStream, inData ); return toRenderPrimitiveData( inData ); }
		PX_INLINE RenderPrimitiveData operator()() { RenderPrimitiveData retval; memset( &retval, 0, sizeof( retval ) ); return retval; }
	};
}

#endif

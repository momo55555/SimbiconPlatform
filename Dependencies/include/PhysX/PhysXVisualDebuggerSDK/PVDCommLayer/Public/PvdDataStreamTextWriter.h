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

#ifndef PVD_PVDCONNECTIONSTREAMTEXTWRITER_H
#define PVD_PVDCONNECTIONSTREAMTEXTWRITER_H
#include "PvdCommLayerErrors.h"
#include "PvdConnectionEvents.h"

namespace PVD
{
	template<typename TWriterType
			, typename TDeleteOperator=SDeleteOperator>
	struct PvdDataStreamTextWriter
	{
		TWriterType* mWriter;
		PvdDataStreamTextWriter( TWriterType* inWriter )
			: mWriter( inWriter )
		{
		}
		
		inline void destroy() { TDeleteOperator()(this); }
		
		inline PvdCommLayerError flush()
		{
			return PvdCommLayerError::None;
		}
		inline bool isConnected() { return true; }

		inline PvdCommLayerError sendEvent( const SCreateInstance& inEvent )
		{
			return doSendEvent( inEvent );
		}
		inline PvdCommLayerError sendEvent( const SAddChild& inEvent )
		{
			return doSendEvent( inEvent );
		}
		inline PvdCommLayerError sendEvent( const SRemoveChild& inEvent )
		{
			return doSendEvent( inEvent );
		}
		inline PvdCommLayerError sendEvent( const SDestroyInstance& inEvent )
		{
			return doSendEvent( inEvent );
		}

		template<typename TDataType>
		inline PvdCommLayerError sendEvent( const TDataType& /*inEvent*/ )
		{
			return PvdCommLayerError::None;
		}

		template<typename TDataType>
		inline PvdCommLayerError doSendEvent( TDataType inEvent )
		{
			mWriter->beginEvent( inEvent.getEventType() );
			inEvent.textStreamify( *mWriter );
			mWriter->endEvent();
			return PvdCommLayerError::None;
		}
	};
}

#endif
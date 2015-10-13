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

#ifndef PVD_PVDCONTAINERCOMMOUTSTREAM_H
#define PVD_PVDCONTAINERCOMMOUTSTREAM_H

#include "PVDCommByteStream.h"

namespace PVD
{
	/**
	 *	Implementation of the PvdCommOutStream interface writing to a container
	 *	that only needs the standard insert function implemented.
	 */
	template<typename TPXU8Container
			, typename TDeleteOperator=SDeleteOperator>
	class PvdContainerCommOutStream : public PvdCommOutStream
	{
		TPXU8Container* mContainer;
		bool mConnected;
	public:
		PvdContainerCommOutStream( TPXU8Container* inContainer )
			: mContainer( inContainer )
			, mConnected( true )
		{
		}
		~PvdContainerCommOutStream(){ mConnected = false; }
		//Skip the vtable lookup
		PX_INLINE void doWriteBytes( const PxU8* inBytes, PxU32 inLength )
		{
			for ( PxU32 idx = 0; idx < inLength; ++idx )
				mContainer->pushBack( inBytes[idx] );
		}
		virtual PvdCommLayerError write( const PxU8* inBytes, PxU32 inLength )
		{
			doWriteBytes( inBytes, inLength );
			return PvdCommLayerError::None;
		}
		virtual PvdCommLayerError flush() { return PvdCommLayerError::None; }
		virtual bool isConnected() const { return mConnected; }
		virtual void disconnect(){ mConnected = false; }
		virtual void destroy() { TDeleteOperator()( this ); }
	};
}

#endif
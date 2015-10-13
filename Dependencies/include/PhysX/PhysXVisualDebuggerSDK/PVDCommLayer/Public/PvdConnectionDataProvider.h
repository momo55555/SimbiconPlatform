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

#ifndef PVDCONNECTIONDATAPROVIDERH
#define PVDCONNECTIONDATAPROVIDERH
#include "PVDCommLayerDatatypes.h"

namespace PVD
{
	class PvdCommInStream;

	class PvdConnectionDataHandler
	{
	protected:
		virtual ~PvdConnectionDataHandler(){}
	public:
		virtual void handleData( const PxU8* inData, PxU32 inLength ) = 0;
		virtual void handleNoMoreData() = 0;
	};

	class PvdConnectionRunningProvider
	{
	protected:
		virtual ~PvdConnectionRunningProvider(){}
	public:
		virtual void destroy() = 0;
	};

	/** 
	 *	Class abstracts reading data from the in stream and giving it to the data handler.
	 *	this is to allow implementations to avoid the read-thread on the PVD
	 *	socket layer if they would like to.
	 *	
	 *	You cannot do the read operation in a thread that calls simulate, however,
	 *	as the pvd system can pause the sending.  It does this by grabbing an event
	 *	that the scene grabs.  Thus you will get one of two results; the scene will
	 *	just ignore the pause or it will hang and there will be no way to 
	 *	get the scene out of the hung state.
	 *
	 *	Once begin handling data is called it will never be called again.
	 * */
	class PvdConnectionDataProvider
	{
	protected:
		virtual ~PvdConnectionDataProvider(){}
	public:
		/**
		 *	Will be called for each new connection.  Destroy will be called on the return value
		 *	when the handler no longer needs the data.
		 */
		virtual PvdConnectionRunningProvider* beginHandlingData( PvdConnectionDataHandler* inHandler, PvdCommInStream* inStream ) = 0;
	};
}
#endif
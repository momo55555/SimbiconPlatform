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

#ifndef PVD_PVDRENDER_H
#define PVD_PVDRENDER_H
#include "PvdRenderTypes.h"
#include "PVDQueryInterface.h"
#include "PVDCommLayerErrors.h"

namespace PVD
{
	/** 
	 *	Render interface to PVD.  Commands sent stored with the frame and replayed 
	 *	when the frame is syncced.
	 *	
	 *	All functions could return a network error at any time.  This should be
	 *	taken into account and dealt with accordingly.
	 */
	class PvdRender
	{
	protected:
		virtual ~PvdRender(){}
	public:

		/**
		 *	Push the render state allowing local modifications to
		 *	the render state to be undone with a pop command.
		 *	
		 *	Errors - StackOverflow
		 */
		virtual PvdCommLayerError pushRenderState() = 0;
		/** 
		 *	Set the current instance the following draw commands will be
		 *	associated with.  A zero instanceId is possible meaning the
		 *	next set of renderings will not be associated with any
		 *	given instance.
		 *	
		 *	Errors - InvalidInstance, if inInstanceId is nonzero but does not
		 *	refer to a known instance.
		 */
		virtual PvdCommLayerError setCurrentInstance( PxU64 inInstanceId ) = 0;
		/**
		 *	Set the color used for rendering any following primitives.
		 *	The color defaults to black so when it is unset primitives
		 *	are obvious against the background.
		 */
		virtual PvdCommLayerError setCurrentColor( Color inColor ) = 0;
		/**
		 *	Set the scale of the text when rendered.  Default is 1.0f,
		 *	meaning a character is about .2x.2 world units high.
		 */
		virtual PvdCommLayerError setCurrentTextScale( PxF32 inScale ) = 0;
		/**
		 *	Set the current render state's render flags
		 */
		virtual PvdCommLayerError  setCurrentRenderFlags( const RenderFlags& inFlags ) = 0;
		/**
		 *	Add these flags to the current state's render flags
		 */
		virtual PvdCommLayerError  addCurrentRenderFlags( const RenderFlags& inFlags ) = 0;
		/**
		 *	Remove these flags from the current state's render flags.
		 */
		virtual PvdCommLayerError  removeCurrentRenderFlags( const RenderFlags& inFlags ) = 0;
		/**
		 *	Pop the render state, restoring any previous settings.
		 *	
		 *	Errors - StackUnderflow.
		 */
		virtual PvdCommLayerError popRenderState() = 0;

		/**
		 *	Push the current transform onto the stack.  Does not
		 *	change the current transform.
		 *
		 *	Errors - StackOverflow
		 */
		virtual PvdCommLayerError pushTransform() = 0;

		/**
		 *	Set the current transform to the desired transform.
		 */
		virtual PvdCommLayerError setTransform(const RenderTransform& inTransform) = 0;

		/**
		 *	Multiply the current transform by the incoming transform, replacing
		 *	the current transform.
		 */
		virtual PvdCommLayerError multiplyTransform( const RenderTransform& inTransform ) = 0;

		/**
		 *	Pop the current transform, replacing the current transform with
		 *	the top transform on the transform stack.
		 *
		 *	Errors - StackUnderflow
		 */
		virtual PvdCommLayerError popTransform() = 0;

		
		/** 
		 *	Render a primitive using the top of stack transform.  This is conceptually
		 *	identical to drawPrimitive( createIdentityTransform )
		 */
		virtual PvdCommLayerError drawPrimitive(const RenderPrimitive& inPrimitive) = 0;

		/**
		 *	Render a given primitive.  The transform may be passed in as the symbolic
		 *	identity, in which case no transform information will be sent.
		 *	
		 *	See the list of primitives described at PVD::RenderPrimitiveType.
		 */
		virtual PvdCommLayerError drawPrimitive(const RenderPrimitive& inPrimitive, const RenderTransform& inTransform) = 0;

		/**
		 * Render a given primitive, completely replacing the render state
		 * with the render state specified.
		 */
		virtual PvdCommLayerError drawPrimitive(const RenderPrimitive& inPrimitive, const RenderTransform& inTransform, const RenderState& inState ) = 0;
	};
}

#endif

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


#if PX_SUPPORT_VISUAL_DEBUGGER

#include "PvdRenderAdapter.h"
#include "PvdDataStream.h"

namespace physx
{
namespace Pvd
{

	static inline void addToBuffer( Ps::Array<PxF32>& ioBuffer, const PxVec3& inVec )
	{
		ioBuffer.pushBack( inVec[0] );
		ioBuffer.pushBack( inVec[1] );
		ioBuffer.pushBack( inVec[2] );
	}
	void PvdRenderAdapter::visualize(PVD::PvdDataStream& pvdConnection, PxU64 instance)
	{
		PX_ASSERT(pvdConnection.isConnected());

		PVD::PvdCommLayerError error;
		error = pvdConnection.pushRenderState();
		error = pvdConnection.pushTransform();
		error = pvdConnection.setTransform(PVD::createIdentityTransform());

		if(instance)
			error = pvdConnection.setCurrentInstance(instance);

		PxDebugPoint* debugPoint = mBuffer.mPoints.begin();
		PxU32 numPoints = mBuffer.mPoints.size();
		while(numPoints--)
		{
			addToBuffer( mF32Buf, debugPoint->pos );
			mU32Buf.pushBack( debugPoint->color );
			mPointRadiiBuf.pushBack( .05f );
			debugPoint++;
		}
		error = pvdConnection.drawPrimitive(PVD::createPoints(
													reinterpret_cast< PVD::Float3* >( mF32Buf.begin() )
													, mPointRadiiBuf.begin()
													, reinterpret_cast< PVD::Color* >( mU32Buf.begin() )
													, mBuffer.mPoints.size() ));

		mU32Buf.clear();
		mF32Buf.clear();
		mPointRadiiBuf.clear();

		PxDebugLine* debugLine = mBuffer.mLines.begin();
		PxU32 numLines = mBuffer.mLines.size();
		if ( numLines )
		{
			while(numLines--)
			{
				addToBuffer( mF32Buf, debugLine->pos0 );
				addToBuffer( mF32Buf, debugLine->pos1 );
				mU32Buf.pushBack( debugLine->color0 );
				mU32Buf.pushBack( debugLine->color1 );
				debugLine++;
			}
		
			error = pvdConnection.drawPrimitive(PVD::createGradientLines(
													reinterpret_cast< PVD::Float3* >( mF32Buf.begin() )
													, reinterpret_cast< PVD::Color* >( mU32Buf.begin() )
													, mBuffer.mLines.size() ));
		}

		mU32Buf.clear();
		mF32Buf.clear();
		PxDebugTriangle* debugTriangle = mBuffer.mTriangles.begin();
		PxU32 numTri = mBuffer.mTriangles.size();
		if ( numTri )
		{
			while(numTri--)
			{
				
				mU32Buf.pushBack( debugTriangle->color0 );
				addToBuffer( mF32Buf, debugTriangle->pos0 );
				addToBuffer( mF32Buf, debugTriangle->pos1 );
				addToBuffer( mF32Buf, debugTriangle->pos2 );
				debugTriangle++;
			}
			
			error = pvdConnection.drawPrimitive(PVD::createTriangles(
													reinterpret_cast< PVD::Float3* >( mF32Buf.begin() )
													, reinterpret_cast< PVD::Color* >( mU32Buf.begin() )
													, mBuffer.mTriangles.size() ));
		}
		
		mU32Buf.clear();
		mF32Buf.clear();

		PxDebugText* debugText = mBuffer.mTexts.begin();
		PxU32 numTexts = mBuffer.mTexts.size();
		while(numTexts--)
		{
			error = pvdConnection.setCurrentColor(PVD::createColor(debugText->color));
			error = pvdConnection.setCurrentTextScale(debugText->size);
			error = pvdConnection.drawPrimitive(PVD::createString(debugText->string), PVD::createPosition(PVD::createFloat3(debugText->position)));
			debugText++;
		}

		mBuffer.clear();

		error = pvdConnection.popTransform();
		error = pvdConnection.popRenderState();
	}

} // namespace Pvd

}

#endif  // PX_SUPPORT_VISUAL_DEBUGGER

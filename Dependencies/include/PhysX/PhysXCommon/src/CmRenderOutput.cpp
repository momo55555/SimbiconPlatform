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


#include "CmRenderOutput.h"
#include "PsMathUtils.h"

#include <stdio.h>
#include <stdarg.h>

#ifdef PX_LINUX
#include <stdint.h> // intptr_t
#endif

#pragma warning(disable:4996) // intentionally suppressing this warning message

namespace physx
{
namespace Cm
{
	RenderOutput& RenderOutput::operator<<(Primitive prim)
	{
		mPrim = prim; mVertexCount = 0; return *this;
	}

	RenderOutput& RenderOutput::operator<<(PxU32 color)
	{
		mColor = color; return *this;
	}

	RenderOutput& RenderOutput::operator<<(const PxMat44& transform)
	{
		mTransform = transform; return *this;
	}

	RenderOutput& RenderOutput::operator<<(const PxTransform&t)
	{
		mTransform = PxMat44(t);
		return *this;
	}

	RenderOutput& RenderOutput::operator<<(PxVec3 vertex)
	{
		// apply transformation
		vertex = mTransform.transform(vertex);
		++mVertexCount;

		// add primitive to render buffer
		switch(mPrim)
		{
		case POINTS:
			mBuffer.mPoints.pushBack(PxDebugPoint(vertex, mColor)); break;
		case LINES:
			if(mVertexCount == 2)
			{
				mBuffer.mLines.pushBack(PxDebugLine(mVertex0, vertex, mColor)); 
				mVertexCount = 0;
			}
			break;
		case LINESTRIP:
			if(mVertexCount >= 2)
				mBuffer.mLines.pushBack(PxDebugLine(mVertex0, vertex, mColor)); 
			break;
		case TRIANGLES:
			if(mVertexCount == 3)
			{
				mBuffer.mTriangles.pushBack(PxDebugTriangle(mVertex1, mVertex0, vertex, mColor)); 
				mVertexCount = 0;
			}
			break;
		case TRIANGLESTRIP:
			if(mVertexCount >= 3)
				mBuffer.mTriangles.pushBack(PxDebugTriangle(
					(mVertexCount & 0x1) ? mVertex0 : mVertex1, 
					(mVertexCount & 0x1) ? mVertex1 : mVertex0, vertex, mColor)); 
			break;
		}

		// cache the last 2 vertices (for strips)
		if(1 < mVertexCount)
		{
			mVertex1 = mVertex0;
			mVertex0 = vertex;
		} else {
			mVertex0 = vertex;
		}
		return *this;
	}

	DebugText::DebugText(const PxVec3& position, PxReal size, const char* string, ...) 
	: position(position), size(size) 
	{
		va_list argList;
		va_start(argList, string);
		if(0 >= vsnprintf(buffer, sBufferSize-1, string, argList))
			buffer[sBufferSize-1] = 0; // terminate string
		va_end(argList);
	}

	RenderOutput& RenderOutput::operator<<(const DebugText& text)
	{
		PxU32 n = PxU32(strlen(text.buffer));
		PxU32 newCharBufSize = mBuffer.mCharBuf.size()+n+1;
		if (mBuffer.mCharBuf.capacity() < newCharBufSize)
		{
			char* oldBuf = mBuffer.mCharBuf.begin(); 
			mBuffer.mCharBuf.reserve(newCharBufSize);
			intptr_t diff =  mBuffer.mCharBuf.begin() - oldBuf;
			for (PxU32 i = 0; i < mBuffer.mTexts.size(); ++i)
				mBuffer.mTexts[i].string += diff;
		}
		mBuffer.mTexts.pushBack(PxDebugText(text.position, text.size, mColor, mBuffer.mCharBuf.end()));
		for(size_t i=0; i<=n; ++i)
			mBuffer.mCharBuf.pushBack(text.buffer[i]);
		return *this;
	}

	RenderOutput& operator<<(RenderOutput& out, const DebugBox& box)
	{
		if(box.wireframe)
		{
			out << RenderOutput::LINESTRIP;
			out << PxVec3(box.minimum.x, box.minimum.y, box.minimum.z);
			out << PxVec3(box.maximum.x, box.minimum.y, box.minimum.z);
			out << PxVec3(box.maximum.x, box.maximum.y, box.minimum.z);
			out << PxVec3(box.minimum.x, box.maximum.y, box.minimum.z);
			out << PxVec3(box.minimum.x, box.minimum.y, box.minimum.z);
			out << PxVec3(box.minimum.x, box.minimum.y, box.maximum.z);
			out << PxVec3(box.maximum.x, box.minimum.y, box.maximum.z);
			out << PxVec3(box.maximum.x, box.maximum.y, box.maximum.z);
			out << PxVec3(box.minimum.x, box.maximum.y, box.maximum.z);
			out << PxVec3(box.minimum.x, box.minimum.y, box.maximum.z);
			out << RenderOutput::LINES;
			out << PxVec3(box.maximum.x, box.minimum.y, box.minimum.z); 
			out << PxVec3(box.maximum.x, box.minimum.y, box.maximum.z);
			out << PxVec3(box.maximum.x, box.maximum.y, box.minimum.z); 
			out << PxVec3(box.maximum.x, box.maximum.y, box.maximum.z);
			out << PxVec3(box.minimum.x, box.maximum.y, box.minimum.z); 
			out << PxVec3(box.minimum.x, box.maximum.y, box.maximum.z);
		} else {
			out << RenderOutput::TRIANGLESTRIP;
			out << PxVec3(box.minimum.x, box.minimum.y, box.minimum.z); // 0
			out << PxVec3(box.minimum.x, box.maximum.y, box.minimum.z); // 2
			out << PxVec3(box.maximum.x, box.minimum.y, box.minimum.z); // 1
			out << PxVec3(box.maximum.x, box.maximum.y, box.minimum.z); // 3
			out << PxVec3(box.maximum.x, box.maximum.y, box.maximum.z); // 7
			out << PxVec3(box.minimum.x, box.maximum.y, box.minimum.z); // 2
			out << PxVec3(box.minimum.x, box.maximum.y, box.maximum.z); // 6
			out << PxVec3(box.minimum.x, box.minimum.y, box.minimum.z); // 0
			out << PxVec3(box.minimum.x, box.minimum.y, box.maximum.z); // 4
			out << PxVec3(box.maximum.x, box.minimum.y, box.minimum.z); // 1
			out << PxVec3(box.maximum.x, box.minimum.y, box.maximum.z); // 5
			out << PxVec3(box.maximum.x, box.maximum.y, box.maximum.z); // 7
			out << PxVec3(box.minimum.x, box.minimum.y, box.maximum.z); // 4
			out << PxVec3(box.minimum.x, box.maximum.y, box.maximum.z); // 6
		}
		return out;
	}

	RenderOutput& operator<<(RenderOutput& out, const DebugArrow& arrow)
	{
		PxVec3 t0 = arrow.tip - arrow.base, t1, t2; 

		t0.normalize();
		Ps::normalToTangents(t0, t1, t2);

		static const PxReal tipAngle = 0.25f;
		t1 *= arrow.headLength * tipAngle; 
		t2 *= arrow.headLength * tipAngle * PxSqrt(3.0f); 
		PxVec3 headBase = arrow.tip - t0 * arrow.headLength;

		out << RenderOutput::LINES;
		out << arrow.base << arrow.tip;

		out << RenderOutput::TRIANGLESTRIP;
		out << arrow.tip;
		out << headBase + t1 + t1;
		out << headBase - t1 - t2;
		out << headBase - t1 + t2;
		out << arrow.tip;
		out << headBase + t1 + t1;
		return out;
	}

	RenderOutput& operator<<(RenderOutput& out, const DebugBasis& basis)
	{
		PxReal headLength = basis.extends.magnitude() * 0.15f;
		out << basis.colorX << DebugArrow(PxVec3(0.0f), PxVec3(basis.extends.x, 0, 0), headLength);
		out << basis.colorY << DebugArrow(PxVec3(0.0f), PxVec3(0, basis.extends.y, 0), headLength);
		out << basis.colorZ << DebugArrow(PxVec3(0.0f), PxVec3(0, 0, basis.extends.z), headLength);
		return out;
	}

	RenderOutput& operator<<(RenderOutput& out, const DebugCircle& circle)
	{
		PxF32 step = PxTwoPi/circle.nSegments, angle = 0;
		out << RenderOutput::LINESTRIP;
		for(PxU32 i=0; i<circle.nSegments; i++, angle += step)
			out << PxVec3(circle.radius * PxSin(angle), circle.radius * PxCos(angle), 0);
		out << PxVec3(0, circle.radius, 0);
		return out;
	}

	RenderOutput& operator<<(RenderOutput& out, const DebugArc& arc)
	{
		PxF32 step = (arc.maxAngle - arc.minAngle) / arc.nSegments, angle = arc.minAngle;
		out << RenderOutput::LINESTRIP;
		for(PxU32 i=0; i<arc.nSegments; i++, angle += step)
			out << PxVec3(arc.radius * PxSin(angle), arc.radius * PxCos(angle), 0);
		out << PxVec3(arc.radius * PxSin(arc.maxAngle), arc.radius * PxCos(arc.maxAngle), 0);
		return out;
	}

} // Cm

}

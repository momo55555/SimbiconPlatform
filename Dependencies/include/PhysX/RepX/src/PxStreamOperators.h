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
#ifndef PX_PXSTREAMOPERATORS_H
#define PX_PXSTREAMOPERATORS_H
#include "PxStream.h"
#include "PxVec3.h"
#include "PxTransform.h"
#include "PxBounds3.h"
#include "PxFiltering.h"
#include "PxProfileBase.h"

namespace physx
{

namespace 
{
	

	inline PxU32 strLen( const char* inStr )
	{
		PxU32 len = 0;
		if ( inStr )
		{
			while ( *inStr )
			{
				++len;
				++inStr;
			}
		}
		return len;
	}
}
inline PxStream& operator << ( PxStream& ioStream, const char* inString )
{
	if ( inString && *inString )
	{
		PxU32 len( strLen( inString ) );
		ioStream.storeBuffer( inString, len );
	}
	return ioStream;
}

template<typename TDataType>
inline PxStream& toStream( PxStream& ioStream, const char* inFormat, const TDataType inData )
{
	char buffer[128] = { 0 };
	sprintf( buffer, inFormat, inData );
	ioStream << buffer;
	return ioStream;
}

struct endl_obj {};
static endl_obj endl;

inline PxStream& operator << ( PxStream& ioStream, bool inData ) { ioStream << (inData ? "true" : "false"); return ioStream; }
inline PxStream& operator << ( PxStream& ioStream, PxU16 inData ) {	return toStream( ioStream, "%u", (PxU32)inData ); }
inline PxStream& operator << ( PxStream& ioStream, PxU8 inData ) {	return toStream( ioStream, "%u", (PxU32)inData ); }
inline PxStream& operator << ( PxStream& ioStream, PxU32 inData ) {	return toStream( ioStream, "%u", inData ); }
#ifdef PX_GNUC
inline PxStream& operator << ( PxStream& ioStream, PxU64 inData ) {	return toStream( ioStream, "%llu", inData ); }
#else
inline PxStream& operator << ( PxStream& ioStream, PxU64 inData ) {	return toStream( ioStream, "%I64u", inData ); }
#endif
inline PxStream& operator << ( PxStream& ioStream, const void* inData ) { return ioStream << PX_PROFILE_POINTER_TO_U64( inData ); }
inline PxStream& operator << ( PxStream& ioStream, PxF32 inData ) { return toStream( ioStream, "%g", (PxF64)inData ); }
inline PxStream& operator << ( PxStream& ioStream, PxF64 inData ) { return toStream( ioStream, "%g", inData ); }
inline PxStream& operator << ( PxStream& ioStream, endl_obj inData ) { return ioStream << "\n"; }
inline PxStream& operator << ( PxStream& ioStream, const PxVec3& inData ) 
{ 
	ioStream << inData[0];
	ioStream << " ";
	ioStream << inData[1];
	ioStream << " ";
	ioStream << inData[2];
	return ioStream;
}

inline PxStream& operator << ( PxStream& ioStream, const PxQuat& inData ) 
{
	ioStream << inData.x;
	ioStream << " ";
	ioStream << inData.y;
	ioStream << " ";
	ioStream << inData.z;
	ioStream << " ";
	ioStream << inData.w;
	return ioStream;
}

inline PxStream& operator << ( PxStream& ioStream, const PxTransform& inData ) 
{
	ioStream << inData.q;
	ioStream << " ";
	ioStream << inData.p;
	return ioStream;
}

inline PxStream& operator << ( PxStream& ioStream, const PxBounds3& inData )
{
	ioStream << inData.minimum;
	ioStream << " ";
	ioStream << inData.maximum;
	return ioStream;
}

inline PxStream& operator << ( PxStream& ioStream, const PxFilterData& inData )
{
	ioStream << inData.word0 << " " << inData.word1 << " " << inData.word2 << " " << inData.word3;
	return ioStream;
}

}

#endif
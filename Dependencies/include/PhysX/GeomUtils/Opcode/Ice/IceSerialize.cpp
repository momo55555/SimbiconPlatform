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


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "PsIntrinsics.h"
#include "IceSerialize.h"
#include "IceUtils.h"
#include "PsUserAllocated.h"

using namespace physx;
using namespace Ice;

void Ice::WriteChunk(PxU8 a, PxU8 b, PxU8 c, PxU8 d, PxStream& stream)
{
	stream.storeByte(a).storeByte(b).storeByte(c).storeByte(d);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Ice::WriteWord(PxU16 value, bool mismatch, PxStream& stream)
{
	if(mismatch)	Flip(value);
	stream.storeWord(value);
}

void Ice::WriteDword(PxU32 value, bool mismatch, PxStream& stream)
{
	if(mismatch)	Flip(value);
	stream.storeDword(value);
}

void Ice::WriteFloat(float value, bool mismatch, PxStream& stream)
{
	union { float f; PxU32 tmp; }; f =value;
	if(mismatch)	Flip(tmp);
	stream.storeDword(tmp);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Ice::ReadWordBuffer(PxU16* dest, PxU32 nb, bool mismatch, const PxStream& stream)
{
	stream.readBuffer(dest, sizeof(PxU16)*nb);
	if(mismatch)
	{
		for(PxU32 i=0;i<nb;i++)
		{
			Flip(dest[i]);
		}
	}
	return true;
}

bool Ice::ReadFloatBuffer(float* dest, PxU32 nb, bool mismatch, const PxStream& stream)
{
	stream.readBuffer(dest, sizeof(float)*nb);
	if(mismatch)
	{
		for(PxU32 i=0;i<nb;i++)
		{
			Flip(dest[i]);
		}
	}
	return true;
}

void Ice::WriteWordBuffer(const PxU16* src, PxU32 nb, bool mismatch, PxStream& stream)
{
	while(nb--)
	{
		PxU16 W = *src++;
		if(mismatch)	Flip(W);
		stream.storeWord(W);
	}
}

void Ice::WriteFloatBuffer(const float* src, PxU32 nb, bool mismatch, PxStream& stream)
{
	union { float f; PxU32 tmp; };
	while(nb--)
	{
		f = *src++;	
		if(mismatch)	Flip(tmp);
		stream.storeDword(tmp);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Ice::WriteHeader(PxU8 a, PxU8 b, PxU8 c, PxU8 d, PxU32 version, bool mismatch, PxStream& stream)
{
	// Store endianness
	PxU8 StreamFlags = LittleEndian();
	if(mismatch)	StreamFlags^=1;

	// Export header
	WriteChunk('I', 'C', 'E', StreamFlags, stream);	// ICE identifier
	WriteChunk(a, b, c, d, stream);					// Chunk identifier
//	stream.StoreDword(version);						// Version number
	WriteDword(version, mismatch, stream);

	return true;
}

bool Ice::ReadHeader(PxU8 a_, PxU8 b_, PxU8 c_, PxU8 d_, PxU32& version, bool& mismatch, const PxStream& stream)
{
	// Import header
	PxU8 a, b, c, d;
	ReadChunk(a, b, c, d, stream);
	if(a!='I' || b!='C' || c!='E')
		return false;

	PxU8 FileLittleEndian = d&1;
	mismatch = FileLittleEndian!=LittleEndian();

	ReadChunk(a, b, c, d, stream);
	if(a!=a_ || b!=b_ || c!=c_ || d!=d_)
		return false;

	version = ReadDword(mismatch, stream);

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Ice::StoreIndices(PxU32 maxIndex, PxU32 nbIndices, const PxU32* indices, PxStream& stream, bool platformMismatch)
	{
	if(maxIndex<=0xff)
		{
		for(PxU32 i=0;i<nbIndices;i++)
			stream.storeByte((PxU8)indices[i]);
		}
	else if(maxIndex<=0xffff)
		{
		for(PxU32 i=0;i<nbIndices;i++)
			WriteWord(indices[i], platformMismatch, stream);
		}
	else
		{
//		WriteDwordBuffer(indices, nbIndices, platformMismatch, stream);
		for(PxU32 i=0;i<nbIndices;i++)
			WriteDword(indices[i], platformMismatch, stream);
		}
	}

void Ice::ReadIndices(PxU32 maxIndex, PxU32 nbIndices, PxU32* indices, const PxStream& stream, bool platformMismatch)
	{
	if(maxIndex<=0xff)
		{
		PxU8* tmp = (PxU8*)PxAlloca(nbIndices*sizeof(PxU8));
		stream.readBuffer(tmp, nbIndices*sizeof(PxU8));
		for(PxU32 i=0;i<nbIndices;i++)
			indices[i] = tmp[i];
//		for(PxU32 i=0;i<nbIndices;i++)
//			indices[i] = stream.ReadByte();
		}
	else if(maxIndex<=0xffff)
		{
		PxU16* tmp = (PxU16*)PxAlloca(nbIndices*sizeof(PxU16));
		ReadWordBuffer(tmp, nbIndices, platformMismatch, stream);
		for(PxU32 i=0;i<nbIndices;i++)
			indices[i] = tmp[i];
//		for(PxU32 i=0;i<nbIndices;i++)
//			indices[i] = ReadWord(platformMismatch, stream);
		}
	else
		{
		ReadDwordBuffer(indices, nbIndices, platformMismatch, stream);
		}
	}

void Ice::StoreIndices(PxU16 maxIndex, PxU32 nbIndices, const PxU16* indices, PxStream& stream, bool platformMismatch)
	{
	if(maxIndex<=0xff)
		{
		for(PxU32 i=0;i<nbIndices;i++)
			stream.storeByte((PxU8)indices[i]);
		}
	else
		{
		for(PxU32 i=0;i<nbIndices;i++)
			WriteWord(indices[i], platformMismatch, stream);
		}
	}

void Ice::ReadIndices(PxU16 maxIndex, PxU32 nbIndices, PxU16* indices, const PxStream& stream, bool platformMismatch)
	{
	if(maxIndex<=0xff)
		{
		PxU8* tmp = (PxU8*)PxAlloca(nbIndices*sizeof(PxU8));
		stream.readBuffer(tmp, nbIndices*sizeof(PxU8));
		for(PxU32 i=0;i<nbIndices;i++)
			indices[i] = tmp[i];
//		for(PxU32 i=0;i<nbIndices;i++)
//			indices[i] = stream.ReadByte();
		}
	else
		{
		PxU16* tmp = (PxU16*)PxAlloca(nbIndices*sizeof(PxU16));
		ReadWordBuffer(tmp, nbIndices, platformMismatch, stream);
		for(PxU32 i=0;i<nbIndices;i++)
			indices[i] = tmp[i];
//		for(PxU32 i=0;i<nbIndices;i++)
//			indices[i] = ReadWord(platformMismatch, stream);
		}
	}

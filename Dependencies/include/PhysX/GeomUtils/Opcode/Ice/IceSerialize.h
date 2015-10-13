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
// Include Guard
#ifndef ICESERIALIZE_H
#define ICESERIALIZE_H

#include "PxStream.h"
#include "IceFPU.h"

namespace physx
{
namespace Ice
{

	void	WriteChunk(PxU8 a, PxU8 b, PxU8 c, PxU8 d, PxStream& stream);

	void	WriteWord(PxU16 value, bool mismatch, PxStream& stream);
	void	WriteDword(PxU32 value, bool mismatch, PxStream& stream);
	void	WriteFloat(float value, bool mismatch, PxStream& stream);

	bool	ReadWordBuffer(PxU16* dest, PxU32 nb, bool mismatch, const PxStream& stream);
	bool	ReadFloatBuffer(float* dest, PxU32 nb, bool mismatch, const PxStream& stream);
	void	WriteWordBuffer(const PxU16* src, PxU32 nb, bool mismatch, PxStream& stream);
	void	WriteFloatBuffer(const float* src, PxU32 nb, bool mismatch, PxStream& stream);

	bool	WriteHeader(PxU8 a, PxU8 b, PxU8 c, PxU8 d, PxU32 version, bool mismatch, PxStream& stream);
	bool	ReadHeader(PxU8 a_, PxU8 b_, PxU8 c_, PxU8 d_, PxU32& version, bool& mismatch, const PxStream& stream);

	PX_INLINE void Flip(PxU16& v)
	{
		PxU8* b = (PxU8*)&v;
		PxU8 temp = b[0];
		b[0] = b[1];
		b[1] = temp;
	}

	PX_INLINE void Flip(PxI16& v)
	{
		Flip((PxU16&)v);
	}

	PX_INLINE void Flip(PxU32& v)
	{
		PxU8* b = (PxU8*)&v;

        PxU8 temp = b[0];
		b[0] = b[3];
		b[3] = temp;
		temp = b[1];
		b[1] = b[2];
		b[2] = temp;
	}

	PX_INLINE void Flip(PxI32& v)
	{
		Flip((PxU32&)v);
	}

	PX_INLINE void Flip(PxF32& v)
	{
		Flip((PxU32&)v);
	}

	PX_INLINE	bool ReadDwordBuffer(PxU32* dest, PxU32 nb, bool mismatch, const PxStream& stream)
	{
		return ReadFloatBuffer((float*)dest, nb, mismatch, stream);
	}

	PX_INLINE	void WriteDwordBuffer(const PxU32* src, PxU32 nb, bool mismatch, PxStream& stream)
	{
		WriteFloatBuffer((const float*)src, nb, mismatch, stream);
	}

	void	StoreIndices(PxU32 maxIndex, PxU32 nbIndices, const PxU32* indices, PxStream& stream, bool platformMismatch);
	void	ReadIndices(PxU32 maxIndex, PxU32 nbIndices, PxU32* indices, const PxStream& stream, bool platformMismatch);

	void	StoreIndices(PxU16 maxIndex, PxU32 nbIndices, const PxU16* indices, PxStream& stream, bool platformMismatch);
	void	ReadIndices(PxU16 maxIndex, PxU32 nbIndices, PxU16* indices, const PxStream& stream, bool platformMismatch);

	PX_INLINE void ReadChunk(PxU8& a, PxU8& b, PxU8& c, PxU8& d, const PxStream& stream)
	{
		a = stream.readByte();
		b = stream.readByte();
		c = stream.readByte();
		d = stream.readByte();
	}

	PX_INLINE PxU16 ReadWord(bool mismatch, const PxStream& stream)
	{
		PxU16 d = stream.readWord();
		if(mismatch)	Flip(d);
		return d;
	}

	PX_INLINE PxU32 ReadDword(bool mismatch, const PxStream& stream)
	{
		PxU32 d = stream.readDword();
		if(mismatch)	Flip(d);
		return d;
	}

	PX_INLINE float ReadFloat(bool mismatch, const PxStream& stream)
	{
		PxU32 d = stream.readDword();
		if(mismatch)	Flip(d);
		return FR(d);
	}
};

}

#endif // ICESERIALIZE_H

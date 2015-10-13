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
#ifndef PX_MEMORYPOOLSTREAMS_H
#define PX_MEMORYPOOLSTREAMS_H
#include "MemoryPool.h"
#include "PxStream.h"

namespace physx { namespace repx {

/** 
 *	Mapping of PxStream to a memory pool manager.
 *	Allows write-then-read semantics of a set of
 *	data.  Can safely write up to 4GB of data; then you
 *	will silently fail...
 */

template<typename TAllocatorType>
struct MemoryBufferBase : public PxStream
{
	TAllocatorType* mManager;
	mutable PxU32	mWriteOffset;
	mutable PxU32	mReadOffset;
	PxU8*	mBuffer;
	PxU32	mCapacity;


	MemoryBufferBase( TAllocatorType* inManager )
		: mManager( inManager )
		, mWriteOffset( 0 )
		, mReadOffset( 0 )
		, mBuffer( NULL )
		, mCapacity( 0 )
	{
	}
	virtual						~MemoryBufferBase()
	{
		mManager->deallocate( mBuffer );
	}
	PxU8* releaseBuffer()
	{
		clear();
		mCapacity = 0;
		PxU8* retval(mBuffer);
		mBuffer = NULL;
		return retval;
	}
	void clear()
	{
		mWriteOffset = mReadOffset = 0;
	}
	template<typename TDataType>
	inline TDataType read() const
	{
		bool fits = ( mReadOffset + sizeof( TDataType ) ) <= mWriteOffset;
		PX_ASSERT( fits );
		TDataType retval;
		if ( fits )
		{
			TDataType* theBufPtr = reinterpret_cast< TDataType* >( mBuffer + mReadOffset );
			retval = theBufPtr[0];
			mReadOffset += sizeof( TDataType );
		}
		return retval;
	}

	virtual		PxU8			readByte()								const { return read<PxU8>(); }
	virtual		PxU16			readWord()								const { return read<PxU16>(); }
	virtual		PxU32			readDword()								const { return read<PxU32>(); }
	virtual		float			readFloat()								const { return read<float>(); }
	virtual		double			readDouble()							const { return read<double>(); }
	virtual		void			readBuffer(void* buffer, PxU32 size)	const
	{
		bool fits = ( mReadOffset + size ) <= mWriteOffset;
		PX_ASSERT( fits );
		if ( fits )
		{
			memcpy( buffer, mBuffer + mReadOffset, size );
			mReadOffset += size;
		}
	}
	inline void checkCapacity( PxU32 inNewCapacity )
	{
		if ( mCapacity < inNewCapacity )
		{
			PxU32 newCapacity = 32;
			while( newCapacity < inNewCapacity )
				newCapacity = newCapacity << 1;

			PxU8* newData( mManager->allocate( newCapacity ) );
			if ( mWriteOffset )
				memcpy( newData, mBuffer, mWriteOffset );
			mManager->deallocate( mBuffer );
			mBuffer = newData;
			mCapacity = newCapacity;
		}
	}
	template<typename TDataType>
	inline PxStream& store( TDataType inValue )
	{
		checkCapacity( mWriteOffset + sizeof( TDataType ) );
		TDataType* theWritePtr( reinterpret_cast< TDataType* >( mBuffer + mWriteOffset ) );
		theWritePtr[0] = inValue;
		mWriteOffset += sizeof( TDataType );
		return *this;
	}

	virtual		PxStream&		storeByte(PxU8 b){ return store(b); }
	virtual		PxStream&		storeWord(PxU16 w) { return store(w); }
	virtual		PxStream&		storeDword(PxU32 d)	{ return store(d); }
	virtual		PxStream&		storeFloat(PxReal f) { return store(f); }
	virtual		PxStream&		storeDouble(PxF64 f) { return store(f); }

	virtual		PxStream&		storeBuffer(const void* buffer, PxU32 size)
	{
		checkCapacity( mWriteOffset + size );
		memcpy( mBuffer + mWriteOffset, buffer, size );
		mWriteOffset += size;
		return *this;
	}
};

class MemoryBuffer : public MemoryBufferBase<CMemoryPoolManager >
{
public:
	MemoryBuffer( CMemoryPoolManager* inManager ) : MemoryBufferBase<CMemoryPoolManager >( inManager ) {}
};

}}

#endif
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

#ifndef PVD_RING_BUFFER_H
#define PVD_RING_BUFFER_H
#include "Px.h"
#include "PxSimpleTypes.h"
#include "PsIntrinsics.h"
#include "PsSync.h"
#include "PsMutext.h"

namespace Pvd
{
	using namespace physx::pubfnd;
	using namespace physx::shdfnd;

	/**
	 *	Ring buffer to be used for POD types *only*.
	 */
	template<typename TPODDataType, typename TAllocator>
	class RingBuffer
	{
	public:
		
		struct RingBufferReadResult
		{
			const TPODDataType* mFirstBuffer;
			PxU32				mFirstBufLen;
			
			const TPODDataType* mSecondBuffer;
			PxU32				mSecondBufLen;

			RingBufferReadResult()
				: mFirstBuffer( 0 )
				, mFirstBufLen( 0 )
				, mSecondBuffer( 0 )
				, mSecondBufLen ( 0 )
			{
			}
		};


		TAllocator		mAllocator;
		TPODDataType*	mBufferStart;
		TPODDataType*	mBufferEnd;
		TPODDataType*	mReadPtr; //traveling ptr along wthe buffer
		PxU32			mSize; //amount of data in buffer <= buffersize.
		PxU32			mCapacity;


		RingBuffer( const RingBuffer& );
		RingBuffer& operator=( const RingBuffer& );

	public:
		RingBuffer( PxU32 inCapacity, const &TAllocator inAllocator = TAllocator() )
			: mAllocator( inAllocator )
			, mCapacity( inCapacity )
			, mBufferStart( mAllocator.allocate( mCapacity ) )
			, mBufferEnd( mBufferStart + mCapacity )
			, mReadPtr( mBufferStart )
			, mSize( 0 )
		{
		}

		PxU32 write( const TPODDataType* inData, PxU32 inLen )
		{
			PxU32 possible = PxMin( mCapacity - mSize, inLen );
			if ( possible )
			{
				TPODDataType* writePtr = mReadPtr + mSize;
				PxU32 nextWriteAmount = 0;
				if ( writePtr < mBufferEnd )
				{
					nextWriteAmount = PxMin( static_cast<PxU32>( mBufferEnd - writePtr ), possible );
					memCopy( writePtr, inData, nextWriteAmount );
					possible -= nextWriteAmount;
					inData += nextWriteAmount;
					writePtr = mBufferStart;
				}
				else
					writePtr = mBufferStart + (writePtr - mBufferEnd);
				if ( possible )
					memCopy( writePtr, inData, possible );
				mSize += possible;
			}
			return possible;
		}

		PxU32 size() const { return mSize; }
		bool empty() const { return mSize == 0; }

		//Get a buffer containing the read result.
		//This empties the entire ring buffer.
		RingBufferReadResult read()
		{
			PxU32 theSize = size();
			RingBufferReadResult theResult;
			if ( theSize )
			{
				PxU32 sizeTillEnd = static_cast<PxU32>( mBufferEnd - mReadPtr );
				if ( sizeTillEnd == 0 )
				{
					mReadPtr = mBufferStart;
					sizeTillEnd = mCapacity;
				}
				theResult.mFirstBuffer = mReadPtr;
				theResult.mFirstBufLen = PxMin( sizeTillEnd, theSize );
				theSize -= theResult.mFirstBufLen;
				updateReadPtr( theResult.mFirstBufLen );
				if ( theSize )
				{
					theResult.mSecondBuffer = mReadPtr;
					theResult.mSecondBufLen = theSize;
					mReadPtr += theSize;
				}
				mSize -= theSize;
				PX_ASSERT( empty() );
			}
			return theResult;
		}

	private:
		void updateReadPtr( PxU32 inAmount )
		{
			mReadPtr += inAmount;
			if ( mReadPtr == mBufferEnd )
				mReadPtr = mBufferStart;
		}
		
	};


	/**
	 *	Ring buffer to be used for POD types *only*.
	 */
	template<typename TPODDataType, typename TAllocator>
	class MTRingBuffer
	{
		typedef RingBuffer<TPODDataType, TAllocator> TRingBufferType;
		typedef Mutex::ScopedLock TLockType;

		Mutex				mMutex;
		Sync				mEmptySync;
		Sync				mWriteSync;
		TRingBufferType		mRingBuffer;
	public:

		MTRingBuffer( PxU32 inCapacity, const &TAllocator inAllocator = TAllocator() )
			: mRingBuffer( inCapacity, inAllocator )
		{
		}
		
		PxU32 write( const TPODDataType* inData, PxU32 inLen )
		{
			PxU32 retval = 0;
			{
				TLockType theLock( mMutex );
				retval = mRingBuffer.write( inData, inLen );
			}
			mWriteSync.set();
			return retval;
		}

		//Wait until we definitely can read at least one byte.
		void waitForRead( PxU32 milliseconds )
		{
			mWriteSync.wait( milliseconds );
		}

		//Wait until we definitely can write at least one byte.
		void waitForWrite( PxU32 milliseconds )
		{
			mEmptySync.wait( milliseconds );
		}
		
		PxU32 size() const { return mRingBuffer.size(); }
		bool empty() const { return mRingBuffer.empty(); }

		template<typename TReadOperator>
		void read( TReadOperator inOperator )
		{
			{
				TLockType theLock( mMutex );	
				RingBufferReadResult theReadResult( mRingBuffer.read() );
				inOperator( theReadResult );
			}
			mEmptySync.set();
		}


	};

}

#endif
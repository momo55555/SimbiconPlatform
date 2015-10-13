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
#include "PsMutex.h"

namespace PVD
{
	using namespace physx::pubfnd;
	using namespace physx::shdfnd;

	template<typename TPODDatatype, typename TAllocator>
	class PvdDoubleBufferedWriteBuffer
	{
	public:
		typedef Array<TPODDatatype, TAllocator> TArrayType;
		typedef Mutex::ScopedLock				TLockType;
		struct ReadResult
		{
			const TPODDatatype* mPtr;
			PxU32				mLen;

			ReadResult( const TPODDatatype* inPtr, PxU32 inLen )
				: mPtr( inPtr )
				, mLen( inLen )
			{
			}
		};

	private:

		PxU32		mCapacity;
		TArrayType	mFirstArray;
		TArrayType	mSecondArray;


		TPODDatatype*		mWritePtr;
		TPODDatatype*		mWriteEndPtr;

		const TPODDatatype* mReadPtr;

		Mutex				mMutex;
		Sync				mReadyToWrite;
		Sync				mReadyToRead;

	public:
		PvdDoubleBufferedWriteBuffer( PxU32 inCap, const TAllocator& inAlloc = TAllocator() )
			: mCapacity( inCap / 2 )
			, mFirstArray( inAlloc )
			, mSecondArray( inAlloc )
			, mReadPtr( NULL )
		{
			mFirstArray.resize( mCapacity );
			mSecondArray.resize( mCapacity );

			mWritePtr = mFirstArray.begin();
			mWriteEndPtr = mFirstArray.end();
		}

		//Write data to the write buffer.
		PxU32 write( const TPODDatatype* inData, PxU32 inLen )
		{
			PxU32 possible = 0;
			{
				TLockType theLock( mMutex );
				possible = PxMin( static_cast<PxU32>( mWriteEndPtr - mWritePtr ), inLen );
				if ( possible )
				{
					memCopy( mWritePtr, inData, possible );
					mWritePtr += possible;
				}
			}
			mReadyToRead.set();
			mReadyToRead.reset();
			return possible;
		}

		//If there is any data in the write buffer, swap the
		//write buffer with the read buffer and return the data.
		ReadResult read()
		{
			const TPODDatatype* theReadPtr = NULL;
			PxU32				theReadSize = 0;
			{
				TLockType theLock( mMutex );
				PxU32 possible = mCapacity - static_cast<PxU32>( mWriteEndPtr - mWritePtr );
				if ( possible )
				{
					//Swap the read/write pointers with the mutex held
					bool usingSecondArray = mWriteEndPtr == mSecondArray.end();
					const TPODDatatype* theBeginPtr( mFirstArray.begin() );
					mWritePtr = mSecondArray.begin();
					mWriteEndPtr = mSecondArray.end();
					if ( usingSecondArray )
					{
						theBeginPtr = mSecondArray.begin();
						mWritePtr = mFirstArray.begin();
						mWriteEndPtr = mFirstArray.end();
					}
					theReadPtr = theBeginPtr;
					theReadSize = possible;
				}
			}
			mReadyToWrite.set();
			mReadyToWrite.reset();
			return ReadResult( theReadPtr, theReadSize );
		}

		void waitTillReadyToWrite( PxU32 inMilliseconds ) { mReadyToWrite.wait( inMilliseconds ); }
		void waitTillReadyToRead( PxU32 inMilliseconds ) { mReadyToRead.wait( inMilliseconds ); }

	};
}

#endif
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
#ifndef REPX_CORE_EXTENSION_DESERIALIZER_H
#define REPX_CORE_EXTENSION_DESERIALIZER_H
#include "RepXVisitorReader.h"

namespace physx { namespace repx {

	struct RepXMemoryAllocateMemoryPoolAllocator
	{
		RepXMemoryAllocator* mAllocator;
		RepXMemoryAllocateMemoryPoolAllocator( RepXMemoryAllocator* inAlloc ) : mAllocator( inAlloc ) {}

		PxU8* allocate( PxU32 inSize ) { return mAllocator->allocate( inSize ); }
		void deallocate( PxU8* inMem ) { mAllocator->deallocate( inMem ); }
	};

	inline void strto( Triangle<PxU32>& ioDatatype, char*& ioData )
	{
		strto( ioDatatype.mIdx0, ioData );
		strto( ioDatatype.mIdx1, ioData );
		strto( ioDatatype.mIdx2, ioData );
	}

	inline void strto( PxHeightFieldSample& ioDatatype, char*& ioData )
	{
		PxU32 tempData;
		strto( tempData, ioData );
		if ( isBigEndian() )
		{
			PxU32& theItem(tempData);
			PxU32 theDest;
			PxU8* theReadPtr( reinterpret_cast< PxU8* >( &theItem ) );
			PxU8* theWritePtr( reinterpret_cast< PxU8* >( &theDest ) );
			//A height field sample is a 16 bit number
			//followed by two bytes.

			//We write this out as a 32 bit integer, LE.
			//Thus, on a big endian, we need to move the bytes
			//around a bit.
			//LE - 1 2 3 4
			//BE - 4 3 2 1 - after convert from xml number
			//Correct BE - 2 1 3 4, just like LE but with the 16 number swapped
			theWritePtr[0] = theReadPtr[2];
			theWritePtr[1] = theReadPtr[3];
			theWritePtr[2] = theReadPtr[1];
			theWritePtr[3] = theReadPtr[0];
			theItem = theDest;
		}
		ioDatatype = *reinterpret_cast<PxHeightFieldSample*>( &tempData );
	}

	template<typename TDataType>
	inline void readStridedBufferProperty( RepXReader& ioReader, const char* inPropName, void*& outData, PxU32& outStride, PxU32& outCount, RepXMemoryAllocator& inAllocator)
	{
		const char* theSrcData;
		outStride = sizeof( TDataType );
		outData = NULL;
		outCount = 0;
		if ( ioReader.read( inPropName, theSrcData ) )
		{
			RepXMemoryAllocateMemoryPoolAllocator tempAllocator( &inAllocator );
			MemoryBufferBase<RepXMemoryAllocateMemoryPoolAllocator> tempBuffer( &tempAllocator );

			if ( theSrcData )
			{
				static PxU32 theCount = 0;
				++theCount;
				char* theStartData = const_cast< char*>( copyStr( &tempAllocator, theSrcData ) );
				char* theData = theStartData;
				PxU32 theLen = strLen( theData );
				char* theEndData = theData + theLen;
				while( theData < theEndData )
				{
					//These buffers are whitespace delimited.
					TDataType theType;
					strto( theType, theData );
					tempBuffer.store( theType );
				}
				outData = reinterpret_cast< TDataType* >( tempBuffer.mBuffer );
				outCount = tempBuffer.mWriteOffset / sizeof( TDataType );
				tempAllocator.deallocate( (PxU8*)theStartData );
			}
			tempBuffer.releaseBuffer();
		}
	}
	
	template<typename TDataType>
	inline void readStridedBufferProperty( RepXReader& ioReader, const char* inPropName, PxStridedData& ioData, PxU32& outCount, RepXMemoryAllocator& inAllocator)
	{
		void* tempData = NULL;
		readStridedBufferProperty<TDataType>( ioReader, inPropName, tempData, ioData.stride, outCount, inAllocator ); 
		ioData.data = tempData;
	}
	
	template<typename TDataType>
	inline void readStridedBufferProperty( RepXReader& ioReader, const char* inPropName, PxTypedStridedData<TDataType>& ioData, PxU32& outCount, RepXMemoryAllocator& inAllocator)
	{
		void* tempData = NULL;
		readStridedBufferProperty<TDataType>( ioReader, inPropName, tempData, ioData.stride, outCount, inAllocator );
		ioData.data = reinterpret_cast<PxMaterialTableIndex*>( tempData );
	}

	template<typename TDataType>
	inline void readStridedBufferProperty( RepXReader& ioReader, const char* inPropName, PxBoundedData& ioData, RepXMemoryAllocator& inAllocator)
	{
		return readStridedBufferProperty<TDataType>( ioReader, inPropName, ioData, ioData.count, inAllocator );
	}

}}

#endif

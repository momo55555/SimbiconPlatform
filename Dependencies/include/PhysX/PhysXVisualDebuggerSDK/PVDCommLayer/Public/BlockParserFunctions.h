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


#ifndef PVD_BLOCKPARSERFUNCTIONS_H
#define PVD_BLOCKPARSERFUNCTIONS_H

#include "PVDCommLayerDatatypes.h"
#include <string.h>//memcpy

namespace PVD
{
	struct BlockParserDataConverter
	{
		union
		{
			PxU8	mU8[8];
			PxU16	mU16[4];
			PxU32	mU32[2];
			PxU64	mU64[1];
			
			PxI8	mI8[8];
			PxI16	mI16[4];
			PxI32	mI32[2];
			PxI64	mI64[1];


			PxF32	mF32[2];
			PxF64	mF64[1];
		};

		template<typename TDataType> inline TDataType convert() { PX_ASSERT( false ); return TDataType(); }

		template<typename TDataType>
		inline void convert( const TDataType& ) {}
	};
	
	template<> inline PxU8 BlockParserDataConverter::convert<PxU8>() { return mU8[0]; }
	template<> inline PxU16 BlockParserDataConverter::convert<PxU16>() { return mU16[0]; }
	template<> inline PxU32 BlockParserDataConverter::convert<PxU32>() { return mU32[0]; }
	template<> inline PxU64 BlockParserDataConverter::convert<PxU64>() { return mU64[0]; }
	template<> inline PxI8 BlockParserDataConverter::convert<PxI8>() { return mI8[0]; }
	template<> inline PxI16 BlockParserDataConverter::convert<PxI16>() { return mI16[0]; }
	template<> inline PxI32 BlockParserDataConverter::convert<PxI32>() { return mI32[0]; }
	template<> inline PxI64 BlockParserDataConverter::convert<PxI64>() { return mI64[0]; }
	template<> inline PxF32 BlockParserDataConverter::convert<PxF32>() { return mF32[0]; }
	template<> inline PxF64 BlockParserDataConverter::convert<PxF64>() { return mF64[0]; }
	
	template<> inline void BlockParserDataConverter::convert<PxU8>( const PxU8& inData ) { mU8[0] = inData; }
	template<> inline void BlockParserDataConverter::convert<PxU16>( const PxU16& inData ) { mU16[0] = inData; }
	template<> inline void BlockParserDataConverter::convert<PxU32>( const PxU32& inData ) { mU32[0] = inData; }
	template<> inline void BlockParserDataConverter::convert<PxU64>( const PxU64& inData ) { mU64[0] = inData; }
	template<> inline void BlockParserDataConverter::convert<PxI8>( const PxI8& inData ) { mI8[0] = inData; }
	template<> inline void BlockParserDataConverter::convert<PxI16>( const PxI16& inData ) { mI16[0] = inData; }
	template<> inline void BlockParserDataConverter::convert<PxI32>( const PxI32& inData ) { mI32[0] = inData; }
	template<> inline void BlockParserDataConverter::convert<PxI64>( const PxI64& inData ) { mI64[0] = inData; }
	template<> inline void BlockParserDataConverter::convert<PxF32>( const PxF32& inData ) { mF32[0] = inData; }
	template<> inline void BlockParserDataConverter::convert<PxF64>( const PxF64& inData ) { mF64[0] = inData; }


	struct BlockParseFunctions
	{
		template<PxU8 ByteCount>
		static inline void swapBytes( PxU8* inData )
		{
			for ( PxU32 idx = 0; idx < ByteCount/2; ++idx )
			{
				PxU32 endIdx = ByteCount-idx-1;
				PxU8 theTemp = inData[idx];
				inData[idx] = inData[endIdx];
				inData[endIdx] = theTemp;
			}
		}

		static inline bool checkLength( const PxU8* inStart, const PxU8* inStop, PxU32 inLength )
		{
			return static_cast<PxU32>(inStop - inStart) >= inLength;
		}
		//Ugghh, useless ugly warning work-around
		template<typename T>
		static inline T val(T v) {return v;}

		template<bool DoSwapBytes, typename TDataType>
		static inline bool parse( const PxU8*& inStart, const PxU8* inStop, TDataType& outData )
		{
			if ( checkLength( inStart, inStop, sizeof( TDataType ) ) )
			{
				BlockParserDataConverter theConverter;
				for ( PxU32 idx =0; idx < sizeof( TDataType ); ++idx )
					theConverter.mU8[idx] = inStart[idx];
				if ( val(DoSwapBytes))
					swapBytes<sizeof(TDataType)>( theConverter.mU8 );
				outData = theConverter.convert<TDataType>();
				inStart += sizeof( TDataType );
				return true;
			}
			return false;
		}

		template<bool DoSwapBytes, typename TDataType>
		static inline bool parseBlock( const PxU8*& inStart, const PxU8* inStop, TDataType* outData, PxU32 inNumItems )
		{
			PxU32 desired = sizeof(TDataType)*inNumItems;
			if ( checkLength( inStart, inStop, desired ) )
			{
				if ( val(DoSwapBytes) )
				{
					for ( PxU32 item = 0; item < inNumItems; ++item )
					{
						BlockParserDataConverter theConverter;
						for ( PxU32 idx =0; idx < sizeof( TDataType ); ++idx )
							theConverter.mU8[idx] = inStart[idx];
						swapBytes<sizeof(TDataType)>( theConverter.mU8 );
						outData[item] = theConverter.convert<TDataType>();
						inStart += sizeof(TDataType);
					}
				}
				else
				{
					PxU8* target = reinterpret_cast<PxU8*>(outData);
					memmove( target, inStart, desired );
					inStart += desired;
				}
				return true;
			}
			return false;
		}
		
		//In-place byte swapping block
		template<bool DoSwapBytes, typename TDataType>
		static inline bool parseBlock( PxU8*& inStart, const PxU8* inStop, PxU32 inNumItems )
		{
			PxU32 desired = sizeof(TDataType)*inNumItems;
			if ( checkLength( inStart, inStop, desired ) )
			{
				if ( val(DoSwapBytes) )
				{
					for ( PxU32 item = 0; item < inNumItems; ++item, inStart += sizeof( TDataType ) )
						swapBytes<sizeof(TDataType)>( inStart ); //In-place swap.
				}
				else
					inStart += sizeof( TDataType ) * inNumItems;
				return true;
			}
			return false;
		}
	};

	// nop for bytes
	template <>
	inline void BlockParseFunctions::swapBytes<1>( PxU8* ) {}

	template<bool DoSwapBytes>
	struct ByteSwapper
	{
		template<typename TDataType>
		void swap( TDataType& ioType ) { BlockParseFunctions::swapBytes<sizeof(TDataType)>( reinterpret_cast<PxU8*>(&ioType) ); }
	};

	template<>
	struct ByteSwapper<false> 
	{ 
		template<typename TDataType>
		void swap( TDataType&) {}
	};

	
	template<bool DoSwapBytes>
	struct BlockParser
	{
		const PxU8* mBegin;
		const PxU8* mEnd;
		BlockParser( const PxU8* inBegin=NULL, const PxU8* inEnd=NULL )
			: mBegin( inBegin )
			, mEnd( inEnd )
		{
		}
		inline bool hasMoreData() const { return mBegin != mEnd; }
		inline bool checkLength( PxU32 inLength ) { return BlockParseFunctions::checkLength( mBegin, mEnd, inLength ); }
		
		template<typename TDataType>
		inline bool read( TDataType& outDatatype ) { return BlockParseFunctions::parse<DoSwapBytes>( mBegin, mEnd, outDatatype ); }

		template<typename TDataType>
		inline bool readBlock( TDataType* outDataPtr, PxU32 inNumItems ) { return BlockParseFunctions::parseBlock<DoSwapBytes>( mBegin, mEnd, outDataPtr, inNumItems ); }

		template<typename TDataType>
		inline bool readBlock( PxU32 inNumItems ) 
		{ 
			PxU8* theTempPtr = const_cast<PxU8*>(mBegin);
			bool retval = BlockParseFunctions::parseBlock<DoSwapBytes, TDataType>( theTempPtr, mEnd, inNumItems ); 
			mBegin = theTempPtr;
			return retval;
		}

		PxU32 amountLeft() const { return static_cast<PxU32>( mEnd - mBegin ); }
	};

	template<typename TDataType, typename TBlockParserType>
	inline TDataType blockParserRead( TBlockParserType& inType )
	{
		TDataType retval = TDataType();
		inType.read( retval );
		return retval;
	}
}

#endif

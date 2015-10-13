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
#ifndef PX_PXFILEBUFTOPXSTREAM_H
#define PX_PXFILEBUFTOPXSTREAM_H
#include "PxStream.h"
#include "PsFile.h"
#include "PxFileBuffer.h"



namespace physx { namespace pubfnd3 {

	class PxFileBufPxStream : public PxStream
	{
		PxFileBuffer& mBuffer;

		PxFileBufPxStream( const PxFileBufPxStream& inOther );
		PxFileBufPxStream& operator=( const PxFileBufPxStream& inOther );
	public:
		PxFileBufPxStream( PxFileBuffer& inBuffer ) : mBuffer( inBuffer ) { }

		template<typename TDataType>
		inline TDataType read() const
		{
			TDataType retval;
			readBuffer( &retval, sizeof( TDataType ) );
			return retval;
		}

		virtual		PxU8			readByte()								const { return read<PxU8>(); }
		virtual		PxU16			readWord()								const { return read<PxU16>(); }
		virtual		PxU32			readDword()								const { return read<PxU32>(); }
		virtual		float			readFloat()								const { return read<float>(); }
		virtual		double			readDouble()							const { return read<double>(); }

		virtual		void			readBuffer(void* buffer, PxU32 size)	const
		{
			PxU32 theSize( mBuffer.read( buffer, size ) );
			PX_ASSERT( theSize == size );
		}

		template<typename TDataType>
		inline PxStream& store( TDataType inValue )
		{
			storeBuffer( &inValue, sizeof( inValue ) );
			return *this;
		}

		virtual		PxStream&		storeByte(PxU8 b){ return store(b); }
		virtual		PxStream&		storeWord(PxU16 w) { return store(w); }
		virtual		PxStream&		storeDword(PxU32 d)	{ return store(d); }
		virtual		PxStream&		storeFloat(PxReal f) { return store(f); }
		virtual		PxStream&		storeDouble(PxF64 f) { return store(f); }

		virtual		PxStream&		storeBuffer(const void* buffer, PxU32 size)
		{
			PxU32 theSize( mBuffer.write( buffer, size ) );
			PX_ASSERT( theSize == size );
			return *this;
		}
			
		//Do an operation with the file stream, returning the result
		//of the operation.
		template<typename TReturnValue, typename TOperator>
		static PX_INLINE TReturnValue WithFileStream( const char* inFilename, PxFileBuf::OpenMode inMode, TOperator inOperator )
		{
			PxFileBuffer theFileBuf( inFilename, inMode );
			PxFileBufPxStream theStream( theFileBuf );
			return inOperator( theStream );
		}
	};
} }

#endif 
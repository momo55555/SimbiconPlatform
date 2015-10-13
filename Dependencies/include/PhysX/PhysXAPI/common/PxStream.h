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


#ifndef PX_STREAM_H
#define PX_STREAM_H

/** \addtogroup common
  @{
*/

#include "common/PxPhysXCommon.h"

#ifndef PX_DOXYGEN
namespace physx
{
#endif

#define PX_SERIAL_FILE_ALIGN	128

/**
\brief Callback class for data serialization.

The user needs to supply an PxStream implementation to a number of methods to allow the SDK to read or write
chunks of binary data. This allows flexibility for the source/destination of the data. For example the PxStream
could store data in a file, memory buffer or custom file format.

\note It is the users resposibility to ensure that the data is written to the appropriate offset. PxStream does not 
expose any seeking functionality.



@see PxPhysics.createTriangleMesh() PxPhysics.createConvexMesh() PxTriangleMesh.load()

*/

class PxStream
{
	public:
	/**
	\brief Empty constructor.
	*/

								PxStream()				{}
	/**
	\brief Virtual destructor.
	*/

	virtual						~PxStream()				{}

	// Loading API

	/**
	\brief Called to read a single unsigned byte(8 bits)

	\return Byte read.
	*/
	virtual		PxU8			readByte()								const	= 0;
	
	/**
	\brief Called to read a single unsigned word(16 bits)

	\return Word read.
	*/
	virtual		PxU16			readWord()								const	= 0;
	
	/**
	\brief Called to read a single unsigned dword(32 bits)

	\return DWord read.
	*/
	virtual		PxU32			readDword()								const	= 0;
	
	/**
	\brief Called to read a single precision floating point value(32 bits)

	\return Floating point value read.
	*/
	virtual		PxF32			readFloat()								const	= 0;
	
	/**
	\brief Called to read a double precision floating point value(64 bits)

	\return Floating point value read.
	*/
	virtual		PxF64			readDouble()							const	= 0;
	
	/**
	\brief Called to read a number of bytes.

	\param[out] buffer Buffer to read bytes into, must be at least size bytes in size.
	\param[in] size The size of the buffer in bytes.
	*/
	virtual		void			readBuffer(void* buffer, PxU32 size)	const	= 0;

	// Saving API
	
	/**
	\brief Called to write a single unsigned byte to the stream(8 bits).

	\param b Byte to store.
	\return Reference to the current PxStream object.
	*/
	virtual		PxStream&		storeByte(PxU8 b)								= 0;
	
	/**
	\brief Called to write a single unsigned word to the stream(16 bits).
	
	\param w World to store.
	\return Reference to the current PxStream object.
	*/
	virtual		PxStream&		storeWord(PxU16 w)								= 0;
	
	/**
	\brief Called to write a single unsigned dword to the stream(32 bits).

	\param d DWord to store.
	\return Reference to the current PxStream object.
	*/
	virtual		PxStream&		storeDword(PxU32 d)								= 0;
	
	/**
	\brief Called to write a single precision floating point value to the stream(32 bits).

	\param f floating point value to store.
	\return Reference to the current PxStream object.
	*/
	virtual		PxStream&		storeFloat(PxF32 f)								= 0;
	
	/**
	\brief Called to write a double precision floating point value to the stream(64 bits).

	\param f floating point value to store.
	\return Reference to the current PxStream object.
	*/
	virtual		PxStream&		storeDouble(PxF64 f)							= 0;
	
	/**
	\brief Called to write an array of bytes to the stream.

	\param[in] buffer Array of bytes, size bytes in size.
	\param[in] size Size, in bytes of buffer.
	\return Reference to the current PxStream object.
	*/
	virtual		PxStream&		storeBuffer(const void* buffer, PxU32 size)		= 0;

	
	/**
	\brief Store a signed byte(wrapper for the unsigned version).

	\param b Byte to store.
	\return Reference to the current PxStream object.
	*/
	PX_INLINE	PxStream&		storeByte(PxI8 b)		{ return storeByte(PxU8(b));	}
	
	/**
	\brief Store a signed word(wrapper for the unsigned version).

	\param w Word to store.
	\return Reference to the current PxStream object.
	*/
	PX_INLINE	PxStream&		storeWord(PxI16 w)		{ return storeWord(PxU16(w));	}
	
	/**
	\brief Store a signed dword(wrapper for the unsigned version).

	\param d DWord to store.
	\return Reference to the current PxStream object.
	*/
	PX_INLINE	PxStream&		storeDword(PxI32 d)		{ return storeDword(PxU32(d));	}
};

/**
\brief Callback class for the new serialization framework.
*/
class PxSerialStream
{
public:
							PxSerialStream()			{}
	virtual					~PxSerialStream()			{}

	virtual		void		storeBuffer(const void* buffer, PxU32 size)		= 0;
	virtual		PxU32		getTotalStoredSize()							= 0;
};

#ifndef PX_DOXYGEN
} // namespace physx
#endif

 /** @} */
#endif

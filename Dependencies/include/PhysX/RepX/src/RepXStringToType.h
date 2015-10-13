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
#ifndef REPX_STRINGTOTYPE_H
#define REPX_STRINGTOTYPE_H
#include <stdio.h>
#include <ctype.h>
#include "PsString.h"
#include "PxCoreUtilityTypes.h"
#include "PxFiltering.h"

//Remapping function name for gcc-based systems.
#ifndef _MSC_VER
#define _strtoui64 strtoull
#endif


namespace physx { namespace repx {

	template<typename TDataType>
	struct StrToImpl
	{
		bool compile_error;
	};

	template<> struct StrToImpl<PxU64> { 
		//Id's (void ptrs) are written to file as unsigned
		//64 bit integers, so this method gets called more
		//often than one might think.
		inline void strto( PxU64& ioDatatype, char*& ioData )
		{
			ioDatatype = _strtoui64( ioData, &ioData, 10 );
		}
	};
	

	template<> struct StrToImpl<PxU32> { 
	inline void strto( PxU32& ioDatatype, char*& ioData )
	{
		ioDatatype = static_cast<PxU32>( strtoul( ioData, &ioData, 10 ) );
	}
	};


	template<> struct StrToImpl<PxU16> {
	inline void strto( PxU16& ioDatatype, char*& ioData )
	{
		ioDatatype = static_cast<PxU16>( strtoul( ioData, &ioData, 10 ) );
	}
	};

	inline void eatwhite( char*& ioData )
	{
		if ( ioData )
		{
			while( isspace( *ioData ) )
				++ioData;
		}
	}

	inline void nullTerminateWhite( char*& ioData )
	{
		if ( ioData )
		{
			while( *ioData && !isspace( *ioData ) )
				++ioData;
			if ( *ioData )
			{
				ioData[0] = 0;
				++ioData;
			}
		}
	}
	
	template<> struct StrToImpl<PxF32> {
	inline void strto( PxF32& ioDatatype, char*& ioData )
	{
		//strtod on windows is fatally flawed.  It calls strlen on
		//the buffer.  Thus if you have a large buffer of space-delimited
		//strings you are taking an n! hit trying to parse it.  On top of
		//just parsing it.

		//So, we have to read in the string.  If it null-terminates, we are fine.
		//if we hit a whitespace, we null terminate it.

		//Hence the char data is not const; this is a destructive parse.

		//Eat the whitespace.
		//This only works for space-delimited number strings.  ANything
		//else will fail
		eatwhite( ioData );
		char* target = ioData;
		nullTerminateWhite( ioData );
		ioDatatype = static_cast<PxF32>( strtod( target, NULL ) );
	}
	};

	
	template<> struct StrToImpl<void*> {
	inline void strto( void*& ioDatatype, char*& ioData )
	{
		PxU64 theData;
		StrToImpl<PxU64>().strto( theData, ioData );
		ioDatatype = reinterpret_cast<void*>( static_cast<size_t>( theData ) );
	}
	};
	

	template<> struct StrToImpl<physx::pubfnd3::PxVec3> {
	inline void strto( physx::pubfnd3::PxVec3& ioDatatype, char*& ioData )
	{
		StrToImpl<PxF32>().strto( ioDatatype[0], ioData );
		StrToImpl<PxF32>().strto( ioDatatype[1], ioData );
		StrToImpl<PxF32>().strto( ioDatatype[2], ioData );
	}
	};
	
	template<> struct StrToImpl<PxU8*> {
	inline void strto( PxU8*& ioDatatype, char*& ioData )
	{
	}
	};

	template<> struct StrToImpl<bool> {
	inline void strto( bool& ioType, char*& inValue )
	{
		ioType = physx::string::stricmp( inValue, "true" ) == 0 ? true : false;
	}
	};
	
	template<> struct StrToImpl<PxU8> {
	PX_INLINE void strto( PxU8& ioType, char*& inValue)
	{
		ioType = static_cast<PxU8>( strtoul( inValue, &inValue, 10 ) );
	}
	};

	template<> struct StrToImpl<PxFilterData> {
	PX_INLINE void strto( PxFilterData& ioType, char*& inValue)
	{
		ioType.word0 = static_cast<PxU32>( strtoul( inValue, &inValue, 10 ) );
		ioType.word1 = static_cast<PxU32>( strtoul( inValue, &inValue, 10 ) );
		ioType.word2 = static_cast<PxU32>( strtoul( inValue, &inValue, 10 ) );
		ioType.word3 = static_cast<PxU32>( strtoul( inValue, NULL, 10 ) );
	}
	};
	

	template<> struct StrToImpl<PxQuat> {
	PX_INLINE void strto( PxQuat& ioType, char*& inValue )
	{
		ioType.x = static_cast<PxReal>( strtod( inValue, &inValue ) );
		ioType.y = static_cast<PxReal>( strtod( inValue, &inValue ) );
		ioType.z = static_cast<PxReal>( strtod( inValue, &inValue ) );
		ioType.w = static_cast<PxReal>( strtod( inValue, &inValue ) );
	}
	};
	
	template<> struct StrToImpl<PxTransform> {
	PX_INLINE void strto( PxTransform& ioType, char*& inValue)
	{
		ioType.q.x = static_cast<PxReal>( strtod( inValue, &inValue ) );
		ioType.q.y = static_cast<PxReal>( strtod( inValue, &inValue ) );
		ioType.q.z = static_cast<PxReal>( strtod( inValue, &inValue ) );
		ioType.q.w = static_cast<PxReal>( strtod( inValue, &inValue ) );

		ioType.p[0] = static_cast<PxReal>( strtod( inValue, &inValue ) );
		ioType.p[1] = static_cast<PxReal>( strtod( inValue, &inValue ) );
		ioType.p[2] = static_cast<PxReal>( strtod( inValue, &inValue ) );
	}
	};

	template<> struct StrToImpl<PxBounds3> {
	PX_INLINE void strto( PxBounds3& ioType, char*& inValue)
	{
		ioType.minimum[0] = static_cast<PxReal>( strtod( inValue, &inValue ) );
		ioType.minimum[1] = static_cast<PxReal>( strtod( inValue, &inValue ) );
		ioType.minimum[2] = static_cast<PxReal>( strtod( inValue, &inValue ) );

		ioType.maximum[0] = static_cast<PxReal>( strtod( inValue, &inValue ) );
		ioType.maximum[1] = static_cast<PxReal>( strtod( inValue, &inValue ) );
		ioType.maximum[2] = static_cast<PxReal>( strtod( inValue, &inValue ) );
	}
	};

	template<typename TDataType>
	inline void strto( TDataType& ioType, char*& ioData )
	{
		if ( ioData && *ioData ) StrToImpl<TDataType>().strto( ioType, ioData );
	}

	template<typename TDataType>
	inline void stringToType( const char* inValue, TDataType& ioType )
	{
		char* theValue( const_cast<char*>( inValue ) );
		return strto( ioType, theValue );
	}
}}

#endif
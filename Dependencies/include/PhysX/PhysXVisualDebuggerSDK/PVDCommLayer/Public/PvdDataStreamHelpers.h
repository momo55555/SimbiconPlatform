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

#ifndef PVD_PVD_DATA_STREAM_HELPERS_H
#define PVD_PVD_DATA_STREAM_HELPERS_H
#include "PVDCommLayerTypes.h"

namespace PVD
{
	class PvdPropertyDefinitionHelper
	{
	protected:
		virtual ~PvdPropertyDefinitionHelper(){}
	public:

		/**
			Push a name c such that it appends such as a.b.c.
		*/
		virtual void pushName( const char* inName, const char* inAppendStr = "." ) = 0;
		/**
			Push a name c such that it appends like a.b[c]
		*/
		virtual void pushBracketedName( const char* inName, const char* leftBracket = "[", const char* rightBracket = "]" ) = 0;
		/**
		 *	Pop the current name
		 */
		virtual void popName() = 0;

		/**
		 *	Get the current name at the top of the name stack.  
		 *	Would return "a.b.c" or "a.b[c]" in the above examples.
		 */
		virtual const char* getTopName() = 0;

		/**
		 *	Define a property using the top of the name stack and the passed-in semantic
		 */
		virtual void defineProperty( PxU32 inClass, const char* inSemantic, PvdCommLayerDatatype inDatatype, PxU32 inPropertyKey ) = 0;
		virtual void defineArrayProperty( PxU32 inClass, PxU32 inArrayClass, PxU32 inKey ) = 0;
		/**
		 *	Define a property.  If the name stack has anything pushed, this will push the new name with the default append str.
		 *	It will then pop the name stack.
		 */
		virtual void defineProperty( PxU32 inClass, const char* inName, const char* inSemantic, PvdCommLayerDatatype inDatatype, PxU32 inPropertyKey ) = 0;

		virtual void addNamedValueDefinition( const char* inName, PxU32 inValue )= 0;
		virtual void defineBitflagNames( PxU32 inClass, PxU32 inPropertyKey ) = 0;
		virtual void defineEnumerationNames( PxU32 inClass, PxU32 inPropertyKey ) = 0;

		//The datatype used for instances needs to be pointer unless you actually have Pvd::InstanceId members on your value structs.
		virtual void addStructPropertyEntry( PxU32 inPropertyKey, PvdCommLayerDatatype inDatatype, PxU32 inOffset ) = 0;
		virtual void definePropertyStruct( PxU32 inStructKey, PxU32 inClass, PxU32 inStructSizeInBytes ) = 0;
	};

	class PvdBeginPropertyBlockHelper
	{
	protected:
		virtual ~PvdBeginPropertyBlockHelper(){}
	public:
		virtual void addProperty( PxU32 inPropertyKey, PvdCommLayerDatatype inDatatype ) = 0;
		//begins the property block and clears the block of all data.
		virtual void beginPropertyBlock(PxU32 inClass) = 0;
	};

	class PvdSendPropertyBlockHelper
	{
	protected:
		virtual ~PvdSendPropertyBlockHelper() {};
	public:
		
		virtual void addValue( PxU8 inVal ) = 0;
		virtual void addValue( PxU16 inVal ) = 0;
		virtual void addValue( PxU32 inVal ) = 0;
		virtual void addValue( PxU64 inVal ) = 0;
		virtual void addValue( PxI8 inVal ) = 0;
		virtual void addValue( PxI16 inVal ) = 0;
		virtual void addValue( PxI32 inVal ) = 0;
		virtual void addValue( PxI64 inVal ) = 0;
		virtual void addValue( PxF32 inVal ) = 0;
		virtual void addValue( PxF64 inVal ) = 0;
		virtual void addValue( PxVec3 inVal ) = 0;
		virtual void addBitflagValue( PxU32 inVal ) = 0;
		virtual void addEnumerationValue( PxU32 inVal ) = 0;
		virtual void addValue( bool inValue ) = 0;
		virtual void addValue( const PxQuat& inVal ) = 0;
		virtual void addValue( const PxTransform& inVal ) = 0;
		virtual void addValue( const PxBounds3& inVal ) = 0;
		virtual void addValue( const PVD::FilterData& inVal ) = 0;
		virtual void addValue( const PVD::Plane& inVal ) = 0;
		virtual void addValue( const char* inVal ) = 0;
		virtual void addValue( InstanceId inVal ) = 0;
		virtual void sendPropertyBlock( PxU64 inInstance ) = 0;
	};
}

#endif
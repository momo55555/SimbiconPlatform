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


//#ifdef REMOVED

#include "CmReflection.h"
#include "PxSerialFramework.h"

using namespace physx;

// PX_SERIALIZATION

// FieldType is used as an offset in the following array.
static PxU32 gFieldSize[] = {
	0,						// FIELD_UNDEFINED
	sizeof(char),			// FIELD_BYTE
	sizeof(short),			// PxField::eWORD
	sizeof(int),			// FIELD_DWORD
	sizeof(float),			// FIELD_FLOAT
	3*sizeof(float),		// PxField::eVEC3
	sizeof(char*),			// FIELD_STRING
	sizeof(void*),			// PxField::eSERIAL_PTR
	0,						// PxField::eSERIAL_EMBEDDED
	0,						// FIELD_PX_ARRAY
	0,						// FIELD_LAST
};

// If you hit this assert, you added a field enum without updating the size array above!
ICE_COMPILE_TIME_ASSERT( ICE_ARRAYSIZE(gFieldSize)==PxField::eLAST + 1 );

PxU32 PxFieldDescriptor::FieldSize() const
{
	if(mType<=PxField::eUNDEFINED || mType>=PxField::eLAST)
		return 0xffffffff;

	PxU32 size = gFieldSize[mType];
	if(!size)	size = mSize;

	return size;
}

//~PX_SERIALIZATION


//#endif

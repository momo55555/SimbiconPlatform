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


#include "CmMetaData.h"
#include "PxSerialFramework.h"
#include "CmRefCountable.h"
#include "CmInvasiveSet.h"
#include "PxVec3.h"
#include "PxVec4.h"
#include "PxQuat.h"
#include "PxBounds3.h"
#include "PxTransform.h"
#include "PxMat33.h"
#include "CmBitMap.h"
#include "CmPtrTable.h"

using namespace physx;
using namespace Ps;
using namespace Cm;
using namespace physx::pubfnd3;

#ifndef PS_GENERATE_META_DATA
	#pragma warning(disable: 4100)
#endif
#pragma warning(disable:4101)
#pragma warning(disable:4189)

namespace physx
{

static void getMetaData_PxVec3(PxSerialStream& stream)
{
	DEFINE_MD_CLASS(PxVec3)
	DEFINE_MD_ITEM(PxVec3,		PxReal,	x,	0)
	DEFINE_MD_ITEM(PxVec3,		PxReal,	y,	0)
	DEFINE_MD_ITEM(PxVec3,		PxReal,	z,	0)
}

static void getMetaData_PxVec4(PxSerialStream& stream)
{
	DEFINE_MD_CLASS(PxVec4)
	DEFINE_MD_ITEM(PxVec4,		PxReal,	x,	0)
	DEFINE_MD_ITEM(PxVec4,		PxReal,	y,	0)
	DEFINE_MD_ITEM(PxVec4,		PxReal,	z,	0)
	DEFINE_MD_ITEM(PxVec4,		PxReal,	w,	0)
}

static void getMetaData_PxQuat(PxSerialStream& stream)
{
	DEFINE_MD_CLASS(PxQuat)
	DEFINE_MD_ITEM(PxQuat,		PxReal,	x,	0)
	DEFINE_MD_ITEM(PxQuat,		PxReal,	y,	0)
	DEFINE_MD_ITEM(PxQuat,		PxReal,	z,	0)
	DEFINE_MD_ITEM(PxQuat,		PxReal,	w,	0)
}

static void getMetaData_PxBounds3(PxSerialStream& stream)
{
	DEFINE_MD_CLASS(PxBounds3)
	DEFINE_MD_ITEM(PxBounds3,	PxVec3,	minimum,	0)
	DEFINE_MD_ITEM(PxBounds3,	PxVec3,	maximum,	0)
}

static void getMetaData_PxTransform(PxSerialStream& stream)
{
	DEFINE_MD_CLASS(PxTransform)
	DEFINE_MD_ITEM(PxTransform,	PxQuat,	q,	0)
	DEFINE_MD_ITEM(PxTransform,	PxVec3,	p,	0)
}

static void getMetaData_PxMat33(PxSerialStream& stream)
{
	DEFINE_MD_CLASS(PxMat33)
	DEFINE_MD_ITEM(PxMat33,	PxVec3,	column0,	0)
	DEFINE_MD_ITEM(PxMat33,	PxVec3,	column1,	0)
	DEFINE_MD_ITEM(PxMat33,	PxVec3,	column2,	0)
}

static void getMetaData_BitMap(PxSerialStream& stream)
{
	DEFINE_MD_TYPEDEF(Allocator, PxU8)	// ### Weeeeell....

	class ShadowBitMap : public BitMap
	{
	public:
		static void getMetaData(PxSerialStream& stream)
		{
			DEFINE_MD_CLASS(ShadowBitMap)
			DEFINE_MD_JUNK(ShadowBitMap, 4)		// ### ghost bytes added because the 2 base classes are empty!
			DEFINE_MD_ITEM(ShadowBitMap, PxU32,		mMap,		MdFlags::ePTR)
			DEFINE_MD_ITEM(ShadowBitMap, PxU32,		mWordCount,	0)
			DEFINE_MD_ITEM(ShadowBitMap, Allocator,	mAllocator,	0)
#ifdef DEFINE_MD_ITEMS2
			DEFINE_MD_ITEMS2(ShadowBitMap, PxU8,	mPadding,	MdFlags::ePADDING)
#else
			DEFINE_MD_ITEMS(ShadowBitMap, PxU8,		mPadding,	MdFlags::ePADDING, 3)
#endif

/*		void	exportExtraData(PxSerialStream& stream)
		{
			if(mMap && getWordCount())
				stream.storeBuffer(mMap, getWordCount()*sizeof(PxU32));
		}*/
			DEFINE_MD_EXTRA_DATA_ARRAY(ShadowBitMap, PxU32, mWordCount, 0, MdFlags::eMASK_MSB_COUNT)
		}
	};
	ShadowBitMap::getMetaData(stream);
	DEFINE_MD_TYPEDEF(BitMap, ShadowBitMap)
}

static void getMetaData_PtrTable(PxSerialStream& stream)
{
	DEFINE_MD_CLASS(PtrTable)

	DEFINE_MD_ITEM(PtrTable, void,	mSingle,		MdFlags::ePTR)		// PT: this is actually a union, beware
	DEFINE_MD_ITEM(PtrTable, PxU16,	mCount,			0)
	DEFINE_MD_ITEM(PtrTable, bool,	mOwnsMemory,	0)
	DEFINE_MD_ITEM(PtrTable, bool,	mBufferUsed,	0)

	//------ Extra-data ------

	DEFINE_MD_EXTRA_DATA_ITEMS(PtrTable, void, mBufferUsed, mCount, MdFlags::eFLIP_CONTROL|MdFlags::ePTR)
}

void PxSerializable::getMetaData(PxSerialStream& stream)
{
	DEFINE_MD_TYPEDEF(PxSerialFlags, PxU16)
	DEFINE_MD_TYPEDEF(PxType, PxU16)
	DEFINE_MD_VCLASS(PxSerializable)
	DEFINE_MD_ITEM(PxSerializable, PxType,			mType,			0)
	DEFINE_MD_ITEM(PxSerializable, PxSerialFlags,	mSerialFlags,	0)
//	DEFINE_MD_ITEM(PxSerializable, PxSerializable,	mMyAddress,		MdFlags::ePTR)
}

void RefCountable::getMetaData(PxSerialStream& stream)
{
	DEFINE_MD_VCLASS(RefCountable)
	DEFINE_MD_ITEM(RefCountable, PxI32,			mRefCount,		0)
}

void InvasiveSetHook::getMetaData(PxSerialStream& stream)
{
	DEFINE_MD_CLASS(InvasiveSetHook)
	DEFINE_MD_ITEM(InvasiveSetHook, PxU32,		index,		0)
}

void Cm::getFoundationMetaData(PxSerialStream& stream)
{
	DEFINE_MD_TYPEDEF(PxU8, char)
	DEFINE_MD_TYPEDEF(PxI8, char)
	DEFINE_MD_TYPEDEF(PxU16, short)
	DEFINE_MD_TYPEDEF(PxI16, short)
	DEFINE_MD_TYPEDEF(PxU32, int)
	DEFINE_MD_TYPEDEF(PxI32, int)
	DEFINE_MD_TYPEDEF(PxReal, float)

	getMetaData_PxVec3(stream);
	getMetaData_PxVec4(stream);
	getMetaData_PxQuat(stream);
	getMetaData_PxBounds3(stream);
	getMetaData_PxTransform(stream);
	getMetaData_PxMat33(stream);
	getMetaData_BitMap(stream);
	getMetaData_PtrTable(stream);

	PxSerializable::getMetaData(stream);
	RefCountable::getMetaData(stream);
	InvasiveSetHook::getMetaData(stream);
}

}

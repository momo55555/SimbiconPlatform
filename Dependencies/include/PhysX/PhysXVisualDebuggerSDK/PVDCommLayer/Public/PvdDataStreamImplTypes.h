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

#ifndef PVD_PVDCONNECTIONIMPLTYPES_H
#define PVD_PVDCONNECTIONIMPLTYPES_H

#include "PVDCommLayerTypes.h"

namespace PVD
{
	inline PxU32 InvalidIndex() { return (PxU32)-1; }
	inline PxU32 InvalidKey() { return (PxU32)0; }
	struct PropertyDescription
	{
		const char*				mName;
		const char*				mSemantic;
		PxU32					mIndex;
		PxU32					mKey;
		PxU32					mArrayClass;
		PvdCommLayerDatatype	mDatatype;
		PX_INLINE PropertyDescription()
			: mName( "" )
			, mSemantic( "" )
			, mIndex( 0 )
			, mKey( 0 )
			, mArrayClass( (PxU32)-1 )
			, mDatatype( PvdCommLayerDatatype::Unknown )
		{
		}
		PX_INLINE PropertyDescription( const char* inName
										, const char* inSemantic
										, PxU32 inIndex
										, PxU32 inKey
										, PvdCommLayerDatatype inDatatype )
			: mName( inName )
			, mSemantic( inSemantic )
			, mIndex( inIndex )
			, mKey( inKey )
			, mArrayClass( (PxU32)-1 )
			, mDatatype( inDatatype ) {}
		
		PX_INLINE PropertyDescription( const char* inName
										, PxU32 inIndex
										, PxU32 inArrayClass
										, PxU32 inKey )
			: mName( inName )
			, mSemantic( "" )
			, mIndex( inIndex )
			, mKey( inKey )
			, mArrayClass( inArrayClass )
			, mDatatype( PvdCommLayerDatatype::Unknown ) {}

		inline bool IsArrayProperty() const { return mArrayClass != (PxU32)-1; }
	};

	struct ClassDescription
	{
		const char*				mName;
		PxU32					mNamespace;
		PxU32					mKey;
		PxU32					mParentKey; //(PxU32)0 if the instance doesn't exist
		bool					mClassLocked; //We it is illegal to add properties to this class.

		PX_INLINE ClassDescription( PxU32 inNamespace, const char* inName, PxU32 inKey )
			: mName( inName )
			, mNamespace( inNamespace )
			, mKey( inKey )
			, mParentKey( InvalidKey() )
			, mClassLocked( false )
		{
		}
		PX_INLINE ClassDescription(){}
	};

	struct InstanceDescription
	{
		PxU32	mNamespace;
		PxU32	mClassKey;
		PxU64	mKey;
		bool	mIsArray;
		PX_INLINE InstanceDescription( PxU32 inNamespace, PxU32 inClsKey, PxU64 inKey, bool inIsArray )
			: mNamespace( inNamespace )
			, mClassKey( inClsKey )
			, mKey( inKey )
			, mIsArray( inIsArray )
		{
		}
		PX_INLINE InstanceDescription() {}
	};

}

#endif
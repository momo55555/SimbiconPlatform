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

#ifndef PVD_PVDCONNECTIONNULLTYPECHECKER_H
#define PVD_PVDCONNECTIONNULLTYPECHECKER_H
#include "PVDCommLayerErrors.h"

namespace PVD
{

	template< typename TMutexType
			, typename TScopedLockType
			, typename TDeleteOperatorType >
	class PvdConnectionNullTypeChecker
	{
		TMutexType						mMutex;
		PxU32							mRefCount;
	public:
		PvdConnectionNullTypeChecker() : mRefCount( 0 ) {}
		inline PvdCommLayerError setNamespace( const char* ) { return PvdCommLayerError::None; }
		inline PvdCommLayerError createClass( PxU32, const char* , PxU32 ) { return PvdCommLayerError::None; }
		inline PvdCommLayerError deriveClass( PxU32, PxU32 , PxU32 ){ return PvdCommLayerError::None; }
		inline PvdCommLayerError defineProperty( PxU32, PxU32 
													, const char* 
													, const char* 
													, PvdCommLayerDatatype 
													, PxU32 ) { return PvdCommLayerError::None; }
		inline PvdCommLayerError definePropertyOnInstance( PxU64 
															, const char* 
															, const char* 
															, PvdCommLayerDatatype 
															, PxU32 ) { return PvdCommLayerError::None; }
		inline PvdCommLayerError definePropertyStruct( PxU32
														, PxU32
															, PxU32 
															, PxU32 
															, const PropertyStructEntry* 
															, PxU32 ) { return PvdCommLayerError::None; }
		inline PvdCommLayerError defineArrayProperty( PxU32, PxU32 
														, const char* 
														, PxU32 
														, PxU32 ) { return PvdCommLayerError::None; }
		inline PvdCommLayerError defineBitflagNames( PxU32, PxU32 
														, PxU32 
														, const NamedValueDefinition* 
														, PxU32 ) { return PvdCommLayerError::None; }
		inline PvdCommLayerError defineEnumerationNames( PxU32, PxU32 
															, PxU32 
															, const NamedValueDefinition* 
															, PxU32 ) { return PvdCommLayerError::None; }
		inline PvdCommLayerError createInstance( PxU32, PxU32 , PxU64 ) { return PvdCommLayerError::None; }
		inline PvdCommLayerError setPropertyValue( PxU64 , PxU32 , PvdCommLayerDatatype ) { return PvdCommLayerError::None; }
		inline PvdCommLayerError beginPropertyBlock( PxU32, PxU32 , const PxU32* , const PvdCommLayerDatatype* , PxU32 ) { return PvdCommLayerError::None; }
		inline PvdCommLayerError sendPropertyBlock( PxU64 , const PvdCommLayerValue*  ) { return PvdCommLayerError::None; }
		inline PvdCommLayerError endPropertyBlock() { return PvdCommLayerError::None; }
		inline PvdCommLayerError sendPropertyStruct( PxU64, PxU32, PxU32, const PxU8*, PxU32 ) { return PvdCommLayerError::None; }
		inline PvdCommLayerError addChild( PxU64 , PxU64  ){ return PvdCommLayerError::None; }
		inline PvdCommLayerError removeChild( PxU64 , PxU64 ) { return PvdCommLayerError::None; }
		inline PvdCommLayerError removeAllChildren( PxU64, PxU32 ) { return PvdCommLayerError::None; }
		inline PvdCommLayerError destroyInstance( PxU64 ) { return PvdCommLayerError::None; }
		inline PvdCommLayerError beginArrayBlock( PxU32, PxU32 , PxU64 , const PxU32* , const PvdCommLayerDatatype* , PxU32 ) { return PvdCommLayerError::None; }
		inline PvdCommLayerError beginArrayPropertyBlock( PxU64, PxU32, const PxU32*, const PvdCommLayerDatatype*, PxU32) { return PvdCommLayerError::None; }
		inline PvdCommLayerError sendArrayObject( const PvdCommLayerValue* ) { return PvdCommLayerError::None; }
		inline PvdCommLayerError sendArrayObjects( const PxU8* , PxU32 , PxU32 ) { return PvdCommLayerError::None; }
		inline PvdCommLayerError endArrayBlock() { return PvdCommLayerError::None; }
		inline PvdCommLayerError endArrayPropertyBlock() { return PvdCommLayerError::None; }
		inline PvdCommLayerError beginSection( const char* ) { return PvdCommLayerError::None; }
		inline PvdCommLayerError endSection( const char* ) { return PvdCommLayerError::None; }
		inline PvdCommLayerError namedEventWithInstance(PxU64, const char* ) { return PvdCommLayerError::None; }
		template<typename TRenderCommandType>
		inline PvdCommLayerError handleRenderCommand( const TRenderCommandType& ) { return PvdCommLayerError::None; }
		inline void addRef() 
		{ 
			TScopedLockType theLocker( mMutex );
			++mRefCount; 
		}
		inline void release()
		{
			PxU32 theRefCount;
			{
				TScopedLockType theLocker( mMutex );
				if ( mRefCount )
					--mRefCount;
				theRefCount = mRefCount;
			}
			if ( !theRefCount )
				TDeleteOperatorType()( this );
		}
	};

}

#endif
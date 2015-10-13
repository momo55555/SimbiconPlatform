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
#ifndef PX_META_DATA_PVD_BINDING_DATA_H
#define PX_META_DATA_PVD_BINDING_DATA_H
#include "PxSimpleTypes.h"
#include "PsArray.h"
#include "PVDCommLayerDatatypes.h"
#include "../src/PxProfileMemoryBuffer.h"
#include "PsHashSet.h"

namespace physx
{
namespace Pvd
{
	using namespace PVD;
	using namespace physx::pubfnd;
	using namespace physx::shdfnd;

	struct PvdMetaDataBindingData : public UserAllocated
	{
		Array<PxU8>							mTempU8Array;
		physx::profile::MemoryBuffer<>		mMemoryBuffer;
		Array<PxU32>						mBodyPropKeys;
		Array<PVD::PvdCommLayerDatatype>	mBodyPropTypes;
		Array<PVD::PvdCommLayerValue>		mValues;
		Array<PxU32>						mNameStack;
		Array<char>							mNameBuffer;
		Array<PVD::NamedValueDefinition>	mTempDefinitions;
		Array<PxActor*>						mActors;
		Array<PxArticulation*>				mArticulations;
		Array<PxArticulationLink*>			mArticulationLinks;
		HashSet<PxActor*>					mSleepingActors;
	};

	struct PvdMetaDataStreamData
	{
		PvdDataStream*				mDataStream;
		PvdMetaDataBindingData&		mBindingData;

		PvdMetaDataStreamData( PVD::PvdDataStream* inStream, PvdMetaDataBindingData& inData )
			: mDataStream( inStream )
			, mBindingData( inData )
		{
			mBindingData.mNameStack.clear();
			mBindingData.mNameBuffer.clear();
			mBindingData.mTempDefinitions.clear();
		}
		

		void appendStrToBuffer( const char* str )
		{
			if ( str == NULL )
				return;
			size_t strLen = strlen( str );
			size_t endBufOffset = mBindingData.mNameBuffer.size();
			size_t resizeLen = endBufOffset;
			//account for null
			if ( mBindingData.mNameBuffer.empty() )
				resizeLen += 1;
			else
				endBufOffset -=1;

			mBindingData.mNameBuffer.resize( static_cast<PxU32>( resizeLen + strLen ) );
			char* endPtr = mBindingData.mNameBuffer.begin() + endBufOffset;
			memcpy( endPtr, str, strLen );
		}

		void pushName( const char* nm, const char* appender = "." ) 
		{ 
			size_t nameBufLen = mBindingData.mNameBuffer.size();
			mBindingData.mNameStack.pushBack( static_cast<PxU32>( nameBufLen ) );
			if ( mBindingData.mNameBuffer.empty() == false )
				appendStrToBuffer( appender );
			appendStrToBuffer( nm );
			mBindingData.mNameBuffer.back() = 0;
		}

		void popName()
		{
			if ( mBindingData.mNameStack.empty() )
				return;
			mBindingData.mNameBuffer.resize( static_cast<PxU32>( mBindingData.mNameStack.back() ) );
			mBindingData.mNameStack.popBack();
			if ( mBindingData.mNameBuffer.empty() == false )
				mBindingData.mNameBuffer.back() = 0;
		}

		const char* getTopName()
		{
			if ( mBindingData.mNameBuffer.size() )
				return mBindingData.mNameBuffer.begin();
			return "";
		}
	};
}

}

#endif

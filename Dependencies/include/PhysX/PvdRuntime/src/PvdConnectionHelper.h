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


#ifndef PVD_CONNECTIONHELPER_H
#define PVD_CONNECTIONHELPER_H

#if PX_SUPPORT_VISUAL_DEBUGGER

#include "PsUserAllocated.h"
#include "PxStrideIterator.h"
#include "PsIntrinsics.h"
#include "CmPhysXCommon.h"


namespace PVD
{
	class PvdDataStream;
	 class PvdCommLayerValue;
	 struct PvdCommLayerError;
	 struct PvdCommLayerDatatype;
}

namespace physx
{
namespace Pvd
{
	
class PvdConnectionHelper : public Ps::UserAllocated
{
public:
	PvdConnectionHelper();

	void							addPropertyGroupProperty(PxU32 inPropertyHandleMinusOne, const PVD::PvdCommLayerValue& inValue);	
	PVD::PvdCommLayerError			sendSinglePropertyGroup(PVD::PvdDataStream* inConnection, PxU64 inInstance, PxU32 inClassMinusOne);
	PVD::PvdCommLayerError			sendPrimitiveIndexArrayProperty(PVD::PvdDataStream* inConnection, 
																	PxU64 inInstance, 
																	PxU32 inProperty, 
																	const PxU8* inData, 
																	PxU32 inPrimitiveStride, 
																	PxU32 inNumPrimitives, 
																	PxU8 inNumObjectsPerPrimitive, 
																	bool inIsU16);

	static PVD::PvdCommLayerError	beginSingleElementArrayProperty(PVD::PvdDataStream* inConnection, 
																	PxU64 inInstance, 
																	PxU32 inProperty, 
																	PxU32 inArrayProperty, 
																	PVD::PvdCommLayerDatatype inArrayDatatype);

	static PVD::PvdCommLayerError	sendSingleElementArrayProperty(PVD::PvdDataStream* inConnection, 
																   PxU64 inInstance, PxU32 inProperty, 
																   PxU32 inArrayProperty, 
																   PVD::PvdCommLayerDatatype inArrayDatatype, 
																   const PxU8* inData, PxU32 inStride, 
																   PxU32 inObjectCount);

	template<class T, class C>
	void						sendParticleArray( PVD::PvdDataStream* inConnection, 
													PxStrideIterator<const T>& iterator, 
													PxU64 instanceId,
													PxU32 instanceProperty,
													PxU32 numValidParticles, 
													PxU32 lastValidParticleIndex, 
													const PxU32* validParticleBitmap ); 

	PVD::PvdCommLayerError sendArrayObjects( PVD::PvdDataStream* inConnection, 
											const PxU8* inData,
											PxU32 inStride,
											PxU32 inObjectCount );

	PVD::PvdCommLayerError endArrayPropertyBlock( PVD::PvdDataStream* inConnection );
private:
	Ps::Array<PxU8>							mTempU8Array;

	Ps::Array<PxU32>						mTempPropertyHandles;
	Ps::Array<PVD::PvdCommLayerDatatype>	mTempPvdDatatypes;
	Ps::Array<PVD::PvdCommLayerValue>		mTempPvdValues;
};
	

template<class T, class C>
void PvdConnectionHelper::sendParticleArray(PVD::PvdDataStream* inConnection, 
											PxStrideIterator<const T>& iterator, 
											PxU64 instanceId,
											PxU32 instanceProperty,
											PxU32 numValidParticles, 
											PxU32 validParticleRange, 
											const PxU32* validParticleBitmap) 
{

	if(numValidParticles == 0)
		return;

	// setup the pvd array
	PxU32 properties[1] = { 1 };
	PVD::PvdCommLayerDatatype propTypes[1];
	PvdConnectionHelper::beginSingleElementArrayProperty( inConnection, instanceId, instanceProperty, 0, C::getPvdDataType() );
	
	
	if(numValidParticles == validParticleRange)
	{
		PvdConnectionHelper::sendArrayObjects(inConnection, reinterpret_cast<const PxU8*>(iterator.ptr()), iterator.stride(), numValidParticles);
	}
	else
	{
		mTempU8Array.clear();
		mTempU8Array.resize(numValidParticles * sizeof(T));
		T* tmpArray  = reinterpret_cast<T*>(mTempU8Array.begin());
		PxU32 tIdx = 0;

		// iterate over bitmap and send all valid particles
		for (PxU32 w = 0; w <= (validParticleRange-1) >> 5; w++)
		{
			for (PxU32 b = validParticleBitmap[w]; b; b &= b-1)
			{
				tmpArray[tIdx++] = iterator[w<<5|Ps::lowestSetBit(b)];
			}
		}
		PX_ASSERT(tIdx == numValidParticles);
		PvdConnectionHelper::sendArrayObjects( inConnection, mTempU8Array.begin(), 0, numValidParticles);
	}
	PvdConnectionHelper::endArrayPropertyBlock(inConnection);
	
	mTempU8Array.clear();
}

} // namespace Pvd

}

#endif // PX_SUPPORT_VISUAL_DEBUGGER

#endif // PVD_CONNECTIONHELPER_H


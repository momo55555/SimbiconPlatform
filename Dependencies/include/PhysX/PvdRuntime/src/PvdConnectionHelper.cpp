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


#if PX_SUPPORT_VISUAL_DEBUGGER

#include "PxPhysXCommon.h"
#include "PvdConnectionHelper.h"
#include "PvdDataStream.h"
#include "PvdClassDefinitions.h"
#include "PsIntrinsics.h"

namespace physx
{
namespace Pvd
{


PvdConnectionHelper::PvdConnectionHelper()
{
}

void PvdConnectionHelper::addPropertyGroupProperty(PxU32 inPropertyHandleMinusOne, const PVD::PvdCommLayerValue& inValue)	
{
	mTempPropertyHandles.pushBack(inPropertyHandleMinusOne+1);
	mTempPvdDatatypes.pushBack(inValue.getDatatype());
	mTempPvdValues.pushBack(inValue);
}


PVD::PvdCommLayerError PvdConnectionHelper::sendSinglePropertyGroup(PVD::PvdDataStream* inConnection, PxU64 inInstance, PxU32 inClassMinusOne)
{
	PxU32 theNumPropertiesAndDatatypes = mTempPvdDatatypes.size();
	if ( theNumPropertiesAndDatatypes )
	{
		inConnection->beginPropertyBlock(inClassMinusOne + 1, mTempPropertyHandles.begin(), mTempPvdDatatypes.begin(), theNumPropertiesAndDatatypes);
		inConnection->sendPropertyBlock(inInstance, mTempPvdValues.begin());

		mTempPropertyHandles.clear();
		mTempPvdDatatypes.clear();
		mTempPvdValues.clear();

		return inConnection->endPropertyBlock();
	}
	return PVD::PvdCommLayerError::None;
}

PVD::PvdCommLayerError PvdConnectionHelper::sendPrimitiveIndexArrayProperty(PVD::PvdDataStream* inConnection, PxU64 inInstance, PxU32 inProperty, 
											const PxU8* inData, PxU32 inPrimitiveStride, PxU32 inNumPrimitives, PxU8 inNumObjectsPerPrimitive, bool inIsU16)
{
	PxU32 theArrayProperty = inIsU16 ? (PxU32)U16ArrayProp::Element : (PxU32)U32ArrayProp::Element;
	PVD::PvdCommLayerDatatype theDatatype = inIsU16 ? PVD::PvdCommLayerDatatype::U16 : PVD::PvdCommLayerDatatype::U32;
	PxU32 primitiveSize = inIsU16 ? sizeof(PxU16)*inNumObjectsPerPrimitive : sizeof(PxU32)*inNumObjectsPerPrimitive;
	PxU32 numObjects = inNumObjectsPerPrimitive * inNumPrimitives;

	const PxU8* wireData = inData;
	if(primitiveSize != inPrimitiveStride)
	{
		mTempU8Array.clear();
		mTempU8Array.resize(primitiveSize*inNumPrimitives);
		wireData = mTempU8Array.begin();
		PxU8* dataPtr = mTempU8Array.begin();
		for(PxU32 i = 0; i < inNumPrimitives; i++, inData+=inPrimitiveStride, dataPtr += primitiveSize)
		{
			Ps::memCopy(dataPtr, inData, primitiveSize);
		}
	}
	PVD::PvdCommLayerError retval = sendSingleElementArrayProperty(inConnection, inInstance, inProperty, theArrayProperty, theDatatype, wireData, 0, numObjects);
	mTempU8Array.clear();
	return retval;
}


PVD::PvdCommLayerError PvdConnectionHelper::beginSingleElementArrayProperty(PVD::PvdDataStream* inConnection, 
																		PxU64 inInstance, 
																		PxU32 inProperty, 
																		PxU32 inArrayProperty, 
																		PVD::PvdCommLayerDatatype inArrayDatatype)
{
	//Take care of PhysX30->Pvd mapping
	inProperty += 1;
	inArrayProperty += 1;
	return inConnection->beginArrayPropertyBlock(inInstance, inProperty, &inArrayProperty, &inArrayDatatype, 1);
}


PVD::PvdCommLayerError PvdConnectionHelper::sendSingleElementArrayProperty(PVD::PvdDataStream* inConnection, 
																	   PxU64 inInstance, PxU32 inProperty, 
																	   PxU32 inArrayProperty, 
																	   PVD::PvdCommLayerDatatype inArrayDatatype, 
																	   const PxU8* inData, PxU32 inStride, 
																	   PxU32 inObjectCount)
{
	beginSingleElementArrayProperty(inConnection, inInstance, inProperty, inArrayProperty, inArrayDatatype);
	inConnection->sendArrayObjects(inData, inStride, inObjectCount);
	return inConnection->endArrayPropertyBlock();
}

PVD::PvdCommLayerError PvdConnectionHelper::sendArrayObjects( PVD::PvdDataStream* inConnection, 
																const PxU8* inData,
																PxU32 inStride,
																PxU32 inObjectCount )
{
	return inConnection->sendArrayObjects( reinterpret_cast< const PxU8* >( inData ), inStride, inObjectCount );
}


PVD::PvdCommLayerError PvdConnectionHelper::endArrayPropertyBlock( PVD::PvdDataStream* inConnection )
{
	return inConnection->endArrayPropertyBlock();
}

} // namespace Pvd

}

#endif // PX_SUPPORT_VISUAL_DEBUGGER

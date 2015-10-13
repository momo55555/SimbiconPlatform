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


#include "PxDeformableExt.h"
#include "PxStrideIterator.h"

using namespace physx;

//----------------------------------------------------------------------------//

template<typename T, int N>
PX_INLINE void getPositions(PxVec3* positions, const PxStrideIterator<const PxVec3>& positionIt, const T* indexTuple)
{
	for (PxU32 i = 0; i < N; i++)
		positions[i] = positionIt[indexTuple[i]];
}

//----------------------------------------------------------------------------//

template<typename T>
PxReal computeTriangleMeshArea(const PxDeformableMeshDesc& meshDesc)
{
	PxStrideIterator<const PxVec3> positionIt(static_cast<const PxVec3*>(meshDesc.vertices), meshDesc.vertexStrideBytes);	
	PxStrideIterator<const T> indexIt(static_cast<const T*>(meshDesc.primitives), meshDesc.primitiveStrideBytes);

	PxReal areaSum = 0.0f;
	for (PxU32 i = 0; i != meshDesc.numPrimitives; ++i, ++indexIt)
	{
		PxVec3 positions[3];
		getPositions<T, 3>(positions, positionIt, indexIt.ptr());
		PxVec3 e0 = positions[1] - positions[0];
		PxVec3 e1 = positions[2] - positions[0];
		PxReal area = e0.cross(e1).magnitude() / 2.0f;
		areaSum += area;
	}
	return areaSum;
}

//----------------------------------------------------------------------------//

template<typename T>
PxReal computeTetrahedronMeshVolume(const PxDeformableMeshDesc& meshDesc)
{
	PxStrideIterator<const PxVec3> positionIt(static_cast<const PxVec3*>(meshDesc.vertices), meshDesc.vertexStrideBytes);	
	PxStrideIterator<const T> indexIt(static_cast<const T*>(meshDesc.primitives), meshDesc.primitiveStrideBytes);

	PxReal volumeSum = 0.0f;
	for (PxU32 i = 0; i != meshDesc.numPrimitives; ++i, ++indexIt)
	{
		PxVec3 positions[4];
		getPositions<T, 4>(positions, positionIt, indexIt.ptr());
		PxVec3 e0 = positions[1] - positions[0];
		PxVec3 e1 = positions[2] - positions[0];
		PxVec3 e2 = positions[3] - positions[0];
		PxReal volume = e0.cross(e1).dot(e2) / 6.0f;
		volumeSum += volume;
	}
	return volumeSum;
}

//----------------------------------------------------------------------------//

PxReal PxDeformableExt::computeTriangleMeshMass(const PxDeformableMeshDesc& meshDesc, PxReal thickness, PxReal density)
{
	PX_ASSERT(meshDesc.primitiveType == PxDeformablePrimitiveType::eTRIANGLE);

	PxReal area;
	if ((meshDesc.flags & PxDeformableMeshFlag::e16_BIT_INDICES) != 0)
		area = computeTriangleMeshArea<PxU16>(meshDesc);
	else
		area = computeTriangleMeshArea<PxU32>(meshDesc);

	return area * thickness * density;
}

//----------------------------------------------------------------------------//

PxReal PxDeformableExt::computeTetrahedronMeshMass(const PxDeformableMeshDesc& meshDesc, PxReal density)
{
	PX_ASSERT(meshDesc.primitiveType == PxDeformablePrimitiveType::eTETRAHEDRON);

	PxReal volume;
	if ((meshDesc.flags & PxDeformableMeshFlag::e16_BIT_INDICES) != 0)
		volume = computeTetrahedronMeshVolume<PxU16>(meshDesc);
	else
		volume = computeTetrahedronMeshVolume<PxU32>(meshDesc);

	return volume * density;
}

//----------------------------------------------------------------------------//

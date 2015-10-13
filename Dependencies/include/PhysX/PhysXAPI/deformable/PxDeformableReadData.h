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


#ifndef PX_PHYSICS_NX_DEFORMABLEREADDATA
#define PX_PHYSICS_NX_DEFORMABLEREADDATA
/** \addtogroup deformable
  @{
*/

#include "PxLockedData.h"
#include "PxStrideIterator.h"
#include "PxVec3.h"

#ifndef PX_DOXYGEN
namespace physx
{
#endif

/**
\brief User-side struct specifying a primitive split generated from a torn deformable.

The user can optionally receive primitive split event data from torn deformables
through the bulk data buffer PxDeformableReadData.primitiveSplitPairBuffer.
One split is represented by a pair of primitive indices and a pair of split-site indices.
The primitive indices can be used to locate the vertex index tuples of the two mesh 
primitives involved in the split.

If the primitive is a triangle, primitiveIndex[0/1] refer to the vertex index triples of the two involved 
triangles while splitSite[0/1] refer to the triangle edges involved in the split. 
The edge enumeration convention is the same as the one returned by PxDeformablePrimitiveOrder.getTriangleEdge.

If the primitive is a tetrahedron, primitiveIndex[0/1] refer to the vertex index quadruples of the two 
involved tetrahedra while splitSite[0/1] refer to the tetrahedron faces involved in the split. 
The face enumeration convention is the same as the one returned by PxDeformablePrimitiveOrder.getTetrahedronFace.

Each pair is garantueed to be reported only once.

@see PxDeformableDesc.deformableReadDataFlags PxDeformableReadData.primitiveSplitPairBuffer PxDeformablePrimitiveOrder
*/
PX_DEPRECATED struct PxDeformablePrimitiveSplitPair
{
	PxU32 primitiveIndex[2];
	PxU8 splitSite[2]; 
};


PX_DEPRECATED class PxDeformablePrimitiveOrder
{
public:
	/**
	\brief Function specifying the triangle edge enumeration order used by the SDK.

	@see PxDeformablePrimitiveSplitPair
	*/
	PX_INLINE static PxU32 getTriangleEdge(PxU32 edgeIndex, PxU32 edgeVertexIndex)
	{
		const static PxU32 edgeIndices[3][2] = {{0,1}, {1,2}, {2,0}};
		return edgeIndices[edgeIndex][edgeVertexIndex];	
	}

	/**
	\brief Function specifying the tetrahedron face enumeration order used by the SDK.

	@see PxDeformablePrimitiveSplitPair
	*/
	PX_INLINE static PxU32 getTetrahedronFace(PxU32 faceIndex, PxU32 faceVertexIndex)
	{
		const static PxU32 faceIndices[4][3] = {{2,1,0}, {0,1,3}, {1,2,3}, {2,0,3}};
		return faceIndices[faceIndex][faceVertexIndex];	
	}
};


/**
Flags to configure PxDeformable simulation output that can be read by the application. 
Disabling unneeded buffers saves memory and improves performance.

@see PxDeformableDesc::deformableReadDataFlags
*/
PX_DEPRECATED struct PxDeformableReadDataFlag
{
	enum Enum
	{
		ePOSITION_BUFFER				= (1<<0),
		eVELOCITY_BUFFER				= (1<<1),
		eINVERSE_MASS_BUFFER			= (1<<2),
		eNORMAL_BUFFER					= (1<<3),
		ePARENT_INDEX_BUFFER			= (1<<4),
		eINDEX_BUFFER					= (1<<5),
		ePRIMITIVE_SPLIT_PAIR_BUFFER	= (1<<6)
	};
};


/**
\brief Data layout descriptor for reading deformable bulk data from the SDK.

PxDeformableReadData is used to retrieve information about the simulated deformable vertices. 
It can be accessed by calling PxDeformable::lockDeformableReadData().

@see PxDeformable::lockDeformableReadData()
*/
PX_DEPRECATED class PxDeformableReadData : public PxLockedData
{
public:
	
	/**
	\brief Number of vertices. 
	*/
	PxU32 numVertices;

	/**
	\brief Dirty buffer flags.
	*/
	PxU32 dirtyBufferFlags;

	/**
	\brief Number of primitive split pairs.
	*/
	PxU32 numPrimitiveSplitPairs;

	/**
	\brief Vertex position data.
	*/
	PxStrideIterator<const PxVec3> positionBuffer;

	/**
	\brief Vertex velocity data.
	*/
	PxStrideIterator<const PxVec3> velocityBuffer;

	/**
	\brief Vertex inverse mass data.
	*/
	PxStrideIterator<const PxReal> inverseMassBuffer;

	/**
	\brief Vertex normal data.
	*/
	PxStrideIterator<const PxVec3>	normalBuffer;

	/**
	\brief Parent index data.
	*/
	PxStrideIterator<const PxU32> parentIndexBuffer;

	/**
	\brief Index data. Not strided.
	*/
	const PxU32* indexBuffer;

	/**
	\brief Split pair data.
	*/
	PxStrideIterator<const PxDeformablePrimitiveSplitPair> primitiveSplitPairBuffer;

	/**
	\brief virtual destructor
	*/
	virtual ~PxDeformableReadData() {};
};

#ifndef PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif

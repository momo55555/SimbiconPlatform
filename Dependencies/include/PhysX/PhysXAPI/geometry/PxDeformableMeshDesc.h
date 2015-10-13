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


#ifndef PX_PHYSICS_NX_DEFORMABLEMESHDESC
#define PX_PHYSICS_NX_DEFORMABLEMESHDESC
/** \addtogroup deformable
@{
*/

#include "PxVec3.h"
#include "common/PxPhysXCommon.h"

#ifndef PX_DOXYGEN
namespace physx
{
#endif

/**
\brief Deformable primitive type ids.

@see PxDeformableMeshDesc.primitiveType
*/
struct PxDeformablePrimitiveType
{
	enum Enum
	{
		/**
		\brief Default mesh primitive identifier.
		*/
		eNONE			= 0,

		/**
		\brief Type identifier for mesh triangles.

		Mesh triangles are described by a buffer of vertex index triples.
		*/
		eTRIANGLE		= 1,

		/**
		\brief Type identifier for mesh tetrahedra.

		Mesh tetrahedra are described by a buffer of vertex index quadruples.
		*/
		eTETRAHEDRON	= 2,
	};
};

/*----------------------------------------------------------------------------*/

/**
\brief Deformable mesh flags.

@see PxDeformableMeshDesc.flags
*/
struct PxDeformableMeshFlag
{
	enum Enum
	{
		/**
		\brief Denotes the use of 16-bit vertex indices.
		*/
		e16_BIT_INDICES			= (1<<0),

		/**
		\brief Welds close vertices. Ineffective for soft bodies.

		If this flag is set, the cooker maps close vertices (i.e. vertices closer than
		PxDeformableMeshDesc.weldingDistance) to a single internal vertex.
		This is useful when more than one vertex at the same location is used for handling
		multiple texture coordinates. With welding, the mesh does not fall apart when simulated.

		@see NxxDeformableMeshDesc.weldingDistance
		*/
		eWELD_VERTICES			= (1<<2),
	};
};

/*----------------------------------------------------------------------------*/

/**
\brief Deformable vertex flags.

@see PxDeformableMeshDesc.vertexFlags
*/
struct PxDeformableVertexFlag
{
	enum Enum
	{
		/**
		\brief Specifies whether a deformable vertex can be torn.
		*/
		eTEARABLE	= (1<<0),

		/**
		\brief A secondary deformable vertex does not influence regular vertices. Ineffective for cloth.

		Interactions between vertices of the same type are treated normally.
		In an interaction between vertices of different types, the regular
		vertex temporarily gets infinite mass (does not move)
		*/
		eSECONDARY	= (1<<1),
	};
};

/*----------------------------------------------------------------------------*/

/**
\brief Descriptor class for #PxDeformableMesh.

The mesh data is *copied* when an PxDeformableMesh object is created from this descriptor. 
After the creation the user may discard the basic mesh data.

@see PxDeformableMesh
*/
class PxDeformableMeshDesc
{
public:
	/**
	\brief The type of primitives the mesh is composed of.

	Currently either triangles for cloth or tetrahedra for soft bodies.
	*/
	PxDeformablePrimitiveType::Enum primitiveType;

	/**
	\brief Number of vertices.
	*/
	PxU32 numVertices;

	/**
	\brief Number of primitives.
	*/
	PxU32 numPrimitives;

	/**
	\brief Offset between vertex points in bytes.
	*/
	PxU32 vertexStrideBytes;

	/**
	\brief Offset between primitive vertex index tuples in bytes.
	*/
	PxU32 primitiveStrideBytes;

	/**
	\brief Pointer to first vertex point.	

	The caller must ensure that adding vertexStrideBytes bytes to the pointer accesses the next vertex point.
	*/
	const void* vertices;

	/**
	\brief Pointer to the first primitive vertex index tuple.

	The caller must ensure that adding primitiveStrideBytes bytes to the pointer accesses the next index tuple.

	For triangles, we have triples of 0 based indices, where each triple is separated by primitiveStrideBytes:
	vert0 vert1 vert2
	vert0 vert1 vert2
	vert0 vert1 vert2
	...

	For tetrahedra, we have quadruples of 0 based indices:
	vert0 vert1 vert2 vert3
	vert0 vert1 vert2 vert3
	vert0 vert1 vert2 vert3
	...

	where vertex is either a 32 or 16 bit unsigned integer. The user should provide numPrimitives*3 indices for a
	triangle mesh and numPrimitives*4 indices for a tetrahedron mesh.

	'primitives' is declared as a void pointer because it is actually either an PxU16 or a PxU32 pointer.
	*/
	const void* primitives;

	/**
	\brief Offset between vertex masses in bytes.
	*/
	PxU32 vertexMassStrideBytes;

	/**
	\brief Offset between vertex flags in bytes.
	*/
	PxU32 vertexFlagStrideBytes; 

	/**
	\brief Pointer to first vertex mass.

	The caller must ensure that adding vertexMassStrideBytes bytes to the pointer accesses the next vertex mass.
	*/
	const void* vertexMasses;

	/**
	\brief Pointer to first vertex flag. Flags are of type #PxDeformableVertexFlag

	The caller must ensure that adding vertexFlagStrideBytes bytes to the pointer accesses the next vertex flag.
	*/
	const void* vertexFlags;

	/**
	\brief For welding close vertices.

	If the PxDeformableMeshFlag::eWELD_VERTICES flag is set, the cooker maps close vertices 
	(i.e. vertices closer than PxDeformableMeshDesc.weldingDistance) to a single internal vertex.
	This is useful when more than one vertex at the same location is used for handling
	multiple texture coordinates. With welding, the mesh does not fall apart when simulated.

	@see PxDeformableMeshFlag
	*/
	PxReal weldingDistance;

	/**
	\brief Specifies the number of additional mesh levels to be employed by the hierarchical solver.

	If this parameter is greater than zero, the cooker generates a hierarchy of 
	smaller and smaller deformable meshes in addition to the base mesh. 
	With this hierarchy, stretchiness artifacts can be avoided without a large increase 
	in the solver iteration count.

	@see PxDeformableDesc.hierarchicalSolverIterations PxDeformable.getHierarchicalSolverIterations() PxDeformable.setHierarchicalSolverIterations()
	*/
	PxU32 maxHierarchyLevels;

	/**
	\brief Flag bits, combined from values of the enum PxDeformableVertexFlag::Enum
	*/
	PxU32 flags;

	/**
	\brief Constructor sets to default.
	*/
	PX_INLINE PxDeformableMeshDesc();

	/**
	\brief (Re)sets the structure to the default.	
	*/
	PX_INLINE void setToDefault();

	/**
	\brief Returns true if the current settings are valid
	*/
	PX_INLINE bool isValid() const;
};

/*----------------------------------------------------------------------------*/

PX_INLINE PxDeformableMeshDesc::PxDeformableMeshDesc()
{
	primitiveType			= PxDeformablePrimitiveType::eNONE;
	numVertices				= 0;
	numPrimitives			= 0;
	vertexStrideBytes		= 0;
	primitiveStrideBytes	= 0;
	vertices				= NULL;
	primitives				= NULL;	
	vertexMassStrideBytes	= sizeof(PxReal);
	vertexFlagStrideBytes	= sizeof(PxU32);
	vertexMasses			= NULL;
	vertexFlags				= NULL;
	weldingDistance			= 0.0f;
	maxHierarchyLevels		= 0;
	flags					= 0;
}

/*----------------------------------------------------------------------------*/

PX_INLINE void PxDeformableMeshDesc::setToDefault()
{
	*this = PxDeformableMeshDesc();
}

/*----------------------------------------------------------------------------*/

PX_INLINE bool PxDeformableMeshDesc::isValid() const
{
	if (primitiveType == PxDeformablePrimitiveType::eNONE)
		return false;

	// Check geometry
	if(numVertices > 0xffff && (flags & PxDeformableMeshFlag::e16_BIT_INDICES) != 0)
		return false;
	if(!vertices)
		return false;
	if(vertexStrideBytes < sizeof(PxVec3))	//should be at least one vector's worth of data
		return false;

	if(!primitives) 
		return false;

	PxU32 minPrimitiveStride = (flags & PxDeformableMeshFlag::e16_BIT_INDICES) != 0 ? sizeof(PxU16) : sizeof(PxU32);
	if (primitiveType == PxDeformablePrimitiveType::eTRIANGLE)
		minPrimitiveStride *= 3;
	else if ((primitiveType == PxDeformablePrimitiveType::eTETRAHEDRON))
		minPrimitiveStride *= 4;
	if (primitiveStrideBytes < minPrimitiveStride)
		return false;

	if(vertexMasses && (vertexMassStrideBytes < sizeof(PxReal)))
		return false;
	if(vertexFlags && (vertexFlagStrideBytes < sizeof(PxU32)))
		return false;
	if(weldingDistance < 0.0f)
		return false;

	return true;
}

/*----------------------------------------------------------------------------*/

#ifndef PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif

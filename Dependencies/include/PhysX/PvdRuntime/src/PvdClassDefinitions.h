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



#ifndef PVD_CLASSDEFINITIONS_H
#define PVD_CLASSDEFINITIONS_H

#if PX_SUPPORT_VISUAL_DEBUGGER

namespace physx
{
namespace Pvd
{
// ----------------------------- ALL CLASSES -------------------------------- //

struct PvdClassKeys
{
	enum Enum
	{
		PhysicsSDK,
		Scene,
		Actor,
		RigidActor,
		RigidStatic,
		RigidBody,
		RigidDynamic,
		Shape,
		SphereGeometry,
		PlaneGeometry,
		CapsuleGeometry,
		BoxGeometry,
		ConvexMeshGeometry,
		TriangleMeshGeometry,
		HeightFieldGeometry,
		TriangleMesh,
		ConvexMesh,
		HeightField,
		SimulationStatistics,
		Material,
		Articulation,
		ArticulationJoint,
		ArticulationLink,
		ParticleSystem,
		ParticleFluid,
		Deformable,
		DeformableMesh, 
		Attachment,
		Camera,
		Group,
		// special array classes
		VectorArray,
		FloatArray,
		U32Array,
		U16Array,
		U8Array,
		HeightFieldSampleArray,
		ObjectIdArray,
		ParticleFlagsArray,
		DeformableMeshVertexFlagsArray,
		ClothFabricPhaseTypeArray,
		ContactsArray,
		HullPolygonArray,
		AttachmentFlagsArray,
		ClothFabric,
		Cloth,
		ClothPhaseSolverConfigArray,
		ClothParticle,
		NUM_ELEMENTS,
	};
};

}

}

#include "PvdClassDefinitionStructs.h"
#include "PvdClassDefinitionsRigidBody.h"
#include "PvdClassDefinitionsParticleSystem.h"
#include "PvdClassDefinitionsDeformable.h"
#include "PvdClassDefinitionsArrays.h"

namespace physx
{
namespace Pvd
{

static const ClassRow gClassKeyTable[PvdClassKeys::NUM_ELEMENTS] =
{
	//Utility array classes defined before use
	//This is required by the property array feature.
	CLASS_ROW(VectorArray),
	CLASS_ROW(FloatArray),
	CLASS_ROW(U32Array),
	CLASS_ROW(U16Array),
	CLASS_ROW(U8Array),
	CLASS_ROW(ParticleFlagsArray),
	CLASS_ROW(ClothPhaseSolverConfigArray),
	CLASS_ROW(ClothFabricPhaseTypeArray),
	CLASS_ROW(DeformableMeshVertexFlagsArray),
	CLASS_ROW(HeightFieldSampleArray),
	CLASS_ROW(ContactsArray),
	CLASS_ROW(HullPolygonArray),
	CLASS_ROW(AttachmentFlagsArray),
	CLASS_ROW(ObjectIdArray),

	//Main set of objects
	CLASS_ROW_EMPTY(PhysicsSDK),
	CLASS_ROW(Scene),
	CLASS_ROW(Actor),
	DERIVED_CLASS_ROW_EMPTY(RigidActor, Actor),
	DERIVED_CLASS_ROW(RigidStatic, RigidActor),
	DERIVED_CLASS_ROW(RigidBody, RigidActor),
	DERIVED_CLASS_ROW(RigidDynamic, RigidBody),
	CLASS_ROW(Shape),
	CLASS_ROW(SphereGeometry),
	CLASS_ROW_EMPTY(PlaneGeometry),
	CLASS_ROW(CapsuleGeometry),
	CLASS_ROW(BoxGeometry),
	CLASS_ROW(ConvexMeshGeometry),
	CLASS_ROW(TriangleMeshGeometry),
	CLASS_ROW(HeightFieldGeometry),
	CLASS_ROW(TriangleMesh),
	CLASS_ROW(ConvexMesh),
	CLASS_ROW(HeightField),
	CLASS_ROW(SimulationStatistics),
	CLASS_ROW(Material),
	CLASS_ROW(Articulation),
	CLASS_ROW(ArticulationJoint),
	DERIVED_CLASS_ROW(ArticulationLink, RigidBody),
	DERIVED_CLASS_ROW(ParticleSystem, Actor),
	DERIVED_CLASS_ROW(ParticleFluid, ParticleSystem),
	DERIVED_CLASS_ROW(Deformable, Actor),
	CLASS_ROW(DeformableMesh),
	CLASS_ROW(Attachment),
	CLASS_ROW_NOPREFIX(Camera),
	CLASS_ROW_NOPREFIX(Group),
	CLASS_ROW(ClothFabric),
	CLASS_ROW(ClothParticle), //the particle has to be defined before the cloth.
	DERIVED_CLASS_ROW(Cloth,Actor),
};

// ------------------------------------------------------------- //

} // namespace Pvd

}

#endif // PX_SUPPORT_VISUAL_DEBUGGER
#endif // CLASSDEFINITIONS_H

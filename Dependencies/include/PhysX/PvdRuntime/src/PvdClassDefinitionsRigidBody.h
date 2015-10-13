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



#ifndef PVD_CLASSDEFINITIONS_RB_H
#define PVD_CLASSDEFINITIONS_RB_H

#if PX_SUPPORT_VISUAL_DEBUGGER

#include "PxSceneDesc.h"			// PxSceneFlag, PxPruningStructure
#include "PxRigidDynamic.h"			// PxRigidDynamicFlag
#include "PxActorDesc.h"			// PxActorFlag
#include "PxHeightFieldFlag.h"		// PxHeightFieldFlag
#include "PxTriangleMeshGeometry.h"	// PxMeshGeometryFlag
#include "PxShape.h"
#include "PxMaterial.h"

namespace physx
{
namespace Pvd
{

// ------------------------------ PHYSICS SDK ------------------------------- //


// ------------------------------ SCENE ------------------------------ //


struct SceneProp
{
	enum Enum
	{
		NUM_ELEMENTS,
	};
};

static const PropertyRow gSceneProp[1] =
{
	PropertyRow( "", 0 ),
};

// ------------------------------ Actor ------------------------------ //

struct ActorProp
{
	enum Enum
	{
		NUM_ELEMENTS,
	};
};

static const PropertyRow gActorProp[1] =
{
	PropertyRow( "", 0 ),
};

struct RigidActorProp
{
	enum Enum
	{
		NUM_ELEMENTS,
	};
};
struct RigidBodyProp
{
	enum Enum
	{
		NUM_ELEMENTS,
	};
};

// ------------------------------ RigidStatic ------------------------------ //

static const PropertyRow gRigidStaticProp[] =
{
	PropertyRow( "", 0 ),
};

// ------------------------------ RigidBody ------------------------------ //

static const PropertyRow gRigidBodyProp[] =
{
	PropertyRow( "", 0 ),
};

// ------------------------------ RigidDynamic ------------------------------ //

static const PropertyRow gRigidDynamicProp[1] =
{
	PropertyRow( "", 0 ),
};

// ------------------------------ Shape ------------------------------ //

static const PropertyRow gShapeProp[1] =
{
	PropertyRow( "", 0 ),
};

// ------------------------------ SphereGeometry ------------------------------ //

struct SphereGeometryProp
{
	enum Enum
	{
		Radius,
		NUM_ELEMENTS,
	};
};

static const PropertyRow gSphereGeometryProp[SphereGeometryProp::NUM_ELEMENTS] =
{
	PROPERTY_ROW(Radius, Float),
};

// ------------------------------ PlaneGeometry ------------------------------ //


// ------------------------------ CapsuleGeometry ------------------------------ //

struct CapsuleGeometryProp
{
	enum Enum
	{
		Radius,
		HalfHeight,
		NUM_ELEMENTS,
	};
};

static const PropertyRow gCapsuleGeometryProp[CapsuleGeometryProp::NUM_ELEMENTS] =
{
	PROPERTY_ROW(Radius, Float),
	PROPERTY_ROW(HalfHeight, Float),
};

// ------------------------------ BoxGeometry ------------------------------ //

struct BoxGeometryProp
{
	enum Enum
	{
		HalfExtents,
		NUM_ELEMENTS,
	};
};

static const PropertyRow gBoxGeometryProp[BoxGeometryProp::NUM_ELEMENTS] =
{
	
	PROPERTY_ROW(Dimensions, Float3),
};

// ------------------------------ ConvexMeshGeometry ------------------------------ //

struct ConvexMeshGeometryProp
{
	enum Enum
	{
		Scale_Scale,
		Scale_Rot,
		ConvexMesh,
		NUM_ELEMENTS,
	};
};

static const PropertyRow gConvexMeshGeometryProp[ConvexMeshGeometryProp::NUM_ELEMENTS] =
{
	PROPERTY_ROW(Scale.Scale, Float3),
	PROPERTY_ROW(Scale.Rot, Quat),
	PROPERTY_ROW(ConvexMesh, ObjectId),
};

// ------------------------------ TriangleMeshGeometry ------------------------------ //

static const PVD::NamedValueDefinition gMeshGeometryFlags[] =
{
	FLAG_ROW(PxMeshGeometryFlag, eDOUBLE_SIDED),
};

struct TriangleMeshGeometryProp
{
	enum Enum
	{
		Scale_Scale,
		Scale_Rot,
		Flags,
		TriangleMesh,
		NUM_ELEMENTS,
	};
};

static const PropertyRow gTriangleMeshGeometryProp[TriangleMeshGeometryProp::NUM_ELEMENTS] =
{
	PROPERTY_ROW(Scale.Scale, Float3),
	PROPERTY_ROW(Scale.Rot, Quat),
	BITFLAG_PROPERTY_ROW(Flags, gMeshGeometryFlags),
	PROPERTY_ROW(TriangleMesh, ObjectId),
};

// ------------------------------ HeightFieldGeometry ------------------------------ //

struct HeightFieldGeometryProp
{
	enum Enum
	{
		HeightScale,
		RowScale,
		ColumnScale,
		Flags,
		HeightField,
		NUM_ELEMENTS,
	};
};

static const PropertyRow gHeightFieldGeometryProp[HeightFieldGeometryProp::NUM_ELEMENTS] =
{
	PROPERTY_ROW(HeightScale, Float),
	PROPERTY_ROW(RowScale, Float),
	PROPERTY_ROW(ColumnScale, Float),
	BITFLAG_PROPERTY_ROW(Flags, gMeshGeometryFlags),
	PROPERTY_ROW(HeightField, ObjectId),
};

// ------------------------------ ConvexMesh ------------------------------ //

struct ConvexMeshProp
{
	enum Enum
	{
		VertexArray,
		IndexArray,
		HullPolygonArray,
		Mass,
		LocalInertia,
		LocalCenterOfMass,
		NUM_ELEMENTS,
	};
};

static const PropertyRow gConvexMeshProp[ConvexMeshProp::NUM_ELEMENTS] =
{
	ARRAY_PROPERTY_ROW(VertexArray, VectorArray),
	ARRAY_PROPERTY_ROW(IndexArray, U8Array),
	ARRAY_PROPERTY_ROW(HullPolygonArray, HullPolygonArray),
	PROPERTY_ROW(Mass, Float),
	PROPERTY_ROW(LocalInertia, Quat),
	PROPERTY_ROW(LocalCenterOfMass, Float3),
};

// ------------------------------ TriangleMesh ------------------------------ //

struct TriangleMeshProp
{
	enum Enum
	{
		VertexArray,
		TriangleIndexArray,
		TriangleMaterialArray,
		NUM_ELEMENTS,
	};
};

static const PropertyRow gTriangleMeshProp[TriangleMeshProp::NUM_ELEMENTS] =
{
	ARRAY_PROPERTY_ROW(VertexArray, VectorArray),
	ARRAY_PROPERTY_ROW(TriangleIndexArray, U32Array),
	ARRAY_PROPERTY_ROW(TriangleMaterialArray, U16Array),
};

// ------------------------------ HeightField ------------------------------ //

static const PVD::NamedValueDefinition gHeightFieldFlags[1] =
{
	FLAG_ROW(PxHeightFieldFlag, eNO_BOUNDARY_EDGES),
};

static const PVD::NamedValueDefinition gHeightFieldFormatEnum[1] =
{
	FLAG_ROW(PxHeightFieldFormat, eS16_TM),
};

struct HeightFieldProp
{
	enum Enum
	{
		Samples,
		NumRows,
		NumColumns,
		HeightFieldFormat,
		Thickness,
		ConvexEdgeThreshold,
		Flags,
		NUM_ELEMENTS,
	};
};

static const PropertyRow gHeightFieldProp[HeightFieldProp::NUM_ELEMENTS] =
{
	ARRAY_PROPERTY_ROW(Samples, HeightFieldSampleArray),
	PROPERTY_ROW(NumRows, U32),
	PROPERTY_ROW(NumColumns, U32),
	ENUM_PROPERTY_ROW(HeightFieldFormat, gHeightFieldFormatEnum),
	PROPERTY_ROW(Thickness, Float),
	PROPERTY_ROW(ConvexEdgeThreshold, Float),
	BITFLAG_PROPERTY_ROW(Flags, gHeightFieldFlags),
};

// ------------------------------ SimulationStatistics ------------------------------ //

static const PropertyRow gSimulationStatisticsProp[] =
{
	PropertyRow( "", 0 ),
};

// ----------------------------- Material -------------------------------- //
// meta data enabled

static const PropertyRow gMaterialProp[1] =
{
	PropertyRow( "", 0 ),
};

// ----------------------------- Articulation -------------------------------- //


static const PropertyRow gArticulationProp[] =
{
	PropertyRow( "", 0 )
};

// ----------------------------- Articulation Joint -------------------------------- //


static const PropertyRow gArticulationJointProp[] =
{
	PropertyRow( "", 0 )
};

// ----------------------------- Articulation Link -------------------------------- //

static const PropertyRow gArticulationLinkProp[] =
{
	PropertyRow( "", 0 )
};

// ----------------------------- Camera -------------------------------- //

struct CameraProp
{
	enum Enum
	{
		Origin,
		Target,
		Up,
		Name,
		NUM_ELEMENTS,
	};
};

static const PropertyRow gCameraProp[CameraProp::NUM_ELEMENTS] =
{
	PROPERTY_ROW(Origin, Float3),
	PROPERTY_ROW(Target, Float3),
	PROPERTY_ROW(Up, Float3),
	PROPERTY_ROW(Name, String),
};

// ----------------------------- Group -------------------------------- //

struct GroupProp
{
	enum Enum
	{
		Name,
		NUM_ELEMENTS,
	};
};

static const PropertyRow gGroupProp[GroupProp::NUM_ELEMENTS] =
{
	PROPERTY_ROW(Name, String),
};

struct ClothFabricProp
{
	enum Enum
	{
		NUM_ELEMENTS,
	};
};

static const PropertyRow gClothFabricProp[1] =
{
	PropertyRow( "", 0 ),
};


struct ClothProp
{
	enum Enum
	{
		NUM_ELEMENTS,
	};
};

static const PropertyRow gClothProp[1] =
{
	PropertyRow( "", 0 ),
};

struct ClothParticleProp
{
	enum Enum
	{
		NUM_ELEMENTS,
	};
};

static const PropertyRow gClothParticleProp[1] =
{
	PropertyRow( "", 0 ),
};
// -------------------------------------------------------------------- //

} // namespace Pvd

}

#endif // PX_SUPPORT_VISUAL_DEBUGGER
#endif // PVD_CLASSDEFINITIONS_RB_H


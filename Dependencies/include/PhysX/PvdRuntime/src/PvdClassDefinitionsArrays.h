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



#ifndef PVD_CLASSDEFINITIONS_ARRAYS_H
#define PVD_CLASSDEFINITIONS_ARRAYS_H

#include "cloth/PxClothTypes.h"
#include "cloth/PxClothFabric.h"

#if PX_SUPPORT_VISUAL_DEBUGGER

namespace physx
{
namespace Pvd
{

// ----------------------------- Vector Array -------------------------------- //

struct VectorArrayProp
{
	enum Enum
	{
		Element,
		NUM_ELEMENTS,
	};
};

static const PropertyRow gVectorArrayProp[VectorArrayProp::NUM_ELEMENTS] =
{
	PROPERTY_ROW(Element, Float3),
};

// ----------------------------- Float Array -------------------------------- //

struct FloatArrayProp
{
	enum Enum
	{
		Element,
		NUM_ELEMENTS,
	};
};

static const PropertyRow gFloatArrayProp[FloatArrayProp::NUM_ELEMENTS] =
{
	PROPERTY_ROW(Element, Float),
};

// ----------------------------- U32 Array -------------------------------- //

struct U32ArrayProp
{
	enum Enum
	{
		Element,
		NUM_ELEMENTS,
	};
};

static const PropertyRow gU32ArrayProp[U32ArrayProp::NUM_ELEMENTS] =
{
	PROPERTY_ROW(Element, U32),
};

// ----------------------------- U16 Array -------------------------------- //

struct U16ArrayProp
{
	enum Enum
	{
		Element,
		NUM_ELEMENTS,
	};
};

static const PropertyRow gU16ArrayProp[U16ArrayProp::NUM_ELEMENTS] =
{
	PROPERTY_ROW(Element, U16),
};

// ----------------------------- U8 Array -------------------------------- //

struct U8ArrayProp
{
	enum Enum
	{
		Element,
		NUM_ELEMENTS,
	};
};

static const PropertyRow gU8ArrayProp[U8ArrayProp::NUM_ELEMENTS] =
{
	PROPERTY_ROW(Element, U8),
};

// ----------------------------- HeightFieldSample Array -------------------------------- //

struct HeightFieldSampleArrayProp
{
	enum Enum
	{
		Element,
		NUM_ELEMENTS,
	};
};

static const PropertyRow gHeightFieldSampleArrayProp[HeightFieldSampleArrayProp::NUM_ELEMENTS] =
{
	PROPERTY_ROW(Element, HeightFieldSample),
};

// ----------------------------- ObjectId Array -------------------------------- //

struct ObjectIdArrayProp
{
	enum Enum
	{
		Element,
		NUM_ELEMENTS,
	};
};

static const PropertyRow gObjectIdArrayProp[ObjectIdArrayProp::NUM_ELEMENTS] =
{
	PROPERTY_ROW(Element, ObjectId),
};

// ----------------------------- ParticleFlags Array -------------------------------- //

struct ParticleFlagsArrayProp
{
	enum Enum
	{
		Element,
		NUM_ELEMENTS,
	};
};

static const PVD::NamedValueDefinition gParticleFlags[7] =
{
	FLAG_ROW(PxParticleFlag, eVALID),
	FLAG_ROW(PxParticleFlag, eCOLLISION_WITH_STATIC),
	FLAG_ROW(PxParticleFlag, eCOLLISION_WITH_DYNAMIC),
	FLAG_ROW(PxParticleFlag, eCOLLISION_WITH_DRAIN),
	FLAG_ROW(PxParticleFlag, eSPATIAL_DATA_STRUCTURE_OVERFLOW),
};


static const PropertyRow gParticleFlagsArrayProp[ParticleFlagsArrayProp::NUM_ELEMENTS] =
{
	BITFLAG_PROPERTY_ROW(Element, gParticleFlags),
};

// ----------------------------- ClothFabricPhaseType Array -------------------------------- //

struct ClothFabricPhaseTypeArrayProp
{
	enum Enum
	{
		Element,
		NUM_ELEMENTS,
	};
};

static const PVD::NamedValueDefinition gClothFabricPhaseTypeFlags[4] =
{
	FLAG_ROW(PxClothFabricPhaseType, eINVALID),
	FLAG_ROW(PxClothFabricPhaseType, eSTRETCHING),
	FLAG_ROW(PxClothFabricPhaseType, eBENDING),
	FLAG_ROW(PxClothFabricPhaseType, eSHEARING),
};

static const PropertyRow gClothFabricPhaseTypeArrayProp[ClothFabricPhaseTypeArrayProp::NUM_ELEMENTS] =
{
	ENUM_PROPERTY_ROW(Element, gClothFabricPhaseTypeFlags),
};

// ----------------------------- ClothPhaseSolverConfig Array -------------------------------- //

struct ClothPhaseSolverConfigArrayProp
{
	enum Enum
	{
		Element,
		NUM_ELEMENTS,
	};
};

static const PVD::NamedValueDefinition gClothPhaseSolverConfigFlags[4] =
{
	FLAG_ROW(PxClothPhaseSolverConfig, eINVALID),
	FLAG_ROW(PxClothPhaseSolverConfig, eFAST),
	FLAG_ROW(PxClothPhaseSolverConfig, eSTIFF),
};

static const PropertyRow gClothPhaseSolverConfigArrayProp[ClothPhaseSolverConfigArrayProp::NUM_ELEMENTS] =
{
	ENUM_PROPERTY_ROW(Element, gClothPhaseSolverConfigFlags),
};

// ----------------------------- DeformableMeshVertexFlags Array -------------------------------- //

struct DeformableMeshVertexFlagsArrayProp
{
	enum Enum
	{
		Element,
		NUM_ELEMENTS,
	};
};

static const PropertyRow gDeformableMeshVertexFlagsArrayProp[DeformableMeshVertexFlagsArrayProp::NUM_ELEMENTS] =
{
	BITFLAG_PROPERTY_ROW(Element, gDeformableVertexFlags),
};

// ----------------------------- AttachmentFlags Array -------------------------------- //

struct AttachmentFlagsArrayProp
{
	enum Enum
	{
		Element,
		NUM_ELEMENTS,
	};
};

static const PropertyRow gAttachmentFlagsArrayProp[AttachmentFlagsArrayProp::NUM_ELEMENTS] =
{
	BITFLAG_PROPERTY_ROW(Element, gAttachmentFlags),
};

// ----------------------------- Contacts Array -------------------------------- //

struct ContactsArrayProp
{
	enum Enum
	{
		Point,
		Normal,
		Shape0,
		Shape1,
		Separation,
		NormalForce,
		FeatureIndex0,
		FeatureIndex1,
		NormalForceAvailable,
		NUM_ELEMENTS,
	};
};

static const PropertyRow gContactsArrayProp[ContactsArrayProp::NUM_ELEMENTS] =
{
	PROPERTY_ROW(Point, Float3),
	PROPERTY_ROW(Normal, Float3),
	PROPERTY_ROW(Shape0, ObjectId),
	PROPERTY_ROW(Shape1, ObjectId),
	PROPERTY_ROW(Separation, Float),
	PROPERTY_ROW(NormalForce, Float),
	PROPERTY_ROW(FeatureIndex0, U32),
	PROPERTY_ROW(FeatureIndex1, U32),
	PROPERTY_ROW(NormalForceAvailable, Boolean),
};

// ----------------------------- HullPolygon Array -------------------------------- //

struct HullPolygonArrayProp
{
	enum Enum
	{
		Plane,
		NumVertices,
		IndexBase,
		NUM_ELEMENTS,
	};
};

static const PropertyRow gHullPolygonArrayProp[HullPolygonArrayProp::NUM_ELEMENTS] =
{
	PROPERTY_ROW(Plane, Plane),
	PROPERTY_ROW(NumVertices, U16),
	PROPERTY_ROW(IndexBase, U16),
};

// ------------------------------------------------------------- //

} // namespace Pvd

}

#endif // PX_SUPPORT_VISUAL_DEBUGGER
#endif // PVD_CLASSDEFINITIONS_ARRAYS_H

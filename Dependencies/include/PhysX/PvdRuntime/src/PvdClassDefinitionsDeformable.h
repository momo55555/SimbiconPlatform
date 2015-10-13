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



#ifndef PVD_CLASSDEFINITIONS_DEFORMABLE_H
#define PVD_CLASSDEFINITIONS_DEFORMABLE_H

#if PX_SUPPORT_VISUAL_DEBUGGER

#include "PxDeformableDesc.h"		// DeformableFlags, DeformableReadDataFlags
#include "PxDeformableMeshDesc.h"	// DeformableMeshFlags
#include "PxAttachment.h"			// AttachmentFlags

namespace physx
{
namespace Pvd
{
// ----------------------------- Deformable -------------------------------- //


static const PropertyRow gDeformableProp[] =
{
	PropertyRow( "", 0 ),
};

// ----------------------------- Deformable Mesh -------------------------------- //

static const PVD::NamedValueDefinition gDeformablePrimitiveTypeEnum[3] =
{
	FLAG_ROW(PxDeformablePrimitiveType, eNONE),
	FLAG_ROW(PxDeformablePrimitiveType, eTRIANGLE),
	FLAG_ROW(PxDeformablePrimitiveType, eTETRAHEDRON),
};

static const PVD::NamedValueDefinition gDeformableMeshFlags[2] =
{
	FLAG_ROW(PxDeformableMeshFlag, e16_BIT_INDICES),
	FLAG_ROW(PxDeformableMeshFlag, eWELD_VERTICES),
};

static const PVD::NamedValueDefinition gDeformableVertexFlags[2] =
{
	FLAG_ROW(PxDeformableVertexFlag, eTEARABLE),
	FLAG_ROW(PxDeformableVertexFlag, eSECONDARY),
};

struct DeformableMeshProp
{
	enum Enum
	{
		PrimitiveType,
		WeldingDistance,
		MaxHierarchyLevels,
		Flags,
		VertexArray,
		PrimitiveArray,
		VertexMassArray,
		VertexFlagsArray,
		NUM_ELEMENTS,
	};
};

static const PropertyRow gDeformableMeshProp[DeformableMeshProp::NUM_ELEMENTS] =
{
	ENUM_PROPERTY_ROW(PrimitiveType, gDeformablePrimitiveTypeEnum),
	PROPERTY_ROW(WeldingDistance, Float),
	PROPERTY_ROW(MaxHierarchyLevels, U32),
	BITFLAG_PROPERTY_ROW(Flags, gDeformableMeshFlags),
	ARRAY_PROPERTY_ROW(VertexArray, VectorArray),
	ARRAY_PROPERTY_ROW(PrimitiveArray, U32Array),
	ARRAY_PROPERTY_ROW(VertexMassArray, FloatArray),
	ARRAY_PROPERTY_ROW(VertexFlagsArray, DeformableMeshVertexFlagsArray),
};

// ----------------------------- Attachment -------------------------------- //

static const PVD::NamedValueDefinition gAttachmentFlags[3] =
{
	FLAG_ROW(PxAttachmentFlag, eTWOWAY),
	FLAG_ROW(PxAttachmentFlag, eTEARABLE),
	FLAG_ROW(PxAttachmentFlag, eTORN),
};

struct AttachmentProp
{
	enum Enum
	{
		PositionArray,
		VertexIndexArray,
		FlagsArray,
		NUM_ELEMENTS,
	};
};

static const PropertyRow gAttachmentProp[AttachmentProp::NUM_ELEMENTS] =
{
	ARRAY_PROPERTY_ROW(PositionArray, VectorArray),
	ARRAY_PROPERTY_ROW(VertexIndexArray, U32Array),
	ARRAY_PROPERTY_ROW(VertexFlagsArray, AttachmentFlagsArray),
};

// ------------------------------------------------------------- //

} // namespace Pvd

}

#endif // PX_SUPPORT_VISUAL_DEBUGGER
#endif // PVD_CLASSDEFINITIONS_DEFORMABLE_H

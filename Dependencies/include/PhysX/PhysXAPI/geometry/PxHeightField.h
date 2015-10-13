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


#ifndef PX_PHYSICS_GEOMUTILS_NX_HEIGHTFIELD
#define PX_PHYSICS_GEOMUTILS_NX_HEIGHTFIELD
/** \addtogroup geomutils
  @{
*/
#include "geometry/PxPhysXGeomUtils.h"
#include "geometry/PxHeightFieldFlag.h"

// PX_SERIALIZATION
#include "common/PxSerialFramework.h"
//~PX_SERIALIZATION

#ifndef PX_DOXYGEN
namespace physx
{
#endif

class PxHeightFieldDesc;

/**
\brief A height field class.  

Height fields work in a similar way as triangle meshes specified to act as 
height fields, with some important differences:

Triangle meshes can be made of nonuniform geometry, while height fields are 
regular, rectangular grids.  This means that with PxHeightField, you sacrifice 
flexibility in return for improved performance and decreased memory consumption.

Like Convexes and TriangleMeshes, HeightFields are referenced by shape instances 
(see #PxHeightFieldGeometry, #PxShape).

To avoid duplicating data when you have several instances of a particular 
height field differently, you do not use this class to represent a 
height field object directly. Instead, you create an instance of this height field 
via the PxHeightFieldGeometry and PxShape classes.

<h3>Creation</h3>

To create an instance of this class call PxPhysics::createHeightField(),
and release() to delete it. This is only possible
once you have released all of its PxHeightFiedShape instances.

<h3>Visualizations:</h3>
\li #PxVisualizationParameter::eCOLLISION_AABBS
\li #PxVisualizationParameter::eCOLLISION_SHAPES
\li #PxVisualizationParameter::eCOLLISION_AXES
\li #PxVisualizationParameter::eCOLLISION_FNORMALS
\li #PxVisualizationParameter::eCOLLISION_EDGES

@see PxHeightFieldDesc PxHeightFieldGeometry PxShape PxPhysics.createHeightField()
*/

class PxHeightField
// PX_SERIALIZATION
	: public PxSerializable
//~PX_SERIALIZATION
	{
	public:
	/**
	\brief Releases the height field.
	
	\note This will decrease the reference count by one.

	Releases the application's reference to the height field.
	The height field is destroyed when the application's reference is released and all shapes referencing the height field are destroyed.

	@see PxPhysics.createHeightField() PxHeightFieldDesc PxHeightFieldGeometry PxShape
	*/
	virtual		void						release() = 0;

	/**
    \brief Writes out the sample data array.
	
	The user provides destBufferSize bytes storage at destBuffer.
	The data is formatted and arranged as PxHeightFieldDesc.samples.

	\param[out] destBuffer The destination buffer for the sample data.
	\param[in] destBufferSize The size of the destination buffer.
	\return The number of bytes written.

	@see PxHeightFieldDesc.samples
	*/
    virtual		PxU32						saveCells(void* destBuffer, PxU32 destBufferSize) const = 0;

	/**
	\brief Retrieves the number of sample rows in the samples array.

	\return The number of sample rows in the samples array.

	@see PxHeightFieldDesc.nbRows
	*/
	virtual		PxU32						getNbRows()					const = 0;

	/**
	\brief Retrieves the number of sample columns in the samples array.

	\return The number of sample columns in the samples array.

	@see PxHeightFieldDesc.nbColumns
	*/
	virtual		PxU32						getNbColumns()				const = 0;

	/**
	\brief Retrieves the format of the sample data.
	
	\return The format of the sample data.

	@see PxHeightFieldDesc.format PxHeightFieldFormat
	*/
	virtual		PxHeightFieldFormat::Enum	getFormat()					const = 0;

	/**
	\brief Retrieves the offset in bytes between consecutive samples in the array.

	\return The offset in bytes between consecutive samples in the array.

	@see PxHeightFieldDesc.sampleStride
	*/
	virtual		PxU32						getSampleStride()			const = 0;

	/**
	\brief Retrieves the thickness of the height volume in the vertical direction.

	\return The thickness of the height volume in the vertical direction.

	@see PxHeightFieldDesc.thickness
	*/
	virtual		PxReal						getThickness()				const = 0;

	/**
	\brief Retrieves the convex edge threshold.

	\return The convex edge threshold.

	@see PxHeightFieldDesc.convexEdgeThreshold
	*/
	virtual		PxReal						getConvexEdgeThreshold()	const = 0;

	/**
	\brief Retrieves the flags bits, combined from values of the enum ::PxHeightFieldFlag.

	\return The flags bits, combined from values of the enum ::PxHeightFieldFlag.

	@see PxHeightFieldDesc.flags PxHeightFieldFlag
	*/
	virtual		PxHeightFieldFlags			getFlags()					const = 0;

	/**
	\brief Retrieves the height at the given coordinates in grid space.

	\return The height at the given coordinates or 0 if the coordinates are out of range.
	*/
	virtual		PxReal						getHeight(PxReal x, PxReal z) const = 0;

	/**
	\brief Returns the reference count for shared heightfields.

	At creation, the reference count of the heightfield is 1. Every shape referencing this heightfield increments the
	count by 1.	When the reference count reaches 0, and only then, the heightfield gets destroyed automatically.

	\return the current reference count.
	*/
	virtual		PxU32						getReferenceCount()			const	= 0;

	/**
	\brief Returns material table index of given triangle

	\note This function takes a post cooking triangle index.

	\param[in] triangleIndex (internal) index of desired triangle
	\return Material table index, or 0xffff if no per-triangle materials are used
	*/
	virtual	PxMaterialTableIndex	getTriangleMaterialIndex(PxTriangleID triangleIndex)	const	= 0;

	protected:
	// PX_SERIALIZATION
	PxHeightField()										{}
	PxHeightField(PxRefResolver& v)	: PxSerializable(v)	{}
	PX_DECLARE_SERIAL_RTTI(PxHeightField, PxSerializable)
	//~PX_SERIALIZATION

	virtual ~PxHeightField(){}
	};

#ifndef PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif

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


#ifndef PX_PHYSICS_GEOMUTILS_NX_CONVEXMESH
#define PX_PHYSICS_GEOMUTILS_NX_CONVEXMESH
/** \addtogroup geomutils
  @{
*/

#include "Px.h"

// PX_SERIALIZATION
#include "common/PxSerialFramework.h"
//~PX_SERIALIZATION

#ifndef PX_DOXYGEN
namespace physx
{
#endif

class PxSimpleTriangleMesh;
class PxConvexMeshDesc;

/**
\brief Polygon data
*/
struct PxHullPolygon
{
	PxReal			mPlane[4];		//!< Plane equation for this polygon
	PxU16			mNbVerts;		//!< Number of vertices/edges in the polygon
	PxU16			mIndexBase;		//!< Offset in index buffer
};

/**
\brief A convex mesh.

Internally represented as a list of convex polygons. The number
of polygons is limited to 256.

To avoid duplicating data when you have several instances of a particular 
mesh positioned differently, you do not use this class to represent a 
convex object directly. Instead, you create an instance of this mesh via
the PxConvexMeshGeometry and PxShape classes.

<h3>Creation</h3>

To create an instance of this class call PxPhysics::createConvexMesh(),
and PxConvexMesh::release() to delete it. This is only possible
once you have released all of its #PxShape instances.

<h3>Visualizations:</h3>
\li #PxVisualizationParameter::eCOLLISION_AABBS
\li #PxVisualizationParameter::eCOLLISION_SHAPES
\li #PxVisualizationParameter::eCOLLISION_AXES
\li #PxVisualizationParameter::eCOLLISION_FNORMALS
\li #PxVisualizationParameter::eCOLLISION_EDGES

@see PxConvexMeshDesc PxPhysics.createConvexMesh()
*/
class PxConvexMesh
// PX_SERIALIZATION
	: public PxSerializable
//~PX_SERIALIZATION
{
	public:

	/**
	\brief Returns the number of vertices.
	\return	Number of vertices.
	@see getVertices()
	*/
	virtual	PxU32				getNbVertices()									const	= 0;

	/**
	\brief Returns the vertices.
	\return	Array of vertices.
	@see getNbVertices()
	*/
	virtual	const PxVec3*		getVertices()									const	= 0;

	/**
	\brief Returns the index buffer.
	\return	Index buffer.
	@see getNbPolygons() getPolygonData()
	*/
	virtual	const PxU8*			getIndexBuffer()								const	= 0;

	/**
	\brief Returns the number of polygons.
	\return	Number of polygons.
	@see getIndexBuffer() getPolygonData()
	*/
	virtual	PxU32				getNbPolygons()									const	= 0;

	/**
	\brief Returns the polygon data.
	\param[in] index	Polygon index in [0 ; getNbPolygons()[.
	\param[out] data	Polygon data.
	\return	True if success.
	@see getIndexBuffer() getNbPolygons()
	*/
	virtual	bool				getPolygonData(PxU32 index, PxHullPolygon& data)	const	= 0;

	/**
	\brief Releases the convex mesh.

	\note This will decrease the reference count by one.

	Releases the application's reference to the convex mesh.
	The mesh is destroyed when the application's reference is released and all shapes referencing the mesh are destroyed.

	@see PxPhysics.createConvexMesh() PxConvexMeshGeometry PxShape
	*/
	virtual	void				release() = 0;

	/**
	\brief Returns the reference count for shared meshes.

	At creation, the reference count of the convex mesh is 1. Every shape referencing this convex mesh increments the
	count by 1.	When the reference count reaches 0, and only then, the convex mesh gets destroyed automatically.

	\return the current reference count.
	*/
	virtual PxU32				getReferenceCount()			const	= 0;

	/**
	\brief Returns the mass properties of the mesh.

	\param[out] mass The mass of the mesh.
	\param[out] localInertia The inertia tensor in mesh local space.
	\param[out] localCenterOfMass Position of center of mass in mesh local space.
	*/
	virtual void				getMassInformation(PxReal& mass, PxMat33& localInertia, PxVec3& localCenterOfMass)		const	= 0;
	
	protected:
	virtual ~PxConvexMesh(){}
	// PX_SERIALIZATION
	PxConvexMesh()										{}
	PxConvexMesh(PxRefResolver& v)	: PxSerializable(v)	{}
	PX_DECLARE_SERIAL_RTTI(PxConvexMesh, PxSerializable)
	//~PX_SERIALIZATION
};

#ifndef PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif

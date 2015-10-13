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


#ifndef PX_PHYSICS_EXTENSIONS_SIMPLE_FACTORY_H
#define PX_PHYSICS_EXTENSIONS_SIMPLE_FACTORY_H
/** \addtogroup extensions
  @{
*/

#include "common/PxPhysXCommon.h"
#include "PxTransform.h"

#ifndef PX_DOXYGEN
namespace physx
{
#endif

	class PxPhysics;
	class PxMaterial;
	class PxRigidDynamic;
	class PxRigidStatic;
	class PxGeometry;

#ifndef PX_DOXYGEN
} // namespace physx
#endif


/** \brief simple method to create a PxRigidDynamic actor with a single PxShape. 

	\param[in] sdk the PxPhysics object
	\param[in] transform the global pose of the new object
	\param[in] geometry the geometry of the new object's shape, which must be a sphere, capsule, box or convex
	\param[in] material the material for the new object's shape
	\param[in] density the density of the new object. Must be greater than zero.
	\param[in] shapeOffset an optional offset for the new shape, defaults to identity

	\return a new dynamic actor with the PxRigidDynamicFlag, or NULL if it could 
	not be constructed

	@see PxRigidDynamic PxShapeFlag
*/

PX_C_EXPORT	physx::PxRigidDynamic*	PX_CALL_CONV	PxCreateDynamic(physx::PxPhysics& sdk,
																	const physx::PxTransform& transform,
																	const physx::PxGeometry& geometry,
																	physx::PxMaterial& material,
																	physx::pubfnd3::PxReal density,
																	const physx::PxTransform &shapeOffset = physx::PxTransform::createIdentity());

/** \brief simple method to create a kinematic PxRigidDynamic actor with a single PxShape. 

	\param[in] sdk the PxPhysics object
	\param[in] transform the global pose of the new object
	\param[in] geometry the geometry of the new object's shape
	\param[in] material the material for the new object's shape
	\param[in] density the density of the new object. Must be greater than zero if the object is to participate in simulation.
	\param[in] shapeOffset an optional offset for the new shape, defaults to identity

	\note unlike PxCreateDynamic, the geometry is not restricted to box, capsule, sphere or convex. However, 
	kinematics of other geometry types may not participate in simulation collision and may be used only for
	triggers or scene queries of moving objects under animation control. In this case the density parameter
	will be ignored and the created shape will be set up as a scene query only shape (see #PxShapeFlag::eSCENE_QUERY_SHAPE)

	\return a new dynamic actor with the PxRigidDynamicFlag::eKINEMATIC set, or NULL if it could 
	not be constructed

	@see PxRigidDynamic PxShapeFlag
*/

PX_C_EXPORT	physx::PxRigidDynamic*	PX_CALL_CONV	PxCreateKinematic(physx::PxPhysics& sdk,
																	  const physx::PxTransform& transform,
																	  const physx::PxGeometry& geometry,
																	  physx::PxMaterial& material,
																	  physx::pubfnd3::PxReal density,
																	  const physx::PxTransform &shapeOffset = physx::PxTransform::createIdentity());

/** \brief simple method to create a PxRigidStatic actor with a single PxShape. 

	\param[in] sdk the PxPhysics object
	\param[in] transform the global pose of the new object
	\param[in] geometry the geometry of the new object's shape
	\param[in] material the material for the new object's shape
	\param[in] shapeOffset an optional offset for the new shape, defaults to identity

	\return a new static actor, or NULL if it could not be constructed

	@see PxRigidStatic
*/

PX_C_EXPORT	physx::PxRigidStatic*	PX_CALL_CONV	PxCreateStatic(physx::PxPhysics& sdk,
																   const physx::PxTransform& transform,
																   const physx::PxGeometry& geometry,
																   physx::PxMaterial& material,
																   const physx::PxTransform &shapeOffset = physx::PxTransform::createIdentity());

/** \brief simple method to create a plane. The plane equation is n.x + d = 0

	\param[in] sdk the PxPhysics object
	\param[in] outwardNormal unit normal of the plane 
	\param[in] distance of the plane from origin
	\param[in] material the material for the new object's shape

	\return a new static actor, or NULL if it could not be constructed

	@see PxRigidStatic
*/

PX_C_EXPORT	physx::PxRigidStatic*	PX_CALL_CONV	PxCreatePlane(physx::PxPhysics& sdk,
																  const physx::pubfnd3::PxVec3 &outwardNormal,
																  const physx::pubfnd3::PxReal distance,
																  physx::PxMaterial& material);

/** @} */
#endif

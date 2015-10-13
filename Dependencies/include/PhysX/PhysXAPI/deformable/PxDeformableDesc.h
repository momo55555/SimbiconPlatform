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


#ifndef PX_PHYSICS_NX_DEFORMABLEDESC
#define PX_PHYSICS_NX_DEFORMABLEDESC
/** \addtogroup deformable
@{
*/

#include "PxPhysX.h"
#include "PxBounds3.h"
#include "PxActorDesc.h"
#include "PxFiltering.h"
#include "deformable/PxDeformableReadData.h"

#ifndef PX_DOXYGEN
namespace physx
{
#endif

class PxDeformableMesh;

/**
\brief Collection of flags describing the behavior of a deformable.

@see PxDeformable PxDeformableMesh PxDeformableMeshDesc
*/
PX_DEPRECATED struct PxDeformableFlag
{
	enum Enum
	{
		/**
		\brief Enable/disable pressure simulation. Ineffective for soft bodies.

		Note: For triangle meshes, pressure only produces meaningful results if the mesh is closed.

		@see PxDeformableDesc.pressure
		*/
		ePRESSURE						= (1<<0),

		/**
		\brief Makes the deformable static. 
		*/
		eSTATIC							= (1<<1),

		/**
		\brief Enable/disable self-collision handling within a single piece of deformable.

		Note: self-collisions are only handled inbetween the deformable's vertices, i.e.,
		vertices do not collide against the triangles/tetrahedra of the deformable.
		The user should therefore specify a large enough selfCollisionOffset to avoid
		most interpenetrations.

		@see PxDeformableDesc.selfCollisionOffset
		*/
		eSELFCOLLISION					= (1<<2),

		/**
		\brief Enable/disable bending resistance. Ineffective for soft bodies.

		Select the bending resistance through PxDeformableDesc.bendingStiffness.

		@see PxDeformableDesc.bendingStiffness
		*/
		eBENDING						= (1<<3),

		/**
		\brief Enable/disable volume conservation. Ineffective for cloth.

		Select volume conservation through PxDeformableDesc.volumeStiffness. 

		@see PxDeformableDesc.volumeStiffness
		*/
		eVOLUME_CONSERVATION			= (1<<4),

		/**
		\brief Enable/disable damping of internal velocities.

		Use PxDeformableDesc.dampingCoefficient to control damping.

		@see PxDeformableDesc.dampingCoefficient
		*/
		eDAMPING						= (1<<5),

		/**
		\brief Enable/disable deformable surface collision detection.

		If eSURFACE_COLLISION is not set, only collisions of deformable vertices are detected.
		If eSURFACE_COLLISION is set, the surface triangles of the deformable are used for collision detection.
		For deformables with tetrahedral meshes, the interior tetrahedron faces are not considered. 
		The flag does not have an impact on self collision behavior.
		*/
		eSURFACE_COLLISION				= (1<<6),

		/**
		\brief Defines whether the deformable is tearable. 

		Note: Make sure meshData.maxVertices and the corresponding buffers
		in meshData are substantially larger (e.g. 2x) than the number 
		of original vertices since tearing will generate new vertices.
		When the buffer cannot hold the new vertices anymore, tearing stops.

		Note: For tearing, make sure you cook the mesh with	the flag
		PxDeformableMeshFlag::eTEARABLE set in the PxDeformableMeshDesc.flags.

		@see PxDeformableDesc.tearFactor PxDeformableDesc.meshData PxDeformableMeshDesc.flags
		*/
		eTEARABLE						= (1<<7),

		/**
		\brief Enable/disable center of mass damping of internal velocities.

		This flag only has an effect if the flag eDAMPING is set. If set, 
		the global rigid body modes (translation and rotation) are extracted from damping. 
		This way, the deformable can freely move and rotate even under high damping. 
		Use PxDeformableDesc.dampingCoefficient to control damping. 

		@see PxDeformableDesc.dampingCoefficient
		*/
		eCOMDAMPING						= (1<<8),

		/**
		\brief If the flag eVALIDBOUNDS is set, deformable vertices outside the volume
		defined by PxDeformableDesc.validBounds are automatically removed from the simulation. 

		@see PxDeformableDesc.validBounds
		*/
		eVALIDBOUNDS					= (1<<9),

		/**
		\brief Enable/disable bulk data double buffering.

		Raising this flag will enable reading and writing vertex data while the simulation is running.

		\note This comes along with a considerable memory cost. All requested vertex read data buffers need to be duplicated.
		\note This flag can only be set in the particle system descriptor at creation time. Changing the flag later on is not supported.
		*/
		eBULK_DATA_DOUBLE_BUFFERING		= (1<<11),

		/**
		\brief Enable/disable two way interaction between deformables and rigid bodies.

		Raising this flag might have severe implications on performance.

		\note Raising this flag might have severe implications on performance.
		\note PxDeformableVertexAttachmentFlag::eTWOWAY will have no effect if not set.
		\note PxDominanceGroup PxActorDesc::dominanceGroup and PxActor::setDominanceGroup() will have no effect if not set. 
		*/
		eTWOWAY						= (1<<12),
	};
};

/*----------------------------------------------------------------------------*/

/**
\brief Deformable vertex attachment flags.

@see PxPhysics.createAttachment()
*/
struct PxDeformableVertexAttachmentFlag
{
	enum Enum
	{
		/**
		\brief The attachment interacts in both ways.
		*/
		eTWOWAY			= (1<<0),

		/**
		\brief The attachment is tearable.
		*/
		eTEARABLE		= (1<<1),
	};
};

/*----------------------------------------------------------------------------*/

/**
\brief Defines a set of per vertex coefficients.

The user can supply a buffer of positions and normals coming from an animation of the deformable. 
The simulated vertices are then corrected via the constrain positions, normals
and the constrain coefficiens.

@see PxDeformable.setConstrainCoefficients() PxDeformable.setConstrainPositions PxDeformable.setConstrainNormals
*/
class PxDeformableConstrainCoefficients
{
public:
	PxReal maxDistance;
	PxReal collisionSphereRadius;
	PxReal collisionSphereDistance;

	PX_INLINE PxDeformableConstrainCoefficients();

	/**
	\brief (Re)sets the structure to the default.	
	*/
	PX_INLINE void setToDefault();

	/**
	\brief Returns true if the current settings are valid
	*/
	PX_INLINE bool isValid() const;
};

PX_INLINE PxDeformableConstrainCoefficients::PxDeformableConstrainCoefficients():
maxDistance(0.0f),
collisionSphereRadius(0.0f),
collisionSphereDistance(0.0f)
{
}

PX_INLINE void PxDeformableConstrainCoefficients::setToDefault()
{
	*this = PxDeformableConstrainCoefficients();
}

PX_INLINE bool PxDeformableConstrainCoefficients::isValid() const
{
	if (maxDistance < 0.0f) return false;
	if (collisionSphereRadius < 0.0f) return false;
	return true;
}

/*----------------------------------------------------------------------------*/

/**
\brief Descriptor class for #PxDeformable.

@see PxDeformable PxDeformable.saveToDesc()
*/

class PxDeformableDesc : public PxActorDesc
{
public:
	/**
	\brief The cooked deformable mesh.

	<b>Default:</b> NULL

	@see PxDeformableMesh PxDeformableMeshDesc PxDeformable.getDeformableMesh()
	*/
	PxDeformableMesh* deformableMesh;

	/**
	\brief Flag bits.

	<b>Default:</b> PxDeformableFlag::eGRAVITY | PxDeformableFlag::eVOLUME_CONSERVATION

	@see PxDeformableFlag PxDeformable.setDeformableFlag()
	*/
	PxU32 deformableFlags;

	/**
	\brief Flags to control which data can be read from the SDK.

	@see PxDeformableReadDataFlag
	*/
	PxU32 deformableReadDataFlags;

	/**
	\brief The maximal number of vertices which can be stored in the vertex buffer.

	If the user has tearing enabled, the SDK might generate new vertices that will be appended
	at the end of the vertex buffers. The maxVertices parameter can be used to cap the total amount.

	In the worst case, a torn triangle mesh might increase the total number of vertices 6 times
	while a torn tetrahedral mesh can increase the total number of vertices 12 times.
	*/
	PxU32 maxVertices;

	/**
	\brief The global pose of the deformable in the world.

	<b>Default:</b> Identity Transform
	*/
	PxTransform globalPose;


	/**
	\brief Mass of the deformable.

	Should not be zero. If the user does not set the vertex masses of the deformable mesh directly, 
	this total mass is used to determine the masses of the individual deformable vertices.
	If the user does provide vertex masses himself, the SDK will instead use the sum of the user masses 
	internally (which is also the value returned by PxDeformable.getMass).

	\note PxDeformableExt provides helper functions	to compute masses out of deformable meshes given density values.

	<b>Range:</b> (0,inf)<br>
	<b>Default:</b> 0.0

	@see PxDeformable.getMass() PxDeformableExt.computeTriangleMeshMass PxDeformableExt.computeTetrahedronMeshMass
	*/
	PxReal mass;

	/**
	\brief Stretching stiffness of the deformable in the range 0 to 1.

	Defines how strongly the deformable counteracts deviations from the rest lengths of edges.

	Note: For soft bodies with very low stretching stiffnesses, tetrahedra edges will cease to stretch 
	further if their length exceeds a certain internal limit. This is done to prevent heavily degenerated 
	tetrahedral elements which could occur otherwise.

	<b>Default:</b> 1.0 <br>
	<b>Range:</b> (0,1]

	@see PxDeformable.setStretchingStiffness()
	*/
	PxReal stretchingStiffness;

	/**
	\brief Bending stiffness of the deformable in the range 0 to 1. Ineffective for soft bodies.

	Only has an effect if the flag PxDeformableFlag::eBENDING is set.

	<b>Default:</b> 1.0 <br>
	<b>Range:</b> [0,1]

	@see PxDeformableFlag::eBENDING PxDeformable.setBendingStiffness()
	*/
	PxReal bendingStiffness;

	/**
	\brief Volume stiffness of the deformable in the range 0 to 1. Ineffective for cloth.

	Defines how strongly the deformable counteracts deviations from the rest volume.
	Only has an effect if the flag PxDeformableFlag::eVOLUME_CONSERVATION is set.

	<b>Default:</b> 1.0 <br>
	<b>Range:</b> [0,1]

	@see PxDeformableFlag::eVOLUME_CONSERVATION PxDeformable.setVolumeStiffness()
	*/
	PxReal volumeStiffness;

	/**
	\brief Defines a factor for the impulse transfer between deformables colliding with rigid bodies.

	<b>Default:</b> 0.6 <br>
	<b>Range:</b> [0,1)

	@see PxDeformable.setCollisionStiffness()
	*/
	PxReal collisionStiffness;

	/**
	\brief Defines a factor for the impulse transfer between deformables attached to rigid bodies.

	Only has an effect if the mode of the attachment is set to PxDeformableAttachmentFlag::eTWOWAY.

	<b>Default:</b> 1.0 <br>
	<b>Range:</b> [0,1]

	@see PxDeformable.attachVertexToShape PxDeformable.setAttachmentStiffness()
	*/
	PxReal attachmentStiffness;

	/**
	\brief Spring damping of the deformable in the range 0 to 1.

	Only has an effect if the flag PxDeformableFlag::eDAMPING is set.

	<b>Default:</b> 0.5 <br>
	<b>Range:</b> [0,1]

	@see PxDeformableFlag::eDAMPING PxDeformable.setDampingCoefficient()
	*/
	PxReal dampingCoefficient;

	/**
	\brief Dynamic friction coefficient. Should be in the range [0, +inf].

	Dynamic friction must be less or equal than static friction.

	<b>Range:</b> [0,+inf]<br>
	<b>Default:</b> 0.2

	@see PxDeformable.setDynamicFriction()
	*/
	PxReal dynamicFriction;

	/**
	\brief Static friction coefficient. Should be in the range [0, +inf].

	Static friction must be greater or equal than dynamic friction.

	<b>Range:</b> [0,+inf]<br>
	<b>Default:</b> 0.2

	@see PxDeformable.setStaticFriction()
	*/
	PxReal staticFriction;

	/**
	\brief If the flag PxDeformableFlag::ePRESSURE is set, this variable defines the volume
	of air inside the mesh as volume = pressure * restVolume. Ineffective for soft bodies.

	For pressure < 1 the mesh contracts w.r.t. the rest shape
	For pressure > 1 the mesh expands w.r.t. the rest shape

	<b>Default:</b> 1.0 <br>
	<b>Range:</b> [0,inf)

	@see PxDeformableFlag::ePRESSURE PxDeformable.setPressure()
	*/
	PxReal pressure;

	/**
	\brief If the flag PxDeformableFlag::eTEARABLE is set, this variable defines the 
	elongation factor that causes the deformable to tear. 

	Must be larger than 1.
	Make sure meshData.maxVertices and the corresponding buffers
	in meshData are substantially larger (e.g. 2x) than the number 
	of original vertices since tearing will generate new vertices.

	When the buffer cannot hold the new vertices anymore, tearing stops.

	<b>Default:</b> 1.5 <br>
	<b>Range:</b> (1,inf)

	@see PxDeformable.setTearFactor()
	*/
	PxReal tearFactor;

	/**
	\brief Specifies a skin around the deformable within which contacts will be generated.

	Objects that come within contactOffset distance of the deformable will count as being in contact (the contactOffset
	of the other object has no influence). The contactOffset has to be greater than zero and also be greater than the
	deformable's restOffset. Having a contactOffset bigger than the restOffset is important to avoid jittering.

	<b>Range:</b> (0,inf)<br>
	<b>Default:</b> 0.02

	@see restOffset PxDeformable.setContactOffset() PxShape.setContactOffset() PxShape.getContactOffset()
	*/
	PxReal contactOffset;

	/**
	\brief Specifies an offset relative to the deformable's surface where other objects will come to rest.

	Deformables and rigid bodies will come to rest on top of each other at a distance equal to the restOffset of the deformable
	(the restOffset of the rigid body has no influence). If the restOffset is zero they should converge to touching exactly. Having a restOffset 
	greater than zero is useful to have	objects slide at a slight distance, to reduce the probability that the deformable falls through
	level geometry and to reduce clipping artifacts when rendering cloth.

	\note This parameter replaces the original thickness parameter in the 2.x releases.

	<b>Range:</b> [0,contactOffset]<br>
	<b>Default:</b> 0.01 

	@see contactOffset PxDeformable.setRestOffset() PxShape.setRestOffset() PxShape.getRestOffset()
	*/
	PxReal restOffset;

	/**
	\brief Special penetration offset useful to reduce artifacts caused by "sandwiched" deformables.

	Deformables, especially cloths, can sometimes get caught ("sandwiched") between other objects which tends to 
	result in jittering	artifacts since the solver cannot find a configuration of deformable vertices that 
	fulfills all constraints. 
	Deformable collision detection tries to detect and dissolve these problematic scenarios by temporarily allowing
	the deformable to penetrate the rigid body objects that have trapped it. The penetrationOffset can be used to
	tweak the amount of penetration that should be allowed for this purpose. If no penetration should ever be
	allowed, the parameter should be set to zero. Otherwise a negative value is expected.


	<b>Range:</b> (-inf,0]<br>
	<b>Default:</b> -0.02 

	@see restOffset PxDeformable.setPenetrationOffset()
	*/
	PxReal penetrationOffset;

	/**
	\brief Rest offset used for self collision.

	When self collision is turned on, two deformable vertices should come to rest at a 
	distance of twice the selfCollisionOffset. In other words, deformable vertices can be 
	thought of as having a radius of selfCollisionOffset while self collision is performed.

	<b>Range:</b> (0,inf]<br>
	<b>Default:</b> 0.02

	@see selfCollisionOffset PxDeformable.setSelfCollisionOffset()
	*/
	PxReal selfCollisionOffset;

	/**
	\brief If the flag PxDeformableAttachmentFlag::eTEARABLE is set in the attachment method of PxDeformable, 
	this variable defines the elongation factor that causes the attachment to tear. 

	Must be larger than 1.

	<b>Default:</b> 1.5 <br>
	<b>Range:</b> (1,inf)

	@see PxDeformable.setAttachmentTearFactor() PxDeformable.attachOverlapToShape PxDeformable.attachToCollidingShapes PxDeformable.attachVertexToShape

	*/
	PxReal attachmentTearFactor;

	/**
	\brief Number of solver iterations. 

	Note: Small numbers make the simulation faster while the deformable gets less stiff.

	<b>Default:</b> 5
	<b>Range:</b> [1,inf)

	@see PxDeformable.setSolverIterations()
	*/
	PxU32 solverIterations;

	/**
	\brief Number of iterations of the hierarchical deformable solver.

	For the this value to have an effect, the parameter PxDeformableMeshDesc.maxHierarchyLevels
	must be greater then one when cooking the deformable mesh. In this case, the hierarchical
	deformable solver uses the mesh hierarchy to speed up convergence, i.e. makes large
	pieces of deformable less stretchy.

	<b>Default:</b> 2
	<b>Range:</b> [0,inf)

	@see PxDeformable.setHierarchicalSolverIterations() PxDeformable.setHierarchicalSolverIterations PxDeformableMeshDesc.maxHierarchyLevels
	*/
	PxU32 hierarchicalSolverIterations;

	/**
	\brief External acceleration which affects all non attached vertices of the deformable. 

	<b>Default:</b> (0,0,0)

	@see PxDeformable.setExternalAcceleration()
	*/
	PxVec3 externalAcceleration;

	/**
	\brief Acceleration which acts normal to the cloth surface at each vertex. Ineffective for soft bodies.

	<b>Default:</b> (0,0,0)

	@see PxDeformable.setWindAcceleration()
	*/
	PxVec3 windAcceleration;

	/**
	\brief The deformable wake up counter.

	<b>Range:</b> [0,inf)<br>
	<b>Default:</b> 20.0f*0.02f

	@see PxDeformable.wakeUp() PxDeformable.putToSleep()
	*/
	PxReal wakeUpCounter;

	/**
	\brief Maximum linear velocity at which deformable can go to sleep.

	<b>Range:</b> [0,inf)<br>

	@see PxDeformable.setSleepLinearVelocity() PxDeformable.getSleepLinearVelocity()
	*/
	PxReal sleepLinearVelocity;

	/**
	\brief If the flag PxDeformableFlag::eVALIDBOUNDS is set, this variable defines the volume
	outside of which deformable vertices are automatically removed from the simulation. 

	@see PxDeformableFlag::eVALIDBOUNDS PxDeformable.setValidBounds()
	*/
	PxBounds3 validBounds;

	/**
	\brief This parameter defines the size of grid cells for collision detection.

	The deformable is represented by a set of world aligned cubical cells in broad phase.
	The size of these cells is determined by multiplying the length of the diagonal
	of the AABB of the initial deformable size with this constant.

	<b>Range:</b> [0.01,inf)<br>
	<b>Default:</b> 0.25
	*/
	PxReal relativeGridSpacing;

	/**
	\brief User definable data for collision filtering.

	@see PxDeformable.setSimulationFilterData(), PxDeformable.getSimulationFilterData()
	*/
	PxFilterData simulationFilterData;

	/**
	\brief Maximum number of primitive split pairs reported for torn deformables.

	@see PxDeformableReadData.primitiveSplitPairBuffer
	*/
	PxU32 maxPrimitiveSplitPairs;

	/**
	\brief Constructor sets to default.
	*/
	PX_INLINE PxDeformableDesc(const PxTolerancesScale& scale);

	/**
	\brief (Re)sets the structure to the default.	
	*/
	PX_INLINE void setToDefault(const PxTolerancesScale& scale);

	/**
	\brief Returns true if the current settings are valid
	*/
	PX_INLINE bool isValid() const;

	/**
	\brief virtual destructor
	*/
	virtual ~PxDeformableDesc() {};

private:

	// for internal use only.
	PX_INLINE void setDeformableDescToDefault();
};

/*----------------------------------------------------------------------------*/

PX_INLINE PxDeformableDesc::PxDeformableDesc(const PxTolerancesScale& /* scale */) :
	PxActorDesc(PxActorType::eDEFORMABLE),
	validBounds(PxVec3(-PX_MAX_F32), PxVec3(PX_MAX_F32))
{
	deformableMesh = NULL;
	globalPose = PxTransform::createIdentity();	
	mass = 0.0f;	
	bendingStiffness = 1.0f;
	volumeStiffness = 1.0f;
	stretchingStiffness = 1.0f;
	dampingCoefficient = 0.0f;
	staticFriction = 0.2f;
	dynamicFriction = 0.2f;
	pressure = 1.0f;
	tearFactor = 1.5f;
	attachmentTearFactor = 1.5f;
	attachmentStiffness = 1.0f;
	collisionStiffness = 0.6f;
	restOffset = 0.01f;
	contactOffset = 0.02f;
	penetrationOffset = -0.05f;
	selfCollisionOffset = 0.02f;
	deformableFlags = PxDeformableFlag::eVOLUME_CONSERVATION;
	deformableReadDataFlags	= PxDeformableReadDataFlag::ePOSITION_BUFFER | PxDeformableReadDataFlag::eNORMAL_BUFFER;
	solverIterations = 5;
	hierarchicalSolverIterations = 2;
	wakeUpCounter = PX_SLEEP_INTERVAL;
	sleepLinearVelocity = -1.0f;
	externalAcceleration = PxVec3(0.0f, 0.0f, 0.0f);
	windAcceleration = PxVec3(0.0f, 0.0f, 0.0f);
	relativeGridSpacing = 0.4f;
	simulationFilterData.setToDefault();
	maxPrimitiveSplitPairs = 0;
	sleepLinearVelocity		= 0.15f;

}

/*----------------------------------------------------------------------------*/

PX_INLINE void PxDeformableDesc::setToDefault(const PxTolerancesScale& scale)
{
	*this = PxDeformableDesc(scale);
}

/*----------------------------------------------------------------------------*/

PX_INLINE bool PxDeformableDesc::isValid() const
{
	if(!deformableMesh) return false;
	if(!globalPose.isFinite()) return false;
	if(mass <= 0.0f) return false;
	if(bendingStiffness < 0.0f || bendingStiffness > 1.0f) return false;
	if(volumeStiffness < 0.0f || volumeStiffness > 1.0f) return false;
	if(stretchingStiffness <= 0.0f || stretchingStiffness > 1.0f) return false;
	if(pressure < 0.0f) return false;
	if(tearFactor <= 1.0f) return false;
	if(attachmentTearFactor <= 1.0f) return false;
	if(solverIterations < 1) return false;
	if(staticFriction < 0.0f) return false;
	if(dynamicFriction < 0.0f) return false;
	if(staticFriction < dynamicFriction) return false;
	if(dampingCoefficient < 0.0f || dampingCoefficient > 1.0f) return false;
	if(collisionStiffness < 0.0f || collisionStiffness > 1.0f) return false;
	if(wakeUpCounter < 0.0f) return false;
	if(attachmentStiffness < 0.0f || attachmentStiffness > 1.0f) return false;
	if(relativeGridSpacing < 0.01f) return false;
	if(restOffset < 0.0f) return false;
	if(contactOffset < restOffset) return false;
	if(penetrationOffset > 0.0f) return false;
	if(selfCollisionOffset <= 0.0f) return false;
	if(validBounds.isEmpty()) return false;

	return PxActorDesc::isValid();
}

/*----------------------------------------------------------------------------*/

#ifndef PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif

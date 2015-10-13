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


#ifndef PX_PHYSICS_PX_PARTICLEBASEDESC
#define PX_PHYSICS_PX_PARTICLEBASEDESC
/** \addtogroup particles
@{
*/

#include "PxActorDesc.h"
#include "PxSceneDesc.h"
#include "PxFiltering.h"
#include "particles/PxParticleReadData.h"
#include "PxFlags.h"
#include "particles/PxParticleBaseFlag.h"

#ifndef PX_DOXYGEN
namespace physx
{
#endif

/**
\brief collection of set bits defined in PxParticleBaseFlag.

@see PxParticleBaseFlag
*/
typedef PxFlags<PxParticleBaseFlag::Enum,PxU16> PxParticleBaseFlags;
PX_FLAGS_OPERATORS(PxParticleBaseFlag::Enum,PxU16);


/**
\brief ParticleBase Descriptor. This structure is used to save and load the state of #PxParticleSystem and #PxParticleFluid objects.
*/

class PxParticleBaseDesc : public PxActorDesc
{
public:

	/**
	\brief Sets the maximal number of particles for the particle system used in the simulation.

	If more particles are added after this limit is reached, they are simply ignored.
	
	@see PxParticleBase.getMaxParticles()
	*/
	PxU32 maxParticles;

	/**
	\brief Maximal distance a particle is allowed to travel within one time step.

	Default value is 0.06.

	The maxMotionDistance needs to be positive and the sum maxMotionDistance + contactOffset must not be higher than gridSize.
	
	\note This value should be chosen as small as possible due to potential negative impact on performance. 

	@see contactOffset PxParticleBase.getMaxMotionDistance()
	*/
	PxReal maxMotionDistance;

	/**
	\brief Specifies a skin around the particles within which contacts will be generated.

	Objects that come within contactOffset distance of the particles will count as being in contact (the contactOffset
	of the other object has no influence). The contactOffset has to be greater than zero and also be greater than the
	particle systems restOffset. Having a contactOffset bigger than the restOffset is important to avoid jittering and sticking.
	The contactOffset needs to be positive and the sum maxMotionDistance + contactOffset must not be higher than gridSize.

	<b>Range:</b> (0,gridSize - maxMotionDistance)<br>
	<b>Default:</b> 0.008

	@see restOffset PxParticleBase.getContactOffset() PxShape.setContactOffset() PxShape.getContactOffset()
	*/
	PxReal contactOffset;

	/**
	\brief Specifies an offset at which particles will come to rest relative to an objects surface.

	Particles will maintain a distance equal to the restOffset to rigid bodies (the restOffset of the rigid body has no influence). 
	If the restOffset is zero the particles should rest exactly on the object surface. 

	The restOffset can alternatively be specified per particle. However the per particle restOffset needs to be smaller or equal 
	to the PxParticleBaseDesc.restOffset.

	\note This parameter replaces the original collisionDistanceMultiplier parameter in the 2.x releases.

	<b>Range:</b> [0,contactOffset]<br>
	<b>Default:</b> 0.004 

	@see contactOffset thickness PxParticleBase.getRestOffset() PxShape.setRestOffset() PxShape.getRestOffset() PxParticleBaseFlag.ePER_PARTICLE_REST_OFFSET
	*/
	PxReal restOffset;

	/**
	\brief This parameter controls the parallelization of the particle system.

	The spatial domain is divided into equal sized cubes, aligned in a grid.

	The parameter given defines the scale of the grid. The sdk may internally choose
	a different, larger value which can be queried with PxParticleBase.getGridSize(). Therefore the parameter 
	has to be considered as a hint only. 
	
	\note Large values will have a negative effect on performance, while too small values are problematic
	due to spatial data structure buffer limits. Balancing this value is important to achieve a good effect
	while maintaining good performance.
	
	Default value is 0.6.
	@see PxParticleFlag.eSPATIAL_DATA_STRUCTURE_OVERFLOW PxParticleBase.getGridSize()
	
	*/
	PxReal gridSize;

	/**
	\brief Velocity damping constant, which is globally applied to each particle.

	It generally reduces the velocity of the particles. Setting the damping to 0 will leave the 
	particles unaffected.

	Must be nonnegative.
	
	@see PxParticleBase.getDamping() PxParticleBase.setDamping()
	*/
	PxReal damping;

	/**
	\brief Acceleration (m/s^2) applied to all particles at all time steps.

	Useful to simulate smoke or fire.
	This acceleration is additive to the scene gravity. The scene gravity can be turned off 
	for the particle system, using the flag PxActorFlag::eDISABLE_GRAVITY.

	@see PxParticleBase.getExternalAcceleration() PxParticleBase.setExternalAcceleration()
	*/
	PxVec3 externalAcceleration;

	/**
	\brief Defines the normal of the plane the particles are projected to. This parameter is only used if
	PxParticleBaseFlag::ePROJECT_TO_PLANE is set.

	Together with the parameter #projectionPlaneDistance a plane is formed. For each point p on the
	plane the following equation has to hold:
	
	(projectionPlaneNormal.x * p.x)  +  (projectionPlaneNormal.y * p.y)  +  (projectionPlaneNormal.z * p.z)  +  projectionPlaneDistance = 0

	<b>Default:</b> (0,0,1) (XY plane in combination with projectionPlaneDistance)

	@see PxParticleBaseFlag::ePROJECT_TO_PLANE PxParticleBase.getProjectionPlane() PxParticleBase.setProjectionPlane()
	*/
	PxVec3 projectionPlaneNormal;

	/**
	\brief Defines the constant term of the plane the particles are projected to. This parameter is only used if
	PxParticleBaseFlag::ePROJECT_TO_PLANE is set.

	Together with the parameter #projectionPlaneNormal a plane is formed. For each point p on the
	plane the following equation has to hold:
	
	(projectionPlaneNormal.x * p.x)  +  (projectionPlaneNormal.y * p.y)  +  (projectionPlaneNormal.z * p.z)  +  projectionPlaneDistance = 0

	<b>Default:</b> 0 (XY plane)

	@see PxParticleBaseFlag::ePROJECT_TO_PLANE PxParticleBase.getProjectionPlane() PxParticleBase.setProjectionPlane()
	*/
	PxReal projectionPlaneDistance;

	/**
	\brief Defines the mass of a particle. 

	The mass is used to translate force or impulses to velocities for collisions 
	(in case PxParticleBaseFlag::eCOLLISION_TWOWAY is set) or for particle updates.

	<b>Default:</b> 0.001 <br>
	<b>Range:</b> [0,inf)

	@see PxParticleBaseFlag::eCOLLISION_TWOWAY PxParticleBase.setParticleMass()
	*/
	PxReal particleMass;

	/**
	\brief Defines the restitution coefficient used for collisions of the particles with shapes.

	Must be between 0 and 1.

	A value of 0 causes the colliding particle to get a zero velocity component in the
	direction of the surface normal of the shape at the collision location; i.e.
	it will not bounce.

	A value of 1 causes a particle's velocity component in the direction of the surface normal to invert;
	i.e. the particle bounces off the surface with the same velocity magnitude as it had before collision. 
	(Caution: values near 1 may have a negative impact on stability)

	@see PxParticleBase.setRestitution() PxParticleBase.getRestitution()
	*/
	PxReal restitution;

	/**
	\brief Defines the dynamic friction of the particles regarding the surface of a shape.

	Must be between 0 and 1.

	A value of 1 will cause the particle to lose its velocity tangential to
	the surface normal of the shape at the collision location; i.e. it will not slide
	along the surface.

	A value of 0 will preserve the particle's velocity in the tangential surface
	direction; i.e. it will slide without resistance on the surface.

	@see PxParticleBase.setDynamicFriction() PxParticleBase.getDynamicFriction()
	*/
	PxReal dynamicFriction;

	/**
	\brief Defines the static friction of the particles regarding the surface of a shape.

	The value defines a limit at which a particle starts to slide along a surface depending on
	relative tangential and normal velocity components of the particle.  

	A value of 0 will turn off static friction.
	
	@see PxParticleBase.setStaticFriction() PxParticleBase.getStaticFriction()
	*/
	PxReal staticFriction;

	/**
	\brief User definable data for collision filtering.

	@see PxParticleBase.setSimulationFilterData(), PxParticleBase.getSimulationFilterData()
	*/
	PxFilterData simulationFilterData;

	/**
	\brief Flags defining certain properties of the particle system.

	@see PxParticleBaseFlag PxParticleBaseFlags PxParticleBase.setParticleBaseFlag() PxParticleBase.getParticleBaseFlags()
	*/
	PxParticleBaseFlags particleBaseFlags;

	/**
	\brief Flags to control which data can be read from the SDK.
	Default: PxParticleReadDataFlag::ePOSITION_BUFFER | PxParticleReadDataFlag::eID_BUFFER | PxParticleReadDataFlag::eFLAGS_BUFFER;
	
	@see PxParticleReadDataFlag PxParticleReadDataFlags
	*/
	PxParticleReadDataFlags particleReadDataFlags;

	/**
	\brief (Re)sets the structure to the default.	

	*/
	PX_INLINE void setToDefault(const PxTolerancesScale& scale, PxActorType::Enum t);
	
	/**
	\brief Returns true if the current settings are valid

	*/
	PX_INLINE bool isValid() const;
	
	/**
	\brief virtual destructor
	*/
	virtual ~PxParticleBaseDesc() {};

protected:

	/**
	\brief Constructor sets to default.

	*/
	PX_INLINE PxParticleBaseDesc(const PxTolerancesScale& scale, PxActorType::Enum t);
	
	
	/**
	\brief Copy constructor.
	*/
	PX_INLINE PxParticleBaseDesc(const PxParticleBaseDesc& desc);
	
};

PX_INLINE PxParticleBaseDesc::PxParticleBaseDesc(const PxTolerancesScale& /* scale */, PxActorType::Enum t) : PxActorDesc(t)
{
	maxParticles				= 32767;
	maxMotionDistance			= 0.06f;
	contactOffset				= 0.008f;
	restOffset					= 0.004f;
	gridSize					= 0.6f;
	particleMass				= 0.001f;
	damping						= 0.0f;
	externalAcceleration		= PxVec3(0);
	projectionPlaneNormal		= PxVec3(0.0f, 0.0f, 1.0f);
	projectionPlaneDistance		= 0.0f;
	restitution					= 0.5f;
	dynamicFriction				= 0.05f;
	staticFriction				= 0.0f;

	simulationFilterData		.setToDefault();

	particleBaseFlags			= PxParticleBaseFlag::eENABLED | 
								  PxParticleBaseFlag::eCOLLISION_WITH_DYNAMIC_ACTORS | 
								  PxParticleBaseFlag::ePER_PARTICLE_COLLISION_CACHE_HINT;
	
	particleReadDataFlags		= PxParticleReadDataFlag::ePOSITION_BUFFER | PxParticleReadDataFlag::eFLAGS_BUFFER;
}

PX_INLINE PxParticleBaseDesc::PxParticleBaseDesc(const PxParticleBaseDesc& desc) : PxActorDesc(desc.type)
{
	*this = desc;
	type = desc.type;
}

PX_INLINE void PxParticleBaseDesc::setToDefault(const PxTolerancesScale& scale, PxActorType::Enum t)
{
	*this = PxParticleBaseDesc(scale, t);
}

PX_INLINE bool PxParticleBaseDesc::isValid() const
{
	if (gridSize <= 0.0f) return false;
	if (maxMotionDistance <= 0.0f) return false;
	if (maxMotionDistance + contactOffset > gridSize) return false;
	if (contactOffset < 0.0f) return false;
	if (contactOffset < restOffset) return false;
	if (particleMass < 0.0f) return false;
	if (damping < 0.0f) return false;
	if (projectionPlaneNormal.isZero()) return false;
	if (restitution < 0.0f || restitution > 1.0f) return false;
	if (dynamicFriction < 0.0f || dynamicFriction > 1.0f) return false;
	if (staticFriction < 0.0f) return false;
	if (maxParticles < 1) return false;

	return PxActorDesc::isValid();
}

#ifndef PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif

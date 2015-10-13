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


#ifndef PX_PHYSICS_PX_PARTICLEFLUIDDESC
#define PX_PHYSICS_PX_PARTICLEFLUIDDESC
/** \addtogroup particles
@{
*/

#include "particles/PxParticleBaseDesc.h"
#include "particles/PxParticleFluidReadData.h"

#ifndef PX_DOXYGEN
namespace physx
{
#endif

/**
\brief Particle Fluid Descriptor. This structure is used to save and load the state of #PxParticleFluid objects.
*/

class PxParticleFluidDesc : public PxParticleBaseDesc
{
public:

	/**
	\brief The typical particle distance of particles in the rest state (relaxed).
	Defines the particle resolution of the fluid. 
	
	\note This property can't be changed after the particle fluid has been created.

	Must be positive.
	Default value is 0.02.
	
	@see PxParticleFluid.getRestParticleDistance()
	*/
	PxReal						restParticleDistance;

	/**
	\brief The stiffness of the particle interaction related to the pressure.

	This factor linearly scales the force which acts on particles which are closer to each other than 
	the rest spacing.

	Setting	this parameter appropriately is crucial for the simulation.  The right value depends on many factors
	such as viscosity and damping.  Values which are too high will result in an
	unstable simulation, whereas too low values will make the fluid appear "springy" (the fluid
	acts more compressible).

	Must be positive.
	Default value is 20.0.

	@see PxParticleFluid.setStiffness() PxParticleFluid.getStiffness()
	*/
	PxReal						stiffness;

	/**
	\brief 	The viscosity of the fluid defines its viscous behavior.

	Higher values will result in a honey-like behavior.  Viscosity is an effect which depends on the 
	relative velocity of neighboring particles; it reduces the magnitude of the relative velocity.

	Must be positive.
	Default value is 6.0.
	
	@see PxParticleFluid.setViscosity() PxParticleFluid.getViscosity()
	*/
	PxReal						viscosity;

	/**
	\brief Constructor sets to default.
	*/
	PX_INLINE PxParticleFluidDesc(const PxTolerancesScale& scale);

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
	virtual ~PxParticleFluidDesc() {};
};




PX_INLINE PxParticleFluidDesc::PxParticleFluidDesc(const PxTolerancesScale& scale) : 
  PxParticleBaseDesc(scale, PxActorType::ePARTICLE_FLUID)
{
	restParticleDistance		= 0.02f;
	stiffness					= 20.0f;
	viscosity					= 6.0f;
}

PX_INLINE void PxParticleFluidDesc::setToDefault(const PxTolerancesScale& scale)
{
	*this = PxParticleFluidDesc(scale);
}

PX_INLINE bool PxParticleFluidDesc::isValid() const
{
	if (restParticleDistance <= 0.0f) return false;

	if (stiffness <= 0.0f) return false;
	if (viscosity <= 0.0f) return false;

	return PxParticleBaseDesc::isValid();
}

#ifndef PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif

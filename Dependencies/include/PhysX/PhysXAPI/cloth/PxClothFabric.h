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


#ifndef PX_PHYSICS_NX_CLOTH_FABRIC
#define PX_PHYSICS_NX_CLOTH_FABRIC
/** \addtogroup cloth
  @{
*/

// PX_SERIALIZATION
#include "common/PxSerialFramework.h"
//~PX_SERIALIZATION

#include "cloth/PxClothFabricTypes.h"

#ifndef PX_DOXYGEN
namespace physx
{
#endif

/**
   \brief A cloth fabric is a structure that contains all the internal solver constraints of a clothing mesh.

   A fabric consists of phases that represent independent sets of fibers and each fiber represents a set of connected internal constraints.

   The data representation for the fabric has layers of indirect indices.

   Each fiber is represented by [ particleIndex0, particleIndex1, particleIndex2, ... ], \n
   and a set of fibers are stored in the fiber array, where the last index (of the particle index table) for each fiber is stored. \n
   Similarly, index of last fiber for each phase is stored in the phase array. \n

   Each phase is associated with a type (use PxClothFabric.getPhaseType() ), representing the type of constraints we apply in the solver.

   \see The fabric structure should be cooked into a PxStream using PxCooking.cookClothFabric().
   \see Instances of PxClothFabric can be created using PxPhysics.createClothFabric() using the PxStream created from the cooker.
   \see Use PxClothFabric.getPhases(), PxClothFabric.getFibers(), PxClothFabric.getParticleIndices() and PxClothFabric.getRestlengths() to retrieve the fabric data.
*/
class PxClothFabric
// PX_SERIALIZATION
	: public PxSerializable
//~PX_SERIALIZATION
{
public:
	/**
	\brief Release the cloth fabric.

	Releases the application's reference to the cloth fabric.
	The fabric is destroyed when the application's reference is released and all cloth instances referencing the fabric are destroyed.

	\see PxPhysics.createClothFabric()
	*/
	virtual void release() = 0;

	/**
       \brief Returns number of particles
       
       \return the number of particles needed when creating a PxCloth instance from the fabric
    */ 
	virtual	PxU32 getNbParticles() const = 0;

    /**
       \brief Returns number of phase data elements
       
       \return the size of phase array (number of phases)
    */ 
	virtual	PxU32 getNbPhases() const = 0;

    /**
       \brief Returns number of fiber data elements
       
       \return the size of fiber array (number of fibers)
    */
	virtual	PxU32 getNbFibers() const = 0;

    /**
       \brief get total number of particle (vertex) indices in the fiber data.
     */
	virtual	PxU32 getNbParticleIndices() const = 0;

    /**
       \brief This function copies the phase array to a user specified buffer.
       
       The size of this array is the same as number of phases (PxClothFabric.getNbPhases()).
       
       One can iterate over fibers that belong to i-th phase ( 0 <= i < N) by searching through the interval 
       
       [ Phases[i-1], Phases[i] )
       
       For the first phase (i = 0), the interval becomes [0, Phase[0] ).
       
       \note The interval is not inclusive at the end. Each element represents one index past last fiber in the fiber array.
       
       \param[in] userPhaseBuffer a pre-allocated buffer to copy the phase data to. 
       \param[in] bufferSize number of indices the provided buffer can store
       \return 0 if the bufferSize is not large enough, or the number of written indices if copy was successful
       
       \note The size of userPhaseBuffer should be large enough, otherwise this function will return 0 and do nothing.\n
       The required buffer size is getNbPhases().
       
       \see PxClothFabric.getFibers() and PxClothFabric.getNbFibers() to retrieve the fiber array.
    */    
    virtual PxU32 getPhases(PxU32* userPhaseBuffer, PxU32 bufferSize) const = 0;

    /**
       This function copies the fiber array to a user specified buffer.
       
       The size of this array is the same as number of fibers (PxClothFabric.getNbFibers()).
       
       One can iterate over particles that belong to i-th fiber ( 0 <= i < N) by searching through the interval 
       
       [ Fibers[i-1], Fibers[i] )
       
       For the first fiber (i = 0), the interval becomes [0, Fibers[0] ).
       
       \note The interval is not inclusive at the end. Each element represents one index past last particle in the particle index array.
       
       \param[in] userFiberBuffer a pre-allocated buffer to copy the fiber data to. 
       \param[in] bufferSize number of indices the provided buffer can store
       \return 0 if the bufferSize is not large enough, or the number of written indices if copy was successful
       
       \note The size of userFiberBuffer should be large enough, otherwise this function will return 0 and do nothing.\n
       The required buffer size is getNbFibers().
       
       \see PxClothFabric.getParticleIndices() and PxClothFabric.getNbParticleIndices() to retrieve the particle index array.
    */    
    virtual PxU32 getFibers(PxU32* userFiberBuffer, PxU32 bufferSize) const = 0;

    /**
       This function copies the particle index array to a user specified buffer.
       
       For each fiber, we store particle indices sequentially, as follows.
       
       [fiber0ParticleIdx0, fiber0ParticleIdx1, ..., fiber1ParticleIdx0, fiber1ParticleIdx1, ...]
       
       \see getPhases() and getFibers() functions to get fiber and phase index onto this array
       
       \param[in] userParticleIndexBuffer a pre-allocated buffer to copy the index data to.
       \param[in] bufferSize number of indices the provided buffer can store
       
       \return 0 if the bufferSize is not large enough or the number of written indices if copy was successful
       
       \note The size of userParticleIndexBuffer should be large enough, otherwise this function will return 0 and do nothing.\n
       The required buffer size is getNbParticleIndices().
       
       \see PxClothFabric.getParticleIndices() and PxClothFabric.getNbParticleIndices() to retrieve the particle index array.
    */
    virtual PxU32 getParticleIndices(PxU32* userParticleIndexBuffer, PxU32 bufferSize) const = 0;

    /**
       \brief fills in rest length data for each fiber element.
       
       \param[in] userRestlengthBuffer output buffer to store restlength data
       \param[in] bufferSize number of restlength entries the provided buffer can store 
       
       \return 0 if the bufferSize is not large enough, or the number of written values if copy was successful

       \note If bufferSize is smaller than necessary, this function returns 0 and does not
       fill the data bufer.

       \note The required buffer size is ( getNbParticleIndices() - getNbFibers() )
    */
	virtual PxU32 getRestlengths(PxReal* userRestlengthBuffer, PxU32 bufferSize) const = 0;

    /**
       get phase type for specified phase.
       
       \return The phase type as PxClothFabricPhaseType::Enum\n
       If phase index is invalid, PxClothFabricPhaseType::eINVALID is returned.
    */
	virtual	PxClothFabricPhaseType::Enum getPhaseType(PxU32 phaseIndex) const = 0;

    /**
       \brief scale the rest length of fiber element by given value.

	   \note Only call this function when no PxCloth instance has been created for this fabric yet.
       
       \param[in] scale The scale factor (each rest length is multiplied by this value)
    */
	virtual	void scaleRestlengths(PxReal scale) = 0;

	/**
	\brief Reference count of the cloth instance
	
	At creation, the reference count of the fabric is 1. Every cloth instance referencing this fabric increments the
	count by 1.	When the reference count reaches 0, and only then, the fabric gets released automatically.

	\see PxCloth
	*/
	virtual	PxU32 getReferenceCount() const = 0;

protected:
	// PX_SERIALIZATION
	PxClothFabric()										{}
	PxClothFabric(PxRefResolver& v)	: PxSerializable(v)	{}
	PX_DECLARE_SERIAL_RTTI(PxClothFabric, PxSerializable)
	//~PX_SERIALIZATION
	virtual						~PxClothFabric() {}
};

#ifndef PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif

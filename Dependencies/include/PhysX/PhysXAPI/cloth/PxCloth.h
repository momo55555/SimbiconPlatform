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


#ifndef PX_PHYSICS_NX_CLOTH
#define PX_PHYSICS_NX_CLOTH
/** \addtogroup cloth
  @{
*/

#include "PxPhysX.h"
#include "PxActor.h"
#include "cloth/PxClothFabricTypes.h"
#include "cloth/PxClothTypes.h"
#include "cloth/PxClothCollisionData.h"

#ifndef PX_DOXYGEN
namespace physx
{
#endif

class PxScene;

//!!!CL document
class PxCloth : public PxActor
{
protected:
// PX_SERIALIZATION
	PxCloth(PxRefResolver& v) : PxActor(v)		{}
//~PX_SERIALIZATION
	PX_INLINE PxCloth() : PxActor() {}
	virtual ~PxCloth() {}

public:
	virtual void release() = 0;

	/**
	\brief Returns a pointer to the corresponding cloth fabric.

	\return The cloth fabric associated with this cloth.
	*/
	virtual	PxClothFabric* getFabric() const = 0;

    /**
       \brief Updates cloth particle location or inverse weight 
       \note use this function if you want to update constraint by changing the invWeight or to update the constrained position.
       \see PxClothParticle
     */
	virtual void setParticles(const PxClothParticle* particles) = 0;
    /**
       \brief Returns number of particles in a cloth instance
       \return the number of particles
     */
	virtual PxU32 getNbParticles() const = 0;

    /**
       \brief Updates motion constraints (position or radius of the constraint sphere)
     */
	virtual void setMotionConstraints(const PxClothParticleMotionConstraint* motionConstraints) = 0;
    /**
       \brief Copies current motion constraints to the user provied buffer
       \see getNbMotionConstraints() to determine the size of <b>motionConstraintBuffer</b>
       \return true if the copy was successful
     */
	virtual bool getMotionConstraints(PxClothParticleMotionConstraint* motionConstraintBuffer) const = 0;
    /**
       \brief Returns number of motion constraints
       \return number of motion constraints
     */
	virtual PxU32 getNbMotionConstraints() const = 0; 

    /**
       \brief Specifies motion constraint scane and bias
       
       The <b> scale </b> factor is a global scalar value multiplied to radius component of every motion constraint sphere (default = 1.0). 

       The <b> bias </b> factor is additional value added to the radius, to guarantee minimum radius for motion constraint spheres (default = 0). 
     */
	virtual void setMotionConstraintScaleBias(PxReal scale, PxReal bias) = 0;
    /**
       \brief Reads back scale and bias factor for motion constraints.

       The <b> scale </b> factor is a global scalar value multiplied to radius component of every motion constraint sphere (default = 1.0).

       The <b> bias </b> factor is additional value added to the radius, to guarantee minimum radius for motion constraint spheres (default = 0.0).
     */
	virtual void getMotionConstraintScaleBias(PxReal& scale, PxReal& bias) const = 0;

    /**
      \brief Updates location or radius of collision spheres used for this cloth
      \note You cannot change number of spheres in this function (see PxPhysics.createCloth() )
     */
	virtual void setCollisionSpheres(const PxClothCollisionSphere* sphereBuffer) = 0;
    /**
      \brief Copies the collision data (spheres and capsule indices) to user provided buffers.

      This function copies collision data to user provided buffers.

      Collisions in PxCloth are specified by spheres and capsule indices to the spheres.\n

      The <b>sphereBuffer</b> stores all the spheres used for collision purposes.\n

      A pair of indices to the sphere array represents a collision capsule that connects the two spheres.\n
      The size of <b>pairIndexBuffer</b> should be thus always even as it stores an array of such index pairs.

      \note Each buffer's size should large enough (see getNbCollisionSpheres() and getNbCollisionSpherePairs())
     */
	virtual void getCollisionData(PxClothCollisionSphere* sphereBuffer, PxU32* pairIndexBuffer) const = 0;
    /**
       \brief Returns the number of collision spheres
     */
	virtual PxU32 getNbCollisionSpheres() const = 0;
    /**
       \brief Returns the number of collision capsule index pairs
     */
	virtual PxU32 getNbCollisionSpherePairs() const = 0;

    /**
       \brief Assigns virtual particle sample data to this cloth

       Virtual particles (VP) provide more robust and accurate collision handling against collision spheres and capsules.\n
       More VP samples will generallly increase the accuracy of collision handling, and thus\n
       a sufficient number of virtual particles can mimic triangle-based collision handling.

       VPs can be put anywhere on the cloth mesh, by the following barycentric interpolation formula.

       Position of a virtual particle = w0 * P0 + w1 * P1 + w2 * P2, where P1,P2,P3 are positions of the vertices of a triangle.

       To create virtual particles, one should set the following variables.
       
       \param[in] numParticles total number of virtual particles.
       \param[in] triangleVertexAndWeightIndices Each virtual particle has four indices, the first three for triangle vertex index, and the last
       for index into the weight table.  Thus, the size of <b>triangleVertexAndWeightIndices</b> array should be 4 times numParticles.
       \param[in] weightTableSize total number of unique weights.
       \param[in] triangleVertexWeightTable array for barycentric weights.

       \note The number of VP samples need not be the same for each triangle. \n
       One can concentrate a large number of virtual particles for an area where high collision accuracy is desired.\n
       (e.g. sleeve around very thin wrist).\n

       \note It is, in general, efficient to share a small number of barycentric weights.
     */
	virtual void setVirtualParticles(PxU32 numParticles, const PxU32* triangleVertexAndWeightIndices, PxU32 weightTableSize, const PxVec3* triangleVertexWeightTable) = 0;
    /**
      \brief Returns number of virtual particles
     */
	virtual PxU32 getNbVirtualParticles() const = 0;
    /**
      \brief Copies index array for the virtual particles to user provided array
      
      \note The size of userTriangleVertexAndWeightIndices should be at least 4 times getNbVirtualParticles().\n
      (see setVirtualParticles() for more detailed description of the data)
     */
	virtual void getVirtualParticles(PxU32* userTriangleVertexAndWeightIndices) const = 0;
    /**
       \brief Returns number of virtual particle weight samples
     */
	virtual PxU32 getNbVirtualParticleWeights() const = 0;
    /**
       \brief Copies weight table to user provided array

       \see getNbVirtualParticleWeights()
     */
	virtual void getVirtualParticleWeights(PxVec3* userTriangleVertexWeightTable) const = 0;

    /**
       \brief Changes global pose matrix of the cloth
       \note This function does not preserve inertia. Use this to reset world space pose of cloth (e.g. teleporting)
       \see setTargetPose for inertia preserving method.
     */
	virtual void setGlobalPose(const PxTransform& pose) = 0;
    /**
       \brief Returns current global pose matrix of the cloth
       \returns global pose as PxTransform
     */
	virtual PxTransform getGlobalPose() const = 0;

    /**
       \brief Changes target pose matrix of the cloth
       \note This function will move cloth in world space.\n
       The resulting simulation may reflect inertia effect due to (possibly) rapid motition.
       \see use setGlobalPose() to ignore inertia effect.
     */
	virtual void setTargetPose(const PxTransform& pose) = 0;

    /**
       \brief Changes additional acceleration to this cloth 
       \note Use this to implement a simple wind effect, etc.
     */
	virtual void setExternalAcceleration(PxVec3 acceleration) = 0;
    /**
       \brief Returns current external acccleration (default = 0)
     */
	virtual PxVec3 getExternalAcceleration() const = 0;

    /**
	\brief Sets the damping coefficient in the range from 0 to 1.

	<b>Default:</b> 0

	\param[in] dampingCoefficient damping coefficient of the cloth.
	*/
	virtual void setDampingCoefficient(PxReal dampingCoefficient) = 0;
    /**
       \brief Returns current damping coefficient
     */
	virtual PxReal getDampingCoefficient() const = 0;

    /**
       \brief Sets the solver frequency parameter

       Solver frequency specifies how often the simulation step is computed per second. 

       For example, a value of 60 represents one simulation step per frame
       in a 60fps scene.  A value of 120 will represent two simulation steps per frame, etc.
     */
	virtual void setSolverFrequency(PxReal) = 0;
    /**
       \brief Returns current solver frequency 
     */
	virtual PxReal getSolverFrequency() const = 0;

    /**
       \brief Sets solver configuration per phase type.
       
       Users can assign different solver configuration (solver type, stiffness, etc.) per each phase type (see PxClothFabricPhaseType).
     */
	virtual void setPhaseSolverConfig(PxClothFabricPhaseType::Enum phaseType, const PxClothPhaseSolverConfig&) = 0;
    /**
       \brief Reads solver configuration for specified phase type.
       \return solver configuration (see PxClothPhaseSolverConfig)

       \note If <b> phaseType </b> is invalid, the returned solver configuration's solverType member will become eINVALID.
     */
	virtual PxClothPhaseSolverConfig getPhaseSolverConfig(PxClothFabricPhaseType::Enum phaseType) const = 0;

    /**
       \brief Sets options for this cloth (e.g. use GPU or not, use CCD or not).
     */
	virtual	void setClothFlag(PxClothFlag::Enum flag, bool val)	= 0;
    /**
       \brief Returns current option flags for this cloth.
     */
	virtual PxClothFlags getClothFlags() const = 0;

    /**
       \brief Returns true if this cloth is in a sleeping mode
     */
	virtual bool isSleeping() const = 0;
    /**
       \brief Returns threshold for linear velocity for this cloth to be put to sleeping mode.
     */
	virtual PxReal getSleepLinearVelocity() const = 0;
    /**
       \brief Sets the threshold for linear velocity for this cloth to be put to sleeping mode.
     */
	virtual void setSleepLinearVelocity(PxReal threshold) = 0;
    /**
       \brief Forces this cloth to wake up from the sleep state
     */
	virtual void wakeUp(PxReal wakeCounterValue = PX_SLEEP_INTERVAL) = 0;
    /**
       \brief Forces this cloth to be put to sleep mode
     */
	virtual void putToSleep() = 0;

    /**
       \brief Locks the cloth solver so that external applications can safely read back simulation data.
       \return see PxClothReadData for available user data for read back.
     */
	virtual PxClothReadData* lockClothReadData() const = 0;

    /**
       \brief Returns previous time step size (for use in computation of velocity or user-controlled adaptive time step scheme)
     */
	virtual PxReal getPreviousTimeStep() const = 0;

    /**
       \brief Returns world space bounding box for this cloth object.
     */
	virtual PxBounds3 getWorldBounds() const = 0;
};

#ifndef PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif

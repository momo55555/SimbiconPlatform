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


#ifndef PX_PHYSICS_NX_DEFORMABLE
#define PX_PHYSICS_NX_DEFORMABLE
/** \addtogroup deformable
  @{
*/

#include "PxPhysX.h"
#include "deformable/PxDeformableDesc.h"
#include "PxActor.h"
#include "PxForceMode.h"

#ifndef PX_DOXYGEN
namespace physx
{
#endif

class PxScene;
class PxShape;

PX_DEPRECATED class PxDeformable : public PxActor
{
protected:
// PX_SERIALIZATION
	PxDeformable(PxRefResolver& v) : PxActor(v)		{}
//~PX_SERIALIZATION
	PX_INLINE PxDeformable() : PxActor() {}
	virtual ~PxDeformable() {}

public:
	/**
	\brief Saves the deformable descriptor.

	\param[out] desc The descriptor used to retrieve the state of the object.
	\return True on success.

	@see PxDeformableDesc
	*/
	virtual	bool saveToDesc(PxDeformableDesc& desc) const = 0;

	/**
	\brief Returns a pointer to the corresponding deformable mesh.

	\return The deformable mesh associated with this deformable.

	<b>Platform:</b>
	\li PC SW: Yes
	\li PS3  : Yes
	\li XB360: Yes
	\li WII	 : Yes

	@see PxDeformableDesc.deformableMesh
	*/
	virtual	PxDeformableMesh* getDeformableMesh() const = 0;

	/**
	\brief Sets the bending stiffness in the range from 0 to 1. Ineffective for soft bodies.
 
	\param[in] bendingStiffness The bending stiffness of this deformable.

	@see PxDeformableDesc.bendingStiffness getBendingStiffness()
	*/
	virtual void setBendingStiffness(PxReal bendingStiffness) = 0;

	/**
	\brief Retrieves the bending stiffness.

	\return Bending stiffness of the deformable.

	@see PxDeformableDesc.bendingStiffness setBendingStiffness()
	*/
	virtual PxReal getBendingStiffness() const = 0;

	/**
	\brief Sets the deformable volume stiffness in the range from 0 to 1. Ineffective for cloth.
 
	\param[in] stiffness The volume stiffness of this deformable.

	@see PxDeformableDesc.volumeStiffness getVolumeStiffness()
	*/
	virtual void setVolumeStiffness(PxReal stiffness) = 0;

	/**
	\brief Retrieves the deformable volume stiffness.

	\return Volume stiffness of the deformable.

	@see PxDeformableDesc.volumeStiffness setVolumeStiffness()
	*/
	virtual PxReal getVolumeStiffness() const = 0;

	/**
	\brief Sets the deformable stretching stiffness in the range from 0 to 1.

	Note: The stretching stiffness must be larger than 0.

	\param[in] stretchingStiffness Stretching stiffness of the deformable.

	@see PxDeformableDesc.stretchingStiffness getStretchingStiffness()
	*/
	virtual void setStretchingStiffness(PxReal stretchingStiffness) = 0;

	/**
	\brief Retrieves the deformable stretching stiffness.

	\return stretching stiffness of the deformable.

	@see PxDeformableDesc.stretchingStiffness setStretchingStiffness()
	*/
	virtual PxReal getStretchingStiffness() const = 0;

	/**
	\brief Sets the damping coefficient in the range from 0 to 1.

	\param[in] dampingCoefficient damping coefficient of the deformable.

	@see PxDeformableDesc.dampingCoefficient getDampingCoefficient()
	*/
	virtual void setDampingCoefficient(PxReal dampingCoefficient) = 0;

	/**
	\brief Retrieves the damping coefficient.

	\return damping coefficient of the deformable.

	@see PxDeformableDesc.dampingCoefficient setDampingCoefficient()
	*/
	virtual PxReal getDampingCoefficient() const = 0;

	/**
	\brief Sets the deformable static friction coefficient in the range from 0 to +inf.

	\param[in] staticFriction The static friction of the deformable.

	@see PxDeformableDesc.staticFriction getStaticFriction()
	*/
	virtual void setStaticFriction(PxReal staticFriction) = 0;

	/**
	\brief Retrieves the deformable static friction coefficient.

	\return Static friction coefficient of deformable.

	@see PxDeformableDesc.staticFriction setStaticFriction()
	*/
	virtual PxReal getStaticFriction() const = 0;

	/**
	\brief Sets the deformable dynamic friction coefficient in the range from 0 to +inf.

	\param[in] dynamicFriction The static friction of the deformable.

	@see PxDeformableDesc.dynamicFriction getDynamicFriction()
	*/
	virtual void setDynamicFriction(PxReal dynamicFriction) = 0;

	/**
	\brief Retrieves the deformable dynamic friction coefficient.

	\return Dynamic friction coefficient of deformable.

	@see PxDeformableDesc.dynamicFriction setDynamicFriction()
	*/
	virtual PxReal getDynamicFriction() const = 0;

	/**
	\brief Sets the deformable pressure coefficient (must be non negative). Ineffective for soft bodies.

	\param[in] pressure The pressure applied to the deformable.

	@see PxDeformableDesc.pressure getPressure()
	*/
	virtual void setPressure(PxReal pressure) = 0;

	/**
	\brief Retrieves the deformable pressure coefficient.

	\return Pressure of deformable.

	@see PxDeformableDesc.pressure setPressure()
	*/
	virtual PxReal getPressure() const = 0;

	/**
	\brief Sets the deformable tear factor (must be larger than one).

	\param[in] factor The tear factor for the deformable

	@see PxDeformableDesc.tearFactor getTearFactor()
	*/
	virtual void setTearFactor(PxReal factor) = 0;

	/**
	\brief Retrieves the deformable tear factor.

	\return tear factor of the deformable.

	@see PxDeformableDesc.tearFactor setTearFactor()
	*/
	virtual PxReal getTearFactor() const = 0;

	/**
	\brief Sets the deformable attachment tear factor (must be larger than one).

	\param[in] factor The attachment tear factor for the deformable

	@see PxDeformableDesc.attachmentTearFactor getAttachmentTearFactor()
	*/
	virtual void setAttachmentTearFactor(PxReal factor) = 0;

	/**
	\brief Retrieves the attachment deformable tear factor.

	\return tear attachment factor of the deformable.

	@see PxDeformableDesc.attachmentTearFactor setAttachmentTearFactor()
	*/
	virtual PxReal getAttachmentTearFactor() const = 0;

	/**
	\brief Gets the deformable mass.

	\return mass of the deformable.

	@see PxDeformableDesc.mass
	*/
	virtual PxReal getMass() const = 0;

	/**
	\brief Gets the relative grid spacing for the broad phase.
	The deformable is represented by a set of world aligned cubical cells in broad phase.
	The size of these cells is determined by multiplying the length of the diagonal
	of the AABB of the initial deformable size with this constant.

	\return relative grid spacing.

	@see PxDeformableDesc.relativeGridSpacing
	*/
	virtual PxReal getRelativeGridSpacing() const = 0;

	/**
	\brief Retrieves the deformable solver iterations.

	\return solver iterations of the deformable.

	@see PxDeformableDesc.solverIterations setSolverIterations()
	*/
	virtual PxU32 getSolverIterations() const = 0;

	/**
	\brief Sets the deformable solver iterations.

	\param[in] iterations The new solver iteration count for the deformable.

	@see PxDeformableDesc.solverIterations getSolverIterations()
	*/
	virtual void setSolverIterations(PxU32 iterations) = 0;

	/**
	\brief Retrieves the number of iterations of the hierarchical deformable solver.

	For the this value to have an effect, the parameter PxDeformableMeshDesc.maxHierarchyLevels
	must be greater then one when cooking the deformable mesh. In this case, the hierarchical
	deformable solver uses the mesh hierarchy to speed up convergence, i.e. makes large
	soft bodies less stretchy.

	\return number of iterations of the hierarchical deformable solver.

	@see PxDeformableDesc.hierarchicalSolverIterations setHierarchicalSolverIterations() PxDeformableMeshDesc.maxHierarchyLevels
	*/
	virtual PxU32 getHierarchicalSolverIterations() const = 0;

	/**
	\brief Sets the number of iterations of the hierarchical deformable solver.

	For the this value to have an effect, the parameter PxDeformableMeshDesc.maxHierarchyLevels
	must be greater then one when cooking the deformable mesh. In this case, the hierarchical
	deformable solver uses the mesh hierarchy to speed up convergence, i.e. makes large
	soft bodies less stretchy.

	\param[in] iterations Number of iterations of the hierarchical deformable solver.

	@see PxDeformableDesc.hierarchicalSolverIterations getHierarchicalSolverIterations() PxDeformableMeshDesc.maxHierarchyLevels
	*/
	virtual void setHierarchicalSolverIterations(PxU32 iterations) = 0;

	/**
	\brief Returns a world space AABB enclosing all deformable points.

	\return world space bounds.

	@see PxBounds3
	*/
	virtual PxBounds3 getWorldBounds() const = 0;

	/**
	\brief Tears the deformable at a given vertex.

	For cloth, connected triangles can separate along their edges.
	For soft bodies, connected tetrahedra can separate along their faces.

	First the vertex is duplicated. The primitive on one side of the split plane keep 
	the original vertex. For all primitives on the opposite side the original vertex is 
	replaced by the new one. The split plane is defined by the world location of the 
	vertex and the normal provided by the user.

	Note: tearVertex performs a user defined vertex split in contrast to an automatic split
	that is performed when the flag PxDeformableFlag::eTEARABLE is set. Therefore, tearVertex works 
	even if PxDeformableFlag::eTEARABLE is not set in PxDeformableDesc.flags.

	Note: For tearVertex to work in hardware mode, the deformableMesh has to be cooked with the
	flag PX_DEFORMABLE_MESH_TEARABLE set in PxDeformableMeshDesc.deformableFlags.

	\param[in] vertexId Index of the vertex to tear.
	\param[in] normal The normal of the split plane.
	\return true if the split had an effect (i.e. there were primitives on both sides of the split plane)

	@see PxDeformableFlag, PxDeformableMeshFlags, PxDeformableDesc.deformableFlags
	*/
	virtual bool tearVertex(const PxU32 vertexId, const PxVec3& normal) = 0;

	/**
	\brief Executes a raycast against the deformable.

	\param[in] rayOrigin Origin of the ray in world space.
	\param[in] rayDir Direction of the ray in world space.
	\param[out] hit The hit position.
	\param[out] vertexId Index to the nearest vertex hit by the raycast.

	\return true if the ray hits the deformable.
	*/
	virtual bool raycast(const PxVec3& rayOrigin, const PxVec3& rayDir, PxVec3& hit, PxU32& vertexId) = 0;

	/**
	Note: Valid bounds do not have an effect on soft bodies in the current version.

	\brief Sets the valid bounds of the deformable in world space.

	If the flag PxDeformableFlag::eVALIDBOUNDS is set, these bounds defines the volume
	outside of which deformable vertices are automatically removed from the simulation. 

	\param[in] validBounds The valid bounds.

	@see PxDeformableDesc.validBounds getValidBounds() PxBounds3
	*/
	virtual void setValidBounds(const PxBounds3& validBounds) = 0;

	/**
	Note: Valid bounds do not have an effect on soft bodies in the current version.

	\brief Returns the valid bounds of the deformable in world space.

	@see PxDeformableDesc.validBounds setValidBounds() PxBounds3
	*/
	virtual PxBounds3 getValidBounds() const = 0;


	/**
	\brief Sets the vertex positions of the deformable.

	The supplied positions are used to change vertex positions in the order the ids are listed in the id buffer.
	Duplicate ids are allowed. A position buffer of stride zero is allowed.
	*/
	virtual void setPositions(PxU32 numVertices, const PxStrideIterator<const PxVec3>& positions, const PxStrideIterator<const PxU32>& indices = PxStrideIterator<const PxU32>()) = 0;

	/**
	\brief Sets the vertex velocities of the deformable.

	The supplied positions are used to change vertex positions in the order the ids are listed in the id buffer.
	Duplicate ids are allowed. A velocity buffer of stride zero is allowed.
	*/
	virtual void setVelocities(PxU32 numVertices, const PxStrideIterator<const PxVec3>& velocities, const PxStrideIterator<const PxU32>& indices = PxStrideIterator<const PxU32>()) = 0;

	/**
	\brief Sets the buffer of constrain positions.

	The user can supply a buffer of positions and normals coming from an animation of the deformable
	for instance. The simulated vertices are then corrected via the constrain positions, normals
	and the constrain coefficiens.

	\param[in] positions The user supplied buffer containing all constrain positions for the deformable.
	The number of positions provided must correspond to the number returned by getNumVertices().

	@see setConstrainCoefficients() setConstrainNormals()
	*/
	virtual void setConstrainPositions(const PxStrideIterator<const PxVec3>& positions) = 0;

		/**
	\brief Sets the buffer of constrain normals.

	The user can supply a buffer of positions and normals coming from an animation of the deformable
	for instance. The simulated vertices are then corrected via the constrain positions, normals
	and the constrain coefficiens set by setConstrainCoefficients.

	\param[in] normals The user supplied buffer containing all constrain normals for the deformable.
	The number of normals provided must correspond to the number returned by getNumVertices().

	@see setConstrainCoefficients() setConstrainPositions()
	*/
	virtual void setConstrainNormals(const PxStrideIterator<const PxVec3>& normals) = 0;

	/**
	\brief Sets the buffer of constrain coefficients of the type PxReal.

	The user can supply a buffer of positions and normals coming from an animation of the deformable
	for instance. The simulated vertices are then corrected via the constrain positions, normals
	and the constrain coefficiens set by setConstrainCoefficients.

	\param[in] coefficients The user supplied buffer containing all constrain coefficients for the deformable.
	The number of coefficients provided must correspond to the number returned by getNumVertices().
	
	@see setConstrainPositions() setConstrainNormals()
	*/
	virtual void setConstrainCoefficients(const PxStrideIterator<const PxDeformableConstrainCoefficients>& coefficients) = 0;

	/**
	\brief Gets the current number of deformable vertices.

	This number will grow beyond the initial number of vertices supplied by the user mesh if tearing
	creates new vertices. It is however garantueed to be smaller than PxDeformableDesc.maxVertices.

	@see setVelocities() getVelocities() setPositions() getPositions() 
	*/
	virtual PxU32 getNumVertices() const = 0;

	/**
	\brief Gets the maximum number of deformable vertices.

	@see setVelocities() getVelocities() setPositions() getPositions() 
	*/
	virtual PxU32 getMaxVertices() const = 0;

	/**
	\brief Sets the collision stiffness.

	\param[in] collisionStiffness The collision stiffness [0-1].

	@see PxDeformableDesc.collisionStiffness getCollisionStiffness()
	*/
	virtual void setCollisionStiffness(PxReal collisionStiffness) = 0;

	/**
	\brief Retrieves the collision stiffness.

	\return The collision stiffness.

	@see PxDeformableDesc.collisionStiffness setCollisionStiffness()
	*/
	virtual PxReal getCollisionStiffness() const = 0;

	/**
	\brief Sets the attachment stiffness.

	\param[in] attachmentStiffness The attachment stiffness [0-1].

	@see PxDeformableDesc.attachmentStiffness getAttachmentStiffness()
	*/
	virtual void setAttachmentStiffness(PxReal attachmentStiffness) = 0;

	/**
	\brief Retrieves the attachment stiffness.

	\return The attachment stiffness.

	@see PxDeformableDesc.attachmentStiffness setAttachmentStiffness()
	*/
	virtual PxReal getAttachmentStiffness() const = 0;

	/**
	\brief Sets the contact offset.

	\param[in] contactOffset The contact offset [restOffset - +inf].

	@see PxDeformableDesc.contactOffset getContactOffset()
	*/
	virtual void setContactOffset(PxReal contactOffset) = 0;

	/**
	\brief Retrieves the contact offset.

	\return The contact offset.

	@see PxDeformableDesc.contactOffset setContactOffset()
	*/
	virtual PxReal getContactOffset() const = 0;

	/**
	\brief Sets the rest offset.

	\param[in] restOffset The rest offset [0 - contactOffset].

	@see PxDeformableDesc.restOffset getRestOffset()
	*/
	virtual void setRestOffset(PxReal restOffset) = 0;

	/**
	\brief Retrieves the rest offset.

	\return The rest offset.

	@see PxDeformableDesc.restOffset setRestOffset()
	*/
	virtual PxReal getRestOffset() const = 0;

	/**
	\brief Sets the penetration offset.

	\param[in] penetrationOffset The penetration offset [-inf - 0].

	@see PxDeformableDesc.penetrationOffset getPenetrationOffset()
	*/
	virtual void setPenetrationOffset(PxReal penetrationOffset) = 0;

	/**
	\brief Retrieves the penetration offset.

	\return The penetration offset.

	@see PxDeformableDesc.penetrationOffset setPenetrationOffset()
	*/
	virtual PxReal getPenetrationOffset() const = 0;

	/**
	\brief Sets the self collision offset.

	\param[in] selfCollisionOffset The self collision offset (0-inf].

	@see PxDeformableDesc.selfCollisionOffset setSelfCollisionOffset()
	*/
	virtual void setSelfCollisionOffset(PxReal selfCollisionOffset) = 0;

	/**
	\brief Retrieves the self collision offset.

	\return The self collision offset.

	@see PxDeformableDesc.selfCollisionOffset setSelfCollisionOffset()
	*/
	virtual PxReal getSelfCollisionOffset() const = 0;

	/**
	\brief Sets an external acceleration which affects all non attached vertices of the deformable

	\param[in] acceleration The acceleration vector (unit length / s^2)

	@see PxDeformableDesc.externalAcceleration getExternalAcceleration()
	*/
	virtual void setExternalAcceleration(PxVec3 acceleration) = 0;

	/**
	\brief Retrieves the external acceleration which affects all non attached vertices of the deformable

	\return The acceleration vector (unit length / s^2)

	@see PxDeformableDesc.externalAcceleration setExternalAcceleration()
	*/
	virtual PxVec3 getExternalAcceleration() const = 0;

	/**
	\brief Sets an acceleration acting normal to the deformable surface at each vertex. Ineffective for soft bodies.

	\param[in] acceleration The acceleration vector (unit length / s^2)

	@see PxDeformableDesc.windAcceleration getWindAcceleration()
	*/
	virtual void setWindAcceleration(PxVec3 acceleration) = 0;

	/**
	\brief Retrieves the acceleration acting normal to the deformable surface at each vertex.

	\return The acceleration vector (unit length / s^2)

	@see PxDeformableDesc.windAcceleration setWindAcceleration()
	*/
	virtual PxVec3 getWindAcceleration() const = 0;

	/**
	\brief Returns true if this deformable is sleeping.

	When a deformable does not move for a period of time, it is no longer simulated in order to save time. This state
	is called sleeping. However, because the object automatically wakes up when it is either touched by an awake object,
	or one of its properties is changed by the user, the entire sleep mechanism should be transparent to the user.
	
	If a deformable is asleep after the call to PxScene::fetchResults() returns, it is guaranteed that the position of the deformable 
	vertices was not changed. You can use this information to avoid updating dependent objects.
	
	\return True if the deformable is sleeping.

	@see isSleeping() getSleepLinearVelocity() wakeUp() putToSleep()
	*/
	virtual bool isSleeping() const = 0;

	/**
	\brief Returns the linear velocity below which a deformable may go to sleep.
	
	A deformable whose linear velocity is above this threshold will not be put to sleep.
    
    @see isSleeping

	\return The threshold linear velocity for sleeping.

	@see isSleeping() getSleepLinearVelocity() wakeUp() putToSleep() setSleepLinearVelocity()
	*/
    virtual PxReal getSleepLinearVelocity() const = 0;

    /**
	\brief Sets the linear velocity below which a deformable may go to sleep.
	
	A deformable whose linear velocity is above this threshold will not be put to sleep.
    
	\param[in] threshold Linear velocity below which a deformable may sleep. <b>Range:</b> (0,inf]

	@see isSleeping() getSleepLinearVelocity() wakeUp() putToSleep()
	*/
    virtual void setSleepLinearVelocity(PxReal threshold) = 0;

	/**
	\brief Wakes up the deformable if it is sleeping.  

	The wakeCounterValue determines how long until the deformable is put to sleep, a value of zero means 
	that the deformable is sleeping. wakeUp(0) is equivalent to PxDeformable::putToSleep().

	\param[in] wakeCounterValue New sleep counter value. <b>Range:</b> [0,inf]

	@see isSleeping() getSleepLinearVelocity() putToSleep()
	*/
	virtual void wakeUp(PxReal wakeCounterValue = PX_SLEEP_INTERVAL) = 0;

	/**
	\brief Forces the deformable to sleep. 
	
	The deformable will fall asleep.

	@see isSleeping() getSleepLinearVelocity() wakeUp()
	*/
	virtual void putToSleep() = 0;

	/**
	\brief Sets the user definable collision filter data.
	
	<b>Sleeping:</b> Does wake up the deformable if the filter data change causes a formerly suppressed
	collision pair to be enabled.

	@see getSimulationFilterData() PxDeformableDesc.simulationFilterData
	*/
	virtual void setSimulationFilterData(const PxFilterData &data) = 0;

	/**
	\brief Retrieves the object's collision filter data.

	@see setSimulationFilterData() PxDeformableDesc.simulationFilterData
	*/
	virtual PxFilterData getSimulationFilterData() const = 0;

	/**
	\brief Marks the object to reset interactions and re-run collision filters in the next simulation step.
	
	See #PxShape::resetFiltering() for more details.

	@see PxSimulationFilterShader PxSimulationFilterCallback
	*/
	virtual void resetFiltering() = 0;

    /**
	\brief Sets deformable flags

    \param flag Member of #PxDeformableFlag.
	\param value New flag value.

	@see PxDeformableDesc.deformableFlags PxDeformableFlag getDeformableFlags()
	*/
    virtual void setDeformableFlag(PxDeformableFlag::Enum flag, bool value) = 0;

    /**
	\brief Retrieves the deformable flags.

	\return The current flag values.

	@see PxDeformableDesc.deformableFlags PxDeformableFlag setDeformableFlag()
	*/
	virtual PxU32 getDeformableFlags() const = 0;

	/**
	\brief Returns deformable read data flags.

	\return The deformable read data flags.

	@see PxDeformableDesc.deformableReadDataFlags
	*/
	virtual PxU32 getDeformableReadDataFlags() const = 0;

	/**
	\brief Applies forces (or impulses), defined in the global coordinate frame, to a set of deformable vertices.

    ::PxForceMode determines if the force is to be conventional or impulsive.

	\param[in] numVertices Number of vertices to be updated by the supplied forces.
	\param[in] forces Forces/impulses to add, defined in the global frame. <b>Range:</b> force vector	
	\param[in] mode The mode to use when applying the force/impulse
	\param[in] indices Optional pointer to an index list of target vertices. Duplicates are allowed.
	(see #PxForceMode, supported modes are PxForceMode::eFORCE, PxForceMode::eIMPULSE, 
	PxForceMode::eACCELERATION, PxForceMode::eVELOCITY_CHANGE)

	@see PxForceMode 
	*/
	virtual	void addForces(PxU32 numVertices, const PxStrideIterator<const PxVec3>& forces, PxForceMode::Enum mode, 
								const PxStrideIterator<const PxU32>& indices = PxStrideIterator<const PxU32>()) = 0;

	/**
	\brief Locks the deformable read data and provides the data descriptor for accessing it.

	@see PxDeformableReadData
	*/
	virtual PxDeformableReadData* lockDeformableReadData() = 0;
};

#ifndef PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif

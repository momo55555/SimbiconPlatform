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


#ifndef PX_PHYSICS_NX_AGGREGATE
#define PX_PHYSICS_NX_AGGREGATE

/** \addtogroup physics
@{
*/

#include "PxPhysX.h"

// PX_SERIALIZATION
#include "common/PxSerialFramework.h"
//~PX_SERIALIZATION

#ifndef PX_DOXYGEN
namespace physx
{
#endif

class PxActor;

/**
\brief Class to aggregate actors into a single broad phase entry.

A PxAggregate object is a collection of PxActors, which will exist as a single entry in the
broad-phase structures. This has 3 main benefits:

1) it reduces "broad phase pollution", where multiple objects of a single entity often overlap
   all the time (e.g. typically in a ragdoll).

2) it reduces broad-phase memory usage (which can be vital e.g. on SPU)

3) filtering can be optimized a lot if self-collisions within an aggregate are not needed. For
   example if you don't need collisions between ragdoll bones, it's faster to simply disable
   filtering once and for all, for the aggregate containing the ragdoll, rather than filtering
   out each bone-bone collision in the filter shader.

@see PxActor PxAggregateDesc
*/

class PxAggregate : public PxSerializable
{
// PX_SERIALIZATION
protected:
						PxAggregate(PxRefResolver& v) : PxSerializable(v) {}
public:
						PX_DECLARE_SERIAL_RTTI(PxAggregate, PxSerializable)
//~PX_SERIALIZATION

protected:
						PxAggregate()	{}
	virtual				~PxAggregate()	{}

	public:

	/**
	\brief Deletes the aggregate object.

	Deleting the PxAggregate object does not delete the aggregated actors. If the PxAggregate object
	belongs to a scene, the aggregated actors are automatically re-inserted in that scene. If you intend
	to delete both the PxAggregate and its actors, it is best to release the actors first, then release
	the PxAggregate when it is empty.
	*/
	virtual	void		release()				= 0;

	/**
	\brief Adds an actor to the aggregate object.

	A warning is output if the total number of actors is reached, or if the incoming actor already belongs
	to an aggregate.

	If the aggregate belongs to a scene, adding an actor to the aggregate also adds the actor to that scene.

	If the actor already belongs to a scene, a warning is output and the call is ignored. You need to remove
	the actor from the scene first, before adding it to the aggregate.

	\param	[in] actor The actor that should be added to the aggregate
	return	true if success
	*/
	virtual	bool		addActor(PxActor& actor)		= 0;

	/**
	\brief Removes an actor from the aggregate object.

	A warning is output if the incoming actor does not belong to the aggregate. Otherwise the actor is
	removed from the aggregate. If the aggregate belongs to a scene, the actor is reinserted in that
	scene. If you intend to delete the actor, it is best to call #PxActor::release() directly. That way
	the actor will be automatically removed from its aggregate (if any) and not reinserted in a scene.

	\param	[in] actor The actor that should be removed from the aggregate
	return	true if success
	*/
	virtual	bool		removeActor(PxActor& actor)		= 0;

	/**
	\brief Adds an articulation to the aggregate object.

	A warning is output if the total number of actors is reached (every articulation link counts as an actor), 
	or if the incoming articulation already belongs	to an aggregate.

	If the aggregate belongs to a scene, adding an articulation to the aggregate also adds the articulation to that scene.

	If the articulation already belongs to a scene, a warning is output and the call is ignored. You need to remove
	the articulation from the scene first, before adding it to the aggregate.

	\param	[in] articulation The articulation that should be added to the aggregate
	return	true if success
	*/
	virtual	bool		addArticulation(PxArticulation& articulation) = 0;

	/**
	\brief Removes an articulation from the aggregate object.

	A warning is output if the incoming articulation does not belong to the aggregate. Otherwise the articulation is
	removed from the aggregate. If the aggregate belongs to a scene, the articulation is reinserted in that
	scene. If you intend to delete the articulation, it is best to call #PxArticulation::release() directly. That way
	the articulation will be automatically removed from its aggregate (if any) and not reinserted in a scene.

	\param	[in] articulation The articulation that should be removed from the aggregate
	return	true if success
	*/
	virtual	bool		removeArticulation(PxArticulation& articulation) = 0;

	/**
	\brief Retrieves the scene which this aggregate belongs to.

	\return Owner Scene. NULL if not part of a scene.

	@see PxScene
	*/
	virtual	PxScene*	getScene()	= 0;

	/**
	\brief Retrieves max amount of actors that can be contained in the aggregate.

	\return Max aggregate size. 

	@see PxAggregateDesc
	*/
	virtual	PxU32		getMaxSize()		const	= 0;

	/**
	\brief Retrieves current number of actors contained in the aggregate.

	\return Current aggregate size.
	*/
	virtual	PxU32		getCurrentSize()	const	= 0;

	/**
	\brief Retrieves a given actor contained in the aggregate.

	\param	[in] i	actor index in [0;getMaxSize()[
	\return Pointer to contained actor, or NULL if the slot is empty
	*/
	virtual	PxActor*	getActor(PxU32 i)	const	= 0;

	/**
	\brief Retrieves aggregate's self-collision flag.

	\return self-collision flag
	*/
	virtual	bool		getSelfCollision()	const	= 0;
};

#ifndef PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif

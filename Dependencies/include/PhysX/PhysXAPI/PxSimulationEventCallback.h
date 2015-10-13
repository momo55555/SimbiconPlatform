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


#ifndef PX_SIMULATION_EVENT_CALLBACK
#define PX_SIMULATION_EVENT_CALLBACK
/** \addtogroup physics
@{
*/

#include "PxVec3.h"
#include "PxPhysX.h"
#include "PxFiltering.h"
#include "PxContactStreamIterator.h"

#ifndef PX_DOXYGEN
namespace physx
{
#endif

class PxShape;
class PxActor;
class PxConstraint;

/**
\brief An instance of this class is passed to PxSimulationEventCallback.onContact().
It contains a contact stream which may be parsed using the class PxContactStreamIterator.

@see PxSimulationEventCallback.onContact()
*/
class PxContactPair
{
	public:
		PX_INLINE	PxContactPair() : stream(NULL)	{}

	/**
	\brief The two actors that make up the pair.

	\note The actor pointers might reference deleted actors. Check the #isDeletedActor member to see
	      whether that is the case. Do not dereference a pointer to a deleted actor. The pointer to a
		  deleted actor is only provided such that user data structures which might depend on the pointer
		  value can be updated.

	@see PxActor
	*/
	PxActor*				actors[2];

	/**
	\brief Use this to create stream iter. See #PxContactStreamIterator.

	@see PxConstContactStream
	*/
	PxConstContactStream	stream;

	/**
	\brief The total contact normal force that was applied for this pair, to maintain nonpenetration constraints. You should set PxPairFlag::eNOTIFY_CONTACT_FORCES in order to receive this value.
	*/
	PxVec3					sumNormalForce;

	/**
	\brief The total tangential force that was applied for this pair. You should set PxPairFlag::eNOTIFY_CONTACT_FORCES in order to receive this value.
	*/
	PxVec3					sumFrictionForce;

	/**
	\brief Specifies for each actor of the pair if the actor has been deleted.

	Before dereferencing the actor pointers of the contact pair you might want to use this member
	to check if the pointers reference deleted actors. This will be the case if an actor for which
	PxPairFlag::eNOTIFY_TOUCH_LOST or PxPairFlag::eNOTIFY_THRESHOLD_FORCE_LOST events were requested gets deleted.

	@see actors
	*/
	bool					isDeletedActor[2];
};


/**
\brief Descriptor for a trigger pair.

An array of these structs gets passed to the PxSimulationEventCallback::onTrigger() report.

@see PxSimulationEventCallback.onTrigger()
*/
struct PxTriggerPair
{
	PX_INLINE PxTriggerPair() {}

	PxShape*				triggerShape;	//!< The shape that has been marked as a trigger.
	PxShape*				otherShape;		//!< The shape causing the trigger event.
	PxPairFlag::Enum		status;			//!< Type of trigger event (eNOTIFY_TOUCH_FOUND, eNOTIFY_TOUCH_PERSISTS or eNOTIFY_TOUCH_LOST).
};


/**
\brief Descriptor for a broken constraint.

An array of these structs gets passed to the PxSimulationEventCallback::onConstraintBreak() report.

@see PxConstraint PxSimulationEventCallback.onConstraintBreak()
*/
struct PxConstraintInfo
{
	PX_INLINE PxConstraintInfo() {}
	PX_INLINE PxConstraintInfo(PxConstraint* c, void* extRef, PxU32 t) : constraint(c), externalReference(extRef), type(t) {}

	PxConstraint*	constraint;				//!< The broken constraint.
	void*			externalReference;		//!< The external object which owns the constraint (see #PxConstraintConnector::getExternalReference())
	PxU32			type;					//!< Unique type ID of the external object. Allows to cast the provided external reference to the appropriate type
};


/**
 \brief An interface class that the user can implement in order to receive simulation events.

  \note SDK state should not be modified from within the callbacks. In particular objects should not
  be created or destroyed. If state modification is needed then the changes should be stored to a buffer
  and performed after the simulation step.

  <b>Threading:</b> It is not necessary to make this class thread safe as it will only be called in the context of the
  user thread.

 @see PxScene.setSimulationEventCallback() PxScene.getSimulationEventCallback()
*/
class PxSimulationEventCallback
	{
	public:
	/**
	\brief This is called when a breakable constraint breaks.
	
	\note The user should not release the constraint shader inside this call!

	\param[in] constraints - The constraints which have been broken.
	\param[in] count       - The number of constraints

	@see PxConstraint PxConstraintDesc.linearBreakForce PxConstraintDesc.angularBreakForce
	*/
	virtual PX_INLINE void onConstraintBreak(PxConstraintInfo* constraints, PxU32 count);

	/**
	\brief This is called during PxScene::fetchResults with the actors which have just been woken up.

	\note Only supported by rigid bodies yet.

	\param[in] actors - The actors which just woke up.
	\param[in] count  - The number of actors

	@see PxScene.setSimulationEventCallback() PxSceneDesc.simulationEventCallback
	*/
	virtual PX_INLINE void onWake(PxActor** actors, PxU32 count);

	/**
	\brief This is called during PxScene::fetchResults with the actors which have just been put to sleep.

	\note Only supported by rigid bodies yet.

	\param[in] actors - The actors which have just been put to sleep.
	\param[in] count  - The number of actors

	@see PxScene.setSimulationEventCallback() PxSceneDesc.simulationEventCallback
	*/
	virtual PX_INLINE void onSleep(PxActor** actors, PxU32 count);

	/**
	\brief The user needs to implement this interface class in order to be notified when
	certain contact events occur.

	The method will be called for each pair of actors which comes into contact, 
	for which this behavior was enabled.

	The events parameter is a combination of:

	<ul>
	<li>PxPairFlag::eNOTIFY_TOUCH_FOUND,</li>
	<li>PxPairFlag::eNOTIFY_TOUCH_PERSISTS,</li>
	<li>PxPairFlag::eNOTIFY_TOUCH_LOST,</li>
	<li>PxPairFlag::eNOTIFY_THRESHOLD_FORCE_FOUND,</li>
	<li>PxPairFlag::eNOTIFY_THRESHOLD_FORCE_PERSISTS,</li>
	<li>PxPairFlag::eNOTIFY_THRESHOLD_FORCE_LOST</li>
	</ul>

	See the documentation of #PxPairFlag for an explanation of each. You request which events 
	are reported using the filter shader/callback mechanism (see #PxSimulationFilterShader,
	#PxSimulationFilterCallback).
	
	Do not keep a reference to the passed object, as it will be 
	invalid after this function returns.

	\param[in] pair The contact pair we are being notified of. See #PxContactPair.
	\param[in] events Flags raised due to the contact. See #PxPairFlag.

	@see PxScene.setSimulationEventCallback() PxSceneDesc.simulationEventCallback PxContactPair PxPairFlag PxSimulationFilterShader PxSimulationFilterCallback
	*/
	virtual PX_INLINE void onContact(PxContactPair& pair, PxU32 events);

	/*
	\brief This is called during PxScene::fetchResults with the current trigger pair events.

	Shapes which have been marked as triggers using PxShapeFlag::eTRIGGER_SHAPE will send events
	according to the pair flag specification in the filter shader (see #PxPairFlag, #PxSimulationFilterShader).

	\param[in] pairs - The trigger pairs which caused events.
	\param[in] count - The number of trigger pairs.

	@see PxScene.setSimulationEventCallback() PxSceneDesc.simulationEventCallback PxPairFlag PxSimulationFilterShader PxShapeFlag PxShape.setFlag()
	*/
	virtual PX_INLINE void onTrigger(PxTriggerPair* pairs, PxU32 count);

	virtual ~PxSimulationEventCallback() {}
	};

PX_INLINE void PxSimulationEventCallback::onConstraintBreak(PxConstraintInfo*, PxU32) {}
PX_INLINE void PxSimulationEventCallback::onWake(PxActor**, PxU32) {}
PX_INLINE void PxSimulationEventCallback::onSleep(PxActor**, PxU32) {}
PX_INLINE void PxSimulationEventCallback::onContact(PxContactPair&, PxU32) {}
PX_INLINE void PxSimulationEventCallback::onTrigger(PxTriggerPair*, PxU32) {}

#ifndef PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif

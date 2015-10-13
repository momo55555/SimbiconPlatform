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


#ifndef PX_PHYSICS_NX_ACTOR
#define PX_PHYSICS_NX_ACTOR

/** \addtogroup physics
  @{
*/

#include "PxPhysX.h"
#include "PxBounds3.h"
#include "PxActorDesc.h"
#include "PxPhysics.h"
#include "PxObserver.h"

// PX_SERIALIZATION
#include "common/PxSerialFramework.h"
//~PX_SERIALIZATION

#ifndef PX_DOXYGEN
namespace physx
{
#endif

class PxRigidActor;
class PxRigidBody;
class PxRigidStatic;
class PxRigidDynamic;
class PxParticleBase;
class PxParticleSystem;
class PxParticleFluid;
class PxDeformable;
class PxArticulation;
class PxArticulationLink;

/**
\brief PxActor is the base class for the main simulation objects in the physics SDK.

The actor is owned by and contained in a PxScene.

See #PxActorDesc for a more detailed description of the parameters which can be set when creating an actor.

@see PxActorDesc release()
*/

class PxActor
// PX_SERIALIZATION
	: public PxSerializable, public PxObservable
//~PX_SERIALIZATION
{
// PX_SERIALIZATION
	protected:
								PxActor(PxRefResolver& v) : PxSerializable(v)	{}
	public:
								PX_DECLARE_SERIAL_RTTI(PxActor, PxSerializable)
//~PX_SERIALIZATION
	protected:

	PX_INLINE					PxActor() : userData(NULL)
											{}
	virtual						~PxActor()	{}

	public:
	/**
	\brief Deletes the actor.
	
	Do not keep a reference to the deleted instance.

	<b>Sleeping:</b> If this is a PxRigidDynamic, this call will awaken any sleeping actors contacting the deleted actor (directly or indirectly).

	If the actor belongs to a #PxAggregate object, it is automatically removed from the aggregate.

	@see PxScene::createRigidStatic(), PxScene::createRigidDynamic(), PxScene::createParticleSystem(), 
	PxScene::createParticleFluid(), PxScene::createDeformable(), PxAggregate
	*/
	virtual		void			release() = 0;

	/**
	\brief Retrieves the type of actor.

	\return The actor type of the actor.

	@see PxActorType
	*/
	virtual		PxActorType::Enum	getType()	const = 0;

	/**
	\brief Type casting operator. The result may be cast to the desired subclass type.

	\param[in] type The type of actor to attempt a cast to.
	\return NULL if the actor is not type. Otherwise a pointer to the actor.

	@see PxActorType
	*/
	virtual void*					is(PxActorType::Enum type)			= 0;
	virtual	const void*				is(PxActorType::Enum type)	const	= 0;

	/**
	\brief Attempts to cast to specific actor type.

	\return NULL if the actor is not of the appropriate type. Otherwise a pointer to the specific actor type.

	\note Since PxParticleFluid inherits from PxParticleSystem, calling isParticleSystem() on a PxParticleFluid instance will
	succeed and return the upcast to PxParticleSystem.

	@see PxActorType PxRigidActor PxRigidBody PxRigidStatic PxRigidDynamic PxParticleBase PxParticleSystem PxParticleFluid PxDeformable
	*/
	PX_INLINE	PxRigidStatic*				isRigidStatic()					{ return (PxRigidStatic*)				is(PxActorType::eRIGID_STATIC);			}
	PX_INLINE	const PxRigidStatic*		isRigidStatic()			const	{ return (const PxRigidStatic*)			is(PxActorType::eRIGID_STATIC);			}
	PX_INLINE	PxRigidDynamic*				isRigidDynamic()				{ return (PxRigidDynamic*)				is(PxActorType::eRIGID_DYNAMIC);		}
	PX_INLINE	const PxRigidDynamic*		isRigidDynamic()		const	{ return (const PxRigidDynamic*)		is(PxActorType::eRIGID_DYNAMIC);		}
	PX_INLINE	PxParticleSystem*			isParticleSystem()				{ return (PxParticleSystem*)			is(PxActorType::ePARTICLE_SYSTEM);		}
	PX_INLINE	const PxParticleSystem*		isParticleSystem()		const	{ return (const PxParticleSystem*)		is(PxActorType::ePARTICLE_SYSTEM);		}
	PX_INLINE	PxParticleFluid*			isParticleFluid()				{ return (PxParticleFluid*)				is(PxActorType::ePARTICLE_FLUID);		}
	PX_INLINE	const PxParticleFluid*		isParticleFluid()		const	{ return (const PxParticleFluid*)		is(PxActorType::ePARTICLE_FLUID);		}
	PX_INLINE	PxDeformable*				isDeformable()					{ return (PxDeformable*)				is(PxActorType::eDEFORMABLE);			}
	PX_INLINE	const PxDeformable*			isDeformable()			const	{ return (const PxDeformable*)			is(PxActorType::eDEFORMABLE);			}
	PX_INLINE	PxArticulationLink*			isArticulationLink()			{ return (PxArticulationLink*)			is(PxActorType::eARTICULATION_LINK);	}
	PX_INLINE	const PxArticulationLink*	isArticulationLink()	const	{ return (const PxArticulationLink*)	is(PxActorType::eARTICULATION_LINK);	}
	PX_INLINE	PxCloth*					isCloth()						{ return (PxCloth*)						is(PxActorType::eCLOTH);				}
	PX_INLINE	const PxCloth*				isCloth()				const	{ return (const PxCloth*)				is(PxActorType::eCLOTH);				}

	virtual		PxRigidActor*				isRigidActor()					{ return 0; }
	virtual		const PxRigidActor*			isRigidActor()			const	{ return 0; }
	virtual		PxRigidBody*				isRigidBody()					{ return 0; }
	virtual		const PxRigidBody*			isRigidBody()			const	{ return 0; }
	virtual		PxParticleBase*				isParticleBase()				{ return 0; }
	virtual		const PxParticleBase*		isParticleBase()		const	{ return 0; }

	/**
	\brief Retrieves the scene which this actor belongs to.

	\return Owner Scene. NULL if not part of a scene.

	@see PxScene
	*/
	virtual		PxScene*		getScene()	const = 0;

	// Runtime modifications

	/**
	\brief Sets a name string for the object that can be retrieved with getName().
	
	This is for debugging and is not used by the SDK. The string is not copied by the SDK, 
	only the pointer is stored.

	\param[in] name String to set the objects name to.

	@see getName()
	*/
	virtual		void			setName(const char* name)		= 0;

	/**
	\brief Retrieves the name string set with setName().

	\return Name string associated with object.

	@see setName()
	*/
	virtual		const char*		getName()			const	= 0;

	/**
	\brief Retrieves the axis aligned bounding box enclosing the actor.

	\return The actor's bounding box.

	@see PxBounds3
	*/
	virtual		PxBounds3		getWorldBounds() const = 0;

	/**
	\brief Raises or clears a particular actor flag.
	
	See the list of flags #PxActorFlag

	<b>Sleeping:</b> Does <b>NOT</b> wake the actor up automatically.

	\param[in] flag  The PxActor flag to raise(set) or clear. See #PxActorFlag.
	\param[in] value The boolean value to assign to the flag.

	@see PxActorFlag getActorFlags() PxActorDesc.actorFlags
	*/
	virtual		void			setActorFlag(PxActorFlag::Enum flag, bool value) = 0;
	/**
	\brief sets the actor flags
	
	See the list of flags #PxActorFlag
	@see PxActorFlag setActorFlag() PxActorDesc.actorFlags
	*/
	virtual		void			setActorFlags( PxActorFlags inFlags ) = 0;

	/**
	\brief Reads the PxActor flags.
	
	See the list of flags #PxActorFlag

	\return The values of the PxActor flags.

	@see PxActorFlag setActorFlag() PxActorDesc.actorFlags
	*/
	virtual		PxActorFlags	getActorFlags()	const = 0;

	/**
	\brief Assigns dynamic actors a dominance group identifier.
	
	PxDominanceGroup is a 5 bit group identifier (legal range from 0 to 31).
	
	The PxScene::setDominanceGroupPair() lets you set certain behaviors for pairs of dominance groups.
	By default every dynamic actor is created in group 0.

	<b>Sleeping:</b> Changing the dominance group does <b>NOT</b> wake the actor up automatically.

	\param[in] dominanceGroup The dominance group identifier. <b>Range:</b> [0..31]

	@see getDominanceGroup() PxDominanceGroup PxScene::setDominanceGroupPair()
	*/
	virtual		void			setDominanceGroup(PxDominanceGroup dominanceGroup)		 = 0;
	
	/**
	\brief Retrieves the value set with setDominanceGroup().

	\return The dominance group of this actor.

	@see setDominanceGroup() PxDominanceGroup PxScene::setDominanceGroupPair()
	*/
	virtual		PxDominanceGroup	getDominanceGroup() const = 0;

	
	/**
	\brief Sets the owner client of an actor.

	This cannot be done once the actor has been placed into a scene.

	@see PxActorDesc::ownerClient PxClientID
	*/
	virtual		void			setOwnerClient( PxClientID inClient ) = 0;

	/**
	\brief Returns the owner client that was specified with PxActorDesc::ownerClient at creation time.

	This value cannot be changed once the object is placed into the scene.

	@see PxActorDesc::ownerClient PxClientID
	*/
	virtual		PxClientID		getOwnerClient() const = 0;

	/**
	\brief Changes the behavior bits initially specified with PxActorDesc::clientBehaviorBits.

	@see PxActorDesc::clientBehaviorBits PxActorClientBehaviorBit
	*/
	virtual		void			setClientBehaviorBits(PxU32) = 0;

	/**
	\brief Retrieves the multiclient behavior bits 

	@see PxActorDesc::clientBehaviorBits PxActorClientBehaviorBit setClientBehaviorBits
	*/
	virtual		PxU32			getClientBehaviorBits()	const = 0;

	/**
	\brief Retrieves the aggregate the actor might be a part of.

	\return The aggregate the actor is a part of, or NULL if the actor does not belong to an aggregate.

	@see PxAggregate
	*/
	virtual		PxAggregate*	getAggregate()	const = 0;

	//public variables:
				void*			userData;	//!< user can assign this to whatever, usually to create a 1:1 relationship with a user object.
};

#ifndef PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif

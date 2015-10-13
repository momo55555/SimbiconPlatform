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


#ifndef PX_PHYSICS_NX_ACTORDESC
#define PX_PHYSICS_NX_ACTORDESC
/** \addtogroup physics
@{
*/

#include "PxPhysX.h"
#include "PxFlags.h"
#include "PxClient.h"

#ifndef PX_DOXYGEN
namespace physx
{
#endif

/** Group index which allows to specify 1- or 2-way interaction */
typedef PxU8 PxDominanceGroup;		// Must be < 32, PxU8.

/**
\brief Flags which control the behavior of an actor.

@see PxActorFlags PxActor PxActorDesc PxActor.setActorFlag() PxActor.getActorFlags()
*/
struct PxActorFlag
{
	enum Enum
	{
		/**
		\brief Enable debug renderer for this actor

		@see PxScene.getRenderBuffer() PxRenderBuffer PxVisualizationParameter
		*/
		eVISUALIZATION					= (1<<0),

		/**
		\brief Disables scene gravity for this actor
		*/
		eDISABLE_GRAVITY				= (1<<1),

		/**
		\brief Enables the sending of PxSimulationEventCallback::onWake() and PxSimulationEventCallback::onSleep() notify events

		@see PxSimulationEventCallback::onWake() PxSimulationEventCallback::onSleep()
		*/
		eSEND_SLEEP_NOTIFIES			= (1<<2)

	};
};

/**
\brief collection of set bits defined in PxActorFlag.

@see PxActorFlag
*/
typedef PxFlags<PxActorFlag::Enum,PxU16> PxActorFlags;
PX_FLAGS_OPERATORS(PxActorFlag::Enum,PxU16);

/**
\brief Identifies each type of actor.
@see PxActor PxActorDesc
*/
struct PxActorType
{
	enum Enum
	{
		/**
		\brief A static rigid body
		@see PxRigidStatic
		*/
		eRIGID_STATIC,

		/**
		\brief A dynamic rigid body
		@see PxRigidDynamic
		*/
		eRIGID_DYNAMIC,

		/**
		\brief A particle system
		@see PxParticleSystem
		*/
		ePARTICLE_SYSTEM,

		/**
		\brief A particle fluid
		@see PxParticleFluid
		*/
		ePARTICLE_FLUID,
		
		/**
		\brief A deformable
		@see PxDeformable
		*/
		eDEFORMABLE,

		/**
		\brief An articulation link
		@see PxArticulationLink
		*/
		eARTICULATION_LINK,

		/**
		\brief A cloth
		@see PxCloth
		*/
		eCLOTH,

		//brief internal use only!
		eACTOR_COUNT,

		eACTOR_FORCE_DWORD = 0x7fffffff
	};
};

/**
\brief Actor Descriptor. This structure is used to save and load the state of #PxActor objects.
*/

class PxActorDesc
{
protected:

	/**
	\brief The type of actor. This is set by the c'tor of the derived class.
	*/
	PxActorType::Enum		type;

public:

	/**
	\brief Dominance group for this body.

	PxDominanceGroup is a 5 bit group identifier (legal range from 0 to 31).
	The PxScene::setDominanceGroupPair() lets you set certain behaviors for pairs of dominance groups.
	By default every actor is created in group 0.

	<b>Default:</b> 0
	*/
	PxDominanceGroup		dominanceGroup;

	/**
	\brief Combination of ::PxActorFlag flags

	<b>Default:</b> PxActorFlag::eVISUALIZATION

	@see PxActor::setActorFlag() PxActor::getActorFlags()
	*/
	PxActorFlags			actorFlags;

	/**
	\brief Immutable client that creates and owns this actor.

	@see PxScene::createClient()
	*/
	PxClientID				ownerClient;

	/**
	\brief The multiclient behavior of this actor.

	A combination of PxActorClientBehaviorBit bits.
	@see PxActorClientBehaviorBit 
	*/
	PxU32					clientBehaviorBits;

	/**
	\brief Will be copied to PxActor::userData

	<b>Default:</b> NULL

	@see PxActor.userData
	*/
	void*					userData; 

	/**
	\brief Possible debug name. The string is not copied by the SDK, only the pointer is stored.

	<b>Default:</b> NULL
	*/
	const char*				name;

	/**
	\brief Returns the type of actor.
	*/
	PX_INLINE PxActorType::Enum getType() const { return type; }

	/**
	\brief Returns true if the descriptor is valid.

	\return True if the current settings are valid
	*/
	PX_INLINE virtual bool isValid() const;

protected:

	/**
	\brief constructor sets to default.
	*/
	PX_INLINE PxActorDesc(PxActorType::Enum t);

	/**
	\brief virtual destructor
	*/
	virtual ~PxActorDesc() {};

private:

	// base constructor not allowed.
	PX_INLINE PxActorDesc() {}

};



PX_INLINE PxActorDesc::PxActorDesc(PxActorType::Enum t) : type(t)
{
	dominanceGroup      = 0;
	actorFlags			= PxActorFlag::eVISUALIZATION;
	ownerClient			= PX_DEFAULT_CLIENT;
	clientBehaviorBits  = 0;
	userData			= NULL;
	name				= NULL;
}

PX_INLINE bool PxActorDesc::isValid() const
{
	return true;
}

#ifndef PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif

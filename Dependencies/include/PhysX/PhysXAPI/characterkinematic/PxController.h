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



#ifndef PX_PHYSICS_NXCONTROLLER
#define PX_PHYSICS_NXCONTROLLER
/** \addtogroup character
  @{
*/

#include "characterkinematic/PxCharacter.h"
#include "characterkinematic/PxExtended.h"

#ifndef PX_DOXYGEN
namespace physx
{
#endif

/**
\brief Specifies which axis is the "up" direction.
*/
struct PxCCTUpAxis
{
	enum Enum
	{
		eX			= 0, //!< X Axis
		eY			= 1, //!< Y Axis
		eZ			= 2, //!< Z Axis
	};
};

/**
\brief The type of controller, eg box, sphere or capsule.
*/
struct PxControllerShapeType
{
	enum Enum
	{
		/**
		\brief A box controller.

		@see PxBoxController PxBoxControllerDesc
		*/
		eBOX,

		/**
		\brief A capsule controller

		@see PxCapsuleController PxCapsuleControllerDesc
		*/
		eCAPSULE,

		eFORCE_DWORD = 0x7fffffff
	};
};

#ifndef PX_DOXYGEN
} // namespace physx
#endif

#include "geometry/PxTriangleMesh.h"

#ifndef PX_DOXYGEN
namespace physx
{
#endif

class PxShape;
class PxScene;
class PxController;
class PxRigidDynamic;
class PxMaterial;
struct PxFilterData;
class PxSceneQueryFilterCallback;

/**
\brief specifies how a CCT interacts with other CCTs.

This member controls if a character controller will collide with another controller. There are 3 options:
always collide, never collide and collide based on the shape group.
This flag only affects other controllers when they move; when this controller moves, the flag is ignored
and the flags of the other controllers determine collision.
*/
struct PxCCTInteractionMode
{
	enum Enum
	{
		eINCLUDE,		//!< Always collide character controllers.
		eEXCLUDE,		//!< Never collide character controllers.

		/**
		\brief Collide based on a group identifier stored in the shape collision filter data.

		The groups to collide against are passed in the activeGroups member of #PxController::move(). The active
		groups flags work on top of the Physics SDK filtering logic of the controllers kinematic actor to determine if a 
		collision should occur:

		activeGroups & ( 1 << shape->getSimulationFilterData().word3 )

		To set the shape filter data the PxController::getActor() method can be called, then the getShapes() method to
		retrieve the shape. Then PxShape::setSimulationFilterData() method is used to set the shapes' collision filter data.

		\note This filter logic uses the member PxFilterData.word3 of the shapes' collision filter data. Be aware that
		if the user filter shader passed into the scene (see #PxSimulationFilterShader) does use this member for non
		CCT actors to specify the collision filtering logic, you might get unexpected behavior. Ways to avoid this
		are to either not use the PxFilterData.word3 members in the filter shader or to use one bit of the filter data
		to mark actors as CCTs and process them separately from other filter objects in the filter shader.

		@see PxController.move() PxController.getActor() PxFilterData PxSimulationFilterShader
		*/
		eUSE_FILTER,	
	};
};

/**
\brief specifies which sides a character is colliding with.
*/
struct PxControllerFlag
{
	enum Enum
	{
		eCOLLISION_SIDES	= (1<<0),	//!< Character is colliding to the sides.
		eCOLLISION_UP		= (1<<1),	//!< Character has collision above.
		eCOLLISION_DOWN		= (1<<2),	//!< Character has collision below.
	};
};

/**
\brief Describes a controller shape hit. Passed to onShapeHit()

@see PxUserControllerHitReport.onShapeHit()
*/
struct PxControllerShapeHit
{
	PxController*	controller;		//!< Current controller
	PxShape*		shape;			//!< Touched shape
	PxExtendedVec3	worldPos;		//!< Contact position in world space
	PxVec3			worldNormal;	//!< Contact normal in world space
	PxVec3			dir;			//!< Motion direction
	PxF32			length;			//!< Motion length
	PxU32			triangleIndex;	//!< touched triangle index (only for meshes/heightfields)
};

/**
\brief Describe a controller hit. Passed to onControllerHit().

@see PxUserControllerHitReport.onControllerHit()
*/
struct PxControllersHit
{
	PxController*	controller;		//!< Current controller
	PxController*	other;			//!< Touched controller
};

/**
\brief User callback class for character controller events.

\note Character controller hit reports are only generated when move is called.

@see PxControllerDesc.callback
*/
class PxUserControllerHitReport
{
public:

	/**
	\brief Called when current controller hits a shape.

	\param[in] hit Provides information about the contact with the touched shape.

	@see PxControllerShapeHit
	*/
	virtual void onShapeHit(const PxControllerShapeHit& hit) = 0;

	/**
	\brief Called when current controller hits another controller.

	\param[in] hit Provides information about the touched controller.

	@see PxControllersHit
	*/
	virtual void onControllerHit(const PxControllersHit& hit) = 0;

protected:
	virtual ~PxUserControllerHitReport(){}
};

/**
\brief Descriptor class for a character controller.

@see PxBoxController PxCapsuleController
*/
class PxControllerDesc
{
protected:

	PxControllerShapeType::Enum type;		//!< The type of the controller. This gets set by the derived class' ctor, the user should not have to change it.

	/**
	\brief constructor sets to default.
	*/
	PX_INLINE										PxControllerDesc(PxControllerShapeType::Enum);
	PX_INLINE virtual								~PxControllerDesc();
public:

	/**
	\brief returns true if the current settings are valid

	\return True if the descriptor is valid.
	*/
	PX_INLINE virtual	bool						isValid()		const;

	/**
	\brief Returns the character controller type

	\return The controllers type.

	@see PxControllerType PxCapsuleControllerDesc PxBoxControllerDesc
	*/
	PX_INLINE			PxControllerShapeType::Enum		getType()		const	{ return type;		}

	/**
	\brief The position of the character

	<b>Default:</b> Zero
	*/
	PxExtendedVec3				position;

	/**
	\brief Specifies the 'up' direction

	In order to provide stepping functionality the SDK must be informed about the up direction.

	\li PxCCTUpAxis::eX => (1, 0, 0)
	\li PxCCTUpAxis::eY => (0, 1, 0)
	\li PxCCTUpAxis::eZ => (0, 0, 1)

	<b>Default:</b> PxCCTUpAxis::eY

	*/
	PxCCTUpAxis::Enum			upDirection;

	/**
	\brief The maximum slope which the character can walk up.

	In general it is desirable to limit where the character can walk, in particular it is unrealistic
	for the character to be able to climb arbitary slopes.

	The limit is expressed as the cosine of desired limit angle. A value of 0 disables this feature.

	<b>Default:</b> 0.707

	@see upDirection invisibleWallHeight maxJumpHeight
	*/
	PxF32						slopeLimit;

	/**
	\brief Height of invisible walls created around non-walkable triangles

	The library can automatically create invisible walls around non-walkable triangles defined
	by the 'slopeLimit' parameter. This defines the height of those walls. If it is 0.0, then
	no extra triangles are created.

	<b>Default:</b> 0.0

	@see upDirection slopeLimit maxJumpHeight
	*/
	PxF32						invisibleWallHeight;

	/**
	\brief Maximum height a jumping character can reach

	This is only used if invisible walls are created ('invisibleWallHeight' is non zero).

	When a character jumps, the non-walkable triangles he might fly over are not found
	by the collision queries (since the character's bounding volume does not touch them).
	Thus those non-walkable triangles do not create invisible walls, and it is possible
	for a jumping character to land on a non-walkable triangle, while he wouldn't have
	reached that place by just walking.

	The 'maxJumpHeight' variable is used to extend the size of the collision volume
	downward. This way, all the non-walkable triangles are properly found by the collision
	queries and it becomes impossible to 'jump over' invisible walls.

	If the character in your game can not jump, it is safe to use 0.0 here. Otherwise it
	is best to keep this value as small as possible, since a larger collision volume
	means more triangles to process.

	<b>Default:</b> 0.0

	@see upDirection slopeLimit invisibleWallHeight
	*/
	PxF32						maxJumpHeight;

	/**
	\brief The contact offset used by the controller.

	Specifies a skin around the object within which contacts will be generated.
	Use it to avoid numerical precision issues.

	This is dependant on the scale of the users world, but should be a small, positive 
	non zero value.

	<b>Default:</b> 0.1
	*/
	PxF32						contactOffset;

	/**
	\brief Defines the maximum height of an obstacle which the character can climb.

	A small value will mean that the character gets stuck and cannot walk up stairs etc, 
	a value which is too large will mean that the character can climb over unrealistically 
	high obstacles.

	<b>Default:</b> 0.5

	@see upDirection 
	*/
	PxF32						stepOffset;

	/**
	\brief Specifies a user callback interface.

	This callback interface is called when the character collides with shapes and other characters.

	Setting this to NULL disables callbacks.

	<b>Default:</b> NULL

	@see PxUserControllerHitReport
	*/
	PxUserControllerHitReport*	callback;

	/**
	\brief The interaction flag controls if a character controller collides with other controllers.

	The default is to collide controllers.

	<b>Default:</b> PxCCTInteractionMode::eINCLUDE

	@see PxCCTInteractionMode
	*/
	PxCCTInteractionMode::Enum	interactionMode;

	/**
	\brief The material for the actor associated with the controller.
	
	The controller internally creates a rigid body actor. This parameter specifies the material of the actor.

	<b>Default:</b> NULL

	@see PxMaterial
	*/
	PxMaterial*					material;

	/**
	\brief User specified data associated with the controller.

	<b>Default:</b> NULL
	*/
	void*						userData;
};

PX_INLINE PxControllerDesc::PxControllerDesc(PxControllerShapeType::Enum t) : type(t)
{
	upDirection			= PxCCTUpAxis::eY;
	slopeLimit			= 0.707f;
	contactOffset		= 0.1f;
	stepOffset			= 0.5f;
	callback			= NULL;
	userData			= NULL;
	interactionMode		= PxCCTInteractionMode::eINCLUDE;
	position.x			= PxExtended(0.0);
	position.y			= PxExtended(0.0);
	position.z			= PxExtended(0.0);
	material			= NULL;
	invisibleWallHeight	= 0.0f;
	maxJumpHeight		= 0.0f;
}

PX_INLINE PxControllerDesc::~PxControllerDesc()
{
}

PX_INLINE bool PxControllerDesc::isValid() const
{
	if (slopeLimit<0)		return false;
	if (stepOffset<0)		return false;
	if (contactOffset<0)	return false;
	if (material == NULL)	return false;
	return true;
}


/**
\brief Base class for character controllers.

@see PxCapsuleController PxBoxController
*/
class PxController
{
protected:
	PX_INLINE							PxController()					{}
	virtual								~PxController()					{}

public:

	/**
	\brief Releases the controller.
	*/
	virtual		void					release() = 0;

	/**
	\brief Moves the character using a "collide-and-slide" algorithm.

	\param[in] disp	a displacement vector
	\param[in] activeGroups	a filtering mask for collision groups. If a bit is set, corresponding group is active.
	\param[in] minDist the minimum travelled distance to consider. If travelled distance is smaller, the character doesn't move. 
	This is used to stop the recursive motion algorithm when remaining distance to travel is small.
	\param[out] collisionFlags returned collision flags, collection of ::PxControllerFlag
	\param[in] sharpness to prevent sudden height changes due to the autostep feature, the motion can be smoothed using a feedback filter.
	This coefficient defines the amount of smoothing. The smaller, the smoother. (1.0 means no smoothing).
	\param[in] filterData Alternative filter data used to filter shapes, see PxBatchQuery::overlapAABBMultiple().
	\param[in] filterCallback Custom filter logic to filter out colliding objects.
	*/
	virtual		void					move(const PxVec3& disp, PxU32 activeGroups, PxF32 minDist, PxU32& collisionFlags, PxF32 sharpness=1.0f, const PxFilterData* filterData=NULL, PxSceneQueryFilterCallback* filterCallback = NULL) = 0;

	/**
	\brief Resets controller's position.

	\warning this is a 'teleport' function, it doesn't check for collisions.

	To move the character under normal conditions use the #move() function.

	\param[in] position The new positon for the controller.
	\return Currently always returns true.

	@see PxControllerDesc.position getPosition() move()
	*/
	virtual		bool					setPosition(const PxExtendedVec3& position) = 0;
	/**
	\brief Retrieve the raw position of the controller.

	The position and filtered position are updated by calls to move(). Calling this method without calling
	move() will result in the last position or the initial position of the controller.

	\return The controllers position

	@see PxControllerDesc.position setPositon() move()
	*/
	virtual		const PxExtendedVec3&	getPosition()			const	= 0;

	/**
	\brief Get the rigid body actor associated with this controller (see PhysX documentation). 
	The behavior upon manually altering this actor is undefined, you should primarily 
	use it for reading const properties.

	\return the actor associated with the controller.
	*/
	virtual		PxRigidDynamic*			getActor()				const	= 0;

	/**
	\brief The step height.

	\param[in] offset The new step offset for the controller.

	@see PxControllerDesc.stepOffset
	*/
	virtual	    void					setStepOffset(const PxF32 offset) =0;

	/**
	\brief Retrieve the step height.

	\return The step offset for the controller.

	@see setStepOffset()
	*/
	virtual	    PxF32					getStepOffset()						const		=0;

	/**
	\brief Sets the interaction mode for the CCT.

	\param[in] flag The new value of the interaction mode.

	\see PxCCTInteractionMode
	*/
	virtual		void					setInteraction(PxCCTInteractionMode::Enum flag)	= 0;

	/**
	\brief Retrieves the interaction mode for the CCT.

	\return The current interaction mode.

	\see PxCCTInteractionMode
	*/
	virtual		PxCCTInteractionMode::Enum	getInteraction()				const		= 0;

	/**
	\brief Retrieve the contact offset.

	\return The contact offset for the controller.

	@see PxControllerDesc.contactOffset
	*/
	virtual	    PxF32					getContactOffset()					const		=0;

	/**
	\brief Retrieve the 'up' direction.

	\return The up direction for the controller.

	@see PxControllerDesc.upDirection
	*/
	virtual	    PxCCTUpAxis::Enum		getUpDirection()					const		=0;

	/**
	\brief Retrieve the slope limit.

	\return The slope limit for the controller.

	@see PxControllerDesc.slopeLimit
	*/
	virtual	    PxF32					getSlopeLimit()						const		=0;

	/**
	\brief The character controller uses caching in order to speed up collision testing, this caching can not detect when static objects have changed in the scene. You need to call this method when such changes have been made.
	*/
	virtual		void					reportSceneChanged()			= 0;

	/**
	\brief Returns the user data associated with this controller.

	\return The user pointer associated with the controller.

	@see PxControllerDesc.userData
	*/
	virtual		void*					getUserData()		const		= 0;

	/**
	\brief Return the type of controller

	@see PxControllerType
	*/
	virtual		PxControllerShapeType::Enum	getType()						= 0;
};

#ifndef PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif

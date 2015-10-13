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

#ifndef PX_COLLISION_CONTROLLER
#define PX_COLLISION_CONTROLLER

/* Exclude from documentation */
/** \cond */

#include "CctCharacterController.h"

namespace physx
{

class PxPhysics;
class PxScene;
class PxRigidDynamic;
class PxGeometry;
class PxMaterial;

//used to make gcc not complain about 'CCTS'
#define PX_MAKEFOURCC(a,b,c,d) ( ((PxU8)a) | (((PxU8)b)<< 8) | (((PxU8)c)<<16) | (((PxU8)d)<<24) ) 

namespace Cct
{
	class CharacterControllerManager;

	class Controller //: public UserAllocated
	{
	public:
												Controller(const PxControllerDesc& desc, PxScene* scene);
		virtual									~Controller();

					void						releaseInternal();

		virtual		bool						getWorldBox(PxExtendedBounds3& box)	const	= 0;
		virtual		PxController*				getNxController()							= 0;
		virtual		PxRigidDynamic*				getActor()			const	{ return mKineActor; };

					PxControllerShapeType::Enum	mType;
					PxCCTInteractionMode::Enum	mInteractionMode;
		// User params
					CCTParams					mUserParams;
					PxUserControllerHitReport*	mCallback;
					void*						mUserData;
		// Internal data
					SweepTest					mCctModule;			// Internal CCT object. Optim test for Ubi.
					PxRigidDynamic*				mKineActor;			// Associated kinematic actor
					PxExtendedVec3				mPosition;			// Current position
					PxExtendedVec3				mFilteredPosition;	// Current position after feedback filter
					PxExtendedVec3				mExposedPosition;	// Position visible from the outside at any given time
					PxExtended					mMemory;			// Memory variable for feedback filter
					PxScene*					mScene;				// Handy scene owner
					CharacterControllerManager*	mManager;			// Owner manager
	protected:
		// Internal methods
					bool						createProxyActor(PxPhysics& sdk, const PxTransform& transform, const PxGeometry& geometry, const PxMaterial& material, PxF32 density);
					bool						setPos(const PxExtendedVec3& pos);
					void						move(SweptVolume& volume, const PxVec3& disp, PxU32 activeGroups, PxF32 minDist, PxU32& collisionFlags, PxF32 sharpness, const PxFilterData* filterData, PxSceneQueryFilterCallback* filterCallback, bool constrainedClimbingMode);
	PX_FORCE_INLINE	void						setInteraction(PxCCTInteractionMode::Enum flag)		{ mInteractionMode = flag;	}
	PX_FORCE_INLINE	PxCCTInteractionMode::Enum	getInteraction()							const	{ return mInteractionMode;	}
	};

} // namespace Cct

}

/** \endcond */
#endif

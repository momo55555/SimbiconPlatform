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

#ifndef PX_COLLISION_CAPSULECONTROLLER
#define PX_COLLISION_CAPSULECONTROLLER

/* Exclude from documentation */
/** \cond */

#include "CctController.h"
#include "PxCapsuleController.h"

namespace physx
{

class PxPhysics;

namespace Cct
{

	class CapsuleController : public PxCapsuleController, public Controller
	{
	public:
											CapsuleController(const PxControllerDesc& desc, PxPhysics& sdk, PxScene* scene);
		virtual								~CapsuleController();

		virtual void						release()										{ Controller::releaseInternal();		}

		virtual	PxController*				getNxController()								{ return this;							}
		virtual	PxControllerShapeType::Enum	getType()										{ return mType;							}

		virtual	void						move(const PxVec3& disp, PxU32 activeGroups, PxF32 minDist, PxU32& collisionFlags, PxF32 sharpness, const PxFilterData* filterData, PxSceneQueryFilterCallback* filterCallback);

		virtual	bool						setPosition(const PxExtendedVec3& position)		{ return setPos(position);				}

		virtual	void						setStepOffset(const float offset)				{ mUserParams.mStepOffset = offset;		}
		virtual	PxF32						getStepOffset()						const		{ return mUserParams.mStepOffset;		}

		virtual PxF32						getContactOffset()					const		{ return mUserParams.mContactOffset;	}
		virtual PxCCTUpAxis::Enum			getUpDirection()					const		{ return mUserParams.mUpDirection;		}
		virtual PxF32						getSlopeLimit()						const		{ return mUserParams.mSlopeLimit;		}

		virtual	PxRigidDynamic*				getActor()							const		{ return Controller::getActor();		}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		// PxCapsuleController
		virtual	PxF32						getRadius()							const		{ return mRadius;						}
		virtual	PxF32						getHeight()							const		{ return mHeight;						}
		virtual	PxCapsuleClimbingMode::Enum	getClimbingMode()					const;
		virtual	bool						setRadius(PxF32 radius);
		virtual	bool						setHeight(PxF32 height);
		virtual	bool						setClimbingMode(PxCapsuleClimbingMode::Enum);
		//~ PxCapsuleController

		virtual	const PxExtendedVec3&		getPosition()						const		{ return mExposedPosition;				}
		virtual const PxExtendedVec3&		getDebugPosition()					const		{ return mPosition;						}
		virtual	bool						getWorldBox(PxExtendedBounds3& box) const;
		virtual	void						setInteraction(PxCCTInteractionMode::Enum flag)	{ Controller::setInteraction(flag);		}
		virtual	PxCCTInteractionMode::Enum	getInteraction()					const		{ return Controller::getInteraction();	}

		virtual	void						reportSceneChanged();
		virtual	void*						getUserData()						const		{ return mUserData;						}

				PxF32						mRadius;
				PxF32						mHeight;
				PxCapsuleClimbingMode::Enum	mClimbingMode;
	};

} // namespace Cct

}

/** \endcond */
#endif

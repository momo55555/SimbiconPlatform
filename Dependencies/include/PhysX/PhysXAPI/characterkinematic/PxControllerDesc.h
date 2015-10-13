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



#ifndef PX_CONTROLLER_DESC_H
#define PX_CONTROLLER_DESC_H

/** \addtogroup character
  @{
*/

#include <characterkinematic/PxController.h>
#include <PxActorDesc.h>
#include <PxFiltering.h>

#ifndef PX_DOXYGEN
namespace physx
{
#endif

class PxScene;

class PxControllerShapeDesc;

class PxControllerDesc
{
	public:
		/**
		\brief what scene the controller will be interacting with...
		*/
		PxScene&						scene;

		/**
		\brief what interaction method does the controller use with the scene.
		*/
		PxControllerType::Enum			type;

		/**
		\brief specifies interaction priority for one-way interaction support.
		\brief  default value 0
		*/
		PxDominanceGroup				dominanceGroup;

		/**
		\brief simulation filter data for the character controller.
		\brief  note: the app should be capable of figuring out if a particular filter data
		\brief  is for a character or not, so probably reserve a 'isCharacter' bit.
		*/
		PxFilterData					controllerSimulationFilterData;

		/**
		\brief the collision shapes of the character controller (for now support only 1 shape).
		*/
		const PxControllerShapeDesc*	shapeDesc;

		/**
		\brief the initial pose of the character controller.
		*/
		PxTransform						globalPose;

		/**
		\brief density for proxy shape
		*/
		PxReal							density;

	public:
		PxControllerDesc(PxScene& scene);
		
		void setToDefault(void);

		bool isValid(void) const;
};

#ifndef PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif

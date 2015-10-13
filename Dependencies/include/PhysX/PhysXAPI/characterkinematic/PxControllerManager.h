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


#ifndef PX_CHARACTER_NXCONTROLLERMANAGER
#define PX_CHARACTER_NXCONTROLLERMANAGER
/** \addtogroup character
  @{
*/

#include "characterkinematic/PxCharacter.h"

#include "PxPhysX.h"
#include "common/PxRenderBuffer.h"
#include "PxFoundation.h"

#ifndef PX_DOXYGEN
namespace physx
{
#endif

class PxPhysics;
class PxScene;
class PxController;
class PxControllerDesc;
class PxControllerManager;


/**
\brief Manages an array of character controllers.

@see PxController PxBoxController PxCapsuleController
*/
class PX_PHYSX_CHARACTER_API PxControllerManager {
public:
	/**
	\brief Releases the controller manager.
	*/
	virtual void			release() = 0;

	/**
	\brief Returns the number of controllers that are being managed.

	\return The number of controllers.
	*/
	virtual PxU32			getNbControllers() const = 0;

	/**
	\brief Retrieve one of the controllers in the manager.

	\param index the index of the controller to return
	\return an array of controller pointers with size getNbControllers().
	*/
	virtual PxController*	getController(PxU32 index) = 0;

	/**
	\brief Creates a new character controller.

	\param[in] sdk The Physics sdk object
	\param[in] scene The scene that the controller will belong to.
	\param[in] desc The controllers descriptor
	\return The new controller

	@see PxController PxController.release() PxControllerDesc
	*/
	virtual PxController*	createController(PxPhysics& sdk, PxScene* scene, const PxControllerDesc& desc) = 0;

	/**
	\brief Releases all the controllers that are being managed.
	*/
	virtual void			purgeControllers() = 0;

	/**
	\brief Updates the exposed position from the filtered position of all controllers.
	*/
	virtual void			updateControllers() = 0;

	/**
	\brief Retrieves debug data. Note that debug rendering is not enabled until this method is called.
	*/
	virtual	PxRenderBuffer&	getRenderBuffer()		= 0;

protected:
	PxControllerManager() {}
	virtual ~PxControllerManager() {}
};

#ifndef PX_DOXYGEN
} // namespace physx
#endif

PX_C_EXPORT PX_PHYSX_CHARACTER_API physx::PxControllerManager* PX_CALL_CONV PxCreateControllerManager(physx::PxFoundation& foundation);

/** @} */
#endif //PX_CHARACTER_NXCONTROLLERMANAGER

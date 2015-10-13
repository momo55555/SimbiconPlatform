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

#ifndef PX_CHARACTER_CONTROLLERMANAGER
#define PX_CHARACTER_CONTROLLERMANAGER

//Exclude file from docs
/** \cond */

#include "PxControllerManager.h"
#include "GuGeomUtils.h"
#include "GuMeshQuery.h"
#include "CmRenderOutput.h"

namespace physx
{
namespace Cct
{

	class Controller;
	class ControllerArray;

	//Implements the PxControllerManager interface, this class used to be called ControllerManager
	class CharacterControllerManager: public PxControllerManager
	{
	public:
								CharacterControllerManager(PxAllocatorCallback* userAlloc);
		virtual					~CharacterControllerManager();

		PxU32					getNbControllers()	const;
		PxController*			getController(PxU32 index);
		Controller**			getControllers();
		PxController*			createController(PxPhysics& sdk, PxScene* scene, const PxControllerDesc& desc);
		void					releaseController(PxController& controller);
		void					purgeControllers();
		void					updateControllers();
		void					release();
		PxRenderBuffer&			getRenderBuffer();

		void					printStats();

		PX_INLINE bool			isValid();

		Cm::RenderBuffer*		mRenderBuffer;
	protected:
		ControllerArray*		mControllers;
		PxAllocatorCallback*	mAllocator;
	};

} // namespace Cct

PX_INLINE bool Cct::CharacterControllerManager::isValid()
{
	return ((mControllers) && (mAllocator));
}

}

/** \endcond */
#endif //PX_CHARACTER_CONTROLLERMANAGER

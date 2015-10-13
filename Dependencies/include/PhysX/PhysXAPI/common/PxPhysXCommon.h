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


#ifndef PX_PHYSICS_COMMON_NX
#define PX_PHYSICS_COMMON_NX

/** \addtogroup common 
@{ */

#include "Px.h"

#define PX_PHYSICS_VERSION_MAJOR 3
#define PX_PHYSICS_VERSION_MINOR 1
#define PX_PHYSICS_VERSION_BUGFIX 0

/** Pass this constant to the PxCreatePhysics function. */
#define PX_PHYSICS_VERSION ((PX_PHYSICS_VERSION_MAJOR<<24) + (PX_PHYSICS_VERSION_MINOR<<16) + (PX_PHYSICS_VERSION_BUGFIX<<8) + 0)

// define API function declaration (public API only needed because of extensions)
#if defined PX_PHYSX_STATIC_LIB || defined PX_PHYSX_CORE_STATIC_LIB
	#define PX_PHYSX_CORE_API
#else
	#if defined(PX_WINDOWS)
		#if defined PX_PHYSX_CORE_EXPORTS
			#define PX_PHYSX_CORE_API __declspec(dllexport)
		#else
			#define PX_PHYSX_CORE_API __declspec(dllimport)
		#endif
    #elif defined(PX_LINUX) && defined(PX_LINUX_USE_VISIBILITY)
		#define PX_PHYSX_CORE_API __attribute__ ((visibility ("default")))
    #else
		#define PX_PHYSX_CORE_API 
    #endif
#endif

// Changing these parameters requires recompilation of the SDK


// Support GPU PhysX
#if defined(PX_WINDOWS)
#define PX_SUPPORT_GPU_PHYSX 1
#else
#define PX_SUPPORT_GPU_PHYSX 0
#endif

namespace physx
{
	// PX_SERIALIZATION
	class PxRefResolver;
	class PxUserReferences;
	class PxCollection;
	class PxSerializable;
	//~PX_SERIALIZATION

	// alias to hide foundation version (needed for external libraries)
	namespace pubfnd = pubfnd3;

	// pull public foundation into physx namespace
	using namespace pubfnd3;
} // namespace physx

// we need this until all our code lives in physx namespace
// using namespace physx;

/** @} */
#endif

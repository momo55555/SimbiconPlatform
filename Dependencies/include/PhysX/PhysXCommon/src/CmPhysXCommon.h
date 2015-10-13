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


#ifndef PX_PHYSICS_COMMON
#define PX_PHYSICS_COMMON

//! \file Top level internal include file for PhysX SDK

#include "Ps.h"

// Enable debug visualization
#define PX_ENABLE_DEBUG_VISUALIZATION	1

// Enable simulation statistics generation
#define PX_ENABLE_SIM_STATS 1

// PT: typical "invalid" value in various CD algorithms
#define	PX_INVALID_U32		0xffffffff
#define PX_INVALID_U16		0xffff

// PT: this used to be replicated everywhere in the code, causing bugs to sometimes reappear (e.g. TTP 3587).
// It is better to define it in a header and use the same constant everywhere. The original value (1e-05f)
// caused troubles (e.g. TTP 1705, TTP 306).
#define PX_PARALLEL_TOLERANCE	1e-02f

// This is version number for cloth fabric 
// starting from version number 1
#define PX_CLOTH_FABRIC_VERSION 1


#ifndef PX_SIMD_CONTACTGENERATION
#define PX_SIMD_CONTACTGENERATION 0
#endif


#ifndef PERSISTENT_CONTACT_MANIFOLD
#define PERSISTENT_CONTACT_MANIFOLD 0
#endif


#ifndef CACHE_LOCAL_CONTACTS_XP
	#if PERSISTENT_CONTACT_MANIFOLD
		#define CACHE_LOCAL_CONTACTS_XP 0
	#else
		#define CACHE_LOCAL_CONTACTS_XP 1
	#endif
#endif

namespace physx
{
	// alias to hide foundation version (needed for external libraries)
	namespace shdfnd = shdfnd3;

	// alias shared foundation to something usable
	namespace Ps = shdfnd3;

	// pull public foundation into physx namespace
	using namespace pubfnd3;
}

// we need this until all our code lives in physx namespace
// using namespace physx;


#endif

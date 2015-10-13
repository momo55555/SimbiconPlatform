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


#ifndef PX_PHYSICS_EXTENSIONS_DEFAULTSIMULATIONFILTERSHADER_H
#define PX_PHYSICS_EXTENSIONS_DEFAULTSIMULATIONFILTERSHADER_H
/** \addtogroup extensions
  @{
*/

#include "PxPhysX.h"

#include "PxFiltering.h"

#ifndef PX_DOXYGEN
namespace physx
{
#endif

/**
\brief Implementation of a simple filter shader

This shader provides the following logic:
\li If one of the two filter objects is a trigger, the pair is acccepted and #PxPairFlag::eTRIGGER_DEFAULT will be used for trigger reports
\li Else, if the filter mask logic (see further below) discards the pair it will be suppressed (#PxFilterFlag::eSUPPRESS)
\li Else, the pair gets accepted and collision response gets enabled (#PxPairFlag::eCONTACT_DEFAULT)

Filter mask logic:
Given the two #PxFilterData structures fd0 and fd1 of two collision objects, the pair passes the filter if the following
condition is met:

(fd0.word0 == fd0.word1 == 0)
OR
(fd1.word0 == fd1.word1 == 0)
OR
(fd0.word0 bitAND fd1.word1) 
OR
(fd1.word0 bitAND fd0.word1)

@see PxSimulationFilterShader
*/

PxFilterFlags PxDefaultSimulationFilterShader(
	PxFilterObjectAttributes attributes0,
	PxFilterData filterData0, 
	PxFilterObjectAttributes attributes1,
	PxFilterData filterData1,
	PxPairFlags& pairFlags,
	const void* constantBlock,
	PxU32 constantBlockSize);

#ifndef PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif

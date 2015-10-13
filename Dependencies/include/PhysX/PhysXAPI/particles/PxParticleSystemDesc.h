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


#ifndef PX_PHYSICS_PX_PARTICLESYSTEMDESC
#define PX_PHYSICS_PX_PARTICLESYSTEMDESC
/** \addtogroup particles
@{
*/

#include "particles/PxParticleBaseDesc.h"
#include "particles/PxParticleReadData.h"

#ifndef PX_DOXYGEN
namespace physx
{
#endif

/**
\brief Particle System Descriptor. This structure is used to save and load the state of #PxParticleSystem objects.
*/

class PxParticleSystemDesc : public PxParticleBaseDesc
{
public:
	/**
	\brief Constructor sets to default.
	*/
	PX_INLINE PxParticleSystemDesc(const PxTolerancesScale& scale) : 
		PxParticleBaseDesc(scale, PxActorType::ePARTICLE_SYSTEM)
	{}

	/**
	\brief (Re)sets the structure to the default.	
	*/
	PX_INLINE void setToDefault(const PxTolerancesScale& scale)
	{
		*this = PxParticleSystemDesc(scale);
	}

	/**
	\brief Returns true if the current settings are valid
	*/
	PX_INLINE bool isValid() const
	{
		return PxParticleBaseDesc::isValid();
	}

	/**
	\brief virtual destructor
	*/
	virtual ~PxParticleSystemDesc() {};

};

#ifndef PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif

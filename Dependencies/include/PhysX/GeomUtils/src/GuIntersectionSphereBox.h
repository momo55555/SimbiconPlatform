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


#ifndef PX_INTERSECTION_SPHERE_BOX_H
#define PX_INTERSECTION_SPHERE_BOX_H

#include "PxBounds3.h"
#include "CmPhysXCommon.h"
#include "PsVecMath.h"

namespace physx
{
namespace Gu
{
	/**
	 \brief computes a sphere-AABB intersection. Based on Jim Arvo's code.
	 */
	bool intersectSphereAABB(const PxVec3& center, float radius, const PxVec3& minimum, const PxVec3& maximum);

	bool intersectSphereAABB(const Ps::aos::Vec3VArg center, const Ps::aos::FloatVArg radius, const Ps::aos::Vec3VArg minimum, const Ps::aos::Vec3VArg maximum);

	PX_INLINE bool intersectSphereAABB(const PxVec3& center, float radius, const PxBounds3& aabb)
	{
		return intersectSphereAABB(center, radius, aabb.minimum, aabb.maximum);
	}

	bool intersectSphereOBB(const Ps::aos::Vec3VArg center, const Ps::aos::FloatVArg radius, const Ps::aos::Vec3VArg ocenter, const Ps::aos::Vec3VArg extents, const Ps::aos::Mat33V& rot33, Ps::aos::Vec3V& p, Ps::aos::Vec3V& normal, Ps::aos::FloatV& depth);

} // namespace Gu

}

#endif
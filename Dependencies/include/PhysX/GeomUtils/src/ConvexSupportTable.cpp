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

#include "GuVecSphere.h"
#include "GuVecBox.h"
#include "GuVecCapsule.h"
#include "GuVecConvexHull.h"
#include "GuVecCylinder.h"
#include "GuVecCone.h"
#include "ConvexSupportTable.h"

namespace physx
{
namespace Gu
{
	Ps::aos::Vec3V BoxSupport(const ConvexV& a, const Ps::aos::Vec3VArg direction)
	{
		const BoxV& box = (const BoxV&)a;
		return box.support(direction);
	}

	Ps::aos::Vec3V SphereSupport(const ConvexV& a, const Ps::aos::Vec3VArg direction)
	{
		const SphereV& sphere = (const SphereV&)a;
		return sphere.support(direction);
	}

	Ps::aos::Vec3V CapsuleSupport(const ConvexV& a, const Ps::aos::Vec3VArg direction)
	{
		const CapsuleV& capsule = (const CapsuleV&)a;
		return capsule.support(direction);
	}

	Ps::aos::Vec3V CylinderSupport(const ConvexV& a, const Ps::aos::Vec3VArg direction)
	{
		const CylinderV& cylinder = (const CylinderV&)a;
		return cylinder.support(direction);
	}

	Ps::aos::Vec3V ConeSupport(const ConvexV& a, const Ps::aos::Vec3VArg direction)
	{
		const ConeV& cone = (const ConeV&)a;
		return cone.support(direction);
	}

	Ps::aos::Vec3V ConvexHullSupport(const ConvexV& a, const Ps::aos::Vec3VArg direction)
	{
		const ConvexHullV& convex = (const ConvexHullV&)a;
		return convex.support(direction);
	}

	Ps::aos::Vec3V BigConvexHullSupport(const ConvexV& a, const Ps::aos::Vec3VArg direction)
	{
		const BigConvexHullV& convex = (const BigConvexHullV&)a;
		return convex.support(direction);
	}

	Ps::aos::Vec3V BoxSupportMargin(const ConvexV& a, const Ps::aos::Vec3VArg direction, const Ps::aos::FloatVArg margin)
	{
		const BoxV& box = (const BoxV&)a;
		return box.supportMargin(direction, margin);
	}

	Ps::aos::Vec3V SphereSupportMargin(const ConvexV& a, const Ps::aos::Vec3VArg direction,  const Ps::aos::FloatVArg margin)
	{
		const SphereV& sphere = (const SphereV&)a;
		return sphere.supportMargin(direction, margin);
	}

	Ps::aos::Vec3V CapsuleSupportMargin(const ConvexV& a, const Ps::aos::Vec3VArg direction,  const Ps::aos::FloatVArg margin)
	{
		const CapsuleV& capsule = (const CapsuleV&)a;
		return capsule.supportMargin(direction, margin);
	}

	Ps::aos::Vec3V CylinderSupportMargin(const ConvexV& a, const Ps::aos::Vec3VArg direction,  const Ps::aos::FloatVArg margin)
	{
		const CylinderV& cylinder = (const CylinderV&)a;
		return cylinder.supportMargin(direction, margin);
	}

	Ps::aos::Vec3V ConeSupportMargin(const ConvexV& a, const Ps::aos::Vec3VArg direction,  const Ps::aos::FloatVArg margin)
	{
		const ConeV& cone = (const ConeV&)a;
		return cone.supportMargin(direction, margin);
	}

	Ps::aos::Vec3V ConvexHullSupportMargin(const ConvexV& a, const Ps::aos::Vec3VArg direction,  const Ps::aos::FloatVArg margin)
	{
		const ConvexHullV& convex = (const ConvexHullV&)a;
		return convex.supportMargin(direction, margin);
	}

	Ps::aos::Vec3V BigConvexHullSupportMargin(const ConvexV& a, const Ps::aos::Vec3VArg direction,  const Ps::aos::FloatVArg margin)
	{
		const BigConvexHullV& convex = (const BigConvexHullV&)a;
		return convex.supportMargin(direction, margin);
	}
}

}

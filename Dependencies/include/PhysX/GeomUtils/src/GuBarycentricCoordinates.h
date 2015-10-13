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


#ifndef PX_BARYCENTRIC_COORDINATES_H
#define PX_BARYCENTRIC_COORDINATES_H

#include "CmPhysXCommon.h"
#include "PsVecMath.h"

namespace physx
{
namespace Gu
{
	void barycentricCoordinates(const Ps::aos::Vec3VArg p, 
		const Ps::aos::Vec3VArg a, 
		const Ps::aos::Vec3VArg b, 
		const Ps::aos::Vec3VArg c, 
		Ps::aos::FloatV& v, 
		Ps::aos::FloatV& w);

	void barycentricCoordinates(const Ps::aos::Vec3VArg v0, 
		const Ps::aos::Vec3VArg v1, 
		const Ps::aos::Vec3VArg v2,  
		Ps::aos::FloatV& v, 
		Ps::aos::FloatV& w);

	inline Ps::aos::BoolV isValidTriangleBarycentricCoord(const Ps::aos::FloatVArg v, const Ps::aos::FloatVArg w)
	{
		using namespace Ps::aos;
		const FloatV zero = FZero();
		const FloatV one = FOne();

		const BoolV con0 = BAnd(FIsGrtrOrEq(v, zero), FIsGrtrOrEq(one, v));
		const BoolV con1 = BAnd(FIsGrtrOrEq(w, zero), FIsGrtrOrEq(one, w));
		const BoolV con2 = FIsGrtr(one, FAdd(v, w));
		return BAnd(con0, BAnd(con1, con2));
	}
} // namespace Gu

}

#endif
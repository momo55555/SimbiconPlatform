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


#include "PsIntrinsics.h"
#include "GuGeomUtilsInternal.h"
#include "GuBox.h"
#include "GuCapsule.h"
#include "PxCapsuleGeometry.h"

using namespace physx;

// PT: TODO: unify those functions

/**
*	Computes an OBB surrounding the capsule.
*	\param		box		[out] the OBB
*/
void Gu::computeBoxAroundCapsule(const Gu::Capsule& capsule, Gu::Box& box)
{
	// Box center = center of the two capsule's endpoints
	box.center = capsule.computeCenter();

	// Box extents
	const PxF32 d = (capsule.p0 - capsule.p1).magnitude();
	box.extents.x = capsule.radius + (d * 0.5f);
	box.extents.y = capsule.radius;
	box.extents.z = capsule.radius;

	// Box orientation
	if(d==0.0f)
	{
		box.rot = PxMat33::createIdentity();
	}
	else
	{
		PxVec3 dir, right, up;
		computeBasis(capsule.p0, capsule.p1, dir, right, up);
		box.setAxes(dir, right, up);
	}
}

//void Gu::computeBoxAroundCapsule(const CapsuleV& capsule, BoxV& box)
//{
//	//Need to test!!!!!
//	using namespace Ps::aos;
//	const FloatV half = FloatV_From_F32(0.5f);
//	const FloatV r =capsule.radius;
//	const FloatV zero = FZero();
//
//	box.setCenter(capsule.getCenter());
//
//	// Box extents
//	const FloatV d = V3Length(V3Sub(capsule.p0, capsule.p1));
//	box.extents = V3Splat(r);
//	const FloatV x = FMulAdd(d, half, r);
//	box.extents = V3SetX(box.extents, x);
//
//	Vec3V dir, right, up;
//	computeBasis(capsule.p0, capsule.p1, dir, right, up);
//
//	const BoolV con = FIsGrtr(d, zero);
//	const Vec3V dirV = V3Sel(con, V3UnitX(), dir);
//	const Vec3V rightV = V3Sel(con, V3UnitY(), dir);
//	const Vec3V upV = V3Sel(con, V3UnitZ(), dir);
//	box.setAxes(dir, right, up);
//
//}
/**
*	Computes an OBB surrounding the capsule.
*/
void Gu::computeBoxAroundCapsule(const PxCapsuleGeometry& capsuleGeom, const PxTransform& capsulePose, Box& box)
{
	box.center = capsulePose.p;

	// Box extents
	box.extents.x = capsuleGeom.radius + (capsuleGeom.halfHeight);
	box.extents.y = capsuleGeom.radius;
	box.extents.z = capsuleGeom.radius;

	// Box orientation
	if(capsuleGeom.halfHeight == 0.0f)
	{
		box.rot = PxMat33::createIdentity();
	}
	else
	{
		const PxVec3 dir = capsulePose.q.getBasisVector0();
		PxVec3 right, up;
		computeBasis(dir, right, up);
		box.setAxes(dir, right, up);
	}
}

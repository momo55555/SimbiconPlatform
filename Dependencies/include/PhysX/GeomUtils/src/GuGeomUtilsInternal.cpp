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
#include "PxMat33Legacy.h"
#include "GuPlane.h"
#include "GuCapsule.h"
#include "PsMathUtils.h"
#include "PxCapsuleGeometry.h"

using namespace physx;

// ----------------------------------------------------------------------------------------

/**
Computes the aabb points.
\param		pts	[out] 8 box points
*/
void Gu::computeBoxPoints(const PxBounds3& bounds, PxVec3* PX_RESTRICT pts)
{
	PX_ASSERT(pts);

	// Get box corners
	const PxVec3& minimum = bounds.minimum;
	const PxVec3& maximum = bounds.maximum;

	//     7+------+6			0 = ---
	//     /|     /|			1 = +--
	//    / |    / |			2 = ++-
	//   / 4+---/--+5			3 = -+-
	// 3+------+2 /    y   z	4 = --+
	//  | /    | /     |  /		5 = +-+
	//  |/     |/      |/		6 = +++
	// 0+------+1      *---x	7 = -++

	// Generate 8 corners of the bbox
	pts[0] = PxVec3(minimum.x, minimum.y, minimum.z);
	pts[1] = PxVec3(maximum.x, minimum.y, minimum.z);
	pts[2] = PxVec3(maximum.x, maximum.y, minimum.z);
	pts[3] = PxVec3(minimum.x, maximum.y, minimum.z);
	pts[4] = PxVec3(minimum.x, minimum.y, maximum.z);
	pts[5] = PxVec3(maximum.x, minimum.y, maximum.z);
	pts[6] = PxVec3(maximum.x, maximum.y, maximum.z);
	pts[7] = PxVec3(minimum.x, maximum.y, maximum.z);
}

//---------------------------------------------------------------------------

void Gu::getWorldSegment(Gu::Segment& worldSegment, const PxCapsuleGeometry& capsuleGeom, const PxTransform& pose)
{
	const PxVec3 tmp = pose.q.getBasisVector0() * capsuleGeom.halfHeight; 
	worldSegment.p1 = pose.p + tmp;
	worldSegment.p0 = pose.p - tmp;
}

void Gu::getWorldSegment(Gu::SegmentV& worldSegment, const PxCapsuleGeometry& capsuleGeom, const PxTransform& pose)
{
	using namespace Ps::aos;
	const PxVec3 tmp = pose.q.getBasisVector0()*capsuleGeom.halfHeight;
	const PxVec3 p0 = pose.p - tmp;
	const PxVec3 p1 = pose.p + tmp;
	worldSegment.p1 = Vec3V_From_PxVec3(p1);
	worldSegment.p0 = Vec3V_From_PxVec3(p0);
}


void Gu::getWorldCapsule(Gu::Capsule& worldCapsule, const PxCapsuleGeometry& capsuleGeom, const PxTransform& pose)
{
	getWorldSegment(worldCapsule, capsuleGeom, pose);
	worldCapsule.radius = capsuleGeom.radius;
}

Gu::Plane Gu::getPlane(const PxTransform& pose)
{ 
	const PxVec3 n = pose.q.getBasisVector0();
	return Gu::Plane(n, -pose.p.dot(n)); 
}

Gu::PlaneV Gu::getPlaneV(const PxTransform& pose)
{ 
	using namespace Ps::aos;
	const PxVec3 n = pose.q.getBasisVector0();
	const PxF32 d = -pose.p.dot(n);
	const Vec3V nv = Vec3V_From_PxVec3(n);
	const FloatV dv = FloatV_From_F32(d);
	return Gu::PlaneV(nv, dv); 
}

PxTransform Gu::getWorldTransform(const Gu::Capsule& worldCapsule, PxReal& halfHeight)
{
	PxTransform capsuleTransform;
	PxVec3 dir = worldCapsule.computeDirection();
	capsuleTransform.p = worldCapsule.getOrigin() + dir * 0.5f;
	halfHeight = 0.5f * dir.normalize();

	if (halfHeight > PX_EPS_F32)	//else it is just a sphere.
	{
	//angle axis representation is the rotation from the world x axis to dir
		PxVec3 t1, t2;
		Ps::normalToTangents(dir, t1, t2);
		PxMat33Legacy x;
		x.setColumn(0, dir);
		x.setColumn(1, t1);
		x.setColumn(2, t2);
		capsuleTransform.q = PxQuat(x.toPxMat33());
	}
	else
	{
		capsuleTransform.q = PxQuat::createIdentity();
	}
	//this won't ever be exactly the same thing as the original because we lost some info on a DOF

	return capsuleTransform;

	// Instead we could do the following:
	//
	//if (halfHeight > PX_EPS_F32)	//else it is just a sphere.
	//{
	//	PxVec3 axis = dir.cross(PxVec3(1.0f, 0.0f, 0.0f));
	//	PxReal sinTheta = axis.normalize();
	//	PxReal cosTheta = dir.x;	//dir|PxVec3(1.0f, 0.0f, 0.0f)
	//
	//	To build the quaternion we need sin(a/2), cos(a/2)
	//  The formulas are:
	//
	//	sin(a/2) = +/- sqrt([1 - cos(a)] / 2)
	//	cos(a/2) = +/- sqrt([1 + cos(a)] / 2)
	//
	// The problem is to find the correct signs based on the spatial relation between
	// the capsule direction and (1,0,0)
	// Not sure this is more efficient and anyway, we want to ditch this method as soon as possible
	//
	//	quat.x = axis.x * sin(a/2)
	//	quat.y = axis.y * sin(a/2)
	//	quat.z = axis.z * sin(a/2)
	//	quat.w = cos(a/2)
}

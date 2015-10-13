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


#include "GuGeometryQuery.h"
#include "GuGeomUtilsInternal.h"	// For getWorldCapsule
#include "PxSphereGeometry.h"
#include "PxBoxGeometry.h"
#include "PxCapsuleGeometry.h"
#include "PxConvexMeshGeometry.h"
#include "GuOverlapTests.h"
#include "GuSweepTests.h"
#include "GuRaycastTests.h"
#include "GuBoxConversion.h"
#include "PxSceneQueryReport.h"
#include "GuInternalTriangleMesh.h"

using namespace physx;

bool Gu::GeometryQuery::sweep(const PxVec3& unitDir,
							  const PxReal distance,
							  const PxGeometry& geom0,
							  const PxTransform& pose0,
							  const PxGeometry& geom1,
							  const PxTransform& pose1,
							  PxSweepHit& sweepHit,
							  PxSceneQueryFlags hintFlags)
{
	PX_CHECK_VALID(pose0.p);
	PX_CHECK_VALID(pose0.q);
	PX_CHECK_VALID(pose1.p);
	PX_CHECK_VALID(pose1.q);
	PX_CHECK_VALID(unitDir);
	PX_CHECK_VALID(distance);

	PX_CHECK_AND_RETURN_VAL(distance > 0, "PxGeometryQuery::sweep(): sweep motion length must be greater than 0.", false);

	switch (geom0.getType())
	{
		case PxGeometryType::eSPHERE :
		{
			const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom0);

			const Gu::Capsule worldCapsule(Gu::Segment(pose0.p, pose0.p), sphereGeom.radius);

			Gu::SweepCapsuleFunc func = Gu::gSweepCapsuleMap[geom1.getType()];
			return func(geom1, pose1, worldCapsule, unitDir, distance, sweepHit, hintFlags);
		}
		break;

		case PxGeometryType::eCAPSULE :
		{
			const PxCapsuleGeometry& capsGeom = static_cast<const PxCapsuleGeometry&>(geom0);

			Gu::Capsule worldCapsule;
			Gu::getWorldCapsule(worldCapsule, capsGeom, pose0);

			Gu::SweepCapsuleFunc func = Gu::gSweepCapsuleMap[geom1.getType()];
			return func(geom1, pose1, worldCapsule, unitDir, distance, sweepHit, hintFlags);
		}
		break;

		case PxGeometryType::eBOX :
		{
			const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom0);

			Gu::Box box;	buildFrom(box, pose0.p, boxGeom.halfExtents, pose0.q);

			Gu::SweepBoxFunc func = Gu::gSweepBoxMap[geom1.getType()];
			return func(geom1, pose1, box, unitDir, distance, sweepHit, hintFlags);
		}
		break;

		case PxGeometryType::eCONVEXMESH :
		{
			const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(geom0);

			Gu::SweepConvexFunc func = Gu::gSweepConvexMap[geom1.getType()];
			return func(geom1, pose1, convexGeom, pose0, unitDir, distance, sweepHit, hintFlags);
		}
		break;

		default :
			PX_CHECK_AND_RETURN_VAL(false, "PxGeometryQuery::sweep(): first geometry object parameter must be sphere, capsule, box or convex geometry.", false);
		break;
	}

	return false;
}


bool Gu::GeometryQuery::sweep(const PxVec3& unitDir,
						  const PxReal distance,
						  const PxGeometry& geom,
						  const PxTransform& pose,
						  PxU32 triangleCount,
						  const Gu::Triangle* triangles,
						  PxSweepHit& sweepHit,
						  PxSceneQueryFlags hintFlags,
						  const PxU32* triangleFlags,
						  const Gu::Triangle* triangleEdgeNormals,
						  const PxU32* cachedIndex)
{
	PX_CHECK_VALID(pose.p);
	PX_CHECK_VALID(pose.q);
	PX_CHECK_VALID(unitDir);
	PX_CHECK_VALID(distance);

	PX_CHECK_AND_RETURN_VAL(distance > 0, "PxGeometryQuery::sweep(): sweep distance must be greater than 0.", false);

	switch (geom.getType())
	{
		case PxGeometryType::eSPHERE :
		{
			const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom);

			PxCapsuleGeometry capsuleGeom;
			capsuleGeom.radius = sphereGeom.radius;
			capsuleGeom.halfHeight = 0.0f;

			return Gu::SweepCapsuleTriangles(	triangleCount, triangles, triangleFlags, capsuleGeom, pose, unitDir, distance,
												cachedIndex, sweepHit.impact, sweepHit.normal, sweepHit.distance, sweepHit.faceIndex);
		}
		break;

		case PxGeometryType::eCAPSULE :
		{
			const PxCapsuleGeometry& capsGeom = static_cast<const PxCapsuleGeometry&>(geom);

			return Gu::SweepCapsuleTriangles(	triangleCount, triangles, triangleFlags, capsGeom, pose, unitDir, distance,
												cachedIndex, sweepHit.impact, sweepHit.normal, sweepHit.distance, sweepHit.faceIndex);
		}
		break;

		case PxGeometryType::eBOX :
		{
			const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom);

			return Gu::SweepBoxTriangles(	triangleCount, triangles, triangleEdgeNormals, triangleFlags, boxGeom, pose, 
											unitDir, distance, sweepHit.impact, sweepHit.normal, sweepHit.distance, sweepHit.faceIndex, cachedIndex);
		}
		break;

		default :
			PX_CHECK_AND_RETURN_VAL(false, "PxGeometryQuery::sweep(): geometry object parameter must be sphere, capsule or box geometry.", false);
		break;
	}

	return false;
}


bool Gu::GeometryQuery::overlap(const PxGeometry& geom0, const PxTransform& pose0,
								const PxGeometry& geom1, const PxTransform& pose1)
{
	PX_CHECK_VALID(pose0.p);
	PX_CHECK_VALID(pose0.q);
	PX_CHECK_VALID(pose1.p);
	PX_CHECK_VALID(pose1.q);

	if(geom0.getType() > geom1.getType())
	{
		Gu::GeomOverlapFunc overlapFunc = Gu::gGeomOverlapMethodTable[geom1.getType()][geom0.getType()];
		PX_ASSERT(overlapFunc);
		return overlapFunc(geom1, pose1, geom0, pose0, NULL);
	}
	else
	{
		Gu::GeomOverlapFunc overlapFunc = Gu::gGeomOverlapMethodTable[geom0.getType()][geom1.getType()];
		PX_ASSERT(overlapFunc);
		return overlapFunc(geom0, pose0, geom1, pose1, NULL);
	}
}

PxU32 Gu::GeometryQuery::raycast(const PxVec3& rayOrigin,
							const PxVec3& rayDir,
							const PxGeometry& geom,
							const PxTransform& pose,
							PxReal maxDist,
							PxSceneQueryFlags hintFlags,
							PxU32 maxHits,
							PxRaycastHit* PX_RESTRICT rayHits,
							bool firstHit)
{
	PX_CHECK_VALID(rayDir);
	PX_CHECK_VALID(rayOrigin);
	PX_CHECK_VALID(pose.p);
	PX_CHECK_VALID(pose.q);
	PX_CHECK_VALID(maxDist);

	PX_CHECK_AND_RETURN_VAL(PxAbs(rayDir.magnitudeSquared()-1)<1e-4, "PxGeometryQuery::raycast(): ray direction must be unit vector.", false);

	Gu::RaycastFunc func = Gu::gRaycastMap[geom.getType()];
	return func(geom, pose, rayOrigin, rayDir, maxDist, hintFlags, maxHits, rayHits, firstHit);
}

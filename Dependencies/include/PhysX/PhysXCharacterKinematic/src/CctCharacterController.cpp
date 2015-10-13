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
  
#include <assert.h>
#include "CctCharacterController.h"
#include "CctSweptBox.h"
#include "CctSweptCapsule.h"
#include "CctUtils.h"
#include "PxController.h"
#include "PxRigidDynamic.h"
#include "PxShape.h"
#include "PxBoxGeometry.h"
#include "PxCapsuleGeometry.h"
#include "PxSphereGeometry.h"
#include "PxFiltering.h"
#include "GuGeometryQuery.h"
#include "CmRenderOutput.h"
#include "PsMathUtils.h"
#include "PxSceneQueryReport.h"

#define ASSERT		assert

#define	MAX_ITER	10

using namespace physx;
using namespace Cct;

static const PxSceneQueryFlags gSweepHintFlags = PxSceneQueryFlag::eDISTANCE | PxSceneQueryFlag::eIMPACT | PxSceneQueryFlag::eNORMAL;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static PX_INLINE void ComputeReflexionVector(PxVec3& reflected, const PxVec3& incoming_dir, const PxVec3& outward_normal)
{
	reflected = incoming_dir - outward_normal * 2.0f * (incoming_dir.dot(outward_normal));
}

static PX_INLINE void DecomposeVector(PxVec3& normal_compo, PxVec3& tangent_compo, const PxVec3& outward_dir, const PxVec3& outward_normal)
{
	normal_compo = outward_normal * (outward_dir.dot(outward_normal));
	tangent_compo = outward_dir - normal_compo;
}

static PX_INLINE void CollisionResponse(PxExtendedVec3& target_position, const PxExtendedVec3& current_position, const PxVec3& current_dir, const PxVec3& hit_normal, PxF32 bump, PxF32 friction, bool normalize=false)
{
	// Compute reflect direction
	PxVec3 ReflectDir;
	ComputeReflexionVector(ReflectDir, current_dir, hit_normal);
	ReflectDir.normalize();

	// Decompose it
	PxVec3 NormalCompo, TangentCompo;
	DecomposeVector(NormalCompo, TangentCompo, ReflectDir, hit_normal);

	// Compute new destination position
	const PxExtended Amplitude = distance(target_position, current_position);

	target_position = current_position;
	if(bump!=0.0f)
	{
		if(normalize)	NormalCompo.normalize();
		target_position += NormalCompo*float(bump*Amplitude);
	}
	if(friction!=0.0)
	{
		if(normalize)	TangentCompo.normalize();
		target_position += TangentCompo*float(friction*Amplitude);
	}
}

static PX_INLINE void RelocateBox(PxBoxGeometry& boxGeom, PxTransform& pose, const PxExtendedVec3& center, const PxVec3& extents, const PxExtendedVec3& origin)
{
	boxGeom.halfExtents = extents;

	pose.p.x = float(center.x - origin.x);
	pose.p.y = float(center.y - origin.y);
	pose.p.z = float(center.z - origin.z);

	pose.q = PxQuat::createIdentity();
}

static PX_INLINE void RelocateBox(PxBoxGeometry& boxGeom, PxTransform& pose, const TouchedUserBox& userBox)
{
	PxExtendedVec3 center;
	PxVec3 extents;
	getExtents(userBox.mBox, extents);
	getCenter(userBox.mBox, center);

	RelocateBox(boxGeom, pose, center, extents, userBox.mOffset);
}

static PX_INLINE void RelocateBox(PxBoxGeometry& boxGeom, PxTransform& pose, const TouchedBox& box)
{
	boxGeom.halfExtents = box.mExtents;

	pose.p = box.mCenter;
	pose.q = box.mRot.toQuat();
}

static PX_INLINE void RelocateCapsule(
	PxCapsuleGeometry& capsuleGeom, PxTransform& pose, const SweptCapsule* sc,
	PxU32 up_direction, const PxExtendedVec3& center, const PxExtendedVec3& origin)
{
	static PxQuat sUpDirectionQuat[] = {PxQuat::createIdentity(),
										PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f)),
										PxQuat(-PxHalfPi, PxVec3(0.0f, 1.0f, 0.0f)) };

	capsuleGeom.radius = sc->mRadius;
	capsuleGeom.halfHeight = 0.5f * sc->mHeight;

	pose.p.x = float(center.x - origin.x);
	pose.p.y = float(center.y - origin.y);
	pose.p.z = float(center.z - origin.z);

	pose.q = sUpDirectionQuat[up_direction];
}

static PX_INLINE void RelocateCapsule(PxCapsuleGeometry& capsuleGeom, PxTransform& pose, const PxVec3& p0, const PxVec3& p1, PxReal radius)
{
	capsuleGeom.radius = radius;

	PxVec3 dir = p1 - p0;
	pose.p = p0 + dir * 0.5f;
	capsuleGeom.halfHeight = 0.5f * dir.normalize();

	if (capsuleGeom.halfHeight > PX_EPS_F32)	//else it is just a sphere.
	{
	//angle axis representation is the rotation from the world x axis to dir
		PxVec3 t1, t2;
		Ps::normalToTangents(dir, t1, t2);
		
		PxMat33Legacy x;
		x.setColumn(0, dir);
		x.setColumn(1, t1);
		x.setColumn(2, t2);
		pose.q = x.toQuat();
	}
	//this won't ever be exactly the same thing as the original because we lost some info on a DOF
	else
		pose.q = PxQuat::createIdentity();
}

static PX_INLINE void RelocateCapsule(PxCapsuleGeometry& capsuleGeom, PxTransform& pose, const TouchedUserCapsule& userCapsule)
{
	PxVec3 p0, p1;
	p0.x = float(userCapsule.mCapsule.p0.x - userCapsule.mOffset.x);
	p0.y = float(userCapsule.mCapsule.p0.y - userCapsule.mOffset.y);
	p0.z = float(userCapsule.mCapsule.p0.z - userCapsule.mOffset.z);
	p1.x = float(userCapsule.mCapsule.p1.x - userCapsule.mOffset.x);
	p1.y = float(userCapsule.mCapsule.p1.y - userCapsule.mOffset.y);
	p1.z = float(userCapsule.mCapsule.p1.z - userCapsule.mOffset.z);

	RelocateCapsule(capsuleGeom, pose, p0, p1, userCapsule.mCapsule.radius);
}

static bool SweepBoxUserBox(const SweepTest*, const SweptVolume* volume, const TouchedGeom* geom, const PxExtendedVec3& center, const PxVec3& dir, SweptContact& impact)
{
	ASSERT(volume->getType()==SweptVolumeType::eBOX);
	ASSERT(geom->mType==TouchedGeomType::eUSER_BOX);
	const SweptBox* SB = static_cast<const SweptBox*>(volume);
	const TouchedUserBox* TC = static_cast<const TouchedUserBox*>(geom);

	PxBoxGeometry boxGeom0;
	PxTransform boxPose0;
	// To precompute
	RelocateBox(boxGeom0, boxPose0, center, SB->mExtents, TC->mOffset);

	PxBoxGeometry boxGeom1;
	PxTransform boxPose1;
	RelocateBox(boxGeom1, boxPose1, *TC);

	PxSweepHit sweepHit;
	if (!Gu::GeometryQuery::sweep(dir, impact.mDistance, boxGeom0, boxPose0, boxGeom1, boxPose1, sweepHit, gSweepHintFlags))
		return false;

	if(sweepHit.distance >= impact.mDistance) return false;

	impact.mWorldNormal		= sweepHit.normal;
	impact.mDistance		= sweepHit.distance;
	impact.mInternalIndex	= PX_INVALID_U32;
	impact.mTriangleIndex	= PX_INVALID_U32;
	impact.setWorldPos(sweepHit.impact, TC->mOffset);
	return true;
}

static bool SweepBoxUserCapsule(const SweepTest*, const SweptVolume* volume, const TouchedGeom* geom, const PxExtendedVec3& center, const PxVec3& dir, SweptContact& impact)
{
	ASSERT(volume->getType()==SweptVolumeType::eBOX);
	ASSERT(geom->mType==TouchedGeomType::eUSER_CAPSULE);
	const SweptBox* SB = static_cast<const SweptBox*>(volume);
	const TouchedUserCapsule* TC = static_cast<const TouchedUserCapsule*>(geom);

	PxBoxGeometry boxGeom;
	PxTransform boxPose;
	// To precompute
	RelocateBox(boxGeom, boxPose, center, SB->mExtents, TC->mOffset);

	PxCapsuleGeometry capsuleGeom;
	PxTransform capsulePose;
	RelocateCapsule(capsuleGeom, capsulePose, *TC);

	PxSweepHit sweepHit;
	if (!Gu::GeometryQuery::sweep(dir, impact.mDistance, boxGeom, boxPose, capsuleGeom, capsulePose, sweepHit, gSweepHintFlags))
		return false;

	if (sweepHit.distance >= impact.mDistance) return false;

	impact.mDistance		= sweepHit.distance;
	impact.mWorldNormal		= sweepHit.normal;
	impact.mInternalIndex	= PX_INVALID_U32;
	impact.mTriangleIndex	= PX_INVALID_U32;
	if(sweepHit.distance==0.0f)
	{
		setZero(impact.mWorldPos);
		impact.mWorldNormal = PxVec3(0);
// ### this fixes the bug on box-capsule but I'm not sure it's valid:
// - when the capsule is moving, it's ok to return false
// - when the box is moving, it's not! because it means the motion is completely free!!
		return false;
	}
	else
	//TO CHECK: Investigate whether any significant performance improvement can be achieved through
	//          making the impact point computation optional in the sweep calls and compute it later
	/*{
		// ### check this
		float t;
		PxVec3 p;
		float d = gUtilLib->PxSegmentOBBSqrDist(Capsule, Box0.center, Box0.extents, Box0.rot, &t, &p);
		Box0.rot.multiply(p,p);
		impact.mWorldPos.x = p.x + Box0.center.x + TC->mOffset.x;
		impact.mWorldPos.y = p.y + Box0.center.y + TC->mOffset.y;
		impact.mWorldPos.z = p.z + Box0.center.z + TC->mOffset.z;
	}*/
	{
		impact.setWorldPos(sweepHit.impact, TC->mOffset);
	}
	return true;
}

static bool sweepVolumeVsMesh(	const SweepTest* sweepTest, const TouchedMesh* touchedMesh, SweptContact& impact,
								const PxVec3& unitDir, const PxGeometry& geom, const PxTransform& pose,
								PxU32 nbTris, const Gu::Triangle* triangles,
								const PxU32* edgeFlags, const Gu::Triangle* triangleEdgeNormals, PxU32 cachedIndex)
{
	PxSweepHit sweepHit;
	if(Gu::GeometryQuery::sweep(unitDir, impact.mDistance, geom, pose, nbTris, triangles, sweepHit, gSweepHintFlags, edgeFlags, triangleEdgeNormals, &cachedIndex))
	{
		if(sweepHit.distance >= impact.mDistance)
			return false;

		impact.mDistance	= sweepHit.distance;
		impact.mWorldNormal	= sweepHit.normal;
		impact.setWorldPos(sweepHit.impact, touchedMesh->mOffset);

		// Returned index is only between 0 and nbTris, i.e. it indexes the array of cached triangles, not the original mesh.
		PX_ASSERT(sweepHit.faceIndex < nbTris);
		sweepTest->mCachedTriIndex[sweepTest->mCachedTriIndexIndex] = sweepHit.faceIndex;

		// The CCT loop will use the index from the start of the cache...
		impact.mInternalIndex = sweepHit.faceIndex + touchedMesh->mIndexWorldTriangles;
		const PxU32* triangleIndices = &sweepTest->mTriangleIndices[touchedMesh->mIndexEdgeFlags];
		impact.mTriangleIndex = triangleIndices[sweepHit.faceIndex];
		return true;
	}
	return false;
}

static bool SweepBoxMesh(const SweepTest* sweep_test, const SweptVolume* volume, const TouchedGeom* geom, const PxExtendedVec3& center, const PxVec3& dir, SweptContact& impact)
{
	ASSERT(volume->getType()==SweptVolumeType::eBOX);
	ASSERT(geom->mType==TouchedGeomType::eMESH);
	const SweptBox* SB = static_cast<const SweptBox*>(volume);
	const TouchedMesh* TM = static_cast<const TouchedMesh*>(geom);

	PxU32 nbTris = TM->mNbTris;
	if(!nbTris)
		return false;

	// Fetch triangle data for current mesh (the stream may contain triangles from multiple meshes)
	const Gu::Triangle* T	= &sweep_test->mWorldTriangles[TM->mIndexWorldTriangles];
	const Gu::Triangle* ET	= &sweep_test->mWorldEdgeNormals[TM->mIndexWorldEdgeNormals];
	const PxU32* EdgeFlags	= &sweep_test->mEdgeFlags[TM->mIndexEdgeFlags];

	// PT: this only really works when the CCT collides with a single mesh, but that's the most common case. When it doesn't, there's just no speedup but it still works.
	PxU32 CachedIndex = sweep_test->mCachedTriIndex[sweep_test->mCachedTriIndexIndex];
	if(CachedIndex>=nbTris)
		CachedIndex=0;

	PxBoxGeometry boxGeom;
	boxGeom.halfExtents = SB->mExtents;
	PxTransform boxPose(PxVec3(float(center.x - TM->mOffset.x), float(center.y - TM->mOffset.y), float(center.z - TM->mOffset.z)), PxQuat::createIdentity());  // Precompute

	return sweepVolumeVsMesh(sweep_test, TM, impact, dir, boxGeom, boxPose, nbTris, T, EdgeFlags, ET, CachedIndex);
}

static bool SweepCapsuleMesh(
	const SweepTest* sweep_test, const SweptVolume* volume, const TouchedGeom* geom,
	const PxExtendedVec3& center, const PxVec3& dir, SweptContact& impact)
{
	ASSERT(volume->getType()==SweptVolumeType::eCAPSULE);
	ASSERT(geom->mType==TouchedGeomType::eMESH);
	const SweptCapsule* SC = static_cast<const SweptCapsule*>(volume);
	const TouchedMesh* TM = static_cast<const TouchedMesh*>(geom);

	PxU32 nbTris = TM->mNbTris;
	if(!nbTris)
		return false;

	// Fetch triangle data for current mesh (the stream may contain triangles from multiple meshes)
	const Gu::Triangle* T	= &sweep_test->mWorldTriangles[TM->mIndexWorldTriangles];
//	const Gu::Triangle* ET	= &sweep_test->mWorldEdgeNormals[TM->mIndexWorldEdgeNormals];
	const PxU32* EdgeFlags	= &sweep_test->mEdgeFlags[TM->mIndexEdgeFlags];

	// PT: this only really works when the CCT collides with a single mesh, but that's the most common case.
	// When it doesn't, there's just no speedup but it still works.
	PxU32 CachedIndex = sweep_test->mCachedTriIndex[sweep_test->mCachedTriIndexIndex];
	if(CachedIndex>=nbTris)
		CachedIndex=0;

	PxCapsuleGeometry capsuleGeom;
	PxTransform capsulePose;
	RelocateCapsule(capsuleGeom, capsulePose, SC, sweep_test->mUserParams.mUpDirection, center, TM->mOffset);

	return sweepVolumeVsMesh(sweep_test, TM, impact, dir, capsuleGeom, capsulePose, nbTris, T, EdgeFlags, NULL, CachedIndex);
}

static bool SweepBoxBox(const SweepTest*, const SweptVolume* volume, const TouchedGeom* geom, const PxExtendedVec3& center, const PxVec3& dir, SweptContact& impact)
{
	ASSERT(volume->getType()==SweptVolumeType::eBOX);
	ASSERT(geom->mType==TouchedGeomType::eBOX);
	const SweptBox* SB = static_cast<const SweptBox*>(volume);
	const TouchedBox* TB = static_cast<const TouchedBox*>(geom);

	PxBoxGeometry boxGeom0;
	PxTransform boxPose0;
	// To precompute
	RelocateBox(boxGeom0, boxPose0, center, SB->mExtents, TB->mOffset);

	PxBoxGeometry boxGeom1;
	PxTransform boxPose1;
	RelocateBox(boxGeom1, boxPose1, *TB);

	PxSweepHit sweepHit;
	if (!Gu::GeometryQuery::sweep(dir, impact.mDistance, boxGeom0, boxPose0, boxGeom1, boxPose1, sweepHit, gSweepHintFlags))
		return false;

	if(sweepHit.distance >= impact.mDistance) return false;

	impact.mWorldNormal		= sweepHit.normal;
	impact.mDistance		= sweepHit.distance;
	impact.mInternalIndex	= PX_INVALID_U32;
	impact.mTriangleIndex	= PX_INVALID_U32;
	impact.setWorldPos(sweepHit.impact, TB->mOffset);
	return true;
}

static bool SweepBoxSphere(const SweepTest*, const SweptVolume* volume, const TouchedGeom* geom, const PxExtendedVec3& center, const PxVec3& dir, SweptContact& impact)
{
	ASSERT(volume->getType()==SweptVolumeType::eBOX);
	ASSERT(geom->mType==TouchedGeomType::eSPHERE);
	const SweptBox* SB = static_cast<const SweptBox*>(volume);
	const TouchedSphere* TS = static_cast<const TouchedSphere*>(geom);

	PxBoxGeometry boxGeom;
	PxTransform boxPose;
	// To precompute
	RelocateBox(boxGeom, boxPose, center, SB->mExtents, TS->mOffset);

	PxSphereGeometry sphereGeom;
	sphereGeom.radius = TS->mRadius;
	PxTransform spherePose;
	spherePose.p = TS->mCenter;
	spherePose.q = PxQuat::createIdentity();

	PxSweepHit sweepHit;
	if (!Gu::GeometryQuery::sweep(dir, impact.mDistance, boxGeom, boxPose, sphereGeom, spherePose, sweepHit, gSweepHintFlags))
		return false;

	impact.mDistance		= sweepHit.distance;
	impact.mWorldNormal		= sweepHit.normal;
	impact.mInternalIndex	= PX_INVALID_U32;
	impact.mTriangleIndex	= PX_INVALID_U32;
	if(sweepHit.distance==0.0f)
	{
		setZero(impact.mWorldPos);
		impact.mWorldNormal = PxVec3(0);
		return false;
	}
	else
	//TO CHECK: Investigate whether any significant performance improvement can be achieved through
	//          making the impact point computation optional in the sweep calls and compute it later
	/*
	{
		// The sweep test doesn't compute the impact point automatically, so we have to do it here.
		PxVec3 NewSphereCenter = TS->mSphere.center - d * dir;
		PxVec3 Closest;
		gUtilLib->PxPointOBBSqrDist(NewSphereCenter, Box0.center, Box0.extents, Box0.rot, &Closest);
		// Compute point on the box, after sweep
		Box0.rot.multiply(Closest, Closest);
		impact.mWorldPos.x = TS->mOffset.x + Closest.x + Box0.center.x + d * dir.x;
		impact.mWorldPos.y = TS->mOffset.y + Closest.y + Box0.center.y + d * dir.y;
		impact.mWorldPos.z = TS->mOffset.z + Closest.z + Box0.center.z + d * dir.z;

		impact.mWorldNormal = -impact.mWorldNormal;
	}*/
	{
		impact.setWorldPos(sweepHit.impact, TS->mOffset);
	}
	return true;
}

static bool SweepBoxCapsule(const SweepTest*, const SweptVolume* volume, const TouchedGeom* geom, const PxExtendedVec3& center, const PxVec3& dir, SweptContact& impact)
{
	ASSERT(volume->getType()==SweptVolumeType::eBOX);
	ASSERT(geom->mType==TouchedGeomType::eCAPSULE);
	const SweptBox* SB = static_cast<const SweptBox*>(volume);
	const TouchedCapsule* TC = static_cast<const TouchedCapsule*>(geom);

	PxBoxGeometry boxGeom;
	PxTransform boxPose;
	// To precompute
	RelocateBox(boxGeom, boxPose, center, SB->mExtents, TC->mOffset);

	PxCapsuleGeometry capsuleGeom;
	PxTransform capsulePose;
	RelocateCapsule(capsuleGeom, capsulePose, TC->mP0, TC->mP1, TC->mRadius);

	PxSweepHit sweepHit;
	if (!Gu::GeometryQuery::sweep(dir, impact.mDistance, boxGeom, boxPose, capsuleGeom, capsulePose, sweepHit, gSweepHintFlags))
		return false;

	if(sweepHit.distance >= impact.mDistance) return false;

	impact.mDistance		= sweepHit.distance;
	impact.mWorldNormal		= sweepHit.normal;
	impact.mInternalIndex	= PX_INVALID_U32;
	impact.mTriangleIndex	= PX_INVALID_U32;
	if(sweepHit.distance==0.0f)
	{
		setZero(impact.mWorldPos);
		impact.mWorldNormal = PxVec3(0);
// ### this fixes the bug on box-capsule but I'm not sure it's valid:
// - when the capsule is moving, it's ok to return false
// - when the box is moving, it's not! because it means the motion is completely free!!
		return false;
	}
	else
	//TO CHECK: Investigate whether any significant performance improvement can be achieved through
	//          making the impact point computation optional in the sweep calls and compute it later
	/*{
		float t;
		PxVec3 p;
		float d = gUtilLib->PxSegmentOBBSqrDist(TC->mCapsule, Box0.center, Box0.extents, Box0.rot, &t, &p);
		Box0.rot.multiply(p,p);
		impact.mWorldPos.x = p.x + Box0.center.x + TC->mOffset.x;
		impact.mWorldPos.y = p.y + Box0.center.y + TC->mOffset.y;
		impact.mWorldPos.z = p.z + Box0.center.z + TC->mOffset.z;
	}*/
	{
		impact.setWorldPos(sweepHit.impact, TC->mOffset);
	}
	return true;
}

static bool SweepCapsuleBox(const SweepTest* test, const SweptVolume* volume, const TouchedGeom* geom, const PxExtendedVec3& center, const PxVec3& dir, SweptContact& impact)
{
	ASSERT(volume->getType()==SweptVolumeType::eCAPSULE);
	ASSERT(geom->mType==TouchedGeomType::eBOX);
	const SweptCapsule* SC = static_cast<const SweptCapsule*>(volume);
	const TouchedBox* TB = static_cast<const TouchedBox*>(geom);

	PxCapsuleGeometry capsuleGeom;
	PxTransform capsulePose;
	RelocateCapsule(capsuleGeom, capsulePose, SC, test->mUserParams.mUpDirection, center, TB->mOffset);

	PxBoxGeometry boxGeom;
	PxTransform boxPose;
	// To precompute
	RelocateBox(boxGeom, boxPose, *TB);

	// The box and capsule coordinates are relative to the center of the cached bounding box
	PxSweepHit sweepHit;
	if (!Gu::GeometryQuery::sweep(dir, impact.mDistance, capsuleGeom, capsulePose, boxGeom, boxPose, sweepHit, gSweepHintFlags))
		return false;

	if(sweepHit.distance >= impact.mDistance) return false;

	impact.mDistance		= sweepHit.distance;
	impact.mWorldNormal		= sweepHit.normal;
	impact.mInternalIndex	= PX_INVALID_U32;
	impact.mTriangleIndex	= PX_INVALID_U32;

	if(sweepHit.distance==0.0f)
	{
	// ### this part makes the capsule goes through the box sometimes
		setZero(impact.mWorldPos);
		impact.mWorldNormal = PxVec3(0);
	// ### this fixes the bug on box-capsule but I'm not sure it's valid:
	// - when the capsule is moving, it's ok to return false
	// - when the box is moving, it's not! because it means the motion is completely free!!
		return false;
	}
	else
	//TO CHECK: Investigate whether any significant performance improvement can be achieved through
	//          making the impact point computation optional in the sweep calls and compute it later
	/*{
		float t;
		PxVec3 p;
		float d = gUtilLib->PxSegmentOBBSqrDist(Capsule, TB->mBox.center, TB->mBox.extents, TB->mBox.rot, &t, &p);
		TB->mBox.rot.multiply(p,p);
		p += TB->mBox.center;
		impact.mWorldPos.x = p.x + TB->mOffset.x;
		impact.mWorldPos.y = p.y + TB->mOffset.y;
		impact.mWorldPos.z = p.z + TB->mOffset.z;
	}*/
	{
		impact.setWorldPos(sweepHit.impact, TB->mOffset);
	}
	return true;
}

static bool SweepCapsuleSphere(const SweepTest* test, const SweptVolume* volume, const TouchedGeom* geom, const PxExtendedVec3& center, const PxVec3& dir, SweptContact& impact)
{
	ASSERT(volume->getType()==SweptVolumeType::eCAPSULE);
	ASSERT(geom->mType==TouchedGeomType::eSPHERE);
	const SweptCapsule* SC = static_cast<const SweptCapsule*>(volume);
	const TouchedSphere* TS = static_cast<const TouchedSphere*>(geom);

	PxCapsuleGeometry capsuleGeom;
	PxTransform capsulePose;
	RelocateCapsule(capsuleGeom, capsulePose, SC, test->mUserParams.mUpDirection, center, TS->mOffset);

	PxSphereGeometry sphereGeom;
	sphereGeom.radius = TS->mRadius;
	PxTransform spherePose;
	spherePose.p = TS->mCenter;
	spherePose.q = PxQuat::createIdentity();

	PxSweepHit sweepHit;
	if (!Gu::GeometryQuery::sweep(-dir, impact.mDistance, sphereGeom, spherePose, capsuleGeom, capsulePose, sweepHit, gSweepHintFlags))
		return false;

	if(sweepHit.distance >= impact.mDistance) return false;

	impact.mDistance		= sweepHit.distance;
	impact.mWorldNormal		= sweepHit.normal;
	impact.mInternalIndex	= PX_INVALID_U32;
	impact.mTriangleIndex	= PX_INVALID_U32;
	if(sweepHit.distance==0.0f)
	{
		setZero(impact.mWorldPos);
		impact.mWorldNormal = PxVec3(0);
		return false;
	}
	else
	{
		impact.setWorldPos(sweepHit.impact, TS->mOffset);
	}
	return true;
}

static bool SweepCapsuleCapsule(const SweepTest* test, const SweptVolume* volume, const TouchedGeom* geom, const PxExtendedVec3& center, const PxVec3& dir, SweptContact& impact)
{
	ASSERT(volume->getType()==SweptVolumeType::eCAPSULE);
	ASSERT(geom->mType==TouchedGeomType::eCAPSULE);
	const SweptCapsule* SC = static_cast<const SweptCapsule*>(volume);
	const TouchedCapsule* TC = static_cast<const TouchedCapsule*>(geom);

	PxCapsuleGeometry capsuleGeom0;
	PxTransform capsulePose0;
	RelocateCapsule(capsuleGeom0, capsulePose0, SC, test->mUserParams.mUpDirection, center, TC->mOffset);

	PxCapsuleGeometry capsuleGeom1;
	PxTransform capsulePose1;
	RelocateCapsule(capsuleGeom1, capsulePose1, TC->mP0, TC->mP1, TC->mRadius);

	PxSweepHit sweepHit;
	if (!Gu::GeometryQuery::sweep(dir, impact.mDistance, capsuleGeom0, capsulePose0, capsuleGeom1, capsulePose1, sweepHit, gSweepHintFlags))
		return false;

	if(sweepHit.distance >= impact.mDistance) return false;

	impact.mDistance		= sweepHit.distance;
	impact.mWorldNormal		= sweepHit.normal;
	impact.mInternalIndex	= PX_INVALID_U32;
	impact.mTriangleIndex	= PX_INVALID_U32;
	if(sweepHit.distance==0.0f)
	{
		setZero(impact.mWorldPos);
		impact.mWorldNormal = PxVec3(0);
		return false;
	}
	else
	{
		impact.setWorldPos(sweepHit.impact, TC->mOffset);
	}
	return true;
}

static bool SweepCapsuleUserCapsule(const SweepTest* test, const SweptVolume* volume, const TouchedGeom* geom, const PxExtendedVec3& center, const PxVec3& dir, SweptContact& impact)
{
	ASSERT(volume->getType()==SweptVolumeType::eCAPSULE);
	ASSERT(geom->mType==TouchedGeomType::eUSER_CAPSULE);
	const SweptCapsule* SC = static_cast<const SweptCapsule*>(volume);
	const TouchedUserCapsule* TC = static_cast<const TouchedUserCapsule*>(geom);

	PxCapsuleGeometry capsuleGeom0;
	PxTransform capsulePose0;
	RelocateCapsule(capsuleGeom0, capsulePose0, SC, test->mUserParams.mUpDirection, center, TC->mOffset);

	PxCapsuleGeometry capsuleGeom1;
	PxTransform capsulePose1;
	RelocateCapsule(capsuleGeom1, capsulePose1, *TC);

	PxSweepHit sweepHit;
	if (!Gu::GeometryQuery::sweep(dir, impact.mDistance, capsuleGeom0, capsulePose0, capsuleGeom1, capsulePose1, sweepHit, gSweepHintFlags))
		return false;

	if(sweepHit.distance >= impact.mDistance) return false;

	impact.mDistance		= sweepHit.distance;
	impact.mWorldNormal		= sweepHit.normal;
	impact.mInternalIndex	= PX_INVALID_U32;
	impact.mTriangleIndex	= PX_INVALID_U32;
	if(sweepHit.distance==0.0f)
	{
		setZero(impact.mWorldPos);
		impact.mWorldNormal = PxVec3(0);
		return false;
	}
	else
	{
		impact.setWorldPos(sweepHit.impact, TC->mOffset);
	}
	return true;
}

static bool SweepCapsuleUserBox(const SweepTest* test, const SweptVolume* volume, const TouchedGeom* geom, const PxExtendedVec3& center, const PxVec3& dir, SweptContact& impact)
{
	ASSERT(volume->getType()==SweptVolumeType::eCAPSULE);
	ASSERT(geom->mType==TouchedGeomType::eUSER_BOX);
	const SweptCapsule* SC = static_cast<const SweptCapsule*>(volume);
	const TouchedUserBox* TB = static_cast<const TouchedUserBox*>(geom);

	PxCapsuleGeometry capsuleGeom;
	PxTransform capsulePose;
	RelocateCapsule(capsuleGeom, capsulePose, SC, test->mUserParams.mUpDirection, center, TB->mOffset);

	PxBoxGeometry boxGeom;
	PxTransform boxPose;
	RelocateBox(boxGeom, boxPose, *TB);

	PxSweepHit sweepHit;
	if (!Gu::GeometryQuery::sweep(dir, impact.mDistance, capsuleGeom, capsulePose, boxGeom, boxPose, sweepHit, gSweepHintFlags))
		return false;

	if(sweepHit.distance >= impact.mDistance) return false;

	impact.mDistance		= sweepHit.distance;
	impact.mWorldNormal		= sweepHit.normal;
	impact.mInternalIndex	= PX_INVALID_U32;
	impact.mTriangleIndex	= PX_INVALID_U32;

	if(sweepHit.distance==0.0f)
	{
	// ### this part makes the capsule goes through the box sometimes
		setZero(impact.mWorldPos);
		impact.mWorldNormal = PxVec3(0);
	// ### this fixes the bug on box-capsule but I'm not sure it's valid:
	// - when the capsule is moving, it's ok to return false
	// - when the box is moving, it's not! because it means the motion is completely free!!
		return false;
	}
	else
	//TO CHECK: Investigate whether any significant performance improvement can be achieved through
	//          making the impact point computation optional in the sweep calls and compute it later
	/*{
		// ### check this
		float t;
		PxVec3 p;
		float d = gUtilLib->PxSegmentOBBSqrDist(Capsule, Box.center, Box.extents, Box.rot, &t, &p);
		p += Box.center;
		impact.mWorldPos.x = p.x + TB->mOffset.x;
		impact.mWorldPos.y = p.y + TB->mOffset.y;
		impact.mWorldPos.z = p.z + TB->mOffset.z;
	}*/
	{
		impact.setWorldPos(sweepHit.impact, TB->mOffset);
	}
	return true;
}

typedef bool (*SweepFunc) (const SweepTest*, const SweptVolume*, const TouchedGeom*, const PxExtendedVec3&, const PxVec3&, SweptContact&);

static SweepFunc gSweepMap[SweptVolumeType::eLAST][TouchedGeomType::eLAST] = {
	// Box funcs
	{
	SweepBoxUserBox,
	SweepBoxUserCapsule,
	SweepBoxMesh,
	SweepBoxBox,
	SweepBoxSphere,
	SweepBoxCapsule
	},

	// Capsule funcs
	{
	SweepCapsuleUserBox,
	SweepCapsuleUserCapsule,
	SweepCapsuleMesh,
	SweepCapsuleBox,
	SweepCapsuleSphere,
	SweepCapsuleCapsule
	},
};

PX_COMPILE_TIME_ASSERT(sizeof(gSweepMap)==SweptVolumeType::eLAST*TouchedGeomType::eLAST*sizeof(SweepFunc));

static bool CollideGeoms(
	const SweepTest* sweep_test, const SweptVolume& volume, const IntArray& geom_stream,
	const PxExtendedVec3& center, const PxVec3& dir, SweptContact& impact)
{
	impact.mInternalIndex	= PX_INVALID_U32;
	impact.mTriangleIndex	= PX_INVALID_U32;
	impact.mGeom			= NULL;

	static const PxU32 GeomSizes[] = 
	{
		sizeof(TouchedUserBox),
		sizeof(TouchedUserCapsule),
		sizeof(TouchedMesh),
		sizeof(TouchedBox),
		sizeof(TouchedSphere),
		sizeof(TouchedCapsule),
	};

	bool Status = false;
	const PxU32* Data = geom_stream.begin();
	const PxU32* Last = geom_stream.end();
	while(Data!=Last)
	{
		TouchedGeom* CurrentGeom = (TouchedGeom*)Data;

		SweepFunc ST = gSweepMap[volume.getType()][CurrentGeom->mType];
		if(ST)
		{
			SweptContact C;
			C.mDistance			= impact.mDistance;	// Initialize with current best distance
			C.mInternalIndex	= PX_INVALID_U32;
			C.mTriangleIndex	= PX_INVALID_U32;
			if((ST)(sweep_test, &volume, CurrentGeom, center, dir, C))
			{
				if(C.mDistance<impact.mDistance)
				{
					impact = C;
					impact.mGeom = CurrentGeom;
					Status = true;
					if(impact.mDistance <= 0)	// there is no point testing for closer hits
						return Status;			// since we are touching a shape already
				}
			}
		}

		PxU8* ptr = (PxU8*)Data;
		ptr += GeomSizes[CurrentGeom->mType];
		Data = (const PxU32*)ptr;
	}
	return Status;
}

SweepTest::SweepTest() :
	mRenderBuffer		(NULL),
	mValidTri			(false),
	mValidateCallback	(false),
	mNormalizeResponse	(false),
	mWorldTriangles		(PX_DEBUG_EXP("sweepTestTrigs")),
	mWorldEdgeNormals	(PX_DEBUG_EXP("sweepTestNormals")),
	mEdgeFlags			(PX_DEBUG_EXP("sweepTestFlags")),
	mTriangleIndices	(PX_DEBUG_EXP("sweepTestTriangleIndices")),
	mGeomStream			(PX_DEBUG_EXP("sweepTestStream"))
{
	mCachedTBV.setEmpty();
	mCachedTriIndexIndex	= 0;
	mCachedTriIndex[0] = mCachedTriIndex[1] = mCachedTriIndex[2] = 0;
	mNbCachedStatic = 0;
	mNbCachedT		= 0;
	mNbCachedEN		= 0;
	mNbCachedF		= 0;
	mUserParams.mHandleSlope			= false;
	mUserParams.mSlopeLimit				= 0.0f;
	mUserParams.mContactOffset			= 0.0f;
	mUserParams.mStepOffset				= 0.0f;
	mUserParams.mUpDirection			= PxCCTUpAxis::eX;
	mUserParams.mInvisibleWallHeight	= 0.0f;
	mUserParams.mMaxJumpHeight			= 0.0f;
//	mVolumeGrowth	= 1.2f;	// Must be >1.0f and not too big
	mVolumeGrowth	= 1.5f;	// Must be >1.0f and not too big
//	mVolumeGrowth	= 2.0f;	// Must be >1.0f and not too big
	mHitNonWalkable	= false;
	mWalkExperiment	= false;
	mMaxIter		= MAX_ITER;
	mFirstUpdate	= false;
}


SweepTest::~SweepTest()
{
}


void SweepTest::findTouchedCCTs(	PxU32 nb_boxes, const PxExtendedBounds3* boxes, const void** box_user_data,
									PxU32 nb_capsules, const PxExtendedCapsule* capsules, const void** capsule_user_data,
									const PxExtendedBounds3& world_box)
{
	PxExtendedVec3 Origin;	// Will be TouchedGeom::mOffset
	getCenter(world_box, Origin);

	// Find touched boxes, i.e. other box controllers
	for(PxU32 i=0;i<nb_boxes;i++)
	{
		if(!intersect(world_box, boxes[i]))
			continue;

		TouchedUserBox* UserBox = (TouchedUserBox*)reserve(mGeomStream, sizeof(TouchedUserBox)/sizeof(PxU32));
		UserBox->mType		= TouchedGeomType::eUSER_BOX;
		UserBox->mUserData	= box_user_data[i];
		UserBox->mOffset	= Origin;
		UserBox->mBox		= boxes[i];
	}

	// Find touched capsules, i.e. other capsule controllers
	PxExtendedVec3 Center;
	PxVec3 Extents;
	getCenter(world_box, Center);
	getExtents(world_box, Extents);
	PxMat33Legacy Idt;
	Idt.setIdentity();
	for(PxU32 i=0;i<nb_capsules;i++)
	{
		// Do a quick AABB check first, to avoid calling the SDK too much
		const PxF32 r = capsules[i].radius;
		if((capsules[i].p0.x - r > world_box.maximum.x) || (world_box.minimum.x > capsules[i].p1.x + r)) continue;
		if((capsules[i].p0.y - r > world_box.maximum.y) || (world_box.minimum.y > capsules[i].p1.y + r)) continue;
		if((capsules[i].p0.z - r > world_box.maximum.z) || (world_box.minimum.z > capsules[i].p1.z + r)) continue;

		// Do a box-capsule intersect, or skip it? => better to skip it, not really useful now
/*		Gu::Capsule tmp;
		tmp.radius = capsules[i].radius;
		tmp.p0.x = float(capsules[i].p0.x);
		tmp.p0.y = float(capsules[i].p0.y);
		tmp.p0.z = float(capsules[i].p0.z);
		tmp.p1.x = float(capsules[i].p1.x);
		tmp.p1.y = float(capsules[i].p1.y);
		tmp.p1.z = float(capsules[i].p1.z);
		float d2 = gUtilLib->PxSegmentOBBSqrDist(tmp, PxVec3(float(Center.x), float(Center.y), float(Center.z)), 
											Extents, 
											Idt, NULL, NULL);
		if(d2<capsules[i].radius*capsules[i].radius)*/
		{
			TouchedUserCapsule* UserCapsule = (TouchedUserCapsule*)reserve(mGeomStream, sizeof(TouchedUserCapsule)/sizeof(PxU32));
			UserCapsule->mType		= TouchedGeomType::eUSER_CAPSULE;
			UserCapsule->mUserData	= capsule_user_data[i];
			UserCapsule->mOffset	= Origin;
			UserCapsule->mCapsule	= capsules[i];
		}
	}
}

static PxU32 gNbIters = 0;
static PxU32 gNbFullUpdates = 0;
static PxU32 gNbPartialUpdates = 0;

static PxBounds3 getBounds3(const PxExtendedBounds3& extended)
{
	return PxBounds3(
		PxVec3(PxReal(extended.minimum.x), PxReal(extended.minimum.y), PxReal(extended.minimum.z)),
		PxVec3(PxReal(extended.maximum.x), PxReal(extended.maximum.y), PxReal(extended.maximum.z)));
}

void SweepTest::updateTouchedGeoms(	void* user_data, const SweptVolume& swept_volume,
									PxU32 nb_boxes, const PxExtendedBounds3* boxes, const void** box_user_data,
									PxU32 nb_capsules, const PxExtendedCapsule* capsules, const void** capsule_user_data,
									const PxExtendedBounds3& world_box, const PxFilterData* filterData, PxSceneQueryFilterCallback* filterCallback)
{
	/*
	- if this is the first iteration (new frame) we have to redo the dynamic objects & the CCTs. The static objects can
	be cached.
	- if this is not, we can cache everything
	*/

	// PT: using "world_box" instead of "mCachedTBV" seems to produce TTP 6207
//#define DYNAMIC_BOX	world_box
#define DYNAMIC_BOX	mCachedTBV

	bool NewCachedBox = false;

	// If the input box is inside the cached box, nothing to do
	if(1 && world_box.isInside(mCachedTBV))
	{
		//printf("CACHEIN%d\n", mFirstUpdate);
		if(mFirstUpdate)
		{
			mFirstUpdate = false;

			// Only redo the dynamic
			mGeomStream.resize(mNbCachedStatic);
			mWorldTriangles.resize(mNbCachedT);
			mWorldEdgeNormals.resize(mNbCachedEN);
			mEdgeFlags.resize(mNbCachedF);
			mTriangleIndices.resize(mNbCachedF);

			findTouchedGeometry(user_data, DYNAMIC_BOX, mWorldTriangles,
				swept_volume.getType()==SweptVolumeType::eBOX ? &mWorldEdgeNormals : NULL,
				mEdgeFlags, mTriangleIndices, mGeomStream, false, true, filterData, filterCallback, mUserParams);

			findTouchedCCTs(
				nb_boxes, boxes, box_user_data,
				nb_capsules, capsules, capsule_user_data,
				DYNAMIC_BOX);

			gNbPartialUpdates++;
		}
	}
	else
	{
		//printf("CACHEOUTNS=%d\n", mNbCachedStatic);
		NewCachedBox = true;

		// Cache BV used for the query
		mCachedTBV = world_box;

		// Grow the volume a bit. The temporal box here doesn't take sliding & collision response into account.
		// In bad cases it is possible to eventually touch a portion of space not covered by this volume. Just
		// in case, we grow the initial volume slightly. Then, additional tests are performed within the loop
		// to make sure the TBV is always correct. There's a tradeoff between the original (artificial) growth
		// of the volume, and the number of TBV recomputations performed at runtime...
		if(1)
		{
			scale(mCachedTBV, mVolumeGrowth);
		}
		else
		{
			PxExtendedVec3 center;	getCenter(mCachedTBV, center);
			PxVec3 extents;	getExtents(mCachedTBV, extents);
/*			PxVec3 scale(mVolumeGrowth, mVolumeGrowth, mVolumeGrowth);
			scale[mUpDirection] = 1.0f;*/
/*			PxVec3 scale(1.0f, 1.0f, 1.0f);
			scale[mUpDirection] = mVolumeGrowth;
			extents.x *= scale.x;
			extents.y *= scale.y;
			extents.z *= scale.z;*/
			extents.x *= 1.8f;
			extents.y += 1.0f;
			extents.z *= 1.8f;
			setCenterExtents(mCachedTBV, center, extents);
		}

		// Gather triangles touched by this box. This covers multiple meshes.
		mWorldTriangles.clear();
		mWorldEdgeNormals.clear();
		mEdgeFlags.clear();
		mTriangleIndices.clear();
		mGeomStream.clear();
		mCachedTriIndexIndex	= 0;
		mCachedTriIndex[0] = mCachedTriIndex[1] = mCachedTriIndex[2] = 0;

		gNbFullUpdates++;
		findTouchedGeometry(
			user_data, mCachedTBV, mWorldTriangles,
			swept_volume.getType()==SweptVolumeType::eBOX ? &mWorldEdgeNormals : NULL,
			mEdgeFlags, mTriangleIndices, mGeomStream, true, false, filterData, filterCallback, mUserParams);

		mNbCachedStatic = mGeomStream.size();
		mNbCachedT = mWorldTriangles.size();
		mNbCachedEN = mWorldEdgeNormals.size();
		mNbCachedF = mEdgeFlags.size();
		PX_ASSERT(mTriangleIndices.size()==mNbCachedF);

		findTouchedGeometry(
			user_data, DYNAMIC_BOX, mWorldTriangles,
			swept_volume.getType()==SweptVolumeType::eBOX ? &mWorldEdgeNormals : NULL,
			mEdgeFlags, mTriangleIndices, mGeomStream, false, true, filterData, filterCallback, mUserParams);
		// We can't early exit when no tris are touched since we also have to handle the boxes

		findTouchedCCTs(
			nb_boxes, boxes, box_user_data,
			nb_capsules, capsules, capsule_user_data,
			DYNAMIC_BOX);

		mFirstUpdate = false;
		//printf("CACHEOUTNSDONE=%d\n", mNbCachedStatic);
	}

	if(mRenderBuffer)
		Cm::RenderOutput(*mRenderBuffer) << PxDebugColor::eARGB_YELLOW << Cm::DebugBox(getBounds3(world_box))
			<< (NewCachedBox ? PxDebugColor::eARGB_RED : PxDebugColor::eARGB_GREEN) << Cm::DebugBox(getBounds3(mCachedTBV));
}


// This is the generic sweep test for all swept volumes, but not character-controller specific
bool SweepTest::doSweepTest(void* user_data,
							void* user_data2,
							PxU32 nb_boxes, const PxExtendedBounds3* boxes, const void** box_user_data,
							PxU32 nb_capsules, const PxExtendedCapsule* capsules, const void** capsule_user_data,
							SweptVolume& swept_volume,
							const PxVec3& direction, PxU32 max_iter, PxU32* nb_collisions,
							float min_dist, const PxFilterData* filterData, PxSceneQueryFilterCallback* filterCallback, bool down_pass)
{
	// Early exit when motion is zero. Since the motion is decomposed into several vectors
	// and this function is called for each of them, it actually happens quite often.
	if(direction.isZero())
		return false;

	bool HasMoved = false;
	mValidTri = false;

	PxExtendedVec3 CurrentPosition = swept_volume.mCenter;
	PxExtendedVec3 TargetOrientation = swept_volume.mCenter;
	TargetOrientation += direction;

/*	if(direction.y==0.0f)
	{
		printf("New pass\n");
	}*/

	PxU32 NbCollisions = 0;
	while(max_iter--)
	{
		gNbIters++;
		// Compute current direction
		PxVec3 CurrentDirection = TargetOrientation - CurrentPosition;

/*		if(direction.y==0.0f)
		{
			printf("CurrentDirection: %f | %f | %f\n", CurrentDirection.x, CurrentDirection.y, CurrentDirection.z);
		}*/

		// Make sure the new TBV is still valid
		{
			// Compute temporal bounding box. We could use a capsule or an OBB instead:
			// - the volume would be smaller
			// - but the query would be slower
			// Overall it's unclear whether it's worth it or not.
			// TODO: optimize this part ?
			PxExtendedBounds3 TemporalBox;
			swept_volume.computeTemporalBox(*this, TemporalBox, CurrentPosition, CurrentDirection);

			// Gather touched geoms
			updateTouchedGeoms(user_data, swept_volume,
								nb_boxes, boxes, box_user_data,
								nb_capsules, capsules, capsule_user_data,
								TemporalBox,filterData,filterCallback);
		}

		const float Length = CurrentDirection.magnitude();
		if(Length<min_dist)
			break;

		CurrentDirection /= Length;

		// From Quake2: "if velocity is against the original velocity, stop dead to avoid tiny occilations in sloping corners"
		if((CurrentDirection.dot(direction)) <= 0.0f)
			break;

		// From this point, we're going to update the position at least once
		HasMoved = true;

		// Find closest collision
		SweptContact C;
		C.mDistance = Length + mUserParams.mContactOffset;

		if(!CollideGeoms(this, swept_volume, mGeomStream, CurrentPosition, CurrentDirection, C))
		{
			// no collision found => move to desired position
			CurrentPosition = TargetOrientation;
			break;
		}

		ASSERT(C.mGeom);	// If we reach this point, we must have touched a geom

		if(C.mGeom->mType==TouchedGeomType::eUSER_BOX || C.mGeom->mType==TouchedGeomType::eUSER_CAPSULE)
		{
			// We touched a user object, typically another CCT
			if(mValidateCallback)
				userHitCallback(user_data2, C, CurrentDirection, Length);

			// Trying to solve the following problem:
			// - by default, the CCT "friction" is infinite, i.e. a CCT will not slide on a slope (this is by design)
			// - this produces bad results when a capsule CCT stands on top of another capsule CCT, without sliding. Visually it looks
			//   like the character is standing on the other character's head, it looks bad. So, here, we would like to let the CCT
			//   slide away, i.e. we don't want friction.
			// So here we simply increase the number of iterations (== let the CCT slide) when the first down collision is with another CCT.
			if(down_pass && !NbCollisions)
				max_iter += 9;
		}
		else
		{
			// We touched a normal object
#ifdef USE_CONTACT_NORMAL_FOR_SLOPE_TEST
			mValidTri = true;
			mCN = C.mWorldNormal;
#else
			if(C.mInternalIndex!=PX_INVALID_U32)
			{
				mValidTri = true;
				mTouched = mWorldTriangles[C.mInternalIndex];
			}
#endif
			{
				if(mValidateCallback)
					shapeHitCallback(user_data2, C, CurrentDirection, Length);
			}
		}

		NbCollisions++;
		mContactPointHeight = (float)C.mWorldPos[mUserParams.mUpDirection];	// UBI

		const float DynSkin = mUserParams.mContactOffset;

		if(C.mDistance>DynSkin/*+0.01f*/)
			CurrentPosition += CurrentDirection*(C.mDistance-DynSkin);

		PxVec3 WorldNormal = C.mWorldNormal;
		if(mWalkExperiment)
		{
			// Make sure the auto-step doesn't bypass this !
			WorldNormal[mUserParams.mUpDirection]=0.0f;
			WorldNormal.normalize();
		}

		const float Bump = 0.0f;	// ### doesn't work when !=0 because of Quake2 hack!
		const float Friction = 1.0f;
		CollisionResponse(TargetOrientation, CurrentPosition, CurrentDirection, WorldNormal, Bump, Friction, mNormalizeResponse);
	}

	if(nb_collisions)
		*nb_collisions = NbCollisions;

	// Final box position that should be reflected in the graphics engine
	swept_volume.mCenter = CurrentPosition;

	// If we didn't move, don't update the box position at all (keeping possible lazy-evaluated structures valid)
	return HasMoved;
}

// ### have a return code to tell if we really moved or not

// Using swept code & direct position update (no physics engine)
// This function is the generic character controller logic, valid for all swept volumes
void SweepTest::moveCharacter(
					void* user_data,
					void* user_data2,
					SweptVolume& volume,
					const PxVec3& direction,
					PxU32 nb_boxes, const PxExtendedBounds3* boxes, const void** box_user_data,
					PxU32 nb_capsules, const PxExtendedCapsule* capsules, const void** capsule_user_data,
					float min_dist,
					PxU32& collision_flags,
					const PxFilterData* filterData,
					PxSceneQueryFilterCallback* filterCallback,
					bool constrainedClimbingMode
					 )
{
	mHitNonWalkable = false;
	PxU32 CollisionFlags = 0;
	const PxU32 MaxIter = mMaxIter;	// 1 for "collide and stop"
	const PxU32 MaxIterUp = MaxIter;
	const PxU32 MaxIterSides = MaxIter;
//	const PxU32 MaxIterDown = gWalkExperiment ? MaxIter : 1;
	const PxU32 MaxIterDown = 1;

	// ### this causes the artificial gap on top of chars
	float StepOffset = mUserParams.mStepOffset;	// Default step offset can be cancelled in some cases.

	// Save initial height
	const PxCCTUpAxis::Enum upDirection = mUserParams.mUpDirection;
	PxExtended OriginalHeight = volume.mCenter[upDirection];
	PxExtended OriginalBottomPoint = OriginalHeight - volume.mHalfHeight;	// UBI

	// TEST! Disable auto-step when flying. Not sure this is really useful.
	if(direction[upDirection]>0.0f)
		StepOffset = 0.0f;

	// Decompose motion into 3 independent motions: up, side, down
	// - if the motion is purely down (gravity only), the up part is needed to fight accuracy issues. For example if the
	// character is already touching the geometry a bit, the down sweep test might have troubles. If we first move it above
	// the geometry, the problems disappear.
	// - if the motion is lateral (character moving forward under normal gravity) the decomposition provides the autostep feature
	// - if the motion is purely up, the down part can be skipped

	PxVec3 UpVector(0.0f, 0.0f, 0.0f);
	PxVec3 DownVector(0.0f, 0.0f, 0.0f);

	if(direction[upDirection]<0.0f)
		DownVector[upDirection] = direction[upDirection];
	else
		UpVector[upDirection] = direction[upDirection];

	PxVec3 SideVector = direction;
	SideVector[upDirection] = 0.0f;

	// If the side motion is zero, i.e. if the character is not really moving, disable auto-step.
	if(!SideVector.isZero())
		UpVector[upDirection] += StepOffset;

	// ==========[ Initial volume query ]===========================
	if(1)
	{
		PxVec3 MotionExtents = UpVector.maximum(SideVector).maximum(DownVector);

		PxExtendedBounds3 TemporalBox;
		volume.computeTemporalBox(*this, TemporalBox, volume.mCenter, MotionExtents);

		// Gather touched geoms
		updateTouchedGeoms(user_data, volume,
							nb_boxes, boxes, box_user_data,
							nb_capsules, capsules, capsule_user_data,
							TemporalBox,filterData,filterCallback);
	}

	// ==========[ UP PASS ]===========================

	mCachedTriIndexIndex = 0;
	const bool PerformUpPass = true;
	PxU32 NbCollisions=0;

	if(PerformUpPass)
	{
		// Prevent user callback for up motion. This up displacement is artificial, and only needed for auto-stepping.
		// If we call the user for this, we might eventually apply upward forces to objects resting on top of us, even
		// if we visually don't move. This produces weird-looking motions.
		mValidateCallback = false;

		// In the walk-experiment we explicitly want to ban any up motions, to avoid characters climbing slopes they shouldn't climb.
		// So let's bypass the whole up pass.
		if(!mWalkExperiment)
		{
			// ### MaxIter here seems to "solve" the V bug
			if(doSweepTest(user_data,
				user_data2,
				nb_boxes, boxes, box_user_data,
				nb_capsules, capsules, capsule_user_data,
				volume, UpVector, MaxIterUp, &NbCollisions, min_dist, filterData, filterCallback))
			{
				if(NbCollisions)
				{
					CollisionFlags |= PxControllerFlag::eCOLLISION_UP;

					// Clamp step offset to make sure we don't undo more than what we did
					PxExtended Delta = volume.mCenter[upDirection] - OriginalHeight;
					if(Delta<StepOffset)
					{
						StepOffset=float(Delta);
					}
				}
			}
		}
	}

	// ==========[ SIDE PASS ]===========================

	mCachedTriIndexIndex = 1;
	mValidateCallback = true;
	const bool PerformSidePass = true;

	if(PerformSidePass)
	{
		NbCollisions=0;
		//printf("BS:%.2f %.2f %.2f NS=%d\n", volume.mCenter.x, volume.mCenter.y, volume.mCenter.z, mNbCachedStatic);
		if(doSweepTest(user_data,
			user_data2,
			nb_boxes, boxes, box_user_data,
			nb_capsules, capsules, capsule_user_data,
			volume, SideVector, MaxIterSides, &NbCollisions, min_dist, filterData, filterCallback))
		{
			if(NbCollisions)
				CollisionFlags |= PxControllerFlag::eCOLLISION_SIDES;
		}
		//printf("AS:%.2f %.2f %.2f NS=%d\n", volume.mCenter.x, volume.mCenter.y, volume.mCenter.z, mNbCachedStatic);
	}

	// ==========[ DOWN PASS ]===========================

	mCachedTriIndexIndex = 2;
	const bool PerformDownPass = true;

	if(PerformDownPass)
	{
		NbCollisions=0;

		if(!SideVector.isZero())	// We disabled that before so we don't have to undo it in that case
			DownVector[upDirection] -= StepOffset;	// Undo our artificial up motion

		mValidTri = false;

		// min_dist actually makes a big difference :(
		// AAARRRGGH: if we get culled because of min_dist here, mValidTri never becomes valid!
		if(doSweepTest(user_data,
			user_data2,
			nb_boxes, boxes, box_user_data,
			nb_capsules, capsules, capsule_user_data,
			volume, DownVector, MaxIterDown, &NbCollisions, min_dist, filterData, filterCallback, true))
		{
			if(NbCollisions)
			{
				CollisionFlags |= PxControllerFlag::eCOLLISION_DOWN;
				if(mUserParams.mHandleSlope)	// PT: I think the following fix shouldn't be performed when mHandleSlope is false.
				{
					// PT: the following code is responsible for a weird capsule behaviour,
					// when colliding against a highly tesselated terrain:
					// - with a large direction vector, the capsule gets stuck against some part of the terrain
					// - with a slower direction vector (but in the same direction!) the capsule manages to move
					// I will keep that code nonetheless, since it seems to be useful for them.

					// constrainedClimbingMode
					if ( constrainedClimbingMode && mContactPointHeight > OriginalBottomPoint + StepOffset)
					{
						mHitNonWalkable = true;
						if(!mWalkExperiment)
							return;
					}
					//~constrainedClimbingMode
				}
			}
		}
		//printf("AD:%.2f %.2f %.2f NS=%d\n", volume.mCenter.x, volume.mCenter.y, volume.mCenter.z, mNbCachedStatic);

		// TEST: do another down pass if we're on a non-walkable poly
		// ### kind of works but still not perfect
		// ### could it be because we zero the Y impulse later?
		// ### also check clamped response vectors
		if(mUserParams.mHandleSlope && mValidTri && direction[upDirection]<0.0f)
		{
			PxVec3 Normal;
		#ifdef USE_CONTACT_NORMAL_FOR_SLOPE_TEST
			Normal = mCN;
		#else
			mTouched.normal(Normal);
		#endif
			if(testSlope(Normal, upDirection, mUserParams.mSlopeLimit))
			{
				mHitNonWalkable = true;
				// Early exit if we're going to run this again anyway...
				if(!mWalkExperiment)	return;
		/*		CatchScene()->GetRenderer()->AddLine(mTouched.mVerts[0], mTouched.mVerts[1], ARGB_YELLOW);
				CatchScene()->GetRenderer()->AddLine(mTouched.mVerts[0], mTouched.mVerts[2], ARGB_YELLOW);
				CatchScene()->GetRenderer()->AddLine(mTouched.mVerts[1], mTouched.mVerts[2], ARGB_YELLOW);
		*/

				// ==========[ WALK EXPERIMENT ]===========================

				mNormalizeResponse=true;

				PxExtended Delta = volume.mCenter[upDirection] > OriginalHeight ? volume.mCenter[upDirection] - OriginalHeight : 0.0f;
				Delta += fabsf(direction[upDirection]);
				PxExtended Recover = Delta;

				NbCollisions=0;
				const PxExtended MD = Recover < min_dist ? Recover/float(MaxIter) : min_dist;

				PxVec3 RecoverPoint(0,0,0);
				RecoverPoint[upDirection]=-float(Recover);

				if(doSweepTest(user_data,
					user_data2,
					nb_boxes, boxes, box_user_data,
					nb_capsules, capsules, capsule_user_data,
					volume, RecoverPoint, MaxIter, &NbCollisions, float(MD),filterData, filterCallback))
				{
		//			if(NbCollisions)	CollisionFlags |= COLLISION_Y_DOWN;
					// PT: why did we do this ? Removed for now. It creates a bug (non registered event) when we land on a steep poly.
					// However this might have been needed when we were sliding on those polygons, and we didn't want the land anim to
					// start while we were sliding.
		//			if(NbCollisions)	CollisionFlags &= ~PxControllerFlag::eCOLLISION_DOWN;
				}
				mNormalizeResponse=false;
			}
		}
	}

	// Setup new collision flags
	collision_flags = CollisionFlags;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// This is an interface between NX users and the internal character controller module.

#include "CctController.h"
#include "CctBoxController.h"
#include "CctCapsuleController.h"
#include "CctCharacterControllerManager.h"
#include "PxActor.h"

static PX_INLINE PxExtended feedbackFilter(PxExtended val, PxExtended& memory, PxExtended sharpness)
{
	PX_ASSERT(sharpness>=PxExtended(0.0) && sharpness<=PxExtended(1.0) && "Invalid sharpness value in feedback filter");
			if(sharpness<PxExtended(0.0))	sharpness = PxExtended(0.0);
	else	if(sharpness>PxExtended(1.0))	sharpness = PxExtended(1.0);
	return memory = val * sharpness + memory * (PxExtended(1.0) - sharpness);
}

void Controller::move(SweptVolume& volume, const PxVec3& disp, PxU32 activeGroups, PxF32 minDist, PxU32& collisionFlags, PxF32 sharpness, const PxFilterData* filterData, PxSceneQueryFilterCallback* filterCallback, bool constrainedClimbingMode)
{
	SweepTest* ST = &mCctModule;

	// Init CCT with per-controller settings
	ST->mRenderBuffer	= mManager->mRenderBuffer;
	ST->mUserParams		= mUserParams;
	ST->mFirstUpdate	= true;

	const PxCCTUpAxis::Enum upDirection = mUserParams.mUpDirection;

	///////////

	Controller** boxUserData = NULL;
	PxExtendedBounds3* boxes = NULL;
	PxU32 nbBoxes = 0;

	Controller** capsuleUserData = NULL;
	PxExtendedCapsule* capsules = NULL;
	PxU32 nbCapsules = 0;

	if(1)
	{
		// Experiment - to do better
		PxU32 nbControllers = mManager->getNbControllers();
		Controller** controllers = mManager->getControllers();

		boxes = (PxExtendedBounds3*)PxAlloca(nbControllers*sizeof(PxExtendedBounds3));
		capsules = (PxExtendedCapsule*)PxAlloca(nbControllers*sizeof(PxExtendedCapsule));	// It's evil to waste that ram
		boxUserData = (Controller**)PxAlloca(nbControllers*sizeof(Controller*));
		capsuleUserData = (Controller**)PxAlloca(nbControllers*sizeof(Controller*));

		while(nbControllers--)
		{
			Controller* currentController = *controllers++;
			if(currentController==this)	continue;

			PxRigidDynamic* pActor = currentController->getActor();
			int nbShapes = pActor->getNbShapes();
			PX_ASSERT( nbShapes == 1 );
			PxShape* pCurrentShape = NULL;
			pActor->getShapes(&pCurrentShape, 1);

			// Depending on user settings the current controller can be:
			// - discarded
			// - always kept
			// - or tested against filtering flags
			PxCCTInteractionMode::Enum interactionMode = currentController->getInteraction();
			bool keepController = true;
			if(interactionMode==PxCCTInteractionMode::eEXCLUDE)			keepController = false;
			else if(interactionMode==PxCCTInteractionMode::eUSE_FILTER)	keepController = (activeGroups & ( 1 << pCurrentShape->getSimulationFilterData().word3))!=0;

			if(keepController)
			{
				if(currentController->mType==PxControllerShapeType::eBOX)
				{
					currentController->getWorldBox(boxes[nbBoxes]);
					boxUserData[nbBoxes++] = currentController;
				}
				else if(currentController->mType==PxControllerShapeType::eCAPSULE)
				{
					CapsuleController* CC = static_cast<CapsuleController*>(currentController);
					PxExtendedVec3 p0 = CC->mPosition;
					PxExtendedVec3 p1 = CC->mPosition;
					p0[upDirection] -= CC->mHeight*0.5f;
					p1[upDirection] += CC->mHeight*0.5f;
					capsules[nbCapsules].p0 = p0;
					capsules[nbCapsules].p1 = p1;
					capsules[nbCapsules].radius = CC->mRadius;
					capsuleUserData[nbCapsules++] = currentController;
				}
				else ASSERT(0);
			}
		}
	}

	///////////

	ST->mWalkExperiment = false;

	PxExtendedVec3 Backup = volume.mCenter;
	ST->moveCharacter(mScene,
		(Controller*)this,
		volume, disp,
		nbBoxes, nbBoxes ? boxes : NULL, nbBoxes ? (const void**)boxUserData : NULL,
		nbCapsules, nbCapsules ? capsules : NULL, nbCapsules ? (const void**)capsuleUserData : NULL,
		minDist, collisionFlags, filterData, filterCallback, constrainedClimbingMode);

	if(ST->mHitNonWalkable)
	{
		// A bit slow, but everything else I tried was less convincing...
		ST->mWalkExperiment = true;
		volume.mCenter = Backup;
		ST->moveCharacter(mScene,
			(Controller*)this,
			volume, disp,
			nbBoxes, nbBoxes ? boxes : NULL, nbBoxes ? (const void**)boxUserData : NULL,
			nbCapsules, nbCapsules ? capsules : NULL, nbCapsules ? (const void**)capsuleUserData : NULL,
			minDist, collisionFlags, filterData, filterCallback, constrainedClimbingMode);
		ST->mWalkExperiment = false;
	}

	if(sharpness<0.0f)
	volume.mCenter = Backup;

	// Copy results back
	mPosition = volume.mCenter;

	PxVec3 Delta = Backup - volume.mCenter;
	PxF32 deltaM2 = Delta.magnitudeSquared();
	if(deltaM2!=0.0f)
	{
		// Update kinematic actor
		if(mKineActor)
		{
			PxTransform targetPose = mKineActor->getGlobalPose();
			targetPose.p = PxVec3((float)mPosition.x, (float)mPosition.y, (float)mPosition.z);
			mKineActor->moveKinematic(targetPose);
		}
	}

	mFilteredPosition = mPosition;

	sharpness = fabsf(sharpness);

	// Apply feedback filter if needed
	if(sharpness<1.0f)
		mFilteredPosition[upDirection] = feedbackFilter(mPosition[upDirection], mMemory, sharpness);


	// if(manager->mRenderBuffer)
	//	Ps::RenderOutput(*mRenderBuffer) << PxDebugColor::eARGB_YELLOW << Ps::DebugBox(getBounds3(cctModule.mCachedTBV));
}


void BoxController::move(const PxVec3& disp, PxU32 activeGroups, PxF32 minDist, PxU32& collisionFlags, PxF32 sharpness, const PxFilterData* filterData, PxSceneQueryFilterCallback* filterCallback)
{
	// Create internal swept box
	SweptBox sweptBox;
	sweptBox.mCenter		= mPosition;
	sweptBox.mExtents		= mExtents;
	sweptBox.mHalfHeight	= mExtents[mUserParams.mUpDirection];	// UBI
	Controller::move(sweptBox, disp, activeGroups, minDist, collisionFlags, sharpness, filterData, filterCallback, false);
}


void CapsuleController::move(const PxVec3& disp, PxU32 activeGroups, PxF32 minDist, PxU32& collisionFlags, PxF32 sharpness, const PxFilterData* filterData, PxSceneQueryFilterCallback* filterCallback)
{
	// Create internal swept capsule
	SweptCapsule sweptCapsule;
	sweptCapsule.mCenter		= mPosition;
	sweptCapsule.mRadius		= mRadius;
	sweptCapsule.mHeight		= mHeight;
	sweptCapsule.mHalfHeight	= mHeight/2.0f + mRadius;	// UBI
	Controller::move(sweptCapsule, disp, activeGroups, minDist, collisionFlags, sharpness, filterData, filterCallback, mClimbingMode==PxCapsuleClimbingMode::eCONSTRAINED);
}

#if defined(PX_WINDOWS)
#include <windows.h>
#endif
#include <stdio.h>
void CharacterControllerManager::printStats()
{
    static volatile bool bPrintThis = false;
    if ( bPrintThis )
    {
        char buffer[256];
        sprintf(buffer, "%d - %d - %d\n", gNbIters, gNbFullUpdates, gNbPartialUpdates);
//      OutputDebugString(buffer);
        printf("%s",buffer);
    }
    gNbIters = 0;
    gNbFullUpdates = 0;
    gNbPartialUpdates = 0;
}

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


#ifndef PX_PHYSICS_GEOMUTILS_PX_OVERLAPTESTS
#define PX_PHYSICS_GEOMUTILS_PX_OVERLAPTESTS

#include "CmPhysXCommon.h"
#include "PxVec3.h"
#include "GuCollisionModel.h"

namespace physx
{

class PxMat34Legacy;
class PxGeometry;
class PxBoxGeometry;
class PxCapsuleGeometry;
class PxSphereGeometry;
class PxPlaneGeometry;
class PxConvexMeshGeometry;
class PxTriangleMeshGeometry;
class PxHeightFieldGeometry;
class PxMeshScale;

namespace Gu
{
	class Capsule;
	class Plane;
	class Sphere;
	class Box;
	class HeightFieldUtil;
	class ConvexMesh;

	// PT: there's no reason to put "world" in any of the names below. It doesn't have to be world space! It just has to be the same space.
	// In fact, in sake of accuracy none of those should be in world space, and we should pass a single relative transform to each of those tests.

	// PT: Sq::Box is redundant and should be removed:
	// * first by passing PxTransform + PxBoxGeometry = PxTransform + PxVec3 (extents)
	// * then by replacing PxTransform with PxVec3/PxQuat
	// - then by replacing PxQuat with the appropriate 3x3 matrix
	// - then by replacing the whole mess with either Gu::Box or PxcBox
	//
	// The design of low-level overlap tests should always be the same:
	// - lowest level code uses atomic types (PxVec3 / PxFloat / etc)
	// - inlined higher-level wrappers are provided for higher-level classes like Gu::Capsule / PxTransform / etc.
	// That way users are free to call the functions from any particular place, without being forced to create "Sq" or "Gu" or whatever class
	// when they don't need to. Unfortunately this is not at all the current design.

	bool intersectPlaneBox		(const Gu::Plane& plane, const Gu::Box& box);
	bool intersectPlaneCapsule	(const Gu::Capsule& capsule, const Gu::Plane& plane);
	bool intersectSphereSphere	(const Gu::Sphere& sphere0, const Gu::Sphere& sphere1);
	bool intersectSphereCapsule	(const Gu::Sphere& sphere, const Gu::Capsule& capsule);
	bool intersectSphereBox		(const Gu::Sphere& sphere, const Gu::Box& box);
	bool intersectBoxCapsule	(const Gu::Box& box, const Gu::Capsule& capsule);

//	bool intersectSphereConvex	(const PxSphereGeometry& sphereGeom, const PxTransform& sphereGlobalPose, const Gu::ConvexMesh& mesh, const PxMeshScale& meshScale, const PxTransform& convexGlobalPose, PxVec3* cachedSepAxis);
	bool intersectSphereConvex	(const Gu::Sphere& sphere,			const Gu::ConvexMesh& mesh, const PxMeshScale& meshScale, const PxTransform& convexGlobalPose, PxVec3* cachedSepAxis);
	bool intersectCapsuleConvex	(const PxCapsuleGeometry& capsGeom,	const PxTransform& capsGlobalPose, const Gu::ConvexMesh& mesh, const PxMeshScale& meshScale, const PxTransform& convexGlobalPose, PxVec3* cachedSepAxis);
	bool intersectBoxConvex		(const PxBoxGeometry& boxGeom,		const PxTransform& boxGlobalPose, const Gu::ConvexMesh& mesh, const PxMeshScale& meshScale, const PxTransform& convexGlobalPose, PxVec3* cachedSepAxis);
	bool intersectSphereMeshAny	(const Gu::Sphere& worldSphere,		const Ice::HybridModel& meshModel, const PxTransform& meshTransform, const PxMeshScale& scaling);
	bool intersectCapsuleMeshAny(const Gu::Capsule& worldCapsule,	const Ice::HybridModel& meshModel, const PxTransform& meshTransform, const PxMeshScale& scaling);
	bool intersectBoxMeshAny	(const Gu::Box& worldOBB,			const Ice::HybridModel& meshModel, const PxTransform& meshTransform, const PxMeshScale& scaling);

	PxU32 findOverlapSphereMesh	(const Gu::Sphere& worldSphere,		const Ice::HybridModel& meshModel, const PxTransform& meshTransform, const PxMeshScale& scaling, PxU32* PX_RESTRICT results, PxU32 maxResults, PxU32 startIndex, bool& overflow);
	PxU32 findOverlapCapsuleMesh(const Gu::Capsule& worldCapsule,	const Ice::HybridModel& meshModel, const PxTransform& meshTransform, const PxMeshScale& scaling, PxU32* PX_RESTRICT results, PxU32 maxResults, PxU32 startIndex, bool& overflow);
	PxU32 findOverlapOBBMesh	(const Gu::Box& worldOBB,			const Ice::HybridModel& meshModel, const PxTransform& meshTransform, const PxMeshScale& scaling, PxU32* PX_RESTRICT results, PxU32 maxResults, PxU32 startIndex, bool& overflow);

	bool intersectHeightFieldSphere		(const Gu::HeightFieldUtil& hfUtil, const Gu::Sphere& sphereInHfShape);
	bool intersectHeightFieldCapsule	(const Gu::HeightFieldUtil& hfUtil, const Gu::Capsule& capsuleInHfShape);
	bool intersectHeightFieldBox		(const Gu::HeightFieldUtil& hfUtil, const Gu::Box& boxInHfShape);
	bool intersectHeightFieldConvex		(const Gu::HeightFieldUtil& hfUtil, const PxMat34Legacy& hfAbsPose, const Gu::ConvexMesh& convexMesh, const PxMat34Legacy& convexAbsPose, const PxMeshScale& convexMeshScaling);

	bool checkOverlapSphere_boxGeom			(const PxGeometry& boxGeom,		const PxTransform& pose, const Gu::Sphere& sphere);
	bool checkOverlapSphere_sphereGeom		(const PxGeometry& sphereGeom,	const PxTransform& pose, const Gu::Sphere& sphere);
	bool checkOverlapSphere_capsuleGeom		(const PxGeometry& capsuleGeom,	const PxTransform& pose, const Gu::Sphere& sphere);
	bool checkOverlapSphere_planeGeom		(const PxGeometry& planeGeom,	const PxTransform& pose, const Gu::Sphere& sphere);
	bool checkOverlapSphere_convexGeom		(const PxGeometry& cvGeom,		const PxTransform& pose, const Gu::Sphere& sphere);
	bool checkOverlapSphere_triangleGeom	(const PxGeometry& triGeom,		const PxTransform& pose, const Gu::Sphere& sphere);
	bool checkOverlapSphere_heightFieldGeom	(const PxGeometry& hfGeom,		const PxTransform& pose, const Gu::Sphere& sphere);

	bool checkOverlapOBB_boxGeom			(const PxGeometry& boxGeom,		const PxTransform& pose, const Gu::Box& box);
	bool checkOverlapOBB_sphereGeom			(const PxGeometry& sphereGeom,	const PxTransform& pose, const Gu::Box& box);
	bool checkOverlapOBB_capsuleGeom		(const PxGeometry& capsuleGeom,	const PxTransform& pose, const Gu::Box& box);
	bool checkOverlapOBB_planeGeom			(const PxGeometry& planeGeom,	const PxTransform& pose, const Gu::Box& box);
	bool checkOverlapOBB_convexGeom			(const PxGeometry& cvGeom,		const PxTransform& pose, const Gu::Box& box);
	bool checkOverlapOBB_triangleGeom		(const PxGeometry& triGeom,		const PxTransform& pose, const Gu::Box& box);
	bool checkOverlapOBB_heightFieldGeom	(const PxGeometry& hfGeom,		const PxTransform& pose, const Gu::Box& box);

	bool checkOverlapCapsule_boxGeom		(const PxGeometry& boxGeom,		const PxTransform& pose, const Gu::Capsule& worldCapsule);
	bool checkOverlapCapsule_sphereGeom		(const PxGeometry& sphereGeom,	const PxTransform& pose, const Gu::Capsule& worldCapsule);
	bool checkOverlapCapsule_capsuleGeom	(const PxGeometry& capsuleGeom,	const PxTransform& pose, const Gu::Capsule& worldCapsule);
	bool checkOverlapCapsule_planeGeom		(const PxGeometry& planeGeom,	const PxTransform& pose, const Gu::Capsule& worldCapsule);
	bool checkOverlapCapsule_convexGeom		(const PxGeometry& cvGeom,		const PxTransform& pose, const Gu::Capsule& worldCapsule);
	bool checkOverlapCapsule_triangleGeom	(const PxGeometry& triGeom,		const PxTransform& pose, const Gu::Capsule& worldCapsule);
	bool checkOverlapCapsule_heightFieldGeom(const PxGeometry& hfGeom,		const PxTransform& pose, const Gu::Capsule& worldCapsule);

	// PT: this is just a shadow of what it used to be. We currently don't use TRIGGER_INSIDE anymore, but I leave it for now,
	// since I really want to put this back the way it was before.
	enum TriggerStatus
	{
		TRIGGER_DISJOINT,
		TRIGGER_INSIDE,
		TRIGGER_OVERLAP,
	};

	// PT: currently only used for convex triggers
	struct TriggerCache
	{
		PxVec3	dir;
		PxU16	state;
	};

	// PT: this is used both for Gu overlap queries and for triggers. Please do not duplicate that code.
	#define GEOM_OVERLAP_CALLBACK_PARAMS	const PxGeometry& geom0, const PxTransform& transform0, const PxGeometry& geom1, const PxTransform& transform1, Gu::TriggerCache* cache
	typedef bool (*GeomOverlapFunc)	(GEOM_OVERLAP_CALLBACK_PARAMS);
	extern const GeomOverlapFunc	gGeomOverlapMethodTable[][7];

	typedef bool (*GeomOverlapSphereFunc)	(const PxGeometry&, const PxTransform&, const Gu::Sphere&);
	extern const GeomOverlapSphereFunc gGeomOverlapSphereMap[7];

	typedef bool (*GeomOverlapOBBFunc)		(const PxGeometry&, const PxTransform&, const Gu::Box&);
	extern const GeomOverlapOBBFunc gGeomOverlapOBBMap[7];

	typedef bool (*GeomOverlapCapsuleFunc)	(const PxGeometry&, const PxTransform&, const Gu::Capsule&);
	extern const GeomOverlapCapsuleFunc gGeomOverlapCapsuleMap[7];

}  // namespace Gu

}

#endif

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


#include "PxMat34Legacy.h"
#include "PsIntrinsics.h"
#include "GuOverlapTests.h"

#include "CmScaling.h"
#include "GuHeightFieldUtil.h"
#include "GuGJKObjectSupport.h"
#include "PsUtilities.h"

#include "GuIntersectionBoxBox.h"
#include "GuIntersectionTriangleBox.h"
#include "GuDistancePointTriangle.h"
#include "GuDistancePointSegment.h"
#include "GuDistanceSegmentBox.h"
#include "GuDistanceSegmentSegment.h"
#include "GuDistanceSegmentTriangle.h"

#include "PxSphereGeometry.h"
#include "PxBoxGeometry.h"
#include "PxCapsuleGeometry.h"
#include "PxPlaneGeometry.h"
#include "PxConvexMeshGeometry.h"
#include "PxTriangleMeshGeometry.h"

#include "GJKSweep.h"

#include "GuCapsule.h"
#include "GuPlane.h"
#include "IceSupport.h"
#include "GuEdgeCache.h"
#include "GuBoxConversion.h"

#include "GuTriangleMesh.h"
#include "GuGeomUtilsInternal.h"
#include "GuConvexUtilsInternal.h"

using namespace physx;
using namespace Gu;

bool Gu::intersectPlaneBox(const Gu::Plane& plane, const Gu::Box& box)
{
	PxVec3 pts[8];
	box.computeBoxPoints(pts);

	for(PxU32 i=0;i<8;i++)
	{
		if(plane.distance(pts[i]) <= 0.0f)
			return true;
	}
	return false;
}

bool Gu::intersectPlaneCapsule(const Gu::Capsule& capsule, const Gu::Plane& plane)
{
	// We handle the capsule-plane collision with 2 sphere-plane collisions.
	// Seems ok so far, since plane is infinite.

	if(plane.distance(capsule.p0) < capsule.radius)
		return true;

	if(plane.distance(capsule.p1) < capsule.radius)
		return true;

	return false;
}

bool Gu::intersectSphereSphere(const Gu::Sphere& sphere0, const Gu::Sphere& sphere1)
{
	const PxVec3 delta = sphere1.center - sphere0.center;

	const PxReal distanceSq = delta.magnitudeSquared();

	const PxReal radSum = sphere0.radius + sphere1.radius;

	return distanceSq < radSum * radSum;
}

bool Gu::intersectSphereCapsule(const Gu::Sphere& sphere, const Gu::Capsule& capsule)
{
	const PxReal r = sphere.radius + capsule.radius;
	return Gu::distancePointSegmentSquared(capsule, sphere.center, NULL) < r*r;
}

bool Gu::intersectSphereBox(const Gu::Sphere& sphere, const Gu::Box& box)
{
	const PxVec3 delta = sphere.center - box.center;
	PxVec3 dRot = box.rot.transformTranspose(delta);	//transform delta into OBB body coords. (use method call!)

	//check if delta is outside ABB - and clip the vector to the ABB.
	bool outside = false;

	if (dRot.x < -box.extents.x)
	{ 
		outside = true; 
		dRot.x = -box.extents.x;
	}
	else if (dRot.x >  box.extents.x)
	{ 
		outside = true; 
		dRot.x = box.extents.x;
	}

	if (dRot.y < -box.extents.y)
	{ 
		outside = true; 
		dRot.y = -box.extents.y;
	}
	else if (dRot.y >  box.extents.y)
	{ 
		outside = true; 
		dRot.y = box.extents.y;
	}

	if (dRot.z < -box.extents.z)
	{ 
		outside = true; 
		dRot.z = -box.extents.z;
	}
	else if (dRot.z >  box.extents.z)
	{ 
		outside = true; 
		dRot.z = box.extents.z;
	}

	if (outside)	//if clipping was done, sphere center is outside of box.
	{
		const PxVec3 clippedDelta = box.rot.transform(dRot);	//get clipped delta back in world coords.

		const PxVec3 clippedVec = delta - clippedDelta;			  //what we clipped away.	
		const PxReal lenSquared = clippedVec.magnitudeSquared();
		const PxReal radius = sphere.radius;
		if (lenSquared > radius * radius) 
			return false;	//disjoint
	}
	return true;
}

bool Gu::intersectBoxCapsule(const Gu::Box& box, const Gu::Capsule& capsule)
{
	return Gu::distanceSegmentBoxSquared(capsule.p0, capsule.p1, box.center, box.extents, box.rot) < capsule.radius*capsule.radius;
}

//bool Gu::intersectSphereConvex(const PxSphereGeometry& sphereGeom, const PxTransform& sphereGlobalPose,
bool Gu::intersectSphereConvex(const Gu::Sphere& sphere,
						   const Gu::ConvexMesh& mesh, const PxMeshScale& meshScale, const PxTransform& convexGlobalPose,
						   PxVec3* cachedSepAxis)
{
	GJKConvexInterfaceCache gjkCache;

	GJKConvexSupport convexSupport(mesh, meshScale);
//	GJKSphereSupport sphereSupport(sphereGeom.radius);
	GJKSphereSupport sphereSupport(sphere.radius);

	PxVec3 sepAxisGuess;
	if(cachedSepAxis)
		sepAxisGuess = *cachedSepAxis;
	else
		sepAxisGuess = PxVec3(0.0f,0.0f,1.0f);

	PxVec3 destWorldNormalOnB;
	PxVec3 destWorldPointA;
	PxVec3 destWorldPointB;
	PxReal destDistance;

//	bool ok = convexConvexDistance(convexSupport, sphereSupport, convexGlobalPose, PxTransform(sphereGlobalPose.p, PxQuat::createIdentity()), sepAxisGuess,
	bool ok = convexConvexDistance(convexSupport, sphereSupport, convexGlobalPose, PxTransform(sphere.center, PxQuat::createIdentity()), sepAxisGuess,
		destWorldNormalOnB, destWorldPointA, destWorldPointB, destDistance, gjkCache);

	if(cachedSepAxis)
		*cachedSepAxis = sepAxisGuess;

	return !ok;	//return true if there is no distance.
}

bool Gu::intersectCapsuleConvex(const PxCapsuleGeometry& capsGeom, const PxTransform& capsGlobalPose,
							const Gu::ConvexMesh& mesh, const PxMeshScale& meshScale, const PxTransform& convexGlobalPose,
							PxVec3* cachedSepAxis)
{
	GJKConvexInterfaceCache gjkCache;

	GJKConvexSupport convexSupport(mesh, meshScale);
	GJKCapsuleSupport capsuleSupport(capsGeom.halfHeight, capsGeom.radius);

	PxVec3 sepAxisGuess;
	if(cachedSepAxis)
		sepAxisGuess = *cachedSepAxis;
	else
		sepAxisGuess = PxVec3(0.0f,0.0f,1.0f);

	PxVec3 destWorldNormalOnB;
	PxVec3 destWorldPointA;
	PxVec3 destWorldPointB;
	PxReal destDistance;

	bool ok = convexConvexDistance(convexSupport, capsuleSupport, convexGlobalPose, capsGlobalPose, sepAxisGuess,
		destWorldNormalOnB, destWorldPointA, destWorldPointB, destDistance, gjkCache);

	if(cachedSepAxis)
		*cachedSepAxis = sepAxisGuess;

	return !ok;	//return true if there is no distance.
}

bool Gu::intersectBoxConvex(const PxBoxGeometry& boxGeom, const PxTransform& boxGlobalPose,
						const Gu::ConvexMesh& mesh, const PxMeshScale& meshScale, const PxTransform& convexGlobalPose,
						PxVec3* cachedSepAxis)
{
	GJKConvexInterfaceCache gjkCache;

	GJKConvexSupport convexSupport(mesh, meshScale);
	GJKBoxSupport boxSupport(boxGeom.halfExtents);

	PxVec3 sepAxisGuess;
	if(cachedSepAxis)
		sepAxisGuess = *cachedSepAxis;
	else
		sepAxisGuess = PxVec3(0.0f,0.0f,1.0f);

	PxVec3 destWorldNormalOnB;
	PxVec3 destWorldPointA;
	PxVec3 destWorldPointB;
	PxReal destDistance;

	bool ok = convexConvexDistance(convexSupport, boxSupport, convexGlobalPose, boxGlobalPose, sepAxisGuess,
		destWorldNormalOnB, destWorldPointA, destWorldPointB, destDistance, gjkCache);

	if(cachedSepAxis)
		*cachedSepAxis = sepAxisGuess;

	return !ok;	//return true if there is no distance.
}

struct IntersectAnyVsMeshCallback : VolumeColliderTrigCallback
{
	IntersectAnyVsMeshCallback(const Ice::HybridModel& meshModel, const PxMat33& vSkew)
		: mMeshModel(meshModel), mVertexToShapeSkew(vSkew), mAnyHits(false)
	{
	}
	virtual	~IntersectAnyVsMeshCallback(){}

	const Ice::HybridModel&	mMeshModel;
	const PxMat33&			mVertexToShapeSkew;

	bool					mAnyHits;
	PxF32					mMinDist2;
	PxVec3					mCenter;
	Gu::Capsule				mOpcodeCapsule;
	Gu::Box					mWorldOBB;
	PxMat34Legacy			mVertexToBox;
};

template <int isSphere, int isCapsule, int isBox>
struct IntersectAnyVsMeshCallback_Any : IntersectAnyVsMeshCallback
{
	IntersectAnyVsMeshCallback_Any(const Ice::HybridModel& meshModel, const PxMat33& vSkew) : IntersectAnyVsMeshCallback(meshModel, vSkew)	{}
	virtual ~IntersectAnyVsMeshCallback_Any() {}

	virtual bool processResults(PxU32 count, const PxVec3* verts, const PxU32* indices)
	{
		if(mAnyHits)
			return false; // we need first contact only, abort traversal

		PxU32 numTrigs = count;
		while(numTrigs--)
		{
			PxVec3 v0, v1, v2;
			if (isBox)
			{
				v0 = mVertexToBox * verts[numTrigs*3+0];
				v1 = mVertexToBox * verts[numTrigs*3+1];
				v2 = mVertexToBox * verts[numTrigs*3+2];
			}
			else
			{
				v0 = mVertexToShapeSkew * verts[numTrigs*3+0];
				v1 = mVertexToShapeSkew * verts[numTrigs*3+1];
				v2 = mVertexToShapeSkew * verts[numTrigs*3+2];
			}

			if(
				(isSphere && Gu::distancePointTriangleSquared(mCenter, v0, v1 - v0, v2 - v0) < mMinDist2) ||
				(isCapsule && Gu::distanceSegmentTriangleSquared(mOpcodeCapsule, v0, v1-v0, v2-v0) < mMinDist2) ||
				(isBox && Gu::intersectTriangleBox(PxVec3(0.0f), mWorldOBB.extents, v0, v1, v2))
			)
			{
				mAnyHits = true;
				return false; // abort traversal if we are only interested in firstContact
			}
		}

		return true; // no triangles were hit if we are here, continue traversal
	}
};

// PT: TODO: unify this and the ones in PX2ICE.h. Also, figure out what it does!
PX_INLINE Gu::Box transform(const PxMat33& transfo, const Gu::Box& box)
{
	Gu::Box ret;
	PxMat33& obbBasis = ret.rot;

	obbBasis.column0 = transfo * (box.rot.column0 * box.extents.x);
	obbBasis.column1 = transfo * (box.rot.column1 * box.extents.y);
	obbBasis.column2 = transfo * (box.rot.column2 * box.extents.z);

	ret.center = transfo * box.center;
	ret.extents = Ps::optimizeBoundingBox(obbBasis);
	return ret;
}

struct ParamsAny
{
	ParamsAny() : mResult(false)	{}
	bool	mResult;
};
static bool gReportCallbackFirstContact(PxU32 primIndex, void* userData)
{
	ParamsAny* params = (ParamsAny*)userData;
	params->mResult = true;
	return false;	// PT: i.e. abort query if possible, we got our result
}

template <unsigned isSphere, unsigned isCapsule, unsigned isBox>
static bool intersectAnyVsMesh_NonIdentity(
	IntersectAnyVsMeshCallback& callback,
	const Gu::Sphere* worldSphere, const Gu::Capsule* worldCapsule, const Gu::Box* worldOBB,
	const HybridModelData& hmd,
	const PxTransform& meshTransform, const PxMeshScale& scaling,
	const Cm::Matrix34& absPose,
	const PxMat33& shapeToVertexSkew)
{
	// sphere center in shape space 
	if (isSphere)
	{
		callback.mCenter = absPose.transformTranspose(worldSphere->center);
		callback.mMinDist2 = worldSphere->radius*worldSphere->radius;

		// sphere bounds in vertex space
		PxBounds3 bounds = PxBounds3::basisExtent(shapeToVertexSkew * callback.mCenter, shapeToVertexSkew, PxVec3(worldSphere->radius));

		// do conservative opcode query
		Ice::HybridAABBCollider collider;
		collider.SetPrimitiveTests(false);

		Ice::CollisionAABB box;
		box.mCenter = bounds.getCenter();
		box.mExtents = bounds.getExtents();

		// AP: SetPrimitiveTests(false) is set above so we pass false here until we clean up the SetPrimitiveTests flags
		collider.Collide(box, hmd, false, &callback);
		return callback.mAnyHits;
	}

	else if (isCapsule)
	{
		callback.mOpcodeCapsule = *worldCapsule;
		callback.mMinDist2 = worldCapsule->radius * worldCapsule->radius;

		//transform world capsule into mesh shape's space
		Gu::Capsule vertexSpaceCapsule;
		callback.mOpcodeCapsule.p0 = absPose.transformTranspose(worldCapsule->p0);
		callback.mOpcodeCapsule.p1 = absPose.transformTranspose(worldCapsule->p1);

		// make vertex space OBB
		Gu::Box box;
		box.create(callback.mOpcodeCapsule);
		box = transform(PxMat34Legacy(shapeToVertexSkew, PxVec3(PxReal(0))), box);

		//do conservative opcode query
		Ice::HybridOBBCollider collider;
		collider.SetPrimitiveTests(false);
		collider.SetFullBoxBoxTest(false);	// PT: usually faster

		collider.Collide(box, hmd, &callback, NULL, NULL);
		return callback.mAnyHits;
	}
	
	else if (isBox)
	{
		callback.mWorldOBB = *worldOBB;

		const PxMat34Legacy absPose(meshTransform);	// PT: TODO: moved here so that we can progressively get rid of legacy matrices
		PxMat34Legacy vertexToWorldSkew(absPose.M * scaling.toMat33(), absPose.t);

		PxMat34Legacy worldToVertexSkew(false);
		vertexToWorldSkew.getInverse(worldToVertexSkew);

		//make vertex space OBB
		const Gu::Box vertexSpaceBox = transform(worldToVertexSkew, *worldOBB);

		// Setup the collider
		Ice::HybridOBBCollider collider;
		collider.SetPrimitiveTests(false);	// TODO: see if this is a good idea when scaling!
		collider.SetFullBoxBoxTest(false);	// PT: usually faster

		callback.mVertexToBox = buildMatrixFromBox(*worldOBB);
		callback.mVertexToBox.getInverseRT(callback.mVertexToBox);
		callback.mVertexToBox = callback.mVertexToBox * vertexToWorldSkew;

		// vertexSpaceBox is used for rtree, worldOBB is used for accurate testing vs vertices inside of callback
		collider.Collide(vertexSpaceBox, hmd, &callback, NULL, NULL);
		return callback.mAnyHits;
	}
	
	PX_ASSERT(0);
	return false;
}

template <unsigned isSphere, unsigned isCapsule, unsigned isBox>
static bool intersectAnyVsMesh_Any(
	const Gu::Sphere* worldSphere, const Gu::Capsule* worldCapsule, const Gu::Box* worldOBB,
	const Ice::HybridModel& meshModel, const PxTransform& meshTransform, const PxMeshScale& scaling)
{
	PX_ASSERT(isSphere + isCapsule + isBox == 1);
	HybridModelData hmd;
	meshModel.getHybridModelData(hmd);

	if(scaling.isIdentity())
	{
		// Convert transform to matrix
		const Cm::Matrix34 vertex2world(meshTransform);

		if (isSphere)
		{
			Ice::HybridSphereCollider collider;
			collider.SetPrimitiveTests(true);	// PT: enable primitive tests in this case since there's no second pass

			ParamsAny params;
			collider.Collide(gReportCallbackFirstContact, &params, *worldSphere, hmd, NULL, &vertex2world);
			return params.mResult;
		}
		else if (isCapsule)
		{
			Ice::HybridLSSCollider collider;
			collider.SetPrimitiveTests(true);	// PT: enable primitive tests in this case since there's no second pass

			ParamsAny params;
			collider.Collide(gReportCallbackFirstContact, &params, *worldCapsule, hmd, NULL, &vertex2world);
			return params.mResult;
		}
		else if (isBox)
		{
			Ice::HybridOBBCollider collider;
			collider.SetPrimitiveTests(true);		// PT: enable primitive tests in this case since there's no second pass

			VolumeColliderAnyHitCallback callback;
			collider.Collide(*worldOBB, hmd, &callback, NULL, &vertex2world, false);
			return callback.anyHits;
		}
	}
	else
	{
		const Cm::Matrix34 absPose(meshTransform);
		PxMat33 vertexToShapeSkew = scaling.toMat33();
		const PxMat33 shapeToVertexSkew = vertexToShapeSkew.getInverse();

		IntersectAnyVsMeshCallback_Any<isSphere, isCapsule, isBox> callback(
			meshModel, vertexToShapeSkew);

		return intersectAnyVsMesh_NonIdentity<isSphere, isCapsule, isBox>(
			callback,
			worldSphere, worldCapsule, worldOBB,
			hmd,
			meshTransform, scaling,
			absPose,
			shapeToVertexSkew);
	}
	return false;
}

struct LimitedResults
{
	PxU32*	mResults;
	PxU32	mNbResults;
	PxU32	mNbSkipped;
	PxU32	mMaxResults;
	PxU32	mStartIndex;
	bool	mOverflow;

	PX_FORCE_INLINE	void	reset()
	{
		mNbResults	= 0;
		mNbSkipped	= 0;
		mOverflow	= false;
	}

	PX_FORCE_INLINE	bool	add(PxU32 index)
	{
		if(mNbResults>=mMaxResults)
		{
			mOverflow = true;
			return false;
		}

		if(mNbSkipped>=mStartIndex)
			mResults[mNbResults++] = index;
		else
			mNbSkipped++;

		return true;
	}
};

struct ParamsAll
{
	ParamsAll(LimitedResults& results) : mResults(results)
	{
		results.reset();
	}
	LimitedResults&	mResults;
};
static bool gReportCallbackAllContacts(PxU32 primIndex, void* userData)
{
	ParamsAll* params = (ParamsAll*)userData;
	return params->mResults.add(primIndex);
}

	struct VolumeColliderLimitedResultsCallback : VolumeColliderTrigCallback
	{
		LimitedResults&	mResults;
		VolumeColliderLimitedResultsCallback(LimitedResults& results) : mResults(results)
		{
			results.reset();
		}
		virtual ~VolumeColliderLimitedResultsCallback() {}

		virtual bool processResults(PxU32 count, const PxVec3*, const PxU32* buf)
		{
			while(count--)
				if(!mResults.add(*buf++))
					return false;
			return true;
		}
	};

template <int isSphere, int isCapsule, int isBox>
struct IntersectAnyVsMeshCallback_All : IntersectAnyVsMeshCallback
{
	LimitedResults&		mResults;

	IntersectAnyVsMeshCallback_All(const Ice::HybridModel& meshModel, const PxMat33& vSkew, LimitedResults& results)
		: IntersectAnyVsMeshCallback(meshModel, vSkew), mResults(results)
	{
		mResults.reset();
	}

	virtual ~IntersectAnyVsMeshCallback_All() {}

	virtual bool processResults(PxU32 count, const PxVec3* verts, const PxU32* indices)
	{
		PxU32 numTrigs = count;
		while(numTrigs--)
		{
			PxVec3 v0, v1, v2;
			if (isBox)
			{
				v0 = mVertexToBox * verts[numTrigs*3+0];
				v1 = mVertexToBox * verts[numTrigs*3+1];
				v2 = mVertexToBox * verts[numTrigs*3+2];
			}
			else
			{
				v0 = mVertexToShapeSkew * verts[numTrigs*3+0];
				v1 = mVertexToShapeSkew * verts[numTrigs*3+1];
				v2 = mVertexToShapeSkew * verts[numTrigs*3+2];
			}

			if(
				(isSphere && Gu::distancePointTriangleSquared(mCenter, v0, v1 - v0, v2 - v0) < mMinDist2) ||
				(isCapsule && Gu::distanceSegmentTriangleSquared(mOpcodeCapsule, v0, v1-v0, v2-v0) < mMinDist2) ||
				(isBox && Gu::intersectTriangleBox(PxVec3(0.0f), mWorldOBB.extents, v0, v1, v2))
			)
			{
				if(!mResults.add(indices[numTrigs]))
					return false;
				mAnyHits = true;
			}
		}
		return true;
	}
};

template <unsigned isSphere, unsigned isCapsule, unsigned isBox>
static PxU32 intersectAnyVsMesh_All(
	const Gu::Sphere* worldSphere, const Gu::Capsule* worldCapsule, const Gu::Box* worldOBB,
	const Ice::HybridModel& meshModel, const PxTransform& meshTransform, const PxMeshScale& scaling,
	LimitedResults& results)
{
	PX_ASSERT(isSphere + isCapsule + isBox == 1);
	HybridModelData hmd;
	meshModel.getHybridModelData(hmd);

	if(scaling.isIdentity())
	{
		// Convert transform to matrix
		const Cm::Matrix34 vertex2world(meshTransform);

		if (isSphere)
		{
			Ice::HybridSphereCollider collider;
			collider.SetPrimitiveTests(true);	// PT: enable primitive tests in this case since there's no second pass

			ParamsAll params(results);
			collider.Collide(gReportCallbackAllContacts, &params, *worldSphere, hmd, NULL, &vertex2world);
			return results.mNbResults;
		}
		else if (isCapsule)
		{
			Ice::HybridLSSCollider collider;
			collider.SetPrimitiveTests(true);	// PT: enable primitive tests in this case since there's no second pass

			ParamsAll params(results);
			collider.Collide(gReportCallbackAllContacts, &params, *worldCapsule, hmd, NULL, &vertex2world);
			return results.mNbResults;
		}
		else if (isBox)
		{
			Ice::HybridOBBCollider collider;
			collider.SetPrimitiveTests(true);		// PT: enable primitive tests in this case since there's no second pass

			VolumeColliderLimitedResultsCallback callback(results);
			collider.Collide(*worldOBB, hmd, &callback, NULL, &vertex2world, false);
			return results.mNbResults;
		}
	}
	else
	{
		const Cm::Matrix34 absPose(meshTransform);
		PxMat33 vertexToShapeSkew = scaling.toMat33();
		const PxMat33 shapeToVertexSkew = vertexToShapeSkew.getInverse();

		IntersectAnyVsMeshCallback_All<isSphere, isCapsule, isBox> callback(
			meshModel, vertexToShapeSkew, results);

		intersectAnyVsMesh_NonIdentity<isSphere, isCapsule, isBox>(
			callback,
			worldSphere, worldCapsule, worldOBB,
			hmd,
			meshTransform, scaling,
			absPose,
			shapeToVertexSkew);
		return results.mNbResults;
	}
	return 0;
}

bool Gu::intersectSphereMeshAny(const Gu::Sphere& worldSphere, const Ice::HybridModel& meshModel,
							 const PxTransform& meshTransform, const PxMeshScale& scaling)
{
	return intersectAnyVsMesh_Any<1,0,0>(&worldSphere, NULL, NULL, meshModel, meshTransform, scaling);
}

bool Gu::intersectCapsuleMeshAny(const Gu::Capsule& worldCapsule, const Ice::HybridModel& meshModel,
							  const PxTransform& meshTransform, const PxMeshScale& scaling)
{
	return intersectAnyVsMesh_Any<0,1,0>(NULL, &worldCapsule, NULL, meshModel, meshTransform, scaling);
}

bool Gu::intersectBoxMeshAny(const Gu::Box& worldOBB, const Ice::HybridModel& meshModel,
						  const PxTransform& meshTransform, const PxMeshScale& scaling)
{
	return intersectAnyVsMesh_Any<0,0,1>(NULL, NULL, &worldOBB, meshModel, meshTransform, scaling);
}

PxU32 Gu::findOverlapSphereMesh(const Gu::Sphere& worldSphere, const Ice::HybridModel& meshModel,
								const PxTransform& meshTransform, const PxMeshScale& scaling,
								PxU32* PX_RESTRICT results, PxU32 maxResults, PxU32 startIndex, bool& overflow)
{
	LimitedResults limitedResults;
	limitedResults.mResults		= results;
	limitedResults.mMaxResults	= maxResults;
	limitedResults.mStartIndex	= startIndex;
	const PxU32 nbResults = intersectAnyVsMesh_All<1,0,0>(&worldSphere, NULL, NULL, meshModel, meshTransform, scaling, limitedResults);
	overflow = limitedResults.mOverflow;
	return nbResults;
}

PxU32 Gu::findOverlapCapsuleMesh(const Gu::Capsule& worldCapsule, const Ice::HybridModel& meshModel,
								const PxTransform& meshTransform, const PxMeshScale& scaling,
								PxU32* PX_RESTRICT results, PxU32 maxResults, PxU32 startIndex, bool& overflow)
{
	LimitedResults limitedResults;
	limitedResults.mResults		= results;
	limitedResults.mMaxResults	= maxResults;
	limitedResults.mStartIndex	= startIndex;
	const PxU32 nbResults = intersectAnyVsMesh_All<0,1,0>(NULL, &worldCapsule, NULL, meshModel, meshTransform, scaling, limitedResults);
	overflow = limitedResults.mOverflow;
	return nbResults;
}

PxU32 Gu::findOverlapOBBMesh(	const Gu::Box& worldOBB, const Ice::HybridModel& meshModel,
								const PxTransform& meshTransform, const PxMeshScale& scaling,
								PxU32* PX_RESTRICT results, PxU32 maxResults, PxU32 startIndex, bool& overflow)
{
	LimitedResults limitedResults;
	limitedResults.mResults		= results;
	limitedResults.mMaxResults	= maxResults;
	limitedResults.mStartIndex	= startIndex;
	const PxU32 nbResults = intersectAnyVsMesh_All<0,0,1>(NULL, NULL, &worldOBB, meshModel, meshTransform, scaling, limitedResults);
	overflow = limitedResults.mOverflow;
	return nbResults;
}

bool Gu::intersectHeightFieldSphere(const Gu::HeightFieldUtil& hfUtil, const Gu::Sphere& sphereInHfShape)
{
	const Gu::HeightField& hf = hfUtil.getHeightField();

	// sample the sphere center in the heightfield to find out
	// if we have penetration with more than the sphere radius
	if (hfUtil.isShapePointOnHeightField(sphereInHfShape.center.x, sphereInHfShape.center.z))
	{
		// The sphere origin projects within the bounds of the heightfield in the X-Z plane
		PxReal sampleHeight = hfUtil.getHeightAtShapePoint(sphereInHfShape.center.x, sphereInHfShape.center.z);
		PxReal deltaHeight = sphereInHfShape.center.y - sampleHeight;
		if (hf.isDeltaHeightInsideExtent(deltaHeight))
		{
			// The sphere origin is 'below' the heightfield surface
			PxU32 feature = hfUtil.getFeatureIndexAtShapePoint(sphereInHfShape.center.x, sphereInHfShape.center.z);
			if (feature != 0xffffffff)
			{
				return true;
			}
			return false;
		}
	}

	const PxReal radiusSquared = sphereInHfShape.radius * sphereInHfShape.radius;

	const PxVec3 sphereInHF = hfUtil.shape2hfp(sphereInHfShape.center);

	const PxReal radiusOverRowScale = sphereInHfShape.radius * PxAbs(hfUtil.getOneOverRowScale());
	const PxReal radiusOverColumnScale = sphereInHfShape.radius * PxAbs(hfUtil.getOneOverColumnScale());

	const PxU32 minRow = hf.getMinRow(sphereInHF.x - radiusOverRowScale);
	const PxU32 maxRow = hf.getMaxRow(sphereInHF.x + radiusOverRowScale);
	const PxU32 minColumn = hf.getMinColumn(sphereInHF.z - radiusOverColumnScale);
	const PxU32 maxColumn = hf.getMaxColumn(sphereInHF.z + radiusOverColumnScale);

	for (PxU32 r = minRow; r < maxRow; r++) 
	{
		for (PxU32 c = minColumn; c < maxColumn; c++) 
		{

			// x--x--x
			// | x   |
			// x  x  x
			// |   x |
			// x--x--x
			PxVec3 pcp[11];
			PxU32 pcf[11];
			PxU32 npcp = 0;
			npcp = hfUtil.findClosestPointsOnCell(r, c, sphereInHfShape.center, pcp, pcf);

			for(PxU32 pi = 0; pi < npcp; pi++)
			{
				if (pcf[pi] == 0xffffffff) continue;

				PxVec3 d = sphereInHfShape.center - pcp[pi];
				
				PxReal ll = d.magnitudeSquared();
					
				if (ll > radiusSquared) 
					// Too far
					continue;

				return true;
			}
		}
	}
	return false;
}

bool Gu::intersectHeightFieldCapsule(const Gu::HeightFieldUtil& hfUtil, const Gu::Capsule& capsuleInHfShape)
{
	const Gu::HeightField& hf = hfUtil.getHeightField();

	const PxReal radius = capsuleInHfShape.radius;
	const PxReal radiusOverRowScale = radius * PxAbs(hfUtil.getOneOverRowScale());
	const PxReal radiusOverColumnScale = radius * PxAbs(hfUtil.getOneOverColumnScale());

	PxVec3 verticesInHfShape[2];
	verticesInHfShape[0] = capsuleInHfShape.p0;
	verticesInHfShape[1] = capsuleInHfShape.p1;

	PxU32 absMinRow = 0xffffffff;
	PxU32 absMaxRow = 0;
	PxU32 absMinColumn = 0xffffffff;
	PxU32 absMaxColumn = 0;

	PxReal radiusSquared = radius * radius;

	for (PxU32 i = 0; i<2; i++)
	{
		const PxVec3& sphereInHfShape = verticesInHfShape[i];

		// we have to do this first to update the absMin / absMax correctly even if
		// we decide to continue from inside the deep penetration code.

		const PxVec3 sphereInHF = hfUtil.shape2hfp(sphereInHfShape);

		const PxU32 minRow = hf.getMinRow(sphereInHF.x - radiusOverRowScale);
		const PxU32 maxRow = hf.getMaxRow(sphereInHF.x + radiusOverRowScale);
		const PxU32 minColumn = hf.getMinColumn(sphereInHF.z - radiusOverColumnScale);
		const PxU32 maxColumn = hf.getMaxColumn(sphereInHF.z + radiusOverColumnScale);

		if (minRow < absMinRow)			absMinRow = minRow;
		if (minColumn < absMinColumn)	absMinColumn = minColumn;
		if (maxRow > absMaxRow)			absMaxRow = maxRow;
		if (maxColumn > absMaxColumn)	absMaxColumn = maxColumn;

		if (hfUtil.isShapePointOnHeightField(sphereInHfShape.x, sphereInHfShape.z))
		{
			// The sphere origin projects within the bounds of the heightfield in the X-Z plane
			PxReal sampleHeight = hfUtil.getHeightAtShapePoint(sphereInHfShape.x, sphereInHfShape.z);
			PxReal deltaHeight = sphereInHfShape.y - sampleHeight;
			if (hf.isDeltaHeightInsideExtent(deltaHeight))
			{
				// The sphere origin is 'below' the heightfield surface
				PxU32 feature = hfUtil.getFeatureIndexAtShapePoint(sphereInHfShape.x, sphereInHfShape.z);
				if (feature != 0xffffffff)
				{
					return true;
				}
				continue;
			}
		}

		for (PxU32 r = minRow; r < maxRow; r++) 
		{
			for (PxU32 c = minColumn; c < maxColumn; c++) 
			{

				// x--x--x
				// | x   |
				// x  x  x
				// |   x |
				// x--x--x
				PxVec3 pcp[11];
				PxU32 pcf[11];
				PxU32 npcp = 0;
				npcp = hfUtil.findClosestPointsOnCell(r, c, sphereInHfShape, pcp, pcf, false);

				for(PxU32 pi = 0; pi < npcp; pi++)
				{
					if (pcf[pi] == 0xffffffff) continue;

					PxVec3 d = sphereInHfShape - pcp[pi];

					if (hf.isDeltaHeightOppositeExtent(d.y))
					{
						// We are 'above' the heightfield

						PxReal ll = d.magnitudeSquared();

						if (ll > radiusSquared) 
							// Too far above
							continue;

						return true;
					}
				}
			}
		}
	}

	PxU32 row, column;
	for(row = absMinRow; row <= absMaxRow; row++)
	{
		for (column = absMinColumn; column <= absMaxColumn; column++)
		{
			const PxU32 vertexIndex = row * hf.getNbColumnsFast() + column;
			const PxU32 firstEdge = 3 * vertexIndex;
			// omg I am sorry about this code but I can't find a simpler way:
			//  last column will only test edge 2
			//  last row will only test edge 0
			//  and most importantly last row and column will not go inside the for
			const PxU32 minEi = (column == absMaxColumn) ? 2 : 0; 
			const PxU32 maxEi = (row    == absMaxRow   ) ? 1 : 3; 
			for (PxU32 ei = minEi; ei < maxEi; ei++)
			{
				const PxU32 edgeIndex = firstEdge + ei;

const PxU32 cell = vertexIndex;
PX_ASSERT(cell == edgeIndex / 3);
const PxU32 row_    = row;
PX_ASSERT(row_ == cell / hf.getNbColumnsFast());
const PxU32 column_ = column;
PX_ASSERT(column_ == cell % hf.getNbColumnsFast());

//				const PxU32 feature = hfUtil.getEdgeFeatureIndex(edgeIndex);
				const PxU32 feature = hfUtil.getEdgeFeatureIndex(edgeIndex, cell, row_, column_);
				if (feature != 0xffffffff)
				{
					PxVec3 origin;
					PxVec3 direction;
//					hfUtil.getEdge(edgeIndex, origin, direction);
					hfUtil.getEdge(edgeIndex, cell, row_, column_, origin, direction);

					PxReal s, t;
					const PxReal ll = Gu::distanceSegmentSegmentSquared(capsuleInHfShape, Gu::Segment(origin, origin + direction), &s, &t);
					if (ll < radiusSquared)
					{
						return true;
					}
				}
			}
		}
	}
	return false;
}

namespace physx
{
namespace Gu
{
	// ptchernev TODO: make sure these are ok before shipping
	const bool gCompileBoxVertex         = true;
	const bool gCompileConvexVertex      = true;
	const bool gCompileEdgeEdge          = true;
	const bool gCompileHeightFieldVertex = true;

	const PxReal signs[24] = 
	{
		-1,-1,-1,
		-1,-1, 1,
		-1, 1,-1,
		-1, 1, 1,
		 1,-1,-1,
		 1,-1, 1,
		 1, 1,-1,
		 1, 1, 1,
	};

	const char edges[24] = 
	{
		0,1,
		1,3,
		3,2,
		2,0,
		4,5,
		5,7,
		7,6,
		6,4,
		0,4,
		1,5,
		2,6,
		3,7,
	};
	
	struct TriggerTraceSegmentCallback
	{
		bool intersection;

		PX_INLINE TriggerTraceSegmentCallback() : intersection(false)
		{
		}

		PX_INLINE bool underFaceHit(
			const Gu::HeightFieldUtil& hfUtil, const PxVec3& normal,
			const PxVec3& point, PxF32 x, PxF32 z, PxF32 rayh, PxU32 triangleIndex)
		{
			/* AP: this callback part is no longer used because we no longer rely on hf thickness for tests to work
			const Gu::HeightField& hf = hfUtil.getHeightField();
			PxF32 y = hfUtil.getHeightAtShapePoint(x, z); // TODO: optmization opportunity - this can be derived cheaply inside traceSegment
			PxReal dy = y - rayh;
			if (hf.isDeltaHeightInsideExtent(dy))
			{
				PxU32 feature = hfUtil.getFeatureIndexAtTriangleIndex(triangleIndex);
				if (feature != 0xffffffff)
				{
					intersection = true;
					return false;
				}
			}
			*/
			return true;
		}

		PX_INLINE bool faceHit(const Gu::HeightFieldUtil& hfUtil, const PxVec3& point, PxU32 triangleIndex)
		{
			intersection = true;
			return true;
		}
	};
} // namespace
}

bool Gu::intersectHeightFieldBox(const Gu::HeightFieldUtil& hfUtil, const Gu::Box& boxInHfShape)
{
	const Gu::HeightField& hf = hfUtil.getHeightField();

	PxU32 i;

	// Get box vertices
	PxVec3 boxVertices[8];
	for (i=0; i<8; i++) 
	{
		boxVertices[i] = PxVec3(boxInHfShape.extents.x*signs[3*i], boxInHfShape.extents.y*signs[3*i+1], boxInHfShape.extents.z*signs[3*i+2]);
	}

	// Transform box vertices to HeightFieldShape space
	PxVec3 boxVerticesInHfShape[8];
	for (i=0; i<8; i++) 
	{
		boxVerticesInHfShape[i] = boxInHfShape.transform(boxVertices[i]);
	}

	// Test box vertices.
	if (gCompileBoxVertex)
	{
		for (i=0; i<8; i++) 
		{
			const PxVec3& boxVertexInHfShape = boxVerticesInHfShape[i];
			if (hfUtil.isShapePointOnHeightField(boxVertexInHfShape.x, boxVertexInHfShape.z))
			{
				PxReal y = hfUtil.getHeightAtShapePoint(boxVertexInHfShape.x, boxVertexInHfShape.z);
				PxReal dy = boxVertexInHfShape.y - y;
				if (hf.isDeltaHeightInsideExtent(dy))
				{
					PxU32 feature = hfUtil.getFeatureIndexAtShapePoint(boxVertexInHfShape.x, boxVertexInHfShape.z);
					if (feature != 0xffffffff)
					{
						return true;
					}
				}
			}
		}
	}

	// Test box edges.
	if (gCompileEdgeEdge)
	{
		for (i=0; i<12; i++) 
		{
			const PxVec3 v0 = boxVerticesInHfShape[edges[2*i]];
			const PxVec3 v1 = boxVerticesInHfShape[edges[2*i+1]];
			TriggerTraceSegmentCallback cb;
			hfUtil.traceSegment<TriggerTraceSegmentCallback, false, false>(v0, v1, &cb);
			if (cb.intersection)
				return true;
		}
	}

	// Test HeightField vertices.
	if (gCompileHeightFieldVertex)
	{
		PxMat34Legacy hfShape2BoxShape(false);
		{
			const PxMat34Legacy boxPose = buildMatrixFromBox(boxInHfShape);
			boxPose.getInverseRT(hfShape2BoxShape);
		}

		PxReal minx(PX_MAX_REAL);
		PxReal minz(PX_MAX_REAL);
		PxReal maxx(-PX_MAX_REAL);
		PxReal maxz(-PX_MAX_REAL);

		for (i=0; i<8; i++) 
		{
			const PxVec3& boxVertexInHfShape = boxVerticesInHfShape[i];
			if (boxVertexInHfShape.x < minx) minx = boxVertexInHfShape.x;
			if (boxVertexInHfShape.z < minz) minz = boxVertexInHfShape.z;
			if (boxVertexInHfShape.x > maxx) maxx = boxVertexInHfShape.x;
			if (boxVertexInHfShape.z > maxz) maxz = boxVertexInHfShape.z;
		}

		const PxReal oneOverRowScale = hfUtil.getOneOverRowScale();
		const PxReal oneOverColumnScale = hfUtil.getOneOverColumnScale();
		const PxU32 minRow = hf.getMinRow(minx * oneOverRowScale);
		const PxU32 maxRow = hf.getMaxRow(maxx * oneOverRowScale);
		const PxU32 minColumn = hf.getMinColumn(minz * oneOverColumnScale);
		const PxU32 maxColumn = hf.getMaxColumn(maxz * oneOverColumnScale);

		for (PxU32 row = minRow; row <= maxRow; row++)
		{
			for (PxU32 column = minColumn; column <= maxColumn; column++)
			{
				PxU32 vertexIndex = row * hf.getNbColumnsFast() + column;
				// ptchernev lastchange
				//PxU32 feature = hfShape.getVertexFeatureIndex(vertexIndex);
				// This is expensive, do we need to have it?
				if (/*(feature != 0xffffffff) && */hfUtil.isCollisionVertex(vertexIndex, row, column))
				{
					// check if hf vertex is inside the box
					const PxHeightFieldGeometry& geom = hfUtil.getHeightFieldGeometry();
					PxVec3 hfVertex(geom.rowScale * row, geom.heightScale * hf.getHeight(vertexIndex), geom.columnScale * column);
					PxVec3 hfVertexInBoxShape = hfShape2BoxShape * hfVertex;
					if ((PxAbs(hfVertexInBoxShape.x) - boxInHfShape.extents.x < 0)
					 && (PxAbs(hfVertexInBoxShape.y) - boxInHfShape.extents.y < 0)
					 && (PxAbs(hfVertexInBoxShape.z) - boxInHfShape.extents.z < 0))
					{
						return true;
					}
				}
			}
		}
	}
	return false;
}


// ptchernev NOTE: 
// I went for the quick and dirty approach here so there will be no data copy when going 
// between PxVec3 and Point. Compile time asserts are used to make sure casting is safe.
bool Gu::intersectHeightFieldConvex(const Gu::HeightFieldUtil& hfUtil, const PxMat34Legacy& hfAbsPose, const Gu::ConvexMesh& convexMesh, const PxMat34Legacy& convexAbsPose, const PxMeshScale& convexMeshScaling)
{
	PxU32 i;
	PxU32 row;
	PxU32 column;

	const PxMat34Legacy vertexToShapeSkew(convexMeshScaling.toMat33(), PxVec3(PxReal(0))); 

	PxMat34Legacy convexShape2HfShapeSkew;
	convexShape2HfShapeSkew.multiplyInverseRTLeft(hfAbsPose, convexAbsPose * vertexToShapeSkew);

	// Allocate space for transformed vertices.
	const ConvexHullData& hull = convexMesh.getHull();
	PxVec3* convexVerticesInHfShape = (PxVec3*)PxAlloca(hull.mNbHullVertices*sizeof(PxVec3));

	// Transform vertices to height field shape
	const PxVec3* hullVerts = hull.getHullVertices();
	for(i = 0; i<hull.mNbHullVertices; i++)
	{
		convexVerticesInHfShape[i] = convexShape2HfShapeSkew * hullVerts[i];
	}

	PxBounds3 convexBoundsInHfShape = PxBounds3::empty();

	// Compute bounds of convex in hf space
	for(i = 0; i<hull.mNbHullVertices; i++)
	{
		convexBoundsInHfShape.include(convexVerticesInHfShape[i]);
	}

	// Compute the height field extreme over the bounds area.
	const Gu::HeightField& hf = hfUtil.getHeightField();
	PxReal hfExtreme = (hf.getThicknessFast() <= 0) ? -PX_MAX_REAL : PX_MAX_REAL;
	const PxReal oneOverRowScale = hfUtil.getOneOverRowScale();
	const PxReal oneOverColumnScale = hfUtil.getOneOverColumnScale();
	const PxReal rowScale = (1.0f / hfUtil.getOneOverRowScale());
	const PxReal columnScale = (1.0f / hfUtil.getOneOverColumnScale());
	const PxReal heightScale = (1.0f / hfUtil.getOneOverHeightScale());

	PxU32 minRow;
	PxU32 maxRow;
	PxU32 minColumn;
	PxU32 maxColumn;

	// negative scale crap
	if (oneOverRowScale > 0)
	{
		minRow = hf.getMinRow(convexBoundsInHfShape.minimum.x * oneOverRowScale);
		maxRow = hf.getMaxRow(convexBoundsInHfShape.maximum.x * oneOverRowScale);
	}
	else
	{
		minRow = hf.getMinRow(convexBoundsInHfShape.maximum.x * oneOverRowScale);
		maxRow = hf.getMaxRow(convexBoundsInHfShape.minimum.x * oneOverRowScale);
	}

	if (oneOverColumnScale > 0)
	{
		minColumn = hf.getMinColumn(convexBoundsInHfShape.minimum.z * oneOverColumnScale);
		maxColumn = hf.getMaxColumn(convexBoundsInHfShape.maximum.z * oneOverColumnScale);
	}
	else
	{
		minColumn = hf.getMinColumn(convexBoundsInHfShape.maximum.z * oneOverColumnScale);
		maxColumn = hf.getMaxColumn(convexBoundsInHfShape.minimum.z * oneOverColumnScale);
	}

	for (row = minRow; row <= maxRow; row++)
	{
		for (column = minColumn; column <= maxColumn; column++)
		{
			PxReal h = hf.getHeight(row * hf.getNbColumnsFast() + column);
			hfExtreme = (hf.getThicknessFast() <= 0) ? PxMax(hfExtreme, h) : PxMin(hfExtreme, h);
		}
	}
	hfExtreme *= heightScale;


	// Return if convex is on the wrong side of the extreme.
	if (hf.getThicknessFast() <= 0)
	{
		if (convexBoundsInHfShape.minimum.y > hfExtreme) return false;
	}
	else
	{
		if (convexBoundsInHfShape.maximum.y < hfExtreme) return false;
	}


	// Test convex vertices
	if (gCompileConvexVertex)
	{
		for (i=0; i<hull.mNbHullVertices; i++) 
		{
			const PxVec3& convexVertexInHfShape = convexVerticesInHfShape[i];
			bool insideExtreme = (hf.getThicknessFast() <= 0) ? (convexVertexInHfShape.y < hfExtreme) : (convexVertexInHfShape.y > hfExtreme);
			if (insideExtreme && hfUtil.isShapePointOnHeightField(convexVertexInHfShape.x, convexVertexInHfShape.z))
			{
				PxReal y = hfUtil.getHeightAtShapePoint(convexVertexInHfShape.x, convexVertexInHfShape.z);
				PxReal dy = convexVertexInHfShape.y - y;
				if (hf.isDeltaHeightInsideExtent(dy))
				{
					PxU32 feature = hfUtil.getFeatureIndexAtShapePoint(convexVertexInHfShape.x, convexVertexInHfShape.z);
					if (feature != 0xffffffff)
					{
						return true;
					}
				}
			}
		} 
	}

	// Test convex edges.
	if (gCompileEdgeEdge)
	{
		EdgeCache edgeCache;
		PxU32 numPolygons = hull.mNbPolygons;
		const Gu::HullPolygonData* polygons = hull.mPolygons;
		const PxU8* const vertexData = hull.getVertexData8();
		while (numPolygons--)
		{
			const Gu::HullPolygonData& polygon = *polygons++;

			const PxU8* verts = vertexData + polygon.mVRef8;

			PxU32 numEdges = polygon.mNbVerts;

			PxU32 a = numEdges - 1;
			PxU32 b = 0;
			while(numEdges--)
			{
				PxU8 vi0 =  verts[a];
				PxU8 vi1 =	verts[b];

				if(vi1 < vi0)
				{
					PxU8 tmp = vi0;
					vi0 = vi1;
					vi1 = tmp;
				}

				if (edgeCache.isInCache(vi0, vi1))	//avoid processing edges 2x if possible (this will typically have cache misses about 5% of the time leading to 5% redundant work.
					continue;

				const PxVec3& sv0 = convexVerticesInHfShape[vi0];
				const PxVec3& sv1 = convexVerticesInHfShape[vi1];
				a = b;
				b++;


				if (hf.getThicknessFast() <= 0) 
				{
					if ((sv0.y > hfExtreme) && (sv1.y > hfExtreme)) continue;
				}
				else
				{
					if ((sv0.y < hfExtreme) && (sv1.y < hfExtreme)) continue;
				}
				const PxVec3 v0 = sv0;
				const PxVec3 v1 = sv1;
				TriggerTraceSegmentCallback cb;
				hfUtil.traceSegment<TriggerTraceSegmentCallback, false, false>(v0, v1, &cb);
				if (cb.intersection)
					return true;
			}
		}
	}

	// Test HeightField vertices
	if (gCompileHeightFieldVertex)
	{
		PxMat34Legacy hfShape2ConvexShapeSkew;
		hfShape2ConvexShapeSkew.multiplyInverseRTLeft(convexAbsPose, hfAbsPose);
		hfShape2ConvexShapeSkew = vertexToShapeSkew * hfShape2ConvexShapeSkew; 
		

		for (row = minRow; row <= maxRow; row++)
		{
			for (column = minColumn; column <= maxColumn; column++)
			{
				PxU32 hfVertexIndex = row * hf.getNbColumnsFast() + column;
				//PxU32 feature = hfShape.getVertexFeatureIndex(hfVertexIndex);
				// ptchernev lastchange
				if (/*(feature != 0xffffffff) && */hfUtil.isCollisionVertex(hfVertexIndex, row, column))
				{
					// Check if hf vertex is inside the convex
					PxVec3 hfVertex(rowScale * row, heightScale * hf.getHeight(hfVertexIndex), columnScale * column);
					PxVec3 hfVertexInConvexShape = hfShape2ConvexShapeSkew * hfVertex;

					bool inside = true;
					for (PxU32 poly = 0; poly < hull.mNbPolygons; poly++)
					{
						PxReal d = hull.mPolygons[poly].mPlane.distance(hfVertexInConvexShape);
						if (d >= 0)
						{
							inside = false;
							break;
						}
					}
					if (inside) return true;
				}
			}
		}
	}
	return false;
}

/////////////////////////////////////////////////  checkOverlapSphere  ///////////////////////////////////////////////////////

bool Gu::checkOverlapSphere_boxGeom(const PxGeometry& geom, const PxTransform& pose, const Gu::Sphere& sphere)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eBOX);
	const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom);

	Gu::Box obb;
	buildFrom(obb, pose.p, boxGeom.halfExtents, pose.q);
	return intersectSphereBox(sphere, obb);
}

bool Gu::checkOverlapSphere_sphereGeom(const PxGeometry& geom, const PxTransform& pose, const Gu::Sphere& sphere)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eSPHERE);
	const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom);

	return intersectSphereSphere(sphere, Gu::Sphere(pose.p, sphereGeom.radius));
}

bool Gu::checkOverlapSphere_capsuleGeom(const PxGeometry& geom, const PxTransform& pose, const Gu::Sphere& sphere)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eCAPSULE);
	const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom);

	Gu::Capsule worldCapsule;
	getWorldCapsule(worldCapsule, capsuleGeom, pose);
	return intersectSphereCapsule(sphere, worldCapsule);
}

bool Gu::checkOverlapSphere_planeGeom(const PxGeometry& geom, const PxTransform& pose, const Gu::Sphere& sphere)
{
	PX_ASSERT(geom.getType() == PxGeometryType::ePLANE);
	const PxPlaneGeometry& planeGeom = static_cast<const PxPlaneGeometry&>(geom);
	PX_UNUSED(planeGeom);

	const Gu::Plane plane = getPlane(pose);
	return plane.distance(sphere.center) - sphere.radius <= 0.0f;
}

bool Gu::checkOverlapSphere_convexGeom(const PxGeometry& geom, const PxTransform& pose, const Gu::Sphere& sphere)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eCONVEXMESH);
	const PxConvexMeshGeometry& cvGeom = static_cast<const PxConvexMeshGeometry&>(geom);

	Gu::ConvexMesh* cm = static_cast<Gu::ConvexMesh*>(cvGeom.convexMesh);

	// PT: TODO: why do we bother doing this first test?
	//TODO: Support scaling
	if (cvGeom.scale.isIdentity())
	{
		// Test if sphere center is inside convex
		PxVec3 sphereCenter = pose.transformInv(sphere.center);
		if(convexHullContains(cm->getHull(), sphereCenter))
			return true;
	}

	return intersectSphereConvex(sphere, *cm, cvGeom.scale, pose, NULL);
}

bool Gu::checkOverlapSphere_triangleGeom(const PxGeometry& geom, const PxTransform& pose, const Gu::Sphere& sphere)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eTRIANGLEMESH);
	const PxTriangleMeshGeometry& triGeom = static_cast<const PxTriangleMeshGeometry&>(geom);

	Gu::TriangleMesh* tm = static_cast<Gu::TriangleMesh*>(triGeom.triangleMesh);
	return intersectSphereMeshAny(sphere, tm->getOpcodeModel(), pose, triGeom.scale);
}

bool Gu::checkOverlapSphere_heightFieldGeom(const PxGeometry& geom, const PxTransform& pose, const Gu::Sphere& sphere)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eHEIGHTFIELD);
	const PxHeightFieldGeometry& hfGeom = static_cast<const PxHeightFieldGeometry&>(geom);

	HeightFieldUtil hfUtil(hfGeom);
	PxMat34Legacy invAbsPose(pose.getInverse());
	Gu::Sphere sphereInHfShape;
	sphereInHfShape.center = invAbsPose * sphere.center;
	sphereInHfShape.radius = sphere.radius;
	return intersectHeightFieldSphere(hfUtil, sphereInHfShape);
}

const Gu::GeomOverlapSphereFunc Gu::gGeomOverlapSphereMap[7] =
{
	checkOverlapSphere_sphereGeom,
	checkOverlapSphere_planeGeom,
	checkOverlapSphere_capsuleGeom,
	checkOverlapSphere_boxGeom,	
	checkOverlapSphere_convexGeom,
	checkOverlapSphere_triangleGeom,
	checkOverlapSphere_heightFieldGeom
};

/////////////////////////////////////////////////  checkOverlapOBB  //////////////////////////////////////////////////////////

bool Gu::checkOverlapOBB_boxGeom(const PxGeometry& geom, const PxTransform& pose, const Gu::Box& box)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eBOX);
	const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom);

	return Gu::intersectOBBOBB(	boxGeom.halfExtents, pose.p, PxMat33(pose.q),
								box.extents, box.center, box.rot,
								true);
}

bool Gu::checkOverlapOBB_sphereGeom(const PxGeometry& geom, const PxTransform& pose, const Gu::Box& box)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eSPHERE);
	const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom);

	return intersectSphereBox(Gu::Sphere(pose.p, sphereGeom.radius), box);
}

bool Gu::checkOverlapOBB_capsuleGeom(const PxGeometry& geom, const PxTransform& pose, const Gu::Box& box)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eCAPSULE);
	const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom);

	Gu::Capsule worldCapsule;
	getWorldCapsule(worldCapsule, capsuleGeom, pose);

	return intersectBoxCapsule(box, worldCapsule);
}

bool Gu::checkOverlapOBB_planeGeom(const PxGeometry& geom, const PxTransform& pose, const Gu::Box& box)
{
	PX_ASSERT(geom.getType() == PxGeometryType::ePLANE);
	const PxPlaneGeometry& planeGeom = static_cast<const PxPlaneGeometry&>(geom);
	PX_UNUSED(planeGeom);

	const Gu::Plane plane = getPlane(pose);

	return intersectPlaneBox(plane, box);
}

bool Gu::checkOverlapOBB_convexGeom(const PxGeometry& geom, const PxTransform& pose, const Gu::Box& box)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eCONVEXMESH);
	const PxConvexMeshGeometry& cvGeom = static_cast<const PxConvexMeshGeometry&>(geom);

	Gu::ConvexMesh* cm = static_cast<Gu::ConvexMesh*>(cvGeom.convexMesh);

	// PT: TODO: why do we bother doing this first test?
	//TODO: Support scaling
	if (cvGeom.scale.isIdentity())
	{
		const PxVec3 boxCenter = pose.transformInv(box.center);
		if(convexHullContains(cm->getHull(), boxCenter))
			return true;
	}

	// PT: ### USELESS CONVERSION - PxBoxGeometry & PxTransform are not necessary here
	return intersectBoxConvex(PxBoxGeometry(box.extents), PxTransform(box.center, PxQuat(box.rot)), *cm, cvGeom.scale, pose, NULL);
}

bool Gu::checkOverlapOBB_triangleGeom(const PxGeometry& geom, const PxTransform& pose, const Gu::Box& box)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eTRIANGLEMESH);
	const PxTriangleMeshGeometry& triGeom = static_cast<const PxTriangleMeshGeometry&>(geom);

	Gu::TriangleMesh* tm = static_cast<Gu::TriangleMesh*>(triGeom.triangleMesh);
	return intersectBoxMeshAny(box, tm->getOpcodeModel(), pose, triGeom.scale);
}

bool Gu::checkOverlapOBB_heightFieldGeom(const PxGeometry& geom, const PxTransform& pose, const Gu::Box& box)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eHEIGHTFIELD);
	const PxHeightFieldGeometry& hfGeom = static_cast<const PxHeightFieldGeometry&>(geom);

	const PxMat34Legacy invAbsPose(pose.getInverse());

	Gu::Box boxInHfShape;
	boxInHfShape.setRotLegacy(invAbsPose.M * PxMat33Legacy(box.rot));
	boxInHfShape.center = invAbsPose * box.center;
	boxInHfShape.extents = box.extents;

	HeightFieldUtil hfUtil(hfGeom);
	return intersectHeightFieldBox(hfUtil, boxInHfShape);
}

const Gu::GeomOverlapOBBFunc Gu::gGeomOverlapOBBMap[7] =
{
	checkOverlapOBB_sphereGeom,
	checkOverlapOBB_planeGeom,
	checkOverlapOBB_capsuleGeom,
	checkOverlapOBB_boxGeom,	
	checkOverlapOBB_convexGeom,
	checkOverlapOBB_triangleGeom,
	checkOverlapOBB_heightFieldGeom
};

/////////////////////////////////////////////////  checkOverlapCapsule  //////////////////////////////////////////////////////

bool Gu::checkOverlapCapsule_boxGeom(const PxGeometry& geom, const PxTransform& pose, const Gu::Capsule& worldCapsule)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eBOX);
	const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom);

	Gu::Box box;	buildFrom(box, pose.p, boxGeom.halfExtents, pose.q);

	return intersectBoxCapsule(box, worldCapsule);
}

bool Gu::checkOverlapCapsule_sphereGeom(const PxGeometry& geom, const PxTransform& pose, const Gu::Capsule& worldCapsule)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eSPHERE);
	const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom);

	return intersectSphereCapsule(Gu::Sphere(pose.p, sphereGeom.radius), worldCapsule);
}

bool Gu::checkOverlapCapsule_capsuleGeom(const PxGeometry& geom, const PxTransform& pose, const Gu::Capsule& worldCapsule)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eCAPSULE);
	const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom);

	Gu::Capsule thisWorldCapsule;
	getWorldCapsule(thisWorldCapsule, capsuleGeom, pose);
	PxReal s,t;
	PxReal squareDist = Gu::distanceSegmentSegmentSquared(thisWorldCapsule, worldCapsule, &s, &t);
	PxReal totRad = thisWorldCapsule.radius + worldCapsule.radius;
	return squareDist < totRad*totRad;
}

bool Gu::checkOverlapCapsule_planeGeom(const PxGeometry& geom, const PxTransform& pose, const Gu::Capsule& worldCapsule)
{
	PX_ASSERT(geom.getType() == PxGeometryType::ePLANE);
	const PxPlaneGeometry& planeGeom = static_cast<const PxPlaneGeometry&>(geom);
	PX_UNUSED(planeGeom);

	Gu::Plane plane = getPlane(pose);
	return intersectPlaneCapsule(worldCapsule, plane);
}

bool Gu::checkOverlapCapsule_convexGeom(const PxGeometry& geom, const PxTransform& pose, const Gu::Capsule& worldCapsule)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eCONVEXMESH);
	const PxConvexMeshGeometry& cvGeom = static_cast<const PxConvexMeshGeometry&>(geom);

	Gu::ConvexMesh* cm = static_cast<Gu::ConvexMesh*>(cvGeom.convexMesh);

	// PT: TODO: why do we bother doing this first test?
	//TODO: Support scaling
	if (cvGeom.scale.isIdentity())
	{
		// Test if capsule center is inside convex
		PxVec3 capsuleCenter = (worldCapsule.p0 + worldCapsule.p1) * 0.5f;
		capsuleCenter = pose.transformInv(capsuleCenter);
		if(convexHullContains(cm->getHull(),capsuleCenter))
			return true;
	}

	PxCapsuleGeometry cg;
	cg.radius = worldCapsule.radius;
	PxTransform capsuleTransform = getWorldTransform(worldCapsule, cg.halfHeight);

	return intersectCapsuleConvex(cg, capsuleTransform, *cm, cvGeom.scale, pose, NULL);
}

bool Gu::checkOverlapCapsule_triangleGeom(const PxGeometry& geom, const PxTransform& pose, const Gu::Capsule& worldCapsule)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eTRIANGLEMESH);
	const PxTriangleMeshGeometry& triGeom = static_cast<const PxTriangleMeshGeometry&>(geom);

	const Gu::TriangleMesh* tm = static_cast<Gu::TriangleMesh*>(triGeom.triangleMesh);
	return intersectCapsuleMeshAny(worldCapsule, tm->getOpcodeModel(), pose, triGeom.scale);
}

bool Gu::checkOverlapCapsule_heightFieldGeom(const PxGeometry& geom, const PxTransform& pose, const Gu::Capsule& worldCapsule)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eHEIGHTFIELD);
	const PxHeightFieldGeometry& hfGeom = static_cast<const PxHeightFieldGeometry&>(geom);

	HeightFieldUtil hfUtil(hfGeom);
	const PxMat34Legacy invAbsPose(pose.getInverse());
	Gu::Capsule capsuleInHfShape;
	capsuleInHfShape.p0 = invAbsPose * worldCapsule.p0;
	capsuleInHfShape.p1 = invAbsPose * worldCapsule.p1;
	capsuleInHfShape.radius = worldCapsule.radius;
	return intersectHeightFieldCapsule(hfUtil, capsuleInHfShape);
}

const Gu::GeomOverlapCapsuleFunc Gu::gGeomOverlapCapsuleMap[7] =
{
	checkOverlapCapsule_sphereGeom,
	checkOverlapCapsule_planeGeom,
	checkOverlapCapsule_capsuleGeom,
	checkOverlapCapsule_boxGeom,	
	checkOverlapCapsule_convexGeom,
	checkOverlapCapsule_triangleGeom,
	checkOverlapCapsule_heightFieldGeom
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// PT: TODO: isn't this the same as Gu::getPlane() in GuGeomUtilsInternal.cpp ?
static PX_INLINE Gu::Plane createPxPlane(const PxTransform& pose)
{
	return Gu::Plane(pose.p, pose.q.rotate(PxVec3(1.0f, 0.0f, 0.0f)));
}

// PT: TODO: isn't this the same as Gu::getWorldSegment() in GuGeomUtilsInternal.cpp ?
static PX_INLINE Gu::Segment createPxSegment(const PxCapsuleGeometry& geom, const PxTransform& pose)
{
	return Gu::Segment(pose.transform(PxVec3(-geom.halfHeight, 0, 0)), pose.transform(PxVec3(geom.halfHeight, 0, 0)));
}

// PT: TODO: isn't this the same as Gu::getWorldCapsule() in GuGeomUtilsInternal.cpp ?
static PX_INLINE Gu::Capsule createPxCapsule(const PxCapsuleGeometry& geom, const PxTransform& pose)
{
	return Gu::Capsule(createPxSegment(geom,pose), geom.radius);
}

static PX_FORCE_INLINE PxVec3* getCachedAxis(Gu::TriggerCache* cache)
{
	if(cache && cache->state==Gu::TRIGGER_OVERLAP)
		return &cache->dir;
	else
		return NULL;
}

static PX_FORCE_INLINE bool updateTriggerCache(bool overlap, Gu::TriggerCache* cache)
{
	if(cache)
	{
		if(overlap)
			cache->state = Gu::TRIGGER_OVERLAP;
		else
			cache->state = Gu::TRIGGER_DISJOINT;
	}
	return overlap;
}

// Sphere-vs-shape

static bool GeomOverlapCallback_SphereSphere(GEOM_OVERLAP_CALLBACK_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eSPHERE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eSPHERE);

	const PxSphereGeometry& sphereGeom0 = static_cast<const PxSphereGeometry&>(geom0);
	const PxSphereGeometry& sphereGeom1 = static_cast<const PxSphereGeometry&>(geom1);

	const PxVec3 delta = transform1.p - transform0.p;
	return delta.magnitudeSquared() < Ps::sqr(sphereGeom0.radius + sphereGeom1.radius);
}

static bool GeomOverlapCallback_SpherePlane(GEOM_OVERLAP_CALLBACK_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eSPHERE);
	PX_ASSERT(geom1.getType()==PxGeometryType::ePLANE);

	const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom0);
	const PxPlaneGeometry& planeGeom = static_cast<const PxPlaneGeometry&>(geom1);
	PX_UNUSED(planeGeom);

	return createPxPlane(transform1).distance(transform0.p) <= sphereGeom.radius;
}

static bool GeomOverlapCallback_SphereCapsule(GEOM_OVERLAP_CALLBACK_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eSPHERE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eCAPSULE);

	const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom0);
	const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom1);

	const Gu::Segment segment = createPxSegment(capsuleGeom, transform1);
	const PxReal totRad = sphereGeom.radius + capsuleGeom.radius;

	return Gu::distancePointSegmentSquared(segment, transform0.p, NULL) < totRad*totRad;
}

static bool GeomOverlapCallback_SphereBox(GEOM_OVERLAP_CALLBACK_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eSPHERE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eBOX);

	const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom0);
	const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom1);

	Gu::Box obb;
	buildFrom(obb, transform1.p, boxGeom.halfExtents, transform1.q);

	return Gu::intersectSphereBox(
		Gu::Sphere(transform0.p, sphereGeom.radius),
		obb);
}

static bool GeomOverlapCallback_SphereConvex(GEOM_OVERLAP_CALLBACK_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eSPHERE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eCONVEXMESH);

	const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom0);
	const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(geom1);

	const Gu::ConvexMesh* cm = static_cast<Gu::ConvexMesh*>(convexGeom.convexMesh);

	PxVec3 cachedSepAxis;
	PxVec3* tmp = getCachedAxis(cache);
	if(tmp)
		cachedSepAxis = *tmp;
	else
		cachedSepAxis = PxVec3(0,0,1);

	const bool overlap = Gu::intersectSphereConvex(Gu::Sphere(transform0.p, sphereGeom.radius), 
		*cm,
		convexGeom.scale, transform1,
		&cachedSepAxis);

	if(cache && overlap)
		cache->dir = cachedSepAxis;

	return updateTriggerCache(overlap, cache);
}

static bool GeomOverlapCallback_SphereMesh(GEOM_OVERLAP_CALLBACK_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eSPHERE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eTRIANGLEMESH);

	const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom0);
	const PxTriangleMeshGeometry& meshGeom = static_cast<const PxTriangleMeshGeometry&>(geom1);

	const Gu::Sphere worldSphere(transform0.p, sphereGeom.radius);

	const Gu::TriangleMesh* tm = static_cast<Gu::TriangleMesh*>(meshGeom.triangleMesh);

	return Gu::intersectSphereMeshAny(worldSphere, 
		tm->getOpcodeModel(), 
		transform1,
		meshGeom.scale);
}

static bool GeomOverlapCallback_SphereHeightfield(GEOM_OVERLAP_CALLBACK_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eSPHERE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eHEIGHTFIELD);

	const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom0);
	const PxHeightFieldGeometry& hfGeom = static_cast<const PxHeightFieldGeometry&>(geom1);

	Gu::Sphere sphereInHf;
	sphereInHf.center = transform1.transformInv(transform0.p);
	sphereInHf.radius = sphereGeom.radius;

	Gu::HeightFieldUtil hfUtil(hfGeom);
	return Gu::intersectHeightFieldSphere(hfUtil, sphereInHf);
}

// Plane-vs-shape

static bool GeomOverlapCallback_PlanePlane(GEOM_OVERLAP_CALLBACK_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::ePLANE);
	PX_ASSERT(geom1.getType()==PxGeometryType::ePLANE);
	PX_ASSERT(!"NOT SUPPORTED");
	return false;
}

static bool GeomOverlapCallback_PlaneCapsule(GEOM_OVERLAP_CALLBACK_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::ePLANE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eCAPSULE);

	const PxPlaneGeometry& planeGeom = static_cast<const PxPlaneGeometry&>(geom0);
	PX_UNUSED(planeGeom);
	const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom1);

	return intersectPlaneCapsule(createPxCapsule(capsuleGeom, transform1), createPxPlane(transform0));
}

static bool GeomOverlapCallback_PlaneBox(GEOM_OVERLAP_CALLBACK_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::ePLANE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eBOX);

	const PxPlaneGeometry& planeGeom = static_cast<const PxPlaneGeometry&>(geom0);
	PX_UNUSED(planeGeom);
	const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom1);

	// I currently use the same code as for contact generation but maybe we could do something faster (in theory testing
	// only 2 pts is enough).

	const PxMat34Legacy absPose(transform1);
	const Gu::Plane worldPlane = createPxPlane(transform0);

	for (int vx=-1; vx<=1; vx+=2)
		for (int vy=-1; vy<=1; vy+=2)
			for (int vz=-1; vz<=1; vz+=2)
			{
				const PxVec3 v = absPose.t + absPose.M * PxVec3(PxReal(vx),PxReal(vy),PxReal(vz)).multiply(boxGeom.halfExtents);

				if(worldPlane.distance(v) <= 0.0f)
					return true;
			}
	return false;
}

static bool GeomOverlapCallback_PlaneConvex(GEOM_OVERLAP_CALLBACK_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::ePLANE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eCONVEXMESH);

	const PxPlaneGeometry& planeGeom = static_cast<const PxPlaneGeometry&>(geom0);
	PX_UNUSED(planeGeom);
	const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(geom1);

	const Gu::ConvexMesh* cm = static_cast<const Gu::ConvexMesh*>(convexGeom.convexMesh);

	//find plane normal in shape space of convex:
	const PxTransform plane2convex = transform1.getInverse().transform(transform0);

	const Gu::Plane shapeSpacePlane = createPxPlane(plane2convex);

	PxReal minimum, maximum;
	Gu::projectHull_(cm->getHull(), minimum, maximum, shapeSpacePlane.normal, convexGeom.scale.toMat33());

	return (maximum > shapeSpacePlane.d);
}

static bool GeomOverlapCallback_PlaneMesh(GEOM_OVERLAP_CALLBACK_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::ePLANE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eTRIANGLEMESH);
	PX_ASSERT(!"NOT SUPPORTED");
	return false;
}

static bool GeomOverlapCallback_PlaneHeightfield(GEOM_OVERLAP_CALLBACK_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::ePLANE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eHEIGHTFIELD);
	PX_ASSERT(!"NOT SUPPORTED");
	return false;
}

// Capsule-vs-shape

static bool GeomOverlapCallback_CapsuleCapsule(GEOM_OVERLAP_CALLBACK_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eCAPSULE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eCAPSULE);

	const PxCapsuleGeometry& capsuleGeom0 = static_cast<const PxCapsuleGeometry&>(geom0);
	const PxCapsuleGeometry& capsuleGeom1 = static_cast<const PxCapsuleGeometry&>(geom1);

	const Gu::Segment segment1 = createPxSegment(capsuleGeom0, transform0);
	const Gu::Segment segment2 = createPxSegment(capsuleGeom1, transform1);

	const PxReal squareDist = Gu::distanceSegmentSegmentSquared(segment1, segment2);
	return squareDist < Ps::sqr(capsuleGeom0.radius + capsuleGeom1.radius);
}

static bool GeomOverlapCallback_CapsuleBox(GEOM_OVERLAP_CALLBACK_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eCAPSULE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eBOX);

	const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom0);
	const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom1);

	const Gu::Segment segment = createPxSegment(capsuleGeom, transform0);

	// PT: it is more efficient to convert to a matrix only once here, rather than 3 times in the code below...
	Gu::Box obb;
	buildFrom(obb, transform1.p, boxGeom.halfExtents, transform1.q);

	// Collision detection
	//  ### is this even useful here ?
	if(Gu::intersectSphereBox(Gu::Sphere(segment.p0, capsuleGeom.radius), obb))	return true;
	if(Gu::intersectSphereBox(Gu::Sphere(segment.p1, capsuleGeom.radius), obb))	return true;
	// 
	return Gu::distanceSegmentBoxSquared(segment.p0, segment.p1, transform1.p, boxGeom.halfExtents, obb.rot) < Ps::sqr(capsuleGeom.radius);
}

static bool GeomOverlapCallback_CapsuleConvex(GEOM_OVERLAP_CALLBACK_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eCAPSULE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eCONVEXMESH);

	const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom0);
	const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(geom1);

	const Gu::ConvexMesh* cm = static_cast<const Gu::ConvexMesh*>(convexGeom.convexMesh);

	PxVec3 cachedSepAxis;
	PxVec3* tmp = getCachedAxis(cache);
	if(tmp)
		cachedSepAxis = *tmp;
	else
		cachedSepAxis = PxVec3(0,0,1);

	const bool overlap = Gu::intersectCapsuleConvex(capsuleGeom, transform0,
		*cm,
		convexGeom.scale, transform1,
		 &cachedSepAxis);

	if(cache && overlap)
		cache->dir = cachedSepAxis;

	return updateTriggerCache(overlap, cache);
}

static bool GeomOverlapCallback_CapsuleMesh(GEOM_OVERLAP_CALLBACK_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eCAPSULE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eTRIANGLEMESH);

	const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom0);
	const PxTriangleMeshGeometry& meshGeom = static_cast<const PxTriangleMeshGeometry&>(geom1);

	const Gu::TriangleMesh* tm = static_cast<Gu::TriangleMesh*>(meshGeom.triangleMesh);

	return Gu::intersectCapsuleMeshAny(createPxCapsule(capsuleGeom, transform0),
		tm->getOpcodeModel(), 
		transform1,
		meshGeom.scale);
}

static bool GeomOverlapCallback_CapsuleHeightfield(GEOM_OVERLAP_CALLBACK_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eCAPSULE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eHEIGHTFIELD);

	const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom0);
	const PxHeightFieldGeometry& hfGeom = static_cast<const PxHeightFieldGeometry&>(geom1);

	const PxTransform capsuleShapeToHfShape = transform1.transformInv(transform0);
	const Gu::Capsule capsuleInHfShape = createPxCapsule(capsuleGeom, capsuleShapeToHfShape);

	const Gu::HeightFieldUtil hfUtil(hfGeom);
	return Gu::intersectHeightFieldCapsule(hfUtil, capsuleInHfShape);
}

// Box-vs-shape

static bool GeomOverlapCallback_BoxBox(GEOM_OVERLAP_CALLBACK_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eBOX);
	PX_ASSERT(geom1.getType()==PxGeometryType::eBOX);

	const PxBoxGeometry& boxGeom0 = static_cast<const PxBoxGeometry&>(geom0);
	const PxBoxGeometry& boxGeom1 = static_cast<const PxBoxGeometry&>(geom1);

	return Gu::intersectOBBOBB(	boxGeom0.halfExtents, transform0.p, PxMat33(transform0.q), 
								boxGeom1.halfExtents, transform1.p, PxMat33(transform1.q), true);
}

static bool GeomOverlapCallback_BoxConvex(GEOM_OVERLAP_CALLBACK_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eBOX);
	PX_ASSERT(geom1.getType()==PxGeometryType::eCONVEXMESH);

	const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom0);
	const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(geom1);

	const Gu::ConvexMesh* cm = static_cast<Gu::ConvexMesh*>(convexGeom.convexMesh);

	PxVec3 cachedSepAxis;
	PxVec3* tmp = getCachedAxis(cache);
	if(tmp)
		cachedSepAxis = *tmp;
	else
		cachedSepAxis = PxVec3(0,0,1);

	const bool overlap = Gu::intersectBoxConvex(boxGeom, transform0,
		*cm,
		convexGeom.scale, transform1,
		 &cachedSepAxis);

	if(cache && overlap)
		cache->dir = cachedSepAxis;

	return updateTriggerCache(overlap, cache);
}

static bool GeomOverlapCallback_BoxMesh(GEOM_OVERLAP_CALLBACK_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eBOX);
	PX_ASSERT(geom1.getType()==PxGeometryType::eTRIANGLEMESH);

	const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom0);
	const PxTriangleMeshGeometry& meshGeom = static_cast<const PxTriangleMeshGeometry&>(geom1);

	const Gu::TriangleMesh* tm = static_cast<Gu::TriangleMesh*>(meshGeom.triangleMesh);

	Gu::Box box;
	buildFrom(box, transform0.p, boxGeom.halfExtents, transform0.q);

	return Gu::intersectBoxMeshAny(
		box,
		tm->getOpcodeModel(), 
		transform1,
		meshGeom.scale);
}

static bool GeomOverlapCallback_BoxHeightfield(GEOM_OVERLAP_CALLBACK_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eBOX);
	PX_ASSERT(geom1.getType()==PxGeometryType::eHEIGHTFIELD);

	const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom0);
	const PxHeightFieldGeometry& hfGeom = static_cast<const PxHeightFieldGeometry&>(geom1);

	const PxTransform boxShape2HfShape = transform1.transformInv(transform0);

	Gu::Box box;
	buildFrom(box, boxShape2HfShape.p, boxGeom.halfExtents, boxShape2HfShape.q);

	Gu::HeightFieldUtil hfUtil(hfGeom);
	return Gu::intersectHeightFieldBox(hfUtil, box);
}

// Convex-vs-shape

static bool GeomOverlapCallback_ConvexConvex(GEOM_OVERLAP_CALLBACK_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eCONVEXMESH);
	PX_ASSERT(geom1.getType()==PxGeometryType::eCONVEXMESH);

	const PxConvexMeshGeometry& convexGeom0 = static_cast<const PxConvexMeshGeometry&>(geom0);
	const PxConvexMeshGeometry& convexGeom1 = static_cast<const PxConvexMeshGeometry&>(geom1);

	const Gu::ConvexMesh* cm0 = static_cast<const Gu::ConvexMesh*>(convexGeom0.convexMesh);
	const Gu::ConvexMesh* cm1 = static_cast<const Gu::ConvexMesh*>(convexGeom1.convexMesh);

	GJKConvexInterfaceCache gjkCache;
	Gu::GJKConvexSupport convexSupport0(*cm0, convexGeom0.scale);
	Gu::GJKConvexSupport convexSupport1(*cm1, convexGeom1.scale);

	PxVec3 cachedSepAxis;
	PxVec3* tmp = getCachedAxis(cache);
	if(tmp)
		cachedSepAxis = *tmp;
	else
		cachedSepAxis = PxVec3(0,0,1);

	PxVec3 destWorldNormalOnB;
	PxVec3 destWorldPointA;
	PxVec3 destWorldPointB;
	PxReal destDistance;

	const bool overlap = !convexConvexDistance(convexSupport0, convexSupport1, transform0, transform1, cachedSepAxis,
		destWorldNormalOnB, destWorldPointA, destWorldPointB, destDistance,	gjkCache);

	if(cache && overlap)
		cache->dir = cachedSepAxis;

	return updateTriggerCache(overlap, cache);
}

///////////////////////////////////////////////////////////////////////////////

	struct ConvexVsMeshOverlapCallback : VolumeColliderTrigCallback
	{
		ConvexVsMeshOverlapCallback(const Gu::ConvexMesh& cm, const PxMeshScale& convexScale, const Cm::FastVertex2ShapeScaling& meshScale, const PxTransform& tr0, const PxTransform& tr1) :
				mConvexSupport	(cm, convexScale),
				mMeshScale		(meshScale),
				mTransform0		(tr0),
				mTransform1		(tr1),
				mAnyHit			(false)			{}
		virtual ~ConvexVsMeshOverlapCallback()	{}

		virtual bool processResults(PxU32 count, const PxVec3* verts, const PxU32*)
		{
			GJKConvexInterfaceCache gjkCache;

			while(count--)
			{
				const PxVec3 v0 = mMeshScale * verts[0];
				const PxVec3 v1 = mMeshScale * verts[1];
				const PxVec3 v2 = mMeshScale * verts[2];
				verts += 3;

				GJKTriangleSupport triangleSupport(v0, v1, v2);

				PxVec3 cachedSepAxis(0,0,1);

				PxVec3 destWorldNormalOnB;
				PxVec3 destWorldPointA;
				PxVec3 destWorldPointB;
				PxReal destDistance;

				const bool overlap = !convexConvexDistance(mConvexSupport, triangleSupport, mTransform0, mTransform1, cachedSepAxis,
					destWorldNormalOnB, destWorldPointA, destWorldPointB, destDistance,	gjkCache);

				if(overlap)
				{
					mAnyHit = true;
					return false;
				}
			}
			return true;
		}
		Gu::GJKConvexSupport				mConvexSupport;
		const Cm::FastVertex2ShapeScaling&	mMeshScale;
		const PxTransform&					mTransform0;
		const PxTransform&					mTransform1;
		bool								mAnyHit;
	};

// PT: TODO: refactor bits of this with convex-vs-mesh code
static bool GeomOverlapCallback_ConvexMesh(GEOM_OVERLAP_CALLBACK_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eCONVEXMESH);
	PX_ASSERT(geom1.getType()==PxGeometryType::eTRIANGLEMESH);

	const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(geom0);
	const PxTriangleMeshGeometry& meshGeom = static_cast<const PxTriangleMeshGeometry&>(geom1);

	const Gu::ConvexMesh* cm = static_cast<const Gu::ConvexMesh*>(convexGeom.convexMesh);
	const Gu::TriangleMesh* tm = static_cast<Gu::TriangleMesh*>(meshGeom.triangleMesh);

	const bool idtScaleConvex = convexGeom.scale.isIdentity();
	const bool idtScaleMesh = meshGeom.scale.isIdentity();

	Cm::FastVertex2ShapeScaling convexScaling;
	if(!idtScaleConvex)
		convexScaling.init(convexGeom.scale);

	Cm::FastVertex2ShapeScaling meshScaling;
	if(!idtScaleMesh)
		meshScaling.init(meshGeom.scale);

	const Cm::Matrix34 world0(transform0);
	const Cm::Matrix34 world1(transform1);

	PxBounds3 hullAABB;
	Gu::transformNoEmptyTest(hullAABB, convexScaling.getVertex2ShapeSkew(), cm->getLocalBounds());

	Gu::Box hullOBB;
	computeHullOBB(hullOBB, hullAABB, 0.0f, transform0, world0, world1, meshScaling, idtScaleMesh);

	Ice::HybridModelData hmd;	// PT: I suppose doing the "conversion" at runtime is fine
	tm->mesh.mData.mOpcodeModel.getHybridModelData(hmd);

	Ice::HybridOBBCollider collider;
	collider.SetFullBoxBoxTest(false);	// PT: usually faster
	collider.SetPrimitiveTests(true);
	collider.SetLoosePrimitiveTests(false);

	ConvexVsMeshOverlapCallback cb(*cm, convexGeom.scale, meshScaling, transform0, transform1);
	collider.Collide(hullOBB, hmd, &cb, NULL, NULL, true);
	return cb.mAnyHit;
}

///////////////////////////////////////////////////////////////////////////////

static bool GeomOverlapCallback_ConvexHeightfield(GEOM_OVERLAP_CALLBACK_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eCONVEXMESH);
	PX_ASSERT(geom1.getType()==PxGeometryType::eHEIGHTFIELD);

// ptchernev TODO: 
// Check with Adam | Pierre about the SUPPORT_CONVEX_SCALE stuff.
// It is in here but has not been tested.

	const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(geom0);
	const PxHeightFieldGeometry& hfGeom = static_cast<const PxHeightFieldGeometry&>(geom1);

	Gu::ConvexMesh* cm = static_cast<Gu::ConvexMesh*>(convexGeom.convexMesh);

	Gu::HeightFieldUtil hfUtil(hfGeom);
	return Gu::intersectHeightFieldConvex(hfUtil,
		PxMat34Legacy(transform1),
		*cm,
		PxMat34Legacy(transform0),
		convexGeom.scale);
}

// Mesh-vs-shape

static bool GeomOverlapCallback_MeshMesh(GEOM_OVERLAP_CALLBACK_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eTRIANGLEMESH);
	PX_ASSERT(geom1.getType()==PxGeometryType::eTRIANGLEMESH);
	PX_ASSERT(!"NOT SUPPORTED");
	return false;
}

static bool GeomOverlapCallback_MeshHeightfield(GEOM_OVERLAP_CALLBACK_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eTRIANGLEMESH);
	PX_ASSERT(geom1.getType()==PxGeometryType::eHEIGHTFIELD);
	PX_ASSERT(!"NOT SUPPORTED");
	return false;
}

// Heightfield-vs-shape

static bool GeomOverlapCallback_HeightfieldHeightfield(GEOM_OVERLAP_CALLBACK_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eHEIGHTFIELD);
	PX_ASSERT(geom1.getType()==PxGeometryType::eHEIGHTFIELD);
	PX_ASSERT(!"NOT SUPPORTED");
	return false;
}

const Gu::GeomOverlapFunc Gu::gGeomOverlapMethodTable[][7] = 
{
	//PxGeometryType::eSPHERE
	{
		GeomOverlapCallback_SphereSphere,		//PxGeometryType::eSPHERE
		GeomOverlapCallback_SpherePlane,		//PxGeometryType::ePLANE
		GeomOverlapCallback_SphereCapsule,		//PxGeometryType::eCAPSULE
		GeomOverlapCallback_SphereBox,			//PxGeometryType::eBOX
		GeomOverlapCallback_SphereConvex,		//PxGeometryType::eCONVEXMESH
		GeomOverlapCallback_SphereMesh,			//PxGeometryType::eTRIANGLEMESH
		GeomOverlapCallback_SphereHeightfield,	//PxGeometryType::eHEIGHTFIELD
		
	},

	//PxGeometryType::ePLANE
	{
		0,										//PxGeometryType::eSPHERE
		GeomOverlapCallback_PlanePlane,			//PxGeometryType::ePLANE
		GeomOverlapCallback_PlaneCapsule,		//PxGeometryType::eCAPSULE
		GeomOverlapCallback_PlaneBox,			//PxGeometryType::eBOX
		GeomOverlapCallback_PlaneConvex,		//PxGeometryType::eCONVEXMESH
		GeomOverlapCallback_PlaneMesh,			//PxGeometryType::eTRIANGLEMESH
		GeomOverlapCallback_PlaneHeightfield,	//PxGeometryType::eHEIGHTFIELD
	},

	//PxGeometryType::eCAPSULE
	{
		0,										//PxGeometryType::eSPHERE
		0,										//PxGeometryType::ePLANE
		GeomOverlapCallback_CapsuleCapsule,		//PxGeometryType::eCAPSULE
		GeomOverlapCallback_CapsuleBox,			//PxGeometryType::eBOX
		GeomOverlapCallback_CapsuleConvex,		//PxGeometryType::eCONVEXMESH
		GeomOverlapCallback_CapsuleMesh,		//PxGeometryType::eTRIANGLEMESH
		GeomOverlapCallback_CapsuleHeightfield,	//PxGeometryType::eHEIGHTFIELD
	},

	//PxGeometryType::eBOX
	{
		0,										//PxGeometryType::eSPHERE
		0,										//PxGeometryType::ePLANE
		0,										//PxGeometryType::eCAPSULE
		GeomOverlapCallback_BoxBox,				//PxGeometryType::eBOX
		GeomOverlapCallback_BoxConvex,			//PxGeometryType::eCONVEXMESH
		GeomOverlapCallback_BoxMesh,			//PxGeometryType::eTRIANGLEMESH
		GeomOverlapCallback_BoxHeightfield,		//PxGeometryType::eHEIGHTFIELD
	},

	//PxGeometryType::eCONVEXMESH
	{
		0,										//PxGeometryType::eSPHERE
		0,										//PxGeometryType::ePLANE
		0,										//PxGeometryType::eCAPSULE
		0,										//PxGeometryType::eBOX
		GeomOverlapCallback_ConvexConvex,		//PxGeometryType::eCONVEXMESH
		GeomOverlapCallback_ConvexMesh,			//PxGeometryType::eTRIANGLEMESH		//not used: mesh always uses swept method for midphase.
		GeomOverlapCallback_ConvexHeightfield,	//PxGeometryType::eHEIGHTFIELD		//TODO: make HF midphase that will mask this
	},

	//PxGeometryType::eTRIANGLEMESH
	{
		0,										//PxGeometryType::eSPHERE
		0,										//PxGeometryType::ePLANE
		0,										//PxGeometryType::eCAPSULE
		0,										//PxGeometryType::eBOX
		0,										//PxGeometryType::eCONVEXMESH
		GeomOverlapCallback_MeshMesh,			//PxGeometryType::eTRIANGLEMESH
		GeomOverlapCallback_MeshHeightfield,	//PxGeometryType::eHEIGHTFIELD
	},

	//PxGeometryType::eHEIGHTFIELD
	{
		0,											//PxGeometryType::eSPHERE
		0,											//PxGeometryType::ePLANE
		0,											//PxGeometryType::eCAPSULE
		0,											//PxGeometryType::eBOX
		0,											//PxGeometryType::eCONVEXMESH
		0,											//PxGeometryType::eTRIANGLEMESH
		GeomOverlapCallback_HeightfieldHeightfield,	//PxGeometryType::eHEIGHTFIELD
	},
};


// PT: this used to be in ScContactBoxConvex.cpp. I think it is still more
// efficient than a vanilla GJK call so we should revisit this at some point.

#if 0	//TODO: this may be more efficient on your platform!  Please check with Adam before removing!
		Cm::FastVertex2ShapeScaling vertex2ShapeSkew(convexShape.getScaling());

		Gu::Box worldOBB;
		boxShape.getWorldOBB(worldOBB);

		const Gu::ConvexMesh& cm = convexShape.getConvexMeshFast();
		const PxMat34Legacy& convexShape2world = convexShape.getSim()->getAbsPose();

		// Move box center to mesh space
		PxVec3 boxCenterMeshSpace;
		convexShape2world.multiplyByInverseRT(worldOBB.center, boxCenterMeshSpace);

		if(cache.state==TRIGGER_DISJOINT)
		{
			const CollisionHull* hull = cm.getHull();

			//Matrix4x4 hullWorldMatrix;
			//PxToICE(hullWorldMatrix, convexShape2world);

			// Compute witness vector (world space)
			//Point worldWitness = hull->GetCenter() * hullWorldMatrix;
			PxVec3 worldWitness = convexShape2world * (vertex2ShapeSkew * *hull->mData.getCenter());

			//worldWitness -= (const Point&)worldOBB.center;
			worldWitness -= worldOBB.center;
			worldWitness.normalize();

			// Test witness as a separating axis

			// Project hull
			float Min0,Max0;
			ProjectHull(cm, Min0, Max0, worldWitness, convexShape2world, vertex2ShapeSkew);

			// Project box
			float Min1, Max1;
			{
				//			OBB iceBox;
				//			PxToICE(iceBox, worldOBB.extents, &worldOBB.center, &worldOBB.rot);

				PxReal BoxCen = worldWitness|worldOBB.center;
				PxReal BoxExt =	fabsf(worldOBB.rot.getColumn(0)|worldWitness) * worldOBB.extents.x
					+	fabsf(worldOBB.rot.getColumn(1)|worldWitness) * worldOBB.extents.y
					+	fabsf(worldOBB.rot.getColumn(2)|worldWitness) * worldOBB.extents.z;

				Min1 = BoxCen - BoxExt; 
				Max1 = BoxCen + BoxExt; 
			}

			// Test overlap
			if(Max0<Min1 || Max1<Min0)
			{
				// Still separated. Keep same state.
				//printf("Early exit - disjoint\n");
				return false;
			}
		}
		else if(cache.state==TRIGGER_OVERLAP)
		{

			const IndexedTriangle& T = (IndexedTriangle&)cm.getTriangle(cache.index);
			PxVec3 p0 = cm.getInternalVertices()[T.mRef[0]];
			PxVec3 p1 = cm.getInternalVertices()[T.mRef[1]];
			PxVec3 p2 = cm.getInternalVertices()[T.mRef[2]];



			// Box matrix
			PxMat34Legacy boxMatrix(false);
			boxMatrix.t = worldOBB.center;
			boxMatrix.M = worldOBB.rot;

			//build transform matrix from triangle to box space:
			//vertex2box = world2box * shape2world * vertex2shape
			PxMat34Legacy vertex2BoxSkew(false);
			//PxMat34Legacy vertex2BoxSkew = inv(boxMatrix) * convexShape2world * vertex2ShapeSkew.getVertex2ShapeSkew()
			vertex2BoxSkew.t = convexShape2world.t;
			vertex2BoxSkew.M = convexShape2world.M * vertex2ShapeSkew.getVertex2ShapeSkew();
			vertex2BoxSkew.multiplyInverseRTLeft(boxMatrix, vertex2BoxSkew);


			vertex2BoxSkew.multiply(p0,p0);
			vertex2BoxSkew.multiply(p1,p1);
			vertex2BoxSkew.multiply(p2,p2);

			// Compute tri-box overlap
			PxVec3 zero(0,0,0);
			if(Ps::triBoxOverlap(&zero, &worldOBB.extents, &p0, &p1, &p2))
			{
				// Still overlapping. Keep same state.
				//printf("Early exit - overlap\n");
				return true;
			}
		}
		else
		{
			//
			PxVec3 p (cache.px, cache.py, cache.pz);
			Gu::Sphere cachedVolume(p, PX_FR(cache.index));
			if(cachedVolume.Contains(worldOBB.center))
			{
				// Still overlapping. Keep same state.
				//printf("Early exit - inside\n");
				return true;
			}
		}

		// Brute force version...
		udword NbPolygons = cm.getHull()->GetNbPolygons();
		PxF32 maxD = -PX_MAX_F32;
		PxU32 index=0xffffffff;
		for(PxU32 ii=0;ii<NbPolygons;ii++)
		{
			PxU32 i=ii;
			if(cache.state == TRIGGER_DISJOINT)
			{
				if(i==0)			i=cache.index;
				else	if(i==cache.index)	i=0;
			}
			const Gu::HullPolygonData& poly = cm.getHull()->GetPolygon(i);

			Gu::Plane& vertexSpacePlane = ((Gu::Plane&)poly.getPlane()[0]);
			Gu::Plane shapeSpacePlane;			
			vertex2ShapeSkew.transformPlaneToShapeSpace(vertexSpacePlane.n,vertexSpacePlane,shapeSpacePlane,n,shapeSpacePlane.d);		//TODO: could optimize by deferring transform.


			PxF32 d = shapeSpacePlane.distance(boxCenterMeshSpace);
			if(d>maxD)
			{
				maxD = d;
			}
			if(d > 0.0f)
			{
				// Outside => switch to distance
				index = i;
				break;
			}
		}

		if(maxD<0.0f)
		{
			// Sphere center is inside convex (behind all face planes)
			cache.state = TRIGGER_INSIDE;

			// The center must travel at least "maxD" to exit the hull. So as long as the moving center is inside
			// the (currentCenter, -maxD) sphere, we know we are still inside the convex.
			cache.px = worldOBB.center.x;
			cache.py = worldOBB.center.y;
			cache.pz = worldOBB.center.z;
			maxD = -maxD;
			cache.index = PX_IR(maxD);
			return true;
		}

		//////////////////

		//drop opcode support for convexes:
		//NEW, OPCODE-less version
		// Box matrix
		PxMat34Legacy boxMatrix(false);
		boxMatrix.t = worldOBB.center;
		boxMatrix.M = worldOBB.rot;

		//build transform matrix from triangle to box space:
		//vertex2box = world2box * shape2world * vertex2shape
		PxMat34Legacy vertex2BoxSkew(false);
		//PxMat34Legacy vertex2BoxSkew = inv(boxMatrix) * convexShape2world * vertex2ShapeSkew.getVertex2ShapeSkew()
		vertex2BoxSkew.t = convexShape2world.t;
		vertex2BoxSkew.M = convexShape2world.M * vertex2ShapeSkew.getVertex2ShapeSkew();
		vertex2BoxSkew.multiplyInverseRTLeft(boxMatrix, vertex2BoxSkew);

		PxVec3 zero(0,0,0);

		//iterate the triangles
		for (PxU32 i=0;i<cm.getNumTriangles();i++)
		{

			const IndexedTriangle& T = (IndexedTriangle&)cm.getTriangle(i);
			const PxVec3& lp0 = cm.getInternalVertices()[T.mRef[0]];
			const PxVec3& lp1 = cm.getInternalVertices()[T.mRef[1]];
			const PxVec3& lp2 = cm.getInternalVertices()[T.mRef[2]];

			const PxVec3 p0 = vertex2BoxSkew * lp0;
			const PxVec3 p1 = vertex2BoxSkew * lp1;
			const PxVec3 p2 = vertex2BoxSkew * lp2;

			// Compute tri-box overlap
			if(Ps::triBoxOverlap(&zero, &worldOBB.extents, &p0, &p1, &p2))
			{
				cache.index = i;
				cache.state = TRIGGER_OVERLAP;
				return true;	// Box and mesh overlap
			}
		}

		// Disjoint
		cache.state = TRIGGER_DISJOINT;
		cache.index = index != 0xffffffff ? index : 0;
		//### cache separating axis?

		return false;
#endif

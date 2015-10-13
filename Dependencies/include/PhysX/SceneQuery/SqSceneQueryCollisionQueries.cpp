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


#include "SqSceneQueryManager.h"
#include "SqFiltering.h"
#include "GuGeomUtilsInternal.h"  // for "computeBoxAroundCapsule"
#include "GuBoxConversion.h"
#include "SqSweepCache.h"
#include "GuSweepTests.h"
#include "GuOverlapTests.h"

using namespace physx;
using namespace Ice;
using namespace Sq;
using namespace Gu;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace physx
{
namespace Sq
{
	enum SweepType
	{
		SWEPT_BOX,
		SWEPT_CAPSULE,

		SWEPT_UNDEFINED
	};

	class SweptVolume
	{
	public:
		PX_FORCE_INLINE	SweptVolume()	: mType(SWEPT_UNDEFINED)	{}
						~SweptVolume()	{}

		SweepType		mType;
	};

	class SweptBox : public SweptVolume
	{
	public:
		PX_FORCE_INLINE	SweptBox()	{ mType = SWEPT_BOX;	}
						~SweptBox()	{}

		Gu::Box			mBox;
	};

	class SweptCapsule : public SweptVolume
	{
	public:
		PX_FORCE_INLINE	SweptCapsule()	{ mType = SWEPT_CAPSULE;	}
						~SweptCapsule()	{}

		Gu::Capsule		mCapsule;
	};
}

}

PX_FORCE_INLINE bool sweepVolumeAgainstShape(
	const SweptVolume& volume,
	const PxVec3& unitDir,
	float distance,
	const SceneQueryShapeData* currentShape,
	PxSweepHit& hit,
	PxSceneQueryFlags hintFlags)
{
	const PxGeometry& geom = currentShape->shapeGeometryEA->get();
	const PxTransform& tr = currentShape->sqGlobalPose;

	if(volume.mType==SWEPT_BOX)
	{
		const Sq::SweptBox& sweptBox = static_cast<const Sq::SweptBox&>(volume);
		const SweepBoxFunc sf = gSweepBoxMap[currentShape->getGeometryType()];
		return (sf)(geom, tr, sweptBox.mBox, unitDir, distance, hit, hintFlags);
	}
	else
	{
		PX_ASSERT(volume.mType==SWEPT_CAPSULE);

		const Sq::SweptCapsule& sweptCapsule = static_cast<const Sq::SweptCapsule&>(volume);
		const SweepCapsuleFunc sf = gSweepCapsuleMap[currentShape->getGeometryType()];
		return (sf)(geom, tr, sweptCapsule.mCapsule, unitDir, distance, hit, hintFlags);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct SqFilteringParams
{
	void initFiltering(	PxU32 filterFlags, PxSceneQueryFilterCallback* filterCall, const PxFilterData* filterData,
						PxBatchQueryPreFilterShader preFilterShader, PxBatchQueryPostFilterShader postFilterShader, const void* constBlock, PxU32 cBlockSize, 
						PxClientID queryClient, Scb::Scene& scene)
	{
		mFilterFlags		= filterFlags;
		mFilterCall			= filterCall;
		mFilterData			= filterData;
		mPreFilterShader	= preFilterShader;
		mPostFilterShader	= postFilterShader;
		mConstBlock			= constBlock;
		mcBlockSize			= cBlockSize;
		mQueryClient		= queryClient;
		mPassForeignShapes	= (scene.getClientBehaviorBits(queryClient) & PxClientBehaviorBit::eREPORT_FOREIGN_OBJECTS_TO_SCENE_QUERY) != 0;
	}

	PX_FORCE_INLINE	void applyPostFilter(bool isBatchQuery, const PxSceneQueryHit& hit, const SceneQueryShapeData* currentShape, PxSceneQueryFilterFlags modFilterFlags, PxSceneQueryHitType::Enum& hitType) const
	{
		if(isBatchQuery)
			hitType = applyBatchedPostFilterTest(mPostFilterShader, *mFilterData, mConstBlock, mcBlockSize, modFilterFlags, *currentShape, hit, hitType);
		else
			hitType = applyNonBatchedFilterPostTest(mFilterCall, modFilterFlags, *mFilterData, hit, hitType);
	}

	PxU32							mFilterFlags;
	PxSceneQueryFilterCallback*		mFilterCall;
	const PxFilterData*				mFilterData;
	PxBatchQueryPreFilterShader		mPreFilterShader;
	PxBatchQueryPostFilterShader	mPostFilterShader;
	const void*						mConstBlock;
	PxU32							mcBlockSize;
	PxClientID						mQueryClient;
	bool							mPassForeignShapes;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct VolumeReportPrunablesCallbackParams : SqFilteringParams
{
	// In
	PxU32							mMaxShapes;
	PxShape**						mBuffer;
	bool							mIsBatchQuery;
	bool							mMultipleOverlaps;
	// Out
	PxU32							mNbHits;
	bool							mBufferOverflow;

	PX_FORCE_INLINE	bool			preOverlapTestFilteringCode(SceneQueryShapeData* currentShape, PxSceneQueryFilterFlags& outFilterFlags, PxSceneQueryHitType::Enum& outHitType);
	PX_FORCE_INLINE	bool			registerHit(SceneQueryShapeData* currentShape, PxSceneQueryHitType::Enum& hitType, PxSceneQueryFilterFlags& modFilterFlags);

	void	initVolumeReport(Scb::Scene& scene,
				bool multipleOverlaps, PxU32 nbShapes, PxShape** objects,
				PxU32 filterFlags, PxSceneQueryFilterCallback* filterCall, const PxFilterData* filterData,
				PxBatchQueryPreFilterShader preFilterShader, PxBatchQueryPostFilterShader postFilterShader,
				const void* constBlock, PxU32 cBlockSize, PxClientID queryClient, bool isBatchQuery)
	{
		initFiltering(filterFlags, filterCall, filterData, preFilterShader, postFilterShader, constBlock, cBlockSize, queryClient, scene);

		mMaxShapes			= nbShapes;
		mBuffer				= objects;
		mIsBatchQuery		= isBatchQuery;
		mMultipleOverlaps	= multipleOverlaps;
		mNbHits				= 0;
		mBufferOverflow		= false;
	}
};

struct SphereReportPrunablesCallbackParams : VolumeReportPrunablesCallbackParams
{
	// In
	const Sphere*					mWorldSphere;
};

struct AabbReportPrunablesCallbackParams : VolumeReportPrunablesCallbackParams
{
	// In
	const PxBounds3*				mWorldBounds;
};

struct ObbReportPrunablesCallbackParams : VolumeReportPrunablesCallbackParams
{
	// In
	const Box*						mWorldBox;
};

struct CapsuleReportPrunablesCallbackParams : VolumeReportPrunablesCallbackParams
{
	// In
	const Capsule*					mWorldCapsule;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PX_FORCE_INLINE bool VolumeReportPrunablesCallbackParams::preOverlapTestFilteringCode(SceneQueryShapeData* currentShape, PxSceneQueryFilterFlags& outFilterFlags, PxSceneQueryHitType::Enum& outHitType)
{
	if(applyClientFilter(mQueryClient, mPassForeignShapes, *currentShape))
		return false;

	if(mIsBatchQuery)
	{
		if(applyBatchedPreFilterPreTest(mPreFilterShader, *mFilterData, mConstBlock, mcBlockSize, mFilterFlags, *currentShape, outFilterFlags, outHitType))
			return false;
	}
	else
	{
		if(applyNonBatchedFilterPreTest(mFilterCall, *mFilterData, mFilterFlags, *currentShape, outFilterFlags, outHitType))
			return false;
	}

	if(outHitType == PxSceneQueryHitType::eNONE)
		return false;

	return true;
}

PX_FORCE_INLINE bool VolumeReportPrunablesCallbackParams::registerHit(SceneQueryShapeData* currentShape, PxSceneQueryHitType::Enum& hitType, PxSceneQueryFilterFlags& modFilterFlags)
{
	PxSceneQueryHit hit;
	hit.shape = currentShape->shape;

	applyPostFilter(mIsBatchQuery, hit, currentShape, modFilterFlags, hitType);

	if(hitType != PxSceneQueryHitType::eNONE)
	{
		if(mMultipleOverlaps && (mNbHits < mMaxShapes))
		{
			mBuffer[mNbHits] = currentShape->shape;
			mNbHits++;
		}
		else if(!mMultipleOverlaps)
		{
			mBuffer[0] = currentShape->shape;
			mNbHits = 1;
			return false;
		}
		else
		{
			mBufferOverflow = true;
			return false;
		}
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static bool sphereReportPrunablesCallback(Prunable** prunables, PxU32 nb, void* userData)
{
	SphereReportPrunablesCallbackParams* PX_RESTRICT params = (SphereReportPrunablesCallbackParams* PX_RESTRICT)userData;

	PX_ASSERT(params->mBuffer);

	if(params->mBufferOverflow)
		return false;

	if(!params->mMultipleOverlaps && params->mNbHits)
		return false;

	Prunable** lastPrunable = prunables + nb;
	while(prunables!=lastPrunable)
	{
		Prunable* p = *prunables++;
		SceneQueryShapeData* currentShape = static_cast<SceneQueryShapeData*>(p);

		PxSceneQueryFilterFlags modFilterFlags;
		PxSceneQueryHitType::Enum hitType = PxSceneQueryHitType::eTOUCH;
		if(!params->preOverlapTestFilteringCode(currentShape, modFilterFlags, hitType))
			continue;

		const GeomOverlapSphereFunc overlap = gGeomOverlapSphereMap[currentShape->getGeometryType()];
		if(!overlap(currentShape->shapeGeometryEA->get(), currentShape->sqGlobalPose, *params->mWorldSphere))
			continue;

		if(!params->registerHit(currentShape, hitType, modFilterFlags))
			return false;
	}
	return true;
}

static bool aabbReportPrunablesCallback(Prunable** prunables, PxU32 nb, void* userData)
{
	AabbReportPrunablesCallbackParams* PX_RESTRICT params = (AabbReportPrunablesCallbackParams* PX_RESTRICT)userData;

	PX_ASSERT(params->mBuffer);

	if(params->mBufferOverflow)
		return false;

	if(!params->mMultipleOverlaps && params->mNbHits)
		return false;

	Prunable** lastPrunable = prunables + nb;
	while(prunables!=lastPrunable)
	{
		Prunable* p = *prunables++;
		SceneQueryShapeData* currentShape = static_cast<SceneQueryShapeData*>(p);

		PxSceneQueryFilterFlags modFilterFlags;
		PxSceneQueryHitType::Enum hitType = PxSceneQueryHitType::eTOUCH;
		if(!params->preOverlapTestFilteringCode(currentShape, modFilterFlags, hitType))
			continue;
		
		const GeomOverlapOBBFunc overlap = gGeomOverlapOBBMap[currentShape->getGeometryType()];
		// PT: TODO: precompute this obb
		Gu::Box worldOBB;
		worldOBB.center = params->mWorldBounds->getCenter();
		worldOBB.extents = params->mWorldBounds->getExtents();
		worldOBB.rot = PxMat33::createIdentity();
		if(!overlap(currentShape->shapeGeometryEA->get(), currentShape->sqGlobalPose, worldOBB))
			continue;

		if(!params->registerHit(currentShape, hitType, modFilterFlags))
			return false;
	}
	return true;
}

static bool obbReportPrunablesCallback(Prunable** prunables, PxU32 nb, void* userData)
{
	ObbReportPrunablesCallbackParams* PX_RESTRICT params = (ObbReportPrunablesCallbackParams* PX_RESTRICT)userData;

	PX_ASSERT(params->mBuffer);

	if(params->mBufferOverflow)
		return false;

	if(!params->mMultipleOverlaps && params->mNbHits)
		return false;

	Prunable** lastPrunable = prunables + nb;
	while(prunables!=lastPrunable)
	{
		Prunable* p = *prunables++;
		SceneQueryShapeData* currentShape = static_cast<SceneQueryShapeData*>(p);

		PxSceneQueryFilterFlags modFilterFlags;
		PxSceneQueryHitType::Enum hitType = PxSceneQueryHitType::eTOUCH;
		if(!params->preOverlapTestFilteringCode(currentShape, modFilterFlags, hitType))
			continue;

		const GeomOverlapOBBFunc overlap = gGeomOverlapOBBMap[currentShape->getGeometryType()];
		if(!overlap(currentShape->shapeGeometryEA->get(), currentShape->sqGlobalPose, *params->mWorldBox))
			continue;

		if(!params->registerHit(currentShape, hitType, modFilterFlags))
			return false;
	}
	return true;
}

static bool capsuleReportPrunablesCallback(Prunable** prunables, PxU32 nb, void* userData)
{
	CapsuleReportPrunablesCallbackParams* PX_RESTRICT params = (CapsuleReportPrunablesCallbackParams* PX_RESTRICT)userData;

	PX_ASSERT(params->mBuffer);

	if(params->mBufferOverflow)
		return false;

	if(!params->mMultipleOverlaps && params->mNbHits)
		return false;

	Prunable** lastPrunable = prunables + nb;
	while(prunables!=lastPrunable)
	{
		Prunable* p = *prunables++;
		SceneQueryShapeData* currentShape = static_cast<SceneQueryShapeData*>(p);

		PxSceneQueryFilterFlags modFilterFlags;
		PxSceneQueryHitType::Enum hitType = PxSceneQueryHitType::eTOUCH;
		if(!params->preOverlapTestFilteringCode(currentShape, modFilterFlags, hitType))
			continue;

		const GeomOverlapCapsuleFunc overlap = gGeomOverlapCapsuleMap[currentShape->getGeometryType()];
		if(!overlap(currentShape->shapeGeometryEA->get(), currentShape->sqGlobalPose, *params->mWorldCapsule))
			continue;

		if(!params->registerHit(currentShape, hitType, modFilterFlags))
			return false;
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// PT: TODO: what is userData here? not used?

PxI32 SceneQueryManager::overlapSphereObjects(const Sphere& worldSphere, bool multipleOverlaps, PxU32 nbShapes, PxShape** objects, void* userData, PxU32 filterFlags, PxSceneQueryFilterCallback* filterCall, const PxFilterData* filterData, PxBatchQueryPreFilterShader preFilterShader, PxBatchQueryPostFilterShader postFilterShader, const void* constBlock, PxU32 cBlockSize, PxClientID queryClient, bool isBatchQuery)
{
	SphereReportPrunablesCallbackParams params;
	params.initVolumeReport(getScene(), multipleOverlaps, nbShapes, objects, filterFlags, filterCall, filterData, preFilterShader, postFilterShader, constBlock, cBlockSize, queryClient, isBatchQuery);
	params.mWorldSphere = &worldSphere;
	overlap(sphereReportPrunablesCallback, &params, worldSphere, filterFlags);

	if(!params.mBufferOverflow)
		return params.mNbHits;
	else
		return -1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxI32 SceneQueryManager::overlapAABBObjects(const PxBounds3& worldBounds, bool multipleOverlaps, PxU32 nbShapes, PxShape** objects, void* userData, PxU32 filterFlags, PxSceneQueryFilterCallback* filterCall, const PxFilterData* filterData, PxBatchQueryPreFilterShader preFilterShader, PxBatchQueryPostFilterShader postFilterShader, const void* constBlock, PxU32 cBlockSize, PxClientID queryClient, bool isBatchQuery)
{
	AabbReportPrunablesCallbackParams params;
	params.initVolumeReport(getScene(), multipleOverlaps, nbShapes, objects, filterFlags, filterCall, filterData, preFilterShader, postFilterShader, constBlock, cBlockSize, queryClient, isBatchQuery);
	params.mWorldBounds = &worldBounds;
	overlap(aabbReportPrunablesCallback, &params, worldBounds, filterFlags);

	if(!params.mBufferOverflow)
		return params.mNbHits;
	else
		return -1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxI32 SceneQueryManager::overlapOBBObjects(const PxVec3& boxCenter, const PxVec3& boxExtents, const PxQuat& boxRot, bool multipleOverlaps,
											   PxU32 nbShapes, PxShape** objects, void* userData,
											   PxU32 filterFlags, PxSceneQueryFilterCallback* filterCall,
											   const PxFilterData* filterData, PxBatchQueryPreFilterShader preFilterShader, PxBatchQueryPostFilterShader postFilterShader, 
											   const void* constBlock, PxU32 cBlockSize, PxClientID queryClient,
											   bool isBatchQuery)
{
	PX_CHECK_VALID(boxCenter); PX_CHECK_VALID(boxExtents); PX_CHECK_VALID(boxRot);

	Box obb;
	buildFrom(obb, boxCenter, boxExtents, boxRot);

	ObbReportPrunablesCallbackParams params;
	params.initVolumeReport(getScene(), multipleOverlaps, nbShapes, objects, filterFlags, filterCall, filterData, preFilterShader, postFilterShader, constBlock, cBlockSize, queryClient, isBatchQuery);
	params.mWorldBox = &obb;
	overlap(obbReportPrunablesCallback, &params, obb, filterFlags);

	if(!params.mBufferOverflow)
		return params.mNbHits;
	else
		return -1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxI32 SceneQueryManager::overlapCapsuleObjects(const Capsule& worldCapsule, bool multipleOverlaps, PxU32 nbShapes, PxShape** objects, void* userData, PxU32 filterFlags, PxSceneQueryFilterCallback* filterCall, const PxFilterData* filterData, PxBatchQueryPreFilterShader preFilterShader, PxBatchQueryPostFilterShader postFilterShader, const void* constBlock, PxU32 cBlockSize, PxClientID queryClient, bool isBatchQuery)
{
	CapsuleReportPrunablesCallbackParams params;
	params.initVolumeReport(getScene(), multipleOverlaps, nbShapes, objects, filterFlags, filterCall, filterData, preFilterShader, postFilterShader, constBlock, cBlockSize, queryClient, isBatchQuery);
	params.mWorldCapsule = &worldCapsule;
	overlap(capsuleReportPrunablesCallback, &params, worldCapsule, filterFlags);

	if(!params.mBufferOverflow)
		return params.mNbHits;
	else
		return -1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define PRE_CULL_TEST_FILTERING_CODE(outFilterFlags, outHitType, defaultHitType)						\
																										\
PxSceneQueryHitType::Enum outHitType = defaultHitType;													\
PxSceneQueryFilterFlags outFilterFlags;																	\
																										\
if (applyBatchedPreFilterPreTest(preFilterShader, *filterData, constBlock,cBlockSize,					\
	 filterFlags, *currentShape, outFilterFlags, outHitType)) continue;									\

static PxU32 reportObjects(PxU32 nb, Prunable** prunables, const PxU32 maxShapes, PxShape** buffer, void* userData, PxU32 filterFlags,
						   const PxFilterData* filterData, PxBatchQueryPreFilterShader preFilterShader, PxBatchQueryPostFilterShader postFilterShader,
						   const void* constBlock, PxU32 cBlockSize, PxClientID queryClient, bool passForeignShapes, bool skipFiltering)
{
	PxU32 totalNbValid = 0;
	Prunable** lastPrunable = prunables + nb;
	while(prunables!=lastPrunable)
	{
		// Handle current batch
		PxShape** runningBuffer = buffer;
		PxShape** last = buffer + maxShapes;
		while(runningBuffer!=last && prunables!=lastPrunable)
		{
			Prunable* p = *prunables++;
			SceneQueryShapeData* currentShape = static_cast<SceneQueryShapeData*>(p);

			PRE_CULL_TEST_FILTERING_CODE(outFilterFlags, hitType, PxSceneQueryHitType::eTOUCH)

			if(hitType == PxSceneQueryHitType::eNONE || hitType == PxSceneQueryHitType::eTOUCH)
				continue;
			
			PX_ASSERT(currentShape);
			*runningBuffer = currentShape->shape;
			runningBuffer++;
		}

		PxU32 nbValid = PxU32(runningBuffer - buffer);
		totalNbValid += nbValid;
	}
	return totalNbValid;
}

PxU32 SceneQueryManager::cullObjects(PxU32 nbPlanes, const Plane* worldPlanes, PxU32 nbShapes, PxShape** objects, void* userData, PxU32 filterFlags, PxSceneQueryFilterCallback* filterCall, const PxFilterData* filterData, PxBatchQueryPreFilterShader preFilterShader, PxBatchQueryPostFilterShader postFilterShader, const void* constBlock, PxU32 cBlockSize, PxClientID queryClient)
{
	QueryBuffersGrab queryBuffers(*this);
	CulledObjects culledObjects(queryBuffers->cullingResults);
	culledObjects.ResetObjects();

	cull(queryBuffers->cullingTemps, culledObjects, (const Plane*)worldPlanes, nbPlanes, filterFlags);

	PxU32 nb = culledObjects.GetNbPrunables();
	Prunable** prunables = (Prunable**)culledObjects.GetData();

	const bool passForeignShapes = (getScene().getClientBehaviorBits(queryClient) & PxClientBehaviorBit::eREPORT_FOREIGN_OBJECTS_TO_SCENE_QUERY) != 0;
	return reportObjects(nb, prunables, nbShapes, objects, userData, filterFlags, filterData, preFilterShader, postFilterShader, constBlock, cBlockSize, queryClient, passForeignShapes, true);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
Questions:
- using SAT is faster?
- multithread ?
- filtering ?
- sync/not ?
- all hits/closest, same API ? different from raycast
- PxActor or PxShape ? No compounds in pruning engine so super painful to go back to actors & filter subobjects!
- how to handle the penetration case ?
- relative motion culling ?
- local tests (no world)
- vs planes?
- check dir/motion difference in sweep tests
- check all normals are correct
- shrink query volume
- refactoring & optimization
- vs heightfields?
- check filtering vs HW scene
- check dynamic pruner signature => seems ok when objects are sleeping
- draw debug bounds, make sure they're fine
- run extra box-box tests out of the cache, since it's loose
- a dynamic container / cache isn't too great
- shrinking + sort object by distance + extra tests? SAP looks really better here.
- sort by distance or just reuse last touched object first...
- sort by distance *inside the cache* ?
- cache extruded tris.........
- better implicit cache volume
- refactor CCT code, or at least move sweep test changes back to CCT
- opcode's sweep test => not compatible with cache & hybrid models! Hmmm.
- check out Bullet for convex casts
- test long raycast-like sweeps in the sample
- make sure the current best distance is passed down to the sphere-vs-tri code
- the extra tests are quite expensive for a mesh, since they go to the triangle level => use box-box instead
- add stats (number of actual queries, etc)
*/

//void sortObjects(Ice::PrunedObjects& objects, const PxVec3& dir)
//{
//	PxU32 nb = objects.GetNbPrunables();
//	Prunable** prunables = objects.GetPrunables();

//	PX_ALLOCA(keys, PxF32, nb);

//	for(PxU32 i=0;i<nb;i++)
//	{
//		Prunable* p = prunables[i];
//		NpShape* currentShape = reinterpret_cast<NpShape*>(p->GetUserData());
//		keys[i] = dir|Sq::getGlobalPose(*currentShape).p;
//	}

//	Ice::RadixSortBuffered RS;
//	const PxU32* sorted = RS.Sort(keys, nb).GetRanks();

//	// Reuse stack memory
//	Ps::memCopy(keys, prunables, nb*sizeof(void*));
//	Prunable** temp = (Prunable**)(PxF32*)keys;
//	for(PxU32 i=0;i<nb;i++)
//		*prunables++ = temp[*sorted++];
//}

// PT: SIGH. "worldBox" wasn't even used. Apparently the whole "shrink motion" optimization vanished.

struct SqLinearSweepParams : SqFilteringParams
{
	void init(	const SweptVolume& volume, const PxSweepHit& blockingHit,
				const PxVec3& unitDir, PxU32 nbHits, PxSweepHit* hits, bool& hasBlockingHit, bool anyHit, 
				const SceneQueryShapeData* cache, PxSceneQueryFlags hintFlags, bool isBatchQuery,
				bool sweepCacheUsed)
	{
		mBlockingHit	= blockingHit;
		mVolume			= &volume;
		mCache			= cache;
		mUnitDir		= &unitDir;
		mNbShapes		= nbHits;
		mObjects		= hits;
		mHasBlockingHit	= &hasBlockingHit;
		mHintFlags		= hintFlags;
		mAnyHit			= anyHit;
		mSweepCacheUsed	= sweepCacheUsed;
		mIsBatchQuery	= isBatchQuery;
	}

	PX_FORCE_INLINE bool PreSweepTestFiltering(const SceneQueryShapeData* currentShape, PxSceneQueryHitType::Enum& outHitType, PxSceneQueryFilterFlags& outFilterFlags)	const
	{
		if(currentShape == mCache)  // Cached shape is tested in advance
			return false;

		if(applyClientFilter(mQueryClient, mPassForeignShapes, *currentShape))
			return false;

		// In the case of a user provided sweep cache, we do not know whether dynamic and/or static objects
		// are in there, hence, we need to filter
		// TODO: Order the objects in the cache accordingly and add API to extract only the relevant ones
		if(mSweepCacheUsed)
		{
			const PxType actorType = currentShape->getActor().getSerialType();
			if(actorType == PxSerialType::eRIGID_DYNAMIC || actorType == PxSerialType::eARTICULATION_LINK)
			{
				if(!(mFilterFlags & PxSceneQueryFilterFlag::eDYNAMIC))
					return false;
			}
			else
			{
				if(!(mFilterFlags & PxSceneQueryFilterFlag::eSTATIC))
					return false;
			}
		}

		if(mIsBatchQuery)
		{
			PX_ASSERT(!mFilterCall);
			if(applyBatchedPreFilterPreTest(mPreFilterShader, *mFilterData, mConstBlock, mcBlockSize,
			   mFilterFlags, *currentShape, outFilterFlags, outHitType))
			   return false;
		}
		else
		{
			if(applyNonBatchedFilterPreTest(mFilterCall, *mFilterData, mFilterFlags, *currentShape,
				outFilterFlags, outHitType))
				return false;
		}
		return true;
	}

	PxSweepHit					mBlockingHit;
	const SweptVolume*			mVolume;
	const SceneQueryShapeData*	mCache;
	const PxVec3*				mUnitDir;
	PxU32						mNbShapes;
	PxSweepHit*					mObjects;
	bool*						mHasBlockingHit;
	PxSceneQueryFlags			mHintFlags;
	bool						mAnyHit;
	bool						mSweepCacheUsed;
	bool						mIsBatchQuery;
};

struct SqLinearSweepSingleHitParams : SqLinearSweepParams
{
	PX_FORCE_INLINE	void	prologue()
	{
		PX_ASSERT(mBlockingHit.distance > 0.0f);
		PX_ASSERT(!mUnitDir->isZero());
	}

	PX_FORCE_INLINE	PxI32	epilogue()
	{
		if(mBlockingHit.shape)
		{
			PX_ASSERT(mObjects);
			*mObjects = mBlockingHit;
			return 1;
		}
		else return 0;
	}
};

struct SqLinearSweepMultipleHitsParams : SqLinearSweepParams
{
	PX_FORCE_INLINE	void	prologue()
	{
		PX_ASSERT(mBlockingHit.distance > 0.0f);
		PX_ASSERT(!mUnitDir->isZero());
		PX_ASSERT(mObjects);

		mBlockingHitCount	= mBlockingHit.shape ? 1 : 0;;
		mNbHits				= 0;
		mMaxTmpBufferSize	= mNbShapes;
		mRunningBuffer		= mObjects;
		mTmpHitBuffer		= mObjects;
		mUserHitBufferAsTmp	= true;
	}

	PX_FORCE_INLINE	PxI32	epilogue()
	{
		PxI32 returnValue;
		if((mNbHits + mBlockingHitCount) <= mNbShapes)
			returnValue = mNbHits + mBlockingHitCount;
		else
		{
			returnValue = -1;
			mNbHits = mNbShapes - mBlockingHitCount;
		}

		if(!mUserHitBufferAsTmp)
		{
			Ps::memCopy(mObjects, mTmpHitBuffer, sizeof(PxSweepHit) * mNbHits);  // Write hit information back if the user buffer did not have enough space for the temporary results
			physx::shdfnd::TempAllocator().deallocate(mTmpHitBuffer);
		}

		if(mBlockingHitCount > 0)
		{
			mObjects[mNbHits] = mBlockingHit;
			*mHasBlockingHit = true;
		}
		else
			*mHasBlockingHit = false;

		return returnValue;
	}

	PxU32		mBlockingHitCount;
	PxU32		mNbHits;		// NOT including a potential blocking hit
	PxU32		mMaxTmpBufferSize;
	PxSweepHit* mRunningBuffer;
	PxSweepHit*	mTmpHitBuffer;
	bool		mUserHitBufferAsTmp;  // The user provided hit buffer is used as temporary buffer (on overflow a new temporary buffer is created)
};

static bool gLinearSweepMultipleHitsCallback(Prunable** prunables, PxU32 nb, void* userData)
{
	// PT: TODO: investigate why the restricted version crashes in Release (SQ unit tests)
//	SqLinearSweepMultipleHitsParams* PX_RESTRICT params = (SqLinearSweepMultipleHitsParams* PX_RESTRICT)userData;
	SqLinearSweepMultipleHitsParams* params = (SqLinearSweepMultipleHitsParams*)userData;

	PxU32 nbHits = params->mNbHits;
	PxU32 maxTmpBufferSize = params->mMaxTmpBufferSize;
	PxSweepHit* runningBuffer = params->mRunningBuffer;
	PxSweepHit*	tmpHitBuffer = params->mTmpHitBuffer;

	// Loop through touched objects
	Prunable** last = prunables + nb;
	while(prunables!=last)
	{
		const Prunable* p = *prunables++;
		const SceneQueryShapeData* currentShape = static_cast<const SceneQueryShapeData*>(p);

		PxSceneQueryHitType::Enum hitType = PxSceneQueryHitType::eTOUCH;
		PxSceneQueryFilterFlags modFilterFlags;
		if(!params->PreSweepTestFiltering(currentShape, hitType, modFilterFlags))
			continue;

		if(hitType == PxSceneQueryHitType::eNONE)
			continue;

		PxSweepHit hit;
		if(!sweepVolumeAgainstShape(*params->mVolume, *params->mUnitDir, params->mBlockingHit.distance, currentShape, hit, params->mHintFlags | PxSceneQueryFlag::eDISTANCE))  // eDISTANCE needed for comparing distances of blocking hits
			continue;
		PX_ASSERT(hit.distance <= params->mBlockingHit.distance);

		hit.shape = currentShape->shape;

		if(hit.distance==0.0f)
			hit.flags |= PxSceneQueryFlag::eINITIAL_OVERLAP;

		params->applyPostFilter(params->mIsBatchQuery, hit, currentShape, modFilterFlags, hitType);

		if (hitType == PxSceneQueryHitType::eNONE)
			continue;

		if (hitType == PxSceneQueryHitType::eTOUCH)
		{
			PX_ASSERT(!(hit.flags & PxSceneQueryFlag::eBLOCKING_HIT));
			hit.flags |= PxSceneQueryFlag::eTOUCHING_HIT;

			if(nbHits < maxTmpBufferSize)
			{
				*runningBuffer++ = hit;
				nbHits++;
			}
			else
			{
				// Buffer overflow: Grow the buffer to cover all objects
				const PxU32 newMaxCount = maxTmpBufferSize * 2 + 10;  // The constant coefficient is necessary because maxTmpBufferSize can be 0 (happens because the method might get called from linearCompoundGeometrySweep() with a full buffer)
				PxSweepHit* newBuffer = (PxSweepHit*)physx::shdfnd::TempAllocator().allocate(sizeof(PxSweepHit) * newMaxCount, __FILE__, __LINE__);

				if (newBuffer)
				{
					Ps::memCopy(newBuffer, tmpHitBuffer, sizeof(PxSweepHit) * maxTmpBufferSize);

					if(tmpHitBuffer != params->mObjects)
						physx::shdfnd::TempAllocator().deallocate(tmpHitBuffer);

					runningBuffer = newBuffer + maxTmpBufferSize;
					tmpHitBuffer = newBuffer;
					maxTmpBufferSize = newMaxCount;

					params->mUserHitBufferAsTmp = false;

					*runningBuffer++ = hit;
					nbHits++;
				}
				else
				{
					physx::shdfnd::getFoundation().error(physx::PxErrorCode::eOUT_OF_MEMORY, __FILE__, __LINE__, "sweep query: Memory allocation failed. Incomplete list of hits should be expected.");
				}
			}
		}
		else
		{
			params->mBlockingHitCount = 1;

			PX_ASSERT(hitType == PxSceneQueryHitType::eBLOCK);
			PX_ASSERT(hit.flags & PxSceneQueryFlag::eDISTANCE);

			PX_ASSERT(!(hit.flags & PxSceneQueryFlag::eTOUCHING_HIT));
			hit.flags |= PxSceneQueryFlag::eBLOCKING_HIT;

			params->mBlockingHit = hit;
			PX_ASSERT(params->mBlockingHit.shape == currentShape->shape);

			PxU32 currentIdx = 0;
			for(PxU32 i=0; i < nbHits; i++)  // check all touching objects and keep the closer ones only
			{
				PX_ASSERT(tmpHitBuffer[i].flags & PxSceneQueryFlag::eDISTANCE);
				if(tmpHitBuffer[i].distance <= params->mBlockingHit.distance)
				{
					tmpHitBuffer[currentIdx] = tmpHitBuffer[i];
					currentIdx++;
				}
			}
			nbHits = currentIdx;
		}
	}

	params->mNbHits = nbHits;
	params->mMaxTmpBufferSize = maxTmpBufferSize;
	params->mRunningBuffer = runningBuffer;
	params->mTmpHitBuffer = tmpHitBuffer;
	return true;
}

static bool gLinearSweepSingleHitCallback(Prunable** prunables, PxU32 nb, void* userData)
{
	SqLinearSweepSingleHitParams* PX_RESTRICT params = (SqLinearSweepSingleHitParams* PX_RESTRICT)userData;

	// Loop through touched objects
	Prunable** last = prunables + nb;
	while(prunables!=last)
	{
		const Prunable* p = *prunables++;
		const SceneQueryShapeData* currentShape = static_cast<const SceneQueryShapeData*>(p);
		PX_ASSERT(currentShape);

		PxSceneQueryHitType::Enum hitType = PxSceneQueryHitType::eBLOCK;
		PxSceneQueryFilterFlags modFilterFlags;
		if(!params->PreSweepTestFiltering(currentShape, hitType, modFilterFlags))
			continue;

		if(hitType == PxSceneQueryHitType::eNONE || hitType == PxSceneQueryHitType::eTOUCH)
			continue;

		PxSweepHit currentHit;
		currentHit.faceIndex = PX_INVALID_U32;

		if(!sweepVolumeAgainstShape(*params->mVolume, *params->mUnitDir, params->mBlockingHit.distance, currentShape, currentHit, params->mHintFlags))
			continue;
		PX_ASSERT(currentHit.distance<=params->mBlockingHit.distance);

		if ((currentHit.distance < params->mBlockingHit.distance) || params->mAnyHit)
		{
			currentHit.shape = currentShape->shape;

			if(currentHit.distance==0.0f)
				currentHit.flags |= PxSceneQueryFlag::eINITIAL_OVERLAP;

			params->applyPostFilter(params->mIsBatchQuery, currentHit, currentShape, modFilterFlags, hitType);

			if (hitType == PxSceneQueryHitType::eNONE || hitType == PxSceneQueryHitType::eTOUCH)
				continue;

			PX_ASSERT(!(currentHit.flags & PxSceneQueryFlag::eTOUCHING_HIT));
			currentHit.flags |= PxSceneQueryFlag::eBLOCKING_HIT;

			params->mBlockingHit = currentHit;

			if(params->mAnyHit)
				return false;
		}
	}
	return true;
}

// PT: runs a linear sweep test from an array of "Prunable". This should only be called when the "sweep cache" is used.
static PxI32 runLinearSweep(Scb::Scene& scene, const SweptVolume& volume, const PxSweepHit& blockingHit,
							const PxVec3& unitDir, PxU32 nbHits, PxSweepHit* hits, bool& hasBlockingHit, bool multipleHits, bool anyHit, 
							PxU32 filterFlags, PxSceneQueryFilterCallback* filterCall, const PxFilterData* filterData, PxBatchQueryPreFilterShader preFilterShader,
							PxBatchQueryPostFilterShader postFilterShader, const void* constBlock, PxU32 cBlockSize, 
							PxClientID queryClient, const PrunedObjects* objects, const SceneQueryShapeData* cache, PxSceneQueryFlags hintFlags, bool isBatchQuery)
{
	if(multipleHits)
	{
		SqLinearSweepMultipleHitsParams params;
		params.initFiltering(filterFlags, filterCall, filterData, preFilterShader, postFilterShader, constBlock, cBlockSize, queryClient, scene);
		params.init(volume, blockingHit, unitDir, nbHits, hits, hasBlockingHit, anyHit, cache, hintFlags, isBatchQuery, true);

		params.prologue();
		gLinearSweepMultipleHitsCallback(const_cast<Prunable**>(objects->GetPrunables()), objects->GetNbPrunables(), &params);
		return params.epilogue();
	}
	else
	{
		SqLinearSweepSingleHitParams params;
		params.initFiltering(filterFlags, filterCall, filterData, preFilterShader, postFilterShader, constBlock, cBlockSize, queryClient, scene);
		params.init(volume, blockingHit, unitDir, nbHits, hits, hasBlockingHit, anyHit, cache, hintFlags, isBatchQuery, true);

		params.prologue();
		gLinearSweepSingleHitCallback(const_cast<Prunable**>(objects->GetPrunables()), objects->GetNbPrunables(), &params);
		return params.epilogue();
	}
}

PxI32 SceneQueryManager::runLinearSweepCB(	const SweptVolume& volume, const PxSweepHit& blockingHit, const Box& querySweptBox, const PxVec3& unitDir, 
											PxU32 nbHits, PxSweepHit* hits, bool& hasBlockingHit, bool multipleHits, bool anyHit, 
											PxU32 filterFlags, PxSceneQueryFilterCallback* filterCall, const PxFilterData* filterData, PxBatchQueryPreFilterShader preFilterShader,
											PxBatchQueryPostFilterShader postFilterShader, const void* constBlock, PxU32 cBlockSize, 
											PxClientID queryClient, const SceneQueryShapeData* cache, PxSceneQueryFlags hintFlags, bool isBatchQuery)
{
	if(multipleHits)
	{
		SqLinearSweepMultipleHitsParams params;
		params.initFiltering(filterFlags, filterCall, filterData, preFilterShader, postFilterShader, constBlock, cBlockSize, queryClient, getScene());
		params.init(volume, blockingHit, unitDir, nbHits, hits, hasBlockingHit, anyHit, cache, hintFlags, isBatchQuery, false);

		params.prologue();
		overlap(gLinearSweepMultipleHitsCallback, &params, querySweptBox, filterFlags);
		return params.epilogue();
	}
	else
	{
		SqLinearSweepSingleHitParams params;
		params.initFiltering(filterFlags, filterCall, filterData, preFilterShader, postFilterShader, constBlock, cBlockSize, queryClient, getScene());
		params.init(volume, blockingHit, unitDir, nbHits, hits, hasBlockingHit, anyHit, cache, hintFlags, isBatchQuery, false);

		params.prologue();
		overlap(gLinearSweepSingleHitCallback, &params, querySweptBox, filterFlags);
		return params.epilogue();
	}
}

PxI32 SceneQueryManager::linearOBBSweep(	const PxVec3& boxCenter, const PxVec3& boxExtents, const PxQuat& boxRot, const PxVec3& unitDir, const PxReal distance, 
											PxU32 nbHits, PxSweepHit* hits, bool& hasBlockingHit, bool multipleHits, bool anyHit, 
											PxU32 filterFlags, PxSceneQueryFilterCallback* filterCall, const PxFilterData* filterData, PxBatchQueryPreFilterShader preFilterShader,
											PxBatchQueryPostFilterShader postFilterShader, const void* constBlock, PxU32 cBlockSize, 
											PxClientID queryClient, PrunedObjects* objects, const SceneQueryShapeData* cache, PxSceneQueryFlags hintFlags, bool isBatchQuery)
{
	// PT: TODO: get rid of this conversion
	SweptBox SB;
	buildFrom(SB.mBox, boxCenter, boxExtents, boxRot);

	PxSweepHit blockingHit;
	blockingHit.shape		= NULL;
	blockingHit.flags		= PxSceneQueryFlags(0);
	blockingHit.distance	= distance;
	blockingHit.faceIndex	= PX_INVALID_U32;

	if(!objects)
	{
		// Handle cache if needed
		if(cache)
		{
			// No filtering for cached shape
			const SweepBoxFunc sf = gSweepBoxMap[cache->getGeometryType()];
			if((sf)(cache->shapeGeometryEA->get(), cache->sqGlobalPose, SB.mBox, unitDir, distance, blockingHit, hintFlags|PxSceneQueryFlag::eDISTANCE))	// compute distance to reduce the sweep distance if the cached shape hits
			{
				PX_ASSERT(blockingHit.distance <= distance);
				PX_ASSERT(blockingHit.flags & PxSceneQueryFlag::eDISTANCE);

				blockingHit.shape = cache->shape;

				hasBlockingHit = true;

				if (anyHit)
				{
					PX_ASSERT(nbHits > 0);
					hits[0] = blockingHit;
					return 1;
				}
			}
			else
				blockingHit.flags = PxSceneQueryFlags(0);
		}

		// Create query OBB
		Box querySweptBox;
		{
			// PT: TODO: isn't it the same as SB.mBox?
			Box obb;
			buildFrom(obb, boxCenter, boxExtents, boxRot);

			CreateOBB(querySweptBox, obb, unitDir, distance);
		}

		/*		// TEST
		OBB querySweptBox2;
		{
		Box dest;
		PxF32 d = motion.magnitude();
		createOBB(dest, worldBox, PxVec3(motion.x, motion.y, motion.z)/d, d);
		PxToICE(querySweptBox2, dest.extents, &dest.center, &dest.rot);
		}
		*/

		return runLinearSweepCB(SB, blockingHit, querySweptBox, unitDir,  nbHits, hits, hasBlockingHit, multipleHits, anyHit, 
								filterFlags, filterCall, filterData, preFilterShader, postFilterShader, constBlock, cBlockSize, 
								queryClient, cache, hintFlags, isBatchQuery);
	}
	else
	{
		return runLinearSweep(	getScene(), SB, blockingHit, unitDir, nbHits, hits, hasBlockingHit, multipleHits, anyHit, 
								filterFlags, filterCall, filterData, preFilterShader, postFilterShader, constBlock, cBlockSize, 
								queryClient, objects, cache, hintFlags, isBatchQuery);
	}
}

PxI32 SceneQueryManager::linearCapsuleSweep(	const Capsule& worldCapsule, const PxVec3& unitDir, const PxReal distance, 
												PxU32 nbHits, PxSweepHit* hits, bool& hasBlockingHit, bool multipleHits, bool anyHit, 
												PxU32 filterFlags, PxSceneQueryFilterCallback* filterCall, const PxFilterData* filterData, PxBatchQueryPreFilterShader preFilterShader,
												PxBatchQueryPostFilterShader postFilterShader, const void* constBlock, PxU32 cBlockSize, 
												PxClientID queryClient, PrunedObjects* objects, const SceneQueryShapeData* cache, PxSceneQueryFlags hintFlags, bool isBatchQuery)
{
	SweptCapsule SC;
	SC.mCapsule = worldCapsule;

	PxSweepHit blockingHit;
	blockingHit.shape		= NULL;
	blockingHit.flags		= PxSceneQueryFlags(0);
	blockingHit.distance	= distance;
	blockingHit.faceIndex	= PX_INVALID_U32;

	const bool passForeignShapes = (getScene().getClientBehaviorBits(queryClient) & PxClientBehaviorBit::eREPORT_FOREIGN_OBJECTS_TO_SCENE_QUERY) != 0;

	if(!objects)
	{
		// Handle cache if needed
		if(cache)
		{
			// No filtering for cached shape
			const SweepCapsuleFunc sf = gSweepCapsuleMap[cache->getGeometryType()];
			if((sf)(cache->shapeGeometryEA->get(), cache->sqGlobalPose, worldCapsule, unitDir, distance, blockingHit, hintFlags|PxSceneQueryFlag::eDISTANCE))	// compute distance to reduce the sweep distance if the cached shape hits
			{
				PX_ASSERT(blockingHit.distance <= distance);
				PX_ASSERT(blockingHit.flags & PxSceneQueryFlag::eDISTANCE);

				blockingHit.shape = cache->shape;

				if (anyHit)
				{
					PX_ASSERT(nbHits > 0);
					hits[0] = blockingHit;
					return 1;
				}
			}
			else
				blockingHit.flags = PxSceneQueryFlags(0);
		}

		// Create query OBB
		Box querySweptBox;
		{
			Box obb;
			computeBoxAroundCapsule(worldCapsule, obb);

			CreateOBB(querySweptBox, obb, unitDir, distance);
		}

		return runLinearSweepCB(SC, blockingHit, querySweptBox, unitDir,  nbHits, hits, hasBlockingHit, multipleHits, anyHit, 
								filterFlags, filterCall, filterData, preFilterShader, postFilterShader, constBlock, cBlockSize, 
								queryClient, cache, hintFlags, isBatchQuery);
	}
	else
	{
		return runLinearSweep(	getScene(), SC, blockingHit, unitDir, nbHits, hits, hasBlockingHit, multipleHits, anyHit, 
								filterFlags, filterCall, filterData, preFilterShader, postFilterShader, constBlock, cBlockSize, 
								queryClient, objects, cache, hintFlags, isBatchQuery);
	}
}

PxI32 SceneQueryManager::linearGeometrySweep(	const PxGeometry& geometry, const PxTransform& pose, const PxVec3& unitDir, const PxReal distance, 
												PxU32 nbHits, PxSweepHit* hits, bool& hasBlockingHit, bool multipleHits, bool anyHit, 
												PxU32 filterFlags, PxSceneQueryFilterCallback* filterCall, const PxFilterData* filterData, PxBatchQueryPreFilterShader preFilterShader,
												PxBatchQueryPostFilterShader postFilterShader, const void* constBlock, PxU32 cBlockSize,
												PxClientID queryClient, PrunedObjects* objects, const SceneQueryShapeData* cache, PxSceneQueryFlags hintFlags, bool isBatchQuery)
{
	switch(geometry.getType())
	{
	case PxGeometryType::eBOX:
		{
			const PxBoxGeometry& geom = static_cast<const PxBoxGeometry&>(geometry);
			return linearOBBSweep(pose.p, geom.halfExtents, pose.q, unitDir, distance, nbHits, hits, hasBlockingHit, multipleHits, anyHit, filterFlags, filterCall, filterData, preFilterShader, postFilterShader, constBlock, cBlockSize, queryClient, objects, cache, hintFlags, isBatchQuery);
		}
		break;
	case PxGeometryType::eSPHERE:
		{
			const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geometry);
			Capsule worldCapsule;
			worldCapsule.p0 = worldCapsule.p1 = pose.p;
			worldCapsule.radius = sphereGeom.radius;
			return linearCapsuleSweep(worldCapsule, unitDir, distance, nbHits, hits, hasBlockingHit, multipleHits, anyHit, filterFlags, filterCall, filterData, preFilterShader, postFilterShader, constBlock, cBlockSize, queryClient, objects, cache, hintFlags, isBatchQuery);
		}
		break;
	case PxGeometryType::eCAPSULE:
		{
			Capsule worldCapsule;
			PxCapsuleGeometry geom = static_cast<const PxCapsuleGeometry&>(geometry);
			getWorldCapsule(worldCapsule, geom, pose);
			return linearCapsuleSweep(worldCapsule, unitDir, distance, nbHits, hits, hasBlockingHit, multipleHits, anyHit, filterFlags, filterCall, filterData, preFilterShader, postFilterShader, constBlock, cBlockSize, queryClient, objects, cache, hintFlags, isBatchQuery);
		}
		break;
	default:
		{
			PX_CHECK_MSG(false, "sweep: Invalid geometry type. Only box, capsule and sphere sweeps are supported.");
			return 0;
		}
		break;
	}
}

PxI32 SceneQueryManager::linearGeometrySweep(	const PxGeometry** geometryList, const PxTransform* poseList, const PxFilterData* filterDataList, PxU32 geometryCount,
												const PxVec3& unitDir, const PxReal distance, PxU32 nbHits, PxSweepHit* hits, bool& hasBlockingHit, bool multipleHits,
												bool anyHit, PxU32 filterFlags, PxSceneQueryFilterCallback* filterCall, PxClientID queryClient,
												const SceneQueryShapeData* cache, PxSceneQueryFlags hintFlags)
{
	InlineArray<GeometryUnion, 10> gList;
	gList.resize(geometryCount);
	for(PxU32 i=0; i < geometryCount; i++)
	{
		gList[i].set(*geometryList[i]);
	}

	return linearCompoundGeometrySweep(gList.begin(), poseList, filterDataList, geometryCount, unitDir, distance, filterFlags, nbHits, hits, hasBlockingHit, multipleHits, anyHit, filterCall, NULL, NULL, NULL, 0, queryClient, NULL, NULL, cache, hintFlags, false);
}

// Remove all old hits below maxDist and append the new ones
static PX_FORCE_INLINE PxU32 mergeSweepHits(PxSweepHit* oldHits, PxU32 oldCount, PxSweepHit* newHits, PxU32 newCount, PxReal maxDist)
{
	PxU32 adjustedOldCount = 0;
	for(PxU32 i=0; i < oldCount; i++)
	{
		if (oldHits[i].distance >= maxDist)
			oldHits[i] = oldHits[i+1];
		else
		{
			oldHits++;
			adjustedOldCount++;
		}
	}
	if (adjustedOldCount != oldCount)  // If some of the old touching hits are removed, the new ones need to be shifted
	{
		Ps::memMove(oldHits, newHits, sizeof(PxSweepHit*) * newCount);
	}
	return adjustedOldCount;
}

// Remove hits below maxDist
static  PX_FORCE_INLINE PxU32 checkTouchingSweepHits(PxSweepHit* newHits, PxU32 newCount, PxReal maxDist)
{
	PxU32 acceptedCount = 0;
	for(PxU32 i=0; i < newCount; i++)
	{
		if ((newHits[i].distance >= maxDist) && (i < (newCount-1)))
			newHits[i] = newHits[i+1];
		else
		{
			acceptedCount++;
			newHits++;
		}
	}
	return acceptedCount;
}

PxI32 SceneQueryManager::linearCompoundGeometrySweep(	const GeometryUnion* geometryList, const PxTransform* poseList, const PxFilterData* filterDataList, PxU32 geometryCount,
														const PxVec3& unitDir, const PxReal distance, PxU32 flags, PxU32 nbHits, PxSweepHit* hits, bool& hasBlockingHit,
														bool multipleHits, bool anyHit, PxSceneQueryFilterCallback* filterCall, PxBatchQueryPreFilterShader preFilterShader,
														PxBatchQueryPostFilterShader postFilterShader, const void* constBlock, PxU32 cBlockSize, PxClientID queryClient,
														PrunedObjects* objects, const PxSweepCache* sweepCache, const SceneQueryShapeData* singleObjectCache, PxSceneQueryFlags hintFlags,
														bool isBatchQuery)
{
	// TODO: PT: this is oh-so-ugly... use the SAP ASAP!

	PrunedObjects* prunedObjects = NULL;

	if(sweepCache)
	{
		PxSweepCache* sc = const_cast<PxSweepCache*>(sweepCache);
		ObjectCache* cache = static_cast<ObjectCache*>(sc);

		// - implicit or explicit caches?
		// - cache world triangles? extruded?
		// - or just store PxShape pointers, saving just the broad phase time?
		// - when do we refill the cache?

		// Not updating the dyna cache:
		// - SAP events
		// - testing all moved dynamic objects against cache bounds? Seems not worth it. Maybe we can run a bipartite
		//   SAP between all the cache bounds and all the moved objects??

		// So the questions are actually:
		// - how do we cache? (implicit? explicit?)
		// - when do we cache? (when is the cache invalidated exactly?)
		// - what do we cache? (object pointers? transformed geoms?)

		// More advanced: do we use a single cache/actor, or a continuous shared cache, as for particles?

		// The bounding volumes:
		// - around the actor
		// - around the actor + its motion
		// - from the cache

		// PT: TODO: use Box
		PxVec3 motionOBB_Center;
		PxVec3 motionOBB_Extents;
		PxQuat motionOBB_Rot;
		{
			PxBounds3 objectBounds;
			objectBounds.setEmpty();

			for(PxU32 i=0; i < geometryCount; i++)
			{
				PxVec3 c,e;
				geometryList[i].computeBounds(poseList[i], NULL, c, e);
				objectBounds.include(PxBounds3(c-e, c+e));
			}

			// PT: don't duplicate code! Use existing function!
			Box src;
			src.center = objectBounds.getCenter();
			src.extents = objectBounds.getExtents();
			src.rot = PxMat33::createIdentity();

			Box tmpDest;
			CreateOBB(tmpDest, src, unitDir, distance);

			// PT: TODO: remove useless conversion
			motionOBB_Extents = tmpDest.extents;
			motionOBB_Center = tmpDest.center;

			PxMat33Legacy tmpRot;
			tmpRot.setColumn(0, tmpDest.rot.column0);
			tmpRot.setColumn(1, tmpDest.rot.column1);
			tmpRot.setColumn(2, tmpDest.rot.column2);
			motionOBB_Rot = PxQuat(tmpRot.toPxMat33());
		}


		/*
		bool StaticUpToDate()
		{
		if static object has been created/deleted
		return false
		else
		return true
		}

		bool DynamicUpToDate()
		{
		if dynamic object has been created/deleted/moved
		return false
		else
		return true
		}

		void DoSweep()
		{
		Compute sweep bounds TBV (initial actor pose + full motion)

		// The cache contains some static & dynamic shapes, valid at time T. The sweep test can be called at any time, multiple
		// times for the same actor during the simulation. Each test has a temporal bounding volume TBV.
		//
		// For dynamic objects:
		// - at the start of each frame, the dynamic cache must be recomputed, regardless of the TBV. Indeed, even if the TBV
		//   doesn't move, the dynamic objects themselves may have entered it.
		// - during a given frame, the cache itself is valid, it only needs updating if the TBV becomes invalid
		//
		// For static objects:
		// - nothing to do at start of frame
		// - during a given frame, the cache itself is valid, it only needs updating if the TBV becomes invalid
		//
		// Now everything is covered by two events:
		// - the objects move
		// - the TBV moves
		// We don't need to explicitely have a "start of frame" event, we can just use pose timestamps, even for static
		// objects. Then the TBV "timestamp" is only a loose one, but it's really the same concept. To the point we might also
		// use loose bounds for the objects themselves, leading to very few cache updates. Careful here though, if we store
		// more than pointers (say transformed triangles) we need to update them more often than that.
		//
		// Note that ideally we only need to test the pose timestamps of dynamic objects *inside our cache*. If the cache
		// doesn't contain any dynamic objects....... wait, that's BS, new external objects may have entered our cache... So let's
		// just recompute the dynamic part "all the time".

		// So, there:

		if(!DynamicUpToDate())
		{
		Update dynamic cache	// regardless of bounds
		}

		if(!StaticUpToDate() || cache does not contain TBV)
		{
		Update static cache
		}

		Sweep against cache

		// We need a "findTouchedGeometry" function that *does not* update the cache with query data, to be more CCT-friendly.
		// The bounds used for the dynamic update are *not* inflated since we redo them all the time anyway. This is a bit subtle,
		// see the CCT update.
		}
		*/

		if(!cache->implicit)
		{
			// Fully explicit, don't do anything anymore
			prunedObjects = &cache->cachedObjects;
		}
		else
		{
			updateCache(*cache, motionOBB_Center, motionOBB_Extents, motionOBB_Rot, NULL, NULL, NULL);

			// We don't want to copy the "no cache" approach because it's too slow: we don't want a scene query for each subshape,
			// we want a single scene query (which is actually our cache), and sweep subshapes against it.

			prunedObjects = &cache->cachedObjects;
		}
	}


	const bool passForeignShapes = (getScene().getClientBehaviorBits(queryClient) & PxClientBehaviorBit::eREPORT_FOREIGN_OBJECTS_TO_SCENE_QUERY) != 0;


	bool dummy;
	if(multipleHits)
	{
		/*
		How do we implement this ?

		Allocating enough space for the maximum number of hits is not enough, since *each geometry object* can potentially collide
		with *each shapes* ! So that would be the total #shapes in scenes * the total #geometries of the query, way too much!

		Could we use their callback/data directly ? No, because the "sweepShape" field is not setup by the scene level
		queries, so we have to patch this before calling the user!! Or we can pass this as a param to the scene function,
		somehow.

		Let's analyze the different cases:
		1) using a fixed-size buffer: the user wants the results in a fixed size buffer, big enough for N hits. When we
		sweep a subshape, it might report M hits already, with M<=N. For next subshapes, we can't use the full buffer
		anymore so we have to adjust the buffer params. When the buffer is full, we can early exit.

		2) using a callback: nothing to do here, it works automatically

		3) using a callback with custom memory: should work too, as the buffer is internally recycled and reused for
		each shape already. So there's no difference when we move to the next subshape, it's just recycled again.
		*/

		//
		// The way this is implemented might not be the best (depending on the use case scenario). Could combine the bounds of all geometries and use the swept representation to get the set of
		// potentially overlapping PrunedObjects once, then re-use the list for all per-geometry checks.
		//

		PxU32 totalNbHits = 0;  // NOT including the blocking hit
		PxReal curDist = distance;
		PxSweepHit blockingHit;
		blockingHit.distance = PX_MAX_F32;
		PxU32 hasBlockingHitInt = 0;
		PxSweepHit* hitBuffer = hits;
		PxU32 hitBufferSize = nbHits;
		PxU32 newHitsStartIdx = 0;
		for(PxU32 i=0; i < geometryCount; i++)
		{
			const PxGeometry& currentGeometry = geometryList[i].get();
			const PxFilterData* filterData = filterDataList ? &filterDataList[i] : NULL;
			
			PxI32 nbHitsCurrentGeometry;
			do
			{
				bool curHasBlockingHit;
				nbHitsCurrentGeometry = linearGeometrySweep(currentGeometry, poseList[i], unitDir, curDist, hitBufferSize-totalNbHits, hitBuffer+totalNbHits, curHasBlockingHit, true, false, flags, filterCall, filterData, preFilterShader, postFilterShader, constBlock, cBlockSize, queryClient, prunedObjects, singleObjectCache, hintFlags, isBatchQuery);
				if (nbHitsCurrentGeometry != 0)
				{
					PxU32 blockingIdx, newCount;
					if (nbHitsCurrentGeometry > 0)
					{
						blockingIdx = totalNbHits + nbHitsCurrentGeometry - 1;
						newCount = nbHitsCurrentGeometry;
					}
					else
					{
						// Buffer overflow: Grow the buffer to cover all objects
						PxU32 newMaxCount = hitBufferSize * 2;
						PxSweepHit* newBuffer = (PxSweepHit*)physx::shdfnd::TempAllocator().allocate(sizeof(PxSweepHit) * newMaxCount, __FILE__, __LINE__);

						if (newBuffer)
						{
							Ps::memCopy(newBuffer, hitBuffer, sizeof(PxSweepHit) * hitBufferSize);
							
							if (hitBuffer != hits)
								physx::shdfnd::TempAllocator().deallocate(hitBuffer);

							hitBuffer = newBuffer;
							hitBufferSize = newMaxCount;

							continue;
						}
						else
						{
							physx::shdfnd::getFoundation().error(physx::PxErrorCode::eOUT_OF_MEMORY, __FILE__, __LINE__, "sweep query: Memory allocation failed. Incomplete list of hits should be expected.");
						}

						blockingIdx = hitBufferSize - 1;
						newCount = hitBufferSize - totalNbHits;
					}
					
					if (curHasBlockingHit)
					{
						hasBlockingHitInt = 1;
						PxSweepHit* curBlockingHit = &hitBuffer[blockingIdx];
						PX_ASSERT(curBlockingHit->distance <= blockingHit.distance);
						curDist = curBlockingHit->distance;
						blockingHit = *curBlockingHit;
						blockingHit.sweepGeometryIndex = i;

						newHitsStartIdx = mergeSweepHits(hitBuffer, totalNbHits, (hitBuffer+totalNbHits), newCount-1, curDist);  // -1 because the blocking hit can be discarded since we keep a local copy
						totalNbHits = newHitsStartIdx + newCount - 1;
					}
					else if (hasBlockingHitInt)
					{
						newCount = checkTouchingSweepHits((hitBuffer+totalNbHits), newCount, blockingHit.distance);
						totalNbHits += newCount;
					}
					else
						totalNbHits += newCount;

					for(PxU32 j=newHitsStartIdx; j < newHitsStartIdx+newCount; j++)
						hitBuffer[j].sweepGeometryIndex = i;

					newHitsStartIdx = totalNbHits;
				}
			}
			while(nbHitsCurrentGeometry < 0);  // The loop is not great but more than one go should not be the case usually.
		}

		PxI32 returnValue;
		if ((totalNbHits + hasBlockingHitInt) <= nbHits)
			returnValue = totalNbHits + hasBlockingHitInt;
		else
		{
			returnValue = -1;
			totalNbHits = nbHits - hasBlockingHitInt;
		}

		if (hitBuffer != hits)
		{
			Ps::memCopy(hits, hitBuffer, sizeof(PxSweepHit) * totalNbHits);  // Write hit information back if the user buffer did not have enough space for the temporary results
			physx::shdfnd::TempAllocator().deallocate(hitBuffer);
		}

		if (hasBlockingHitInt > 0)
		{
			hits[totalNbHits] = blockingHit;
			hasBlockingHit = true;
		}
		else
			hasBlockingHit = false;

		return returnValue;
	}
	else if (!anyHit)
	{
		PX_ASSERT(hits);

		PxSweepHit closestHit;
		closestHit.distance = PX_MAX_F32;
		PxReal curDist = distance;

		// Loop through shapes in actors, sweep each shape against scene, keep closest hit
		for(PxU32 i=0; i < geometryCount; i++)
		{
			const PxGeometry& currentGeometry = geometryList[i].get();
			const PxFilterData* filterData = filterDataList ? &filterDataList[i] : NULL;

			PxSweepHit closestSubHit;	// Closest hit for current geometry object

			PxI32 nbHitsCurrentGeometry = linearGeometrySweep(currentGeometry, poseList[i], unitDir, curDist, 1, &closestSubHit, dummy, false, false, flags, filterCall, filterData, preFilterShader, postFilterShader, constBlock, cBlockSize, queryClient, prunedObjects, singleObjectCache, hintFlags, isBatchQuery);

			// If current geoemtry object hit something, and if that hit is closer than best one, keep it
			if(nbHitsCurrentGeometry && closestSubHit.distance<closestHit.distance)
			{
				closestHit						= closestSubHit;
				closestHit.sweepGeometryIndex	= i;
				curDist							= closestSubHit.distance;
			}
		}

		// Report closest hit, if exists
		if(closestHit.distance != PX_MAX_F32)
		{
			*hits = closestHit;
			return 1;
		}
	}
	else
	{
		PX_ASSERT(hits);

		// Loop through shapes in actors, sweep each shape against scene
		for(PxU32 i=0; i < geometryCount; i++)
		{
			const PxGeometry& currentGeometry = geometryList[i].get();
			const PxFilterData* filterData = filterDataList ? &filterDataList[i] : NULL;

			if (linearGeometrySweep(currentGeometry, poseList[i], unitDir, distance, 1, hits, dummy, false, true, flags, filterCall, filterData, preFilterShader, postFilterShader, constBlock, cBlockSize, queryClient, prunedObjects, singleObjectCache, hintFlags, isBatchQuery) > 0)
			{
				hits[0].sweepGeometryIndex = i;
				return 1;
			}
		}
	}

	return 0;
}

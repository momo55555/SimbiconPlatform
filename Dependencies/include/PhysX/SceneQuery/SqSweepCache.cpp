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


#include "CmPhysXCommon.h"
#include "SqSweepCache.h"
#include "PxBoxGeometry.h"
#include "SqSceneQueryManager.h"
#include "SqUtilities.h"
#include "NpShape.h"
#include "GuBoxConversion.h"
#include "GuOverlapTests.h"

using namespace physx;
using namespace Sq;
using namespace Gu;

ObjectCache::ObjectCache(SceneQueryManager* owner, PxReal dims) : cachedObjects(cachedObjectsContainer)
{
	sceneQueryManager = owner;

	nbStatic = 0;
	dimensions = dims;

	// PT: TODO: setEmpty call on Gu::Box
	cachedVolume_Center = PxVec3(0.0f, 0.0f, 0.0f);
	cachedVolume_Extents = PxVec3(-PX_MAX_REAL, -PX_MAX_REAL, -PX_MAX_REAL);
	cachedVolume_Rot = PxQuat::createIdentity();

	implicit = true;
}


ObjectCache::~ObjectCache()
{
}


void ObjectCache::setVolume(const PxBoxGeometry& boxGeom, const PxTransform& pose)
{
	PX_CHECK_VALID(pose);

	// PT: the lock is in the updateCache function
	implicit	= false;
	sceneQueryManager->updateCache(*this,
									pose.p, boxGeom.halfExtents, pose.q,
									pose.p, boxGeom.halfExtents, pose.q,
									true, true);
}


void ObjectCache::release()
{
	sceneQueryManager->releaseSweepCache(this);
}


PxReal ObjectCache::getDimensions()
{
	return dimensions;
}


// PT: I don't know what to expose from Scene exactly, so let's do all the cache management here for the moment.
// PT: TODO: use Gu::Box in the interfaces already, instead of converting to this all the time
static bool isContained(const PxVec3& center_a, const PxVec3& extents_a, const PxQuat& rot_a,
						const PxVec3& center_b, const PxVec3& extents_b, const PxQuat& rot_b
						)
{
	Gu::Box tmp0, tmp1;
	buildFrom(tmp0, center_a, extents_a, rot_a);
	buildFrom(tmp1, center_b, extents_b, rot_b);
	return tmp0.isInside(tmp1)!=0;
}


void SceneQueryManager::updateCache(ObjectCache& cache,
										const PxVec3& worldBox_Center, const PxVec3& worldBox_Extents, const PxQuat& worldBox_Rot,
										const PxVec3* cacheBox_Center, const PxVec3* cacheBox_Extents, const PxQuat* cacheBox_Rot
										)
{
	// ### check filter changes
	// should we cache filtered or non-filtered objects anyway? it's more efficient to cache already filtered stuff,
	// but how do we detect filter changes? i.e. when filters are changed for *objects in the cache* ? It seems we
	// don't have a choice here, we must cache the non-filtered objects.

	bool updateStatic = false;
	bool updateDynamic = false;

	// The bounds are invalid if the motion's box is not included in cached volume. When this happens,
	// the cache has to be fully regenerated (static & dynamic parts)
	const bool invalidBounds = !isContained(	worldBox_Center, worldBox_Extents, worldBox_Rot,
												cache.cachedVolume_Center, cache.cachedVolume_Extents, cache.cachedVolume_Rot);
	if(invalidBounds)
	{
		updateStatic	= true;
		updateDynamic	= true;
	}

	// Now, even when the bounds are valid, we must still invalidate the cache when objects have
	// been moved and/or added and/or deleted. We run the following code all the time, even when
	// we already know the cache must be updated, so that the stored signature are up-to-date.
	{
		const Pruner* staticPruner = getStaticPruner();
		if(staticPruner)
		{
			const Signature& ss = staticPruner->GetSignature();
			if(cache.ss!=ss)
			{
				updateStatic = true;
				updateDynamic = true;	// Because we can't easily just update the static part in the cache => update all then
				cache.ss = ss;
			}
		}
	}
	{
		// PT: looks like the signature for the dynamic pruner is always invalid, even when objects don't move. Grrr!
		const Pruner* dynamicPruner = getDynamicPruner();
		if(dynamicPruner)
		{
			const Signature& sd = dynamicPruner->GetSignature();
			if(cache.sd!=sd)
			{
				updateDynamic = true;
				cache.sd = sd;
			}
		}
	}

	if(!updateStatic && !updateDynamic)
		return;	// Cache still valid, don't update at all

	// Figure out new cached volume
	PxVec3 cachedVolume_Center;
	PxVec3 cachedVolume_Extents;
	PxQuat cachedVolume_Rot;

	if(cacheBox_Center && cacheBox_Extents && cacheBox_Rot)
	{
		// Explicit cached volume
		PX_ASSERT(isContained(worldBox_Center, worldBox_Extents, worldBox_Rot,
							*cacheBox_Center, *cacheBox_Extents, *cacheBox_Rot));
		cachedVolume_Center = *cacheBox_Center;
		cachedVolume_Extents = *cacheBox_Extents;
		cachedVolume_Rot = *cacheBox_Rot;
	}
	else
	{
		// Implicit cached volume
		cachedVolume_Center = worldBox_Center;
		cachedVolume_Extents = worldBox_Extents;
		cachedVolume_Rot = worldBox_Rot;

		const PxReal dimensions = cache.getDimensions();
		cachedVolume_Extents.x += dimensions;
		cachedVolume_Extents.y += dimensions;
		cachedVolume_Extents.z += dimensions;
	}

	updateCache(cache,
				cachedVolume_Center, cachedVolume_Extents, cachedVolume_Rot,
				worldBox_Center, worldBox_Extents, worldBox_Rot,
				updateStatic, updateDynamic);
}

// PT: TODO: replace this hook with something better. The dynamic array has to go eventually.
static bool gCacheFillCallback(Prunable** prunables, PxU32 nb, void* userData)
{
	PrunedObjects* cachedObjects = (PrunedObjects*)userData;
	while(nb--)
		cachedObjects->AddPrunable(*prunables++);
	return true;
}

void SceneQueryManager::updateCache(ObjectCache& cache,
										const PxVec3& cachedVolume_Center, const PxVec3& cachedVolume_Extents, const PxQuat& cachedVolume_Rot,
										const PxVec3& worldBox_Center, const PxVec3& worldBox_Extents, const PxQuat& worldBox_Rot,
										bool updateStatic, bool updateDynamic)
{
	// If we must update the static part, then we must update everything => reset cache
	// If we must update the dynamic part, then keep the static part alive
	if(updateStatic)
		cache.cachedObjects.ResetObjects();
	else if(updateDynamic)
		cache.cachedObjectsContainer.ForceSize(cache.nbStatic);	// #static from previous call, since we didn't compute it here

	// Perform static update - no filtering
	if(updateStatic)
	{
		Gu::Box tmp;
		buildFrom(tmp, cachedVolume_Center, cachedVolume_Extents, cachedVolume_Rot);
		overlap(gCacheFillCallback, &cache.cachedObjects, tmp, PxSceneQueryFilterFlag::eSTATIC);
	}
	// Keep track of number of static objects in the cache
	PxU32 nbStatic = cache.cachedObjects.GetNbPrunables();

	// ### USELESS CONVERSION - we should eventually directly pass a Gu::Box to those functions
	Gu::Box worldBox;
	buildFrom(worldBox, worldBox_Center, worldBox_Extents, worldBox_Rot);

	// Perform dynamic update - no filtering
	if(updateDynamic)
	{
		// We are going to update this part all the time when objects move anyway, so while we're at it we can fill the cache
		// with the actual motion bounds, instead of the (bigger) cached volume.......
		overlap(gCacheFillCallback, &cache.cachedObjects, worldBox, PxSceneQueryFilterFlag::eDYNAMIC);
	}

	// We can't do the filtering here so we only perform the more accurate overlap tests
	// It is unclear whether this is really needed. We could very well run them all the time instead, in the sweep tests.
	PxU32 nbObjects = cache.cachedObjects.GetNbPrunables();
	Prunable** current = cache.cachedObjects.GetPrunables();

	// IMPORTANT: Take care when changing the following loop. We read from and write to the same buffer!
	cache.cachedObjects.ResetObjects();
	PxU32 nbStaticFiltered = 0;
	for(PxU32 i=0;i<nbObjects;i++)
	{
		Prunable* p = *current++;
		SceneQueryShapeData* currentShape = static_cast<SceneQueryShapeData*>(p);

		// For some reasons the shapes are not filtered here. The reason might have been the following scenario:
		// If shapes were filtered before added to the cache, there could be misses when the cached shapes are accessed next time
		// since filtering properties may have changed.
		// However, filtering changes declare the dynamic/static pruning structures dirty which then should trigger
		// a cache update. So, filtering would seem valid at this point. There is one remaining case: Explicit caches.
		// These ignore the pruning structure dirty flags and then the mentioned troublesome scenario kicks in. Although
		// one could argue that in the case of explicit caches, it is the users responsibility to refresh the cache.

		const GeomOverlapOBBFunc overlap = gGeomOverlapOBBMap[currentShape->getGeometryType()];
		if(!overlap(currentShape->shapeGeometryEA->get(), currentShape->sqGlobalPose, worldBox))
			continue;

		// If we made a full update, updateStatic==true, and we must count the number of non-filtered static objects
		// that we are going to add to the cache.
		if(i<nbStatic)
			nbStaticFiltered++;

		cache.cachedObjects.AddPrunable(p);
	}

	// Save query info
	cache.cachedVolume_Center	= cachedVolume_Center;
	cache.cachedVolume_Extents	= cachedVolume_Extents;
	cache.cachedVolume_Rot		= cachedVolume_Rot;
	cache.nbStatic				= nbStaticFiltered;
}

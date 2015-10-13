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
#include "SqUtilities.h" 
#include "SqFiltering.h" 

#include "GuRaycastTests.h"

using namespace physx;
using namespace Ice;
using namespace Sq;

/*
Raycast design issues:

We want to expose individual raycast functions in Objects, so that the user can raycast against a single shape if needed.
However the "groups" param is not taken into account out there. It's done in this file. But if we move the test to all
raycasting functions, along with the "disable" flag test, it means this filtering will suddenly be enabled for internal
usages too, like CCD. Not good.

All the filtering should just be moved outside, and the "groups" param removed from the interface. Since it was only used
by compounds, it shouldn't be needed anymore now. Hell, let's do it.
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define PRE_TEST_FILTERING_CODE(params, currentShape, outFilterFlags, outHitType, defaultHitType)							\
																															\
PxSceneQueryHitType::Enum outHitType = defaultHitType;																		\
PxSceneQueryFilterFlags outFilterFlags;																						\
																															\
if (params->isBatchQuery)																									\
{																															\
	PX_ASSERT(!params->nonBatchedFilter);																					\
	if (applyBatchedPreFilterPreTest(params->preFilterShader, *params->filterData, params->constBlock, params->cBlockSize,	\
	    params->filterFlags, *currentShape, outFilterFlags, outHitType)) return STAB_CONTINUE;								\
}																															\
else																														\
{																															\
	if (applyNonBatchedFilterPreTest(params->nonBatchedFilter, *params->filterData, params->filterFlags, *currentShape,		\
		outFilterFlags, outHitType)) return STAB_CONTINUE;																	\
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	struct RaycastAnyParams
	{
		PxVec3							rayOrigin;
		PxVec3							rayDir;
		PxReal							rayLength;
		PxSceneQueryFilterCallback*		nonBatchedFilter;
		PxU32							filterFlags;
		const PxFilterData*				filterData;
		PxBatchQueryPreFilterShader			preFilterShader;
		PxBatchQueryPostFilterShader			postFilterShader; 
		const void*						constBlock;
		PxU32							cBlockSize;
		PxClientID						queryClient;			
		bool							passForeignShapes;
		SceneQueryShapeData*			hitShape;
		PxU32							faceIndex;
		SceneQueryShapeData*			cachedShape;
		bool							isBatchQuery;
	};

// PT: please keep this close to the calling code if possible
static PxU32 raycastAnyCallback(const Prunable* p, float& max_dist, void* user_data)
{
	RaycastAnyParams* params = (RaycastAnyParams*)user_data;

	const SceneQueryShapeData* currentShape = static_cast<const SceneQueryShapeData*>(p);

	if(currentShape==params->cachedShape)  // If there is a cached shape and it gets hit by the ray, then we can skip the test because cached shapes are tested in advance (and since we got here no hit has been found)
		return STAB_CONTINUE;

	if (applyClientFilter(params->queryClient, params->passForeignShapes, *currentShape))
		return STAB_CONTINUE;

	PRE_TEST_FILTERING_CODE(params, currentShape, filterFlags, hitType, PxSceneQueryHitType::eBLOCK)

	if (hitType == PxSceneQueryHitType::eNONE || hitType == PxSceneQueryHitType::eTOUCH)
		return STAB_CONTINUE;

	PxRaycastHit subHit;
	const Gu::RaycastFunc& raycast = Gu::gRaycastMap[currentShape->shapeGeometryEA->getType()];
	const PxU32 nbHits = raycast(currentShape->shapeGeometryEA->get(), currentShape->sqGlobalPose, params->rayOrigin, params->rayDir, params->rayLength, (PxSceneQueryFlags)0, 1, &subHit, true);
	if(nbHits)
	{
		subHit.shape = currentShape->shape;

		if(params->isBatchQuery)
			hitType = applyBatchedPostFilterTest(params->postFilterShader, *params->filterData, params->constBlock, params->cBlockSize,
															filterFlags, *currentShape, subHit, hitType);
		else
			hitType = applyNonBatchedFilterPostTest(params->nonBatchedFilter, filterFlags, *params->filterData, subHit, hitType);

		if (hitType == PxSceneQueryHitType::eNONE || hitType == PxSceneQueryHitType::eTOUCH)
			return STAB_CONTINUE;

		PX_ASSERT(hitType == PxSceneQueryHitType::eBLOCK);
		params->hitShape = const_cast<SceneQueryShapeData*>(currentShape);
		params->faceIndex = subHit.faceIndex;
		return STAB_STOP;
	}
	return STAB_CONTINUE;
}

bool SceneQueryManager::raycastAny(const PxVec3& rayOrigin, const PxVec3& rayDir, PxReal rayLength, PxSceneQueryHit& hit, 
									   PxU32 filterFlags, PxSceneQueryFilterCallback* filterCall, const PxFilterData* filterData, PxBatchQueryPreFilterShader preFilterShader, PxBatchQueryPostFilterShader postFilterShader, const void* constBlock, PxU32 cBlockSize, 
									   PxClientID queryClient, SceneQueryShapeData* cache, bool isBatchQuery)
{
	PX_CHECK_AND_RETURN_NULL(PxAbs(rayDir.magnitudeSquared()-1)<1e-4, "ray direction not valid: must be unit vector.");

	// Handle cache if needed
	if(cache)
	{
		// No filtering for cached shape
		const Gu::RaycastFunc& raycast = Gu::gRaycastMap[cache->shapeGeometryEA->getType()];
		PxRaycastHit tmpHit;
		if(raycast(cache->shapeGeometryEA->get(), cache->sqGlobalPose, rayOrigin, rayDir, rayLength, (PxSceneQueryFlags)0, 1, &tmpHit, true))
		{
			hit.shape = cache->shape;
			hit.faceIndex = tmpHit.faceIndex;
			return true;
		}
	}

	const bool passForeignShapes = (getScene().getClientBehaviorBits(queryClient) & PxClientBehaviorBit::eREPORT_FOREIGN_OBJECTS_TO_SCENE_QUERY) != 0;

	RaycastAnyParams asp;
	asp.rayOrigin			= rayOrigin;
	asp.rayDir				= rayDir;
	asp.rayLength			= rayLength;
	asp.nonBatchedFilter	= filterCall;
	asp.filterFlags			= filterFlags;
	asp.filterData			= filterData;
	asp.preFilterShader		= preFilterShader;
	asp.postFilterShader	= postFilterShader;
	asp.constBlock			= constBlock;
	asp.cBlockSize			= cBlockSize;
	asp.queryClient			= queryClient;			
	asp.passForeignShapes	= passForeignShapes;
	asp.hitShape			= NULL;
	asp.faceIndex			= 0;
	asp.cachedShape			= cache;
	asp.isBatchQuery		= isBatchQuery;

	// PT: TODO: reactivate pruning groups!!!
	stab(raycastAnyCallback, &asp, rayOrigin, rayDir, rayLength, filterFlags);

	if(asp.hitShape)
	{
		hit.shape = asp.hitShape->shape;
		hit.faceIndex = asp.faceIndex;
		return true;
	}
	else
		return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	struct RaycastSingleParams
	{
		PxVec3							rayOrigin;
		PxVec3							rayDir;
		// The ray length gets stored in hit.distance becasue that's where it gets updated anyway when the ray needs to shrink
		PxRaycastHit*					hit;
		PxSceneQueryFlags				hintFlags;
		PxSceneQueryFilterCallback*		nonBatchedFilter;
		PxU32							filterFlags;
		const PxFilterData*				filterData;
		PxBatchQueryPreFilterShader			preFilterShader;
		PxBatchQueryPostFilterShader			postFilterShader; 
		const void*						constBlock;
		PxU32							cBlockSize;
		PxClientID						queryClient;			
		bool							passForeignShapes;
		SceneQueryShapeData*			cachedShape;
		bool							isBatchQuery;
	};

// PT: please keep this close to the calling code if possible
static PxU32 raycastSingleCallback(const Prunable* p, float& max_dist, void* user_data)
{
	RaycastSingleParams* params = (RaycastSingleParams*)user_data;

	const SceneQueryShapeData* currentShape = static_cast<const SceneQueryShapeData*>(p);

	if(currentShape==params->cachedShape)  // If there is a cached shape, then we can skip the test because cached shapes are tested in advance
		return STAB_CONTINUE;

	if (applyClientFilter(params->queryClient, params->passForeignShapes, *currentShape))
		return STAB_CONTINUE;

	PRE_TEST_FILTERING_CODE(params, currentShape, filterFlags, hitType, PxSceneQueryHitType::eBLOCK)

	if (hitType == PxSceneQueryHitType::eNONE || hitType == PxSceneQueryHitType::eTOUCH)
		return STAB_CONTINUE;

	PxRaycastHit subHit;
	const Gu::RaycastFunc& raycast = Gu::gRaycastMap[currentShape->shapeGeometryEA->getType()];
	if(!raycast(currentShape->shapeGeometryEA->get(), currentShape->sqGlobalPose, params->rayOrigin, params->rayDir, params->hit->distance, (params->hintFlags | PxSceneQueryFlag::eDISTANCE), 1, &subHit, false))  // We need the distance to shrink the ray
		return STAB_CONTINUE;

	PX_ASSERT(subHit.flags & PxSceneQueryFlag::eDISTANCE);	// There used to be some logic which implicitly assumed that either impact or distance info was always available. 
															// This is now explicitly requested and since all raycast routines seem to compute the distance anyway, the distance is chosen.

	if(subHit.distance <= params->hit->distance)
	{
		subHit.shape = currentShape->shape;

		if(params->isBatchQuery)
			hitType = applyBatchedPostFilterTest(params->postFilterShader, *params->filterData, params->constBlock, params->cBlockSize,
															filterFlags, *currentShape, subHit, hitType);
		else
			hitType = applyNonBatchedFilterPostTest(params->nonBatchedFilter, filterFlags, *params->filterData, subHit, hitType);

		if (hitType == PxSceneQueryHitType::eNONE || hitType == PxSceneQueryHitType::eTOUCH)
			return STAB_CONTINUE;

		subHit.flags |= PxSceneQueryFlag::eBLOCKING_HIT;

		*params->hit = subHit;
		PX_ASSERT(params->hit->shape == currentShape->shape);

		// Shrink the segment
		max_dist = subHit.distance;
		return STAB_UPDATE_MAX_DIST;
	}
	return STAB_CONTINUE;
}

bool SceneQueryManager::raycastSingle(const PxVec3& rayOrigin, const PxVec3& rayDir, PxReal rayLength, PxRaycastHit& hit, PxSceneQueryFlags hintFlags, 
										  PxU32 filterFlags, PxSceneQueryFilterCallback* filterCall, const PxFilterData* filterData, PxBatchQueryPreFilterShader preFilterShader, PxBatchQueryPostFilterShader postFilterShader, const void* constBlock, PxU32 cBlockSize, 
										  PxClientID queryClient, SceneQueryShapeData* cache, bool isBatchQuery)
{
	PX_CHECK_AND_RETURN_NULL(PxAbs(rayDir.magnitudeSquared()-1)<1e-4, "ray direction not valid: must be unit vector.");

	hit.shape			= NULL;  // "NULL" is a valid result
	hit.flags			= PxSceneQueryFlags(0);
	hit.distance		= rayLength;
	hit.faceIndex		= 0;

	// Handle cache if needed
	if(cache)
	{
		// No filtering for cached shape
		const Gu::RaycastFunc& raycast = Gu::gRaycastMap[cache->shapeGeometryEA->getType()];
		if (raycast(cache->shapeGeometryEA->get(), cache->sqGlobalPose, rayOrigin, rayDir, hit.distance, hintFlags|PxSceneQueryFlag::eDISTANCE, 1, &hit, false) != 0)  // compute distance to shrink the ray if the cached shape hits
		{
			PX_ASSERT(hit.flags & PxSceneQueryFlag::eDISTANCE);
			hit.shape = cache->shape;
		}
		else
			hit.flags = PxSceneQueryFlags(0);
	}

	const bool passForeignShapes = (getScene().getClientBehaviorBits(queryClient) & PxClientBehaviorBit::eREPORT_FOREIGN_OBJECTS_TO_SCENE_QUERY) != 0;

	RaycastSingleParams csp;
	csp.rayOrigin		= rayOrigin;
	csp.rayDir			= rayDir;
	csp.hit				= &hit;
	csp.hintFlags		= hintFlags;
	csp.nonBatchedFilter= filterCall;
	csp.filterFlags		= filterFlags;
	csp.filterData		= filterData;
	csp.preFilterShader = preFilterShader;
	csp.postFilterShader= postFilterShader;
	csp.constBlock		= constBlock;
	csp.cBlockSize		= cBlockSize;
	csp.queryClient		= queryClient;			
	csp.passForeignShapes = passForeignShapes;
	csp.cachedShape		= cache;
	csp.isBatchQuery	= isBatchQuery;

	// PT: TODO: reactivate pruning groups!!!
	stab(raycastSingleCallback, &csp, rayOrigin, rayDir, hit.distance, filterFlags);

	return (hit.shape != NULL);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	struct RaycastMultipleParams
	{
		PxVec3							rayOrigin;
		PxVec3							rayDir;
		// The ray length gets stored in hit.distance because that's where it gets updated anyway when the ray needs to shrink
		PxRaycastHit*					blockingHit;
		PxSceneQueryFlags				hintFlags;
		PxSceneQueryFilterCallback*		nonBatchedFilter;
		PxU32							filterFlags;
		const PxFilterData*				filterData;
		PxBatchQueryPreFilterShader			preFilterShader;
		PxBatchQueryPostFilterShader			postFilterShader; 
		const void*						constBlock;
		PxU32							cBlockSize;
		PxClientID						queryClient;			
		bool							passForeignShapes;
		PxU32							nbHits;  // Number of hits NOT including the blocking one
		PxRaycastHit*					hits;
		PxU32							maxHitsNb;
		SceneQueryShapeData*			cachedShape;
		bool							isBatchQuery;
		bool							userHitBufferAsTmpBuffer;  // Specifies whether hits points to the user buffer or is temporary allocated memory
	};

// PT: please keep this close to the calling code if possible
static PxU32 raycastMultipleCallback(const Prunable* p, float& max_dist, void* user_data)
{
	RaycastMultipleParams* params = (RaycastMultipleParams*)user_data;

	const SceneQueryShapeData* currentShape = static_cast<const SceneQueryShapeData*>(p);

	if(currentShape==params->cachedShape)  // If there is a cached shape, then we can skip the test because cached shapes are tested in advance
		return STAB_CONTINUE;

	if (applyClientFilter(params->queryClient, params->passForeignShapes, *currentShape))
		return STAB_CONTINUE;

	PRE_TEST_FILTERING_CODE(params, currentShape, filterFlags, hitType, PxSceneQueryHitType::eTOUCH)

	if (hitType == PxSceneQueryHitType::eNONE)
		return STAB_CONTINUE;

	const Gu::RaycastFunc& raycast = Gu::gRaycastMap[currentShape->shapeGeometryEA->getType()];

	if(1)
	{
		// Single hit codepath

		PxRaycastHit hit;
		if(!raycast(currentShape->shapeGeometryEA->get(), currentShape->sqGlobalPose, params->rayOrigin, params->rayDir, params->blockingHit->distance, params->hintFlags|PxSceneQueryFlag::eDISTANCE, 1, &hit, false))  // compute distance to shrink the ray or compare distances on new blocking hit
			return STAB_CONTINUE;

		hit.shape = currentShape->shape;

		if(params->isBatchQuery)
			hitType = applyBatchedPostFilterTest(params->postFilterShader, *params->filterData, params->constBlock, params->cBlockSize,
															filterFlags, *currentShape, hit, hitType);
		else
			hitType = applyNonBatchedFilterPostTest(params->nonBatchedFilter, filterFlags, *params->filterData, hit, hitType);

		if (hitType == PxSceneQueryHitType::eNONE)
			return STAB_CONTINUE;

		if (hitType == PxSceneQueryHitType::eTOUCH)
		{
			PX_ASSERT(!(hit.flags & PxSceneQueryFlag::eBLOCKING_HIT));
			hit.flags |= PxSceneQueryFlag::eTOUCHING_HIT;

			if (params->nbHits < params->maxHitsNb)
			{
				params->hits[params->nbHits] = hit;
				params->nbHits++;
			}
			else
			{
				if (params->isBatchQuery)
				{
					params->nbHits = params->maxHitsNb + 1;
					return STAB_STOP;	//!!!SQ  For now until batched queries adopt the new scheme. Non-batched don't abort, they just grow the temporary buffer and continue.
										//       Only at the end, when the data should be written back to the user buffer, it will be decided whether an overflow occured.
				}

				// Buffer overflow: Grow the buffer to cover all objects
				PxU32 newMaxCount = params->maxHitsNb * 2;
				PxRaycastHit* newBuffer = (PxRaycastHit*)physx::shdfnd::TempAllocator().allocate(sizeof(PxRaycastHit) * newMaxCount, __FILE__, __LINE__);

				if (newBuffer)
				{
					Ps::memCopy(newBuffer, params->hits, sizeof(PxRaycastHit) * params->maxHitsNb);
					
					if (!params->userHitBufferAsTmpBuffer)  // This is not the first time the buffer had to be resized -> the current buffer is temp memory
						physx::shdfnd::TempAllocator().deallocate(params->hits);

					params->hits = newBuffer;
					params->maxHitsNb = newMaxCount;

					params->userHitBufferAsTmpBuffer = false;

					params->hits[params->nbHits] = hit;
					params->nbHits++;
				}
				else
				{
					physx::shdfnd::getFoundation().error(physx::PxErrorCode::eOUT_OF_MEMORY, __FILE__, __LINE__, "raycastMultiple: Memory allocation failed. Incomplete list of hits should be expected.");
				}
			}
		}
		else if (hit.distance <= params->blockingHit->distance)
		{
			PX_ASSERT(hitType == PxSceneQueryHitType::eBLOCK);
			PX_ASSERT(hit.flags & PxSceneQueryFlag::eDISTANCE);

			PX_ASSERT(!(hit.flags & PxSceneQueryFlag::eTOUCHING_HIT));
			hit.flags |= PxSceneQueryFlag::eBLOCKING_HIT;

			*params->blockingHit = hit;
			PX_ASSERT(params->blockingHit->shape == currentShape->shape);

			// Shrink the segment
			max_dist = hit.distance;

			PxU32 currentIdx = 0;
			for(PxU32 i=0; i < params->nbHits; i++)
			{
				PX_ASSERT(params->hits[i].flags & PxSceneQueryFlag::eDISTANCE);
				if (params->hits[i].distance <= hit.distance)
				{
					params->hits[currentIdx] = params->hits[i];
					currentIdx++;
				}
			}
			params->nbHits = currentIdx;

			return STAB_UPDATE_MAX_DIST;
		}

		return STAB_CONTINUE;
	}
	else
	{
		// Multiple hits codepath

		PxRaycastHit hits[32];
		PxU32 nbHits =  raycast(currentShape->shapeGeometryEA->get(), currentShape->sqGlobalPose, params->rayOrigin, params->rayDir, params->blockingHit->distance, params->hintFlags, 32, hits, false);
		if(!nbHits)
			return STAB_CONTINUE;
		
		for(PxU32 i=0;i<nbHits;i++)
		{
			hits[i].shape = currentShape->shape;

			//Increase here because the codes outside can check if nbHits>maxHit to find if result buffer is full 
			params->nbHits++;		

			if( params->nbHits > params->maxHitsNb)
				return STAB_STOP;
			else
				params->hits[params->nbHits-1] = hits[i];
		}
	}

	return STAB_CONTINUE;
}

PxI32 SceneQueryManager::raycastMultiple(const PxVec3& rayOrigin, const PxVec3& rayDir, PxReal rayLength, PxRaycastHit* hits, const PxU32 maxHitsNb, bool& hasBlockingHit, PxSceneQueryFlags hintFlags, 
											 PxU32 filterFlags, PxSceneQueryFilterCallback* filterCall, const PxFilterData* filterData, PxBatchQueryPreFilterShader preFilterShader, PxBatchQueryPostFilterShader postFilterShader, const void* constBlock, PxU32 cBlockSize, 
											 PxClientID queryClient, SceneQueryShapeData* cache, bool isBatchQuery)
{
	PX_CHECK_AND_RETURN_NULL(PxAbs(rayDir.magnitudeSquared()-1)<1e-4, "ray direction not valid: must be unit vector.");

	hasBlockingHit = false;

	PxRaycastHit blockingHit;
	blockingHit.shape			= NULL;  // "NULL" is a valid result
	blockingHit.flags			= PxSceneQueryFlags(0);
	blockingHit.distance		= rayLength;
	blockingHit.faceIndex		= PX_INVALID_U32;

	// Handle cache if needed
	PxU32 nbCacheHits = 0;
	if(cache)
	{
		// No filtering for cached shape
		const Gu::RaycastFunc& raycast = Gu::gRaycastMap[cache->shapeGeometryEA->getType()];
		//!!!SQ  TODO: When multiple hits per shape (meshes/hf/...) are supported, we have to define what should happen in the cached shape case (store multiple hits or not?).
		PX_ASSERT((filterFlags & PxSceneQueryFilterFlag::eMESH_MULTIPLE) == 0);
		nbCacheHits = raycast(cache->shapeGeometryEA->get(), cache->sqGlobalPose, rayOrigin, rayDir, blockingHit.distance, hintFlags|PxSceneQueryFlag::eDISTANCE, 1, &blockingHit, false);  // compute distance to shrink the ray if the cached shape hits
		if(nbCacheHits > 0)
		{
			PX_ASSERT(blockingHit.flags & PxSceneQueryFlag::eDISTANCE);
			blockingHit.shape = cache->shape;
		}
		else
			blockingHit.flags = PxSceneQueryFlags(0);
	}

	const bool passForeignShapes = (getScene().getClientBehaviorBits(queryClient) & PxClientBehaviorBit::eREPORT_FOREIGN_OBJECTS_TO_SCENE_QUERY) != 0;

	RaycastMultipleParams msp;
	msp.rayOrigin		= rayOrigin;
	msp.rayDir			= rayDir;
	msp.blockingHit		= &blockingHit;
	msp.hintFlags		= hintFlags;
	msp.nonBatchedFilter= filterCall;
	msp.filterFlags		= filterFlags;
	msp.filterData		= filterData;
	msp.preFilterShader = preFilterShader;
	msp.postFilterShader= postFilterShader;
	msp.constBlock		= constBlock;
	msp.cBlockSize		= cBlockSize;
	msp.queryClient		= queryClient;			
	msp.passForeignShapes = passForeignShapes;
	msp.nbHits			= 0;
	msp.hits			= hits;
	msp.maxHitsNb		= maxHitsNb;
	msp.cachedShape		= cache;
	msp.isBatchQuery	= isBatchQuery;
	msp.userHitBufferAsTmpBuffer = true;

	// PT: TODO: reactivate pruning groups!!!
	stab(raycastMultipleCallback, &msp, rayOrigin, rayDir, blockingHit.distance, filterFlags);

	PX_ASSERT(maxHitsNb > 0);
	
	// Note: msp.nbHits does not include the blocking hit

	PxU32 nbTouchingHitsForWriteBack;
	PxI32 returnValue;
	if (blockingHit.shape)
	{
		hasBlockingHit = true;

		if (msp.nbHits < maxHitsNb)
		{
			nbTouchingHitsForWriteBack = msp.nbHits;
			returnValue = msp.nbHits + 1;  // Does not count the blocking hit
		}
		else
		{
			nbTouchingHitsForWriteBack = maxHitsNb - 1;
			returnValue = -1;
		}

		hits[nbTouchingHitsForWriteBack] = blockingHit;
	}
	else
	{
		if (msp.nbHits <= maxHitsNb)
		{
			nbTouchingHitsForWriteBack = msp.nbHits;
			returnValue = msp.nbHits;
		}
		else
		{
			nbTouchingHitsForWriteBack = maxHitsNb;
			returnValue = -1;
		}
	}
	
	if (msp.userHitBufferAsTmpBuffer)
	{
		return returnValue;
	}
	else
	{
		PX_ASSERT(hits != msp.hits);
		Ps::memCopy(hits, msp.hits, sizeof(PxRaycastHit) * nbTouchingHitsForWriteBack);  // Write hit information back if the user buffer did not have enough space for the temporary results
		physx::shdfnd::TempAllocator().deallocate(msp.hits);  // The buffer was resized -> the memory is temporary and needs to be freed

		return returnValue;
	}
}

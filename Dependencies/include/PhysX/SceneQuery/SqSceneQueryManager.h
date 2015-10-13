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


#ifndef PX_PHYSICS_SCENEQUERYMANAGER
#define PX_PHYSICS_SCENEQUERYMANAGER
/** \addtogroup physics 
@{ */

#include "PsArray.h"
#include "PsPool.h"
#include "PxBounds3.h"
#include "PxSceneDesc.h" // PxPruningStructure
#include "PxSceneQueryFiltering.h"
#include "PxBatchQuery.h"
#include "PxClient.h"
#include "GuInternalTriangleMesh.h"
#include "SqQueryBuffers.h"
#include "CmReaderWriterLock.h"
#include "SqBatchQuery.h"
#include "../SimulationController/include/ScScene.h"

// threading
#include "PsSList.h"

namespace physx
{

class PxSweepCache;
class NpShape;

namespace Scb
{
	class Scene;
}

namespace Sq
{
	class BatchQuery;
	struct SceneQueryShapeData;
	class SweptVolume;
	class ObjectCache;

	//Temp put here, all data are required by user callback TODO: move to shared header for ppu and spu
	struct RaycastQueryResult
	{
		PxRaycastHit	hit;
		PxU32			queryType;
		void*			userdata;
		PxU32			queryEA;	//So we can trace which query ask for this result
	};

	struct SceneQueryManagerDesc
	{
		SceneQueryManagerDesc():
			maxBounds					(PxBounds3::empty()),
			maxNbStaticShapes			(0),
			maxNbDynamicShapes			(0),
			upAxis						(0),
			subdivisionLevel			(0),
			dynamicStructure			(PxPruningStructure::eNONE),
			staticStructure				(PxPruningStructure::eNONE),
			dynamicTreeRebuildRateHint	(0)
		{
		}

		PxBounds3					maxBounds;
		PxU32						maxNbStaticShapes;
		PxU32						maxNbDynamicShapes;
		PxU32						upAxis;
		PxU32						subdivisionLevel;
		PxPruningStructure::Enum	dynamicStructure;
		PxPruningStructure::Enum	staticStructure;
		PxU32						dynamicTreeRebuildRateHint;
	};

	class SceneQueryManager : public Ps::UserAllocated
	{
	public:
														SceneQueryManager(Scb::Scene& scene, const PxSceneDesc& desc);
														~SceneQueryManager();

						SceneQueryShapeData*			addShape(const NpShape& shape, bool dynamic);
						void							addShapes(PxU32 nbShapes, NpShape*const* PX_RESTRICT shapes, PxActor** PX_RESTRICT owners, PxType actorType, bool isDynamic);
						void							removeShape(NpShape& shape);
		// queries
						bool							raycastAny				(const PxVec3& rayOrigin, const PxVec3& rayDir, PxReal rayLength, PxSceneQueryHit& hit, PxU32 filterFlags, PxSceneQueryFilterCallback* filterCall, const PxFilterData* filterData, PxBatchQueryPreFilterShader preFilterShader, PxBatchQueryPostFilterShader postFilterShader, const void* constBlock, PxU32 cBlockSize, PxClientID queryClient, SceneQueryShapeData* cache, bool isBatchQuery);
						bool							raycastSingle			(const PxVec3& rayOrigin, const PxVec3& rayDir, PxReal rayLength, PxRaycastHit& hit, PxSceneQueryFlags hintFlags, PxU32 filterFlags, PxSceneQueryFilterCallback* filterCall, const PxFilterData* filterData, PxBatchQueryPreFilterShader preFilterShader, PxBatchQueryPostFilterShader postFilterShader, const void* constBlock, PxU32 cBlockSize, PxClientID queryClient, SceneQueryShapeData* cache, bool isBatchQuery);
						PxI32							raycastMultiple			(const PxVec3& rayOrigin, const PxVec3& rayDir, PxReal rayLength, PxRaycastHit* hits, const PxU32 maxHitsNb, bool& hasBlockingHit, PxSceneQueryFlags hintFlags, PxU32 filterFlags, PxSceneQueryFilterCallback* filterCall, const PxFilterData* filterData, PxBatchQueryPreFilterShader preFilterShader, PxBatchQueryPostFilterShader postFilterShader, const void* constBlock, PxU32 cBlockSize, PxClientID queryClient, SceneQueryShapeData* cache, bool isBatchQuery);

						PxI32							overlapSphereObjects	(const Gu::Sphere& worldSphere, bool multipleOverlaps, PxU32 nbObjects, PxShape** objects, void* userData, PxU32 filterFlags, PxSceneQueryFilterCallback* filterCall, const PxFilterData* filterData, PxBatchQueryPreFilterShader preFilterShader, PxBatchQueryPostFilterShader postFilterShader, const void* constBlock, PxU32 cBlockSize, PxClientID queryClient, bool isBatchQuery);
						PxI32							overlapAABBObjects		(const PxBounds3& worldBounds, bool multipleOverlaps, PxU32 nbObjects, PxShape** objects, void* userData, PxU32 filterFlags, PxSceneQueryFilterCallback* filterCall, const PxFilterData* filterData, PxBatchQueryPreFilterShader preFilterShader, PxBatchQueryPostFilterShader postFilterShader, const void* constBlock, PxU32 cBlockSize, PxClientID queryClient, bool isBatchQuery);
						PxI32							overlapOBBObjects		(const PxVec3& boxCenter, const PxVec3& boxExtents, const PxQuat& boxRot, bool multipleOverlaps, PxU32 nbObjects, PxShape** objects, void* userData, PxU32 filterFlags, PxSceneQueryFilterCallback* filterCall, const PxFilterData* filterData, PxBatchQueryPreFilterShader preFilterShader, PxBatchQueryPostFilterShader postFilterShader, const void* constBlock, PxU32 cBlockSize, PxClientID queryClient, bool isBatchQuery);
						PxI32							overlapCapsuleObjects	(const Gu::Capsule& worldCapsule, bool multipleOverlaps, PxU32 nbObjects, PxShape** objects, void* userData, PxU32 filterFlags, PxSceneQueryFilterCallback* filterCall, const PxFilterData* filterData, PxBatchQueryPreFilterShader preFilterShader, PxBatchQueryPostFilterShader postFilterShader, const void* constBlock, PxU32 cBlockSize, PxClientID queryClient, bool isBatchQuery);

						PxU32							cullObjects				(PxU32 nbPlanes, const Gu::Plane* worldPlanes, PxU32 nbObjects, PxShape** objects, void* userData, PxU32 filterFlags, PxSceneQueryFilterCallback* filterCall, const PxFilterData* filterData, PxBatchQueryPreFilterShader preFilterShader, PxBatchQueryPostFilterShader postFilterShader, const void* constBlock, PxU32 cBlockSize, PxClientID queryClient);

						PxI32							linearOBBSweep			(const PxVec3& boxCenter, const PxVec3& boxExtents, const PxQuat& boxRot, const PxVec3& unitDir, const PxReal distance, PxU32 nbObjects, PxSweepHit* objects, bool& hasBlockingHit, bool multipleHits, bool anyHit, PxU32 filterFlags, PxSceneQueryFilterCallback* filterCall, const PxFilterData* filterData, PxBatchQueryPreFilterShader preFilterShader, PxBatchQueryPostFilterShader postFilterShader, const void* constBlock, PxU32 cBlockSize, PxClientID queryClient, PrunedObjects* prunedObjects, const Sq::SceneQueryShapeData* cache, PxSceneQueryFlags hintFlags, bool isBatchQuery);
						PxI32							linearCapsuleSweep		(const Gu::Capsule& worldCapsule, const PxVec3& unitDir, const PxReal distance, PxU32 nbObjects, PxSweepHit* objects, bool& hasBlockingHit, bool multipleHits, bool anyHit, PxU32 filterFlags, PxSceneQueryFilterCallback* filterCall, const PxFilterData* filterData, PxBatchQueryPreFilterShader preFilterShader, PxBatchQueryPostFilterShader postFilterShader, const void* constBlock, PxU32 cBlockSize, PxClientID queryClient, PrunedObjects* prunedObjects, const Sq::SceneQueryShapeData* cache, PxSceneQueryFlags hintFlags, bool isBatchQuery);
						PxI32							linearGeometrySweep		(const PxGeometry& geometry, const PxTransform& pose, const PxVec3& unitDir, const PxReal distance, PxU32 nbHits, PxSweepHit* hits, bool& hasBlockingHit, bool multipleHits, bool anyHit, PxU32 filterFlags, PxSceneQueryFilterCallback* filterCall, const PxFilterData* filterData, PxBatchQueryPreFilterShader preFilterShader, PxBatchQueryPostFilterShader postFilterShader, const void* constBlock, PxU32 cBlockSize, PxClientID queryClient, PrunedObjects* objects, const Sq::SceneQueryShapeData* cache, PxSceneQueryFlags hintFlags, bool isBatchQuery);
						PxI32							linearGeometrySweep		(const PxGeometry** geometryList, const PxTransform* poseList, const PxFilterData* filterDataList, PxU32 geometryCount, const PxVec3& unitDir, const PxReal distance, PxU32 nbHits, PxSweepHit* hits, bool& hasBlockingHit, bool multipleHits, bool anyHit, PxU32 filterFlags, PxSceneQueryFilterCallback* filterCall, PxClientID queryClient, const Sq::SceneQueryShapeData* cache, PxSceneQueryFlags hintFlags);
						PxI32							linearCompoundGeometrySweep(const Gu::GeometryUnion* geometryList, const PxTransform* poseList, const PxFilterData* filterDataList, PxU32 geometryCount, const PxVec3& unitDir, const PxReal distance, PxU32 flags, PxU32 nbHits, PxSweepHit* hits, bool& hasBlockingHit, bool multipleHits, bool anyHit, PxSceneQueryFilterCallback* filterCall, PxBatchQueryPreFilterShader preFilterShader, PxBatchQueryPostFilterShader postFilterShader, const void* constBlock, PxU32 cBlockSize, PxClientID queryClient, PrunedObjects* objects, const PxSweepCache* sweepCache, const Sq::SceneQueryShapeData* cache, PxSceneQueryFlags hintFlags, bool isBatchQuery);

		// is create/release SweepCache needed?
						PxSweepCache*					createSweepCache(PxReal dimensions);
						void							releaseSweepCache(PxSweepCache* cache);
						void							updateCache(ObjectCache& cache,
																	const PxVec3& cachedVolume_Center, const PxVec3& cachedVolume_Extents, const PxQuat& cachedVolume_Rot,
																	const PxVec3& worldBox_Center, const PxVec3& worldBox_Extents, const PxQuat& worldBox_Rot,
																	bool updateStatic, bool updateDynamic);
						void							updateCache(ObjectCache& cache,
																	const PxVec3& worldBox_Center, const PxVec3& worldBox_Extents, const PxQuat& worldBox_Rot,
																	const PxVec3* cacheBox_Center, const PxVec3* cacheBox_Extents, const PxQuat* cacheBox_Rot
																	);
	public:
		PX_FORCE_INLINE	Scb::Scene&						getScene()						const	{ return mScene;							}
		PX_FORCE_INLINE	void							addToDirtyShapes(NpShape& s)			{ mDirtyShapes.pushBack(&s);				}
		PX_FORCE_INLINE	PxU32							getDynamicTreeRebuildRateHint()	const	{ return mDesc.dynamicTreeRebuildRateHint;	}
		PX_FORCE_INLINE	Cm::ReaderWriterLock&			getSceneQueryLock()				const	{ return mSceneQueryLock;					}

		PX_FORCE_INLINE	const PxBounds3&				getMaxBounds()					const	{ return mDesc.maxBounds;					}
		PX_FORCE_INLINE	PxPruningStructure::Enum		getStaticStructure()			const	{ return mDesc.staticStructure;				}
		PX_FORCE_INLINE	PxPruningStructure::Enum		getDynamicStructure()			const	{ return mDesc.dynamicStructure;			}
		PX_FORCE_INLINE	PxU32							getSubdivisionLevel()			const	{ return mDesc.subdivisionLevel;			}
		PX_FORCE_INLINE	PxU32							getUpAxis()						const	{ return mDesc.upAxis;						}

		PX_FORCE_INLINE	Pruner*							getStaticPruner()						{ return mPruners[0];						}
		PX_FORCE_INLINE	const Pruner*					getStaticPruner()				const	{ return mPruners[0];						}
		PX_FORCE_INLINE	Pruner*							getDynamicPruner()						{ return mPruners[1];						}
		PX_FORCE_INLINE	const Pruner*					getDynamicPruner()				const	{ return mPruners[1];						}

						void							setDynamicTreeRebuildRateHint(PxU32 dynTreeRebuildRateHint);
						void							flushUpdates();
		// Force a rebuild of the aabb/loose octree etc to allow raycasting on multiple threads.
						void							eagerUpdatePruningTrees();

						void							growObjectTags(PxU32 nbStamps);
						// fetches/releases buffers per thread using TLS and a pool of buffers  (former get/put NPhaseContext)
						QueryBuffers*					aquireQueryBuffers() const;
						void							releaseQueryBuffers(QueryBuffers* b) const;

						void							visualize(Cm::RenderOutput& out, bool visStatic, bool visDynamic);
	private:
						Pruner*							mPruners[2];	// 0 = static, 1 = dynamic

						Ps::Array<PxSweepCache*>		mSweepCaches;
#ifdef USE_MY_POOL
						PoolT<SceneQueryShapeData>		mShapeSqDataPool;	// maybe this pool should be in the pruning engine?? PT: just put "Prunable" in all shapes and drop the pool?
#else
						Ps::Pool<SceneQueryShapeData>	mShapeSqDataPool;	// maybe this pool should be in the pruning engine?? PT: just put "Prunable" in all shapes and drop the pool?
#endif
						PxU32							mNumShapes;	// PT: TODO: remove this, info already available in PruningEngine
						Ps::Array<NpShape*>				mDirtyShapes;

						Scb::Scene&						mScene;

						SceneQueryManagerDesc			mDesc;

						// threading
						PxU32 							mMaxObjectTags;
						PxU32							mQueryBufferTlsSlot;
		mutable			Ps::SList						mQueryBuffersPool;
		mutable			Cm::ReaderWriterLock			mSceneQueryLock;

						bool							AddObject(Prunable& object);
						bool							RemoveObject(Prunable& object);
		PX_FORCE_INLINE	bool							UpdateObject(Prunable& object)
														{
															// Checkings
															if(object.mHandle==INVALID_PRUNING_HANDLE)
																return false;	// the object has not been added to the engine

															// Invalidate world box
															object.mPRNFlags &= ~PRN_VALIDAABB;

															// Update appropriate pool
															return mPruners[object.PrunerIndex()]->UpdateObject(object);
														}
		// Queries dispatcher
						void							cull(PruningTemps& temps, CulledObjects& objects, const Gu::Plane* planes, PxU32 nb_planes, PxU32 flags);
						void							stab(StabCallback callback, void* user_data, const PxVec3& orig, const PxVec3& dir, float max_dist, PxU32 flags);
						void							overlap(ReportPrunablesCallback cb, void* userData, const Gu::Sphere& sphere, PxU32 flags);
						void							overlap(ReportPrunablesCallback cb, void* userData, const PxBounds3& box, PxU32 flags);
						void							overlap(ReportPrunablesCallback cb, void* userData, const Gu::Box& box, PxU32 flags);
						void							overlap(ReportPrunablesCallback cb, void* userData, const Gu::Capsule& capsule, PxU32 flags);

						PxI32							runLinearSweepCB(const SweptVolume& volume, const PxSweepHit& blockingHit, const Gu::Box& querySweptBox, const PxVec3& unitDir, 
																	PxU32 nbHits, PxSweepHit* hits, bool& hasBlockingHit, bool multipleHits, bool anyHit, 
																	PxU32 filterFlags, PxSceneQueryFilterCallback* filterCall, const PxFilterData* filterData, PxBatchQueryPreFilterShader preFilterShader,
																	PxBatchQueryPostFilterShader postFilterShader, const void* constBlock, PxU32 cBlockSize, 
																	PxClientID queryClient, const SceneQueryShapeData* cache, PxSceneQueryFlags hintFlags, bool isBatchQuery);
	};



	class QueryBuffersGrab
	{
	public:
		QueryBuffersGrab(const SceneQueryManager& sqm) : mSceneQueryManager(sqm)
		{
			mBuffers = sqm.aquireQueryBuffers();
		}
		~QueryBuffersGrab()
		{
			mSceneQueryManager.releaseQueryBuffers(mBuffers);
		}

		PX_FORCE_INLINE			QueryBuffers*	operator->()			{ return mBuffers;	}
		PX_FORCE_INLINE	const	QueryBuffers*	operator->()	const	{ return mBuffers;	}
		PX_FORCE_INLINE			QueryBuffers&	operator*()				{ return *mBuffers;	}
		PX_FORCE_INLINE	const	QueryBuffers&	operator*()		const	{ return *mBuffers;	}

	private:
		QueryBuffers*					mBuffers;
		const SceneQueryManager&		mSceneQueryManager;
	};

} // namespace Sq

}

/** @} */
#endif

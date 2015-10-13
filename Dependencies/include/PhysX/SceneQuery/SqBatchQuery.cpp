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
#include "SqBatchQuery.h"
#include "SqSceneQueryManager.h"
#include "SqUtilities.h"
#include "PxBatchQueryDesc.h"

#if defined(PX_PS3)
#define SPU_RAYCAST 1
#define SPU_SWEEPTEST 0
#define FORCE_SINGLE_SPU 0

#define SPU_RAYCAST_PROFILE 1

#if SPU_RAYCAST_PROFILE
#include "CellSPUProfiling.h"

using namespace physx;

PxU64 PX_ALIGN(16, gProfileCounters[MAX_NUM_SPU_PROFILE_ZONES])={0};
#endif

#include "PS3Support.h"

#if SPU_RAYCAST
#include "CellRaycastTask.h"
#include "SqStaticPruner.h"
#include "SqDynamicPruner2.h"
#include "OPC_AABBTree.h"

CellRaycastSPUInput	PX_ALIGN(128, gCellRaycastSPUInput[6]);				
CellRaycastSPUOutput PX_ALIGN(128, gCellRaycastSPUOutput[6]);

//Size of PxRaycastQueryResult must be a multiple of 16 bytes 
PX_COMPILE_TIME_ASSERT((sizeof(PxRaycastQueryResult)&0x0f) == 0);

//Size of PxSweepQueryResult must be a multiple of 16 bytes 
PX_COMPILE_TIME_ASSERT((sizeof(PxSweepQueryResult)&0x0f) == 0);

#endif
#endif

using namespace physx;

Sq::BatchQuery::BatchQuery(SceneQueryManager& owner, const PxBatchQueryDesc& desc) : mSceneQueryManager(owner), mDesc(desc)
{
	PX_ASSERT(	((desc.filterShaderData) && (desc.filterShaderDataSize > 0)) ||
		(!(desc.filterShaderData) && (desc.filterShaderDataSize == 0))	);
	if (desc.filterShaderData)
	{
		mDesc.filterShaderData = PX_ALLOC(desc.filterShaderDataSize);
		Ps::memCopy(mDesc.filterShaderData, desc.filterShaderData, desc.filterShaderDataSize);
	}
	else
	{
		mDesc.filterShaderData = NULL;
		mDesc.filterShaderDataSize = 0;
	}
#if defined(PX_PS3)
	mIsRaycastPS3SupportTaskStarted = false;
#endif
}

Sq::BatchQuery::~BatchQuery()
{
#if defined(PX_PS3)
	if(mIsRaycastPS3SupportTaskStarted)
		g_PS3SupportRaycast.stopSPU();
#endif
	PX_FREE_AND_RESET(mDesc.filterShaderData);
}

void Sq::BatchQuery::execute()
{
	mSceneQueryManager.flushUpdates();

	Cm::TakeReaderLock lock(mSceneQueryManager.getSceneQueryLock());	

	// get filtering params
	PxBatchQueryPreFilterShader	preFilterShader		= getPreFilterShader();
	PxBatchQueryPostFilterShader	postFilterShader	= getPostFilterShader();
	const void* constBlock = getFilterShaderData();
	const PxU32 cBlockSize = getFilterShaderDataSize();

	const QueryData* queryData;	

	PxU32 nbRaycastQuery	= mBatchQueryStream.getNbRaycastQuery();
#if SPU_SWEEPTEST	
	PxU32 nbSweepQuery		= mBatchQueryStream.getNbSweepQuery();
#else
	PxU32 nbSweepQuery		= 0;
#endif

	PxU32 nbTotalQuery		= nbRaycastQuery + nbSweepQuery;

	PxU32 usedSPUs = 0;
	mRaycastHitOffset = 0;
	mSweepHitOffset = 0;
	mOverlapHitOffset = 0;

	bool bValidForRaycast = true;
	bool bValidForSweep = true;
	bool bValidForOverlap = true;

	PxRaycastQueryResult*	pRaycastResult  = mDesc.userRaycastResultBuffer;
	PxRaycastHit*			pRaycastHits	= mDesc.userRaycastHitBuffer; 

	PxSweepQueryResult*		pSweepResult	= mDesc.userSweepResultBuffer;
	PxSweepHit*				pSweepHit		= mDesc.userSweepHitBuffer; 

	PxOverlapQueryResult*	pOverlapResult	= mDesc.userOverlapResultBuffer;
	PxShape**				pOverlapHit		= mDesc.userOverlapHitBuffer; 

	if( nbTotalQuery )
	{
		bValidForRaycast = (mDesc.userRaycastResultBuffer && mDesc.userRaycastHitBuffer && mDesc.raycastHitBufferSize);
		bValidForSweep = (mDesc.userSweepResultBuffer && mDesc.userSweepHitBuffer && mDesc.sweepHitBufferSize);

		if(bValidForRaycast)
		{
			for(PxU32 i = 0; i < nbRaycastQuery; i++)
			{
				pRaycastResult[i].queryStatus = PxBatchQueryStatus::ePENDING;
				pRaycastResult[i].nbHits = 0;
			}
		}
		if(bValidForSweep)
		{
			for(PxU32 i = 0; i < nbSweepQuery; i++)
			{
				pSweepResult[i].queryStatus = PxBatchQueryStatus::ePENDING;
				pSweepResult[i].nbHits = 0;
			}
		}

#if defined(PX_PS3)
	#if FORCE_SINGLE_SPU
			PX_ASSERT(g_iPhysXQuerySPUCount>=1);
			const PxU32 numSPUs = 1;
	#else
			const PxU32 numSPUs = mSceneQueryManager.getScene().getSceneParamInt(PxPS3ConfigParam::eSPU_RAYCAST);;
	#endif
#endif
			
		if( !bValidForRaycast && !bValidForSweep)
		{
			Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "BatchedRaycast: "
				"the userRaycastResultBuffer, userRaycastHitBuffer and raycastHitBufferSize or "
				"userSweepResultBuffer, userSweepHitBuffer and sweepHitBufferSize in PxBatchQueryDesc must not be NULL.");
			//bValidForRaycast = false;
		}
		else if((	mSceneQueryManager.getDynamicStructure() == PxPruningStructure::eDYNAMIC_AABB_TREE	||
					mSceneQueryManager.getDynamicStructure() == PxPruningStructure::eSTATIC_AABB_TREE	||
					mSceneQueryManager.getDynamicStructure() == PxPruningStructure::eNONE)				&&
				(	mSceneQueryManager.getStaticStructure() == PxPruningStructure::eDYNAMIC_AABB_TREE	||
					mSceneQueryManager.getStaticStructure() == PxPruningStructure::eSTATIC_AABB_TREE	||
					mSceneQueryManager.getStaticStructure() == PxPruningStructure::eNONE)
#if defined(PX_PS3)
					&& numSPUs > 0
#endif
					)
		{
			//Cm::TakeReaderLock lock(mSceneQueryManager.getSceneQueryLock());			

//TODO move to CellRaycast later
#if SPU_RAYCAST
			PxU32 queriesPerSPU = nbTotalQuery/numSPUs;
			queriesPerSPU = PxMin(nbTotalQuery, PxMax(queriesPerSPU, 8u));
			usedSPUs = nbTotalQuery/queriesPerSPU;

			//only process even number of queries first, the last spu process all the other queries
			PxU32 rayQueriesPerSPU = (nbRaycastQuery/usedSPUs)&~0x1; 
			PxU32 sweepQueriesPerSPU = (nbSweepQuery/usedSPUs)&~0x1; 

			const Sq::StaticPruner* staticPruner = reinterpret_cast<const Sq::StaticPruner*>(mSceneQueryManager.getStaticPruner());			
			const Ice::AABBTree* staticTree = staticPruner->GetAABBTree();
			
			
			//For dynamic prunables
			Prunable**		addedPrunables = NULL;
			PxU32			nbAddedPrunables = 0;
			PxU32			dynamicPrunables = 0;
			const Ice::AABBTree*	dynamicTree = NULL;

			const Sq::Pruner*	dynamicPruner = mSceneQueryManager.getDynamicPruner();
			Prunable**		dynamicObjects = dynamicPruner->GetObjects();

			if(mSceneQueryManager.getDynamicStructure() == PxPruningStructure::eDYNAMIC_AABB_TREE)
			{
				const Sq::DynamicPruner2* dynamicPruner2 = reinterpret_cast<const Sq::DynamicPruner2*>(dynamicPruner);
				addedPrunables = dynamicPruner2->GetAddedPrunables();
				nbAddedPrunables = dynamicPruner2->GetNbAddedPrunables();
				dynamicTree = dynamicPruner2->GetAABBTree();
			}
			if(mSceneQueryManager.getDynamicStructure() == PxPruningStructure::eSTATIC_AABB_TREE)
			{
				dynamicTree = reinterpret_cast<const Sq::StaticPruner*>(dynamicPruner)->GetAABBTree();
			}
			//else if(mSceneQueryManager.getDynamicStructure() == PxPruningStructure::eNONE)
			//{
				dynamicPrunables = dynamicPruner->GetNbObjects();
			//}

			//const Ice::AABBTree* dynamicTree = dynamicPruner2->GetAABBTree();

			const RaycastQueryData* rayQuery = reinterpret_cast<const RaycastQueryData*>(mBatchQueryStream.getNextRaycastQueryDataBatch(nbRaycastQuery));
#if SPU_SWEEPTEST			
			const QueryData*		sweepQuery = reinterpret_cast<const QueryData*>(mBatchQueryStream.getNextSweepQueryDataBatch(nbSweepQuery));
#else
			const QueryData*		sweepQuery = NULL;
#endif
			if(!mIsRaycastPS3SupportTaskStarted)
			{
				mIsRaycastPS3SupportTaskStarted = true;
				g_PS3SupportRaycast.startSPU();				
			}				

			const bool passForeignShapes = (mSceneQueryManager.getScene().getClientBehaviorBits(mDesc.ownerClient) & PxClientBehaviorBit::eREPORT_FOREIGN_OBJECTS_TO_SCENE_QUERY) != 0;

			for( PxU32 uiTask = 0; uiTask < usedSPUs; uiTask++ )
			{
				//The last spu
				if( uiTask == usedSPUs - 1)
					rayQueriesPerSPU = nbRaycastQuery - rayQueriesPerSPU*uiTask;

				gCellRaycastSPUInput[uiTask].mQueries = const_cast<RaycastQueryData*>(rayQuery);
				gCellRaycastSPUInput[uiTask].mSweepQuery = const_cast<QueryData*>(sweepQuery);
				gCellRaycastSPUInput[uiTask].mRaycastResult = pRaycastResult;
				gCellRaycastSPUInput[uiTask].mSweepResult = pSweepResult;
				gCellRaycastSPUInput[uiTask].mRaycastHits = mDesc.userRaycastHitBuffer;
				gCellRaycastSPUInput[uiTask].mSweepHits = mDesc.userSweepHitBuffer;
				gCellRaycastSPUInput[uiTask].mRaycastHitsOffsetEA = (PxU32)&mRaycastHitOffset;
				gCellRaycastSPUInput[uiTask].mStaticPrunerAABBTreeNode = (staticTree!= NULL)?const_cast<Ice::AABBTreeNode*>(staticTree->GetNodes()):NULL;
				gCellRaycastSPUInput[uiTask].mStaticPrunerObjects = staticPruner->GetObjects();
				gCellRaycastSPUInput[uiTask].mDynamicPrunerObjects = dynamicObjects;
				gCellRaycastSPUInput[uiTask].mDynamicPruner2AABBTreeNode = (dynamicTree!= NULL)?const_cast<Ice::AABBTreeNode*>(dynamicTree->GetNodes()):NULL;;
				gCellRaycastSPUInput[uiTask].mDynamicPruner2ObjectsAdded = addedPrunables;
				gCellRaycastSPUInput[uiTask].mNbStaticPrunerObjects = staticPruner->GetNbObjects();
				gCellRaycastSPUInput[uiTask].mNbDynamicPrunerObjects = dynamicPrunables;
				gCellRaycastSPUInput[uiTask].mNbAddedDynamicPrunerObjects = nbAddedPrunables;
				gCellRaycastSPUInput[uiTask].mPreFilterShader = mDesc.spuPreFilterShader;
				gCellRaycastSPUInput[uiTask].mPreFilterShaderSize = mDesc.spuPreFilterShaderSize;
				gCellRaycastSPUInput[uiTask].mPostFilterShader = mDesc.spuPostFilterShader;
				gCellRaycastSPUInput[uiTask].mPostFilterShaderSize = mDesc.spuPostFilterShaderSize;
				gCellRaycastSPUInput[uiTask].mNumQueries = rayQueriesPerSPU;
				gCellRaycastSPUInput[uiTask].mNumSweepQueries = sweepQueriesPerSPU;
				gCellRaycastSPUInput[uiTask].mHitBufferSize = mDesc.raycastHitBufferSize;
				gCellRaycastSPUInput[uiTask].mSweepHitBufferSize = mDesc.sweepHitBufferSize;
				gCellRaycastSPUInput[uiTask].mDynamicTreeType = mSceneQueryManager.getDynamicStructure();
				gCellRaycastSPUInput[uiTask].mConstBlock = const_cast<void*>(constBlock);				
				gCellRaycastSPUInput[uiTask].mBlockSize = cBlockSize;
				gCellRaycastSPUInput[uiTask].mQueryClient = mDesc.ownerClient;	//TODO: test multi client
				gCellRaycastSPUInput[uiTask].mPassForeignShapes = passForeignShapes;								;

				gCellRaycastSPUInput[uiTask].mDebugEnable = false;
#if SPU_RAYCAST_PROFILE
				gCellRaycastSPUInput[uiTask].mProfileZones = gProfileCounters;
#endif
				
				rayQuery += rayQueriesPerSPU;
				sweepQuery += sweepQueriesPerSPU;
				pRaycastResult += rayQueriesPerSPU;
				pSweepResult += sweepQueriesPerSPU;

				g_PS3SupportRaycast.sendRequest(uiTask , (unsigned int)&gCellRaycastSPUInput[uiTask], 0);
			}

#endif
		}
	}
	
#if SPU_RAYCAST
	if( nbRaycastQuery && usedSPUs)
	{
		//Wait for all the spu tasks to complete.
		for (PxU32 uiTask=0; uiTask < usedSPUs; uiTask++)
		{
			uint32_t arg0,arg1;
			g_PS3SupportRaycast.waitForResponse(&arg0, &arg1);
		}		
#if SPU_RAYCAST_PROFILE
		for( int i = 0; i < PROF_DMA_WAIT_NUM; i++)
		{
			//printf("%s\t\t\t\t\t%d\n",proName[i],gProfileCounters[i]);
			//printf("%d\t",gProfileCounters[i]);
			gProfileCounters[i] = 0;
		}
		//printf("\n");
		if(gProfileCounters[0]>100000)
		{
			int stop = 0;
			PX_UNUSED(stop);
		}
#endif
	}
#endif

	//Process PPU raycast
	if(bValidForRaycast)
	{
		while ((queryData = mBatchQueryStream.getNextRaycastQueryData()))
		{
			//Cm::TakeReaderLock lock(mSceneQueryManager.getSceneQueryLock());

			// PT: TODO: a jump table would help here
			switch(queryData->mType)
			{
			case SceneQueryID::QUERY_RAYCAST_ANY_OBJECT:
				{
					const RaycastQueryData* data = reinterpret_cast<const RaycastQueryData*>(queryData);
					pRaycastResult->userData = data->mUserData;

					PxRaycastHit hit;
					hit.shape = NULL;
					mSceneQueryManager.raycastAny(data->mRayOrigin, data->mRayDir, data->mMaxDist, hit, data->mFilterFlags, NULL, data->mFilterDataValid ? &data->mFilterData : NULL, preFilterShader, postFilterShader, constBlock, cBlockSize, mDesc.ownerClient, data->mCache, true);
					
					//if(data->mUserCache) *data->mUserCache = hit.shape;

					// PT: for "raycast any" we don't return a hit structure
					pRaycastResult->hits = NULL;
					pRaycastResult->nbHits = NULL!=hit.shape;
					pRaycastResult->queryStatus = PxBatchQueryStatus::eSUCCESS;
					pRaycastResult++;
				}
				break;

			case SceneQueryID::QUERY_RAYCAST_CLOSEST_OBJECT:
				{
					const RaycastQueryData* data = reinterpret_cast<const RaycastQueryData*>(queryData);
					pRaycastResult->userData = data->mUserData;

					PxRaycastHit hit;
					hit.shape = NULL;
					mSceneQueryManager.raycastSingle(data->mRayOrigin, data->mRayDir, data->mMaxDist, hit, data->mHintFlags, data->mFilterFlags, NULL, data->mFilterDataValid ? &data->mFilterData : NULL, preFilterShader, postFilterShader, constBlock, cBlockSize, mDesc.ownerClient, data->mCache, true);

					//if(data->mUserCache) *data->mUserCache = hit.shape;						

					if(hit.shape)
					{
						//Allocate hit from user hits buffer
						PxU32 hitStartOffset = mRaycastHitOffset;
						mRaycastHitOffset++;

						//Tell user whether hits buffer is full.
						if( mRaycastHitOffset > mDesc.raycastHitBufferSize)
						{
							//Has 1 hit, but not have enough buffer to store.
							pRaycastResult->queryStatus = PxBatchQueryStatus::eABORTED;
						}
						else
						{
							pRaycastResult->queryStatus = PxBatchQueryStatus::eSUCCESS;
							pRaycastResult->hits = pRaycastHits + hitStartOffset;
							*pRaycastResult->hits = hit;
						}
						pRaycastResult->nbHits = 1;
					}
					else
					{
						pRaycastResult->hits = NULL;
						pRaycastResult->nbHits = 0;
						pRaycastResult->queryStatus = PxBatchQueryStatus::eSUCCESS;
					}
					pRaycastResult++;
				}
				break;

			case SceneQueryID::QUERY_RAYCAST_ALL_OBJECTS:
				{
					//These cannot run parallelly with spu raycast.
					const RaycastQueryData* data = static_cast<const RaycastQueryData*>(queryData);
					pRaycastResult->userData = data->mUserData;

					const PxU32 maxHitNb = mDesc.raycastHitBufferSize - mRaycastHitOffset;
					pRaycastResult->hits = pRaycastHits + mRaycastHitOffset;

					bool dummy;
					const PxI32 nbHits = mSceneQueryManager.raycastMultiple(data->mRayOrigin, data->mRayDir, data->mMaxDist, pRaycastResult->hits, maxHitNb, dummy, data->mHintFlags, data->mFilterFlags, NULL, data->mFilterDataValid ? &data->mFilterData : NULL, preFilterShader, postFilterShader, constBlock, cBlockSize, mDesc.ownerClient, NULL, true);

					if( nbHits >= 0)  // -1 if buffer overflow
					{
						pRaycastResult->nbHits = nbHits;
						pRaycastResult->queryStatus = PxBatchQueryStatus::eSUCCESS;
						mRaycastHitOffset += pRaycastResult->nbHits;
					}
					else
					{
						pRaycastResult->nbHits = 0;
						pRaycastResult->queryStatus = PxBatchQueryStatus::eABORTED;
						mRaycastHitOffset += maxHitNb;
					}			

					pRaycastResult++;
				}
				break;

			}
		}
	}

	if(bValidForSweep)
	{
		while ((queryData = mBatchQueryStream.getNextSweepQueryData()))
		{
			bool dummy;
			//Cm::TakeReaderLock lock(mSceneQueryManager.getSceneQueryLock());
			switch(queryData->mType)
			{
				case SceneQueryID::QUERY_LINEAR_OBB_SWEEP_CLOSEST_OBJECT:
					{
						const LinearOBBSweepQueryData* data = reinterpret_cast<const LinearOBBSweepQueryData*>(queryData);
						pSweepResult->userData = data->mUserData;

						const PxU32 maxHitNb = mDesc.sweepHitBufferSize - mSweepHitOffset;
						pSweepResult->hits = pSweepHit + mSweepHitOffset;

						PxI32 nb =  mSceneQueryManager.linearOBBSweep(
							data->mBoxCenter, data->mBoxExtents, data->mBoxRot,
							data->mUnitDir, data->mDistance,
							maxHitNb, pSweepResult->hits, dummy, false, false,
							data->mFilterFlags, NULL, data->mFilterDataValid ? &data->mFilterData : NULL, preFilterShader, postFilterShader, constBlock, cBlockSize, mDesc.ownerClient,
							NULL, NULL, data->mHintFlags, true);

						if( nb >= 0)  // -1 if buffer overflow
						{
							pSweepResult->nbHits = nb;
							pSweepResult->queryStatus = PxBatchQueryStatus::eSUCCESS;
							mSweepHitOffset += pSweepResult->nbHits;
						}
						else
						{
							pSweepResult->nbHits = 0;
							pSweepResult->queryStatus = PxBatchQueryStatus::eABORTED;
							mSweepHitOffset += maxHitNb;
						}			
						
						pSweepResult++;
					}
					break;

				case SceneQueryID::QUERY_LINEAR_CAPSULE_SWEEP_CLOSEST_OBJECT:
					{
						const LinearCapsuleSweepQueryData* data = reinterpret_cast<const LinearCapsuleSweepQueryData*>(queryData);
						pSweepResult->userData = data->mUserData;

						const PxU32 maxHitNb = mDesc.sweepHitBufferSize - mSweepHitOffset;
						pSweepResult->hits = pSweepHit + mSweepHitOffset;

						PxI32 nb =  mSceneQueryManager.linearCapsuleSweep(data->mWorldCapsule, data->mUnitDir, data->mDistance,
							maxHitNb, pSweepResult->hits, dummy, false, false, data->mFilterFlags, NULL, data->mFilterDataValid ? &data->mFilterData : NULL,
							preFilterShader, postFilterShader, constBlock, cBlockSize, mDesc.ownerClient, NULL, NULL, data->mHintFlags, true);

						if( nb >= 0)  // -1 if buffer overflow
						{
							pSweepResult->nbHits = nb;
							pSweepResult->queryStatus = PxBatchQueryStatus::eSUCCESS;
							mSweepHitOffset += pSweepResult->nbHits;
						}
						else
						{
							pSweepResult->nbHits = 0;
							pSweepResult->queryStatus = PxBatchQueryStatus::eABORTED;
							mSweepHitOffset += maxHitNb;
						}			
						
						pSweepResult++;
					}
					break;

				case SceneQueryID::QUERY_LINEAR_COMPOUND_GEOMETRY_SWEEP_CLOSEST_OBJECT:
					{
						const LinearCompoundGeometrySweepQueryData* data = reinterpret_cast<const LinearCompoundGeometrySweepQueryData*>(queryData);
						pSweepResult->userData = data->mUserData;

						const PxU32 maxHitNb = mDesc.sweepHitBufferSize - mSweepHitOffset;
						pSweepResult->hits = pSweepHit + mSweepHitOffset;

						const Gu::GeometryUnion*	geometryList = reinterpret_cast<const Gu::GeometryUnion*>(mBatchQueryStream.offsetToPointer(data->mGeometryListOffset));
						const PxTransform*			poseList = reinterpret_cast<const PxTransform*>(mBatchQueryStream.offsetToPointer(data->mPoseListOffset));
						const PxFilterData*			filterDataList = reinterpret_cast<const PxFilterData*>(mBatchQueryStream.offsetToPointer(data->mFilterDataListOffset));
						PxI32 nb =  mSceneQueryManager.linearCompoundGeometrySweep(geometryList, poseList, filterDataList, data->mGeometryCount, data->mUnitDir,
							data->mDistance, data->mFilterFlags, /*data->mUserData,*/ maxHitNb, pSweepResult->hits, dummy, false, false,
							NULL, preFilterShader, postFilterShader, constBlock, cBlockSize, mDesc.ownerClient, NULL, data->mSweepCache, NULL, data->mHintFlags, true);

						if( nb >= 0)  // -1 if buffer overflow
						{
							pSweepResult->nbHits = nb;
							pSweepResult->queryStatus = PxBatchQueryStatus::eSUCCESS;
							mSweepHitOffset += pSweepResult->nbHits;
						}
						else
						{
							pSweepResult->nbHits = 0;
							pSweepResult->queryStatus = PxBatchQueryStatus::eABORTED;
							mSweepHitOffset += maxHitNb;
						}			
						
						pSweepResult++;
					}
					break;

				case SceneQueryID::QUERY_LINEAR_OBB_SWEEP_ALL_OBJECTS:
					{
						const LinearOBBSweepQueryData* data = reinterpret_cast<const LinearOBBSweepQueryData*>(queryData);
						pSweepResult->userData = data->mUserData;

						const PxU32 maxHitNb = mDesc.sweepHitBufferSize - mSweepHitOffset;
						pSweepResult->hits = pSweepHit + mSweepHitOffset;

						PxI32 nb =  mSceneQueryManager.linearOBBSweep(
							data->mBoxCenter, data->mBoxExtents, data->mBoxRot,
							data->mUnitDir, data->mDistance,
							maxHitNb, pSweepResult->hits, dummy, true, false,
							data->mFilterFlags, NULL, data->mFilterDataValid ? &data->mFilterData : NULL, preFilterShader, postFilterShader, constBlock, cBlockSize, mDesc.ownerClient,
							NULL, NULL, data->mHintFlags, true);

						if( nb >= 0)  // -1 if buffer overflow
						{
							pSweepResult->nbHits = nb;
							pSweepResult->queryStatus = PxBatchQueryStatus::eSUCCESS;
							mSweepHitOffset += pSweepResult->nbHits;
						}
						else
						{
							pSweepResult->nbHits = 0;
							pSweepResult->queryStatus = PxBatchQueryStatus::eABORTED;
							mSweepHitOffset += maxHitNb;
						}			
						
						pSweepResult++;
					}
					break;

				case SceneQueryID::QUERY_LINEAR_CAPSULE_SWEEP_ALL_OBJECTS:
					{
						const LinearCapsuleSweepQueryData* data = reinterpret_cast<const LinearCapsuleSweepQueryData*>(queryData);
						pSweepResult->userData = data->mUserData;

						const PxU32 maxHitNb = mDesc.sweepHitBufferSize - mSweepHitOffset;
						pSweepResult->hits = pSweepHit + mSweepHitOffset;

						PxI32 nb =  mSceneQueryManager.linearCapsuleSweep(data->mWorldCapsule, data->mUnitDir, data->mDistance,
							maxHitNb, pSweepResult->hits, dummy, true, false,
							data->mFilterFlags, NULL, data->mFilterDataValid ? &data->mFilterData : NULL, preFilterShader, postFilterShader, constBlock, cBlockSize, mDesc.ownerClient,
							NULL, NULL, data->mHintFlags, true);

						if( nb >= 0)  // -1 if buffer overflow
						{
							pSweepResult->nbHits = nb;
							pSweepResult->queryStatus = PxBatchQueryStatus::eSUCCESS;
							mSweepHitOffset += pSweepResult->nbHits;
						}
						else
						{
							pSweepResult->nbHits = 0;
							pSweepResult->queryStatus = PxBatchQueryStatus::eABORTED;
							mSweepHitOffset += maxHitNb;
						}			
						
						pSweepResult++;
					}
					break;

				case SceneQueryID::QUERY_LINEAR_COMPOUND_GEOMETRY_SWEEP_ALL_OBJECTS:
					{
						const LinearCompoundGeometrySweepQueryData* data = reinterpret_cast<const LinearCompoundGeometrySweepQueryData*>(queryData);
						pSweepResult->userData = data->mUserData;

						const PxU32 maxHitNb = mDesc.sweepHitBufferSize - mSweepHitOffset;
						pSweepResult->hits = pSweepHit + mSweepHitOffset;

						const Gu::GeometryUnion*	geometryList = reinterpret_cast<const Gu::GeometryUnion*>(mBatchQueryStream.offsetToPointer(data->mGeometryListOffset));
						const PxTransform*			poseList = reinterpret_cast<const PxTransform*>(mBatchQueryStream.offsetToPointer(data->mPoseListOffset));
						const PxFilterData*			filterDataList = reinterpret_cast<const PxFilterData*>(mBatchQueryStream.offsetToPointer(data->mFilterDataListOffset));
						PxI32 nb =  mSceneQueryManager.linearCompoundGeometrySweep(geometryList, poseList, filterDataList, data->mGeometryCount, data->mUnitDir,
							data->mDistance, data->mFilterFlags,
							maxHitNb, pSweepResult->hits, dummy, true, false,
							NULL, preFilterShader, postFilterShader, constBlock, cBlockSize, mDesc.ownerClient, NULL, data->mSweepCache, NULL, data->mHintFlags, true);

						if( nb >= 0)  // -1 if buffer overflow
						{
							pSweepResult->nbHits = nb;
							pSweepResult->queryStatus = PxBatchQueryStatus::eSUCCESS;
							mSweepHitOffset += pSweepResult->nbHits;
						}
						else
						{
							pSweepResult->nbHits = 0;
							pSweepResult->queryStatus = PxBatchQueryStatus::eABORTED;
							mSweepHitOffset += maxHitNb;
						}			
						
						pSweepResult++;

					}
					break;
			}
		}
	}

	if(bValidForOverlap)
	{
		while ((queryData = mBatchQueryStream.getNextOverlapQueryData()))
		{
			//Cm::TakeReaderLock lock(mSceneQueryManager.getSceneQueryLock());
			switch(queryData->mType)
			{
				case SceneQueryID::QUERY_OVERLAP_SPHERE_ALL_OBJECTS:
					{
						const SphereQueryData* data = reinterpret_cast<const SphereQueryData*>(queryData);
						pOverlapResult->userData = data->mUserData;

						const PxU32 maxHitNb = mDesc.overlapHitBufferSize - mOverlapHitOffset;
						pOverlapResult->hits = pOverlapHit + mOverlapHitOffset;

						PxI32 nb =  mSceneQueryManager.overlapSphereObjects(data->mWorldSphere, true,
							maxHitNb, pOverlapResult->hits, 
							data->mUserData, data->mFilterFlags, NULL, data->mFilterDataValid ? &data->mFilterData : NULL, preFilterShader, postFilterShader, constBlock, cBlockSize,
							mDesc.ownerClient, true);

						if( nb >= 0)  // -1 if buffer overflow
						{
							pOverlapResult->nbHits = nb;
							pOverlapResult->queryStatus = PxBatchQueryStatus::eSUCCESS;
							mOverlapHitOffset += pOverlapResult->nbHits;
						}
						else
						{
							pOverlapResult->nbHits = 0;
							pOverlapResult->queryStatus = PxBatchQueryStatus::eABORTED;
							mOverlapHitOffset += maxHitNb;
						}			
						
						pOverlapResult++;
					}
					break;

				case SceneQueryID::QUERY_OVERLAP_AABB_ALL_OBJECTS:
					{
						const AABBQueryData* data = reinterpret_cast<const AABBQueryData*>(queryData);
						pOverlapResult->userData = data->mUserData;

						const PxU32 maxHitNb = mDesc.overlapHitBufferSize - mOverlapHitOffset;
						pOverlapResult->hits = pOverlapHit + mOverlapHitOffset;

						PxI32 nb =  mSceneQueryManager.overlapAABBObjects(data->mWorldBounds, true,
							maxHitNb, pOverlapResult->hits, 
							data->mUserData, data->mFilterFlags, NULL, data->mFilterDataValid ? &data->mFilterData : NULL, preFilterShader, postFilterShader, constBlock, cBlockSize,
							mDesc.ownerClient, true);

						if( nb >= 0)  // -1 if buffer overflow
						{
							pOverlapResult->nbHits = nb;
							pOverlapResult->queryStatus = PxBatchQueryStatus::eSUCCESS;
							mOverlapHitOffset += pOverlapResult->nbHits;
						}
						else
						{
							pOverlapResult->nbHits = 0;
							pOverlapResult->queryStatus = PxBatchQueryStatus::eABORTED;
							mOverlapHitOffset += maxHitNb;
						}			
						
						pOverlapResult++;
					}
					break;

				case SceneQueryID::QUERY_OVERLAP_OBB_ALL_OBJECTS:
					{
						const OBBQueryData* data = reinterpret_cast<const OBBQueryData*>(queryData);
						pOverlapResult->userData = data->mUserData;

						const PxU32 maxHitNb = mDesc.overlapHitBufferSize - mOverlapHitOffset;
						pOverlapResult->hits = pOverlapHit + mOverlapHitOffset;

						PxI32 nb =  mSceneQueryManager.overlapOBBObjects(
							data->mBoxCenter, data->mBoxExtents, data->mBoxRot,
							true, maxHitNb, pOverlapResult->hits, 
							data->mUserData, data->mFilterFlags, NULL, data->mFilterDataValid ? &data->mFilterData : NULL, preFilterShader, postFilterShader, constBlock, cBlockSize,
							mDesc.ownerClient, true);

						if( nb >= 0)  // -1 if buffer overflow
						{
							pOverlapResult->nbHits = nb;
							pOverlapResult->queryStatus = PxBatchQueryStatus::eSUCCESS;
							mOverlapHitOffset += pOverlapResult->nbHits;
						}
						else
						{
							pOverlapResult->nbHits = 0;
							pOverlapResult->queryStatus = PxBatchQueryStatus::eABORTED;
							mOverlapHitOffset += maxHitNb;
						}			
						
						pOverlapResult++;
					}
					break;

				case SceneQueryID::QUERY_OVERLAP_CAPSULE_ALL_OBJECTS:
					{
						const CapsuleQueryData* data = reinterpret_cast<const CapsuleQueryData*>(queryData);
						pOverlapResult->userData = data->mUserData;

						const PxU32 maxHitNb = mDesc.overlapHitBufferSize - mOverlapHitOffset;
						pOverlapResult->hits = pOverlapHit + mOverlapHitOffset;

						PxI32 nb =  mSceneQueryManager.overlapCapsuleObjects(data->mWorldCapsule, true,
							maxHitNb, pOverlapResult->hits, 
							data->mUserData, data->mFilterFlags, NULL, data->mFilterDataValid ? &data->mFilterData : NULL, preFilterShader, postFilterShader, constBlock, cBlockSize,
							mDesc.ownerClient, true);

						if( nb >= 0)  // -1 if buffer overflow
						{
							pOverlapResult->nbHits = nb;
							pOverlapResult->queryStatus = PxBatchQueryStatus::eSUCCESS;
							mOverlapHitOffset += pOverlapResult->nbHits;
						}
						else
						{
							pOverlapResult->nbHits = 0;
							pOverlapResult->queryStatus = PxBatchQueryStatus::eABORTED;
							mOverlapHitOffset += maxHitNb;
						}			
						
						pOverlapResult++;
					}
					break;
				case SceneQueryID::QUERY_CULL_OBJECTS:
					{
						const CullQueryData* data = reinterpret_cast<const CullQueryData*>(queryData);
						const Gu::Plane* worldPlanes = reinterpret_cast<const Gu::Plane*>(data+1);
						pOverlapResult->userData = data->mUserData;

						const PxU32 maxHitNb = mDesc.overlapHitBufferSize - mOverlapHitOffset;
						pOverlapResult->hits = pOverlapHit + mOverlapHitOffset;

						PxI32 nb =  mSceneQueryManager.cullObjects(data->mNbPlanes, worldPlanes,
							maxHitNb, pOverlapResult->hits, 
							data->mUserData, data->mFilterFlags, NULL, data->mFilterDataValid ? &data->mFilterData : NULL, preFilterShader, postFilterShader, constBlock, cBlockSize,
							mDesc.ownerClient);

						if( nb >= 0)  // -1 if buffer overflow
						{
							pOverlapResult->nbHits = nb;
							pOverlapResult->queryStatus = PxBatchQueryStatus::eSUCCESS;
							mOverlapHitOffset += pOverlapResult->nbHits;
						}
						else
						{
							pOverlapResult->nbHits = 0;
							pOverlapResult->queryStatus = PxBatchQueryStatus::eABORTED;
							mOverlapHitOffset += maxHitNb;
						}			
						
						pOverlapResult++;
					}
					break;
			}
		}
	}

	mBatchQueryStream.reset();
}

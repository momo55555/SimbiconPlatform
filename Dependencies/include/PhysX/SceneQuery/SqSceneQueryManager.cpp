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
#include "SqSweepCache.h" // move file to SQ project
#include "SqFiltering.h"
#include "PsThread.h"
#include "NpRigidStatic.h"
#include "NpRigidDynamic.h"
#include "SqDynamicPruner2.h"
#include "SqFreePruner.h"

using namespace physx;
using namespace Ice;
using namespace Sq;

// maybe we could make the add/remove/update methods inline

Pruner* CreateOctreePruner();
Pruner* CreateQuadtreePruner();

bool SceneQueryManager::AddObject(Prunable& object)
{
	// Checkings
	if(object.mHandle!=INVALID_PRUNING_HANDLE)
		return false;	// the object has already been added to the engine

	// Adds the object to appropriate pool
	PX_ASSERT(mPruners[object.PrunerIndex()]);
	// - Add to pruner
	mPruners[object.PrunerIndex()]->AddObject(object);

	// Not mandatory, yet handy
//	object.mEngine = mPruner[object.mType];

	// Initialize it
	return UpdateObject(object);
}

bool SceneQueryManager::RemoveObject(Prunable& object)
{
	// Checkings
	if(object.mHandle==INVALID_PRUNING_HANDLE)
		return false;	// the object has not been added to the engine

	const PxU32 index = object.PrunerIndex();
	if(!mPruners[index])
		return false;	// no pruner allocated

	// Remove the object from appropriate pool
	return mPruners[index]->RemoveObject(object);
}

void SceneQueryManager::cull(PruningTemps& temps, CulledObjects& objects, const Gu::Plane* planes, PxU32 nb_planes, PxU32 flags)
{
	PX_ASSERT(planes);

	// Flush visible list
	objects.ResetObjects();

	for(PxU32 i=0;i<2;i++)
		if(mPruners[i] && flags&(1<<i))
			if(!mPruners[i]->cull(temps, objects, planes, nb_planes, 0))
				return;
}

void SceneQueryManager::stab(StabCallback callback, void* user_data, const PxVec3& orig, const PxVec3& dir, float max_dist, PxU32 flags)
{
	PX_ASSERT(callback);

	float MaxDist = max_dist;
	for(PxU32 i=0;i<2;i++)
	{
		if(mPruners[i] && flags&(1<<i))
		{
			PxU32 Status = mPruners[i]->stab(callback, user_data, orig, dir, MaxDist);
			if(Status & STAB_STOP)
				return;
		}
	}
}

void SceneQueryManager::overlap(ReportPrunablesCallback cb, void* userData, const Gu::Sphere& sphere, PxU32 flags)
{
//	const bool first_contact = (flags&PQF_FIRST_CONTACT)!=0;
	const bool first_contact = false;
	for(PxU32 i=0;i<2;i++)
		if(mPruners[i] && flags&(1<<i))
			if(!mPruners[i]->overlapSphere(cb, userData, sphere, first_contact))
				return;
}

void SceneQueryManager::overlap(ReportPrunablesCallback cb, void* userData, const PxBounds3& box, PxU32 flags)
{
//	const bool first_contact = (flags&PQF_FIRST_CONTACT)!=0;
	const bool first_contact = false;
	for(PxU32 i=0;i<2;i++)
		if(mPruners[i] && flags&(1<<i))
			if(!mPruners[i]->overlapAABB(cb, userData, box, first_contact))
				return;
}

void SceneQueryManager::overlap(ReportPrunablesCallback cb, void* userData, const Gu::Box& box, PxU32 flags)
{
//	const bool first_contact = (flags&PQF_FIRST_CONTACT)!=0;
	const bool first_contact = false;
	for(PxU32 i=0;i<2;i++)
		if(mPruners[i] && flags&(1<<i))
			if(!mPruners[i]->overlapOBB(cb, userData, box, first_contact))
				return;
}

void SceneQueryManager::overlap(ReportPrunablesCallback cb, void* userData, const Gu::Capsule& capsule, PxU32 flags)
{
//	const bool first_contact = (flags&PQF_FIRST_CONTACT)!=0;
	const bool first_contact = false;
	for(PxU32 i=0;i<2;i++)
		if(mPruners[i] && flags&(1<<i))
			if(!mPruners[i]->overlapCapsule(cb, userData, capsule, first_contact))
				return;
}

///////////////////////////////////////////////////////////////////////////////

static Pruner* CreatePruner(PxPruningStructure::Enum type)
{
	switch(type)
	{
		case PxPruningStructure::eSTATIC_AABB_TREE:		return PX_NEW(StaticPruner);
		case PxPruningStructure::eOCTREE:				return CreateOctreePruner();
		case PxPruningStructure::eQUADTREE:				return CreateQuadtreePruner();
		case PxPruningStructure::eNONE:					return PX_NEW(FreePruner);
		case PxPruningStructure::eDYNAMIC_AABB_TREE:	return PX_NEW(DynamicPruner2);
	}
	return NULL;
}

SceneQueryManager::SceneQueryManager(Scb::Scene& scene, const PxSceneDesc& desc) :
	mScene			(scene),
	mNumShapes		(0),
	mMaxObjectTags	(0),
#ifdef USE_MY_POOL
#else
	mShapeSqDataPool(PX_DEBUG_EXP("SQMSqDataPool")),
#endif
	mSweepCaches	(PX_DEBUG_EXP("SQMSweepCaches")),
	mDirtyShapes	(PX_DEBUG_EXP("SQMDirtyShapes"))
{
	mDesc.maxBounds						= desc.maxBounds;
	mDesc.maxNbStaticShapes				= desc.limits.maxNbStaticShapes;
	mDesc.maxNbDynamicShapes			= desc.limits.maxNbDynamicShapes;
	mDesc.upAxis						= desc.upAxis;
	mDesc.subdivisionLevel				= desc.subdivisionLevel;
	mDesc.dynamicStructure				= desc.dynamicStructure;
	mDesc.staticStructure				= desc.staticStructure;
	mDesc.dynamicTreeRebuildRateHint	= desc.dynamicTreeRebuildRateHint;

	mQueryBufferTlsSlot = Ps::TlsAlloc();
	{
		PRUNERCREATE create;
		create.mStaticType	= mDesc.staticStructure;
		create.mDynamicType	= mDesc.dynamicStructure;

		if(mDesc.staticStructure==PxPruningStructure::eQUADTREE || mDesc.staticStructure==PxPruningStructure::eOCTREE)
		{
			PX_ASSERT(!mDesc.maxBounds.isEmpty());
		}

		if(!mDesc.maxBounds.isEmpty())
			create.mExpectedWorldBox = mDesc.maxBounds;

		create.mUpAxis				= mDesc.upAxis;
		create.mSubdivisionLevel	= mDesc.subdivisionLevel;
		create.mNbStaticObjects		= mDesc.maxNbStaticShapes;
		create.mNbDynamicObjects	= mDesc.maxNbDynamicShapes;

		// Allocate pruners
		mPruners[0] = CreatePruner(create.mStaticType);
		mPruners[0]->Setup(create);
		mPruners[0]->Init(create.mNbStaticObjects);

		mPruners[1] = CreatePruner(create.mDynamicType);
		mPruners[1]->Setup(create);
		mPruners[1]->Init(create.mNbDynamicObjects);
	}

	setDynamicTreeRebuildRateHint(mDesc.dynamicTreeRebuildRateHint);

	// PT: pre-allocate pool if needed
	const PxU32 totalNbShapes = desc.limits.maxNbStaticShapes + desc.limits.maxNbDynamicShapes;
	if(totalNbShapes)
	{
		mShapeSqDataPool.preAllocate(totalNbShapes);
	}
}

SceneQueryManager::~SceneQueryManager()
{
	// Release all pruners
	for(PxU32 i=0;i<2;i++)
		PX_DELETE_AND_RESET(mPruners[i]);

	PX_ASSERT(mNumShapes == 0);
	
	// release sweep caches
	PxU32 numSweepCaches = mSweepCaches.size();
	while(numSweepCaches--)
	{
		releaseSweepCache(mSweepCaches[numSweepCaches]);
	}

	// release query buffers from pool
	QueryBuffers* buffers = static_cast<QueryBuffers*>(mQueryBuffersPool.flush());
	while (buffers != NULL)
	{
		QueryBuffers* nextBuffers = static_cast<QueryBuffers*>(buffers->next());
		
		//check we are not releasing an in use context.
		PX_ASSERT(buffers->getRefCount() == 0);
		PX_DELETE(buffers);
		buffers = nextBuffers;
	}

	// release tls slot
	Ps::TlsFree(mQueryBufferTlsSlot);
}

SceneQueryShapeData* SceneQueryManager::addShape(const NpShape& shape, bool dynamic)
{
	SceneQueryShapeData* data = mShapeSqDataPool.construct(*this);

	// PT: TODO: what's the point of initializing "hitObject" here?
	data->shape = const_cast<NpShape*>(&shape);

	PxActor& actor = shape.getActorFast();

	data->sqGlobalPose = Sq::getGlobalPose(shape);

	// filterShape
	data->actorClientID = actor.getOwnerClient();					// ### VIRTUAL
	data->actorClientBehaviorBits = actor.getClientBehaviorBits();	// ### VIRTUAL

	data->attr = getFilterAttributesFast(shape);
	data->queryFilterData = shape.getQueryFilterDataFast();

	data->shapeGeometryEA = (Gu::GeometryUnion*)(&shape.getGeometryFast());

	Prunable& prn = *data;
	prn.SetDynamicClearDirty(dynamic);

	Cm::TakeWriterLock lock(mSceneQueryLock);
	AddObject(prn);
	growObjectTags(++mNumShapes);

	return data;
}

#ifndef NDEBUG
// PT: this function is only used in Debug to assert the type is what we expect
static bool checkIsDynamic(PxType expectedType, bool isDynamicExpected, PxActor* actor)
{
	const PxType actorType = actor->getSerialType();
	if(actorType!=expectedType)
		return false;
	const bool isDynamic = actorType == PxSerialType::eRIGID_DYNAMIC || actorType == PxSerialType::eARTICULATION_LINK;
	return isDynamic == isDynamicExpected;
}
#endif

// PT: this is really the same as Sq::getGlobalPose() specialized for static shapes
static PX_INLINE void getGlobalPose(PxTransform& transform, const NpShape& shape, const NpRigidStatic& npRigidStatic)
{
	// Same as:
//	transform = npRigidStatic.getGlobalPoseFast() * shape.getLocalPose();

	// Same as:
//	const PxTransform& globalPose = npRigidStatic.getGlobalPoseFast();
//	const PxTransform localPose = globalPose.transformInv(shape.getScbShape().getShape2Body());
//	transform = globalPose * localPose;

	// Same as:
	transform = shape.getScbShape().getShape2Body();
}

// PT: this is really the same as Sq::getGlobalPose() specialized for dynamic shapes
static PX_INLINE void getGlobalPose(PxTransform& transform, const NpShape& shape, const NpRigidDynamic& npRigidDynamic)
{
	// Same as:
//	transform = npRigidDynamic.getGlobalPoseFast() * shape.getLocalPose();

	// Same as:
//	transform = npRigidDynamic.getGlobalPoseFast() * npRigidDynamic.getCMassLocalPoseFast() * shape.getScbShape().getShape2Body();

	// Same as:
	transform = npRigidDynamic.getScbBodyFast().getBody2World() * shape.getScbShape().getShape2Body();
}

#define MAX_BATCHED_SHAPES	1024
void SceneQueryManager::addShapes(PxU32 nbShapes, NpShape*const* PX_RESTRICT shapes, PxActor** PX_RESTRICT owners, PxType actorType, bool isDynamic)
{ 
	if(!nbShapes)
		return;

	PX_ASSERT(nbShapes<=MAX_BATCHED_SHAPES);

	// PT: same pruner for all objects here
	Pruner* PX_RESTRICT pruner = mPruners[isDynamic];
	PX_ASSERT(pruner);

	// PT: ok, here's the thing: we're going to compute sqGlobalPose once when creating the SceneQueryShapeData object, and then
	// immediately add it to a pruner, which in most cases will end up calling Prunable::GetWorldAABB, which for some reason recomputes s->sqGlobalPose
	// (even though the original code didn't). So we end up computing the pose twice, and the code doing so is (currently) criminally lame. To avoid
	// redundant computations we skip them here if the pruner the shapes belong to will recompute the poses.
//	const bool needsToComputePose = !pruner->updateRecomputesPose();

	PxU32 nbToAdd = 0;
	SceneQueryShapeData* sqDataPtrs[MAX_BATCHED_SHAPES];
	Ps::invalidateCache(sqDataPtrs, MAX_BATCHED_SHAPES*sizeof(SceneQueryShapeData*));

	// PT: figure out correct prefetch offset depending on type. TODO: complete this for articulations, etc
	PxU32 prefetchOffset0;
	PxU32 prefetchOffset1;
	if(actorType==PxSerialType::eRIGID_DYNAMIC)
	{
		// PT: we access 2 things from the actor pointer:
		prefetchOffset0 = (PxU32)(size_t)(((NpRigidDynamic*)0)->getScbBodyFast().getBodyCoreFast().getFlagsAddress());	// 188
		prefetchOffset1 = (PxU32)(size_t)(&((NpRigidDynamic*)0)->getScbBodyFast().getBody2World());						// 256
	}
	else
	{
		// PT: all right, bite the bullet and prefetch 2 cache lines. It's just too painful otherwise.
		prefetchOffset0 = 0;
		prefetchOffset1 = 128;
	}

	NpShape* nextShape = shapes[0];
	SceneQueryShapeData* nextData = mShapeSqDataPool.preAllocateAndPrefetch();
	PxActor* nextActor = owners[0];
	for(PxU32 i=0; i<nbShapes; i++)
	{
		NpShape* npShape = nextShape;
		SceneQueryShapeData* sqData = nextData;
		PxActor* actor = nextActor;
		if(i!=nbShapes-1)
		{
			nextShape = shapes[i+1];
			Ps::prefetch128(nextShape);

			nextData = mShapeSqDataPool.preAllocateAndPrefetch();

			nextActor = owners[i+1];
//			Ps::prefetch128(nextActor, prefetchOffset);	// PT: specialized to avoid L2 in getGlobalPose for dynamics
			Ps::prefetch128(nextActor, prefetchOffset0);
			Ps::prefetch128(nextActor, prefetchOffset1);
		}

		// PT: TODO: that test is useless, we should keep shapes sorted with SQ shapes first
		// At time of writing, the prefetch doesn't work here because we really read:
		// int(&npShape->mShape.mBufferFlags) - int(npShape) = 36 bytes away
		// int(&npShape->mShape.mShape.mCore.mShapeFlags) - int(npShape) = 140 bytes away
		if(npShape->getFlagsInternal() & PxShapeFlag::eSCENE_QUERY_SHAPE)
		{
			PX_ASSERT(checkIsDynamic(actorType, isDynamic, owners[i]));	// PT: check that the actor is the kind of actor we expect

			{
				SceneQueryShapeData* data = mShapeSqDataPool.construct(sqData, *this);	// ### addActor alloc

				// PT: TODO: what's the point of initializing "hitObject" here?
				data->shape = npShape;

//				PxActor& actor = *owners[i];	// PT: "shape.getActorFast()" is "fast", but it still reads 192 bytes away from the base address at time of writing

				// PT: the following isn't nice, but it's certainly faster than the clean-looking code
				// filterShape
				if(actorType==PxSerialType::eRIGID_STATIC)
				{
					NpRigidStatic& npRigidStatic = static_cast<NpRigidStatic&>(*actor);

//					if(needsToComputePose)
						::getGlobalPose(data->sqGlobalPose, *npShape, npRigidStatic);

					const Sc::RigidCore& rigidCore = npRigidStatic.getScbRigidStaticFast().getRigidCoreFast();
					data->actorClientID = rigidCore.getOwnerClientFast();
					data->actorClientBehaviorBits = rigidCore.getClientBehaviorBitsFast();

					data->attr = getFilterAttributesFast(*npShape, npRigidStatic);
				}
				else if(actorType==PxSerialType::eRIGID_DYNAMIC)
				{
					NpRigidDynamic& npRigidDynamic = static_cast<NpRigidDynamic&>(*actor);

					// PT: at time of writing we get a huge L2 here despite the prefetch, since "getScbBodyFast().getBody2World()" reads 272 bytes away from the base
//					if(needsToComputePose)
						::getGlobalPose(data->sqGlobalPose, *npShape, npRigidDynamic);

					const Sc::RigidCore& rigidCore = npRigidDynamic.getScbBodyFast().getRigidCoreFast();
					data->actorClientID = rigidCore.getOwnerClientFast();
					data->actorClientBehaviorBits = rigidCore.getClientBehaviorBitsFast();

					data->attr = getFilterAttributesFast(*npShape, npRigidDynamic);
				}
				else
				{
//					if(needsToComputePose)
						data->sqGlobalPose = Sq::getGlobalPose(*npShape);

					// PT: TODO: those calls have 3 virtual calls each!!! madness!
					data->actorClientID = actor->getOwnerClient();					// ### VIRTUAL
					data->actorClientBehaviorBits = actor->getClientBehaviorBits();	// ### VIRTUAL

					data->attr = getFilterAttributesFast(*npShape);
				}

				data->queryFilterData = npShape->getQueryFilterDataFast();
				data->shapeGeometryEA = (Gu::GeometryUnion*)(&npShape->getGeometryFast());

				data->SetDynamicClearDirty(isDynamic);

				npShape->setSqData(data);

				sqDataPtrs[nbToAdd++] = data;
			}
		}
	}

	{
		Cm::TakeWriterLock lock(mSceneQueryLock);
		pruner->addShapes(nbToAdd, sqDataPtrs);
		mNumShapes += nbToAdd;
		growObjectTags(mNumShapes);
	}
}

void SceneQueryManager::removeShape(NpShape& shape)
{
	Cm::TakeWriterLock lock(mSceneQueryLock);

	SceneQueryShapeData* data = shape.getSqData();
	if(data->IsSet(PRN_SQ_DIRTY))
		mDirtyShapes.findAndReplaceWithLast(&shape);

	mNumShapes--;
	RemoveObject(*data);
	mShapeSqDataPool.destroy(data);
}

PxSweepCache* SceneQueryManager::createSweepCache(PxReal dimensions)
{
	PxSweepCache* cache = PX_NEW(ObjectCache)(this,dimensions);
	mSweepCaches.pushBack(cache);
	return cache;
}

void SceneQueryManager::releaseSweepCache(PxSweepCache* sweepCache)
{
	mSweepCaches.findAndReplaceWithLast(sweepCache);
	ObjectCache* cache = static_cast<ObjectCache*>(sweepCache);
	PX_DELETE(cache);
}

void SceneQueryManager::setDynamicTreeRebuildRateHint(PxU32 dynTreeRebuildRateHint)
{
	mDesc.dynamicTreeRebuildRateHint = dynTreeRebuildRateHint;

	if ((mDesc.dynamicStructure == PxPruningStructure::eDYNAMIC_AABB_TREE) && getDynamicPruner())
	{
		DynamicPruner2* pruner = (DynamicPruner2*)getDynamicPruner();
		pruner->SetRebuildRateHint(dynTreeRebuildRateHint);
	}
}

#define USE_BODY_ITERATOR
#ifndef USE_BODY_ITERATOR
#include "ScBodySim.h"
#endif

void SceneQueryManager::eagerUpdatePruningTrees()
{
	Cm::TakeWriterLock lock(mSceneQueryLock);

	// update all active objects
#ifdef USE_BODY_ITERATOR
	Sc::BodyIterator actorIterator;
	mScene.initActiveBodiesIterator(actorIterator);

	Sc::BodyCore* b = NULL;
	while((b = actorIterator.getNext()))
	{
#else
	Sc::Range<Sc::Actor*const> range = mScene.getScScene().getInteractionScene().getActiveActors();
	for(; !range.empty(); range.popFront())		
	{
		Sc::Actor* const activeActor = range.front();

		if(!activeActor->isDynamicRigid())
			continue;

		Sc::BodyCore* b = &static_cast<Sc::BodySim*>(activeActor)->getCore();
#endif
		Sc::ShapeIterator shapeIterator;
		mScene.initActiveShapesIterator(*b, shapeIterator);
//		shapeIterator.init(b->getSim());

		Sc::ShapeCore* s = NULL;
		while ((s = shapeIterator.getNext()))
		{
			if(s->getFlags() & PxShapeFlag::eSCENE_QUERY_SHAPE)
			{
				NpShape* shape = static_cast<NpShape*>(s->getPxShape());
				SceneQueryShapeData* data = shape->getSqData();
				if(!data->IsSet(PRN_SQ_DIRTY))	// PT: if dirty, will be updated in "flushUpdates"
					UpdateObject(*data);
			}
		}
	}

	// flush user modified objects
	flushUpdates();

	for(PxU32 i=0;i<2;i++)
		if(mPruners[i])
			mPruners[i]->eagerUpdatePruningTrees();
}

void SceneQueryManager::flushUpdates()
{
	CM_PROFILE_ZONE_WITH_SUBSYSTEM(mScene,SceneQuery,flushUpdates);

	Cm::TakeWriterLock lock(mSceneQueryLock);

	PxU32 numDirtyShapes = mDirtyShapes.size();
	for(PxU32 i = 0; i < numDirtyShapes; i++)
	{
		NpShape* shape = mDirtyShapes[i];
		SceneQueryShapeData* data = shape->getSqData();
		PX_ASSERT(data->IsSet(PRN_SQ_DIRTY));
		data->ClearSQDirtyFlag();
		UpdateObject(*data);
	}
	mDirtyShapes.clear();
}

void SceneQueryManager::growObjectTags(PxU32 nbStamps)
{
	Ps::atomicMax((PxI32*)&mMaxObjectTags, nbStamps);
}

QueryBuffers* SceneQueryManager::aquireQueryBuffers() const
{
	QueryBuffers* buffer = static_cast<QueryBuffers*>(Ps::TlsGet(mQueryBufferTlsSlot));

	if (buffer == NULL)
	{
		buffer = static_cast<QueryBuffers*>(mQueryBuffersPool.pop());
		if (buffer == NULL)
			buffer = PX_NEW(QueryBuffers)();

		Ps::TlsSet(mQueryBufferTlsSlot, buffer);
	}

	/*
	  Ideally we would remove all recursive context acquisition. and reduce this to an SList push/pop
	  However that is quite dangerous and we would have too look at bounds computation carefully.
	 */
	buffer->addRef();

	return buffer;
}

void SceneQueryManager::releaseQueryBuffers(QueryBuffers* b) const
{
	b->decRef();
	if (b->getRefCount() == 0)
	{
		Ps::TlsSet(mQueryBufferTlsSlot, NULL);
		mQueryBuffersPool.push(*b);
	}
}

#include "CmRenderOutput.h"
void SceneQueryManager::visualize(Cm::RenderOutput& out, bool visStatic, bool visDynamic)
{
	if(visStatic && mPruners[0])
		mPruners[0]->visualize(out, PxDebugColor::eARGB_BLUE);
	if(visDynamic && mPruners[1])
		mPruners[1]->visualize(out, PxDebugColor::eARGB_RED);
}

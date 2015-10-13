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


// TODO:
// - does the swapcallback post fix really work? Didn't handles change?
// the new mapping maps the *new pool* to the new tree, why should we fix the pool again?

// - stress test create/update/delete random objects each frame
// - temp hack....
// * check mAdded removal => always ok?
// * fix mAddedSize when removing from mAdded
// * queries vs mAdded
// * skip work when objects didn't move => important if we use this for statics as well
// - optimize full refit for new tree => partial refit
// * optimize "AddUnique"
// - heuristics/etc for progressive rebuild
// * bit arrays & refit optims
// - recycle memory buffers/limit realloc
// - words in mRecorded
// - progressive mapping? ie computed in AABB-tree during build

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "PsIntrinsics.h"
#include "PsUserAllocated.h"
#include "PsBitUtils.h"
#include "SqDynamicPruner2.h"
#include "SqFreePruner.h"
#include "OPC_AABBTree.h"

using namespace physx;
using namespace Ice;
using namespace Sq;

//#define MODIFIED_POINTERS

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

DynamicPruner2::DynamicPruner2() :
	mAllowBuild			(false),
	mAllowRecord		(false),
	mMapping			(NULL),
	mNbMappingEntries	(0),
	mNbCalls			(0),
	mNewTree			(NULL),
	mCachedBoxes		(NULL),
	mNbCachedBoxes		(0),
	mProgress			(BUILD_NOT_STARTED),
	mAddedSize			(0),
	mRebuildRateHint	(100),
	mAdaptiveRebuildTerm (0),
	mAdded				(mAddedContainer)
{
	mCullFunc			= (CullFunc)			&DynamicPruner2::Cull;
	mStabFunc			= (StabFunc)			&DynamicPruner2::Stab;
	mOverlapSphereFunc	= (OverlapSphereFunc)	&DynamicPruner2::OverlapSphere;
	mOverlapAABBFunc	= (OverlapAABBFunc)		&DynamicPruner2::OverlapAABB;
	mOverlapOBBFunc		= (OverlapOBBFunc)		&DynamicPruner2::OverlapOBB;
	mOverlapCapsuleFunc	= (OverlapCapsuleFunc)	&DynamicPruner2::OverlapCapsule;
}

DynamicPruner2::~DynamicPruner2()
{
	Release();
}

void DynamicPruner2::Release()
{
	mAddedContainer.Reset();
	mRecorded.Reset();

	PX_FREE_AND_RESET(mMapping);
	mNbMappingEntries = 0;
#ifndef SUPPORT_REFIT_BITMASK
	mModifiedIndices.Reset();
#endif

	PX_FREE_AND_RESET(mCachedBoxes);
	mBuilder.Reset();
	PX_DELETE_AND_RESET(mNewTree);
	PX_DELETE_AND_RESET(mAABBTree);

	mAddedSize = mNbCachedBoxes = 0;
	mProgress = BUILD_NOT_STARTED;
}

// Maps handles to AABB-tree indices (i.e. locates the object's box in the aabb-tree nodes pool)
void DynamicPruner2::ComputeMapping()
{
	if(mMapping)	return;

	PxU32 NbObjects = GetNbActiveObjects();
	if(!NbObjects)	return;

	if(mNbCachedBoxes>NbObjects)
		NbObjects = mNbCachedBoxes;

	PxU32 mapSize = NbObjects + (NbObjects >> 2);  // Add a bit more to delay resizing
	mMapping = (PxU32*)PX_ALLOC(sizeof(PxU32)*mapSize);

	mNbMappingEntries = mapSize;
	for(PxU32 i=0;i<mapSize;i++)
		mMapping[i] = PX_INVALID_U32;

	const AABBTree* T = GetAABBTree();
	PxU32 NbNodes = T->GetNbNodes();
	const AABBTreeNode*	Nodes = T->GetNodes();
	for(PxU32 i=0;i<NbNodes;i++)
	{
		if(Nodes[i].IsLeaf() && Nodes[i].GetPrimitives())
		{
			PX_ASSERT(Nodes[i].GetNbPrimitives()==1);
			PxU32 Index = Nodes[i].GetPrimitives()[0];
			PX_ASSERT(Index<NbObjects);		// temp hack
			mMapping[Index] = i;
		}
	}
}

bool DynamicPruner2::CheckTree() const
{
	const AABBTree* T = HasAABBTree();
	if(!T)	return true;
	const AABBTreeNode* Nodes = T->GetNodes();
	PxU32 Nb = T->GetNbNodes();
	Prunable** Objects = GetObjects();
	for(PxU32 i=0;i<Nb;i++)
	{
		const AABBTreeNode& CurrentNode = Nodes[i];
		if(CurrentNode.IsLeaf() && CurrentNode.GetNbPrimitives())
		{
			Prunable* P = Objects[*CurrentNode.GetPrimitives()];
/*			if(P->GetPruningSection()==PRP_LOW)
			{
				return false;
			}*/
		}
	}
	return true;
}

bool DynamicPruner2::CheckMapping() const
{
	if(!mMapping)	return false;
	PxU32 NbObjects = GetNbActiveObjects();

	const AABBTree* T = HasAABBTree();
	const AABBTreeNode* Nodes = T->GetNodes();
	for(PxU32 i=0;i<NbObjects;i++)
	{
		PxU32 Index = mMapping[i];
/*		ASSERT(Index<T->GetNbNodes());
		ASSERT(Nodes[Index].IsLeaf());
		ASSERT(Nodes[Index].GetPrimitives());
//		ASSERT(Nodes[Index].GetNbPrimitives()==1);
		ASSERT(i == Nodes[Index].GetPrimitives()[0]);*/
		if(i != Nodes[Index].GetPrimitives()[0])	return false;
	}
	return true;
}

static void gSwapCallback(PxU32 old_index, PxU32 new_index, void* user_data)
{
	DynamicPruner2* DP = (DynamicPruner2*)user_data;

	PxU32* Mapping = DP->GetMapping();
	if(!Mapping)
		return;

	if(DP->AllowRecording())
	{
		DP->GetRecords().Add(old_index).Add(new_index);
	}

	if(old_index==PX_INVALID_U32)
	{
		// New object => doesn't exist in the tree => we can't map it to the tree

		// Resize mapping array if needed
		Mapping = DP->ResizeMapping(new_index);

		// Mark as invalid in the tree
		Mapping[new_index] = PX_INVALID_U32;
	}
	else if(new_index==PX_INVALID_U32)
	{
		// Removed object

		Mapping = DP->ResizeMapping(old_index);
		PX_ASSERT(old_index<DP->GetNbMappingEntries());

		PxU32 Index = Mapping[old_index];
		if(Index!=PX_INVALID_U32)
		{
			const AABBTree* T = DP->HasAABBTree();
			if(T)
			{
				const AABBTreeNode* Nodes = T->GetNodes();

				PX_ASSERT(Index<T->GetNbNodes());
				PX_ASSERT(Nodes[Index].IsLeaf());
				PX_ASSERT(Nodes[Index].GetPrimitives());
				PX_ASSERT(Nodes[Index].GetNbPrimitives()==1);
				PX_ASSERT(old_index == Nodes[Index].GetPrimitives()[0]);
				const_cast<AABBTreeNode*>(Nodes)[Index].mNbPrimitives = 0;

				PxU32* primitive = const_cast<PxU32*>(Nodes[Index].GetPrimitives());
				*primitive = PX_INVALID_U32;	// Mark primitive index as invalid
			}
		}
	}
	else
	{
		// Moved object
		Mapping = DP->ResizeMapping(old_index);
		Mapping = DP->ResizeMapping(new_index);
		PX_ASSERT(old_index<DP->GetNbMappingEntries());
		PX_ASSERT(new_index<DP->GetNbMappingEntries());

		PxU32 Index0 = Mapping[old_index];
		PxU32 Index1 = Mapping[new_index];

		// Swap primitive indices stored in the tree nodes
		const AABBTree* T = DP->HasAABBTree();
		if(T)
		{
			const AABBTreeNode* Nodes = T->GetNodes();

			if ((Index0!=PX_INVALID_U32) && (Nodes[Index0].GetPrimitives()[0] != PX_INVALID_U32))
			{
				PX_ASSERT(Nodes[Index0].GetPrimitives()[0] == old_index);
//				PX_ASSERT(new_index<DP->GetNbActiveObjects());
				const_cast<AABBTreeNode*>(Nodes)[Index0].mNodePrimitives[0] = new_index;
			}
			if ((Index1!=PX_INVALID_U32) && (Nodes[Index1].GetPrimitives()[0] != PX_INVALID_U32))
			{
				PX_ASSERT(Nodes[Index1].GetPrimitives()[0] == new_index);
//				PX_ASSERT(old_index<DP->GetNbActiveObjects());
				const_cast<AABBTreeNode*>(Nodes)[Index1].mNodePrimitives[0] = old_index;
			}
		}

		// Swap mapping indices
		PX_ASSERT(old_index<DP->GetNbMappingEntries());
		PX_ASSERT(new_index<DP->GetNbMappingEntries());
		Mapping[old_index] = Index1;
		Mapping[new_index] = Index0;
	}
}

PxU32* DynamicPruner2::ResizeMapping(PxU32 new_index)
{
	if(new_index>=mNbMappingEntries)
	{
		PxU32 newSize = new_index + (new_index >> 2) + 1;	// Note: If the growing scheme gets adjusted, consider
															//       adjusting the start size in ComputeMapping() as well.
		PxU32* NewMapping = (PxU32*)PX_ALLOC(sizeof(PxU32)*newSize);

		for(PxU32 i=mNbMappingEntries; i < newSize; i++)
			NewMapping[i] = PX_INVALID_U32;

		if(mNbMappingEntries)
			Ps::memCopy(NewMapping, mMapping, mNbMappingEntries*sizeof(PxU32));

		PX_FREE_AND_RESET(mMapping);
		mMapping = NewMapping;

		mNbMappingEntries = newSize;
	}
	return mMapping;
}


//TODO: Check this is correct/thread safe(DynamicPruner2 is not used anyway)
void DynamicPruner2::eagerUpdatePruningTrees()
{
	ComputeMapping();
//	CheckMapping();

	StaticPruner::eagerUpdatePruningTrees();

	if(mProgress!=BUILD_FINISHED)	// Else we're going to switch to a new tree this frame, so don't bother...
	{
		Refit();
	}

	if(mAllowBuild)
	{
		if(mProgress==BUILD_NOT_STARTED)
		{
			PxU32 NbObjects = GetNbActiveObjects();
			if(!NbObjects)	return;

			PX_DELETE(mNewTree);
			mNewTree = PX_NEW(AABBTree);

			mNbCachedBoxes = NbObjects;
			mCachedBoxes = (PxBounds3*)PX_ALLOC(sizeof(PxBounds3)*NbObjects);

			Prunable** Objects = GetObjects();
			for(PxU32 i=0;i<NbObjects;i++)
			{
				Prunable* Current = *Objects++;
				PX_ASSERT(Current->GetHandle()==i);
				mCachedBoxes[i] = *GetWorldAABB(*Current);
			}
			// Objects currently in mAdded will be part of the new tree. However more objects can
			// get added while we compute the new tree, and those ones will not be part of it.
			mAddedSize = mAdded.GetNbPrunables();

			mBuilder.Reset();
			mBuilder.mNbPrimitives		= mNbCachedBoxes;
			mBuilder.mAABBArray			= mCachedBoxes;
			mBuilder.mSettings.mRules	= SPLIT_SPLATTER_POINTS|SPLIT_GEOM_CENTER;
			mBuilder.mSettings.mLimit	= 1;
			mRecorded.Reset();		// ########################
			mProgress = BUILD_INIT;
		}
		else if(mProgress==BUILD_INIT)
		{
			mNewTree->Build(&mBuilder, 0, 0);
			mProgress = BUILD_IN_PROGRESS;
			mNbCalls = 0;

			// Use a heuristic to estimate the number of work units needed for rebuilding the tree.
			// The general idea is to use the number of work units of the previous tree to build the new tree.
			// This works fine as long as the number of leaves remains more or less the same for the old and the
			// new tree. If that is not the case, this estimate can be way off and the work units per step will
			// be either much too small or too large. Hence, in that case we will try to estimate the number of work
			// units based on the number of leaves of the new tree as follows:
 			//
			// - Assume new tree with n leaves is perfectly-balanced
			// - Compute the depth of perfectly-balanced tree with n leaves
			// - Estimate number of working units for the new tree

			PxU32 depth = Ps::ilog2(mBuilder.mNbPrimitives);	// Note: This is the depth without counting the leaf layer
			PxU32 estimatedNbWorkUnits = depth * mBuilder.mNbPrimitives;	// Estimated number of work units for new tree
			PxU32 estimatedNbWorkUnitsOld = mAABBTree->GetTotalPrims();
			if ((estimatedNbWorkUnits <= (estimatedNbWorkUnitsOld << 1)) && (estimatedNbWorkUnits >= (estimatedNbWorkUnitsOld >> 1)))
			{	// The two estimates do not differ by more than a factor 2

				mTotalWorkUnits = estimatedNbWorkUnitsOld;
			}
 			else
			{
 				mAdaptiveRebuildTerm = 0;
				mTotalWorkUnits = estimatedNbWorkUnits;
 			}
 
 			PxI32 totalWorkUnits = mTotalWorkUnits + (mAdaptiveRebuildTerm * mBuilder.mNbPrimitives);
 			mTotalWorkUnits = PxMax(totalWorkUnits, 0);
		}
		else if(mProgress==BUILD_IN_PROGRESS)
		{
			mNbCalls++;
			const PxU32 Limit = 1 + (mTotalWorkUnits / mRebuildRateHint);
			if(!mNewTree->Build(&mBuilder, 1, Limit))
			{
				// Done
				mProgress = BUILD_FINISHED;
			}
		}
		else
		{
			// Finalize
			PX_FREE_AND_RESET(mCachedBoxes);
			mProgress = BUILD_NOT_STARTED;

			// Adjust adaptive term to get closer to specified rebuild rate.
 			if (mNbCalls > mRebuildRateHint)
 				mAdaptiveRebuildTerm++;
 			else if (mNbCalls < mRebuildRateHint)
 				mAdaptiveRebuildTerm--;

			// Switch trees
			PX_DELETE(mAABBTree);
			mAABBTree = mNewTree;
			mNewTree = NULL;

			// Switch mapping
			PX_FREE_AND_RESET(mMapping);
			ComputeMapping();
			// The new mapping has been computed using only indices stored in the new tree. Those indices map the pruning pool
			// we had when starting to build the tree. We need to re-apply recorded swaps to fix the tree.

			// Fix tree
			PxU32 NbData = mRecorded.GetNbEntries();
			if(NbData)
			{
				mAllowRecord = false;

				const PxU32* Data = mRecorded.GetEntries();
				const PxU32* LastData = Data + NbData;
				while(Data!=LastData)
				{
					gSwapCallback(Data[0], Data[1], this);
					Data+=2;
				}

				mRecorded.Reset();
				mAllowRecord = true;
			}

			//### Full refit, to be replaced
			// We need to refit the new tree because objects may have moved while we were building it.
			PxU32 NbObjects = GetNbActiveObjects();
			Prunable** Objects = GetObjects();
			for(PxU32 i=0;i<NbObjects;i++)
			{
				Prunable* P = (*Objects++);

	//			if(!mRemoved.Contains((PxU32)P))
					GetWorldAABB(*P);
			}

			AABBTreeOfAABBsBuilder TB;
			TB.mNbPrimitives	= NbObjects;
			TB.mAABBArray		= GetCurrentWorldBoxes();
	//		TB.mRules			= SPLIT_COMPLETE|SPLIT_SPLATTER_POINTS;
			TB.mSettings.mRules	= SPLIT_SPLATTER_POINTS;
			TB.mSettings.mLimit	= 1;
			((AABBTree*)GetAABBTree())->Refit2(&TB);

#ifndef SUPPORT_REFIT_BITMASK
			mModifiedIndices.Reset();
#endif

			// The mAddedSize first objects can now be removed from the array
			{
				if(mAddedSize)
				{
					PxU32 NbAddedObjects = mAdded.GetNbPrunables();
					Prunable** AddedObjects = mAdded.GetPrunables();
					const PxU32 NewSize = NbAddedObjects - mAddedSize;
					// ### do overlapping CopyMemory work?
					for(PxU32 i=0;i<NewSize;i++)
						AddedObjects[i] = AddedObjects[i+mAddedSize];
					mAddedSize = 0;
					mAddedContainer.ForceSize(NewSize);
				}
			}

			mAllowBuild = false;

//			printf("New tree\n");
		}
	}
}

bool DynamicPruner2::AddObject(Prunable& object)
{
	if(mMapping)
	{
		mAllowBuild = true;

		// The new objects will not be in the tree right away, but we still need to collide/raycast/etc
		// against them. We will do that manually for a few frames, until they are inserted in the tree.
//		if(object.GetPruningSection()!=PRP_LOW)
			mAdded.AddPrunable(&object);

		// The new objects *must* be inserted in a pool anyway, immediately, because the outside world
		// might want to access their bounds right away. And the bounds are actually stored in the pool.
		//
		// We insert the new object in *current* pool, which has severe consequences:
		// - objects within the pool might be swapped, and their handles changed
		// - those changes have to be reflected in the handle-to-tree mapping
		// - those changes have to be reflected in the current AABB-tree
		// - those changes have to be reflected in the AABB-tree being rebuilt
		mAllowRecord = true;
//		PruningPool::AddObject(object, object.GetPruningSection()==PRP_LOW ? NULL : gSwapCallback, this);
		PruningPool::AddObject(object, gSwapCallback, this);

		// Skip StaticPruner level since it deletes the tree.
		// Skip Pruner level since it adds the object to the current pool (without the callback)
		// So we just copy here whatever is left from the Pruner level.
		mSignature.Invalidate();

		return true;
	}
	else
	{
		// Initial call when the tree has never been created
//		Release();
//if(object.GetPruningSection()!=PRP_LOW)
		return StaticPruner::AddObject(object);
//else return true;
	}
}

bool DynamicPruner2::RemoveObject(Prunable& object)
{
	mAllowBuild = true;

//	ASSERT(!mModifiedIndices.GetNbEntries());

#ifdef SUPPORT_REFIT_BITMASK
	if(object.GetHandle()<mNbMappingEntries)
	{
		PxU32 Index = mMapping[object.GetHandle()];
		if(Index!=PX_INVALID_U32)
			mAABBTree->MarkForRefit(Index);
	}
#else
	#ifdef MODIFIED_POINTERS
	mModifiedIndices.Delete(PxU32(&object));
	#else
//	mModifiedIndices.Delete(mMapping[object.GetHandle()]);
	PxU32 Index = mMapping[object.GetHandle()];
	if(Index!=PX_INVALID_U32)
		mModifiedIndices.AddUnique(Index);
	#endif
#endif
//	printf("Delete object\n");

	PxU32 Pos = mAddedContainer.DeleteKeepingOrder(size_t(&object));	// ### mAddedSize?
	if(Pos<mAddedSize)	mAddedSize--;

	// ### we must still do the swaps when Pos!=PX_INVALID_U32, because removing the object from the PruningPool still changes the handles
	// for objects inside the mapping/tree.
	bool UseCB = true;
//	if(object.GetPruningSection()==PRP_LOW/* || Pos!=PX_INVALID_U32*/)
//		UseCB=false;

	mAllowRecord = true;
	PruningPool::RemoveObject(object, UseCB ? gSwapCallback : NULL, this);

	mSignature.Invalidate();

	PxU32 NbObjects = GetNbActiveObjects();
	if(!NbObjects)
	{
		Release();
	}

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Updates an object, i.e. updates the pruner's spatial database.
 *	\param		object	[in] the object to update
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DynamicPruner2::UpdateObject(Prunable& object)
{
	mAllowBuild = true;

	GetWorldAABB(object);

/*	// ### do that faster!
	// ### still useful??
	if(mAdded.Contains((size_t)&object))
	{
		PxU32 i = mMapping[object.GetHandle()];

		// Object is not in current tree
		return Pruner::UpdateObject(object);
	}
*/
	if(mMapping)
	{
#ifdef SUPPORT_REFIT_BITMASK
		if(object.GetHandle()<mNbMappingEntries)
		{
			PxU32 i = mMapping[object.GetHandle()];
			if(i!=PX_INVALID_U32)
				mAABBTree->MarkForRefit(i);
		}
#else
	#ifdef MODIFIED_POINTERS
		mModifiedIndices.AddUnique(PxU32(&object));
	#else
		PxU32 i = mMapping[object.GetHandle()];
//		ASSERT(i!=PX_INVALID_U32);
		if(i!=PX_INVALID_U32)
			mModifiedIndices.AddUnique(i);
	#endif
#endif
	}

	// Skip the StaticPruner level since it deletes the tree
	return Pruner::UpdateObject(object);
}

void DynamicPruner2::addShapes(PxU32 nbShapes, SceneQueryShapeData*const* PX_RESTRICT shapes)
{
	// PT: call generic code for now
	Pruner::addShapes(nbShapes, shapes);
}


// Refit current tree
void DynamicPruner2::Refit()
{
	const AABBTree* Tree = GetAABBTree();
	if(!Tree)	return;

// TEST!
#ifdef _DEBUG
	struct Local
	{
		static void _Walk(const AABBTreeNode* parent_node, const AABBTreeNode* current_node, const AABBTreeNode* root)
		{
			if(!current_node)	return;

			if(parent_node)
			{
				PX_ASSERT(size_t(parent_node) < size_t(current_node));
			}

			if(current_node->GetPos())	{ _Walk(current_node, current_node->GetPos(), root);	}
			if(current_node->GetNeg())	{ _Walk(current_node, current_node->GetNeg(), root);	}
		}
	};
	Local::_Walk(NULL, Tree->GetNodes(), Tree->GetNodes());
#endif


	//### missing a way to skip work if not needed

	PxU32 NbObjects = GetNbActiveObjects();
	if(!NbObjects)	return;

	#ifdef MODIFIED_POINTERS
	for(PxU32 i=0;i<mModifiedIndices.GetNbEntries();i++)
	{
		Prunable* P = (Prunable*)mModifiedIndices[i];
		uword Handle = P->GetHandle();
		ASSERT(Handle<NbObjects);
		mModifiedIndices[i] = mMapping[Handle];
	}
	#endif
//	printf("Refit modified\n");
#ifdef SUPPORT_REFIT_BITMASK
	((AABBTree*)Tree)->RefitMarked(NbObjects, GetCurrentWorldBoxes());
#else
	((AABBTree*)Tree)->Refit3(NbObjects, GetCurrentWorldBoxes(), mModifiedIndices);
/*		AABBTreeOfAABBsBuilder TB;
		TB.mNbPrimitives	= NbObjects;
		TB.mAABBArray		= GetCurrentWorldBoxes();
//		TB.mRules			= SPLIT_COMPLETE|SPLIT_SPLATTER_POINTS;
		TB.mSettings.mRules	= SPLIT_SPLATTER_POINTS;
		TB.mSettings.mLimit	= 1;
		((AABBTree*)Tree)->Refit2(&TB);
*/
	mModifiedIndices.Reset();
#endif
}

bool DynamicPruner2::Cull(PruningTemps& temps, CulledObjects& objects, const Gu::Plane* planes, PxU32 nb_planes, PxU32 culling_flags)
{
	if(mAdded.GetNbPrunables())
	{
		if(!cullObjects(this, mAdded.GetPrunables(), mAdded.GetNbPrunables(), objects, planes, nb_planes, culling_flags))
			return false;
	}
	return StaticPruner::Cull(temps, objects, planes, nb_planes, culling_flags);
}

PxU32 DynamicPruner2::Stab(StabCallback callback, void* user_data, const PxVec3& orig, const PxVec3& dir, float& max_dist)
{
	PxU32 Status = StaticPruner::Stab(callback, user_data, orig, dir, max_dist);
	if(!(Status & STAB_STOP) && mAdded.GetNbPrunables())
	{
		PxU32 Status2 = stabObjects(this, mAdded.GetPrunables(), mAdded.GetNbPrunables(), callback, user_data, orig, dir, max_dist);
		Status |= Status2;
	}
	return Status;
}

bool DynamicPruner2::OverlapSphere(ReportPrunablesCallback cb, void* userData, const Gu::Sphere& sphere, bool first_contact)
{
	if(mAdded.GetNbPrunables())
	{
		if(!overlapObjects(this, mAdded.GetPrunables(), mAdded.GetNbPrunables(), cb, userData, sphere, first_contact))
			return false;
	}
	return StaticPruner::OverlapSphere(cb, userData, sphere, first_contact);
}

bool DynamicPruner2::OverlapAABB(ReportPrunablesCallback cb, void* userData, const PxBounds3& box, bool first_contact)
{
	if(mAdded.GetNbPrunables())
	{
		if(!overlapObjects(this, mAdded.GetPrunables(), mAdded.GetNbPrunables(), cb, userData, box, first_contact))
			return false;
	}
	return StaticPruner::OverlapAABB(cb, userData, box, first_contact);
}

bool DynamicPruner2::OverlapOBB(ReportPrunablesCallback cb, void* userData, const Gu::Box& box, bool first_contact)
{
	if(mAdded.GetNbPrunables())
	{
		if(!overlapObjects(this, mAdded.GetPrunables(), mAdded.GetNbPrunables(), cb, userData, box, first_contact))
			return false;
	}
	return StaticPruner::OverlapOBB(cb, userData, box, first_contact);
}

bool DynamicPruner2::OverlapCapsule(ReportPrunablesCallback cb, void* userData, const Gu::Capsule& capsule, bool first_contact)
{
	if(mAdded.GetNbPrunables())
	{
		if(!overlapObjects(this, mAdded.GetPrunables(), mAdded.GetNbPrunables(), cb, userData, capsule, first_contact))
			return false;
	}
	return StaticPruner::OverlapCapsule(cb, userData, capsule, first_contact);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "CmRenderOutput.h"
void DynamicPruner2::visualize(Cm::RenderOutput& out, PxU32 color)
{
	// Render current tree (being refit)
	StaticPruner::visualize(out, color);

	// Render added objects not yet in the tree
	PxU32 nb = mAdded.GetNbPrunables();
	Prunable** Objects = mAdded.GetPrunables();

	PxTransform idt = PxTransform::createIdentity();
	out << idt;
	out << PxDebugColor::eARGB_YELLOW;
	while(nb--)
	{
		Prunable* Current = *Objects++;
		out << Cm::DebugBox(*GetWorldAABB(*Current), true);
	}

}
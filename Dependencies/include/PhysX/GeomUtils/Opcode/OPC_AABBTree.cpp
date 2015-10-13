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


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "PsIntrinsics.h"
#include "PsUserAllocated.h"
#include "PsMathUtils.h"
#include "./Ice/CTC_PlaneAABBOverlap.h"
#include "./Ice/IceContainer.h"
#include "OPC_AABBTree.h"
#include "OPC_TreeBuilders.h"

using namespace physx;
using namespace Ice;

PX_INLINE void	StartProfile(PxU32& val)
	{
#if defined(PX_WINDOWS) && !defined(PX_X64)
		__asm{
			cpuid
			rdtsc
			mov		ebx, val
			mov		[ebx], eax
		}
#endif
	}

	PX_INLINE void	EndProfile(PxU32& val)
	{
#if defined(PX_WINDOWS) && !defined(PX_X64)
		__asm{
			cpuid
			rdtsc
			mov		ebx, val
			sub		eax, [ebx]
			mov		[ebx], eax
		}
#endif
	}

#ifdef SUPPORT_PROGRESSIVE_BUILDING
	class Ice::FIFOStack2 : public ContainerSizeT
	{
		public:
		//! Constructor
								FIFOStack2() : mCurIndex(0)	{}
		//! Destructor
								~FIFOStack2()				{}
		// Management
		PX_INLINE	FIFOStack2&		Push(size_t entry)			{	Add(entry);	return *this;	}
				bool			Pop(size_t &entry);
		private:
				PxU32			mCurIndex;			//!< Current index within the container
	};

bool FIFOStack2::Pop(size_t & entry)
{
	PxU32 NbEntries = GetNbEntries();						// Get current number of entries
	if(!NbEntries)	return false;							// Can be NULL when no value has been pushed. This is an invalid pop call.
	entry = GetEntry(mCurIndex++);							// Get oldest entry, move to next one
	if(mCurIndex==NbEntries)	{ Reset(); mCurIndex=0; }	// All values have been poped
	return true;
}
#endif


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Constructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
AABBTreeNode::AABBTreeNode() :
#ifdef KEEP_PARENT_PTR
	mParent			(NULL),
#endif
	mPos			(NULL),
	mNbPrimitives	(0),
	mNodePrimitives	(NULL)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Destructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
AABBTreeNode::~AABBTreeNode()
{
	// Opcode 1.3:
	const AABBTreeNode* Pos = GetPos(); 
	const AABBTreeNode* Neg = GetNeg();
	if(!(mPos&1))	PX_DELETE_ARRAY(Pos); 
	mNodePrimitives	= NULL;	// This was just a shortcut to the global list => no release
	mNbPrimitives	= 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Splits the node along a given axis.
 *	The list of indices is reorganized according to the split values.
 *	\param		axis		[in] splitting axis index
 *	\param		builder		[in] the tree builder
 *	\return		the number of primitives assigned to the first child
 *	\warning	this method reorganizes the internal list of primitives
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PxU32 AABBTreeNode::Split(PxU32 axis, AABBTreeBuilder* builder)
{
	// Get node split value
	const float SplitValue = builder->GetSplittingValue(mNodePrimitives, mNbPrimitives, mBV, axis);

	PxU32 NbPos = 0;
	// Loop through all node-related primitives. Their indices range from mNodePrimitives[0] to mNodePrimitives[mNbPrimitives-1].
	// Those indices map the global list in the tree builder.
	for(PxU32 i=0;i<mNbPrimitives;i++)
	{
		// Get index in global list
		PxU32 Index = mNodePrimitives[i];

		// Test against the splitting value. The primitive value is tested against the enclosing-box center.
		// [We only need an approximate partition of the enclosing box here.]
		float PrimitiveValue = builder->GetSplittingValue(Index, axis);

		// Reorganize the list of indices in this order: positive - negative.
		if(PrimitiveValue > SplitValue)
		{
			// Swap entries
			PxU32 Tmp = mNodePrimitives[i];
			mNodePrimitives[i] = mNodePrimitives[NbPos];
			mNodePrimitives[NbPos] = Tmp;
			// Count primitives assigned to positive space
			NbPos++;
		}
	}
	return NbPos;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Subdivides the node.
 *	
 *	          N
 *	        /   \
 *	      /       \
 *	   N/2         N/2
 *	  /   \       /   \
 *	N/4   N/4   N/4   N/4
 *	(etc)
 *
 *	A well-balanced tree should have a O(log n) depth.
 *	A degenerate tree would have a O(n) depth.
 *	Note a perfectly-balanced tree is not well-suited to collision detection anyway.
 *
 *	\param		builder		[in] the tree builder
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool AABBTreeNode::Subdivide(AABBTreeBuilder* builder)
{
	// Checkings
	if(!builder)	return false;

	// Stop subdividing if we reach a leaf node. This is always performed here,
	// else we could end in trouble if user overrides this.
	if(mNbPrimitives==1)	return true;

	// Let the user validate the subdivision
	if(!builder->ValidateSubdivision(mNodePrimitives, mNbPrimitives, mBV))	return true;

	bool ValidSplit = true;	// Optimism...
	PxU32 NbPos;
	if(builder->mSettings.mRules & SPLIT_LARGEST_AXIS)
	{
		// Find the largest axis to split along
		PxVec3 Extents = mBV.getExtents();	// Box extents
		PxU32 Axis	= Ps::largestAxis(Extents);		// Index of largest axis

		// Split along the axis
		NbPos = Split(Axis, builder);

		// Check split validity
		if(!NbPos || NbPos==mNbPrimitives)	ValidSplit = false;
	}
	else if(builder->mSettings.mRules & SPLIT_SPLATTER_POINTS)
	{
		// Compute the means
		PxVec3 Means(0.0f, 0.0f, 0.0f);
		const PxU32* Prims = mNodePrimitives;
		const PxU32* Last = mNodePrimitives + mNbPrimitives;
		while(Prims!=Last)
//		for(PxU32 i=0;i<mNbPrimitives;i++)
		{
//			PxU32 Index = mNodePrimitives[i];
			const PxU32 Index = *Prims++;
//			Means.x += builder->GetSplittingValue(Index, 0);
//			Means.y += builder->GetSplittingValue(Index, 1);
//			Means.z += builder->GetSplittingValue(Index, 2);

			PxVec3 Tmp;
			builder->GetSplittingValues(Index, Tmp);
			Means += Tmp;
		}
		Means/=float(mNbPrimitives);

		// Compute variances
		PxVec3 Vars(0.0f, 0.0f, 0.0f);
		Prims = mNodePrimitives;
		while(Prims!=Last)
//		for(PxU32 i=0;i<mNbPrimitives;i++)
		{
//			PxU32 Index = mNodePrimitives[i];
			const PxU32 Index = *Prims++;

//			float Cx = builder->GetSplittingValue(Index, 0);
//			float Cy = builder->GetSplittingValue(Index, 1);
//			float Cz = builder->GetSplittingValue(Index, 2);
//			Vars.x += (Cx - Means.x)*(Cx - Means.x);
//			Vars.y += (Cy - Means.y)*(Cy - Means.y);
//			Vars.z += (Cz - Means.z)*(Cz - Means.z);

			PxVec3 C;
			builder->GetSplittingValues(Index, C);
			Vars.x += (C.x - Means.x)*(C.x - Means.x);
			Vars.y += (C.y - Means.y)*(C.y - Means.y);
			Vars.z += (C.z - Means.z)*(C.z - Means.z);
		}
		Vars/=float(mNbPrimitives-1);

		// Choose axis with greatest variance
		PxU32 Axis = Ps::largestAxis(Vars);

		// Split along the axis
		NbPos = Split(Axis, builder);

		// Check split validity
		if(!NbPos || NbPos==mNbPrimitives)	ValidSplit = false;
	}
	else if(builder->mSettings.mRules & SPLIT_BALANCED)
	{
		// Test 3 axis, take the best
		float Results[3];
		NbPos = Split(0, builder);	Results[0] = float(NbPos)/float(mNbPrimitives);
		NbPos = Split(1, builder);	Results[1] = float(NbPos)/float(mNbPrimitives);
		NbPos = Split(2, builder);	Results[2] = float(NbPos)/float(mNbPrimitives);
		Results[0]-=0.5f;	Results[0]*=Results[0];
		Results[1]-=0.5f;	Results[1]*=Results[1];
		Results[2]-=0.5f;	Results[2]*=Results[2];
		PxU32 Min=0;
		if(Results[1]<Results[Min])	Min = 1;
		if(Results[2]<Results[Min])	Min = 2;
		
		// Split along the axis
		NbPos = Split(Min, builder);

		// Check split validity
		if(!NbPos || NbPos==mNbPrimitives)	ValidSplit = false;
	}
	else if(builder->mSettings.mRules & SPLIT_BEST_AXIS)
	{
		// Test largest, then middle, then smallest axis...

		// Sort axis
		PxVec3 Extents = mBV.getExtents();	// Box extents
		PxU32 SortedAxis[] = { 0, 1, 2 };
		float* Keys = (float*)&Extents.x;
		for(PxU32 j=0;j<3;j++)
		{
			for(PxU32 i=0;i<2;i++)
			{
				if(Keys[SortedAxis[i]]<Keys[SortedAxis[i+1]])
				{
					PxU32 Tmp = SortedAxis[i];
					SortedAxis[i] = SortedAxis[i+1];
					SortedAxis[i+1] = Tmp;
				}
			}
		}

		// Find the largest axis to split along
		PxU32 CurAxis = 0;
		ValidSplit = false;
		while(!ValidSplit && CurAxis!=3)
		{
			NbPos = Split(SortedAxis[CurAxis], builder);
			// Check the subdivision has been successful
			if(!NbPos || NbPos==mNbPrimitives)	CurAxis++;
			else								ValidSplit = true;
		}
	}
	else if(builder->mSettings.mRules & SPLIT_FIFTY)
	{
		// Don't even bother splitting (mainly a performance test)
		NbPos = mNbPrimitives>>1;
	}
	else return false;	// Unknown splitting rules

	// Check the subdivision has been successful
	if(!ValidSplit)
	{
		// Here, all boxes lie in the same sub-space. Two strategies:
		// - if the tree *must* be complete, make an arbitrary 50-50 split
		// - else stop subdividing
//		if(builder->mSettings.mRules&SPLIT_COMPLETE)
//		if(builder->mSettings.mLimit==1)
		if(mNbPrimitives>builder->mSettings.mLimit)
		{
			builder->IncreaseNbInvalidSplits();
			NbPos = mNbPrimitives>>1;
		}
		else return true;
	}

	// Now create children and assign their pointers.
	if(builder->mNodeBase)
	{
		// We use a pre-allocated linear pool for complete trees [Opcode 1.3]
		AABBTreeNode* Pool = (AABBTreeNode*)builder->mNodeBase;
		PxU32 Count = builder->GetCount();

		// Set last bit to tell it shouldn't be freed ### pretty ugly, find a better way. Maybe one bit in mNbPrimitives
		PX_ASSERT(!(size_t(&Pool[Count+0])&1));
		PX_ASSERT(!(size_t(&Pool[Count+1])&1));
		mPos = size_t(&Pool[Count+0])|1;
	}
	else
	{
		// Non-complete trees and/or Opcode 1.2 allocate nodes on-the-fly
		AABBTreeNode* PosNeg = PX_NEW(AABBTreeNode)[2];
		mPos = (size_t)PosNeg;
	}

	// Update stats
	builder->IncreaseCount(2);

	// Assign children
	AABBTreeNode* Pos = (AABBTreeNode*)GetPos();
	AABBTreeNode* Neg = (AABBTreeNode*)GetNeg();
	Pos->mNodePrimitives	= &mNodePrimitives[0];
	Pos->mNbPrimitives		= NbPos;
	Neg->mNodePrimitives	= &mNodePrimitives[NbPos];
	Neg->mNbPrimitives		= mNbPrimitives - NbPos;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Recursive hierarchy building in a top-down fashion.
 *	\param		builder		[in] the tree builder
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static PxU32 Count=0;
void AABBTreeNode::_BuildHierarchy(AABBTreeBuilder* builder)
{
	Count++;
	// 1) Compute the global box for current node. The box is stored in mBV.
	builder->ComputeGlobalBox(mNodePrimitives, mNbPrimitives, mBV);

	// 1b) Box extension [Opcode 1.4]
	if(builder->mSettings.mAxis!=PX_INVALID_U32)
	{
		// Save initial bounding box on first call
		if(builder->mInitNode)
		{
			builder->mInitBox = mBV;
			builder->mInitNode = false;
		}

		// Always test against initial box, but modify current one
		PxVec3 Min = mBV.minimum;
		PxVec3 Max = mBV.maximum;

		bool WriteBack = false;
		if(builder->mSettings.mExtensionValue<builder->mInitBox.minimum[builder->mSettings.mAxis])
		{
			// Extend towards negative side
			Min[builder->mSettings.mAxis] = builder->mSettings.mExtensionValue;
			WriteBack = true;
		}
		else if(builder->mSettings.mExtensionValue>builder->mInitBox.maximum[builder->mSettings.mAxis])
		{
			// Extend towards positive side
			Max[builder->mSettings.mAxis] = builder->mSettings.mExtensionValue;
			WriteBack = true;
		}
		// else error

		if(WriteBack)
		{
			mBV.minimum = Min;
			mBV.maximum = Max;
		}
	}

	// 1c) Skin extension [Opcode 1.4]
	if(IR(builder->mSettings.mSkinSize))
	{
		const float SkinSize = builder->mSettings.mSkinSize;
		const PxVec3 Skin(SkinSize, SkinSize, SkinSize);
		mBV.minimum = mBV.minimum - Skin;
		mBV.maximum = mBV.maximum + Skin;
	}

	// 2) Subdivide current node
	Subdivide(builder);

	// 3) Recurse
	AABBTreeNode* Pos = (AABBTreeNode*)GetPos();
	AABBTreeNode* Neg = (AABBTreeNode*)GetNeg();
#ifdef KEEP_PARENT_PTR
	if(Pos)	Pos->mParent = this;
	if(Neg)	Neg->mParent = this;
#endif
	if(Pos)	Pos->_BuildHierarchy(builder);
	if(Neg)	Neg->_BuildHierarchy(builder);

	builder->mTotalPrims += mNbPrimitives;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Refits the tree (top-down).
 *	\param		builder		[in] the tree builder
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AABBTreeNode::_Refit(AABBTreeBuilder* builder)
{
	// 1) Recompute the new global box for current node
	builder->ComputeGlobalBox(mNodePrimitives, mNbPrimitives, mBV);

	// 2) Recurse
	AABBTreeNode* Pos = (AABBTreeNode*)GetPos();
	AABBTreeNode* Neg = (AABBTreeNode*)GetNeg();
	if(Pos)	Pos->_Refit(builder);
	if(Neg)	Neg->_Refit(builder);
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Constructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
AABBTree::AABBTree() : mIndices(NULL), mTotalNbNodes(0), mTotalPrims(0), mPool(NULL)
{
#ifdef SUPPORT_REFIT_BITMASK
	#ifndef DWORD_REFIT_BITMASK
	mRefitBitmask	= NULL;
	#endif
#endif
#ifdef SUPPORT_PROGRESSIVE_BUILDING
	mStack = NULL;
#endif
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Destructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
AABBTree::~AABBTree()
{
	Release();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Releases the tree.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AABBTree::Release()
{
#ifdef SUPPORT_REFIT_BITMASK
	#ifndef DWORD_REFIT_BITMASK
	ICE_FREE(mRefitBitmask);
	#endif
#endif

#ifdef SUPPORT_PROGRESSIVE_BUILDING
	PX_DELETE_AND_RESET(mStack);
#endif
	PX_DELETE_ARRAY(mPool); 
	PX_FREE_AND_RESET(mIndices);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Builds a generic AABB tree from a tree builder.
 *	\param		builder		[in] the tree builder
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool AABBTree::Build(AABBTreeBuilder* builder)
{
	// Checkings
	if(!builder || !builder->mNbPrimitives)	return false;

	// Release previous tree
	Release();

	// Init stats
	builder->SetCount(1);
	builder->SetNbInvalidSplits(0);

	// Initialize indices. This list will be modified during build.
//	mIndices = new PxU32[builder->mNbPrimitives];
	mIndices = (PxU32*)PX_ALLOC(sizeof(PxU32)*builder->mNbPrimitives);
	// Identity permutation
	for(PxU32 i=0;i<builder->mNbPrimitives;i++)	mIndices[i] = i;

	// Use a linear array for complete trees (since we can predict the final number of nodes) [Opcode 1.3]
//	if(builder->mRules&SPLIT_COMPLETE)
//	if(builder->mSettings.mLimit==1)	// XP: we allocate more memory but...
#pragma message("Remove this XP")
	{
		// Allocate a pool of nodes
		mPool = PX_NEW(AABBTreeNode)[builder->mNbPrimitives*2 - 1];

		builder->mNodeBase = mPool;	// ### ugly !
	}

	// Setup initial node. Here we have a complete permutation of the app's primitives.
	mPool->mNodePrimitives	= mIndices;
	mPool->mNbPrimitives	= builder->mNbPrimitives;

	// Build the hierarchy
	builder->mInitNode = true;	// [Opcode 1.4]
	Count = 0;
	mPool->_BuildHierarchy(builder);

	// Get back total number of nodes
	mTotalNbNodes	= builder->GetCount();
	mTotalPrims		= builder->mTotalPrims;

	// For complete trees, check the correct number of nodes has been created [Opcode 1.3]
	if(mPool && builder->mSettings.mLimit==1)	PX_ASSERT(mTotalNbNodes==builder->mNbPrimitives*2 - 1);
	return true;
}

#ifdef SUPPORT_PROGRESSIVE_BUILDING
static PxU32 BuildHierarchy(FIFOStack2& stack, AABBTreeNode* node, AABBTreeBuilder* builder)
{
	// 1) Compute the global box for current node. The box is stored in mBV.
	builder->ComputeGlobalBox(node->GetPrimitives(), node->GetNbPrimitives(), node->GetAABB());

	// 2) Subdivide current node
	node->Subdivide(builder);

	// 3) Recurse
	AABBTreeNode* Pos = (AABBTreeNode*)node->GetPos();
	AABBTreeNode* Neg = (AABBTreeNode*)node->GetNeg();
#ifdef KEEP_PARENT_PTR
	if(Pos)	Pos->mParent = node;
	if(Neg)	Neg->mParent = node;
#endif
//	if(Pos)	Pos->_BuildHierarchy(builder);
//	if(Neg)	Neg->_BuildHierarchy(builder);
	if(Pos)	stack.Add(size_t(Pos));
	if(Neg)	stack.Add(size_t(Neg));

	builder->mTotalPrims += node->GetNbPrimitives();

	return node->GetNbPrimitives();
}

PxU32 AABBTree::Build(AABBTreeBuilder* builder, PxU32 progress, PxU32 limit)
{
	if(progress==0)
	{
		// Checkings
		if(!builder || !builder->mNbPrimitives)	return PX_INVALID_U32;

		// Release previous tree
		Release();

		// Init stats
		builder->SetCount(1);
		builder->SetNbInvalidSplits(0);

		// Initialize indices. This list will be modified during build.
	//	mIndices = new PxU32[builder->mNbPrimitives];
		mIndices = (PxU32*)PX_ALLOC(sizeof(PxU32)*builder->mNbPrimitives);
		// Identity permutation
		for(PxU32 i=0;i<builder->mNbPrimitives;i++)	mIndices[i] = i;

		// Use a linear array for complete trees (since we can predict the final number of nodes) [Opcode 1.3]
	//	if(builder->mRules&SPLIT_COMPLETE)
	//	if(builder->mSettings.mLimit==1)	// XP: we allocate more memory but...
	#pragma message("Remove this XP")
		{
			// Allocate a pool of nodes
			mPool = PX_NEW(AABBTreeNode)[builder->mNbPrimitives*2 - 1];

			builder->mNodeBase = mPool;	// ### ugly !
		}
		builder->mInitNode = true;	// [Opcode 1.4]

		// Setup initial node. Here we have a complete permutation of the app's primitives.
		mPool->mNodePrimitives	= mIndices;
		mPool->mNbPrimitives	= builder->mNbPrimitives;

		mStack = PX_NEW(FIFOStack2);
		AABBTreeNode* FirstNode = mPool;
		mStack->Add(size_t(FirstNode));
		return progress++;
	}
	else if(progress==1)
	{
		// Build the hierarchy
//		_BuildHierarchy(builder);

/*
		PxU32 Entry;
		if(stack.Pop(Entry))
		{
			BuildHierarchy(stack, (AABBTreeNode*)Entry, builder);
			return progress;
		}
*/
		if(mStack->GetNbEntries())
		{
			PxU32 Total = 0;
//#define USE_CYCLES

#ifdef USE_CYCLES
			const PxU32 Limit = 300000;
#else
			//mTotalNbNodes
//			PxU32 Nb = 4000/60;
//			while(Nb--)
//			const PxU32 Limit = 500;
//			const PxU32 Limit = 1;
//			const PxU32 Limit = 1 + builder->mNbPrimitives/100;
			const PxU32 Limit = limit;
#endif
			while(Total<Limit)
			{
//PxU32 Time;
//StartProfile(Time);
				size_t Entry;
				if(mStack->Pop(Entry))
				{
#ifndef USE_CYCLES
					Total +=
#endif
						BuildHierarchy(*mStack, (AABBTreeNode*)Entry, builder);
				}
				else break;
//EndProfile(Time);
#ifdef USE_CYCLES
Total += Time;
#endif
			}
			return progress;
		}

		// Get back total number of nodes
		mTotalNbNodes	= builder->GetCount();
		mTotalPrims		= builder->mTotalPrims;

		// For complete trees, check the correct number of nodes has been created [Opcode 1.3]
		if(mPool && builder->mSettings.mLimit==1)	PX_ASSERT(mTotalNbNodes==builder->mNbPrimitives*2 - 1);

		PX_DELETE_AND_RESET(mStack);

		return 0;	// Done!
	}
	return PX_INVALID_U32;
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes the depth of the tree.
 *	A well-balanced tree should have a log(n) depth. A degenerate tree O(n) depth.
 *	\return		depth of the tree
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PxU32 AABBTree::ComputeDepth() const
{
	return Walk(NULL, NULL);	// Use the walking code without callback
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Walks the tree, calling the user back for each node.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PxU32 AABBTree::Walk(WalkingCallback callback, void* user_data) const
{
	// Call it without callback to compute maximum depth
	PxU32 MaxDepth = 0;
	PxU32 CurrentDepth = 0;

	struct Local
	{
		static void _Walk(const AABBTreeNode* current_node, PxU32& max_depth, PxU32& current_depth, WalkingCallback callback, void* user_data)
		{
			// Checkings
			if(!current_node)	return;
			// Entering a new node => increase depth
			current_depth++;
			// Keep track of maximum depth
			if(current_depth>max_depth)	max_depth = current_depth;

			// Callback
			if(callback && !(callback)(current_node, current_depth, user_data))	return;

			// Recurse
			if(current_node->GetPos())	{ _Walk(current_node->GetPos(), max_depth, current_depth, callback, user_data);	current_depth--;	}
			if(current_node->GetNeg())	{ _Walk(current_node->GetNeg(), max_depth, current_depth, callback, user_data);	current_depth--;	}
		}
	};
	Local::_Walk(mPool, MaxDepth, CurrentDepth, callback, user_data);
	return MaxDepth;
}

void AABBTree::Walk2(WalkingCallback callback, void* user_data) const
{
	if(!callback)	return;
	struct Local
	{
		static void _Walk(const AABBTreeNode* current_node, WalkingCallback callback, void* user_data)
		{
			const AABBTreeNode* P = current_node->GetPos();
			const AABBTreeNode* N = current_node->GetNeg();

			if(P && !(callback)(P, 0, user_data))	return;
			if(N && !(callback)(N, 0, user_data))	return;

			if(P)	{ _Walk(P, callback, user_data);	}
			if(N)	{ _Walk(N, callback, user_data);	}
		}
	};
	if(!(callback)(GetNodes(), 0, user_data))	return;
	Local::_Walk(GetNodes(), callback, user_data);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Refits the tree in a top-down way.
 *	\param		builder		[in] the tree builder
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool AABBTree::Refit(AABBTreeBuilder* builder)
{
	if(!builder)	return false;
	mPool->_Refit(builder);
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Refits the tree in a bottom-up way.
 *	\param		builder		[in] the tree builder
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PX_INLINE Ps::IntBool SameBoxes(const PxBounds3& a, const PxBounds3& b)
{
	if(a.minimum[0] != b.minimum[0])	return Ps::IntFalse;
	if(a.minimum[1] != b.minimum[1])	return Ps::IntFalse;
	if(a.minimum[2] != b.minimum[2])	return Ps::IntFalse;
	if(a.maximum[0] != b.maximum[0])	return Ps::IntFalse;
	if(a.maximum[1] != b.maximum[1])	return Ps::IntFalse;
	if(a.maximum[2] != b.maximum[2])	return Ps::IntFalse;
	return Ps::IntTrue;
//	return a.IsInside(b);
}

// A fast inlined version of this stuff
PX_INLINE void ComputeUnionBox(PxBounds3& global_box, const PxU32* primitives, PxU32 nb_prims, const PxBounds3* boxes)
{
//	if(!primitives || !nb_prims)	return false;

	if(!nb_prims)
	{
		// Might happen after a node has been invalidated
		global_box.setEmpty();
	}
	else
	{
		global_box = boxes[*primitives];

		if(nb_prims>1)
		{
			const PxU32* Last = primitives + nb_prims;
			primitives++;

			PxVec3 Min = global_box.minimum;
			PxVec3 Max = global_box.maximum;

			while(primitives!=Last)
			{
				const PxBounds3& aabb = boxes[*primitives++];
				if(aabb.minimum.x<Min.x)	Min.x = aabb.minimum.x;
				if(aabb.minimum.y<Min.y)	Min.y = aabb.minimum.y;
				if(aabb.minimum.z<Min.z)	Min.z = aabb.minimum.z;
				if(aabb.maximum.x>Max.x)	Max.x = aabb.maximum.x;
				if(aabb.maximum.y>Max.y)	Max.y = aabb.maximum.y;
				if(aabb.maximum.z>Max.z)	Max.z = aabb.maximum.z;
			}
			global_box.minimum = Min;
			global_box.maximum = Max;
		}
	}
}

bool AABBTree::Refit2(AABBTreeBuilder* builder)
{
	// Checkings
	if(!builder)	return false;

	PX_ASSERT(mPool);

	// ### EVIL TEST
	const PxBounds3* Boxes = ((const AABBTreeOfAABBsBuilder*)builder)->mAABBArray;

//#define NEW_APPROACH
#ifdef NEW_APPROACH
	PxU32 Index = mTotalNbNodes;
	bool* Flags = (bool*)StackAlloc(Index);
	ZeroMemory(Flags, Index);
	const float Coeff = 1.2f;
	while(Index--)
	{
		AABBTreeNode& Current = mPool[Index];
		if(Index)	Ps::prefetch(mPool + Index - 1);

		if(Current.IsLeaf())
		{
			AABB NewBox;
//			builder->ComputeGlobalBox(Current.GetPrimitives(), Current.GetNbPrimitives(), NewBox);
			ComputeUnionBox(NewBox, Current.GetPrimitives(), Current.GetNbPrimitives(), Boxes);
//			if(!SameBoxes(NewBox, Current.GetAABB()))
			if(!NewBox.IsInside(Current.GetAABB()))
			{
				NewBox *= Coeff;
				Current.GetAABB() = NewBox;
				Flags[Index] = true;
			}
		}
		else
		{
			PxU32 PIndex = (PxU32(Current.GetPos()) - PxU32(mPool))/sizeof(AABBTreeNode);
			PxU32 NIndex = (PxU32(Current.GetNeg()) - PxU32(mPool))/sizeof(AABBTreeNode);
// ### this one doesn't seem to work
//			if(!Flags[PIndex] && !Flags[NIndex])
//				return true;
			if(Flags[PIndex] || Flags[NIndex])
			{
				const ShadowAABB& PBox = (const ShadowAABB&)Current.GetPos()->GetAABB();
				const ShadowAABB& NBox = (const ShadowAABB&)Current.GetNeg()->GetAABB();
				AABB NewBox;
				ShadowAABB& cb = (ShadowAABB&)NewBox;
				cb.mMin.x = MIN(PBox.mMin.x, NBox.mMin.x);
				cb.mMin.y = MIN(PBox.mMin.y, NBox.mMin.y);
				cb.mMin.z = MIN(PBox.mMin.z, NBox.mMin.z);
				cb.mMax.x = MAX(PBox.mMax.x, NBox.mMax.x);
				cb.mMax.y = MAX(PBox.mMax.y, NBox.mMax.y);
				cb.mMax.z = MAX(PBox.mMax.z, NBox.mMax.z);

//				if(!SameBoxes(NewBox, Current.GetAABB()))
				if(!NewBox.IsInside(Current.GetAABB()))
				{
					NewBox *= Coeff;
					Current.GetAABB() = NewBox;
					Flags[Index] = true;
				}
			}
		}
	}

#else

	// Bottom-up update
	PxU32 Index = mTotalNbNodes;
	while(Index--)
	{
		AABBTreeNode& Current = mPool[Index];
		if(Index)	Ps::prefetch(mPool + Index - 1);

		if(Current.IsLeaf())
		{
///////
//			if(!Current.GetNbPrimitives())
//				Current.GetAABB().SetEmpty();
//			else
///////
//			builder->ComputeGlobalBox(Current.GetPrimitives(), Current.GetNbPrimitives(), Current.GetAABB());
			ComputeUnionBox(Current.GetAABB(), Current.GetPrimitives(), Current.GetNbPrimitives(), Boxes);
		}
		else
		{
			const PxBounds3& PBox = Current.GetPos()->GetAABB();
			const PxBounds3& NBox = Current.GetNeg()->GetAABB();
			PxBounds3& cb = Current.GetAABB();
			cb.minimum.x = PxMin(PBox.minimum.x, NBox.minimum.x);
			cb.minimum.y = PxMin(PBox.minimum.y, NBox.minimum.y);
			cb.minimum.z = PxMin(PBox.minimum.z, NBox.minimum.z);
			cb.maximum.x = PxMax(PBox.maximum.x, NBox.maximum.x);
			cb.maximum.y = PxMax(PBox.maximum.y, NBox.maximum.y);
			cb.maximum.z = PxMax(PBox.maximum.z, NBox.maximum.z);
		}
	}

/*#ifdef _DEBUG
	AABB debugBox;
	debugBox.SetEmpty();
	for(PxU32 i=0;i<mTotalNbNodes-1;i++)
	{
		const AABBTreeNode& Current = mPool[i];
		debugBox.Add(Current.GetAABB());
	}
#endif*/
#endif

	return true;
}

#ifdef DWORD_REFIT_BITMASK

	PX_INLINE PxU32 BitsToBytes(PxU32 nb_bits)
	{
		return (nb_bits>>3) + ((nb_bits&7) ? 1 : 0);
	}

	PX_INLINE PxU32 BitsToDwords(PxU32 nb_bits)
	{
		return (nb_bits>>5) + ((nb_bits&31) ? 1 : 0);
	}

BitArray::BitArray() : mSize(0), mBits(NULL)
{
}

BitArray::BitArray(PxU32 nb_bits)
{
	Init(nb_bits);
}

bool BitArray::Init(PxU32 nb_bits)
{
	mSize = BitsToDwords(nb_bits);
	// Get ram for n bits
	PX_DELETE_ARRAY(mBits);
	mBits = new PxU32[mSize];
	// Set all bits to 0
	ClearAll();
	return true;
}

BitArray::~BitArray()
{
	PX_DELETE_ARRAY(mBits);
	mSize = 0;
}

#endif


bool AABBTree::	Refit3(PxU32 nb_objects, const PxBounds3* boxes, const Container& indices)
{
	PX_ASSERT(mPool);

#define METHOD_1

#ifdef METHOD_2
	if(1)
	{
		static Container tmp;
		tmp.Reset();

		PxU32 Nb = indices.GetNbEntries();
		const PxU32* in = indices.GetEntries();
		while(Nb--)
		{
			AABBTreeNode* Current = mPool + *in++;
			while(Current)
			{
				tmp.Add(PxU32(Current));
				Current = Current->mParent;
			}
		}

		Nb = tmp.GetNbEntries();
		RadixSort RS;
		const PxU32* Sorted = RS.Sort(tmp.GetEntries(), Nb).GetRanks();

		AABBTreeNode** Entries = (AABBTreeNode**)tmp.GetEntries();

		AABBTreeNode* Previous=NULL;
		for(PxU32 i=0;i<Nb;i++)
		{
			AABBTreeNode* Current = Entries[Nb - 1 - Sorted[i]];
			if(Current!=Previous)
			{
				Previous = Current;
				if(Current->IsLeaf())
				{
					ComputeUnionBox(Current->GetAABB(), Current->GetPrimitives(), Current->GetNbPrimitives(), boxes);
				}
				else
				{
					const ShadowAABB& PBox = (const ShadowAABB&)Current->GetPos()->GetAABB();
					const ShadowAABB& NBox = (const ShadowAABB&)Current->GetNeg()->GetAABB();
					ShadowAABB& cb = (ShadowAABB&)Current->GetAABB();
					cb.mMin.x = MIN(PBox.mMin.x, NBox.mMin.x);
					cb.mMin.y = MIN(PBox.mMin.y, NBox.mMin.y);
					cb.mMin.z = MIN(PBox.mMin.z, NBox.mMin.z);
					cb.mMax.x = MAX(PBox.mMax.x, NBox.mMax.x);
					cb.mMax.y = MAX(PBox.mMax.y, NBox.mMax.y);
					cb.mMax.z = MAX(PBox.mMax.z, NBox.mMax.z);
				}
			}
		}
	}
#endif

#ifdef METHOD_1
	if(1)
	{
		//PxU32 Time;
		//StartProfile(Time);

//		printf("Size: %d\n", sizeof(AABBTreeNode));
		PxU32 Nb = indices.GetNbEntries();
		if(!Nb)	return true;

		PxU32 Index = mTotalNbNodes;

		// ### those flags could be written directly, no need for the indices array
		bool* Flags = (bool*)PxAlloca(Index);
		Ps::memZero(Flags, Index);
//		BitArray BA(Index);
//		PxU32 NbDwords = BitsToDwords(Index);
//		PxU32* Flags = (PxU32*)StackAlloc(NbDwords);
//		ZeroMemory(Flags, NbDwords);

		const PxU32* in = indices.GetEntries();
		while(Nb--)
		{
			PxU32 Index = *in++;
			PX_ASSERT(Index<mTotalNbNodes);
			AABBTreeNode* Current = mPool + Index;
			while(Current)
			{
				PxU32 CurrentIndex = PxU32(size_t(Current) - size_t(mPool)) / sizeof(AABBTreeNode);
				if(Flags[CurrentIndex])
//				if(BA.IsSet(CurrentIndex))
				{
					// We can early exit if we already visited the node!
					break;
				}
				else
				{
					Flags[CurrentIndex] = true;
//					BA.SetBit(CurrentIndex);
					Current = Current->mParent;
				}

			}
		}
		//EndProfile(Time);
//		printf("Time1: %d\n", Time);

		//StartProfile(Time);
		while(Index--)
		{
			if(Flags[Index])
//			if(PxU8(Flags[Index])==2)	// TEST
//			if(BA.IsSet(Index))
			{
				AABBTreeNode* Current = mPool + Index;
				if(Current->IsLeaf())
				{
					ComputeUnionBox(Current->GetAABB(), Current->GetPrimitives(), Current->GetNbPrimitives(), boxes);
				}
				else
				{
					const PxBounds3& PBox = Current->GetPos()->GetAABB();
					const PxBounds3& NBox = Current->GetNeg()->GetAABB();
					PxBounds3& cb = Current->GetAABB();
					cb.minimum.x = PxMin(PBox.minimum.x, NBox.minimum.x);
					cb.minimum.y = PxMin(PBox.minimum.y, NBox.minimum.y);
					cb.minimum.z = PxMin(PBox.minimum.z, NBox.minimum.z);
					cb.maximum.x = PxMax(PBox.maximum.x, NBox.maximum.x);
					cb.maximum.y = PxMax(PBox.maximum.y, NBox.maximum.y);
					cb.maximum.z = PxMax(PBox.maximum.z, NBox.maximum.z);
				}
			}
		}
		//EndProfile(Time);
//		printf("Time2: %d\n", Time);
	}
#endif

#ifdef METHOD_0
	// ### wrong box mapping here. And why aren't we starting from leaves? We miss a mapping
	// between the Prunable and its index in the tree

	for(PxU32 i=0;i<indices.GetNbEntries();i++)
	{
		AABBTreeNode* Current = mPool + indices[i];

		while(Current)
		{
			if(Current->IsLeaf())
			{
				ComputeUnionBox(Current->GetAABB(), Current->GetPrimitives(), Current->GetNbPrimitives(), boxes);
			}
			else
			{
				const ShadowAABB& PBox = (const ShadowAABB&)Current->GetPos()->GetAABB();
				const ShadowAABB& NBox = (const ShadowAABB&)Current->GetNeg()->GetAABB();
				ShadowAABB& cb = (ShadowAABB&)Current->GetAABB();
				cb.mMin.x = MIN(PBox.mMin.x, NBox.mMin.x);
				cb.mMin.y = MIN(PBox.mMin.y, NBox.mMin.y);
				cb.mMin.z = MIN(PBox.mMin.z, NBox.mMin.z);
				cb.mMax.x = MAX(PBox.mMax.x, NBox.mMax.x);
				cb.mMax.y = MAX(PBox.mMax.y, NBox.mMax.y);
				cb.mMax.z = MAX(PBox.mMax.z, NBox.mMax.z);
			}
			Current = Current->mParent;
		}
	}
#endif

	return true;
}

#ifdef SUPPORT_REFIT_BITMASK
void AABBTree::MarkForRefit(PxU32 index)
{
#ifdef DWORD_REFIT_BITMASK
	if(!mRefitBitmask.GetBits())
		mRefitBitmask.Init(mTotalNbNodes);
#else
	if(!mRefitBitmask)
	{
		mRefitBitmask = (bool*)ICE_ALLOC(mTotalNbNodes*sizeof(bool));
		ZeroMemory(mRefitBitmask, mTotalNbNodes*sizeof(bool));
	}
	bool* Flags = mRefitBitmask;
#endif

	PX_ASSERT(index<mTotalNbNodes);

	AABBTreeNode* Current = mPool + index;
	while(Current)
	{
		PxU32 CurrentIndex = PxU32(size_t(Current) - size_t(mPool)) / sizeof(AABBTreeNode);
		PX_ASSERT(CurrentIndex<mTotalNbNodes);
#ifdef DWORD_REFIT_BITMASK
		if(mRefitBitmask.IsSet(CurrentIndex))
#else
		if(Flags[CurrentIndex])
#endif
		{
			// We can early exit if we already visited the node!
			return;
		}
		else
		{
#ifdef DWORD_REFIT_BITMASK
			mRefitBitmask.SetBit(CurrentIndex);
#else
			Flags[CurrentIndex] = true;
#endif
			Current = Current->mParent;
		}
	}
}

void AABBTree::RefitMarked(PxU32 nb_objects, const PxBounds3* boxes)
{
#ifdef DWORD_REFIT_BITMASK
	if(!mRefitBitmask.GetBits())	return;	// No refit needed
#else
	if(!mRefitBitmask)	return;	// No refit needed
#endif

//PxU32 Time;
//StartProfile(Time);

#ifdef DWORD_REFIT_BITMASK
	const PxU32* Bits = mRefitBitmask.GetBits();
	PxU32 Size = mRefitBitmask.GetSize();
	while(Size--)
	{
		// Test 32 bits at a time
		if(!Bits[Size])
			continue;

		PxU32 Index = (Size+1)<<5;
		PxU32 Count=32;
		while(Count--)
		{
			Index--;

			if(mRefitBitmask.IsSet(Index))
			{
				mRefitBitmask.ClearBit(Index);

				AABBTreeNode* Current = mPool + Index;
				if(Current->IsLeaf())
				{
					ComputeUnionBox(Current->GetAABB(), Current->GetPrimitives(), Current->GetNbPrimitives(), boxes);
				}
				else
				{
					const PxBounds3& PBox = Current->GetPos()->GetAABB();
					const PxBounds3& NBox = Current->GetNeg()->GetAABB();
					PxBounds3& cb = Current->GetAABB();
					cb.minimum.x = PxMin(PBox.minimum.x, NBox.minimum.x);
					cb.minimum.y = PxMin(PBox.minimum.y, NBox.minimum.y);
					cb.minimum.z = PxMin(PBox.minimum.z, NBox.minimum.z);
					cb.maximum.x = PxMax(PBox.maximum.x, NBox.maximum.x);
					cb.maximum.y = PxMax(PBox.maximum.y, NBox.maximum.y);
					cb.maximum.z = PxMax(PBox.maximum.z, NBox.maximum.z);
				}
			}
		}
	}
#else
	PxU32 Index = mTotalNbNodes;
	bool* Flags = mRefitBitmask;

	while(Index--)
	{
		if(Flags[Index])
		{
			Flags[Index] = false;

			AABBTreeNode* Current = mPool + Index;
			if(Current->IsLeaf())
			{
				ComputeUnionBox(Current->GetAABB(), Current->GetPrimitives(), Current->GetNbPrimitives(), boxes);
			}
			else
			{
				const ShadowAABB& PBox = (const ShadowAABB&)Current->GetPos()->GetAABB();
				const ShadowAABB& NBox = (const ShadowAABB&)Current->GetNeg()->GetAABB();
				ShadowAABB& cb = (ShadowAABB&)Current->GetAABB();
				cb.mMin.x = MIN(PBox.mMin.x, NBox.mMin.x);
				cb.mMin.y = MIN(PBox.mMin.y, NBox.mMin.y);
				cb.mMin.z = MIN(PBox.mMin.z, NBox.mMin.z);
				cb.mMax.x = MAX(PBox.mMax.x, NBox.mMax.x);
				cb.mMax.y = MAX(PBox.mMax.y, NBox.mMax.y);
				cb.mMax.z = MAX(PBox.mMax.z, NBox.mMax.z);
			}
		}
	}
#endif

//EndProfile(Time);
//printf("RefitMarked: %d\n", Time);
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Checks the tree is a complete tree or not.
 *	A complete tree is made of 2*N-1 nodes, where N is the number of primitives in the tree.
 *	\return		true for complete trees
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool AABBTree::IsComplete() const
{
	return (GetNbNodes()==mPool->GetNbPrimitives()*2-1);
}



// Culling should be done outside anyway since it touches other concepts (occlusion, etc)

// ### to customize ?
void AABBTreeNode::_TestAgainstPlanes(const Gu::Plane* planes, PxU32 clip_mask, CullingCallback cb, void* user_data) const
{
	PxU32 OutClipMask;

	// Test the box against the planes. If the box is completely culled, so are its children, hence we exit.
	if(!PlanesAABBOverlap(mBV, planes, OutClipMask, clip_mask))	return;

	// If the box is completely included, so are its children. We don't need to do extra tests, we
	// can immediately output a list of visible children. Those ones won't need to be clipped.
	if(!OutClipMask)
	{
		(cb)(mNbPrimitives, mNodePrimitives, Ps::IntFalse, user_data);
		return;
	}

	// Else the box straddles one or several planes, so we need to recurse down the tree.
	if(IsLeaf())
	{
		// We're in a leaf -> output primitives (they will need clipping)
		(cb)(mNbPrimitives, mNodePrimitives, Ps::IntTrue, user_data);
	}
	else
	{
		GetPos()->_TestAgainstPlanes(planes, OutClipMask, cb, user_data);
		GetNeg()->_TestAgainstPlanes(planes, OutClipMask, cb, user_data);
	}
}


// ### to customize ?
void AABBTreeNode::_TestAgainstPlanes(const Gu::Plane* planes, PxU32 clip_mask, Container& box_indices_clip, Container& box_indices_noclip) const
{
	PxU32 OutClipMask;

	// Test the box against the planes. If the box is completely culled, so are its children, hence we exit.
	if(!PlanesAABBOverlap(mBV, planes, OutClipMask, clip_mask))	return;

	// If the box is completely included, so are its children. We don't need to do extra tests, we
	// can immediately output a list of visible children. Those ones won't need to be clipped.
	if(!OutClipMask)
	{
		box_indices_noclip.Add(mNodePrimitives, mNbPrimitives);
		return;
	}

	// Else the box straddles one or several planes, so we need to recurse down the tree.
	if(IsLeaf())
	{
		// We're in a leaf -> output primitives (they will need clipping)
		box_indices_clip.Add(mNodePrimitives, mNbPrimitives);
	}
	else
	{
		GetPos()->_TestAgainstPlanes(planes, OutClipMask, box_indices_clip, box_indices_noclip);
		GetNeg()->_TestAgainstPlanes(planes, OutClipMask, box_indices_clip, box_indices_noclip);
	}
}

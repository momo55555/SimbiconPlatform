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
#include "./Ice/IceSerialize.h"
#include "./Ice/IceUtils.h"
#include "OPC_OptimizedTree.h"
#include "OPC_AABBTree.h"

using namespace physx;
using namespace Ice;

//! Compilation flag:
//! - true to fix quantized boxes (i.e. make sure they enclose the original ones)
//! - false to see the effects of quantization errors (faster, but wrong results in some cases)
static bool gFixQuantized = true;

// Quantization notes:
// - We could use the highest bits of mData to store some more quantized bits. Dequantization code
//   would be slightly more complex, but number of overlap tests would be reduced (and anyhow those
//   bits are currently wasted). Of course it's not possible if we move to 16 bits mData.
// - Something like "16 bits floats" could be tested, to bypass the int-to-float conversion.
// - A dedicated BV-BV test could be used, dequantizing while testing for overlap. (i.e. it's some
//   lazy-dequantization which may save some work in case of early exits). At the very least some
//   muls could be saved by precomputing several more matrices. But maybe not worth the pain.
// - Do we need to dequantize anyway? Not doing the extents-related muls only implies the box has
//   been scaled, for example.
// - The deeper we move into the hierarchy, the smaller the extents should be. May not need a fixed
//   number of quantization bits. Even better, could probably be best delta-encoded.

/*
	- has pos leaf => pos data = triangle index, node++
	- has neg leaf => neg data = triangle index, node++

	only use escape index when no overlap, no pos leaf, no neg leaf
	=> then we can put the escape index in the pos part, it's free

	P, N:
		two leaves, two triangle indices, no escape index

	P, ~N:
		one leaf, one triangle index, no escape index

	~P, N:
		one leaf, one triangle index, no escape index

	~P, ~N:
		no leaves, escape index

*/

static void _BuildStacklessNoLeafTree(AABBStacklessNoLeafNode* const linear, const PxU32 box_id, PxU32& current_id, const AABBTreeNode* current_node)
{
	const AABBTreeNode* P = current_node->GetPos();
	const AABBTreeNode* N = current_node->GetNeg();
	// Leaf nodes here?!
	PX_ASSERT(P);
	PX_ASSERT(N);
	// Internal node => keep the box
	linear[box_id].mAABB.mCenter = current_node->GetAABB().getCenter();
	linear[box_id].mAABB.mExtents = current_node->GetAABB().getExtents();

	// PNSWAP Experiment => save 2 tests in the traversal
	if(!P->IsLeaf() && N->IsLeaf())
	{
		TSwap(P, N);
	}

	const PxU32 saved = current_id;

	bool NeedEscapeIndex = true;

	if(P->IsLeaf())
	{
		// The input tree must be complete => i.e. one primitive/leaf
		PX_ASSERT(P->GetNbPrimitives()==1);
		// Get the primitive index from the input tree
		PxU32 PrimitiveIndex = P->GetPrimitives()[0];
		// Setup prev box data as the primitive index, marked as leaf
		linear[box_id].mPosIndexData = StoreAsLeaf(PrimitiveIndex);
	}
	else
	{
		// Get a new id for positive child
		PxU32 PosID = current_id++;
		linear[box_id].mPosIndexData = StoreAsNonLeaf(0xdead);  // index of pos child is always parent_index+1
		                                                        // so store any value but make sure it does not
		                                                        // interfere with the bits that are set for leaf node
		// Recurse
		_BuildStacklessNoLeafTree(linear, PosID, current_id, P);
	}

	if(N->IsLeaf())
	{
		// The input tree must be complete => i.e. one primitive/leaf
		PX_ASSERT(N->GetNbPrimitives()==1);
		// Get the primitive index from the input tree
		PxU32 PrimitiveIndex = N->GetPrimitives()[0];

			// REMAP_XP
			PX_ASSERT(P->IsLeaf());
			PX_ASSERT(PrimitiveIndex = P->GetPrimitives()[0]+1);
			linear[box_id].mPosIndexData = MarkAsLeafN(linear[box_id].mPosIndexData);

		NeedEscapeIndex = false;
	}
	else
	{
		// Get a new id for negative child
		PxU32 NegID = current_id++;

		// Recurse
		_BuildStacklessNoLeafTree(linear, NegID, current_id, N);
	}

	if(NeedEscapeIndex)
//		linear[box_id].mPosIndexData = StoreAsNonLeaf(current_id - saved + 1);	// SEPT25BUG
//		linear[box_id].mEscapeIndex = current_id - saved + 1;	// SEPT25BUG
        linear[box_id].mEscapeIndex = current_id - saved;	// there should be a +1 to actually get the correct
                                                            // delta, however this is an optimization to save a
                                                            // branch in the collision tree traversal code
    else
        linear[box_id].mEscapeIndex = 0;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Constructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
AABBStacklessQuantizedNoLeafTree::AABBStacklessQuantizedNoLeafTree() : mNbNodes(0), mNodes(NULL)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Destructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
AABBStacklessQuantizedNoLeafTree::~AABBStacklessQuantizedNoLeafTree()
{
	Release();
}

void AABBStacklessQuantizedNoLeafTree::Release()
{
// PX_SERIALIZATION
	if(!isInUserMemory())
//~PX_SERIALIZATION
		PX_FREE(mNodes);			// This part conditional
	mNodes = NULL;					// This part mandatory
	mNbNodes = 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Builds the collision tree from a generic AABB tree.
 *	\param		tree			[in] generic AABB tree
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool AABBStacklessQuantizedNoLeafTree::Build(AABBTree* tree)
{
	// Checkings
	if(!tree)	return false;
	// Check the input tree is complete
	const PxU32 NbTriangles	= tree->GetNodes()->GetNbPrimitives();
	const PxU32 NbNodes		= tree->GetNbNodes();
	if(NbNodes!=NbTriangles*2-1)	return false;

	// Get nodes
	SetNbNodes(NbTriangles-1);
	PX_FREE(mNodes);
	AABBStacklessNoLeafNode* Nodes = (AABBStacklessNoLeafNode*)PX_ALLOC(sizeof(AABBStacklessNoLeafNode)*GetNbNodes());

	// Build the tree
	PxU32 CurID = 1;
	_BuildStacklessNoLeafTree(Nodes, 0, CurID, tree->GetNodes());
	PX_ASSERT(CurID==GetNbNodes());

	// Quantize
	{
		mNodes = (AABBStacklessQuantizedNoLeafNode*)PX_ALLOC(sizeof(AABBStacklessQuantizedNoLeafNode)*GetNbNodes());

		// Find maximum values. Some people asked why I wasn't simply using the first node. Well, I can't.
		// I'm not looking for (minimum, maximum) values like in a standard AABB, I'm looking for the extremal
		// centers/extents in order to quantize them. The first node would only give a single center and
		// a single extents. While extents would be the biggest, the center wouldn't.
		// Get maximum values
		PxVec3 CMax(-PX_MAX_F32, -PX_MAX_F32, -PX_MAX_F32);
		PxVec3 EMax(-PX_MAX_F32, -PX_MAX_F32, -PX_MAX_F32);
		for(PxU32 i=0;i<GetNbNodes();i++)
		{
			if(PxAbs(Nodes[i].mAABB.mCenter.x)>CMax.x)	CMax.x = PxAbs(Nodes[i].mAABB.mCenter.x);
			if(PxAbs(Nodes[i].mAABB.mCenter.y)>CMax.y)	CMax.y = PxAbs(Nodes[i].mAABB.mCenter.y);
			if(PxAbs(Nodes[i].mAABB.mCenter.z)>CMax.z)	CMax.z = PxAbs(Nodes[i].mAABB.mCenter.z);
			if(PxAbs(Nodes[i].mAABB.mExtents.x)>EMax.x)	EMax.x = PxAbs(Nodes[i].mAABB.mExtents.x);
			if(PxAbs(Nodes[i].mAABB.mExtents.y)>EMax.y)	EMax.y = PxAbs(Nodes[i].mAABB.mExtents.y);
			if(PxAbs(Nodes[i].mAABB.mExtents.z)>EMax.z)	EMax.z = PxAbs(Nodes[i].mAABB.mExtents.z);
		}

		// Quantization
		PxU32 nbc=15;	// Keep one bit for sign
		PxU32 nbe=15;	// Keep one bit for fix
		if(!gFixQuantized) nbe++;

		// Compute quantization coeffs
		PxVec3 CQuantCoeff, EQuantCoeff;
		CQuantCoeff.x = CMax.x!=0.0f ? float((1<<nbc)-1)/CMax.x : 0.0f;
		CQuantCoeff.y = CMax.y!=0.0f ? float((1<<nbc)-1)/CMax.y : 0.0f;
		CQuantCoeff.z = CMax.z!=0.0f ? float((1<<nbc)-1)/CMax.z : 0.0f;
		EQuantCoeff.x = EMax.x!=0.0f ? float((1<<nbe)-1)/EMax.x : 0.0f;
		EQuantCoeff.y = EMax.y!=0.0f ? float((1<<nbe)-1)/EMax.y : 0.0f;
		EQuantCoeff.z = EMax.z!=0.0f ? float((1<<nbe)-1)/EMax.z : 0.0f;
		// Compute and save dequantization coeffs 
		mCenterCoeff.x = CQuantCoeff.x!=0.0f ? 1.0f / CQuantCoeff.x : 0.0f;
		mCenterCoeff.y = CQuantCoeff.y!=0.0f ? 1.0f / CQuantCoeff.y : 0.0f;
		mCenterCoeff.z = CQuantCoeff.z!=0.0f ? 1.0f / CQuantCoeff.z : 0.0f;
		mExtentsCoeff.x = EQuantCoeff.x!=0.0f ? 1.0f / EQuantCoeff.x : 0.0f;
		mExtentsCoeff.y = EQuantCoeff.y!=0.0f ? 1.0f / EQuantCoeff.y : 0.0f;
		mExtentsCoeff.z = EQuantCoeff.z!=0.0f ? 1.0f / EQuantCoeff.z : 0.0f;

		// Quantize
		for(PxU32 i=0;i<GetNbNodes();i++)
		{
			// Quantize
			mNodes[i].mAABB.mCenter[0] = PxI16(Nodes[i].mAABB.mCenter.x * CQuantCoeff.x);
			mNodes[i].mAABB.mCenter[1] = PxI16(Nodes[i].mAABB.mCenter.y * CQuantCoeff.y);
			mNodes[i].mAABB.mCenter[2] = PxI16(Nodes[i].mAABB.mCenter.z * CQuantCoeff.z);
			mNodes[i].mAABB.mExtents[0] = PxU16(Nodes[i].mAABB.mExtents.x * EQuantCoeff.x);
			mNodes[i].mAABB.mExtents[1] = PxU16(Nodes[i].mAABB.mExtents.y * EQuantCoeff.y);
			mNodes[i].mAABB.mExtents[2] = PxU16(Nodes[i].mAABB.mExtents.z * EQuantCoeff.z);
			// Fix quantized boxes
			if(gFixQuantized)
			{
				// Make sure the quantized box is still valid
				const PxVec3 Max = Nodes[i].mAABB.mCenter + Nodes[i].mAABB.mExtents;
				const PxVec3 Min = Nodes[i].mAABB.mCenter - Nodes[i].mAABB.mExtents;
				// For each axis
				for(PxU32 j=0;j<3;j++)
				{	// Dequantize the box center
					const float qc = float(mNodes[i].mAABB.mCenter[j]) * mCenterCoeff[j];
					bool FixMe=true;
					do
					{	// Dequantize the box extent
						const float qe = float(mNodes[i].mAABB.mExtents[j]) * mExtentsCoeff[j];
						// Compare real & dequantized values
						if(qc+qe<Max[j] || qc-qe>Min[j])	mNodes[i].mAABB.mExtents[j]++;
						else								FixMe=false;
						// Prevent wrapping
						if(!mNodes[i].mAABB.mExtents[j])
						{
							mNodes[i].mAABB.mExtents[j]=0xffff;
							FixMe=false;
						}
					}while(FixMe);
				}
			}


			mNodes[i].mPosIndexData = Nodes[i].mPosIndexData;
//			mNodes[i].mNegIndexData = Nodes[i].mNegIndexData;	// REMAP_XP
			mNodes[i].mEscapeIndex = Nodes[i].mEscapeIndex;	// SEPT25BUG
		}

		PX_FREE(Nodes);
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Serializes the collision tree.
 *	\param		mismatch	[in] true if big-little endian conversion has to be done
 *	\param		stream		[out] model data
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool AABBStacklessQuantizedNoLeafTree::Save(bool mismatch, PxStream& stream) const
{
#if defined(OPC_64)
	PX_ASSERT(0);
#else
	WriteDword(GetNbNodes(), mismatch, stream);

	// We can't really save the buffer in one run anymore
	for(PxU32 i=0; i < GetNbNodes(); i++)
	{
		AABBStacklessQuantizedNoLeafNode TmpCopy = mNodes[i];

		if(mismatch)
		{
			Flip(TmpCopy.mAABB.mCenter[0]);
			Flip(TmpCopy.mAABB.mCenter[1]);
			Flip(TmpCopy.mAABB.mCenter[2]);
			Flip(TmpCopy.mAABB.mExtents[0]);
			Flip(TmpCopy.mAABB.mExtents[1]);
			Flip(TmpCopy.mAABB.mExtents[2]);

			Flip(TmpCopy.mPosIndexData);
			//Flip(TmpCopy.mNegIndexData);	// REMAP_XP
			Flip(TmpCopy.mEscapeIndex);
		}

		stream.storeBuffer(&TmpCopy, sizeof(AABBStacklessQuantizedNoLeafNode));
	}

	WriteFloat(mCenterCoeff.x, mismatch, stream);
	WriteFloat(mCenterCoeff.y, mismatch, stream);
	WriteFloat(mCenterCoeff.z, mismatch, stream);
	WriteFloat(mExtentsCoeff.x, mismatch, stream);
	WriteFloat(mExtentsCoeff.y, mismatch, stream);
	WriteFloat(mExtentsCoeff.z, mismatch, stream);
#endif

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Loads a precomputed collision model.
 *	\param		mismatch	[in] true if big-little endian conversion has to be done
 *	\param		array		[in] model data
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool AABBStacklessQuantizedNoLeafTree::Load(bool mismatch, const PxStream& array)
{
	SetNbNodes(ReadDword(mismatch, array));
	PxU32 Size = GetNbNodes() * sizeof(AABBStacklessQuantizedNoLeafNode);

	PX_FREE(mNodes);
	mNodes = (AABBStacklessQuantizedNoLeafNode*)PX_ALLOC(sizeof(AABBStacklessQuantizedNoLeafNode)*GetNbNodes());

	// Get buffer
//	CopyMemory(mNodes, array.GetBuffer(Size), Size);
	array.readBuffer(mNodes, Size);

	if(mismatch)
	{
		for(PxU32 i=0;i<GetNbNodes();i++)
		{
			Flip(mNodes[i].mAABB.mCenter[0]);
			Flip(mNodes[i].mAABB.mCenter[1]);
			Flip(mNodes[i].mAABB.mCenter[2]);
			Flip(mNodes[i].mAABB.mExtents[0]);
			Flip(mNodes[i].mAABB.mExtents[1]);
			Flip(mNodes[i].mAABB.mExtents[2]);

			Flip(mNodes[i].mPosIndexData);
//			Flip(mNodes[i].mNegIndexData);	// REMAP_XP
			Flip(mNodes[i].mEscapeIndex);
		}
	}

	mCenterCoeff.x = ReadFloat(mismatch, array);
	mCenterCoeff.y = ReadFloat(mismatch, array);
	mCenterCoeff.z = ReadFloat(mismatch, array);
	mExtentsCoeff.x = ReadFloat(mismatch, array);
	mExtentsCoeff.y = ReadFloat(mismatch, array);
	mExtentsCoeff.z = ReadFloat(mismatch, array);

	return true;
}

// PX_SERIALIZATION
void AABBStacklessQuantizedNoLeafTree::exportExtraData(PxSerialStream& stream)
{
	stream.storeBuffer(mNodes, GetNbNodes()*sizeof(AABBStacklessQuantizedNoLeafNode));
}

char* AABBStacklessQuantizedNoLeafTree::importExtraData(char* address, PxU32& totalPadding)
{
	mNodes = (AABBStacklessQuantizedNoLeafNode*)address;
	address += GetNbNodes()*sizeof(AABBStacklessQuantizedNoLeafNode);

	return address;
}
//~PX_SERIALIZATION

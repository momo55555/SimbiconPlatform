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
#include "./Ice/IceContainer.h"
#include "Serialize.h"
#include "OPC_HybridModel.h"
#include "OPC_AABBTree.h"
#include "OPC_MeshInterface.h"

using namespace physx;
using namespace Ice;

OPCODECREATE::OPCODECREATE()
{
	mIMesh				= NULL;
//	mModelData			= NULL;
	mSettings.mRules	= SPLIT_SPLATTER_POINTS | SPLIT_GEOM_CENTER;
	mSettings.mLimit	= 1;	// Mandatory for complete trees
	mKeepOriginal		= false;
	mCanRemap			= false;
}

// 1: Stackless trees for non-recursive collision queries
static const PxU32 BaseModelVersion = 1;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Constructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HybridModel::HybridModel() :
	mIMesh			(NULL),
	mModelCode		(0),
	mSource			(NULL),
	mGeomEpsilon	(0.0f),
	mNbLeaves		(0),
	mNbPrimitives	(0),
	mTriangles		(NULL),
	mIndices		(NULL)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Destructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HybridModel::~HybridModel()
{
	Release();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Releases everything.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void HybridModel::Release()
{
	PX_DELETE_AND_RESET(mSource);
// PX_SERIALIZATION
	if(!isInUserMemory())
//~PX_SERIALIZATION
	{
		PX_FREE(mIndices);
		PX_FREE(mTriangles);
	}
	mIndices = NULL;
	mTriangles = NULL;
	mNbPrimitives	= 0;
	SetNbLeaves(0);
}

	struct Internal
	{
		Internal()
		{
			mNbLeaves	= 0;
			mLeaves		= NULL;
			mTriangles	= NULL;
			mBase		= NULL;
		}
		~Internal()
		{
			PX_FREE_AND_RESET(mLeaves);
		}

		PxU32			mNbLeaves;
		PxBounds3*		mLeaves;
		LeafTriangles*	mTriangles;
		const PxU32*	mBase;
	};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Builds a collision model.
 *	\param		create		[in] model creation structure
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool HybridModel::Build(const OPCODECREATE& create, AABBStacklessQuantizedNoLeafTree& resultTree)
{
	// 1) Checkings
	if(!create.mIMesh || !create.mIMesh->IsValid())	return false;

	// Look for degenerate faces.
	PxU32 NbDegenerate = create.mIMesh->CheckTopology();
//	if(NbDegenerate)	Log(_F("OPCODE WARNING: found %d degenerate faces in model! Collision might report wrong results!\n", NbDegenerate));
	// We continue nonetheless.... 

	Release();	// Make sure previous tree has been discarded

	// 1-1) Setup mesh interface automatically
	SetMeshInterface(create.mIMesh);

	bool Status = false;
	AABBTree* LeafTree = NULL;
	Internal Data;

	// 2) Build a generic AABB Tree.
	mSource = PX_NEW(AABBTree);

	// 2-1) Setup a builder. Our primitives here are triangles from input mesh,
	// so we use an AABBTreeOfTrianglesBuilder.....
	{
		AABBTreeOfTrianglesBuilder TB;
		TB.mIMesh			= create.mIMesh;
		TB.mNbPrimitives	= create.mIMesh->GetNbTriangles();
		TB.mSettings		= create.mSettings;
//		TB.mSettings.mLimit	= 16;	// ### Hardcoded, but maybe we could let the user choose 8 / 16 / 32 ...
		TB.mSettings.mLimit	= 8;	// ### Hardcoded, but maybe we could let the user choose 8 / 16 / 32 ...
		if(!mSource->Build(&TB))	goto FreeAndExit;
	}

	// 2-2) Here's the trick : create *another* AABB tree using the leaves of the first one (which are boxes, this time)
	struct Local
	{
/*		static bool CountStuff(const AABBTreeNode* current, PxU32 depth, void* user_data)
		{
			if(!current->IsLeaf())
			{
				if(current->
				Internal* Data = (Internal*)user_data;
				Data->mNbLeaves++;
			}
			return true;
		}*/

		// A callback to count leaf nodes
		static bool CountLeaves(const AABBTreeNode* current, PxU32 depth, void* user_data)
		{
			if(current->IsLeaf())
			{
				Internal* Data = (Internal*)user_data;
				Data->mNbLeaves++;
			}
			return true;
		}

		// A callback to setup leaf nodes in our internal structures
		static bool SetupLeafData(const AABBTreeNode* current, PxU32 depth, void* user_data)
		{
			if(current->IsLeaf())
			{
				Internal* Data = (Internal*)user_data;

				// Get current leaf's box
				Data->mLeaves[Data->mNbLeaves] = current->GetAABB();

				// Setup leaf data
				PxU32 Index = (PxU32)(size_t(current->GetPrimitives()) - size_t(Data->mBase))/sizeof(PxU32);
				Data->mTriangles[Data->mNbLeaves].SetData(current->GetNbPrimitives(), Index);

				Data->mNbLeaves++;
			}
			return true;
		}

		static bool RemapTree(const AABBTreeNode* current, PxU32 depth, void* user_data)
		{
			if(current->IsLeaf())
			{
				ContainerSizeT * c = (ContainerSizeT*)user_data;
				c->Add(size_t(current));
			}
			return true;
		}
	};

	// Walk the tree & count number of leaves
	Data.mNbLeaves = 0;
	mSource->Walk(Local::CountLeaves, &Data);
	SetNbLeaves(Data.mNbLeaves);	// Keep track of it

	// Special case for 1-leaf meshes
	if(Data.mNbLeaves==1)
	{
		mModelCode |= OPC_SINGLE_NODE;
		Status = true;
		goto FreeAndExit;
	}

	// Allocate our structures
	Data.mLeaves = (PxBounds3*)PX_ALLOC_TEMP(sizeof(PxBounds3)*Data.mNbLeaves);
	mTriangles = (LeafTriangles*)PX_ALLOC(sizeof(LeafTriangles)*Data.mNbLeaves);

	// Walk the tree again & setup leaf data
	Data.mTriangles	= mTriangles;
	Data.mBase		= mSource->GetIndices();
	Data.mNbLeaves	= 0;	// Reset for incoming walk
	mSource->Walk(Local::SetupLeafData, &Data);
//	mSource->Walk2(Local::SetupLeafData, &Data);

	// Handle source indices
	{
		bool MustKeepIndices = true;
		if(create.mCanRemap)
		{
			// We try to get rid of source indices (saving more ram!) by reorganizing triangle arrays...
			// Remap can fail when we use callbacks => keep track of indices in that case (it still
			// works, only using more memory)
			if(create.mIMesh->RemapClient(mSource->GetNodes()->GetNbPrimitives(), mSource->GetIndices()))
			{
				MustKeepIndices = false;
			}
		}

		if(MustKeepIndices)
		{
			// Keep track of source indices (from vanilla tree)
			mNbPrimitives = mSource->GetNodes()->GetNbPrimitives();
			mIndices = (PxU32*)PX_ALLOC(sizeof(PxU32)*mNbPrimitives);
			Ps::memCopy(mIndices, mSource->GetIndices(), mNbPrimitives*sizeof(PxU32));
		}
	}

	// Now, create our optimized tree using previous leaf nodes
	LeafTree = PX_NEW(AABBTree);
	{
		AABBTreeOfAABBsBuilder TB;	// Now using boxes !
		TB.mSettings		= create.mSettings;
		TB.mSettings.mLimit	= 1;	// We now want a complete tree so that we can "optimize" it
		TB.mNbPrimitives	= Data.mNbLeaves;
		TB.mAABBArray		= Data.mLeaves;
		if(!LeafTree->Build(&TB))	goto FreeAndExit;
	}

	if(1)	// REMAP_XP
	{
		ContainerSizeT tmp;
		LeafTree->Walk2(Local::RemapTree, &tmp);

		PxU32 Nb = tmp.GetNbEntries();
		LeafTriangles* NewLeafTriangles = (LeafTriangles*)PX_ALLOC(sizeof(LeafTriangles)*Nb);

		PxU32* Indices = (PxU32*)LeafTree->GetIndices();

		for(PxU32 i=0;i<Nb;i++)
		{
			// Get node from LeafTree
			const AABBTreeNode* current = (const AABBTreeNode*)tmp.GetEntry(i);
			PX_ASSERT(current->GetNbPrimitives()==1);
			// Get implicit index to start of leaves
//			PxU32 Index = (PxU32)(size_t(current->GetPrimitives()) - size_t(Indices))/sizeof(PxU32);
			PxU32 Index = *current->GetPrimitives();
			// Catch data associated with node
			LeafTriangles LT = mTriangles[Index];

			*(PxU32*)current->GetPrimitives() = i;
			NewLeafTriangles[i] = LT;
		}

		PX_FREE(mTriangles);
		mTriangles = NewLeafTriangles;
	}

	// 3-2) Create optimized tree
	if(!resultTree.Build(LeafTree))	goto FreeAndExit;

	// Finally ok...
	Status = true;

FreeAndExit:	// Allow me this one...
	PX_DELETE_AND_RESET(LeafTree);

	// 3-3) Delete generic tree if needed
	if(!create.mKeepOriginal)
		PX_DELETE_AND_RESET(mSource);

	return Status;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Loads a collision model.
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool HybridModel::Build(const MeshInterface& mesh, const PxStream& data)
{
    // 1) Checkings
	if(!mesh.IsValid())	return false;

	Release();	// Make sure previous tree has been discarded

	// 1-1) Setup mesh interface automatically
	SetMeshInterface(&mesh);

	// 1-2) Loads precomputed model if needed
	return Load(data);
}

static const PxU32 HybridModelVersion = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Serializes the collision model.
 *	\param		stream		[out] model data
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool HybridModel::Save(bool mismatch, PxStream& stream)	const
{
	// 1) Call base method
	{
		// Store endianness
		PxU8 StreamFlags = LittleEndian();
		if(mismatch) StreamFlags^=1;

		// Export header
		WriteChunk('O', 'P', 'C', StreamFlags, stream);
		WriteDword(BaseModelVersion, mismatch, stream);

		// Things not saved that should be restored in another way:
		// - mesh interface pointer (mIMesh)	=> use SetMeshInterface()
		// - source tree (mSource)				=> no solution yet
		WriteDword(mModelCode, mismatch, stream);
	}

	// 2) Save current level

	// Store endianness
	PxU8 StreamFlags = LittleEndian();
	if(mismatch) StreamFlags^=1;

	// Export header
	WriteChunk('H', 'B', 'M', StreamFlags, stream);
	WriteDword(HybridModelVersion, mismatch, stream);

	WriteDword(GetNbLeaves(), mismatch, stream);
	if(GetNbLeaves()>1)
	{
		PxU32 MaxIndex = computeMaxIndex(&mTriangles->Data, GetNbLeaves());
		WriteDword(MaxIndex, mismatch, stream);
		StoreIndices(MaxIndex, GetNbLeaves(), &mTriangles->Data, stream, mismatch);
	}

	WriteDword(mNbPrimitives, mismatch, stream);
	if(mNbPrimitives)
	{
		PxU32 MaxIndex = computeMaxIndex(mIndices, mNbPrimitives);
		WriteDword(MaxIndex, mismatch, stream);
		StoreIndices(MaxIndex, mNbPrimitives, mIndices, stream, mismatch);
	}

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Loads a precomputed collision model.
 *	\param		array		[in] model data
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool HybridModel::Load(const PxStream& stream)
{
	// 1) Call base method
	{
		PX_DELETE_AND_RESET(mSource);

		// Import header
		PxU8 a, b, c, d;
		ReadChunk(a, b, c, d, stream);
		if(a!='O' || b!='P' || c!='C')
			return false;

		bool Mismatch = d!=LittleEndian();

	//	PxU32 Version = array.GetDword();
		PxU32 Version = ReadDword(Mismatch, stream);

		// Only stackless trees are supported. Old formats have to be converted using the converter tool.
		if (Version < 1)
			return false;

	//	mModelCode = array.GetDword();
		mModelCode = ReadDword(Mismatch, stream);
	}

	// 2) Load current level

	PX_FREE_AND_RESET(mIndices);
	PX_FREE_AND_RESET(mTriangles);
	mNbPrimitives	= 0;
	SetNbLeaves(0);

	// Import header
	PxU8 a, b, c, d;
	ReadChunk(a, b, c, d, stream);
	if(a!='H' || b!='B' || c!='M')
		return false;

	bool Mismatch = d!=LittleEndian();

	PxU32 Version = ReadDword(Mismatch, stream);
//	if(Version==0)
	{
		SetNbLeaves(ReadDword(Mismatch, stream));
		if(GetNbLeaves()>1)
		{
			mTriangles = (LeafTriangles*)PX_ALLOC(sizeof(LeafTriangles)*GetNbLeaves());
			ReadIndices(ReadDword(Mismatch, stream), GetNbLeaves(), &mTriangles->Data, stream, Mismatch);
		}

		mNbPrimitives = ReadDword(Mismatch, stream);
		if(mNbPrimitives)
		{
			mIndices = (PxU32*)PX_ALLOC(sizeof(PxU32)*mNbPrimitives);
			ReadIndices(ReadDword(Mismatch, stream), mNbPrimitives, mIndices, stream, Mismatch);
		}
	}
/*	else
	{
		mNbLeaves = ReadDword(Mismatch, stream);
		mNbPrimitives = ReadDword(Mismatch, stream);

		PxU8* Memory = (PxU8*)ICE_ALLOC(sizeof(LeafTriangles)*mNbLeaves + sizeof(PxU32)*mNbPrimitives);

		if(mNbLeaves>1)
		{
			mTriangles = (LeafTriangles*)Memory;
			ReadIndices(ReadDword(Mismatch, stream), mNbLeaves, &mTriangles->Data, stream, Mismatch);
		}

		if(mNbPrimitives)
		{
			mIndices = (PxU32*)(Memory + sizeof(LeafTriangles)*mNbLeaves);
			ReadIndices(ReadDword(Mismatch, stream), mNbPrimitives, mIndices, stream, Mismatch);
		}
	}*/
	return true;
}

/*PX_INLINE void ComputeMinMax(PxVec3& minimum, PxVec3& maximum, const VertexPointers& vp)
{
	// Compute triangle's AABB = a leaf box
	minimum = *vp.vertex[0];
	maximum = *vp.vertex[0];
	minimum.Min(*vp.vertex[1]);
	maximum.Max(*vp.vertex[1]);
	minimum.Min(*vp.vertex[2]);
	maximum.Max(*vp.vertex[2]);
}*/

// PX_SERIALIZATION
void HybridModel::exportExtraData(PxSerialStream& stream)
{
	PX_ASSERT(!mSource);
	mRTree.exportExtraData(stream);

	stream.storeBuffer(mTriangles, GetNbLeaves()*sizeof(LeafTriangles));
	if(mIndices)
		stream.storeBuffer(mIndices, mNbPrimitives*sizeof(PxU32));
}

char* HybridModel::importExtraData(char* address, PxU32& totalPadding)
{
	address = mRTree.importExtraData(address, totalPadding);

	mTriangles = (LeafTriangles*)address;
	address += GetNbLeaves()*sizeof(LeafTriangles);
	if(mIndices)
	{
		mIndices = (PxU32*)address;
		address += mNbPrimitives*sizeof(PxU32);
	}

	return address;
}

//~PX_SERIALIZATION

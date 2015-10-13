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
#include "SqStaticPruner.h"
#include "OPC_AABBTree.h"
#include "OPC_TreeBuilders.h"
#include "SqAABBColliderExt.h"
#include "SqLSSColliderExt.h"
#include "SqOBBColliderExt.h"
#include "SqSphereColliderExt.h"
#include "SqRayColliderExt.h"
#include "SqUtilities.h"

using namespace physx;
using namespace Sq;

// The following macro test is only needed for the dynamic trees. However, since the dynamic trees share some
// functionality with the static trees, this test is defined and used in the static trees to avoid code duplication.
#define SKIP_IF_INVALID_PRIMITIVE(primitive) if ((primitive) == PX_INVALID_U32) continue;	// The primitive index belongs to a deleted pruning object -> skip

StaticPruner::StaticPruner() :
	mAABBTree	(NULL)
{
	mCullFunc			= (CullFunc)			&StaticPruner::Cull;
	mStabFunc			= (StabFunc)			&StaticPruner::Stab;
	mOverlapSphereFunc	= (OverlapSphereFunc)	&StaticPruner::OverlapSphere;
	mOverlapAABBFunc	= (OverlapAABBFunc)		&StaticPruner::OverlapAABB;
	mOverlapOBBFunc		= (OverlapOBBFunc)		&StaticPruner::OverlapOBB;
	mOverlapCapsuleFunc	= (OverlapCapsuleFunc)	&StaticPruner::OverlapCapsule;
}

StaticPruner::~StaticPruner()
{
	PX_DELETE_AND_RESET(mAABBTree);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Builds an AABB-tree for objects in the pruning pool.
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool StaticPruner::BuildAABBTree()
{
	// Release possibly already existing tree
	PX_DELETE_AND_RESET(mAABBTree);

	// Don't bother building an AABB-tree if there isn't a single static object
	PxU32 NbObjects = GetNbActiveObjects();
	if(!NbObjects)	return true;

	// Profile it with a big warning so that we can catch
	// broken situations where the tree is built each frame (this is bad of course)
	bool Status;
	{
		// Create a new tree
		mAABBTree = PX_NEW(AABBTree);

		// Build a tree of AABBs
		// WARNING: this is where we need all the internal boxes to be valid
		// So update them all first !
		Prunable** Objects = GetObjects();
		for(PxU32 i=0;i<NbObjects;i++)	GetWorldAABB(*(*Objects++));

		// Then we can safely access the whole array.
		AABBTreeOfAABBsBuilder TB;
		TB.mNbPrimitives	= NbObjects;
		TB.mAABBArray		= GetCurrentWorldBoxes();
//		TB.mRules			= SPLIT_COMPLETE|SPLIT_SPLATTER_POINTS;
//		TB.mSettings.mRules	= SPLIT_SPLATTER_POINTS;
		TB.mSettings.mRules	= SPLIT_SPLATTER_POINTS|SPLIT_GEOM_CENTER;
		TB.mSettings.mLimit	= 1;
//		TB.mSettings.mLimit	= 4;
//		TB.mSettings.mLimit	= 8;
		Status = mAABBTree->Build(&TB);
	}

	PostBuildCallback();

	return Status;
}

void StaticPruner::eagerUpdatePruningTrees()
{
	if(!mAABBTree)
		BuildAABBTree();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Adds an object to the pruner.
 *	\param		object	[in] the object to register
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool StaticPruner::AddObject(Prunable& object)
{
	// Maybe deletion isn't always needed

	// If a tree has already been built for some objects, we need to discard it
	PX_DELETE_AND_RESET(mAABBTree);

	return Pruner::AddObject(object);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Removes an object from the pruner.
 *	\param		object	[in] the object to remove
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool StaticPruner::RemoveObject(Prunable& object)
{
	// Maybe deletion isn't always needed

	// The AABB-tree-to-arrays mapping is now invalid...  => rebuild the whole tree
	PX_DELETE_AND_RESET(mAABBTree);

	return Pruner::RemoveObject(object);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Updates an object, i.e. updates the pruner's spatial database.
 *	\param		object	[in] the object to update
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool StaticPruner::UpdateObject(Prunable& object)
{
	// Maybe deletion isn't always needed

	// Delete current tree - it will get lazy-rebuilt.
	PX_DELETE_AND_RESET(mAABBTree);

	return Pruner::UpdateObject(object);
}

void StaticPruner::addShapes(PxU32 nbShapes, SceneQueryShapeData*const* PX_RESTRICT shapes)
{
	SceneQueryShapeData* nextSqData = shapes[0];
	for(PxU32 i=0; i<nbShapes; i++)
	{
		SceneQueryShapeData* sqData = nextSqData;
		PX_ASSERT(sqData);

		if(i!=nbShapes-1)
		{
			nextSqData = shapes[i+1];
			Ps::prefetch128(nextSqData);
		}

		// PT: same as "AddObject", optimized
		PX_ASSERT(sqData->mHandle==INVALID_PRUNING_HANDLE);

		// AddObject
		{
			// Add the object to the pool
			PruningPool::AddObject(*sqData);
		}

		// Update object is empty
	}

	if(nbShapes)
	{
		// Invalidate acceleration structure
		mSignature.Invalidate();

		// If a tree has already been built for some objects, we need to discard it
		PX_DELETE_AND_RESET(mAABBTree);
	}
}



	struct CullData
	{
		CulledObjects*		objects;
		Prunable**			Objects;
		const Gu::Plane*	planes;
		PxU32				nb_planes;
	};

bool StaticPruner::Cull(PruningTemps& temps, CulledObjects& objects, const Gu::Plane* planes, PxU32 nb_planes, PxU32 culling_flags)
{
// ### check shared arrays

	// Lazy build
	if(!mAABBTree)	BuildAABBTree();
	if(!mAABBTree)	return true;

#define USING_CALLBACKS

#ifdef USING_CALLBACKS
	struct Local
	{
		// Could use multiple callbacks instead
		static void CullCB(PxU32 nb_primitives, PxU32* node_primitives, Ps::IntBool need_clipping, void* user_data)
		{
			CullData* Data = (CullData*)user_data;

			CulledObjects*	list = Data->objects;
			Prunable**	Objects = Data->Objects;

			if(need_clipping)
			{
				{
					while(nb_primitives--)
					{
						SKIP_IF_INVALID_PRIMITIVE(*node_primitives)

						Prunable* PRN = Objects[*node_primitives++];

						list->AddPrunable(PRN);
					}
				}
			}
			else
			{
				while(nb_primitives--)
				{
					SKIP_IF_INVALID_PRIMITIVE(*node_primitives)

					Prunable* PRN = Objects[*node_primitives++];
					list->AddPrunable(PRN);
				}
			}
		}
	};

	CullData CD;
	CD.objects		= &objects;
	CD.Objects		= GetObjects();
	CD.planes		= planes;
	CD.nb_planes	= nb_planes;
	mAABBTree->TestAgainstPlanes(planes, (1<<nb_planes)-1, Local::CullCB, &CD);
#else
	// OLD WAY :

	// Indices in mWorldBoxes
	/*
	WARNING: using this code would result in both threading problems and 64bit issues.
	Container is static and contains pointers.
	*/
	static Container VisibleBoxIndicesClip;
	static Container VisibleBoxIndicesNoClip;

	VisibleBoxIndicesClip.Reset();
	VisibleBoxIndicesNoClip.Reset();

		mAABBTree->_TestAgainstPlanes(planes, (1<<nb_planes)-1, VisibleBoxIndicesClip, VisibleBoxIndicesNoClip);

		Prunable** Objects = mPool.GetObjects();

		// Handle clipped objects

		PxU32 NbVis = VisibleBoxIndicesClip.GetNbEntries();
		PxU32* Indices = VisibleBoxIndicesClip.GetEntries();
		while(NbVis--)
		{
			Prunable* PRN = Objects[*Indices++];

if(HullCulling)
{
CullingHull* CH = PRN->GetCullingHull();
if(CH)
{
	ASSERT(PRN->GetWorldTransform());
	if(!CH->IsVisible(nb_planes, planes, *PRN->GetWorldTransform()))	continue;
}
}

//			if(setupclip)	list.AddPrunable(PRN, true);
//			else			list.AddPrunable(PRN);
			list.AddPrunable(PRN, SetupClip);
		}

		// Handle non-clipped objects

		NbVis = VisibleBoxIndicesNoClip.GetNbEntries();
		Indices = VisibleBoxIndicesNoClip.GetEntries();
		while(NbVis--)
		{
			const Prunable* PRN = Objects[*Indices++];

//			if(setupclip)	list.AddPrunable(PRN, false);
//			else			list.AddPrunable(PRN);
			list.AddPrunable(PRN);
		}
#endif

	return true;
}

PxU32 StaticPruner::Stab(StabCallback callback, void* user_data, const PxVec3& orig, const PxVec3& dir, float& max_dist)
{
	// Lazy build
	if(!mAABBTree)	BuildAABBTree();
	if(!mAABBTree)	return STAB_CONTINUE;

	struct LocalParams
	{
		Prunable**		mObjects;
		StabCallback	mCallback;
		void*			mUserData;
	};

	struct Local
	{
		static PxU32 CB(const PxU32* indices, PxU32 nbIndices, float& maxDist, void* user_data)
		{
			const LocalParams* Params = (const LocalParams*)user_data;

			PxU32 Ret = STAB_CONTINUE;
			Prunable** PX_RESTRICT Objects = Params->mObjects;
			while(nbIndices--)
			{
				Prunable* P = Objects[*indices++];

				PxU32 Status = (Params->mCallback)(P, maxDist, Params->mUserData);
				if(Status & STAB_STOP)
					return STAB_STOP;
				if(Status & STAB_UPDATE_MAX_DIST)
					Ret = STAB_UPDATE_MAX_DIST;
			}
			return Ret;
		}
	};

	LocalParams LP;
	LP.mObjects		= GetObjects();
	LP.mCallback	= callback;
	LP.mUserData	= user_data;

	RayColliderExt RC;
	RC.SetFirstContact(false);
	RC.SetMaxDist(max_dist);

	if(max_dist==PX_MAX_F32)
		RC.collideExt(orig, dir, mAABBTree, Local::CB, &LP);
	else
		RC.closestHitExt(orig, dir, mAABBTree, Local::CB, &LP);
	return STAB_CONTINUE;
}

// PT: TODO: remove 'first_contact' parameter
bool StaticPruner::OverlapSphere(ReportPrunablesCallback cb, void* userData, const Gu::Sphere& sphere, bool first_contact)
{
	// Lazy build
	if(!mAABBTree)	BuildAABBTree();
	if(!mAABBTree)	return true;

	SphereColliderExt collider;
	collider.collideExt(GetObjects(), cb, userData, sphere, mAABBTree);
	return collider.AbortQuery()==0;
}

bool StaticPruner::OverlapAABB(ReportPrunablesCallback cb, void* userData, const PxBounds3& box, bool first_contact)
{
	// Lazy build
	if(!mAABBTree)	BuildAABBTree();
	if(!mAABBTree)	return true;

	AABBColliderExt collider;
	collider.collideExt(GetObjects(), cb, userData, CollisionAABB(box), mAABBTree);
	return collider.AbortQuery()==0;
}

bool StaticPruner::OverlapOBB(ReportPrunablesCallback cb, void* userData, const Gu::Box& box, bool first_contact)
{
	// Lazy build
	if(!mAABBTree)	BuildAABBTree();
	if(!mAABBTree)	return true;

	OBBColliderExt collider;
	collider.collideExt(GetObjects(), cb, userData, box, mAABBTree);
	return collider.AbortQuery()==0;
}

bool StaticPruner::OverlapCapsule(ReportPrunablesCallback cb, void* userData, const Gu::Capsule& capsule, bool first_contact)
{
	// Lazy build
	if(!mAABBTree)	BuildAABBTree();
	if(!mAABBTree)	return true;

	LSSColliderExt collider;
	collider.collideExt(GetObjects(), cb, userData, capsule, mAABBTree);
	return collider.AbortQuery()==0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "CmRenderOutput.h"
void StaticPruner::visualize(Cm::RenderOutput& out, PxU32 color)
{
	const AABBTree* tree = HasAABBTree();

	if(tree)
	{
		struct Local
		{
			static void _Draw(const AABBTreeNode* node, Cm::RenderOutput& out)
			{
				if(!node)	return;

				out << Cm::DebugBox(node->GetAABB(), true);

				_Draw(node->GetPos(), out);
				_Draw(node->GetNeg(), out);
			}
		};
		PxTransform idt = PxTransform::createIdentity();
		out << idt;
		out << color;
		Local::_Draw(tree->GetNodes(), out);
	}
}
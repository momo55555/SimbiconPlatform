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
#include "OPC_RayCollider.h"

using namespace physx;
using namespace Ice;

RayCollider::RayCollider() :
#ifdef OPC_RAYHIT_CALLBACK
	mHitCallback		(NULL),
	mUserData			(0),
#else
	mStabbedFaces		(NULL),
#endif
	mNbIntersections	(0),
	mMaxDist			(PX_MAX_F32),
#ifdef OPC_GEOM_EPSILON
	mGeomEpsilon		(0.0f),
#endif
#ifndef OPC_RAYHIT_CALLBACK
	mClosestHit			(false),
#endif
	mCulling			(true)
{
}

Ps::IntBool RayCollider::InitQuery(const PxVec3& orig, const PxVec3& dir, const Cm::Matrix34* world, PxU32* face_id)
{
	// Reset stats & contact status
	mNbIntersections	= 0;
#ifndef OPC_RAYHIT_CALLBACK
	if(mStabbedFaces)
		mStabbedFaces->Reset();
#endif

	// Compute ray in local space
	// The (Origin/Dir) form is needed for the ray-triangle test anyway (even for segment tests)
	if(world)
	{
		mDir = world->rotateTranspose(dir);

		Cm::Matrix34 World = world->getInverseRT();
		mOrigin = World.transform(orig);
	}
	else
	{
		mDir	= dir;
		mOrigin	= orig;
	}

	// 4) Special case: 1-triangle meshes [Opcode 1.3]
//	if(mCurrentModel && mCurrentModel->HasSingleNode())
	if(mCurrentModel && mCurrentModel->HasSingleNode())
	{
		// We simply perform the BV-Prim overlap test each time. We assume single triangle has index 0.
		if(!SkipPrimitiveTests())
		{
			// Perform overlap test between the unique triangle and the ray (and set contact status if needed)
			PerformSegmentPrimTest(PxU32(0), OPC_CONTACT);

			// Return immediately regardless of status
			return Ps::IntTrue;
		}
	}

	// Precompute data (moved after temporal coherence since only needed for ray-AABB)
	if(mMaxDist!=PX_MAX_F32)
	{
		SetupSegment();
	}
	else
	{
		// For Ray-AABB overlap
//		PxU32 x = SIR(mDir.x)-1;
//		PxU32 y = SIR(mDir.y)-1;
//		PxU32 z = SIR(mDir.z)-1;
//		mData.x = FR(x);
//		mData.y = FR(y);
//		mData.z = FR(z);

		// Precompute mFDir;
		mFDir.x = PxAbs(mDir.x);
		mFDir.y = PxAbs(mDir.y);
		mFDir.z = PxAbs(mDir.z);
	}

	return Ps::IntFalse;
}


#ifdef USE_SHRINKING_CALLBACK
// ### we can't shrink the "rays" so we need this one here
void HybridRayCollider::TestLeaf(PxU32 touched_box)
{
//	const HybridModel* model = (const HybridModel*)mCurrentModel;
	const HybridModelData* PX_RESTRICT model = static_cast<const HybridModelData*>(mCurrentModel);

	const PxU32* PX_RESTRICT Indices = model->GetIndices();
	const LeafTriangles* PX_RESTRICT LT = model->GetLeafTriangles();
	const LeafTriangles& CurrentLeaf = LT[touched_box];

	// Each leaf box has a set of triangles
	PxU32 NbTris = CurrentLeaf.GetNbTriangles();
	if(Indices)
	{
		const PxU32* PX_RESTRICT T = &Indices[CurrentLeaf.GetTriangleIndex()];

		// Loop through triangles and test each of them
		while(NbTris--)
		{
			PerformSegmentPrimTest(*T++, OPC_CONTACT);
 			if(ContactFound())
				return;
		}
	}
	else
	{
		PxU32 BaseIndex = CurrentLeaf.GetTriangleIndex();

		// Loop through triangles and test each of them
		while(NbTris--)
		{
			PerformSegmentPrimTest(BaseIndex++, OPC_CONTACT);
 			if(ContactFound())
				return;
		}
	}
}

void HybridRayCollider::TestLeafAndShrink(PxU32 touched_box)
{
//	const HybridModel* model = (const HybridModel*)mCurrentModel;
	const HybridModelData* PX_RESTRICT model = static_cast<const HybridModelData*>(mCurrentModel);

	const PxU32* PX_RESTRICT Indices = model->GetIndices();
	const LeafTriangles* PX_RESTRICT LT = model->GetLeafTriangles();
	const LeafTriangles& CurrentLeaf = LT[touched_box];

	// Each leaf box has a set of triangles
	PxU32 NbTris = CurrentLeaf.GetNbTriangles();
	if(Indices)
	{
		const PxU32* PX_RESTRICT T = &Indices[CurrentLeaf.GetTriangleIndex()];

		// Loop through triangles and test each of them
		while(NbTris--)
		{
			PerformSegmentPrimTestAndShrink(*T++, OPC_CONTACT);
 			if(ContactFound())
				return;
		}
	}
	else
	{
		PxU32 BaseIndex = CurrentLeaf.GetTriangleIndex();

		// Loop through triangles and test each of them
		while(NbTris--)
		{
			PerformSegmentPrimTestAndShrink(BaseIndex++, OPC_CONTACT);
 			if(ContactFound())
				return;
		}
	}
}
#endif


//bool HybridRayCollider::Collide(const PxVec3& orig, const PxVec3& dir, const HybridModel& model, const Cm::Matrix34* world, PxU32* cache)
bool HybridRayCollider::Collide(const PxVec3& orig, const PxVec3& dir, const HybridModelData& model, const Cm::Matrix34* world, PxU32* cache, RayColliderContactCallback* contactCallback)
{
	// - disable primitive tests since we don't store triangles in our leaves
	// AP: this seems to be really necessary since otherwise InitQuery fails one of the unit tests
	// The reason it's counterintuitive is it forces no prim tests, even though external sticky state
	// settings can indicate otherwise.. The rationale behind this is not entirely clear.
	// In general it appears that we'd be better off without sticky state in colliders
	// Perhaps some form of "ColliderSettings" struct passed as an argument would be a more intuitive
	// and less error prone alternative
	mFlags |= OPC_NO_PRIMITIVE_TESTS;

	// Checkings
	if(!Setup(&model))
		return false;

	const MeshInterface* mi = mIMesh;
	PxMemFetchSmallBuffer buf0, buf1, buf2;
	PxU32* has16BitIndices = pxMemFetchAsync<PxU32>(PxMemFetchPtr(mi)+OFFSET_OF(MeshInterface, mHas16BitIndices), 5, buf0);
	PxMemFetchPtr* mTris = pxMemFetchAsync<PxMemFetchPtr>(PxMemFetchPtr(mi)+OFFSET_OF(MeshInterface, mTris), 5, buf1);
	PxMemFetchPtr* mVerts = pxMemFetchAsync<PxMemFetchPtr>(PxMemFetchPtr(mi)+OFFSET_OF(MeshInterface, mVerts), 5, buf2);
	pxMemFetchWait(5);
	#undef sdmaGet

	// Init collision query
	if(InitQuery(orig, dir, world, cache))
	{
		if(mNbIntersections>0 && contactCallback)
		{
			PxVec3 v0, v1, v2;
			MeshInterface::GetTriangleVerts(*has16BitIndices, *mTris, *mVerts, mStabbedFace.mFaceID, v0, v1, v2);

			contactCallback->processResults(*has16BitIndices, mTris, &mStabbedFace, v0, v1, v2);
		}

		return true;
	}

	// Special case for 1-leaf trees
	if(mCurrentModel && mCurrentModel->HasSingleNode())
	{
		PxMemFetchSmallBuffer buf4;
		PxU32* Nb = pxMemFetchAsync<PxU32>(PxMemFetchPtr(mi)+OFFSET_OF(MeshInterface, mNbTris), 5, buf4);
		pxMemFetchWait(5);

		// Here we're supposed to perform a normal query, except our tree has a single node, i.e. just a few triangles

		PxU32 BaseIndex = 0;
		const PxU32 N = 8;
		for(PxU32 iTri = 0; iTri < *Nb; iTri+=N)
		{
			PxVec3 v[N][3];
			PxU32 countLeft = iTri+N > *Nb ? *Nb-iTri : N;
			MeshInterface::getTriangleVertsN<N>(*has16BitIndices, *mTris, *mVerts, BaseIndex+iTri, countLeft, v);

			for(PxU32 jj = 0; jj < countLeft; jj++)
			{
				const PxU32 TriangleIndex = BaseIndex++;
				const PxVec3& v0 = v[jj][0];
				const PxVec3& v1 = v[jj][1];
				const PxVec3& v2 = v[jj][2];

				if(RayTriOverlap(v0, v1, v2))
				{

					// Intersection point is valid if dist < segment's length
					// We know dist>0 so we can use integers
					if(PX_IR(mStabbedFace.mDistance)<=PX_IR(mMaxDist))
					{
						HandleContact(TriangleIndex, OPC_CONTACT);

						//If we want all hit or any hit, call callback for each contact, eithewise call at the end
						if(!mClosestHit && contactCallback)
						{
							contactCallback->processResults(*has16BitIndices, mTris, &mStabbedFace, v0, v1, v2);
						}
					}
				}
				//If found first contact, early out
				if(ContactFound())
					return true; // force early out
			}
		}


		//callback for closet hit
		if(mClosestHit && contactCallback && (mNbIntersections > 0))
		{
			PxVec3 v0,v1,v2;
			MeshInterface::GetTriangleVerts(*has16BitIndices, *mTris, *mVerts, const_cast<CollisionFace*>(mStabbedFaces->GetFaces())->mFaceID, v0, v1, v2);
			contactCallback->processResults(*has16BitIndices, mTris, const_cast<CollisionFace*>(mStabbedFaces->GetFaces()), v0, v1, v2);
		}

		return true;
	}

		struct RayRTreeCallback : Gu::RTree::Callback
		{
			HybridRayCollider* parent;
			const HybridModelData& model;
			RayColliderContactCallback* contactCallback;
			PxU32 has16BitIndices;
			PxMemFetchPtr* mTris;
			PxMemFetchPtr* mVerts;

			RayRTreeCallback(HybridRayCollider* parent, const HybridModelData& model, RayColliderContactCallback* callback, PxU32 has16BitIndices, PxMemFetchPtr* tris, PxMemFetchPtr* verts)
				: parent(parent), model(model), contactCallback(callback), has16BitIndices(has16BitIndices), mTris(tris), mVerts(verts)
			{
			}

			virtual bool processResults(PxU32 NumTouched, PxU32* Touched)
			{
				PX_ASSERT(NumTouched > 0);

				const PxU32* Indices = model.GetIndices();

				// Loop through touched leaves
				PxU32 Nb = NumTouched;
				while(Nb--)
				{
					LeafTriangles CurrentLeaf;
					CurrentLeaf.Data = *Touched++;

					// Each leaf box has a set of triangles
					PxU32 NbTris = CurrentLeaf.GetNbTriangles();

					if(Indices)
					{
						const PxU32* T = &Indices[CurrentLeaf.GetTriangleIndex()];

						PxVec3 v0,v1,v2;
						// Loop through triangles and test each of them
						while(NbTris--)
						{
							const PxU32 TriangleIndex = *T++;
							{
								// Request vertices from the app
								MeshInterface::GetTriangleVerts(has16BitIndices, *mTris, *mVerts, TriangleIndex, v0, v1, v2);

								// Perform ray-tri overlap test
								if(parent->RayTriOverlap(v0, v1, v2))
								{
									// Intersection point is valid if dist < segment's length
									// We know dist>0 so we can use integers
									if(PX_IR(parent->mStabbedFace.mDistance)<=PX_IR(parent->mMaxDist))
									{
										parent->HandleContact(TriangleIndex, OPC_CONTACT);

										//If we want all hit or any hit, call callback for each contact, eithewise call at the end
										if(!parent->mClosestHit && contactCallback)
										{
											contactCallback->processResults(has16BitIndices, mTris, &parent->mStabbedFace, v0, v1, v2);
										}
									}
								}

								//If found first contact, early out
								if(parent->ContactFound())
									return false;

							//Now is the time to report closest hit
	
							}
						}
					}
					else
					{
						PxU32 BaseIndex = CurrentLeaf.GetTriangleIndex();
						const PxU32 N = 8;
						for(PxU32 iTri = 0; iTri < NbTris; iTri+=N)
						{
							PX_ASSERT(!Indices);
							PxVec3 v[N][3];
							PxU32 countLeft = iTri+N > NbTris ? NbTris-iTri : N;
							MeshInterface::getTriangleVertsN<N>(has16BitIndices, *mTris, *mVerts, BaseIndex+iTri, countLeft, v);

							for(PxU32 jj = 0; jj < countLeft; jj++)
							{
								const PxU32 TriangleIndex = BaseIndex++;
								const PxVec3& v0 = v[jj][0];
								const PxVec3& v1 = v[jj][1];
								const PxVec3& v2 = v[jj][2];

								if(parent->RayTriOverlap(v0, v1, v2))
								{
									// Intersection point is valid if dist < segment's length
									// We know dist>0 so we can use integers
									if(PX_IR(parent->mStabbedFace.mDistance)<=PX_IR(parent->mMaxDist))
									{
										parent->HandleContact(TriangleIndex, OPC_CONTACT);

										//If we want all hit or any hit, call callback for each contact, eithewise call at the end
										if(!parent->mClosestHit && contactCallback)
										{
											contactCallback->processResults(has16BitIndices, mTris, &parent->mStabbedFace, v0, v1, v2);
										}
									}
								}

								if(parent->ContactFound())
									return false; // force early out
							}
						}				
					}
				}

				// since this is an early-out test, we tell rtree to abort traversal as soon as we see any results
				if (parent->ContactFound())
					return false;
				else
					return true;
			}
		} callback(this, model, contactCallback, *has16BitIndices, mTris, mVerts);

		const PxU32 maxResults = Gu::RTreePage::SIZE; // maxResults=rtree page size for more efficient early out
		PxU32 buf[maxResults];

		if (mMaxDist != PX_MAX_F32)
			model.mRTree->traverseRay<0, 1>(mOrigin, mDir * mMaxDist, maxResults, buf, &callback, PxVec3(0.0f));
		else
			model.mRTree->traverseRay<0, 0>(mOrigin, mDir, maxResults, buf, &callback, PxVec3(0.0f));

		PxU32 numTrigs = mStabbedFaces->GetNbFaces();
		if(!numTrigs)
			return true;

		 if(mClosestHit && contactCallback)
		{
			PxVec3 v0,v1,v2;
			MeshInterface::GetTriangleVerts(*has16BitIndices, *mTris, *mVerts, const_cast<CollisionFace*>(mStabbedFaces->GetFaces())->mFaceID, v0, v1, v2);
			contactCallback->processResults(*has16BitIndices, mTris, const_cast<CollisionFace*>(mStabbedFaces->GetFaces()), v0, v1, v2);

		}

	return true; // AP: scaffold, why do we always return true here?
}

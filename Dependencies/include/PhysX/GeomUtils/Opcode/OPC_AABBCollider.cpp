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
#include "OPC_AABBCollider.h"
#include "OPC_OBBCollider.h"

using namespace physx;
using namespace Ice;

#include "CmSimd.h"
#include "CmReflection.h"

//#include "OPC_BoxBoxOverlap.h"

void HybridAABBCollider::Collide(
	const CollisionAABB& box,
	const HybridModelData& model, bool primTests,
	VolumeColliderTrigCallback* parentCallback
)
{
	mCurrentModel = &model;
	mIMesh = model.mIMesh;
	mBox = box;
	mMin = box.mCenter - box.mExtents;
	mMax = box.mCenter + box.mExtents;

	PX_ASSERT(model.mRTree->mPages);

	struct PrimTestCallback : Gu::RTree::Callback
	{
		HybridAABBCollider*			parent;
		const HybridModelData*		model;
		VolumeColliderTrigCallback*	filteredCallback;
		bool						primTests;

		PrimTestCallback(HybridAABBCollider* parent, const HybridModelData* model, VolumeColliderTrigCallback* fc, bool primTests)
			: parent(parent), model(model), filteredCallback(fc), primTests(primTests)
		{
		}

		virtual bool processResults(PxU32 Nb, PxU32* Touched)
		{
			const PxU32* PX_RESTRICT Indices = model->GetIndices();
			PX_FORCE_PARAMETER_REFERENCE(Indices);

			const MeshInterface* mi = parent->mIMesh;

			PxMemFetchSmallBuffer buf0, buf1, buf2;
			PxU32* has16BitIndices = pxMemFetchAsync<PxU32>(PxMemFetchPtr(mi)+OFFSET_OF(MeshInterface, mHas16BitIndices), 5, buf0);
			PxMemFetchPtr* mTris = pxMemFetchAsync<PxMemFetchPtr>(PxMemFetchPtr(mi)+OFFSET_OF(MeshInterface, mTris), 5, buf1);
			PxMemFetchPtr* mVerts = pxMemFetchAsync<PxMemFetchPtr>(PxMemFetchPtr(mi)+OFFSET_OF(MeshInterface, mVerts), 5, buf2);
			pxMemFetchWait(5);

			// new buffer for trig results, can be bigger than touched leaves so we can't reuse Touched in place
			const PxU32 trigBufferSize = 32;
			PxVec3 trigBuffer[trigBufferSize*3];
			PxU32 indexBuffer[trigBufferSize];
			PxU32 numTrigsHit = 0;
			while(Nb--)
			{
				// Each leaf box has a set of triangles
				LeafTriangles CurrentLeaf;
				CurrentLeaf.Data = *Touched++;
				PxU32 NbTris = CurrentLeaf.GetNbTriangles();
				PxU32 BaseIndex = CurrentLeaf.GetTriangleIndex();

				PX_ASSERT(!Indices);	// PT: TODO: remove dead code now that Indices are not supported anymore

				#if defined(__SPU__)
				const PxU32 N = 8;
				for(PxU32 iTri = 0; iTri < NbTris; iTri+=N)
				{
					PxVec3 v[N][3];
					PxU32 countLeft = iTri+N > NbTris ? NbTris-iTri : N;
					MeshInterface::getTriangleVertsN<N>(*has16BitIndices, *mTris, *mVerts, BaseIndex+iTri, countLeft, v);

					for(PxU32 jj = 0; jj < countLeft; jj++)
					{
						const PxVec3& v0 = v[jj][0];
						const PxVec3& v1 = v[jj][1];
						const PxVec3& v2 = v[jj][2];
						if(!primTests || parent->TriBoxOverlap(v0, v1, v2))										
						{
							trigBuffer[numTrigsHit*3+0] = v0;
							trigBuffer[numTrigsHit*3+1] = v1;
							trigBuffer[numTrigsHit*3+2] = v2;
							indexBuffer[numTrigsHit++] = BaseIndex+iTri+jj;
							if (numTrigsHit == trigBufferSize)
							{
								// flush partial results, reset trig count
								filteredCallback->processResults(numTrigsHit, trigBuffer, indexBuffer);
								numTrigsHit = 0;
							}
						}
					}
				}
				#else // #ifdef __SPU__
				while(NbTris--)
				{
					const PxU32 TriangleIndex = BaseIndex++;
#ifdef PX_X360
					if(NbTris)
						MeshInterface::prefetchTriangleVerts(*has16BitIndices, mi->GetTris(), mi->GetVerts(), BaseIndex);
#endif
					PxVec3 v0, v1, v2;
					MeshInterface::GetTriangleVerts(*has16BitIndices, *mTris, *mVerts, TriangleIndex, v0, v1, v2);

					if(!primTests || parent->TriBoxOverlap(v0, v1, v2))										
					{
						trigBuffer[numTrigsHit*3+0] = v0;
						trigBuffer[numTrigsHit*3+1] = v1;
						trigBuffer[numTrigsHit*3+2] = v2;
						indexBuffer[numTrigsHit++] = TriangleIndex;
						if(numTrigsHit == trigBufferSize)
						{
							// flush partial results, reset trig count
							filteredCallback->processResults(numTrigsHit, trigBuffer, indexBuffer);
							numTrigsHit = 0;
						}
					}
				}
				#endif // #ifdef __SPU__ #else
			}

			if(!numTrigsHit)
				return true;

			return filteredCallback->processResults(numTrigsHit, trigBuffer, indexBuffer); // invoke parent callback with reduced buffer

		} // processResults
	} primTestCallback(this, &model, parentCallback, primTests);

	const PxU32 bufSize = 64;
	PxU32 resBuffer[bufSize];
	model.mRTree->traverseAABB(mMin, mMax, bufSize, resBuffer, &primTestCallback);
}


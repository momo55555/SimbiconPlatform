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
#include "CmReflection.h" // for OFFSET_OF
#include "../../LowLevel/common/include/utils/PxcMemFetch.h"
#include "OPC_AABBCollider.h"
#include "OPC_OBBCollider.h"

using namespace physx;
using namespace Ice;

//#include "OPC_BoxBoxOverlap.h"

Ps::IntBool OBBCollider::InitQuery(const Gu::Box& box, const Cm::Matrix34* worldb, const Cm::Matrix34* worldm)
{
	// 1) Call the base method
	VolumeCollider::InitQuery();

	// 2) Compute obb in world space
	mBoxExtents = box.extents;

	Cm::Matrix34 WorldB;

	if(worldb)
	{
		const PxMat33 tmp = PxMat33(worldb->base0, worldb->base1, worldb->base2) * box.rot;
		WorldB.base0 = tmp.column0;
		WorldB.base1 = tmp.column1;
		WorldB.base2 = tmp.column2;
		WorldB.base3 = worldb->transform(box.center);
	}
	else
	{
		WorldB.base0 = box.rot.column0;
		WorldB.base1 = box.rot.column1;
		WorldB.base2 = box.rot.column2;
		WorldB.base3 = box.center;
	}

	// Setup matrices
	if(worldm)
	{
		Cm::Matrix34 WorldToModel	= worldm->getInverseRT();
		Cm::Matrix34 WorldToBox		= WorldB.getInverseRT();

		Cm::Matrix34 ModelToBox = WorldToBox * *worldm;
		Cm::Matrix34 BoxToModel = WorldToModel * WorldB;

		mRModelToBox.column0	= ModelToBox.base0;
		mRModelToBox.column1	= ModelToBox.base1;
		mRModelToBox.column2	= ModelToBox.base2;
		mTModelToBox			= ModelToBox.base3;

		mRBoxToModel.column0	= BoxToModel.base0;
		mRBoxToModel.column1	= BoxToModel.base1;
		mRBoxToModel.column2	= BoxToModel.base2;
		mTBoxToModel			= BoxToModel.base3;
	}
	else
	{
		Cm::Matrix34 ModelToBox	= WorldB.getInverseRT();

		mRModelToBox.column0	= ModelToBox.base0;
		mRModelToBox.column1	= ModelToBox.base1;
		mRModelToBox.column2	= ModelToBox.base2;
		mTModelToBox			= ModelToBox.base3;

		mRBoxToModel.column0	= WorldB.base0;
		mRBoxToModel.column1	= WorldB.base1;
		mRBoxToModel.column2	= WorldB.base2;
		mTBoxToModel			= WorldB.base3;
	}

	return Ps::IntFalse;
}

void OBBCollider::InitTraversal()
{
	// Now we can precompute box-box data

	// Precompute absolute box-to-model rotation matrix
	for(PxU32 i=0;i<3;i++)
	{
		// Epsilon value prevents floating-point inaccuracies (strategy borrowed from RAPID)
		mAR[i][0] = 1e-6f + PxAbs(mRBoxToModel[i][0]);
		mAR[i][1] = 1e-6f + PxAbs(mRBoxToModel[i][1]);
		mAR[i][2] = 1e-6f + PxAbs(mRBoxToModel[i][2]);
	}

	// Precompute box-box data - Courtesy of Erwin de Vries
	mBBx1 = mBoxExtents.x*mAR[0][0] + mBoxExtents.y*mAR[1][0] + mBoxExtents.z*mAR[2][0];
	mBBy1 = mBoxExtents.x*mAR[0][1] + mBoxExtents.y*mAR[1][1] + mBoxExtents.z*mAR[2][1];
	mBBz1 = mBoxExtents.x*mAR[0][2] + mBoxExtents.y*mAR[1][2] + mBoxExtents.z*mAR[2][2];

	mBB_1 = mBoxExtents.y*mAR[2][0] + mBoxExtents.z*mAR[1][0];
	mBB_2 = mBoxExtents.x*mAR[2][0] + mBoxExtents.z*mAR[0][0];
	mBB_3 = mBoxExtents.x*mAR[1][0] + mBoxExtents.y*mAR[0][0];
	mBB_4 = mBoxExtents.y*mAR[2][1] + mBoxExtents.z*mAR[1][1];
	mBB_5 = mBoxExtents.x*mAR[2][1] + mBoxExtents.z*mAR[0][1];
	mBB_6 = mBoxExtents.x*mAR[1][1] + mBoxExtents.y*mAR[0][1];
	mBB_7 = mBoxExtents.y*mAR[2][2] + mBoxExtents.z*mAR[1][2];
	mBB_8 = mBoxExtents.x*mAR[2][2] + mBoxExtents.z*mAR[0][2];
	mBB_9 = mBoxExtents.x*mAR[1][2] + mBoxExtents.y*mAR[0][2];

	// Precompute bounds for box-in-box test
	mB0 = mBoxExtents - mTModelToBox;
	mB1 = - mBoxExtents - mTModelToBox;

#ifdef OPC_SUPPORT_VMX128

	mBBxyz1.x = mBBx1; mBBxyz1.y = mBBy1; mBBxyz1.z = mBBz1;

	mBB_123.x = mBB_1; mBB_123.y = mBB_2; mBB_123.z = mBB_3;
	mBB_456.x = mBB_4; mBB_456.y = mBB_5; mBB_456.z = mBB_6;
	mBB_789.x = mBB_7; mBB_789.y = mBB_8; mBB_789.z = mBB_9;

#endif
}



//#define RTREE_PROFILE_ENABLE
#ifdef RTREE_PROFILE_ENABLE
#include "PsTime.h" // scaffold - temp for perf measurements
#endif

template <int DoPrimTests, int ReportVerts>
struct OBBRTreeCallback : Gu::RTree::Callback
{
	HybridOBBCollider*			parent;
	const HybridModelData*		model;
	VolumeColliderTrigCallback*	filteredCallback;
	PxU32						totalResults;

	OBBRTreeCallback(HybridOBBCollider* parent, const HybridModelData* model, VolumeColliderTrigCallback* fc)
		: parent(parent), model(model), filteredCallback(fc), totalResults(0)
	{
	}

	virtual bool processResults(PxU32 Nb, PxU32* Touched)
	{
		totalResults += Nb;
		const PxU32* PX_RESTRICT Indices = model->GetIndices();
		PX_FORCE_PARAMETER_REFERENCE(Indices);

		PxMemFetchSmallBuffer buf0, buf1, buf2;
		PxU32* has16BitIndices = pxMemFetchAsync<PxU32>(PxMemFetchPtr(parent->mIMesh)+OFFSET_OF(MeshInterface, mHas16BitIndices), 5, buf0);
		PxMemFetchPtr* mTris = pxMemFetchAsync<PxMemFetchPtr>(PxMemFetchPtr(parent->mIMesh)+OFFSET_OF(MeshInterface, mTris), 5, buf1);
		PxMemFetchPtr* mVerts = pxMemFetchAsync<PxMemFetchPtr>(PxMemFetchPtr(parent->mIMesh)+OFFSET_OF(MeshInterface, mVerts), 5, buf2);
		pxMemFetchWait(5);

		// new buffer for trig results, can be bigger than touched leaves so we can't reuse Touched in place
		const PxU32 trigBufferSize = 96;
		PxU32 indexBuffer[trigBufferSize];
		PxVec3 trigBuffer[trigBufferSize*3];
		PxU32 numTrigsHit = 0;
		while(Nb--)
		{
			PxU32 leafData = *Touched++;

			// Each leaf box has a set of triangles
			PxU32 NbTris = (leafData & 15)+1;
			PxU32 BaseIndex = leafData >> 4;

			if (!DoPrimTests && !ReportVerts)
			{
				if (numTrigsHit + NbTris >= trigBufferSize)
				{
					// flush the buffer to make sure we have space for Duff's device
					filteredCallback->processResults(numTrigsHit, trigBuffer, indexBuffer);
					numTrigsHit = 0;
				}
				const int NBITS = 4;
				const int N = 16;
				PX_ASSERT(NbTris < 64 && NbTris < trigBufferSize);
			    int nExtraBlocks = NbTris >> NBITS;
				PxU32* to = indexBuffer+numTrigsHit;
				char entry = NbTris & (N-1);
				to += entry;
				switch (entry)
				{
					case 16: do{ to[-16] = BaseIndex+15;
					case 15:     to[-15] = BaseIndex+14;
					case 14:     to[-14] = BaseIndex+13;
					case 13:     to[-13] = BaseIndex+12;
					case 12:     to[-12] = BaseIndex+11;
					case 11:     to[-11] = BaseIndex+10;
					case 10:     to[-10] = BaseIndex+9;
					case 9:      to[-9]  = BaseIndex+8;
					case 8:      to[-8]  = BaseIndex+7;
					case 7:      to[-7]  = BaseIndex+6;
					case 6:      to[-6]  = BaseIndex+5;
					case 5:      to[-5]  = BaseIndex+4;
					case 4:      to[-4]  = BaseIndex+3;
					case 3:      to[-3]  = BaseIndex+2;
					case 2:      to[-2]  = BaseIndex+1;
					case 1:      to[-1]  = BaseIndex+0;
					case 0:
								to += N;  BaseIndex += N;
						} while (--nExtraBlocks>=0);
				}
				numTrigsHit += NbTris;
			} else
			{
				#if defined(__SPU__)
				const PxU32 N = 8;
				for(PxU32 iTri = 0; iTri < NbTris; iTri+=N)
				{
					PX_ASSERT(!Indices);
					PxVec3 v[N][3];
					PxU32 countLeft = iTri+N > NbTris ? NbTris-iTri : N;
					if (ReportVerts || DoPrimTests)
						MeshInterface::getTriangleVertsN<N>(*has16BitIndices, *mTris, *mVerts, BaseIndex+iTri, countLeft, v);
					for(PxU32 jj = 0; jj < countLeft; jj++)
					{
						const PxVec3& v0 = v[jj][0];
						const PxVec3& v1 = v[jj][1];
						const PxVec3& v2 = v[jj][2];
						if(!DoPrimTests || parent->TriBoxOverlap(v0, v1, v2))
						{
							if (ReportVerts)
							{
								trigBuffer[numTrigsHit*3+0] = v0;
								trigBuffer[numTrigsHit*3+1] = v1;
								trigBuffer[numTrigsHit*3+2] = v2;
							}
							indexBuffer[numTrigsHit++] = BaseIndex+iTri+jj;
							if (numTrigsHit == trigBufferSize)
							{
								// flush partial result, reset trig count
								filteredCallback->processResults(numTrigsHit, trigBuffer, indexBuffer);
								numTrigsHit = 0;
							}
						}
					}
				}
				#else
				for(PxU32 iTri = 0; iTri < NbTris; iTri++)
				{
					PX_ASSERT(!Indices);
					PxU32 TriangleIndex = BaseIndex++;
					PxVec3 v0, v1, v2;
					if (ReportVerts || DoPrimTests)
						MeshInterface::GetTriangleVerts(*has16BitIndices, *mTris, *mVerts, TriangleIndex, v0, v1, v2);
					if(!DoPrimTests || parent->TriBoxOverlap(v0, v1, v2))
					{
						if (ReportVerts)
						{
							trigBuffer[numTrigsHit*3+0] = v0;
							trigBuffer[numTrigsHit*3+1] = v1;
							trigBuffer[numTrigsHit*3+2] = v2;
						}
						indexBuffer[numTrigsHit++] = TriangleIndex;
						if (numTrigsHit == trigBufferSize)
						{
							// flush partial result, reset trig count
							filteredCallback->processResults(numTrigsHit, trigBuffer, indexBuffer);
							numTrigsHit = 0;
						}
					}
				}
				#endif
			}
		}

		if (!numTrigsHit)
			return true;

		// flush any outstanding results
		return filteredCallback->processResults(numTrigsHit, trigBuffer, indexBuffer);

	} // processResults
};

void HybridOBBCollider::Collide(
	const Gu::Box& box, const HybridModelData& model, VolumeColliderTrigCallback* parentCallback,
	const Cm::Matrix34* worldb, const Cm::Matrix34* worldm, bool reportVerts)
{
	const Ps::IntBool NoPrimitiveTests = mFlags & OPC_NO_PRIMITIVE_TESTS;

	// Checkings
	if(!Setup(&model))
		return;

	// Init collision query
	//Note does not do precompute for box-box(done in InitTraversal())
	if(InitQuery(box, worldb, worldm))
		return;

	//Perform additional setup here(avoids overhead when we can early out).
	InitTraversal();

#if 1
	OBBRTreeCallback<1, 1> callbackTestsVerts(this, &model, parentCallback);
	OBBRTreeCallback<0, 0> callbackNoTestsNoVerts(this, &model, parentCallback);
	OBBRTreeCallback<1, 0> callbackTestsNoVerts(this, &model, parentCallback);
	OBBRTreeCallback<0, 1> callbackNoTestsVerts(this, &model, parentCallback);
	Gu::RTree::Callback* callback;
	if (NoPrimitiveTests && !reportVerts)
		callback = &callbackNoTestsNoVerts;
	else if (NoPrimitiveTests && reportVerts)
		callback = &callbackNoTestsVerts;
	else if (reportVerts)
		callback = &callbackTestsVerts;
	else
		callback = &callbackTestsNoVerts;
#else
	OBBRTreeCallback<0, 1> callbackNoTestsVerts(this, &model, parentCallback);
	Gu::RTree::Callback* callback = &callbackNoTestsVerts;
#endif

	Gu::Box meshSpaceBox;
	if (worldm)
	{
		Cm::Matrix34 invWorldM = worldm->getInverseRT();
		meshSpaceBox.rot.column0 = invWorldM.rotate(box.rot.column0);
		meshSpaceBox.rot.column1 = invWorldM.rotate(box.rot.column1);
		meshSpaceBox.rot.column2 = invWorldM.rotate(box.rot.column2);
		meshSpaceBox.center = invWorldM.transform(box.center);
		meshSpaceBox.extents = box.extents;
	} else
		meshSpaceBox = box;

	#ifdef RTREE_PROFILE_ENABLE
			// scaffold
			Ps::Time timer;
			static PxU64 totalUSec = 0;
			static PxU32 queryCount = 0;
			PxU64 clk = timer.getCurrentCounterValue();
			queryCount++;
	#endif

	const PxU32 bufSize = 32;
	PxU32 buf[bufSize];
	model.mRTree->traverseOBB(meshSpaceBox, bufSize, buf, callback);

	#ifdef RTREE_PROFILE_ENABLE
			static PxU32 sum;
			sum += PxMax(callbackTestsVerts.totalResults, callbackNoTestsNoVerts.totalResults);
			// scaffold
			clk = timer.getCurrentCounterValue() - clk;
			const PxU32 limit = 0x10000;
			if (queryCount >= 5 && queryCount < limit)
				totalUSec += clk;
			else if (queryCount == limit)
			{
				fprintf(stderr, "AVG CYC=%.4f US=%.4f COUNT=%d\n",
					PxF32(totalUSec)/PxF32(limit-5), PxF32(timer.sCounterFreq.toTensOfNanos(clk)*100)/PxF32(limit-5), sum);
				fflush(stderr);
			}
	#endif
}

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
#include "OPC_LSSCollider.h"

using namespace physx;
using namespace Ice;

void LSSCollider::InitQuery(const Gu::Capsule& lss, const Cm::Matrix34* worldl, const Cm::Matrix34* worldm)
{
	VolumeCollider::InitQuery();

	// Compute LSS in model space:
	// - Precompute R^2
	mRadius = lss.radius;
	mRadius2 = lss.radius * lss.radius;
	// - Compute segment
	mSeg.p0 = lss.p0;
	mSeg.p1 = lss.p1;
	// -> to world space
	if(worldl)
	{
		mSeg.p0 = worldl->transform(mSeg.p0);
		mSeg.p1 = worldl->transform(mSeg.p1);
	}
	// -> to model space
	if(worldm)
	{
		// Invert model matrix
		Cm::Matrix34 InvWorldM = worldm->getInverseRT();

		mSeg.p0 = InvWorldM.transform(mSeg.p0);
		mSeg.p1 = InvWorldM.transform(mSeg.p1);
	}

	// Precompute segment data
	mSDir = 0.5f * (mSeg.p1 - mSeg.p0);
	mFDir.x = PxAbs(mSDir.x);
	mFDir.y = PxAbs(mSDir.y);
	mFDir.z = PxAbs(mSDir.z);
	mSCen = 0.5f * (mSeg.p1 + mSeg.p0);

	mOBB.create(Gu::Capsule(mSeg, mRadius));	// Create box around transformed capsule
}


bool HybridLSSCollider::processLeafTriangles(PxU32 count, const PxU32* PX_RESTRICT buf)
{
	const Ps::IntBool NoPrimitiveTests = mFlags & OPC_NO_PRIMITIVE_TESTS;
	const Ps::IntBool LoosePrimitiveTests = mFlags & OPC_LOOSE_PRIMITIVE_TESTS;

	const HybridModelData* modelData = static_cast<const HybridModelData*>(mCurrentModel);

	const LeafTriangles* PX_RESTRICT LT = modelData->GetLeafTriangles();
	const PxU32* PX_RESTRICT Indices = modelData->GetIndices();

	while(count--)
	{
		const PxU32 leafData = *buf++;

		// Each leaf box has a set of triangles
		PxU32 NbTris = (leafData & 15)+1;
		PxU32 BaseIndex = leafData >> 4;

		// Each leaf box has a set of triangles
		if(Indices)
		{
			const PxU32* PX_RESTRICT T = &Indices[BaseIndex];

			// Loop through triangles and test each of them
			if(!NoPrimitiveTests)
			{
				if(LoosePrimitiveTests)
				{
					while(NbTris--)
						if(!PerformLooseLSSPrimOverlapTest(*T++, OPC_CONTACT))
							return true;
				}
				else
				{
					while(NbTris--)
						if(!PerformLSSPrimOverlapTest(*T++, OPC_CONTACT))
							return true;
				}
			}
			else
			{
				while(NbTris--)
					if(!setContact(*T++, OPC_CONTACT))
						return true;
			}
		}
		else
		{
			// Loop through triangles and test each of them
			if(!NoPrimitiveTests)
			{
				if(LoosePrimitiveTests)
				{
					while(NbTris--)
						if(!PerformLooseLSSPrimOverlapTest(BaseIndex++, OPC_CONTACT))
							return true;
				}
				else
				{
					while(NbTris--)
						if(!PerformLSSPrimOverlapTest(BaseIndex++, OPC_CONTACT))
							return true;
				}
			}
			else
			{
				while(NbTris--)
					if(!setContact(BaseIndex++, OPC_CONTACT))
						return true;
			}
		}
	}
	return true;
}

bool HybridLSSCollider::Collide(ReportCapsuleCallback cb, void* userData,
								const Gu::Capsule& lss, const HybridModelData& model,
								const Cm::Matrix34* worldl, const Cm::Matrix34* worldm)
{
	if(!Setup(&model))
		return false;

	InitQuery(lss, worldl, worldm);

	mCallback = cb;
	mUserData = userData;

	struct ProcessingCallback : Gu::RTree::Callback
	{
		HybridLSSCollider& collider;
		ProcessingCallback(HybridLSSCollider& collider) : collider(collider)
		{
		}
		virtual ~ProcessingCallback() {}

		virtual bool processResults(PxU32 count, PxU32* buf)
		{
			return collider.processLeafTriangles(count, buf);
		}
	} callback(*this);

	const PxU32 bufSize = 32;
	PxU32 buf[bufSize];
	model.mRTree->traverseRay<1, 1>(mSeg.p0, mSeg.p1-mSeg.p0, bufSize, buf, &callback, PxVec3(mRadius));
	return true;
}

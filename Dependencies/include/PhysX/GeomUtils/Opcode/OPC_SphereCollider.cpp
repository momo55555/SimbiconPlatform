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
#include "GuSphere.h"
#include "OPC_SphereCollider.h"

using namespace physx;
using namespace Ice;

SphereCollider::SphereCollider()
{
	mCenter = PxVec3(0);
	mRadius2 = 0.0f;
	mRadius = 0.0f;
}

void SphereCollider::InitQuery(const Gu::Sphere& sphere, const Cm::Matrix34* worlds, const Cm::Matrix34* worldm)
{
	VolumeCollider::InitQuery();

	// Compute sphere in model space:
	// - Precompute R^2
	mRadius2 = sphere.radius * sphere.radius;
	mRadius = sphere.radius;
	// - Compute center position
	mCenter = sphere.center;
	// -> to world space
	if(worlds)
	{
		mCenter = worlds->transform(mCenter);
	}
	// -> to model space
	if(worldm)
	{
		// Invert model matrix
		Cm::Matrix34 InvWorldM = worldm->getInverseRT();

		mCenter = InvWorldM.transform(mCenter);
	}
}

#ifndef OPC_SUPPORT_VMX128

#define LOOSE_SPHERE_PRIM_SETUP
#define LOOSE_SPHERE_PRIM_FAST(prim_index, flag) PerformLooseSpherePrimOverlapTest(prim_index, flag);

#else

#define LOOSE_SPHERE_PRIM_SETUP													\
	Cm::PxSimd::Vector4 LSP_center = Cm::PxSimd::load(mCenter);						\
	Cm::PxSimd::Vector4 LSP_radius = Cm::PxSimd::splatX(Cm::PxSimd::load(mRadius));		\
	Cm::PxSimd::Vector4 LSP_sphereMin = Cm::PxSimd::subtract(LSP_center, LSP_radius);	\
	Cm::PxSimd::Vector4 LSP_sphereMax = Cm::PxSimd::add(LSP_center, LSP_radius);		\
	Cm::PxSimd::Vector4 LSP_zero = Cm::PxSimd::zero();

//! OBB-triangle test
#define LOOSE_SPHERE_PRIM_FAST(prim_index, flag)								\
	/* Request vertices from the app */											\
	VertexPointers VP;	mIMesh->GetTriangle(VP, prim_index);					\
	/* Perform triangle-sphere overlap test */									\
	if(LooseSphereTriOverlap(*VP.vertex[0], *VP.vertex[1], *VP.vertex[2],		\
		LSP_center, LSP_radius, LSP_sphereMin, LSP_sphereMax, LSP_zero))		\
	{																			\
		setContact(prim_index, flag);											\
	}

#endif

bool HybridSphereCollider::processLeafTriangles(PxU32 count, const PxU32* PX_RESTRICT buf)
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
					LOOSE_SPHERE_PRIM_SETUP

					while(NbTris--)
					{
						const PxU32 TriangleIndex = *T++;
						LOOSE_SPHERE_PRIM_FAST(TriangleIndex, OPC_CONTACT)
					}
				}
				else
				{
					while(NbTris--)
						if(!PerformSpherePrimOverlapTest(*T++, OPC_CONTACT))
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
					LOOSE_SPHERE_PRIM_SETUP

					while(NbTris--)
					{
						const PxU32 TriangleIndex = BaseIndex++;
						LOOSE_SPHERE_PRIM_FAST(TriangleIndex, OPC_CONTACT)
					}
				}
				else
				{
					while(NbTris--)
						if(!PerformSpherePrimOverlapTest(BaseIndex++, OPC_CONTACT))
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


bool HybridSphereCollider::Collide(	ReportSphereCallback cb, void* userData,
									const Gu::Sphere& sphere, const HybridModelData& model,
									const Cm::Matrix34* worlds, const Cm::Matrix34* worldm)
{
	if(!Setup(&model))
		return false;

	InitQuery(sphere, worlds, worldm);

	mCallback = cb;
	mUserData = userData;

	struct ProcessingCallback : Gu::RTree::Callback
	{
		HybridSphereCollider& collider;
		ProcessingCallback(HybridSphereCollider& collider) : collider(collider)
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
	model.mRTree->traverseAABB(mCenter-PxVec3(mRadius), mCenter+PxVec3(mRadius), bufSize, buf, &callback);
	return true;
}

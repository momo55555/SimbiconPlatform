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
// Include Guard
#ifndef OPC_SPHERECOLLIDER_H
#define OPC_SPHERECOLLIDER_H

#include "CmSimd.h"
#include "OPC_VolumeCollider.h"
#include "OPC_ModelData.h"
#include "OPC_MeshInterface.h"

namespace physx
{

namespace Gu
{
	class Sphere;
}

namespace Ice
{
	class SphereCollider : public VolumeCollider
	{
		public:
											SphereCollider();
		PX_FORCE_INLINE						~SphereCollider()	{}

		protected:
		// Sphere in model space
							PxVec3			mCenter;			//!< Sphere center
							float			mRadius2;			//!< Sphere radius squared
							float			mRadius;			//!< Sphere radius

			// Overlap tests
#ifdef OPC_SUPPORT_VMX128
		PX_FORCE_INLINE			Ps::IntBool		SphereAABBOverlap(const Cm::PxSimd::Vector4& center, const Cm::PxSimd::Vector4& extents);

		PX_FORCE_INLINE			Ps::IntBool		LooseSphereTriOverlap(const PxVec3& vert0, const PxVec3& vert1, const PxVec3& vert2,
																	const Cm::PxSimd::Vector4 &center, const Cm::PxSimd::Vector4 &radius,
																	const Cm::PxSimd::Vector4 &sphereMin, const Cm::PxSimd::Vector4 &sphereMax,
																	const Cm::PxSimd::Vector4 &zero);
#endif
		PX_FORCE_INLINE			Ps::IntBool		SphereAABBOverlap(const PxVec3& center, const PxVec3& extents);
		PX_FORCE_INLINE			Ps::IntBool		SphereTriOverlap(const PxVec3& vert0, const PxVec3& vert1, const PxVec3& vert2);
		PX_FORCE_INLINE			Ps::IntBool		LooseSphereTriOverlap(const PxVec3& vert0, const PxVec3& vert1, const PxVec3& vert2);

								void			InitQuery(
									const Gu::Sphere& sphere,
									const Cm::Matrix34* worlds=NULL, const Cm::Matrix34* worldm=NULL);
	};

	typedef bool	(*ReportSphereCallback)	(PxU32 primIndex, void* userData);

	class HybridSphereCollider : public SphereCollider
	{
	public:
												HybridSphereCollider() : mCallback(NULL), mUserData(NULL)	{}
		PX_FORCE_INLINE							~HybridSphereCollider()										{}

						bool					Collide(ReportSphereCallback cb, void* userData, const Gu::Sphere& sphere, const HybridModelData& model, const Cm::Matrix34* worlds=NULL, const Cm::Matrix34* worldm=NULL);
	protected:

		// Sphere-triangle overlap test
		PX_FORCE_INLINE	bool					PerformSpherePrimOverlapTest(PxU32 primIndex, PxU32 flag)
												{
													// Request vertices from the app
													VertexPointers VP;
													mIMesh->GetTriangle(VP, primIndex);

													// Perform sphere-tri overlap test
													if(SphereTriOverlap(*VP.vertex[0], *VP.vertex[1], *VP.vertex[2]))
														return setContact(primIndex, flag);
													return true;
												}

		// Approximate sphere-triangle overlap test
		PX_FORCE_INLINE	bool					PerformLooseSpherePrimOverlapTest(PxU32 primIndex, PxU32 flag)
												{
													// Request vertices from the app
													VertexPointers VP;
													mIMesh->GetTriangle(VP, primIndex);

													// Perform sphere-tri overlap test
													if(LooseSphereTriOverlap(*VP.vertex[0], *VP.vertex[1], *VP.vertex[2]))
														return setContact(primIndex, flag);
													return true;
												}

		PX_FORCE_INLINE	bool					setContact(PxU32 primIndex, PxU32 flag)
												{
													mFlags |= flag;	// Set contact status
													return (mCallback)(primIndex, mUserData);
												}
						bool					processLeafTriangles(PxU32 count, const PxU32* PX_RESTRICT buf);

						ReportSphereCallback	mCallback;
						void*					mUserData;
	};

} // namespace Ice

}

#include "OPC_SphereAABBOverlap.h"
#include "OPC_SphereTriOverlap.h"

#endif // OPC_SPHERECOLLIDER_H

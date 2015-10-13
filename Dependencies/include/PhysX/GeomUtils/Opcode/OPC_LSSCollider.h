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
#ifndef OPC_LSSCOLLIDER_H
#define OPC_LSSCOLLIDER_H

#include "CmSimd.h"
#include "GuCapsule.h"
#include "OPC_VolumeCollider.h"
#include "OPC_ModelData.h"
#include "OPC_MeshInterface.h"

namespace physx
{
namespace Ice
{
	class LSSCollider : public VolumeCollider
	{
		public:
		PX_FORCE_INLINE						LSSCollider()		{}
		PX_FORCE_INLINE						~LSSCollider()		{}

		protected:
		// LSS in model space
							Gu::Segment		mSeg;			//!< Segment
							PxVec3			mSDir;			//!< Precomputed segment data
							PxVec3			mFDir;			//!< Precomputed segment data
							PxVec3			mSCen;			//!< Precomputed segment data
							float			mRadius;		//!< LSS radius
							float			mRadius2;		//!< LSS radius squared

							Gu::Box			mOBB;

			// Overlap tests
#ifdef OPC_SUPPORT_VMX128
		PX_FORCE_INLINE			Ps::IntBool		LSSAABBOverlap(const Cm::PxSimd::Vector4& center, const Cm::PxSimd::Vector4& extents);
#endif
		PX_FORCE_INLINE			Ps::IntBool		LSSAABBOverlap(const PxVec3& center, const PxVec3& extents);
		PX_FORCE_INLINE			Ps::IntBool		LSSTriOverlap(const PxVec3& vert0, const PxVec3& vert1, const PxVec3& vert2);
		PX_FORCE_INLINE			Ps::IntBool		LooseLSSTriOverlap(const PxVec3& vert0, const PxVec3& vert1, const PxVec3& vert2);

								void			InitQuery(const Gu::Capsule& lss, const Cm::Matrix34* worldl=NULL, const Cm::Matrix34* worldm=NULL);
	};

	typedef bool	(*ReportCapsuleCallback)	(PxU32 primIndex, void* userData);

	class HybridLSSCollider : public LSSCollider
	{
		public:
		PX_FORCE_INLINE							HybridLSSCollider() : mCallback(NULL), mUserData(NULL)	{}
		PX_FORCE_INLINE							~HybridLSSCollider()									{}	

						bool					Collide(ReportCapsuleCallback cb, void* userData, const Gu::Capsule& lss, const HybridModelData& model, const Cm::Matrix34* worldl=NULL, const Cm::Matrix34* worldm=NULL);
		protected:
		// LSS-triangle overlap test
		PX_FORCE_INLINE	bool					PerformLSSPrimOverlapTest(PxU32 primIndex, PxU32 flag)
												{
													// Request vertices from the app
													VertexPointers VP;
													mIMesh->GetTriangle(VP, primIndex);

													// Perform LSS-tri overlap test
													if(LSSTriOverlap(*VP.vertex[0], *VP.vertex[1], *VP.vertex[2]))
														return setContact(primIndex, flag);
													return true;
												}

		// Approximate LSS-triangle overlap test
		PX_FORCE_INLINE	bool					PerformLooseLSSPrimOverlapTest(PxU32 primIndex, PxU32 flag)
												{
													// Request vertices from the app
													VertexPointers VP;
													mIMesh->GetTriangle(VP, primIndex);

													// Perform LSS-tri overlap test
													if(LooseLSSTriOverlap(*VP.vertex[0], *VP.vertex[1], *VP.vertex[2]))
														return setContact(primIndex, flag);
													return true;
												}

		PX_FORCE_INLINE	bool					setContact(PxU32 primIndex, PxU32 flag)
												{
													mFlags |= flag;	// Set contact status
													return (mCallback)(primIndex, mUserData);
												}
						bool					processLeafTriangles(PxU32 count, const PxU32* PX_RESTRICT buf);

						ReportCapsuleCallback	mCallback;
						void*					mUserData;
	};
} // namespace Ice

}

#include "OPC_LSSAABBOverlap.h"
#include "OPC_LSSTriOverlap.h"

#endif // OPC_LSSCOLLIDER_H

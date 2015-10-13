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
#ifndef OPC_OBBCOLLIDER_H
#define OPC_OBBCOLLIDER_H

#include "OPC_VolumeCollider.h"
#include "OPC_MeshInterface.h"

namespace physx
{
namespace Ice
{
	class OBBCollider : public VolumeCollider
	{
		public:
		PX_FORCE_INLINE						OBBCollider() : mFullBoxBoxTest(true)	{}
		PX_FORCE_INLINE						~OBBCollider()							{}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Settings: select between full box-box tests or "SAT-lite" tests (where Class III axes are discarded)
		 *	\param		flag		[in] true for full tests, false for coarse tests
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		PX_FORCE_INLINE				void			SetFullBoxBoxTest(bool flag)	{ mFullBoxBoxTest = flag;	}

		protected:
		// Precomputed data
							PxMat33		mAR;				//!< Absolute rotation matrix
							PxMat33		mRModelToBox;		//!< Rotation from model space to obb space
							PxMat33		mRBoxToModel;		//!< Rotation from obb space to model space
							PxVec3			mTModelToBox;		//!< Translation from model space to obb space
							PxVec3			mTBoxToModel;		//!< Translation from obb space to model space

							PxVec3			mBoxExtents;
							PxVec3			mB0;				//!< - mTModelToBox + mBoxExtents
							PxVec3			mB1;				//!< - mTModelToBox - mBoxExtents

							float			mBBx1;
							float			mBBy1;
							float			mBBz1;

							float			mBB_1;
							float			mBB_2;
							float			mBB_3;
							float			mBB_4;
							float			mBB_5;
							float			mBB_6;
							float			mBB_7;
							float			mBB_8;
							float			mBB_9;

#ifdef OPC_SUPPORT_VMX128
							Cm::PxSimd::Vector4 mBBxyz1;

							Cm::PxSimd::Vector4 mBB_123;
							Cm::PxSimd::Vector4 mBB_456;
							Cm::PxSimd::Vector4 mBB_789;
#endif

		// Leaf description
							PxVec3			mLeafVerts[3];		//!< Triangle vertices
		// Settings
							bool			mFullBoxBoxTest;	//!< Perform full BV-BV tests (true) or SAT-lite tests (false)

	public:
			// Overlap tests
#ifdef OPC_SUPPORT_VMX128
		PX_FORCE_INLINE			Ps::IntBool		BoxBoxOverlap(const Cm::PxSimd::Vector4& extents, const Cm::PxSimd::Vector4& center);

		PX_FORCE_INLINE			Ps::IntBool		BoxBoxOverlap(const Cm::PxSimd::Vector4& extents, const Cm::PxSimd::Vector4& center,
														  const Cm::PxSimd::Vector4 &TBoxToModel, const Cm::PxSimd::Vector4 &BB,
														  const Cm::PxSimd::Vector4 &rBoxToModel_0, const Cm::PxSimd::Vector4 &rBoxToModel_1, const Cm::PxSimd::Vector4 &rBoxToModel_2,
														  const Cm::PxSimd::Vector4 &ar_0, const Cm::PxSimd::Vector4 &ar_1, const Cm::PxSimd::Vector4 &ar_2,
														  const Cm::PxSimd::Vector4 &thisExtents,
														  const Cm::PxSimd::Vector4 &BB_123,	const Cm::PxSimd::Vector4 &BB_456, const Cm::PxSimd::Vector4 &BB_789);

		PX_FORCE_INLINE			Ps::IntBool		TriBoxOverlap(const PxVec3& vert0, const PxVec3& vert1, const PxVec3& vert2,
														  const Cm::PxSimd::Vector4 &rot_0, const Cm::PxSimd::Vector4 &rot_1, const Cm::PxSimd::Vector4 &rot2, 
														  const Cm::PxSimd::Vector4 &tran, const Cm::PxSimd::Vector4 &extents);
		PX_FORCE_INLINE			Ps::IntBool		OBBCollider::TriBoxOverlap(const PxVec3& vert0, const PxVec3& vert1, const PxVec3& vert2,
														  const Cm::PxSimd::Vector4 &tran, const Cm::PxSimd::Vector4 &extents);
#endif
		PX_FORCE_INLINE			Ps::IntBool		BoxBoxOverlap(const PxVec3& extents, const PxVec3& center);
		PX_FORCE_INLINE			Ps::IntBool		TriBoxOverlap(const PxVec3& vert0, const PxVec3& vert1, const PxVec3& vert2);

							Ps::IntBool		InitQuery(const Gu::Box& box, const Cm::Matrix34* worldb=NULL, const Cm::Matrix34* worldm=NULL);
							//Perform additional setup for obb-obb tests
							void			InitTraversal();
	};

	class HybridOBBCollider : public OBBCollider
	{
	public:
		void Collide(
			const Gu::Box& box, const HybridModelData& model, VolumeColliderTrigCallback* callback,
			const Cm::Matrix34* worldb=NULL, const Cm::Matrix34* worldm=NULL, bool reportVerts = true);
	};

} // namespace Ice

}

#include "OPC_OBBColliderOverlap.h"

#endif // OPC_OBBCOLLIDER_H

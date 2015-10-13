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

#ifndef PX_CHARACTER_CONTROLLER
#define PX_CHARACTER_CONTROLLER

//#define USE_CONTACT_NORMAL_FOR_SLOPE_TEST

#include "PxController.h"
#include "CctUtils.h"
#include "GuTriangle.h"
#include "PsArray.h"
#include "CmPhysXCommon.h"

namespace physx
{

struct PxFilterData;
class PxSceneQueryFilterCallback;

namespace Cm
{
	class RenderBuffer;
}

namespace Cct
{
	struct CCTParams
	{
		PxCCTUpAxis::Enum	mUpDirection;
		PxF32				mSlopeLimit;
		PxF32				mContactOffset;
		PxF32				mStepOffset;
		PxF32				mInvisibleWallHeight;
		PxF32				mMaxJumpHeight;
		bool				mHandleSlope;		// True to handle walkable parts according to slope
	};

	template<class T, class A>
	PX_INLINE T* reserve(Ps::Array<T, A>& array, PxU32 nb)
	{
		PxU32 currentSize = array.size();
		array.resize(array.size() + nb, T());
		return array.begin() + currentSize;
	}

	// Sigh. The function above doesn't work with typedefs apparently
	typedef Ps::Array<Gu::Triangle>	TriArray;
	typedef Ps::Array<PxU32>		IntArray;

	/* Exclude from documentation */
	/** \cond */

	struct TouchedGeomType
	{
		enum Enum
		{
			eUSER_BOX,
			eUSER_CAPSULE,
			eMESH,
			eBOX,
			eSPHERE,
			eCAPSULE,

			eLAST,

			eFORCE_DWORD	= 0x7fffffff
		};
	};

	class SweptVolume;

// PT: apparently stupid .Net aligns some of them on 8-bytes boundaries for no good reason. This is bad.
#pragma pack(push,4)

	struct TouchedGeom
	{
		TouchedGeomType::Enum	mType;
		const void*				mUserData;	// PxController or PxShape PxVec3er
		PxExtendedVec3			mOffset;	// Local origin, typically the center of the world bounds around the character. We translate both
											// touched shapes & the character so that they are nearby this PxVec3, then add the offset back to
											// computed "world" impacts.
	};

	struct TouchedUserBox : public TouchedGeom
	{
		PxExtendedBounds3		mBox;
	};
	PX_COMPILE_TIME_ASSERT(sizeof(TouchedUserBox)==sizeof(TouchedGeom)+sizeof(PxExtendedBounds3));

	struct TouchedUserCapsule : public TouchedGeom
	{
		PxExtendedCapsule		mCapsule;
	};
	PX_COMPILE_TIME_ASSERT(sizeof(TouchedUserCapsule)==sizeof(TouchedGeom)+sizeof(PxExtendedCapsule));

	struct TouchedMesh : public TouchedGeom
	{
		PxU32			mNbTris;
		PxU32			mIndexWorldTriangles;
		PxU32			mIndexWorldEdgeNormals;
		PxU32			mIndexEdgeFlags;
	};

	struct TouchedBox : public TouchedGeom
	{
		PxVec3			mCenter;
		PxVec3			mExtents;
		PxMat33Legacy			mRot;
	};

	struct TouchedSphere : public TouchedGeom
	{
		PxVec3			mCenter;		//!< Sphere's center
		PxF32			mRadius;		//!< Sphere's radius
	};

	struct TouchedCapsule : public TouchedGeom
	{
		PxVec3			mP0;		//!< Start of segment
		PxVec3			mP1;		//!< End of segment
		PxF32			mRadius;	//!< Capsule's radius
	};

#pragma pack(pop)

	struct SweptContactType
	{
		enum Enum
		{
			E_SHAPE,		// We touched another shape
			E_CONTROLLER,	// We touched another controller
		};
	};

	struct SweptContact
	{
		PxExtendedVec3		mWorldPos;		// Contact position in world space
		PxVec3				mWorldNormal;	// Contact normal in world space
		PxF32				mDistance;		// Contact distance
		PxU32				mInternalIndex;	// Reserved for internal usage
		PxU32				mTriangleIndex;	// Triangle index for meshes/heightfields
		TouchedGeom*		mGeom;

		PX_FORCE_INLINE		void	setWorldPos(const PxVec3& localImpact, const PxExtendedVec3& offset)
		{
			mWorldPos.x = localImpact.x + offset.x;
			mWorldPos.y = localImpact.y + offset.y;
			mWorldPos.z = localImpact.z + offset.z;
		}
	};

	class SweepTest
	{
	public:
		SweepTest();
		~SweepTest();

		void				moveCharacter(
			void* user_data,
			void* user_data2,
			SweptVolume& volume,
			const PxVec3& direction,
			PxU32 nb_boxes, const PxExtendedBounds3* boxes, const void** box_user_data,
			PxU32 nb_capsules, const PxExtendedCapsule* capsules, const void** capsule_user_data,
			PxF32 min_dist,
			PxU32& collision_flags,
			const PxFilterData* filterData,
			PxSceneQueryFilterCallback* filterCallback,
			bool constrainedClimbingMode
			);

		bool				doSweepTest(
			void* user_data,
			void* user_data2,
			PxU32 nb_boxes, const PxExtendedBounds3* boxes, const void** box_user_data,
			PxU32 nb_capsules, const PxExtendedCapsule* capsules, const void** capsule_user_data,
			SweptVolume& swept_volume,
			const PxVec3& direction, PxU32 max_iter,
			PxU32* nb_collisions, PxF32 min_dist, const PxFilterData* filterData, PxSceneQueryFilterCallback* filterCallback, bool down_pass=false);

		void				findTouchedCCTs(
			PxU32 nb_boxes, const PxExtendedBounds3* boxes, const void** box_user_data,
			PxU32 nb_capsules, const PxExtendedCapsule* capsules, const void** capsule_user_data,
			const PxExtendedBounds3& world_box
			);

		void				voidTestCache()	{ mCachedTBV.setEmpty();	}

		// private:
		Cm::RenderBuffer*	mRenderBuffer;
		TriArray			mWorldTriangles;
		TriArray			mWorldEdgeNormals;
		IntArray			mEdgeFlags;
		IntArray			mTriangleIndices;
		IntArray			mGeomStream;
		PxExtendedBounds3	mCachedTBV;
		PxU32				mCachedTriIndexIndex;
		mutable	PxU32		mCachedTriIndex[3];
		PxU32				mNbCachedStatic;
		PxU32				mNbCachedT;
		PxU32				mNbCachedEN;
		PxU32				mNbCachedF;
	public:
#ifdef USE_CONTACT_NORMAL_FOR_SLOPE_TEST
		PxVec3				mCN;
#else
		Gu::Triangle		mTouched;
#endif
		CCTParams			mUserParams;
		PxF32				mVolumeGrowth;	// Must be >1.0f and not too big
		PxF32				mContactPointHeight;	// UBI
		PxU32				mMaxIter;
		bool				mHitNonWalkable;
		bool				mWalkExperiment;
		bool				mValidTri;
		bool				mValidateCallback;
		bool				mNormalizeResponse;
		bool				mFirstUpdate;
	private:
		void				updateTouchedGeoms(void* user_data, const SweptVolume& swept_volume,
			PxU32 nb_boxes, const PxExtendedBounds3* boxes, const void** box_user_data,
			PxU32 nb_capsules, const PxExtendedCapsule* capsules, const void** capsule_user_data,
			const PxExtendedBounds3& world_box, const PxFilterData* filterData, PxSceneQueryFilterCallback* filterCallback);
	};

	void findTouchedGeometry(void* user_data,
		const PxExtendedBounds3& world_aabb,

		TriArray& world_triangles,
		TriArray* world_edge_normals,
		IntArray& edgeFlagsArray,
		IntArray& triIndicesArray,
		IntArray& geomStream,

		bool static_shapes,
		bool dynamic_shapes,
		const PxFilterData* filterData,
		PxSceneQueryFilterCallback* filterCallback,
		const CCTParams& params);

	void shapeHitCallback(void* user_data2, const SweptContact& contact, const PxVec3& dir, PxF32 length);
	void userHitCallback(void* user_data2, const SweptContact& contact, const PxVec3& dir, PxF32 length);

} // namespace Cct

}

/** \endcond */
#endif

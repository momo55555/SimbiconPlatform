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
#ifndef OPC_RAYCOLLIDER_H
#define OPC_RAYCOLLIDER_H

#include "CmSimd.h"
#include "./Ice/IceContainer.h"
#include "OPC_Collider.h"
#include "OPC_ModelData.h"
#include "OPC_MeshInterface.h"

namespace physx
{
namespace Ice
{
	struct HybridModelData;

	class CollisionFace
	{
		public:
		//! Constructor
		PX_FORCE_INLINE				CollisionFace()			{}
		//! Destructor
		PX_FORCE_INLINE				~CollisionFace()		{}

				PxU32		mFaceID;				//!< Index of touched face
				float		mDistance;				//!< Distance from collider to hitpoint
				float		mU, mV;					//!< Impact barycentric coordinates
	};

	class CollisionFaces : public Container
	{
		public:
		//! Constructor
										CollisionFaces()						{}
		//! Destructor
										~CollisionFaces()						{}

		PX_FORCE_INLINE	PxU32					GetNbFaces()					const	{ return GetNbEntries()>>2;											}
		PX_FORCE_INLINE	const CollisionFace*	GetFaces()						const	{ return (const CollisionFace*)GetEntries();						}

		PX_FORCE_INLINE	void					Reset()									{ Container::Reset();												}

		PX_FORCE_INLINE	void					AddFace(const CollisionFace& face)		{ Add((const PxU32*)&face, sizeof(CollisionFace)/sizeof(PxU32));	}
	};

#ifdef OPC_RAYHIT_CALLBACK
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	 *	User-callback, called by OPCODE to record a hit. You can use this to implement custom collision filters.
	 *	\param		hit			[in] current hit
	 *	\param		user_data	[in] user-defined data from SetCallback()
	 */
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	typedef void	(*HitCallback)	(const CollisionFace& hit, void* user_data);
#endif

	class RayCollider : public Collider
	{
		public:
											RayCollider();
		PX_FORCE_INLINE						~RayCollider()	{}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Reports all contacts (false) or first contact only (true)
		 *	\param		flag		[in] true for first contact, false for all contacts
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		PX_FORCE_INLINE				void		SetFirstContact(bool flag)
											{
												if(flag)	mFlags |= OPC_FIRST_CONTACT;
												else		mFlags &= ~OPC_FIRST_CONTACT;
											}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Checks a first contact has already been found.
		 *	\return		true if a first contact has been found and we can stop a query
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		PX_FORCE_INLINE				Ps::IntBool	ContactFound()				const	{ return (mFlags&OPC_CONTACT_FOUND)==OPC_CONTACT_FOUND;	}

#ifndef OPC_RAYHIT_CALLBACK
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Settings: enable or disable "closest hit" mode.
		 *	\param		flag		[in] true to report closest hit only
		 *	\see		SetCulling(bool flag)
		 *	\see		SetMaxDist(float max_dist)
		 *	\see		SetDestination(StabbedFaces* sf)
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		PX_FORCE_INLINE				void			SetClosestHit(bool flag)				{ mClosestHit	= flag;		}
#endif
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Settings: enable or disable backface culling.
		 *	\param		flag		[in] true to enable backface culling
		 *	\see		SetClosestHit(bool flag)
		 *	\see		SetMaxDist(float max_dist)
		 *	\see		SetDestination(StabbedFaces* sf)
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		PX_FORCE_INLINE				void			SetCulling(bool flag)					{ mCulling		= flag;		}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Settings: sets the higher distance bound.
		 *	\param		max_dist	[in] higher distance bound. Default = maximal value, for ray queries (else segment)
		 *	\see		SetClosestHit(bool flag)
		 *	\see		SetCulling(bool flag)
		 *	\see		SetDestination(StabbedFaces* sf)
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		PX_FORCE_INLINE				void			SetMaxDist(float max_dist=PX_MAX_F32)	{ mMaxDist		= max_dist;	}

#ifdef OPC_GEOM_EPSILON
		PX_FORCE_INLINE				void			SetGeomEpsilon(float epsilon=0.0f)		{ mGeomEpsilon	= epsilon;	}
#endif

#ifdef OPC_RAYHIT_CALLBACK
		PX_FORCE_INLINE				void			SetHitCallback(HitCallback cb)			{ mHitCallback	= cb;			}
		PX_FORCE_INLINE				void			SetUserData(void* user_data)			{ mUserData		= user_data;	}
#else
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Settings: sets the destination array for stabbed faces.
		 *	\param		cf			[in] destination array, filled during queries
		 *	\see		SetClosestHit(bool flag)
		 *	\see		SetCulling(bool flag)
		 *	\see		SetMaxDist(float max_dist)
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		PX_FORCE_INLINE				void			SetDestination(CollisionFaces* cf)		{ mStabbedFaces	= cf;		}
#endif
		// In-out test
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Stats: gets the number of intersection found after a collision query. Can be used for in/out tests.
		 *	\see		GetNbRayBVTests()
		 *	\see		GetNbRayPrimTests()
		 *	\return		the number of valid intersections during last query
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		PX_FORCE_INLINE				PxU32			GetNbIntersections()			const	{ return mNbIntersections;	}

		protected:
		// Ray in local space
							PxVec3			mOrigin;			//!< Ray origin
							PxVec3			mDir;				//!< Ray direction (normalized)
							PxVec3			mFDir;				//!< fabsf(mDir)
							PxVec3			mData, mData2;
		// Stabbed faces
							CollisionFace	mStabbedFace;		//!< Current stabbed face
#ifdef OPC_RAYHIT_CALLBACK
							HitCallback		mHitCallback;		//!< Callback used to record a hit
							void*			mUserData;			//!< User-defined data
#else
							CollisionFaces*	mStabbedFaces;		//!< List of stabbed faces
#endif
		// In-out test
							PxU32			mNbIntersections;	//!< Number of valid intersections
		// Settings
							float			mMaxDist;			//!< Valid segment on the ray
#ifdef OPC_GEOM_EPSILON
							float			mGeomEpsilon;		//!< Geometry-dependent epsilon
#endif
#ifndef OPC_RAYHIT_CALLBACK
							bool			mClosestHit;		//!< Report closest hit only
#endif
							bool			mCulling;			//!< Stab culled faces or not
			// Overlap tests
		PX_FORCE_INLINE		Ps::IntBool		RayAABBOverlap(const PxVec3& center, const PxVec3& extents);

		PX_FORCE_INLINE		Ps::IntBool		SegmentAABBOverlap(const PxVec3& center, const PxVec3& extents);
		PX_FORCE_INLINE		Ps::IntBool		RayTriOverlap(const PxVec3& vert0, const PxVec3& vert1, const PxVec3& vert2);

#ifdef OPC_SUPPORT_VMX128
		PX_FORCE_INLINE		Ps::IntBool		RayAABBOverlap(const Cm::PxSimd::Vector4& center, const Cm::PxSimd::Vector4& extents);
		PX_FORCE_INLINE		Ps::IntBool		RayAABBOverlap(const Cm::PxSimd::Vector4& center, const Cm::PxSimd::Vector4& extents,
															const Cm::PxSimd::Vector4& origin, const Cm::PxSimd::Vector4& dir,
															const Cm::PxSimd::Vector4& dirYZX, const Cm::PxSimd::Vector4& fDir,
															const Cm::PxSimd::Vector4& fDirYZZ, const Cm::PxSimd::Vector4& fDirXYX,
															const Cm::PxSimd::Vector4& zero);
#endif
			// Init methods
							Ps::IntBool		InitQuery(const PxVec3& orig, const PxVec3& dir, const Cm::Matrix34* world=NULL, PxU32* face_id=NULL);

		PX_FORCE_INLINE		void			SetupSegment()
											{
												// For Segment-AABB overlap
												mData = 0.5f * mDir * mMaxDist;
												mData2 = mOrigin + mData;

												// Precompute mFDir;
												mFDir.x = PxAbs(mData.x);
												mFDir.y = PxAbs(mData.y);
												mFDir.z = PxAbs(mData.z);
											}

		PX_FORCE_INLINE		void			SetContact(PxU32 primIndex, PxU32 flag)
											{
												mNbIntersections++;
												// Set contact status
												mFlags |= flag;
												// The contact has been found and recorded in mStabbedFace
												mStabbedFace.mFaceID = primIndex;
											}

		PX_FORCE_INLINE		void			HandleContact(PxU32 primIndex, PxU32 flag)
											{
												SetContact(primIndex, flag);
#ifdef OPC_RAYHIT_CALLBACK
												if(mHitCallback)	(mHitCallback)(mStabbedFace, mUserData);
#else
												// Now we can also record it in mStabbedFaces if available
												if(mStabbedFaces)
												{
													// If we want all faces or if that's the first one we hit
													if(!mClosestHit || !mStabbedFaces->GetNbFaces())
													{
														mStabbedFaces->AddFace(mStabbedFace);
													}
													else
													{
														// We only keep closest hit
														CollisionFace* Current = const_cast<CollisionFace*>(mStabbedFaces->GetFaces());
														if(Current && mStabbedFace.mDistance<Current->mDistance)
															*Current = mStabbedFace;
													}
												}
#endif
											}

		PX_FORCE_INLINE		void			PerformRayPrimTest(PxU32 primIndex, PxU32 flag)
											{
												const MeshInterface* mi = mIMesh;
												PxMemFetchSmallBuffer buf0, buf1, buf2;
												PxU32* has16BitIndices = pxMemFetchAsync<PxU32>(PxMemFetchPtr(mi)+OFFSET_OF(MeshInterface, mHas16BitIndices), 5, buf0);
												PxMemFetchPtr* mTris = pxMemFetchAsync<PxMemFetchPtr>(PxMemFetchPtr(mi)+OFFSET_OF(MeshInterface, mTris), 5, buf1);
												PxMemFetchPtr* mVerts = pxMemFetchAsync<PxMemFetchPtr>(PxMemFetchPtr(mi)+OFFSET_OF(MeshInterface, mVerts), 5, buf2);
												pxMemFetchWait(5);

												PxVec3 v0, v1, v2;
												MeshInterface::GetTriangleVerts(*has16BitIndices, *mTris, *mVerts, primIndex, v0, v1, v2);

												// Perform ray-tri overlap test
												if(RayTriOverlap(v0, v1, v2))
													HandleContact(primIndex, flag);
											}

		PX_FORCE_INLINE		void			PerformSegmentPrimTest(PxU32 primIndex, PxU32 flag)
											{
												const MeshInterface* mi = mIMesh;
												PxMemFetchSmallBuffer buf0, buf1, buf2;
												PxU32* has16BitIndices = pxMemFetchAsync<PxU32>(PxMemFetchPtr(mi)+OFFSET_OF(MeshInterface, mHas16BitIndices), 5, buf0);
												PxMemFetchPtr* mTris = pxMemFetchAsync<PxMemFetchPtr>(PxMemFetchPtr(mi)+OFFSET_OF(MeshInterface, mTris), 5, buf1);
												PxMemFetchPtr* mVerts = pxMemFetchAsync<PxMemFetchPtr>(PxMemFetchPtr(mi)+OFFSET_OF(MeshInterface, mVerts), 5, buf2);
												pxMemFetchWait(5);

												PxVec3 v0, v1, v2;
												MeshInterface::GetTriangleVerts(*has16BitIndices, *mTris, *mVerts, primIndex, v0, v1, v2);

												// Perform ray-tri overlap test
												if(RayTriOverlap(v0, v1, v2))
												{
													// Intersection point is valid if dist < segment's length
													// We know dist>0 so we can use integers
													if(PX_IR(mStabbedFace.mDistance)<PX_IR(mMaxDist))
														HandleContact(primIndex, flag);
												}
												
											}

		PX_FORCE_INLINE		void			PerformSegmentPrimTestAndShrink(PxU32 primIndex, PxU32 flag)
											{
												// Request vertices from the app
												VertexPointers VP;
												mIMesh->GetTriangle(VP, primIndex);

												// Perform ray-tri overlap test
												if(RayTriOverlap(*VP.vertex[0], *VP.vertex[1], *VP.vertex[2]))
												{
													// Intersection point is valid if dist < segment's length
													// We know dist>0 so we can use integers
													if(PX_IR(mStabbedFace.mDistance)<PX_IR(mMaxDist))
													{
														HandleContact(primIndex, OPC_CONTACT);

														// Shrink the ray
														mMaxDist = mStabbedFace.mDistance;
														// Recompute derived:
														SetupSegment();
													}
												}
											}
		};

#define USE_SHRINKING_CALLBACK

	struct RayColliderContactCallback
	{
		// return false for early out
		virtual bool processResults(PxU32 has16BitIndices, void* pTris, CollisionFace* faces, const PxVec3& lp0, const PxVec3& lp1, const PxVec3& lp2) = 0;
	};

	class HybridRayCollider : public RayCollider
	{
	public:
		PX_FORCE_INLINE	HybridRayCollider()	{}
		PX_FORCE_INLINE	~HybridRayCollider(){}

		bool	Collide(const PxVec3& orig, const PxVec3& dir, const HybridModelData& model, const Cm::Matrix34* world=NULL, PxU32* cache=NULL, RayColliderContactCallback* callback=NULL);

	protected:

#ifdef USE_SHRINKING_CALLBACK
		void	TestLeaf(PxU32 touched_box);
		void	TestLeafAndShrink(PxU32 touched_box);
#else
		// AM: the following array is now a pointer so that we can reuse
		// the same array for all queries of all colliders.
							Container		mTouchedBoxes;
#endif
	};

} // namespace Ice

}

#include "OPC_RayAABBOverlap.h"
#include "OPC_RayTriOverlap.h"

#endif // OPC_RAYCOLLIDER_H

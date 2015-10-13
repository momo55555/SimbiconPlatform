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


#include "PsIntrinsics.h"
#include "GuMeshQuery.h"

#include "GuGeomUtilsInternal.h"
#include "PxTriangleMeshGeometry.h"
#include "PxSphereGeometry.h"
#include "PxBoxGeometry.h"
#include "PxCapsuleGeometry.h"
#include "GuEntityReport.h"
#include "GuHeightFieldUtil.h"
#include "GuOverlapTests.h"
#include "GuBoxConversion.h"
#include "GuIntersectionTriangleBox.h"
#include "GuTriangleMesh.h"
#include "CmScaling.h"
#include "GuAdjacencies.h"

using namespace physx;

namespace {

	class HfTrianglesEntityReport2 : public Gu::EntityReport<PxU32>
	{
	public:
		HfTrianglesEntityReport2(
			PxU32* results,
			PxU32 maxResults,
			PxU32 startIndex,
			Gu::HeightFieldUtil& hfUtil,
			const PxVec3& boxCenter,
			const PxVec3& boxExtents,
			const PxQuat& boxRot,
			bool aabbOverlap) :

			mResults		(results),
			mMaxResults		(maxResults),
			mStartIndex		(startIndex),
			mNbResults		(0),
			mNbSkipped		(0),
			mHfUtil			(hfUtil),
			mBoxCenter		(boxCenter),
			mBoxExtents		(boxExtents),
			mBoxRot			(boxRot),
			mAABBOverlap	(aabbOverlap),
			mOverflow		(false)
		{
		}

		PX_FORCE_INLINE	bool	add(PxU32 index)
		{
			if(mNbResults>=mMaxResults)
			{
				mOverflow = true;
				return false;
			}

			if(mNbSkipped>=mStartIndex)
				mResults[mNbResults++] = index;
			else
				mNbSkipped++;

			return true;
		}

		virtual bool onEvent(PxU32 nbEntities, PxU32* entities)
		{
			if(mAABBOverlap)
			{
				while(nbEntities--)
					if(!add(*entities++))
						return false;
			}
			else
			{
				// PT: TODO: use a matrix here
				const PxTransform box2Hf(mBoxCenter, mBoxRot);

				for(PxU32 i=0; i < nbEntities; i++)
				{
					Gu::Triangle tri;
					mHfUtil.getTriangle(PxTransform::createIdentity(), tri, NULL, NULL, NULL, entities[i], false, false);  // First parameter not needed if local space triangle is enough

					// Transform triangle vertices to box space
					const PxVec3 v0 = box2Hf.transformInv(tri.verts[0]);
					const PxVec3 v1 = box2Hf.transformInv(tri.verts[1]);
					const PxVec3 v2 = box2Hf.transformInv(tri.verts[2]);

					const PxVec3 zero(0.0f);
					if(Gu::intersectTriangleBox(zero, mBoxExtents, v0, v1, v2))
					{
						if(!add(entities[i]))
							return false;
					}
				}
			}

			return true;
		}

			Gu::HeightFieldUtil&	mHfUtil;
			PxVec3					mBoxCenter;
			PxVec3					mBoxExtents;
			PxQuat					mBoxRot;
			PxU32*					mResults;
			PxU32					mMaxResults;
			PxU32					mStartIndex;
			PxU32					mNbResults;
			PxU32					mNbSkipped;
			bool					mAABBOverlap;
			bool					mOverflow;
	};


} // namespace

void physx::Gu::MeshQuery::getTriangle(const PxTriangleMeshGeometry& triGeom, const PxTransform& globalPose,  PxTriangleID triangleIndex, Gu::Triangle& triangle, Gu::Triangle* edgeTri, PxU32* returnedFlags, PxU32* vertexIndices)
{
	Cm::Matrix34 vertex2worldSkew = globalPose * triGeom.scale;

	Gu::TriangleMesh* tm = static_cast<Gu::TriangleMesh*>(triGeom.triangleMesh);

	tm->computeWorldTriangle(triangle, triangleIndex, vertex2worldSkew, vertexIndices);

	// Edge-triangle

	// Edge normals
	PxU32 flags = 0;
	if(edgeTri || returnedFlags)
	{
		const Adjacencies* Adj = tm->getAdjacencies();

		Gu::Triangle& WT = triangle;
		Gu::Triangle* ET = edgeTri;

		PxVec3 Normal;
		WT.normal(Normal);

		const AdjTriangle& CurTri = Adj->mFaces[triangleIndex];

		// Compute edge normals = average(adjacent face normals)

		if(CurTri.HasActiveEdge01())
		{
			flags |= PxTriangleFlags::eACTIVE_EDGE01;

			if(CurTri.IsBoundaryEdge(EDGE01))
			{
				flags |= PxTriangleFlags::eBOUNDARY_EDGE01;
				if(ET)	ET->verts[0] = Normal;
			}
			else
			{
				Gu::Triangle T;
				tm->computeWorldTriangle(T, CurTri.GetAdjTri(EDGE01), vertex2worldSkew);

				PxVec3 Normal0;	T.normal(Normal0);
				if(ET)	ET->verts[0] = (Normal + Normal0) * 0.5f;
			}
		}
#ifdef _DEBUG
//		else if(ET)	ET->verts[0].SetNotUsed();
#endif

		if(CurTri.HasActiveEdge12())
		{
			flags |= PxTriangleFlags::eACTIVE_EDGE12;

			if(CurTri.IsBoundaryEdge(EDGE12))
			{
				flags |= PxTriangleFlags::eBOUNDARY_EDGE12;
				if(ET)	ET->verts[1] = Normal;
			}
			else
			{
				Gu::Triangle T;
				tm->computeWorldTriangle(T, CurTri.GetAdjTri(EDGE12), vertex2worldSkew);

				PxVec3 Normal2;	T.normal(Normal2);
				if(ET)	ET->verts[1] = (Normal + Normal2) * 0.5f;
			}
		}
#ifdef _DEBUG
//		else if(ET)	ET->verts[1].SetNotUsed();
#endif

		if(CurTri.HasActiveEdge20())
		{
			flags |= PxTriangleFlags::eACTIVE_EDGE20;

			if(CurTri.IsBoundaryEdge(EDGE02))
			{
				flags |= PxTriangleFlags::eBOUNDARY_EDGE20;
				if(ET)	ET->verts[2] = Normal;
			}
			else
			{
				Gu::Triangle T;
				tm->computeWorldTriangle(T, CurTri.GetAdjTri(EDGE02), vertex2worldSkew);

				PxVec3 Normal1;	T.normal(Normal1);
				if(ET)	ET->verts[2] = (Normal + Normal1) * 0.5f;
			}
		}
#ifdef _DEBUG
//		else if(ET)	ET->verts[2].SetNotUsed();
#endif
	}

	// Return triangle flags if needed
	if(returnedFlags)
		*returnedFlags = flags;
}


void physx::Gu::MeshQuery::getTriangle(const PxHeightFieldGeometry& hfGeom, const PxTransform& globalPose,  PxTriangleID triangleIndex, Gu::Triangle& triangle, Gu::Triangle* edgeTri, PxU32* flags, PxU32* vertexIndices)
{
	Gu::HeightFieldUtil hfUtil(hfGeom);
	
	hfUtil.getTriangle(globalPose, triangle, edgeTri, flags, vertexIndices, triangleIndex, true, true);
}


PxU32 physx::Gu::MeshQuery::findOverlapTriangleMesh(	const PxGeometry& geom0, const PxTransform& pose0,
												const PxTriangleMeshGeometry& geom1, const PxTransform& pose1,
												PxU32* results, PxU32 maxResults, PxU32 startIndex, bool& overflow)
{
	switch(geom0.getType())
	{
		case PxGeometryType::eBOX :
		{
			const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom0);

			Gu::Box box;
			buildFrom(box, pose0.p, boxGeom.halfExtents, pose0.q);

			Gu::TriangleMesh* tm = static_cast<Gu::TriangleMesh*>(geom1.triangleMesh);
			return Gu::findOverlapOBBMesh(box, tm->getOpcodeModel(), pose1, geom1.scale, results, maxResults, startIndex, overflow);
		}
		break;

		case PxGeometryType::eCAPSULE :
		{
			const PxCapsuleGeometry& capsGeom = static_cast<const PxCapsuleGeometry&>(geom0);

			Gu::Capsule worldCapsule;	// PT: TODO: fix this wrong name
			Gu::getWorldCapsule(worldCapsule, capsGeom, pose0);
			
			Gu::TriangleMesh* tm = static_cast<Gu::TriangleMesh*>(geom1.triangleMesh);
			return Gu::findOverlapCapsuleMesh(worldCapsule, tm->getOpcodeModel(), pose1, geom1.scale, results, maxResults, startIndex, overflow);
		}
		break;

		case PxGeometryType::eSPHERE :
		{
			const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom0);

			Gu::TriangleMesh* tm = static_cast<Gu::TriangleMesh*>(geom1.triangleMesh);
			return Gu::findOverlapSphereMesh(Gu::Sphere(pose0.p, sphereGeom.radius), tm->getOpcodeModel(), pose1, geom1.scale, results, maxResults, startIndex, overflow);
		}
		break;

		default :
			PX_CHECK_AND_RETURN_VAL(false, "findOverlapTriangleMesh: Only box, capsule and sphere geometries are supported.", false);
		break;
	}

	return 0;
}

PxU32 physx::Gu::MeshQuery::findOverlapHeightField(const PxGeometry& geom0, const PxTransform& pose0,
											const PxHeightFieldGeometry& geom1, const PxTransform& pose1,
											PxU32* results, PxU32 maxResults, PxU32 startIndex, bool& overflow)
{
	const PxTransform localPose0 = pose1.transformInv(pose0);

	switch(geom0.getType())
	{
		case PxGeometryType::eBOX :
		{
			const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom0);

			const bool isAABB = ((localPose0.q.x == 0.0f) && (localPose0.q.y == 0.0f) && (localPose0.q.z == 0.0f));
			
			PxBounds3 bounds;
			if (isAABB)
				bounds = PxBounds3::centerExtents(localPose0.p, boxGeom.halfExtents);
			else
				bounds = PxBounds3::poseExtent(localPose0, boxGeom.halfExtents); // box.halfExtents is really extent

			Gu::HeightFieldUtil hfUtil(geom1);
			HfTrianglesEntityReport2 entityReport(results, maxResults, startIndex, hfUtil, localPose0.p, boxGeom.halfExtents, localPose0.q, isAABB);

			// PT: TODO: add a helper to expose this number?
/*			const PxU32 maxNbOverlapRows = static_cast<PxU32>( Ps::ceil(((bounds.getDimensions().x * (1.0f / geom1.rowScale)) + 1.0f)) );
			const PxU32 maxNbOverlapCols = static_cast<PxU32>( Ps::ceil(((bounds.getDimensions().z * (1.0f / geom1.columnScale)) + 1.0f)) );

			PxU32 maxNbTriangles = (maxNbOverlapCols * maxNbOverlapRows) << 1;  // maximum number of height field triangles overlapping the local AABB
			maxNbTriangles = PxMax(maxNbTriangles, (PxU32)8);  // No matter how small the AABB is, it can always have its center at the shared point of 4 cells*/

			hfUtil.overlapAABBTriangles(pose1, bounds, 0, &entityReport);
			overflow = entityReport.mOverflow;
			return entityReport.mNbResults;
		}
		break;

		case PxGeometryType::eCAPSULE :
		case PxGeometryType::eSPHERE :
		default :
			PX_CHECK_AND_RETURN_VAL(false, "findOverlapHeightField: Only box queries are supported.", false);
		break;
	}

	return 0;
}

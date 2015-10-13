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

#include "GuGeometryUnion.h"
#include "PxBounds3.h"
#include "PsMathUtils.h"
#include "GuConvexMeshData.h"
#include "GuTriangleMeshData.h"
#include "GuHeightField.h"
#include "GuHeightFieldData.h"
#include "GuPlane.h"

using namespace physx;

// PT: specialized version only computing the extent. TODO: refactor
static PX_FORCE_INLINE void basisExtent(PxVec3& w, const PxMat33& basis, const PxVec3& extent)
{
	// extended basis vectors
	const PxVec3 c0 = basis.column0 * extent.x;
	const PxVec3 c1 = basis.column1 * extent.y;
	const PxVec3 c2 = basis.column2 * extent.z;

	// find combination of base vectors that produces max. distance for each component = sum of abs()
	w.x = PxAbs(c0.x) + PxAbs(c1.x) + PxAbs(c2.x);
	w.y = PxAbs(c0.y) + PxAbs(c1.y) + PxAbs(c2.y);
	w.z = PxAbs(c0.z) + PxAbs(c1.z) + PxAbs(c2.z);
}

static PX_FORCE_INLINE void basisExtent(PxVec3& w, const Cm::Matrix34& basis, const PxVec3& extent)
{
	// extended basis vectors
	const PxVec3 c0 = basis.base0 * extent.x;
	const PxVec3 c1 = basis.base1 * extent.y;
	const PxVec3 c2 = basis.base2 * extent.z;

	// find combination of base vectors that produces max. distance for each component = sum of abs()
	w.x = PxAbs(c0.x) + PxAbs(c1.x) + PxAbs(c2.x);
	w.y = PxAbs(c0.y) + PxAbs(c1.y) + PxAbs(c2.y);
	w.z = PxAbs(c0.z) + PxAbs(c1.z) + PxAbs(c2.z);
}

// PT: this one is not used here but we should revisit the original code, looks like there's a useless conversion
/*
static PX_FORCE_INLINE void transformNoEmptyTest(PxVec3& c, PxVec3& ext, const PxTransform& t, const PxBounds3& bounds)
{
//PX_INLINE PxBounds3 PxBounds3::basisExtent(const PxVec3& center, const PxMat33& basis, const PxVec3& extent)

	const PxVec3 boundsCenter = bounds.getCenter();
	c = t.transform(boundsCenter);
	const PxMat33 basis(t.q);
	PxVec3 c2 = (basis * boundsCenter) + t.p;
//	return PxBounds3::basisExtent(center, basis, bounds.getExtents());
	basisExtent(ext, basis, bounds.getExtents());
}
*/

// PT: no "isEmpty" test in that one.
static PX_FORCE_INLINE void transformNoEmptyTest(PxVec3& c, PxVec3& ext, const Cm::Matrix34& matrix, const PxBounds3& bounds)
{
	const PxVec3 boundsCenter = bounds.getCenter();
	c = matrix.transform( boundsCenter );
	basisExtent(ext, matrix, bounds.getExtents());
}

// PT: this version shows what is really going on here, and it's not pretty. Still, is has no FCMP and an order of magnitude less LHS than the original code.
#define IEEE_1_0				0x3f800000					//!< integer representation of 1.0
static PX_FORCE_INLINE void transformNoEmptyTest(PxVec3& c, PxVec3& ext, const PxTransform& transform, const PxMeshScale& scale, const PxBounds3& bounds)
{
/*	PT: so the decomposition is:

	const PxMat33Legacy tmp(transform.q);	// quat-to-matrix

	const PxMat33 tmp2 = scale.toMat33();	// quat-to-matrix + transpose + matrix multiply ==> could be precomputed

	const PxMat34Legacy legacy(tmp * tmp2, transform.p);	// matrix conversion + legacy matrix multiply

	transformNoEmptyTest(c, ext, legacy, bounds);

	Writing it with a single line has less LHS, so:
*/
#ifdef _XBOX
	if(IR(scale.scale.x)==IEEE_1_0 && IR(scale.scale.y)==IEEE_1_0 && IR(scale.scale.z)==IEEE_1_0)
#else
	if(scale.isIdentity())
#endif
	{
		transformNoEmptyTest(c, ext, Cm::Matrix34(transform), bounds);
	}
	else
	{
		transformNoEmptyTest(c, ext, Cm::Matrix34(PxMat33(transform.q) * scale.toMat33(), transform.p), bounds);
	}
}

static void computeMeshBounds(const PxTransform& pose, const PxBounds3* PX_RESTRICT localSpaceBounds, const PxMeshScale& meshScale, PxVec3& origin, PxVec3& extent)
{
	Ps::prefetch128(localSpaceBounds);	// PT: this one helps reducing L2 misses in transformNoEmptyTest
	transformNoEmptyTest(origin, extent, pose, meshScale, *localSpaceBounds);
}

// PT: this is really the same as in GuObjectBounds.cpp, unify!
void Gu::GeometryUnion::computeBounds(const PxTransform& pose, const PxBounds3* PX_RESTRICT localSpaceBounds, PxVec3& origin, PxVec3& extent) const
{
	// Box, Convex, Mesh and HeightField will compute local bounds and pose to world space.
	// Sphere, Capsule & Plane will compute world space bounds directly.

	switch (getType())
	{
	case PxGeometryType::eSPHERE:
		{
			const PxSphereGeometry& shape = get<const PxSphereGeometry>();
			origin = pose.p;
			extent = PxVec3(shape.radius, shape.radius, shape.radius);
			return;
		}
	case PxGeometryType::ePLANE:
		{
			// PT: A plane is infinite, so usually the bounding box covers the whole world.
			// Now, in particular cases when the plane is axis-aligned, we can take
			// advantage of this to compute a smaller bounding box anyway.

			const PxF32 bigValue = 1000000.0f;	// PT: don't use PX_MAX_REAL as it doesn't work well with the broad-phase
			PxVec3 minPt = PxVec3(-bigValue, -bigValue, -bigValue);
			PxVec3 maxPt = PxVec3(bigValue, bigValue, bigValue);

			const PxVec3 planeNormal = pose.q.getBasisVector0();
			const Gu::Plane plane(pose.p, planeNormal);

			const float nx = PxAbs(planeNormal.x);
			const float ny = PxAbs(planeNormal.y);
			const float nz = PxAbs(planeNormal.z);
			const float epsilon = 1e-6f;
			const float oneMinusEpsilon = 1.0f - epsilon;
			if(nx>oneMinusEpsilon && ny<epsilon && nz<epsilon)
			{
				if(planeNormal.x>0.0f)	maxPt.x = -plane.d;
				else					minPt.x = plane.d;
			}
			else if(nx<epsilon && ny>oneMinusEpsilon && nz<epsilon)
			{
				if(planeNormal.y>0.0f)	maxPt.y = -plane.d;
				else					minPt.y = plane.d;
			}
			else if(nx<epsilon && ny<epsilon && nz>oneMinusEpsilon)
			{
				if(planeNormal.z>0.0f)	maxPt.z = -plane.d;
				else					minPt.z = plane.d;
			}
			origin = (maxPt + minPt)*0.5f;
			extent = (maxPt - minPt)*0.5f;
			return;
		}
	case PxGeometryType::eCAPSULE:
		{
			const PxCapsuleGeometry& shape = get<const PxCapsuleGeometry>();
			origin = pose.p;
			const PxVec3 d = pose.q.getBasisVector0();
			for(PxU32 ax = 0; ax<3; ax++)
				extent[ax] = PxAbs(d[ax]) * shape.halfHeight + shape.radius;
			return;
		}

	case PxGeometryType::eBOX:
		{
			const PxBoxGeometry& shape = get<const PxBoxGeometry>();

			::basisExtent(extent, PxMat33(pose.q), shape.halfExtents);
			origin = pose.p;
		}
		break;

	case PxGeometryType::eCONVEXMESH:
		{
			const PxConvexMeshGeometryLL& shape = get<const PxConvexMeshGeometryLL>();

			computeMeshBounds(pose, localSpaceBounds ? localSpaceBounds : &shape.hullData->mAABB, shape.scale, origin, extent);
			return;
		}
	case PxGeometryType::eTRIANGLEMESH:
		{
			const PxTriangleMeshGeometryLL& shape = get<const PxTriangleMeshGeometryLL>();

			computeMeshBounds(pose, localSpaceBounds ? localSpaceBounds : &shape.meshData->mAABB, shape.scale, origin, extent);
			return;
		}

	case PxGeometryType::eHEIGHTFIELD:
		{
			const PxHeightFieldGeometryLL& shape = get<const PxHeightFieldGeometryLL>();
			const PxMeshScale scale(PxVec3(shape.rowScale, shape.heightScale, shape.rowScale), PxQuat::createIdentity());

			computeMeshBounds(pose, localSpaceBounds ? localSpaceBounds : &shape.heightFieldData->mAABB, scale, origin, extent);
		}
		break;
	default:
		{
			PX_ASSERT(0);		
#if !__SPU__
			Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "Gu::GeometryUnion::computeBounds: Unknown shape type.");
#endif
		}
	}
}

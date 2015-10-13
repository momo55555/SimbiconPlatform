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
#include "PsUserAllocated.h"
#include "ConvexHull.h"
#include "GuPlane.h"
#include "GuHillClimbing.h"
#include "GuBigConvexData.h"
#include "PxMat33.h"
#include "GuCubeIndex.h"
#include "GuConvexMeshData.h"

// This is horrible. Really need to make cooking stay self contained or link
// to libraries it needs instead of this conditionally compiled chaos. -jd
#if !(defined(PX_COOKING) && defined(PX_PHYSX_STATIC_LIB))

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Constructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace physx
{
namespace Gu
{

void initConvexHullData(Gu::ConvexHullData &data)
{
	data.mAABB.setEmpty();
	data.mNbHullVertices = 0;
	data.mCenterOfMass = PxVec3(0);
	data.mNbPolygons = 0;
	data.mPolygons = NULL;
	data.mNbEdges = 0;
	data.mBigConvexRawData = NULL;
	data.mInternal.mRadius = 0.0f;
	data.mInternal.mExtents[0] = data.mInternal.mExtents[1] = data.mInternal.mExtents[2] = 0.0f;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Checks a point is inside the hull.
 *	\param		p	[in] point in local space
 *	\return		true if the hull contains the point
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool convexHullContains(const ConvexHullData &data, const PxVec3& p)
{
	PxU32 Nb = data.mNbPolygons;
	const Gu::HullPolygonData* Polygons = data.mPolygons;
	while(Nb--)
	{
		const Gu::Plane& pl = Polygons->mPlane;
		if(pl.distance(p) > 0.0f)	return false;
		Polygons++;
	}
	return true;
}

PxVec3 hullInverseSupportMapping(const ConvexHullData& hull, const PxVec3& point, int& isFace, const PxMat33& vert2ShapeSkew)
{
	PX_ASSERT(hull.mNbPolygons > 0);

	PxU32 minIndex = 0;
	PxReal minD = PX_MAX_REAL;
    const PxReal eps = 1e-3f;
    PxU32 countOnFace = 0;
	const PxVec3 vertexSpacePoint = vert2ShapeSkew.getInverse() * point;
	for (int j = 0; j < hull.mNbPolygons; j++)
	{
		PxReal d = hull.mPolygons[j].mPlane.distance(vertexSpacePoint);
		d = PxAbs(d);
		if (d < eps)
			countOnFace ++;
		if (d < minD)
		{
			minIndex = j;
			minD = d;
		}
	}

	isFace = (countOnFace == 1);

	return hull.mPolygons[minIndex].mPlane.normal;
}

//returns the maximal vertex in shape space
// PT: this function should be removed. We already have 2 different project hull functions in PxcShapeConvex & GuGJKObjectSupport, this one looks like a weird mix of both!
PxVec3 projectHull_(const ConvexHullData &hull,
				   float& minimum, 
				   float& maximum, 
				   const PxVec3& localDir, // not necessarily normalized
				   const PxMat33& vert2ShapeSkew)
{
	//use property that x|My == Mx|y for symmetric M to avoid having to transform vertices.
	const PxVec3 vertexSpaceDir = vert2ShapeSkew * localDir.getNormalized();

	const PxVec3* Verts = hull.getHullVertices();
	const PxVec3* bestVert = NULL;

	if(!hull.mBigConvexRawData)	// Brute-force, local space. Experiments show break-even point is around 32 verts.
	{
		PxU32 NbVerts = hull.mNbHullVertices;
		minimum = PX_MAX_F32;
		maximum = -PX_MAX_F32;
		while(NbVerts--)
		{
			const float dp = (*Verts).dot(vertexSpaceDir);
			if(dp < minimum)	minimum = dp;
			if(dp > maximum)	{ maximum = dp; bestVert = Verts; }

			Verts++;
		}

		return vert2ShapeSkew * *bestVert;
	}
	else //*/if(1)	// This version is better for objects with a lot of vertices
	{
		const PxU32 Offset = ComputeCubemapNearestOffset(vertexSpaceDir, hull.mBigConvexRawData->mSubdiv);
		PxU32 MinID = hull.mBigConvexRawData->mSamples[Offset];
		PxU32 MaxID = hull.mBigConvexRawData->getSamples2()[Offset];

		localSearch(MinID, -vertexSpaceDir, Verts, hull.mBigConvexRawData);
		localSearch(MaxID, vertexSpaceDir, Verts, hull.mBigConvexRawData);

		minimum = (Verts[MinID].dot(vertexSpaceDir));
		maximum = (Verts[MaxID].dot(vertexSpaceDir));

		PX_ASSERT(maximum >= minimum);

		return vert2ShapeSkew * Verts[MaxID];
	}
}

}

}

//~PX_SERIALIZATION

#endif // #if !(defined(PX_COOKING) && defined(PX_PHYSX_STATIC_LIB))

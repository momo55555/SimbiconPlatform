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
#include "GuTriangle.h"
#include "GuPlane.h"
#include "./Ice/IcePolygon.h"

using namespace physx;
using namespace Ice;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Finds the plane equation of a polygon.
 *	\fn			CreatePolygonPlane(Plane& plane, PxU32 nb_verts, const PxU32* indices, const PxVec3* verts)
 *	\relates	Polygon
 *	\param		plane		[out] plane equation
 *	\param		nb_verts	[in] number of vertices
 *	\param		indices		[in] array of vertex indices
 *	\param		verts		[in] array of vertices
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Ice::CreatePolygonPlane(Gu::Plane& plane, PxU32 nb_verts, const PxU32* indices, const PxVec3* verts)
{
	// Checkings
	if(!nb_verts || !indices || !verts)	return false;

	// The problem here is to find 3 good polygon vertices, to create a robust plane equation.
	//
	// The naive way is to take the 3 first vertices, but it doesn't work because sometimes a
	// polygon segment has been cut in multiple sub-segments, and the 3 first vertices might
	// then be aligned.

	// This one can still fail when the polygon has 5 vertices and the 3 first ones are aligned, etc.
	// So we "rotate" in search of best triangle.
	PxU32 Step = nb_verts/3;
	float MaxArea = -PX_MAX_F32;
	PxU32 BestVRef0 = 0;
	for(PxU32 VRef0=0;VRef0<nb_verts;VRef0++)
	{
		const PxU32 VRef1 = (VRef0 + Step) % nb_verts;
		const PxU32 VRef2 = (VRef1 + Step) % nb_verts;

		const Gu::Triangle T(verts[indices[VRef0]], verts[indices[VRef1]], verts[indices[VRef2]]);

		const float Area = T.area();
		if(Area>MaxArea)
		{
			MaxArea = Area;
			BestVRef0 = VRef0;
		}
	}

	// Compute plane using best triangle
	{
		const PxU32 VRef0 = BestVRef0;
		const PxU32 VRef1 = (VRef0 + Step) % nb_verts;
		const PxU32 VRef2 = (VRef1 + Step) % nb_verts;
		plane.set(verts[indices[VRef0]], verts[indices[VRef1]], verts[indices[VRef2]]);
	}

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes the plane equation of a polygon using Newell's method
 *	\fn			ComputeNewellPlane(Plane& plane, PxU32 nb_verts, const PxU32* indices, const PxVec3* verts)
 *	\relates	Polygon
 *	\param		plane		[out] plane equation
 *	\param		nb_verts	[in] number of vertices
 *	\param		indices		[in] array of vertex indices
 *	\param		verts		[in] array of vertices
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//bool Ice::ComputeNewellPlane(Plane& plane, PxU32 nb_verts, const PxU32* indices, const PxVec3* verts)
bool Ice::ComputeNewellPlane(Gu::Plane& plane, PxU32 nb_verts, const PxU8* indices, const PxVec3* verts)
{
	// Checkings
	if(!nb_verts || !indices || !verts)	return false;

	PxVec3 Centroid(0,0,0), Normal(0,0,0);
	for(PxU32 i=nb_verts-1, j=0; j<nb_verts; i=j, j++)
	{
		Normal.x += (verts[indices[i]].y - verts[indices[j]].y) * (verts[indices[i]].z + verts[indices[j]].z);
		Normal.y += (verts[indices[i]].z - verts[indices[j]].z) * (verts[indices[i]].x + verts[indices[j]].x);
		Normal.z += (verts[indices[i]].x - verts[indices[j]].x) * (verts[indices[i]].y + verts[indices[j]].y);
		Centroid += verts[indices[j]];
	}
	plane.normal = Normal;
	plane.normal.normalize();
	plane.d = -(Centroid.dot(plane.normal))/float(nb_verts);

	return true;
}


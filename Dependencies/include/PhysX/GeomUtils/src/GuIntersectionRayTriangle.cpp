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


#include "PsFPU.h"
#include "GuIntersectionRayTriangle.h"
#include "PxVec3.h"

#define LOCAL_EPSILON 0.000001f

using namespace physx;

bool Gu::intersectLineTriangleCulling(	const PxVec3& orig, const PxVec3& dir,
										const PxVec3& vert0, const PxVec3& vert1, const PxVec3& vert2,
										PxReal&	t, PxReal& u, PxReal& v,
										PxF32 enlarge)
{
	// Find vectors for two edges sharing vert0
	const PxVec3 edge1 = vert1 - vert0;
	const PxVec3 edge2 = vert2 - vert0;

	// Begin calculating determinant - also used to calculate U parameter
	const PxVec3 pvec = dir.cross(edge2);

	// If determinant is near zero, ray lies in plane of triangle
	const PxReal det = edge1.dot(pvec);

	if(det<LOCAL_EPSILON)
		return false;

	// Calculate distance from vert0 to ray origin
	const PxVec3 tvec = orig - vert0;

	// Calculate U parameter and test bounds
	u = tvec.dot(pvec);
	if(u<-enlarge || u>det+enlarge)
		return false;

	// Prepare to test V parameter
	const PxVec3 qvec = tvec.cross(edge1);

	// Calculate V parameter and test bounds
	v = dir.dot(qvec);
	if(v<-enlarge || u+v>det+enlarge)
		return false;

	// Calculate t, scale parameters, ray intersects triangle
	t = edge2.dot(qvec);
	const PxReal inv_det = 1.0f / det;
	t *= inv_det;
	u *= inv_det;
	v *= inv_det;

	return true;
}

bool Gu::intersectLineTriangleNoCulling(const PxVec3& orig, const PxVec3& dir,
										const PxVec3& vert0, const PxVec3& vert1, const PxVec3& vert2,
										PxReal& t, PxReal& u, PxReal& v,
										PxF32 enlarge)
{
	// Find vectors for two edges sharing vert0
	const PxVec3 edge1 = vert1 - vert0;
	const PxVec3 edge2 = vert2 - vert0;

	// Begin calculating determinant - also used to calculate U parameter
	const PxVec3 pvec = dir.cross(edge2);

	// If determinant is near zero, ray lies in plane of triangle
	const PxReal det = edge1.dot(pvec);

	// the non-culling branch
	if(det>-LOCAL_EPSILON && det<LOCAL_EPSILON)
		return false;
	const PxReal inv_det = 1.0f / det;

	// Calculate distance from vert0 to ray origin
	const PxVec3 tvec = orig - vert0;

	// Calculate U parameter and test bounds
	u = tvec.dot(pvec) * inv_det;
	if(u<-enlarge || u>1.0f+enlarge)
		return false;

	// prepare to test V parameter
	const PxVec3 qvec = tvec.cross(edge1);

	// Calculate V parameter and test bounds
	v = dir.dot(qvec) * inv_det;
	if(v<-enlarge || u+v>1.0f+enlarge)
		return false;

	// Calculate t, ray intersects triangle
	t = edge2.dot(qvec) * inv_det;

	return true;
}



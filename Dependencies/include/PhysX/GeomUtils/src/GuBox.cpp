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
#include "GuBox.h"
#include "GuCapsule.h"
#include "PsMathUtils.h"
#include "GuGeomUtilsInternal.h"

#define	INVSQRT2 0.707106781188f	//!< 1 / sqrt(2)
#define	INVSQRT3 0.577350269189f	//!< 1 / sqrt(3)

using namespace physx;

//duplicated code.
PX_FORCE_INLINE void computeBasis2(const PxVec3& dir, PxVec3& right, PxVec3& up)
{
	PxU32 Coord = Ps::closestAxis(dir);
	if(Coord==0)
	{
		// P = (0,0,1)
		right.x = - dir.y;
		right.y = dir.x;
		right.z = 0.0f;

		up.x = - dir.z * dir.x;
		up.y = - dir.z * dir.y;
		up.z = dir.x * dir.x + dir.y * dir.y;
	}
	else if(Coord==1)
	{
		// P = (1,0,0)
		right.x = 0.0f;
		right.y = - dir.z;
		right.z = dir.y;

		up.x = dir.y * dir.y + dir.z * dir.z;
		up.y = - dir.x * dir.y;
		up.z = - dir.x * dir.z;
	}
	else //if(dir.closestAxis()==2)
	{
		// P = (0,1,0)
		right.x = dir.z;
		right.y = 0.0f;
		right.z = - dir.x;

		up.x = - dir.y * dir.x;
		up.y = dir.z * dir.z + dir.x * dir.x;
		up.z = - dir.y * dir.z;
	}
right.normalize();	// ### added after above fix, to do better
}


void Gu::Box::create(const Gu::Capsule& capsule)
{
	// Box center = center of the two LSS's endpoints
	center = capsule.computeCenter();

	PxVec3 dir = capsule.p1 - capsule.p0;
	const float d = dir.magnitude();
	rot.column0 = dir / d;

	// Box extents
	extents.x = capsule.radius + (d * 0.5f);
	extents.y = capsule.radius;
	extents.z = capsule.radius;

	// Box orientation
	computeBasis2(rot.column0, rot.column1, rot.column2);
}

Ps::IntBool Gu::Box::isInside(const Gu::Box& box) const
{
	// Make a 4x4 from the box & inverse it
	Cm::Matrix34 M0Inv;
	{
		Cm::Matrix34 M0(box.rot, box.center);
		M0Inv = M0.getInverseRT();
	}

	// With our inversed 4x4, create box1 in space of box0
	Gu::Box _1in0;
	rotate(M0Inv, _1in0);

	// This should cancel out box0's rotation, i.e. it's now an AABB.
	// => Center(0,0,0), Rot(identity)

	// The two boxes are in the same space so now we can compare them.

	// Create the AABB of (box1 in space of box0)
	const PxMat33& mtx = _1in0.rot;

	float f = PxAbs(mtx[0][0] * extents.x) + PxAbs(mtx[1][0] * extents.y) + PxAbs(mtx[2][0] * extents.z) - box.extents.x;
	if(f > _1in0.center.x)	return Ps::IntFalse;
	if(-f < _1in0.center.x)	return Ps::IntFalse;

	f = PxAbs(mtx[0][1] * extents.x) + PxAbs(mtx[1][1] * extents.y) + PxAbs(mtx[2][1] * extents.z) - box.extents.y;
	if(f > _1in0.center.y)	return Ps::IntFalse;
	if(-f < _1in0.center.y)	return Ps::IntFalse;

	f = PxAbs(mtx[0][2] * extents.x) + PxAbs(mtx[1][2] * extents.y) + PxAbs(mtx[2][2] * extents.z) - box.extents.z;
	if(f > _1in0.center.z)	return Ps::IntFalse;
	if(-f < _1in0.center.z)	return Ps::IntFalse;

	return Ps::IntTrue;
}



/**
Returns edges.
\return		24 indices (12 edges) indexing the list returned by ComputePoints()
*/
const PxU8* Gu::getBoxEdges()
{
	//     7+------+6			0 = ---
	//     /|     /|			1 = +--
	//    / |    / |			2 = ++-
	//   / 4+---/--+5			3 = -+-
	// 3+------+2 /    y   z	4 = --+
	//  | /    | /     |  /		5 = +-+
	//  |/     |/      |/		6 = +++
	// 0+------+1      *---x	7 = -++

	static PxU8 Indices[] = {
		0, 1,	1, 2,	2, 3,	3, 0,
		7, 6,	6, 5,	5, 4,	4, 7,
		1, 5,	6, 2,
		3, 7,	4, 0
	};
	return Indices;
}

/**
Returns vertex normals.
\return		24 floats (8 normals)
*/
const PxF32* Gu::getBoxVertexNormals()
{
	//     7+------+6			0 = ---
	//     /|     /|			1 = +--
	//    / |    / |			2 = ++-
	//   / 4+---/--+5			3 = -+-
	// 3+------+2 /    y   z	4 = --+
	//  | /    | /     |  /		5 = +-+
	//  |/     |/      |/		6 = +++
	// 0+------+1      *---x	7 = -++

	static PxF32 VertexNormals[] = 
	{
		-INVSQRT3,	-INVSQRT3,	-INVSQRT3,
		INVSQRT3,	-INVSQRT3,	-INVSQRT3,
		INVSQRT3,	INVSQRT3,	-INVSQRT3,
		-INVSQRT3,	INVSQRT3,	-INVSQRT3,
		-INVSQRT3,	-INVSQRT3,	INVSQRT3,
		INVSQRT3,	-INVSQRT3,	INVSQRT3,
		INVSQRT3,	INVSQRT3,	INVSQRT3,
		-INVSQRT3,	INVSQRT3,	INVSQRT3
	};

	return VertexNormals;
}

/**
*	Returns triangles.
*	\return		36 indices (12 triangles) indexing the list returned by ComputePoints()
*/
const PxU8* Gu::getBoxTriangles()
{
	static PxU8 Indices[] = {
		0,2,1,	0,3,2,
		1,6,5,	1,2,6,
		5,7,4,	5,6,7,
		4,3,0,	4,7,3,
		3,6,2,	3,7,6,
		5,0,1,	5,4,0
	};
	return Indices;
}


/**
Returns local edge normals.
\return		edge normals in local space
*/
const PxVec3* Gu::getBoxLocalEdgeNormals()
{
	static PxVec3 EdgeNormals[] = 
	{
		PxVec3(0,			-INVSQRT2,	-INVSQRT2),	// 0-1
		PxVec3(INVSQRT2,	0,			-INVSQRT2),	// 1-2
		PxVec3(0,			INVSQRT2,	-INVSQRT2),	// 2-3
		PxVec3(-INVSQRT2,	0,			-INVSQRT2),	// 3-0

		PxVec3(0,			INVSQRT2,	INVSQRT2),	// 7-6
		PxVec3(INVSQRT2,	0,			INVSQRT2),	// 6-5
		PxVec3(0,			-INVSQRT2,	INVSQRT2),	// 5-4
		PxVec3(-INVSQRT2,	0,			INVSQRT2),	// 4-7

		PxVec3(INVSQRT2,	-INVSQRT2,	0),			// 1-5
		PxVec3(INVSQRT2,	INVSQRT2,	0),			// 6-2
		PxVec3(-INVSQRT2,	INVSQRT2,	0),			// 3-7
		PxVec3(-INVSQRT2,	-INVSQRT2,	0)			// 4-0
	};
	return EdgeNormals;
}

/**
Returns world edge normal
\param		edge_index		[in] 0 <= edge index < 12
\param		world_normal	[out] edge normal in world space
*/
void Gu::computeBoxWorldEdgeNormal(const Gu::Box& box, PxU32 edge_index, PxVec3& world_normal)
{
	PX_ASSERT(edge_index<12);
	world_normal = box.rotate(getBoxLocalEdgeNormals()[edge_index]);
}

void Gu::computeOBBPoints(PxVec3* PX_RESTRICT pts, const PxVec3& center, const PxVec3& extents, const PxVec3& base0, const PxVec3& base1, const PxVec3& base2)
{
	PX_ASSERT(pts);

	// "Rotated extents"
	const PxVec3 axis0 = base0 * extents.x;
	const PxVec3 axis1 = base1 * extents.y;
	const PxVec3 axis2 = base2 * extents.z;

	//     7+------+6			0 = ---
	//     /|     /|			1 = +--
	//    / |    / |			2 = ++-
	//   / 4+---/--+5			3 = -+-
	// 3+------+2 /    y   z	4 = --+
	//  | /    | /     |  /		5 = +-+
	//  |/     |/      |/		6 = +++
	// 0+------+1      *---x	7 = -++

	// Original code: 24 vector ops
	/*	pts[0] = box.center - Axis0 - Axis1 - Axis2;
	pts[1] = box.center + Axis0 - Axis1 - Axis2;
	pts[2] = box.center + Axis0 + Axis1 - Axis2;
	pts[3] = box.center - Axis0 + Axis1 - Axis2;
	pts[4] = box.center - Axis0 - Axis1 + Axis2;
	pts[5] = box.center + Axis0 - Axis1 + Axis2;
	pts[6] = box.center + Axis0 + Axis1 + Axis2;
	pts[7] = box.center - Axis0 + Axis1 + Axis2;*/

	// Rewritten: 12 vector ops
	pts[0] = pts[3] = pts[4] = pts[7] = center - axis0;
	pts[1] = pts[2] = pts[5] = pts[6] = center + axis0;

	PxVec3 tmp = axis1 + axis2;
	pts[0] -= tmp;
	pts[1] -= tmp;
	pts[6] += tmp;
	pts[7] += tmp;

	tmp = axis1 - axis2;
	pts[2] += tmp;
	pts[3] += tmp;
	pts[4] -= tmp;
	pts[5] -= tmp;
}


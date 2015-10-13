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


#include "CmSimd.h"
#include "Opcode.h"
#include "OPC_CommonColliderOverlap.h"

namespace physx
{
namespace Ice
{

#ifndef OPC_SUPPORT_VMX128

//! A dedicated version where the box is constant
PX_FORCE_INLINE Ps::IntBool OBBCollider::TriBoxOverlap(const PxVec3& vert0, const PxVec3& vert1, const PxVec3& vert2)
{
	PxVec3 v0, v1, v2;

	/* Transform them in a common space */
	TransformPoint(v0, vert0, mRModelToBox, mTModelToBox);
	TransformPoint(v1, vert1, mRModelToBox, mTModelToBox);
	TransformPoint(v2, vert2, mRModelToBox, mTModelToBox);

	const PxVec3& extents = mBoxExtents;

	// use separating axis theorem to test overlap between triangle and box 
	// need to test for overlap in these directions: 
	// 1) the {x,y,z}-directions (actually, since we use the AABB of the triangle 
	//    we do not even need to test these) 
	// 2) normal of the triangle 
	// 3) crossproduct(edge from tri, {x,y,z}-directin) 
	//    this gives 3x3=9 more tests 

	// Box center is already in (0,0,0)

	// First, test overlap in the {x,y,z}-directions
	float minimum,maximum;
	// Find minimum, maximum of the triangle in x-direction, and test for overlap in X
	FINDMINMAX(v0.x, v1.x, v2.x, minimum, maximum);
	if(minimum>mBoxExtents.x || maximum<-mBoxExtents.x) return Ps::IntFalse;

	FINDMINMAX(v0.y, v1.y, v2.y, minimum, maximum);
	if(minimum>mBoxExtents.y || maximum<-mBoxExtents.y) return Ps::IntFalse;

	FINDMINMAX(v0.z, v1.z, v2.z, minimum, maximum);
	if(minimum>mBoxExtents.z || maximum<-mBoxExtents.z) return Ps::IntFalse;

	// 2) Test if the box intersects the plane of the triangle
	// compute plane equation of triangle: normal*x+d=0
	// ### could be precomputed since we use the same leaf triangle several times
	const PxVec3 e0 = v1 - v0;
	const PxVec3 e1 = v2 - v1;
	const PxVec3 normal = e0.cross(e1);
	const float d = -normal.dot(v0);
	if(!planeBoxOverlap(normal, d, mBoxExtents)) return Ps::IntFalse;

	// 3) "Class III" tests - here we always do full tests since the box is a primitive (not a BV)
	{
		IMPLEMENT_CLASS3_TESTS
	}
	return Ps::IntTrue;
}
#else
//! A dedicated version where the box is constant
// using vmx128
//TODO: we should try and unify this code with that in the loose primitive test for LSS

PX_FORCE_INLINE Ps::IntBool OBBCollider::TriBoxOverlap(const PxVec3& vert0, const PxVec3& vert1, const PxVec3& vert2)
{
	Cm::PxSimd::Vector4 rot_0 = Cm::PxSimd::load(mRModelToBox[0]);
	Cm::PxSimd::Vector4 rot_1 = Cm::PxSimd::load(mRModelToBox[1]);
	Cm::PxSimd::Vector4 rot_2 = Cm::PxSimd::load(mRModelToBox[2]);

	Cm::PxSimd::Vector4 tran = Cm::PxSimd::load(mTModelToBox);
	Cm::PxSimd::Vector4 extents = Cm::PxSimd::load(mBoxExtents);

	return TriBoxOverlap(vert0, vert1, vert2, rot_0, rot_1, rot_2, tran, extents);
}

PX_FORCE_INLINE Ps::IntBool OBBCollider::TriBoxOverlap(const PxVec3& vert0, const PxVec3& vert1, const PxVec3& vert2,
										const Cm::PxSimd::Vector4 &tran, const Cm::PxSimd::Vector4 &extents)
{
	//transform obb to origin
	Cm::PxSimd::Vector4 v0 = Cm::PxSimd::load(vert0);
	Cm::PxSimd::Vector4 v1 = Cm::PxSimd::load(vert1);
	Cm::PxSimd::Vector4 v2 = Cm::PxSimd::load(vert2);

	//Note this transform is different to the LSS loose version.
	v0 += tran;
	v1 += tran;
	v2 += tran;

	// Test triangle AABB
	Cm::PxSimd::Vector4 triMin = Cm::PxSimd::minimum(v0, Cm::PxSimd::minimum(v1, v2));
	Cm::PxSimd::Vector4 triMax = Cm::PxSimd::maximum(v0, Cm::PxSimd::maximum(v1, v2));

	Cm::PxSimd::Vector4 mask = Cm::PxSimd::or4(Cm::PxSimd::greater(triMin, extents), Cm::PxSimd::greater(-extents, triMax));
	
	return !Cm::PxSimd::anyTrue(mask);
		
}

Ps::IntBool OBBCollider::TriBoxOverlap(const PxVec3& vert0, const PxVec3& vert1, const PxVec3& vert2,
										const Cm::PxSimd::Vector4 &rot_0, const Cm::PxSimd::Vector4 &rot_1, const Cm::PxSimd::Vector4 &rot_2, 
										const Cm::PxSimd::Vector4 &tran, const Cm::PxSimd::Vector4 &extents)
{
	//transform obb to origin
	Cm::PxSimd::Vector4 v0 = Cm::PxSimd::load(vert0);
	Cm::PxSimd::Vector4 v1 = Cm::PxSimd::load(vert1);
	Cm::PxSimd::Vector4 v2 = Cm::PxSimd::load(vert2);

	//Note this transform is different to the LSS loose version.
	v0 = Cm::PxSimd::transform(rot_0, rot_1, rot_2, tran, v0);
	v1 = Cm::PxSimd::transform(rot_0, rot_1, rot_2, tran, v1);
	v2 = Cm::PxSimd::transform(rot_0, rot_1, rot_2, tran, v2);

	Cm::PxSimd::Vector4 minusExtents =  -extents;

	// Test triangle AABB
	Cm::PxSimd::Vector4 triMin = Cm::PxSimd::minimum(v0, Cm::PxSimd::minimum(v1, v2));
	Cm::PxSimd::Vector4 triMax = Cm::PxSimd::maximum(v0, Cm::PxSimd::maximum(v1, v2));

	Cm::PxSimd::Vector4 mask = Cm::PxSimd::or4(Cm::PxSimd::greater(triMin, extents), Cm::PxSimd::greater(minusExtents, triMax));
	if(Cm::PxSimd::anyTrue(mask))
		return Ps::IntFalse;

	// test tri plane.
	Cm::PxSimd::Vector4 edge0 = Cm::PxSimd::subtract(v1, v0);
	Cm::PxSimd::Vector4 edge2 = Cm::PxSimd::subtract(v0, v2);

	Cm::PxSimd::Vector4 normal = Cm::PxSimd::cross(edge2, edge0);
	Cm::PxSimd::Vector4 dist = Cm::PxSimd::dot(normal, v0);
	Cm::PxSimd::Vector4 maxDist = Cm::PxSimd::dot(extents, Cm::PxSimd::abs(normal));


	if(Cm::PxSimd::outBounds4Bool(dist, maxDist))
		return Ps::IntFalse;

	//////// test edge axis
	Cm::PxSimd::Vector4 radius = extents + extents;

	Cm::PxSimd::Vector4 axis, p0, p1, p2, pMin, pMax, axisRadius;

	/*
		y*other.z - z*other.y,
		z*other.x - x*other.z,
		x*other.y - y*other.x

		0 - 0
		0 - e0.z
		e0.y - 0
	*/


	// axis == [1,0,0] x e0 == [0, -e0.z, e0.y]
	// x, y, z, w,    x, y, z, w
	// 0, 1, 2, 3,    4, 5, 6, 7

	OPC_SIMD_PERMUTE(PERMUTE_0W1Z0Y0W, OPC_SIMD_0W, OPC_SIMD_1Z, OPC_SIMD_0Y, OPC_SIMD_0W);

	Cm::PxSimd::Vector4 zero = Cm::PxSimd::zero();
	edge0 = Cm::PxSimd::insertW(edge0, zero);
	Cm::PxSimd::Vector4 minusEdge0 = -edge0;

	
	axis = Cm::PxSimd::permute(edge0, minusEdge0, PERMUTE_0W1Z0Y0W );
	p0 = Cm::PxSimd::dot(v0, axis);
	p2 = Cm::PxSimd::dot(v2, axis);
	axisRadius = Cm::PxSimd::dot(radius, Cm::PxSimd::abs(axis));

	pMin = p0 + p2;
	pMax = Cm::PxSimd::abs(p0-p2)+axisRadius;

	if(Cm::PxSimd::outBounds4Bool(pMin, pMax))
		return Ps::IntFalse;
	

	Cm::PxSimd::Vector4 edge1 = Cm::PxSimd::subtract(v2, v1);
	edge1 = Cm::PxSimd::insertW(edge1, zero);
	Cm::PxSimd::Vector4 minusEdge1 = -edge1;
	
	
	// axis == [1,0,0] x e1 == [0, -e1.z, e1.y]
	axis = Cm::PxSimd::permute(edge1, minusEdge1, PERMUTE_0W1Z0Y0W );
	p0 = Cm::PxSimd::dot(v0, axis);
	p1 = Cm::PxSimd::dot(v1, axis);
	axisRadius = Cm::PxSimd::dot(radius, Cm::PxSimd::abs(axis));
	pMin = p0 + p1;
	pMax = Cm::PxSimd::abs(p0-p1)+axisRadius;

	if(Cm::PxSimd::outBounds4Bool(pMin, pMax))
		return Ps::IntFalse;
	

	edge2 = Cm::PxSimd::insertW(edge2, zero);
	Cm::PxSimd::Vector4 minusEdge2 = -edge2;

	
	// axis == [1,0,0] x e2 == [0, -e2.z, e2.y]
	axis = Cm::PxSimd::permute(edge2, minusEdge2, PERMUTE_0W1Z0Y0W);
	p0 = Cm::PxSimd::dot(v0, axis);
	p1 = Cm::PxSimd::dot(v1, axis);
	axisRadius = Cm::PxSimd::dot(radius, Cm::PxSimd::abs(axis));

	pMin = p0 + p1;
	pMax = Cm::PxSimd::abs(p0-p1)+axisRadius;

	if(Cm::PxSimd::outBounds4Bool(pMin, pMax))
		return Ps::IntFalse;
	

	/*
		y*other.z - z*other.y,
		z*other.x - x*other.z,
		x*other.y - y*other.x

		e0.z - 0
		0 - 0
		0 - e0.x
	*/
	// axis == [0,1,0] x e0 == [e0.z, 0, -e0.x]
	// x, y, z, w,    x, y, z, w
	// 0, 1, 2, 3,    4, 5, 6, 7

	OPC_SIMD_PERMUTE(PERMUTE_0Z0W1X0W, OPC_SIMD_0Z, OPC_SIMD_0W, OPC_SIMD_1X, OPC_SIMD_0W);

	axis = Cm::PxSimd::permute(edge0, minusEdge0, PERMUTE_0Z0W1X0W);
	p0 = Cm::PxSimd::dot(v0, axis);
	p2 = Cm::PxSimd::dot(v2, axis);
	axisRadius = Cm::PxSimd::dot(radius, Cm::PxSimd::abs(axis));
	pMin = p0 + p2;
	pMax = Cm::PxSimd::abs(p0-p2)+axisRadius;

	if(Cm::PxSimd::outBounds4Bool(pMin, pMax))
		return Ps::IntFalse;

	// axis == [0,1,0] x e1 == [e1.z, 0, -e1.x]
	axis = Cm::PxSimd::permute(edge1, minusEdge1, PERMUTE_0Z0W1X0W);
	p0 = Cm::PxSimd::dot(v0, axis);
	p1 = Cm::PxSimd::dot(v1, axis);
	axisRadius = Cm::PxSimd::dot(radius, Cm::PxSimd::abs(axis));

	pMin = p0 + p1;
	pMax = Cm::PxSimd::abs(p0-p1)+axisRadius;

	if(Cm::PxSimd::outBounds4Bool(pMin, pMax))
		return Ps::IntFalse;

	// axis == [0, 1, 0] x e2 == [e2.z, 0, -e2.x]
	axis = Cm::PxSimd::permute(edge2, minusEdge2, PERMUTE_0Z0W1X0W);
	p0 = Cm::PxSimd::dot(v0, axis);
	p1 = Cm::PxSimd::dot(v1, axis);
	axisRadius = Cm::PxSimd::dot(radius, Cm::PxSimd::abs(axis));

	pMin = p0 + p1;
	pMax = Cm::PxSimd::abs(p0-p1)+axisRadius;

	if(Cm::PxSimd::outBounds4Bool(pMin, pMax))
		return Ps::IntFalse;


	/*
		y*other.z - z*other.y,
		z*other.x - x*other.z,
		x*other.y - y*other.x

		0 - e0.y
		e0.x - 0
		0 - 0
	*/

	// axis == [0, 0, 1] x e0 == [-e0.y, e0.x, 0]
	// x, y, z, w,    x, y, z, w
	// 0, 1, 2, 3,    4, 5, 6, 7

	OPC_SIMD_PERMUTE(PERMUTE_1Y0X0W0W, OPC_SIMD_1Y, OPC_SIMD_0X, OPC_SIMD_0W, OPC_SIMD_0W);

	axis = Cm::PxSimd::permute(edge0, minusEdge0, PERMUTE_1Y0X0W0W);
	p0 = Cm::PxSimd::dot(v0, axis);
	p2 = Cm::PxSimd::dot(v2, axis);
	axisRadius = Cm::PxSimd::dot(radius, Cm::PxSimd::abs(axis));

	pMin = p0 + p2;
	pMax = Cm::PxSimd::abs(p0-p2)+axisRadius;

	if(Cm::PxSimd::outBounds4Bool(pMin, pMax))
		return Ps::IntFalse;


	// axis == [0, 0, 1] x e1 == [-e1.y, e1.x, 0]
	axis = Cm::PxSimd::permute(edge1, minusEdge1, PERMUTE_1Y0X0W0W);
	p0 = Cm::PxSimd::dot(v0, axis);
	p1 = Cm::PxSimd::dot(v1, axis);
	axisRadius = Cm::PxSimd::dot(radius, Cm::PxSimd::abs(axis));

	pMin = p0 + p1;
	pMax = Cm::PxSimd::abs(p0-p1)+axisRadius;

	if(Cm::PxSimd::outBounds4Bool(pMin, pMax))
		return Ps::IntFalse;

	// axis == [0, 0, 1] x e2 == [-e2.y, e2.x, 0]
	axis = Cm::PxSimd::permute(edge2, minusEdge2, PERMUTE_1Y0X0W0W);
	p0 = Cm::PxSimd::dot(v0, axis);
	p1 = Cm::PxSimd::dot(v1, axis);
	axisRadius = Cm::PxSimd::dot(radius, Cm::PxSimd::abs(axis));

	pMin = p0 + p1;
	pMax = Cm::PxSimd::abs(p0-p1)+axisRadius;

	if(Cm::PxSimd::outBounds4Bool(pMin, pMax))
		return Ps::IntFalse;

	return Ps::IntTrue;
}

#endif //OPC_SUPPORT_VMX128

} // namespace Ice

}

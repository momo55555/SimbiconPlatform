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


#include "GuIntersectionTriangleTriangle.h"
#include "PxVec3.h"

using namespace physx;

// copied and modified from http://jgt.akpeters.com/papers/Moller97/tritri.html
//
// Triangle/triangle intersection test routine, 
// by Tomas Moller, 1997.
// See article "A Fast Triangle - Triangle Intersection Test", 
// Journal of Graphics Tools, 2(2), 1997
// updated: 2001 - 06 - 20 (added line of intersection)
// 
// if USE_EPSILON_TEST is true then we do a check: 
//         if |dv| < EPSILON then dv = 0.0f;
//    else no check is done (which is less robust)
//
// this edge to edge test is based on Franlin Antonio's gem:
// "Faster Line Segment Intersection", in Graphics Gems III, 
// pp. 199 - 202 */ 

#define USE_EPSILON_TEST 1
#define EPSILON 0.000001f

#define EDGE_EDGE_TEST(v0, u0, u1)									\
	bx = u0[i0] - u1[i0];											\
	by = u0[i1] - u1[i1];											\
	cx = v0[i0] - u0[i0];											\
	cy = v0[i1] - u0[i1];											\
	f = ay*bx - ax*by;												\
	d = by*cx - bx*cy;												\
	if ((f > 0 && d >= 0 && d <= f) || (f < 0 && d <= 0 && d >= f)) \
	{																\
		e = ax*cy - ay*cx;											\
		if (f > 0)													\
		{															\
			if (e >= 0 && e <= f) return true;                      \
		}															\
		else														\
		{															\
			if (e <= 0 && e >= f) return true;                      \
		}															\
	}                                

#define EDGE_AGAINST_TRI_EDGES(v0, v1, u0, u1, u2)					\
{																	\
  PxReal ax, ay, bx, by, cx, cy, e, d, f;							\
  ax = v1[i0] - v0[i0];												\
  ay = v1[i1] - v0[i1];												\
  /* test edge u0, u1 against v0, v1 */          					\
  EDGE_EDGE_TEST(v0, u0, u1);                    					\
  /* test edge u1, u2 against v0, v1 */          					\
  EDGE_EDGE_TEST(v0, u1, u2);                    					\
  /* test edge u2, u1 against v0, v1 */          					\
  EDGE_EDGE_TEST(v0, u2, u0);                    					\
}

#define POINT_IN_TRI(v0, u0, u1, u2)								\
{																	\
	PxReal a, b, c, d0, d1, d2;										\
	/* is T1 completly inside T2? */								\
	/* check if v0 is inside tri(u0, u1, u2) */						\
	a = u1[i1] - u0[i1];											\
	b = -(u1[i0] - u0[i0]);											\
	c = -a*u0[i0] - b*u0[i1];										\
	d0 = a*v0[i0] + b*v0[i1] + c;                   				\
																	\
	a = u2[i1] - u1[i1];											\
	b = -(u2[i0] - u1[i0]);											\
	c = -a*u1[i0] - b*u1[i1];										\
	d1 = a*v0[i0] + b*v0[i1] + c;                   				\
																	\
	a = u0[i1] - u2[i1];											\
	b = -(u0[i0] - u2[i0]);											\
	c = -a*u2[i0] - b*u2[i1];										\
	d2 = a*v0[i0] + b*v0[i1] + c;                   				\
	if (d0*d1 > 0.0)												\
	{																\
		if (d0*d2 > 0.0)											\
			return true;											\
	}																\
}

static bool CoplanarTriangleTriangleTest(const PxVec3& n, 
								    const PxVec3& v0, const PxVec3& v1, const PxVec3& v2, 
								    const PxVec3& u0, const PxVec3& u1, const PxVec3& u2)
{
	PxVec3 a;
	short i0, i1;
	/* first project onto an axis - aligned plane, that maximizes the area */
	/* of the triangles, compute indices: i0, i1. */
	a.x = PxAbs(n.x);
	a.y = PxAbs(n.y);
	a.z = PxAbs(n.z);
	if (a.x > a.y)
	{
		if (a.x > a.z)  
		{
			i0 = 1;      /* a.x is greatest */
			i1 = 2;
		}
		else
		{
			i0 = 0;      /* a.z is greatest */
			i1 = 1;
		}
	}
	else   /* a.x <= a.y */
	{
		if (a.z > a.y)
		{
			i0 = 0;      /* a.z is greatest */
			i1 = 1;                                           
		}
		else
		{
			i0 = 0;      /* a.y is greatest */
			i1 = 2;
		}
	}               

	/* test all edges of triangle 1 against the edges of triangle 2 */
	EDGE_AGAINST_TRI_EDGES(v0, v1, u0, u1, u2);
	EDGE_AGAINST_TRI_EDGES(v1, v2, u0, u1, u2);
	EDGE_AGAINST_TRI_EDGES(v2, v0, u0, u1, u2);

	/* finally, test if tri1 is totally contained in tri2 or vice versa */
	POINT_IN_TRI(v0, u0, u1, u2);
	POINT_IN_TRI(u0, v0, v1, v2);

	return false;
}

PX_INLINE void Isect2(const PxVec3& vtx0, const PxVec3& vtx1, const PxVec3& vtx2, 
					   PxReal vv0, PxReal vv1, PxReal vv2, PxReal d0, PxReal d1, PxReal d2, 
					   PxReal& isect0, PxReal& isect1, PxVec3& isectpoint0, PxVec3& isectpoint1) 
{
	PxReal tmp = d0/(d0 - d1);          
	PxVec3 diff;
	isect0 = vv0 + (vv1 - vv0)*tmp;         
	diff = vtx1 - vtx0;              
	diff *= tmp;               
	isectpoint0 = diff + vtx0;        
	tmp = d0/(d0 - d2);                    
	isect1 = vv0 + (vv2 - vv0)*tmp;          
	diff = vtx2 - vtx0;                   
	diff *= tmp;                 
	isectpoint1 = vtx0 + diff;          
}

PX_INLINE bool ComputeIntervalsIsectline(const PxVec3& vert0, const PxVec3& vert1, const PxVec3& vert2, 
											PxReal vv0, PxReal vv1, PxReal vv2, PxReal d0, PxReal d1, PxReal d2, 
											PxReal d0d1, PxReal d0d2, PxReal& isect0, PxReal& isect1, 
											PxVec3& isectpoint0, PxVec3& isectpoint1)
{
	if (d0d1 > 0.0f)                                        
	{                                                    
		/* here we know that d0d2 <= 0.0 */                  
		/* that is d0, d1 are on the same side, d2 on the other or on the plane */
		Isect2(vert2, vert0, vert1, vv2, vv0, vv1, d2, d0, d1, isect0, isect1, isectpoint0, isectpoint1);
	} 
	else if (d0d2 > 0.0f)                                   
	{                                                   
		/* here we know that d0d1 <= 0.0 */             
		Isect2(vert1, vert0, vert2, vv1, vv0, vv2, d1, d0, d2, isect0, isect1, isectpoint0, isectpoint1);
	}                                                  
	else if (d1*d2 > 0.0f || d0  != 0.0f)   
	{                                   
		/* here we know that d0d1 <= 0.0 or that d0 != 0.0 */
		Isect2(vert0, vert1, vert2, vv0, vv1, vv2, d0, d1, d2, isect0, isect1, isectpoint0, isectpoint1);   
	}                                                  
	else if (d1  != 0.0f)                                  
	{                                               
		Isect2(vert1, vert0, vert2, vv1, vv0, vv2, d1, d0, d2, isect0, isect1, isectpoint0, isectpoint1); 
	}                                         
	else if (d2 != 0.0f)                                  
	{                                                   
		Isect2(vert2, vert0, vert1, vv2, vv0, vv1, d2, d0, d1, isect0, isect1, isectpoint0, isectpoint1);     
	}                                                 
	else                                               
	{                                                   
		/* triangles are coplanar */    
		return true;
	}
	return false;
}

/* sort so that a <= b */
#define SORT(a, b, smallest)	\
             if (a > b)			\
             {					\
               PxReal c;		\
               c = a;			\
               a = b;			\
               b = c;			\
               smallest = 1;	\
             }					\
             else smallest = 0;

bool Gu::intersectTriangleTriangle(const PxVec3* triVerts0, const PxVec3* triVerts1, bool& areCoplanar, PxVec3* isec0, PxVec3* isec1)
{
	const PxVec3& v0 = triVerts0[0];
	const PxVec3& v1 = triVerts0[1];
	const PxVec3& v2 = triVerts0[2];
	const PxVec3& u0 = triVerts1[0];
	const PxVec3& u1 = triVerts1[1];
	const PxVec3& u2 = triVerts1[2];

	PxVec3 e1, e2;
	PxVec3 n1, n2;
	PxReal d1, d2;
	PxReal du0, du1, du2, dv0, dv1, dv2;
	PxReal du0du1, du0du2, dv0dv1, dv0dv2;
	short index;
	PxReal vp0, vp1, vp2;
	PxReal up0, up1, up2;
	PxReal b, c, maximum;
	PxVec3 diff;
	int smallest1, smallest2;
  
	/* compute plane equation of triangle(v0, v1, v2) */
	e1 = v1 - v0;
	e2 = v2 - v0;

	n1 = e1.cross(e2);
	d1 = -n1.dot(v0);
	/* plane equation 1: n1.X + d1 = 0 */

	/* put u0, u1, u2 into plane equation 1 to compute signed distances to the plane*/
	du0 = n1.dot(u0) + d1;
	du1 = n1.dot(u1) + d1;
	du2 = n1.dot(u2) + d1;

  /* coplanarity robustness check */
#if USE_EPSILON_TEST
	if (PxAbs(du0) < EPSILON) du0 = 0.0f;
	if (PxAbs(du1) < EPSILON) du1 = 0.0f;
	if (PxAbs(du2) < EPSILON) du2 = 0.0f;
#endif
	du0du1 = du0*du1;
	du0du2 = du0*du2;

	if (du0du1 > 0.0f && du0du2 > 0.0f) /* same sign on all of them + not equal 0 ? */
		return false;                    /* no intersection occurs */

	/* compute plane of triangle (u0, u1, u2) */
	e1 = u1 - u0;
	e2 = u2 - u0;
	n2 = e1.cross(e2);
	d2 = -n2.dot(u0);
	/* plane equation 2: n2.X + d2 = 0 */

	/* put v0, v1, v2 into plane equation 2 */
	dv0 = n2.dot(v0) + d2;
	dv1 = n2.dot(v1) + d2;
	dv2 = n2.dot(v2) + d2;

#if USE_EPSILON_TEST
	if (PxAbs(dv0) < EPSILON) dv0 = 0.0f;
	if (PxAbs(dv1) < EPSILON) dv1 = 0.0f;
	if (PxAbs(dv2) < EPSILON) dv2 = 0.0f;
#endif

	dv0dv1 = dv0*dv1;
	dv0dv2 = dv0*dv2;
        
	if (dv0dv1 > 0.0f && dv0dv2 > 0.0f) /* same sign on all of them + not equal 0 ? */
		return false;                    /* no intersection occurs */

	/* compute direction of intersection line */
	PxVec3 dVec = n1.cross(n2);

	/* compute and index to the largest component of D */
	maximum = PxAbs(dVec.x);
	index = 0;
	b = PxAbs(dVec.y);
	c = PxAbs(dVec.z);
	if (b > maximum) maximum = b, index = 1;
	if (c > maximum) maximum = c, index = 2;

	/* this is the simplified projection onto L*/
	vp0 = v0[index];
	vp1 = v1[index];
	vp2 = v2[index];

	up0 = u0[index];
	up1 = u1[index];
	up2 = u2[index];

	/* compute interval for triangle 1 */
	PxVec3 isectpointA1, isectpointA2;
	PxReal isect1[2];
	areCoplanar = ComputeIntervalsIsectline(v0, v1, v2, vp0, vp1, vp2, dv0, dv1, dv2, dv0dv1, dv0dv2, 
						isect1[0], isect1[1], isectpointA1, isectpointA2);
	
	if (areCoplanar) 
		return CoplanarTriangleTriangleTest(n1, v0, v1, v2, u0, u1, u2);     

	/* compute interval for triangle 2 */
	PxVec3 isectpointB1, isectpointB2;
	PxReal isect2[2];
	ComputeIntervalsIsectline(u0, u1, u2, up0, up1, up2, du0, du1, du2, du0du1, du0du2, 
			isect2[0], isect2[1], isectpointB1, isectpointB2);

	SORT(isect1[0], isect1[1], smallest1);
	SORT(isect2[0], isect2[1], smallest2);

	if (isect1[1] < isect2[0] || isect2[1] < isect1[0]) 
		return false;

	/* at this point, we know that the triangles intersect */
	PX_ASSERT((isec0 != NULL) == (isec1 != NULL));

	if (isec0 && isec1)
	{
		if (isect2[0] < isect1[0])
		{
			if (smallest1 == 0) { *isec0 = isectpointA1; }
			else				{ *isec0 = isectpointA2; }

		if (isect2[1] < isect1[1])
		{
			if (smallest2 == 0) { *isec1 = isectpointB2; }
			else				{ *isec1 = isectpointB1; }
		}
		else
		{
			if (smallest1 == 0) { *isec1 = isectpointA2; }
			else				{ *isec1 = isectpointA1; }
		}
		}
		else
		{
			if (smallest2 == 0) { *isec0 = isectpointB1; }
			else				{ *isec0 = isectpointB2; }

			if (isect2[1] > isect1[1])
			{
				if (smallest1 == 0) { *isec1 = isectpointA2; }
				else				{ *isec1 = isectpointA1; }      
			}
			else
			{
				if (smallest2 == 0) { *isec1 = isectpointB2; }
				else				{ *isec1 = isectpointB1; } 
			}
		}
	}
	return true;
}

bool Gu::intersectTriangleTriangle(const Ps::aos::Vec3VArg a0, const Ps::aos::Vec3VArg b0, const Ps::aos::Vec3VArg c0,
												const Ps::aos::Vec3VArg a1, const Ps::aos::Vec3VArg b1, const Ps::aos::Vec3VArg c1,
												Ps::aos::BoolV& areCoplanar, Ps::aos::Vec3V& isec0, Ps::aos::Vec3V& isec1)
{
	using namespace Ps::aos;

	const Vec3V zero = V3Zero();
	const Vec3V ab1 = V3Sub(b1, a1);
	const Vec3V ac1 = V3Sub(c1, a1);
	const Vec3V n1 = V3Cross(ab1, ac1);
	const FloatV d1 = FNeg(V3Dot(n1, a1));

	const Vec3V ab0 = V3Sub(b0, a0);
	const Vec3V ac0 = V3Sub(c0, a0);
	const Vec3V n0  = V3Cross(ab0, ac0);
	const FloatV d0 = FNeg(V3Dot(n0, a0));

	//test whether all the points of triange0 in one side of triangle1's plane
	const Vec3V tDist0 = V3Merge(V3Dot(n1, a0),V3Dot(n1, b0),V3Dot(n1, c0));
	const Vec3V signDist0 = V3Add(tDist0, d1);
	const BoolV con00 = BAllTrue3(V3IsGrtr(signDist0, zero));
	const BoolV con01 = BAllTrue3(V3IsGrtr(zero, signDist0));
	const BoolV con02 = BAllTrue3(V3IsEq(signDist0, zero));
	const BoolV con03 =BOr(con00, con01);


	//test whether all the points of triange1 in one side of triangle0's plane
	const Vec3V tDist1 = V3Merge(V3Dot(n0, a1),V3Dot(n0, b1),V3Dot(n0, c1));
	const Vec3V signDist1 = V3Add(tDist1, d0);
	const BoolV con10 = BAllTrue3(V3IsGrtr(signDist1, zero));
	const BoolV con11 = BAllTrue3(V3IsGrtr(zero, signDist1));
	const BoolV con12 = BAllTrue3(V3IsEq(signDist1, zero));
	const BoolV con13 = BOr(con11, con10);

	areCoplanar = BOr(con12, con02);

	const BoolV con = BOr(BOr(con12, con02), BOr(con13, con03));

	//PxU32 bSamePlaneOrNotIntersect;
	//Store_From_BoolV(con, &bSamePlaneOrNotIntersect);

	//if(bSamePlaneOrNotIntersect)
	//{
	//	//on the same plane or not intersect, return false
	//	return BFFFF();
	//}

	if(BAllEq(con, BTTTT()))//on the same place but not intersect
	{
		return false;
	}

	const Vec3V d = V3Cross(n0, n1);

	//make sure v00, v02 on one side, v01 on the other
	const FloatV sd00 = V3GetX(signDist0);
	const FloatV sd01 = V3GetY(signDist0);
	const FloatV sd02 = V3GetZ(signDist0);
	const Vec3V s0 = V3Scale(signDist0, sd00);
	const BoolV sCon0 = V3IsGrtrOrEq(s0, zero );
	const BoolV sy0 = BGetY(sCon0);
	const BoolV sz0 = BGetZ(sCon0);
	const BoolV bCon00 = BAnd(sy0, BNot(sz0));
	const BoolV bCon01 = BAnd(BNot(sy0), sz0);
	const BoolV bCon02 = BOr(bCon00, bCon01);
	const Vec3V v00 = V3Sel(bCon02, a0, b0);
	const Vec3V v01 = V3Sel(bCon00, c0, V3Sel(bCon01, b0, a0));
	const Vec3V v02 = V3Sel(bCon00, b0, c0);
	const FloatV d00 = FSel(bCon02, sd00, sd01);
	const FloatV d01 = FSel(bCon00, sd02, FSel(bCon01, sd01, sd00));
	const FloatV d02 = FSel(bCon00, sd01, sd02);

	//make sure v10, v12 on one side, v11 on the other
	const FloatV sd10 = V3GetX(signDist1);
	const FloatV sd11 = V3GetY(signDist1);
	const FloatV sd12 = V3GetZ(signDist1);
	const Vec3V s1 = V3Scale(signDist1, sd10);
	const BoolV sCon1 = V3IsGrtrOrEq(s1, zero );
	const BoolV sy1 = BGetY(sCon1);
	const BoolV sz1 = BGetZ(sCon1);
	const BoolV bCon10 = BAnd(sy1, BNot(sz1));
	const BoolV bCon11 = BAnd(BNot(sy1), sz1);
	const BoolV bCon12 = BOr(bCon00, bCon01);
	const Vec3V v10 = V3Sel(bCon12, a1, b1);
	const Vec3V v11 = V3Sel(bCon10, c1, V3Sel(bCon11, b1, a1));
	const Vec3V v12 = V3Sel(bCon10, b1, c1);
	const FloatV d10 = FSel(bCon12, sd10, sd11);
	const FloatV d11 = FSel(bCon10, sd12, FSel(bCon11, sd11, sd10));
	const FloatV d12 = FSel(bCon10, sd11, sd12);

	
	//calculate the intersect interval
	const FloatV pv00 = V3Dot(d, v00);
	const FloatV pv01 = V3Dot(d, v01);
	const FloatV pv02 = V3Dot(d, v02);
	const FloatV pv001 = FSub(pv01, pv00);
	const FloatV pv021 = FSub(pv01, pv02);
	const FloatV scale00 = FDiv(d00, FSub(d00, d11));
	const FloatV scale01 = FDiv(d02, FSub(d02, d11));
	const FloatV t00 = FMulAdd(pv001, scale00, pv00);
	const FloatV t01 = FMulAdd(pv021, scale01, pv02);
	const Vec3V ip00 = V3MulAdd(V3Sub(v01, v00), t00, v00);
	const Vec3V ip01 = V3MulAdd(V3Sub(v01, v02), t01, v02);

	const FloatV pv10 = V3Dot(d, v10);
	const FloatV pv11 = V3Dot(d, v11);
	const FloatV pv12 = V3Dot(d, v12);
	const FloatV pv101 = FSub(pv11, pv10);
	const FloatV pv121 = FSub(pv11, pv12);
	const FloatV scale10 = FDiv(d10, FSub(d10, d11));
	const FloatV scale11 = FDiv(d12, FSub(d12, d11));
	const FloatV t10 = FMulAdd(pv101, scale10, pv10);
	const FloatV t11 = FMulAdd(pv121, scale11, pv12);
	const Vec3V ip10 = V3MulAdd(V3Sub(v11, v10), t10, v10);
	const Vec3V ip11 = V3MulAdd(V3Sub(v11, v12), t11, v12);

	const BoolV tCon0 = FIsGrtr(t01, t00);
	const FloatV tMin0 = FSel(tCon0, t00, t01);
	const FloatV tMax0 = FSel(tCon0, t00, t01);
	const Vec3V ipp00 = V3Sel(tCon0, ip00, ip01);
	const Vec3V ipp01 = V3Sel(tCon0, ip01, ip00);

	const BoolV tCon1 = FIsGrtr(t11, t10);
	const FloatV tMin1 = FSel(tCon1, t10, t11);
	const FloatV tMax1 = FSel(tCon1, t10, t11);
	const Vec3V ipp10 = V3Sel(tCon1, ip10, ip11);
	const Vec3V ipp11 = V3Sel(tCon1, ip11, ip00);


	const BoolV iCon0 = FIsGrtr(tMin1, tMax0); //not overlap
	const BoolV iCon1 = FIsGrtr(tMin0, tMax1); //not overlap

	const BoolV iCon2 = FIsGrtr(tMin1, tMin0);
	isec0 = V3Sel(iCon2, ip10, ip00);
	isec1 = V3Sel(iCon2, ip01, ip11);
	
	//return BAnd(BNot(iCon0), BNot(iCon1)); 

	const BoolV bCon = BAnd(BNot(iCon0), BNot(iCon1)); 
	return BAllEq(bCon, BTTTT())==1;
}

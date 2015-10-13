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


#ifndef PX_GJK_H
#define PX_GJK_H

#include "CmPhysXCommon.h"
#include "PsVecMath.h"
#include "ConvexSupportTable.h"
#include "EPA.h"
#include "PsFPU.h"

namespace physx
{
namespace Gu
{

	PX_NOALIAS Ps::aos::Vec3V GJKCPairDoSimplex(Ps::aos::Vec3V* PX_RESTRICT Q, Ps::aos::Vec3V* PX_RESTRICT A, Ps::aos::Vec3V* PX_RESTRICT B, PxU32& size, Ps::aos::Vec3V& closestA, Ps::aos::Vec3V& closestB);

	class EPA;
	class ConvexV;

#define GJKSTATUS PxU32

#define GJK_NON_INTERSECT 0 
#define GJK_MARGIN 1
#define GJK_DEPENETRATION 2
#define GJK_CONTACT 3
#define GJK_UNDEFINED 4

	template<class ConvexA, class ConvexB>
	GJKSTATUS GJKPenetrationNoMargin(const ConvexA& a, const ConvexB& b, Ps::aos::Vec3V& contactA, Ps::aos::Vec3V& contactB, Ps::aos::Vec3V& normal, Ps::aos::FloatV& penetrationDepth);

	PX_NOALIAS Ps::aos::Vec3V closestPtPointTriangle(const Ps::aos::Vec3V a, const Ps::aos::Vec3V b, const Ps::aos::Vec3V c, Ps::aos::Vec3V* PX_RESTRICT Q, Ps::aos::Vec3V* PX_RESTRICT A, Ps::aos::Vec3V* PX_RESTRICT B, PxU32& size, Ps::aos::Vec3V& closestA, Ps::aos::Vec3V& closestB);

	PX_NOALIAS Ps::aos::Vec3V closestPtPointTetrahedron(Ps::aos::Vec3V* PX_RESTRICT Q, Ps::aos::Vec3V* PX_RESTRICT A, Ps::aos::Vec3V* PX_RESTRICT B, PxU32& size, Ps::aos::Vec3V& closestA, Ps::aos::Vec3V& closestB);

	PX_NOALIAS PX_FORCE_INLINE Ps::aos::Vec3V closestPtPointSegment(Ps::aos::Vec3V* PX_RESTRICT Q, Ps::aos::Vec3V* PX_RESTRICT A, Ps::aos::Vec3V* PX_RESTRICT B, PxU32& size, Ps::aos::Vec3V& closestA, Ps::aos::Vec3V& closestB)
	{
		using namespace Ps::aos;
		const Vec3V a = Q[0];
		const Vec3V b = Q[1];

		const BoolV bTrue = BTTTT();
		const Vec3V origin = V3Zero();
		const FloatV zero = FZero();
		const FloatV one = FOne();

		//Test degenerated case
		const Vec3V ab = V3Sub(b, a);
		const FloatV denom = V3Dot(ab, ab);
		const Vec3V ap = V3Sub(origin, a);
		const FloatV nom = V3Dot(ap, ab);
		const BoolV con = FIsEq(denom, zero);
		const Vec3V v = V3Sub(A[1], A[0]);
		const Vec3V w = V3Sub(B[1], B[0]);
		const FloatV tValue = FClamp(FMul(nom, FRecip(denom)), zero, one);
		const FloatV t = FSel(con, zero, tValue);
		//TODO - can we get rid of this branch? The problem is size, which isn't a vector!
		if(BAllEq(con, bTrue))
		{
			size = 1;
			closestA = A[0];
			closestB = B[0];
			return V3Sub(A[0], B[0]);
		}
		
		const Vec3V tempClosestA = V3ScaleAdd(v, t, A[0]);
		const Vec3V tempClosestB = V3ScaleAdd(w, t, B[0]);
		closestA = tempClosestA;
		closestB = tempClosestB;
		return V3Sub(tempClosestA, tempClosestB);
	}

	PX_NOALIAS PX_FORCE_INLINE Ps::aos::Vec3V closestPtPointSegment(const Ps::aos::Vec3VArg Q0, const Ps::aos::Vec3VArg Q1, const Ps::aos::Vec3VArg A0, const Ps::aos::Vec3VArg A1,
		const Ps::aos::Vec3VArg B0, const Ps::aos::Vec3VArg B1, PxU32& size, Ps::aos::Vec3V& closestA, Ps::aos::Vec3V& closestB)
	{
		using namespace Ps::aos;
		const Vec3V a = Q0;
		const Vec3V b = Q1;

		const BoolV bTrue = BTTTT();
		const Vec3V origin = V3Zero();
		const FloatV zero = FZero();
		const FloatV one = FOne();

		//Test degenerated case
		const Vec3V ab = V3Sub(b, a);
		const FloatV denom = V3Dot(ab, ab);
		const Vec3V ap = V3Sub(origin, a);
		const FloatV nom = V3Dot(ap, ab);
		const BoolV con = FIsEq(denom, zero);
		const Vec3V v = V3Sub(A1, A0);
		const Vec3V w = V3Sub(B1, B0);
		const FloatV tValue = FClamp(FMul(nom, FRecip(denom)), zero, one);
		const FloatV t = FSel(con, zero, tValue);
		//TODO - can we get rid of this branch? The problem is size, which isn't a vector!
		if(BAllEq(con, bTrue))
		{
			size = 1;
			closestA = A0;
			closestB = B0;
			return V3Sub(A0, B0);
		}
		
		const Vec3V tempClosestA = V3ScaleAdd(v, t, A0);
		const Vec3V tempClosestB = V3ScaleAdd(w, t, B0);
		closestA = tempClosestA;
		closestB = tempClosestB;
		return V3Sub(tempClosestA, tempClosestB);
	}



	PX_FORCE_INLINE PX_NOALIAS Ps::aos::Vec3V closestPtPointTriangleInline(const Ps::aos::Vec3V a, const Ps::aos::Vec3V b, const Ps::aos::Vec3V c, Ps::aos::Vec3V* PX_RESTRICT Q, Ps::aos::Vec3V* PX_RESTRICT A, Ps::aos::Vec3V* PX_RESTRICT B, PxU32& size, Ps::aos::Vec3V& closestA, Ps::aos::Vec3V& closestB)
	{
		using namespace Ps::aos;

		const FloatV zero = FZero();
		const BoolV bTrue = BTTTT();
		const FloatV eps = FloatV_From_F32(0.002f);//eps should be 0.001f, (2*eps)*(2*eps)
		const FloatV eps2 = FMul(eps, eps);
		
		////const Vec3V zero = V3Zero();
		//const Vec3V a = Q[0];
		//const Vec3V b = Q[1];
		//const Vec3V c = Q[2];

		//const FloatV half = FloatV_From_F32(0.5f);
		

		const Vec3V ab = V3Sub(b, a);
		const Vec3V ac = V3Sub(c, a);
		//const Vec3V bc = V3Sub(c, b);

		const Vec3V ap = V3Neg(a);
		const Vec3V bp = V3Neg(b);
		const Vec3V cp = V3Neg(c);

		

		const FloatV d1 = V3Dot(ab, ap); //  snom
		const FloatV d2 = V3Dot(ac, ap); //  tnom
		const FloatV d3 = V3Dot(ab, bp); // -sdenom
		const FloatV d4 = V3Dot(ac, bp); //  unom = d4 - d3
		const FloatV d5 = V3Dot(ab, cp); //  udenom = d5 - d6
		const FloatV d6 = V3Dot(ac, cp); // -tdenom
		const FloatV unom = FSub(d4, d3);
		const FloatV udenom = FSub(d5, d6);

		//Detect degenerated case, calculate the triangle area
		const FloatV sqAb = V3Dot(ab, ab);
		const FloatV sqAc = V3Dot(ac, ac);
		const FloatV abac = V3Dot(ab, ac);
		
		const FloatV v = FSub(FMul(sqAb, sqAc), FMul(abac, abac));
		const FloatV va = FSub(FMul(d3, d6), FMul(d5, d4));//edge region of BC
		const FloatV vb = FSub(FMul(d5, d2), FMul(d1, d6));//edge region of AC
		const FloatV vc = FSub(FMul(d1, d4), FMul(d3, d2));//edge region of AB

		//const FloatV v = FNegMulSub(abac, abac, FMul(sqAb, sqAc));
		//const FloatV va = FNegMulSub(d5, d4, FMul(d3, d6));//edge region of BC
		//const FloatV vb = FNegMulSub(d1, d6, FMul(d5, d2));//edge region of AC
		//const FloatV vc = FNegMulSub(d3, d2, FMul(d1, d4));//edge region of AB
	
		if(FAllGrtr(eps2, v))
		{
			size = 2;
			//if(FAllEq(sqAb, zero))
			
			const BoolV b = FIsEq(sqAb, zero);
			//a and b are the same
		/*	const Vec3V Q1 = V3Sel(b, Q[2], Q[1]);
			const Vec3V A1 = V3Sel(b, A[2], A[1]);
			const Vec3V B1 = V3Sel(b, B[2], B[1]);

			Q[1] = Q1;
			A[1] = A1;
			B[1] = B1;

			return closestPtPointSegment(Q[0], Q1, A[0], A1, B[0], B1, size, closestA, closestB);*/

			Q[1] = V3Sel(b, Q[2], Q[1]);
			A[1] = V3Sel(b, A[2], A[1]);
			B[1] = V3Sel(b, B[2], B[1]);
			
			return closestPtPointSegment(Q, A, B, size, closestA, closestB);
		}

		
		//check if p in vertex region outside a
		const BoolV con00 = FIsGrtrOrEq(zero, d1); // snom <= 0
		const BoolV con01 = FIsGrtrOrEq(zero, d2); // tnom <= 0
		const BoolV con0 = BAnd(con00, con01); // vertex region a
		//const Vec3V closestA0 = A[0];
		//const Vec3V closestB0 = B[0];

		//check if p in vertex region outside b
		const BoolV con10 = FIsGrtrOrEq(d3, zero);
		const BoolV con11 = FIsGrtrOrEq(d3, d4);
		const BoolV con1 = BAnd(con10, con11); // vertex region b

		const BoolV con20 = FIsGrtrOrEq(d6, zero);
		const BoolV con21 = FIsGrtrOrEq(d6, d5); 
		const BoolV con2 = BAnd(con20, con21); // vertex region c

		const BoolV bCondition = BOr(con0, BOr(con1, con2));
		if(BAllEq(bCondition, bTrue))
		{
			const Vec3V tempClosestA = V3Sel(con0, A[0], V3Sel(con1, A[1], A[2]));
			const Vec3V tempClosestB = V3Sel(con0, B[0], V3Sel(con1, B[1], B[2]));
			closestA = tempClosestA;
			closestB = tempClosestB;
			return  V3Sub(tempClosestA, tempClosestB);
		}		

		//check if p in edge region of AB
		const BoolV con30 = FIsGrtrOrEq(zero, vc);
		const BoolV con31 = FIsGrtrOrEq(d1, zero);
		const BoolV con32 = FIsGrtrOrEq(zero, d3);
		const BoolV con3 = BAnd(con30, BAnd(con31, con32));

		//check if p in edge region of BC
		const BoolV con40 = FIsGrtrOrEq(zero, va);
		const BoolV con41 = FIsGrtrOrEq(d4, d3);
		const BoolV con42 = FIsGrtrOrEq(d5, d6);
		const BoolV con4 = BAnd(con40, BAnd(con41, con42)); 

		const BoolV con50 = FIsGrtrOrEq(zero, vb);
		const BoolV con51 = FIsGrtrOrEq(d2, zero);
		const BoolV con52 = FIsGrtrOrEq(zero, d6);
		const BoolV con5 = BAnd(con50, BAnd(con51, con52));
		

		const FloatV toRecipA = FSub(d1, d3);
		const FloatV toRecipB = FAdd(unom, udenom);
		const FloatV toRecipC = FSub(d2, d6);
		const FloatV toRecipD = FAdd(va, FAdd(vb, vc));
		

		const Vec4V tmp = V4Merge(toRecipA, toRecipB, toRecipC, toRecipD);
		const Vec4V recipTmp = V4Recip(tmp);

		const FloatV sScale = FMul(d1, V4GetX(recipTmp));
		const FloatV uScale = FMul(unom, V4GetY(recipTmp));
		const FloatV tScale = FMul(d2, V4GetZ(recipTmp));
		const FloatV denom = V4GetW(recipTmp);

		//TODO - can we roll these loops into 1???
		//const Vec3V closest3 = V3Add(a, V3Scale(ab, sScale));
		const BoolV bOr1 = BOr(con3, BOr(con4, con5));

		if(BAllEq(bOr1, bTrue))
		{
			const Vec3V A1 = V3Sel(con3, A[1], A[2]);
			const Vec3V B1 = V3Sel(con3, B[1], B[2]);
			const Vec3V A0 = V3Sel(con3, A[0], V3Sel(con4, A[1], A[0]));
			const Vec3V B0 = V3Sel(con3, B[0], V3Sel(con4, B[1], B[0]));

			const Vec3V v = V3Sub(A1, A0);
			const Vec3V w = V3Sub(B1, B0);

			const FloatV scale = FSel(con3, sScale, FSel(con4, uScale, tScale));

			const Vec3V tempClosestA = V3ScaleAdd(v, scale, A0);
			const Vec3V tempClosestB = V3ScaleAdd(w, scale, B0);
			closestA = tempClosestA;
			closestB = tempClosestB;
			return V3Sub(tempClosestA, tempClosestB);
		}
		//P must project inside face region. Compute Q using Barycentric coordinates
		
	
		const Vec3V v0 = V3Sub(A[1], A[0]);
		const Vec3V v1 = V3Sub(A[2], A[0]);
		const Vec3V w0 = V3Sub(B[1], B[0]);
		const Vec3V w1 = V3Sub(B[2], B[0]);

		const FloatV t = FMul(vb, denom);
		const FloatV w = FMul(vc, denom);
		const Vec3V vA1 = V3Scale(v1, w);
		const Vec3V vB1 = V3Scale(w1, w);
		const Vec3V tempClosestA = V3Add(A[0], V3ScaleAdd(v0, t, vA1));
		const Vec3V tempClosestB = V3Add(B[0], V3ScaleAdd(w0, t, vB1));
		closestA = tempClosestA;
		closestB = tempClosestB;
		return V3Sub(tempClosestA, tempClosestB);
	}

	PX_NOALIAS PX_FORCE_INLINE PxU32 PointOutsideOfPlane(const Ps::aos::Vec3VArg a, const Ps::aos::Vec3VArg b, const Ps::aos::Vec3VArg c, const Ps::aos::Vec3VArg d)
	{
		using namespace Ps::aos;
		const FloatV zero = FZero();
		const BoolV bTrue = BTTTT();
		const Vec3V ab = V3Sub(b, a);
		const Vec3V ac = V3Sub(c, a);
		const Vec3V v = V3Cross(ab, ac);
		const FloatV signa = V3Dot(v, a);
		const FloatV signd = V3Dot(v, d);
		const BoolV con = FIsGrtrOrEq(FMul(signa, signd), zero);//same side, outside of the plane
		return BAllEq(con, bTrue);
	}

	//static const PxU32 indexStrategy[4][4] = {{0,1,2,3}, {0,2,3,1},{0,3,1,2},{1,3,2,0}};

	//PX_FORCE_INLINE PX_NOALIAS Ps::aos::Vec3V closestPtPointTetrahedron(Ps::aos::Vec3V* PX_RESTRICT Q, Ps::aos::Vec3V* PX_RESTRICT A, Ps::aos::Vec3V* PX_RESTRICT B, PxU32& size, Ps::aos::Vec3V& closestA, Ps::aos::Vec3V& closestB)
	//{
	//	using namespace Ps::aos;
	//	//const Vec3V origin = V3Zero();
	//	Vec3V tempClosestA = closestA;
	//	Vec3V tempClosestB = closestB;
	//	PxU32 tempSize = size;
	//	
	//	const FloatV eps = FloatV_From_F32(0.001f);	
	//	
	//	FloatV bestSqDist = FloatV_From_F32(PX_MAX_REAL);
	//	const Vec3V a = Q[0];
	//	const Vec3V b = Q[1];
	//	const Vec3V c = Q[2];
	//	const Vec3V d = Q[3];
	//	const BoolV bTrue = BTTTT();

	//	Vec3V _Q[] = {Q[0], Q[1], Q[2], Q[3]};
	//	Vec3V _A[] = {A[0], A[1], A[2], A[3]};
	//	Vec3V _B[] = {B[0], B[1], B[2], B[3]};
	//	

	//	//test for degenerate case
	//	const Vec3V db = V3Sub(b, d);
	//	const Vec3V dc = V3Sub(c, d);
	//	const Vec3V da = V3Sub(a, d);
	//	const FloatV volume = FAbs(V3Dot(da, V3Cross(db, dc)));
	//	if(FAllGrtr(eps, volume))
	//	{
	//		
	//		Vec3V Qt[3] = {_Q[0], _Q[1], _Q[3]};
	//		Vec3V At[3] = {_A[0], _A[1], _A[3]};
	//		Vec3V Bt[3] = {_B[0], _B[1], _B[3]};
	//		PxU32 _size = 3;
	//		Vec3V closestPtA, closestPtB;
	//		Vec3V result = closestPtPointTriangleInline(_Q[0], _Q[1], _Q[3], Qt, At, Bt, _size, closestPtA, closestPtB);

	//		Q[0] = Qt[0]; Q[1] = Qt[1]; Q[2] = Qt[2];
	//		A[0] = At[0]; A[1] = At[1]; A[2] = At[2];
	//		B[0] = Bt[0]; B[1] = Bt[1]; B[2] = Bt[2];
	//		size = _size;
	//		closestA = closestPtA;
	//		closestB = closestPtB;
	//		return result;
	//	}

	//	Vec3V result = V3Zero();

	//	

	//	for(PxU32 a = 0; a < 4; ++a)
	//	{
	//		if(PointOutsideOfPlane(_Q[indexStrategy[a][0]],_Q[indexStrategy[a][1]],_Q[indexStrategy[a][2]],_Q[indexStrategy[a][3]]))
	//		{
	//			Vec3V Qt[3] = {_Q[indexStrategy[a][0]], _Q[indexStrategy[a][1]], _Q[indexStrategy[a][2]]};
	//			Vec3V At[3] = {_A[indexStrategy[a][0]], _A[indexStrategy[a][1]], _A[indexStrategy[a][2]]};
	//			Vec3V Bt[3] = {_B[indexStrategy[a][0]], _B[indexStrategy[a][1]], _B[indexStrategy[a][2]]};
	//			PxU32 _size = 3;
	//			
	//			Vec3V closestPtA, closestPtB;
	//			const Vec3V q = closestPtPointTriangleInline(_Q[indexStrategy[a][0]], _Q[indexStrategy[a][1]], _Q[indexStrategy[a][2]], 
	//				Qt, At, Bt, _size, closestPtA, closestPtB);

	//			const FloatV sqDist = V3Dot(q, q);
	//			const BoolV con = FIsGrtr(bestSqDist, sqDist);
	//			if(BAllEq(con, bTrue))
	//			{
	//				result = q;
	//				bestSqDist = sqDist;
	//				Q[0] = Qt[0]; Q[1] = Qt[1]; Q[2] = Qt[2];
	//				A[0] = At[0]; A[1] = At[1]; A[2] = At[2];
	//				B[0] = Bt[0]; B[1] = Bt[1]; B[2] = Bt[2];
	//				tempSize = _size;
	//				tempClosestA = closestPtA;
	//				tempClosestB = closestPtB;
	//			}
	//		}								
	//	}

	//	closestA = tempClosestA;
	//	closestB = tempClosestB;
	//	size = tempSize;
	//	return result;
	//}



	PX_NOALIAS PX_FORCE_INLINE Ps::aos::Vec3V GJKCPairDoSimplex(Ps::aos::Vec3V* PX_RESTRICT Q, Ps::aos::Vec3V* PX_RESTRICT A, Ps::aos::Vec3V* PX_RESTRICT B, PxU32& size, Ps::aos::Vec3V& closestA, Ps::aos::Vec3V& closestB)
	{
		using namespace Ps::aos;

		//const PxU32 tempSize = size;
		//calculate a closest from origin to the simplex
		switch(size)
		{
		case 1:
			{
				closestA = A[0];
				closestB = B[0];
				return V3Sub(A[0], B[0]);
			}
		case 2:
			{
			return closestPtPointSegment(Q, A, B, size, closestA, closestB);
			}
		case 3:
			return closestPtPointTriangleInline(Q[0], Q[1], Q[2], Q, A, B, size, closestA, closestB);
			//return closestPtPointTriangle(Q[0], Q[1], Q[2], Q, A, B, size, closestA, closestB);
		case 4:
			return closestPtPointTetrahedron(Q, A, B, size, closestA, closestB);
		default:
			PX_ASSERT(0);
		}
		return V3Sub(closestA, closestB);
	}

	PX_NOALIAS PX_FORCE_INLINE Ps::aos::Vec3V GJKCPairDoSimplex(Ps::aos::Vec3V* PX_RESTRICT Q, Ps::aos::Vec3V* PX_RESTRICT A, Ps::aos::Vec3V* PX_RESTRICT B, 
		const Ps::aos::Vec3VArg support, const Ps::aos::Vec3VArg supportA, const Ps::aos::Vec3VArg supportB, PxU32& size, Ps::aos::Vec3V& closestA, Ps::aos::Vec3V& closestB)
	{
		using namespace Ps::aos;

		//const PxU32 tempSize = size;
		//calculate a closest from origin to the simplex
		switch(size)
		{
		case 1:
			{
				closestA = supportA;
				closestB = supportB;
				return support;
			}
		case 2:
			{
			//return closestPtPointSegment(Q, A, B, size, closestA, closestB);
			return closestPtPointSegment(Q[0], support, A[0], supportA, B[0], supportB, size, closestA, closestB);
			}
		case 3:
			return closestPtPointTriangleInline(Q[0], Q[1], support, Q, A, B, size, closestA, closestB);
			//return closestPtPointTriangle(Q[0], Q[1], Q[2], Q, A, B, size, closestA, closestB);
		case 4:
			return closestPtPointTetrahedron(Q, A, B, size, closestA, closestB);
		default:
			PX_ASSERT(0);
		}
		return support;
	}


#ifndef	__SPU__
	/*
		
	*/
	template<class ConvexA, class ConvexB>
	bool GJK(const ConvexA& a, const ConvexB& b, Ps::aos::Vec3V& closestA, Ps::aos::Vec3V& closestB, Ps::aos::FloatV& sqDist, const PxU32 numIteration=10)
	{
		using namespace Ps::aos;
		const Vec3V zeroV = V3Zero();
		const FloatV zero = FZero();
		PxU32 size=1;
		const Vec3V initialSearchDir(V3Sub(a.getCenter(), b.getCenter()));

		const Vec3V initialSupportA(a.support(V3Neg(initialSearchDir)));
		const Vec3V initialSupportB(b.support(initialSearchDir));
		 
		Vec3V Q[4] = {V3Sub(initialSupportA, initialSupportB), zeroV, zeroV, zeroV}; //simplex set
		Vec3V A[4] = {initialSupportA, zeroV, zeroV, zeroV}; //ConvexHull a simplex set
		Vec3V B[4] = {initialSupportB, zeroV, zeroV, zeroV}; //ConvexHull b simplex set
		 

		Vec3V v = Q[0];
		Vec3V supportA = initialSupportA;
		Vec3V supportB = initialSupportB;
		const FloatV eps1 = FloatV_From_F32(0.00005f);
		const FloatV eps2 = FloatV_From_F32(0.000025f);
		Vec3V closA(initialSupportA), closB(initialSupportB);
		FloatV sDist = V3Dot(v, v);
		PxI32 maxIterations = 10;
		
		while(FAllGrtr(sDist,eps2) && (maxIterations --))
		{
			supportA=a.support(V3Neg(v));
			supportB=b.support(v);
			//calculate the support point
			const Vec3V support = V3Sub(supportA, supportB);
			const FloatV signDist = V3Dot(v, support);
			const FloatV tmp0 = FSub(sDist, signDist);
	
			if(FAllGrtr(eps1, tmp0))
			{
				closestA = closA;
				closestB = closB;
				sqDist = sDist;
				return false;
			}

			PX_ASSERT(size < 4);
			A[size]=supportA;
			B[size]=supportB;
			Q[size++]=support;
			//calculate the closest point between two convex hull
			Vec3V tempV = GJKCPairDoSimplex(Q, A, B, size, closA, closB);
			v = tempV;
			sDist = V3Dot(v, v);
		}		
		closestA = closA;
		closestB = closB;
		sqDist = sDist;
		return maxIterations >=0 ;
	}

	template<class ConvexA, class ConvexB>
	bool GJK(const ConvexA& a, const ConvexB& b, Ps::aos::Vec3V* PX_RESTRICT Q, Ps::aos::Vec3V* PX_RESTRICT A, Ps::aos::Vec3V* PX_RESTRICT B, PxU32& size, Ps::aos::Vec3V& closestA, Ps::aos::Vec3V& closestB, Ps::aos::FloatV& sqDist, const PxU32 numIteration=10)
	{
		using namespace Ps::aos;
		const Vec3V zeroV = V3Zero();
		const FloatV zero = FZero();
		size=1;
		const Vec3V initialSearchDir(V3Sub(a.getCenter(), b.getCenter()));

		const Vec3V initialSupportA(a.support(V3Neg(initialSearchDir)));
		const Vec3V initialSupportB(b.support(initialSearchDir));
		 
		A[0] = initialSupportA; //ConvexHull a simplex set
		B[0] = initialSupportB; //ConvexHull b simplex set
		Q[0] = V3Sub(initialSupportA, initialSupportB);
		 

		Vec3V v = Q[0];
		Vec3V supportA = initialSupportA;
		Vec3V supportB = initialSupportB;
		const FloatV eps1 = FloatV_From_F32(0.0001f);
		const FloatV eps2 = FloatV_From_F32(0.000025f);
		Vec3V closA(initialSupportA), closB(initialSupportB);
		FloatV sDist = V3Dot(v, v);
		PxI32 maxIterations = 10;
		FloatV minDist = sDist;
		Vec3V tempClosA(initialSupportA), tempClosB(initialSupportB);
		
		while(FAllGrtr(sDist,eps2))
		{
			supportA=a.support(V3Neg(v));
			supportB=b.support(v);
			//calculate the support point
			const Vec3V support = V3Sub(supportA, supportB);
			const FloatV signDist = V3Dot(v, support);
			const FloatV tmp0 = FSub(sDist, signDist);
	
			if(FAllGrtr(eps1, tmp0))
			{
				closestA = closA;
				closestB = closB;
				sqDist = sDist;
				return false;
			}

			PX_ASSERT(size < 4);
			A[size]=supportA;
			B[size]=supportB;
			Q[size++]=support;
			//calculate the closest point between two convex hull
			v = GJKCPairDoSimplex(Q, A, B, size, closA, closB);
			sDist = V3Dot(v, v);
			if(FAllGrtr(minDist, sDist))
			{
				minDist = sDist;
				tempClosA = closA;
				tempClosB = closB;
			}
			else
			{
				closestA = tempClosA;
				closestB = tempClosB;
				return false;
			}
		}		
		closestA = closA;
		closestB = closB;
		sqDist = sDist;
		return true ;
	}

	template<class ConvexA, class ConvexB>
	bool GJKMargin(const ConvexA& a, const ConvexB& b, Ps::aos::Vec3V& closestA, Ps::aos::Vec3V& closestB, Ps::aos::FloatV& sqDist, const PxU32 numIteration=10)
	{
		using namespace Ps::aos;
		const Vec3V zeroV = V3Zero();
		const FloatV zero = FZero();
		const BoolV bTrue = BTTTT();
		PxU32 size=1;

		const FloatV marginA = a.getMargin();
		const FloatV marginB = b.getMargin();
		const FloatV sumMargin = FAdd(marginA, marginB);
		const FloatV sqMargin = FMul(sumMargin, sumMargin);

		const Vec3V initialSearchDir(V3Sub(a.getCenter(), b.getCenter()));
		const Vec3V initialSupportA(a.supportMargin(V3Neg(initialSearchDir), marginA));
		const Vec3V initialSupportB(b.supportMargin(initialSearchDir, marginB));
		 
		Vec3V Q[4] = {V3Sub(initialSupportA, initialSupportB), zeroV, zeroV, zeroV}; //simplex set
		Vec3V A[4] = {initialSupportA, zeroV, zeroV, zeroV}; //ConvexHull a simplex set
		Vec3V B[4] = {initialSupportB, zeroV, zeroV, zeroV}; //ConvexHull b simplex set
		 

		Vec3V v = Q[0];
		Vec3V supportA = initialSupportA;
		Vec3V supportB = initialSupportB;
		const FloatV eps1 = FloatV_From_F32(0.0001f);
		const FloatV eps2 = FloatV_From_F32(0.0005f);
		Vec3V closA(initialSupportA), closB(initialSupportB);
		FloatV sDist = V3Dot(v, v);
		PxI32 maxIterations = numIteration;

	
		while(FAllGrtr(sDist,eps2) && (maxIterations --))
		{
			//dir = V3Normalise(V3Neg(v));
			supportA=a.supportMargin(V3Neg(v), marginA);
			supportB=b.supportMargin(v, marginB);
			//calculate the support point
			const Vec3V support = V3Sub(supportA, supportB);

			const FloatV vw = V3Dot(v, support);
			const FloatV sqVW = FMul(vw, vw);
			const FloatV tmp = FMul(sDist, sqMargin);
	
			const BoolV con = BAnd(FIsGrtr(vw, zero), FIsGrtr(sqVW, tmp));
			if(BAllEq(con, bTrue))
			{
				//not intersect
				const Vec3V n = V3Normalize(v);
				closestA = V3Sub(closA, V3Scale(n, marginA));
				closestB = V3Add(closB, V3Scale(n, marginB));
				sqDist = sDist;
				return false;
			}

			const FloatV tmp1 = FSub(sDist, vw);
			//const FloatV tmp2 = FMul(eps1, sDist);
			if(FAllGrtrOrEq(eps1, tmp1))
			{
				//intersect in margin
				const Vec3V n = V3Normalize(v);
				closestA = V3Sub(closA, V3Scale(n, marginA));
				closestB = V3Add(closB, V3Scale(n, marginB));
				sqDist = sDist;
				return true;
			}

			PX_ASSERT(size < 4);
			A[size]=supportA;
			B[size]=supportB;
			Q[size++]=support;
			//calculate the closest point between two convex hull
			v = GJKCPairDoSimplex(Q, A, B, size, closA, closB);
			sDist = V3Dot(v, v);
		}

		const Vec3V n = V3Normalize(v);
		closestA = V3Sub(closA, V3Scale(n, marginA));
		closestB = V3Add(closB, V3Scale(n, marginB));
		sqDist = sDist;
		return maxIterations >=0 ;
	}

	template<class ConvexA, class ConvexB>
	bool GJKSeparatingAxis(const ConvexA& a, const ConvexB& b, const PxU32 maxIteration = 10)
	{
		using namespace Ps::aos;
		const FloatV zero = FZero();
		const Vec3V zeroV = V3Zero();
		PxU32 size=1;
		const Vec3V initiDir =V3Sub(a.getCenter(), b.getCenter());
		Vec3V supportA = a.support(V3Neg(initiDir));
		Vec3V supportB = b.support(initiDir);
		Vec3V Q[4] = {V3Sub(supportA, supportB), zeroV, zeroV, zeroV}; //simplex set
		Vec3V A[4] = {supportA, zeroV, zeroV, zeroV}; //ConvexHull a simplex set
		Vec3V B[4] = {supportB, zeroV, zeroV, zeroV}; //ConvexHull b simplex set
		
		
		Vec3V closA(supportA), closB(supportB);
		Vec3V v(Q[0]);
		const FloatV epsilon = FloatV_From_F32(0.1f);
		const FloatV epsilonSq = FMul(epsilon, epsilon);
		FloatV sDist = V3Dot(v, v);
	
		PxI32 numIteration = maxIteration;
		
		while(FAllGrtr(sDist, epsilonSq)&& (numIteration--))
		{
			//const Vec3V dir = V3Normalise(V3Neg(v));
			supportA=a.support(V3Neg(v));
			supportB=b.support(v);
			//calculate the support point
			Vec3V support = V3Sub(supportA, supportB);
			const FloatV signDist = V3Dot(v, support);
			if(FAllGrtr(signDist, zero))
				return false;

			PX_ASSERT(size < 4);
			A[size]=supportA;
			B[size]=supportB;
			Q[size++]=support;
			//calculate the closest point between two convex hull
			v = GJKCPairDoSimplex(Q, A, B, size, closA, closB);

			sDist = V3Dot(v, v);
		}

		return numIteration >= 0;
	}


	template<class ConvexA, class ConvexB>
	GJKSTATUS GJKRecalculateSimplex(const ConvexA& a, const ConvexB& b, Ps::aos::Vec3V& contactA, Ps::aos::Vec3V& contactB, Ps::aos::Vec3V& normal, Ps::aos::FloatV& penetrationDepth)
	{
		using namespace Ps::aos;
		const Vec3V zeroV = V3Zero();
		const FloatV zero = FZero();
		const BoolV bTrue = BTTTT();
		const BoolV bFalse = BFFFF();
		PxU32 size=1;


		const Vec3V _initialSearchDir(V3Sub(a.getCenter(), b.getCenter()));
		const FloatV dotDir = V3Dot(_initialSearchDir, _initialSearchDir);
		const Vec3V initialSearchDir = V3Sel(FIsGrtr(dotDir, zero), _initialSearchDir, V3UnitX());
		const Vec3V initialSupportA(a.support(V3Neg(initialSearchDir)));
		const Vec3V initialSupportB(b.support(initialSearchDir));
		 
		Vec3V v = V3Sub(initialSupportA, initialSupportB);
		
		register Vec3V A[4] = {initialSupportA, zeroV, zeroV, zeroV}; //ConvexHull a simplex set
		register Vec3V B[4] = {initialSupportB, zeroV, zeroV, zeroV}; //ConvexHull b simplex set
		register Vec3V Q[4] = {v, zeroV, zeroV, zeroV}; //simplex set
		 
		Vec3V supportA = initialSupportA;
		Vec3V supportB = initialSupportB;

		const FloatV _marginA = a.getMargin();
		const FloatV _marginB = b.getMargin();
		const FloatV minMargin = FMin(_marginA, _marginB);

		const FloatV eps1 = FloatV_From_F32(0.001f);
		const FloatV eps = FMul(minMargin, FloatV_From_F32(0.001f));
		const FloatV eps2 = FMul(eps, eps);

		Vec3V closA(initialSupportA), closB(initialSupportB);
		FloatV sDist = V3Dot(v, v);
		FloatV minDist = sDist;
		//Vec3V tempClosA(initialSupportA), tempClosB(initialSupportB);

		//GJKSTATUS status = GJK_DEPENETRATION;

		BoolV bNotTerminated = FIsGrtr(sDist, eps2);
		BoolV bCon = bTrue;

		while(BAllEq(bNotTerminated, bTrue))
		{

			minDist = sDist;
		/*	tempClosA = closA;
			tempClosB = closB;*/

			supportA=a.support(V3Neg(v));
			supportB=b.support(v);
			//calculate the support point
			const Vec3V support = V3Sub(supportA, supportB);

			PX_ASSERT(size < 4);
			A[size]=supportA;
			B[size]=supportB;
			Q[size++]=support;

			//calculate the closest point between two convex hull
			//v = GJKCPairDoSimplex(Q, A, B, size, closA, closB);
			v = GJKCPairDoSimplex(Q, A, B, support, supportA, supportB, size, closA, closB);

			sDist = V3Dot(v, v);

			bCon = FIsGrtr(minDist, sDist);
			bNotTerminated = BAnd(FIsGrtr(sDist, eps2), bCon);
		}

		if(BAllEq(bCon, bFalse))
		{
			return GJK_NON_INTERSECT;
		}
	
		EPA epa;
		if(epa.PenetrationDepth<ConvexA, ConvexB>(a, b, Q, A, B, size, closA, closB))
		{
			const Vec3V v1 = V3Sub(closB, closA);    
			const FloatV dist = V3Length(v1);
			normal = V3Scale(v1, FRecip(dist));
			contactA = closA;
			contactB = closB;
			penetrationDepth = FNeg(dist);
			return GJK_CONTACT;
		}
	
		return GJK_NON_INTERSECT;
	}

	template<class ConvexA, class ConvexB>
	GJKSTATUS GJKPenetration(const ConvexA& a, const ConvexB& b, Ps::aos::Vec3V& contactA, Ps::aos::Vec3V& contactB, Ps::aos::Vec3V& normal, Ps::aos::FloatV& penetrationDepth)
	{
		//PIX_PROFILE_ZONE(GJKPenetration);
		using namespace Ps::aos;

		Vec3V A[4]; 
		Vec3V B[4];
		Vec3V Q[4];

		const FloatV zero = FZero();

		const FloatV _marginA = a.getMargin();
		const FloatV _marginB = b.getMargin();
		const Vec3V centerAToCenterB =  V3Sub(b.getCenter(), a.getCenter());		

		const BoolV bHasMarginEqRadius = BOr(a.isMarginEqRadiusV(), b.isMarginEqRadiusV());

		const Vec3V _initialSearchDir = V3Neg(centerAToCenterB);
		Vec3V v = V3Sel(FIsGrtr(V3Dot(_initialSearchDir, _initialSearchDir), zero), _initialSearchDir, V3UnitX());

		const FloatV minMargin = FMin(_marginA, _marginB);
		const FloatV marginA = FSel(bHasMarginEqRadius, _marginA, minMargin);
		const FloatV marginB = FSel(bHasMarginEqRadius, _marginB, minMargin);
		

		const FloatV eps1 = FloatV_From_F32(0.001f);
		const FloatV eps2 = FMul(minMargin, FloatV_From_F32(0.001f));
		const FloatV ratio = FloatV_From_F32(0.05f);
		const Vec3V zeroV = V3Zero();
		const BoolV bTrue = BTTTT();
		const BoolV bFalse = BFFFF();
		PxU32 size=0;

		const FloatV tenthMargin = FMul(minMargin, ratio);
	
		const FloatV sumMargin = FAdd(FAdd(marginA, marginB), tenthMargin);
		const FloatV sqMargin = FMul(sumMargin, sumMargin);

		Vec3V closA = zeroV;
		Vec3V closB = zeroV;
		FloatV sDist = FMax();
		FloatV minDist;
		Vec3V tempClosA;
		Vec3V tempClosB;

		BoolV bNotTerminated = bTrue;
		BoolV bCon = bTrue;
	
		do
		{
			minDist = sDist;
			tempClosA = closA;
			tempClosB = closB;

			const Vec3V nv = V3Neg(v);
			
			const Vec3V supportA=a.supportMargin(nv, marginA, A[size]);
			const Vec3V supportB=b.supportMargin(v, marginB, B[size]);
			//calculate the support point
			const Vec3V support = V3Sub(supportA, supportB);
			Q[size++]=support;

			PX_ASSERT(size <= 4);

			const FloatV tmp = FMul(sDist, sqMargin);//FMulAdd(sDist, sqMargin, eps3);
			const FloatV vw = V3Dot(v, support);
			const FloatV sqVW = FMul(vw, vw);
			
			const BoolV bTmp1 = FIsGrtr(vw, zero);
			const BoolV bTmp2 = FIsGrtr(sqVW, tmp);
			BoolV con = BAnd(bTmp1, bTmp2);

			const FloatV tmp1 = FSub(sDist, vw);
			const BoolV conGrtr = FIsGrtrOrEq(FMul(eps1, sDist), tmp1);

			const BoolV conOrconGrtr(BOr(con, conGrtr));

			if(BAllEq(conOrconGrtr, bTrue))
			{
				size--;
				if(BAllEq(con, bFalse)) //must be true otherwise we wouldn't be in here...
				{
					const FloatV dist = FSqrt(sDist);
					PX_ASSERT(FAllGrtr(dist, FEps()));
					const Vec3V n = V3Scale(v, FRecip(dist));//normalise
					contactA = V3Sub(closA, V3Scale(n, marginA));
					contactB = V3Add(closB, V3Scale(n, marginB));
					penetrationDepth = FSub(sumMargin, dist);
					
					normal = n;
					PX_ASSERT(isFiniteVec3V(normal));
					return GJK_CONTACT;
					
				}
				else
				{
					return GJK_NON_INTERSECT;
				}
			}

			//calculate the closest point between two convex hull
			const Vec3V tempV = GJKCPairDoSimplex(Q, A, B, size, closA, closB);
			//v = V3Sub(closA, closB);
			v = tempV;
			sDist = V3Dot(tempV, tempV);

			bCon = FIsGrtr(minDist, sDist);
			bNotTerminated = BAnd(FIsGrtr(sDist, eps2), bCon);
		}
		while(BAllEq(bNotTerminated, bTrue));

		
		if(BAllEq(bCon, bFalse)) //must be true otherwise we wouldn't be in here...
		{
			//Reset back to older closest point
			closA = tempClosA;
			closB = tempClosB;
			sDist = minDist;
			v = V3Sub(closA, closB);

			const FloatV dist = FSqrt(sDist);
			PX_ASSERT(FAllGrtr(dist, FEps()));
			const Vec3V n = V3Scale(v, FRecip(dist));//normalise
			contactA = V3Sub(closA, V3Scale(n, marginA));
			contactB = V3Add(closB, V3Scale(n, marginB));
			penetrationDepth = FSub(sumMargin, dist);
			normal = n;
			PX_ASSERT(isFiniteVec3V(normal));
			return GJK_CONTACT;
		}
		else
		{
			//return GJKPenetrationNoMargin<ConvexA, ConvexB>(a, b, contactA, contactB, normal, penetrationDepth);
			if( !GJKRecalculateSimplex<ConvexA, ConvexB>(a, b, contactA, contactB, normal, penetrationDepth))
			{
				const Vec3V v = V3Sub(tempClosA, tempClosB);
				const FloatV recipDist = FRsqrt(minDist);
				const FloatV dist = FRecip(minDist);//FSqrt(minDist);
				const Vec3V n = V3Scale(v, recipDist);//normalise
				contactA = V3Sub(tempClosA, V3Scale(n, marginA));
				contactB = V3Add(tempClosB, V3Scale(n, marginB));
				penetrationDepth = FSub(sumMargin, dist);
				normal = n;
				PX_ASSERT(isFiniteVec3V(normal));
			}
			return GJK_CONTACT;
		}
	}

	template<class ConvexA, class ConvexB>
	GJKSTATUS GJKPenetration(const ConvexA& a, const ConvexB& b, const Ps::aos::FloatVArg contactDist, Ps::aos::Vec3V& contactA, Ps::aos::Vec3V& contactB, Ps::aos::Vec3V& normal, Ps::aos::FloatV& penetrationDepth)
	{
		//PIX_PROFILE_ZONE(GJKPenetration);
		using namespace Ps::aos;

		Vec3V A[4]; 
		Vec3V B[4];
		Vec3V Q[4];

		const FloatV zero = FZero();

		const FloatV _marginA = a.getMargin();
		const FloatV _marginB = b.getMargin();
		const Vec3V centerAToCenterB =  V3Sub(b.getCenter(), a.getCenter());		

		const BoolV bHasMarginEqRadius = BOr(a.isMarginEqRadiusV(), b.isMarginEqRadiusV());

		const Vec3V _initialSearchDir = V3Neg(centerAToCenterB);
		Vec3V v = V3Sel(FIsGrtr(V3Dot(_initialSearchDir, _initialSearchDir), zero), _initialSearchDir, V3UnitX());

		const FloatV minMargin = FMin(_marginA, _marginB);
		const FloatV marginA = FSel(bHasMarginEqRadius, _marginA, minMargin);
		const FloatV marginB = FSel(bHasMarginEqRadius, _marginB, minMargin);
		

		const FloatV eps1 = FloatV_From_F32(0.0001f);
		const FloatV eps2 = FMul(minMargin, FloatV_From_F32(0.001f));
		//const FloatV ratio = FloatV_From_F32(0.05f);
		const Vec3V zeroV = V3Zero();
		const BoolV bTrue = BTTTT();
		const BoolV bFalse = BFFFF();
		PxU32 size=0;

		//const FloatV tenthMargin = FMul(minMargin, ratio);
	
		//const FloatV sumMargin = FAdd(FAdd(marginA, marginB), tenthMargin);
		const FloatV sumMargin0 = FAdd(marginA, marginB);
		const FloatV sumMargin = FAdd(sumMargin0, contactDist);
		//const FloatV sumMargin = FAdd(marginA, marginB);
		const FloatV sqMargin = FMul(sumMargin, sumMargin);

		Vec3V closA = zeroV;
		Vec3V closB = zeroV;
		FloatV sDist = FMax();
		FloatV minDist;
		Vec3V tempClosA;
		Vec3V tempClosB;

		BoolV bNotTerminated = bTrue;
		BoolV bCon = bTrue;
	
		//do
		//{
		//	minDist = sDist;
		//	tempClosA = closA;
		//	tempClosB = closB;

		//	const Vec3V nv = V3Neg(v);
		//	
		//	const Vec3V supportA=a.supportMargin(nv, marginA, A[size]);
		//	const Vec3V supportB=b.supportMargin(v, marginB, B[size]);
		//	//calculate the support point
		//	const Vec3V support = V3Sub(supportA, supportB);
		//	Q[size++]=support;

		//	PX_ASSERT(size <= 4);
		//
		//	const FloatV vw = V3Dot(v, support);
		//	const FloatV tmp1 = FSub(sDist, vw);
		//	const BoolV con = FIsGrtrOrEq(FMul(eps1, sDist), tmp1);

		//	//found the closest point
		//	if(BAllEq(con, bTrue))
		//	{
		//		size--;

		//		if(FAllGrtrOrEq(sqMargin, sDist))//intersect in margin
		//		{
		//			const FloatV dist = FSqrt(sDist);
		//			PX_ASSERT(FAllGrtr(dist, FEps()));
		//			const Vec3V n = V3Scale(v, FRecip(dist));//normalise
		//			/*contactA = V3Sub(closA, V3Scale(n, marginA));
		//			contactB = V3Add(closB, V3Scale(n, marginB));*/
		//			contactA = V3NegScaleSub(n, marginA, closA);
		//			contactB = V3ScaleAdd(n, marginB, closB);
		//			normal = n;
		//			PX_ASSERT(isFiniteVec3V(normal));
		//			//penetrationDepth = FSub(sumMargin0, dist);
		//			penetrationDepth = FSub(dist, sumMargin0);
		//			//PX_ASSERT(FAllGrtr(zero, penetrationDepth));
		//			return GJK_CONTACT;
		//		}
		//		else
		//		{
		//			return GJK_NON_INTERSECT;
		//		}
		//	}

		//	//calculate the closest point between two convex hull
		//	v = GJKCPairDoSimplex(Q, A, B, size, closA, closB);

		//	sDist = V3Dot(v, v);

		//	bCon = FIsGrtr(minDist, sDist);
		//	bNotTerminated = BAnd(FIsGrtr(sDist, eps2), bCon);
		//}
		//while(BAllEq(bNotTerminated, bTrue));

		do
		{
			minDist = sDist;
			tempClosA = closA;
			tempClosB = closB;

			const Vec3V nv = V3Neg(v);   
			
			const Vec3V supportA=a.supportMargin(nv, marginA, A[size]);
			const Vec3V supportB=b.supportMargin(v, marginB, B[size]);
			//calculate the support point
			const Vec3V support = V3Sub(supportA, supportB);
			Q[size++]=support;

			PX_ASSERT(size <= 4);

			const FloatV tmp = FMul(sDist, sqMargin);//FMulAdd(sDist, sqMargin, eps3);
			const FloatV vw = V3Dot(v, support);
			const FloatV sqVW = FMul(vw, vw);
			
			const BoolV bTmp1 = FIsGrtr(vw, zero);
			const BoolV bTmp2 = FIsGrtr(sqVW, tmp);
			BoolV con = BAnd(bTmp1, bTmp2);

			const FloatV tmp1 = FSub(sDist, vw);
			const BoolV conGrtr = FIsGrtrOrEq(FMul(eps1, sDist), tmp1);

			const BoolV conOrconGrtr(BOr(con, conGrtr));

			if(BAllEq(conOrconGrtr, bTrue))
			{
				//size--; if you want to get the correct size, this line need to be on
				if(BAllEq(con, bFalse)) //must be true otherwise we wouldn't be in here...
				{
					const FloatV recipDist = FRsqrt(sDist);
					const FloatV dist = FRecip(recipDist);//FSqrt(sDist);
					PX_ASSERT(FAllGrtr(dist, FEps()));
					const Vec3V n = V3Scale(v, recipDist);//normalise
					/*contactA = V3Sub(closA, V3Scale(n, marginA));
					contactB = V3Add(closB, V3Scale(n, marginB));*/
					contactA = V3NegScaleSub(n, marginA, closA);
					contactB = V3ScaleAdd(n, marginB, closB);
					penetrationDepth = FSub(dist, sumMargin0);
					normal = n;
					PX_ASSERT(isFiniteVec3V(normal));
					return GJK_CONTACT;
					
				}
				else
				{
					return GJK_NON_INTERSECT;
				}
			}

			//calculate the closest point between two convex hull

			//v = GJKCPairDoSimplex(Q, A, B, size, closA, closB);
			v = GJKCPairDoSimplex(Q, A, B, support, supportA, supportB, size, closA, closB);

			sDist = V3Dot(v, v);

			bCon = FIsGrtr(minDist, sDist);
			bNotTerminated = BAnd(FIsGrtr(sDist, eps2), bCon);
		}
		while(BAllEq(bNotTerminated, bTrue));

		
		if(BAllEq(bCon, bFalse))
		{
			//Reset back to older closest point
			closA = tempClosA;
			closB = tempClosB;
			sDist = minDist;
			v = V3Sub(closA, closB);

			const FloatV recipDist = FRsqrt(sDist);
			const FloatV dist = FRecip(recipDist);//FSqrt(sDist);
			PX_ASSERT(FAllGrtr(dist, FEps()));
			const Vec3V n = V3Scale(v, recipDist);//normalise
			/*contactA = V3Sub(closA, V3Scale(n, marginA));
			contactB = V3Add(closB, V3Scale(n, marginB));*/
			contactA = V3NegScaleSub(n, marginA, closA);
			contactB = V3ScaleAdd(n, marginB, closB);
			penetrationDepth = FSub(dist, sumMargin0);
			normal = n;
			PX_ASSERT(isFiniteVec3V(normal));

			return GJK_CONTACT;
		}
		else
		{
			//return GJKPenetrationNoMargin<ConvexA, ConvexB>(a, b, contactA, contactB, normal, penetrationDepth);
			if( !GJKRecalculateSimplex<ConvexA, ConvexB>(a, b, contactA, contactB, normal, penetrationDepth))
			{
				const Vec3V v = V3Sub(tempClosA, tempClosB);
				const FloatV recipDist = FRsqrt(minDist);
				const FloatV dist = FRecip(recipDist);//FSqrt(minDist);
				const Vec3V n = V3Scale(v, recipDist);//normalise
				/*contactA = V3Sub(tempClosA, V3Scale(n, marginA));
				contactB = V3Add(tempClosB, V3Scale(n, marginB));*/

				contactA = V3NegScaleSub(n, marginA, tempClosA);
				contactB = V3ScaleAdd(n, marginB, tempClosB);
				//penetrationDepth = FSub(sumMargin0, dist);
				penetrationDepth = FSub(dist, sumMargin0); 
				//penetrationDepth = FMax(FSub(sumMargin0, dist), zero);
				normal = n;
				PX_ASSERT(isFiniteVec3V(normal));	
			}
			return GJK_CONTACT;
		}
	}


	//template<class ConvexA, class ConvexB>
	//GJKSTATUS GJKPenetration(const ConvexA& a, const ConvexB& b, const Ps::aos::FloatVArg contactDist, Ps::aos::Vec3V& contactA, Ps::aos::Vec3V& contactB, Ps::aos::Vec3V& normal, Ps::aos::FloatV& penetrationDepth)
	//{
	//	//PIX_PROFILE_ZONE(GJKPenetration);
	//	using namespace Ps::aos;

	//	Vec3V A[4]; 
	//	Vec3V B[4];
	//	Vec3V Q[4];

	//	const FloatV zero = FZero();

	//	const FloatV _marginA = a.getMargin();
	//	const FloatV _marginB = b.getMargin();
	//	const Vec3V centerAToCenterB =  V3Sub(b.getCenter(), a.getCenter());		

	//	const BoolV bHasMarginEqRadius = BOr(a.isMarginEqRadiusV(), b.isMarginEqRadiusV());

	//	const Vec3V _initialSearchDir = V3Neg(centerAToCenterB);
	//	Vec3V v = V3Sel(FIsGrtr(V3Dot(_initialSearchDir, _initialSearchDir), zero), _initialSearchDir, V3UnitX());

	//	const FloatV minMargin = FMin(_marginA, _marginB);
	//	const FloatV marginA = FSel(bHasMarginEqRadius, _marginA, minMargin);
	//	const FloatV marginB = FSel(bHasMarginEqRadius, _marginB, minMargin);
	//	

	//	const FloatV eps1 = FloatV_From_F32(0.001f);
	//	const FloatV eps2 = FMul(minMargin, FloatV_From_F32(0.001f));
	//	const FloatV ratio = FloatV_From_F32(0.05f);
	//	const Vec3V zeroV = V3Zero();
	//	const BoolV bTrue = BTTTT();
	//	const BoolV bFalse = BFFFF();
	//	PxU32 size=0;
	//
	//	//const FloatV sumMargin = FAdd(FAdd(marginA, marginB), tenthMargin);
	//	const FloatV sumMargin0 = FAdd(marginA, marginB);
	//	const FloatV sumMargin = FAdd(sumMargin0, contactDist);
	//	//const FloatV sumMargin = FAdd(marginA, marginB);
	//	const FloatV sqMargin = FMul(sumMargin, sumMargin);

	//	Vec3V closA = zeroV;
	//	Vec3V closB = zeroV;
	//	FloatV sDist = FMax();
	//	FloatV minDist;
	//	Vec3V tempClosA;
	//	Vec3V tempClosB;

	//	BoolV bNotTerminated = bTrue;
	//	BoolV bCon = bTrue;
	//
	//	do
	//	{
	//		minDist = sDist;
	//		tempClosA = closA;
	//		tempClosB = closB;

	//		const Vec3V nv = V3Neg(v);
	//		
	//		const Vec3V supportA=a.supportMargin(nv, marginA, A[size]);
	//		const Vec3V supportB=b.supportMargin(v, marginB, B[size]);
	//		//calculate the support point
	//		const Vec3V support = V3Sub(supportA, supportB);
	//		Q[size++]=support;

	//		PX_ASSERT(size <= 4);

	//		const FloatV tmp = FMul(sDist, sqMargin);//FMulAdd(sDist, sqMargin, eps3);
	//		const FloatV vw = V3Dot(v, support);
	//		const FloatV sqVW = FMul(vw, vw);
	//		
	//		const BoolV bTmp1 = FIsGrtr(vw, zero);
	//		const BoolV bTmp2 = FIsGrtr(sqVW, tmp);
	//		BoolV con = BAnd(bTmp1, bTmp2);

	//		const FloatV tmp1 = FSub(sDist, vw);
	//		const BoolV conGrtr = FIsGrtrOrEq(FMul(eps1, sDist), tmp1);

	//		const BoolV conOrconGrtr(BOr(con, conGrtr));

	//		if(BAllEq(conOrconGrtr, bTrue))
	//		{
	//			
	//			if(BAllEq(con, bFalse)) //must be true otherwise we wouldn't be in here...
	//			{
	//				const FloatV dist = FSqrt(sDist);
	//				PX_ASSERT(FAllGrtr(dist, FEps()));
	//				const Vec3V n = V3Scale(v, FRecip(dist));//normalise
	//				contactA = V3Sub(closA, V3Scale(n, marginA));
	//				contactB = V3Add(closB, V3Scale(n, marginB));
	//				//penetrationDepth = FMax(FSub(sumMargin0, dist), zero);
	//				penetrationDepth = FSub(sumMargin0, dist);
	//				normal = n;
	//				size--;
	//				PX_ASSERT(isFiniteVec3V(normal));
	//				return GJK_CONTACT;
	//				
	//			}
	//			else
	//			{
	//				return GJK_NON_INTERSECT;
	//			}

	//		}

	//		//calculate the closest point between two convex hull
	//		const Vec3V tempV = GJKCPairDoSimplex(Q, A, B, size, closA, closB);
	//		//v = V3Sub(closA, closB);
	//		v = tempV;
	//		sDist = V3Dot(tempV, tempV);

	//		bCon = FIsGrtr(minDist, sDist);
	//		bNotTerminated = BAnd(FIsGrtr(sDist, eps2), bCon);
	//	}
	//	while(BAllEq(bNotTerminated, bTrue));

	//	
	//	if(BAllEq(bCon, bFalse))
	//	{
	//		//Reset back to older closest point
	//		closA = tempClosA;
	//		closB = tempClosB;
	//		sDist = minDist;
	//		v = V3Sub(closA, closB);

	//		const FloatV dist = FSqrt(sDist);
	//		PX_ASSERT(FAllGrtr(dist, FEps()));
	//		const Vec3V n = V3Scale(v, FRecip(dist));//normalise
	//		contactA = V3Sub(closA, V3Scale(n, marginA));
	//		contactB = V3Add(closB, V3Scale(n, marginB));
	//		penetrationDepth = FSub(sumMargin0, dist);
	//		normal = n;
	//		PX_ASSERT(isFiniteVec3V(normal));

	//		return GJK_CONTACT;
	//	}
	//	else
	//	{
	//		//return GJKPenetrationNoMargin<ConvexA, ConvexB>(a, b, contactA, contactB, normal, penetrationDepth);
	//		if( !GJKRecalculateSimplex<ConvexA, ConvexB>(a, b, contactA, contactB, normal, penetrationDepth))
	//		{
	//			const Vec3V v = V3Sub(tempClosA, tempClosB);
	//			const FloatV dist = FSqrt(minDist);
	//			const Vec3V n = V3Scale(v, FRecip(dist));//normalise
	//			contactA = V3Sub(tempClosA, V3Scale(n, marginA));
	//			contactB = V3Add(tempClosB, V3Scale(n, marginB));
	//			penetrationDepth = FSub(sumMargin0, dist);
	//			normal = n;
	//			PX_ASSERT(isFiniteVec3V(normal));	
	//		}
	//		return GJK_CONTACT;
	//	}
	//}

	template<class ConvexA, class ConvexB>
	GJKSTATUS GJKPenetrationNoMargin(const ConvexA& a, const ConvexB& b, Ps::aos::Vec3V& contactA, Ps::aos::Vec3V& contactB, Ps::aos::Vec3V& normal, Ps::aos::FloatV& penetrationDepth)
	{
		using namespace Ps::aos;
		const Vec3V zeroV = V3Zero();
		const FloatV zero = FZero();
		const BoolV bTrue = BTTTT();
		const BoolV bFalse = BFFFF();
		PxU32 size=1;


		const Vec3V _initialSearchDir(V3Sub(a.getCenter(), b.getCenter()));
		const FloatV dotDir = V3Dot(_initialSearchDir, _initialSearchDir);
		const Vec3V initialSearchDir = V3Sel(FIsGrtr(dotDir, zero), _initialSearchDir, V3UnitX());
		const Vec3V initialSupportA(a.support(V3Neg(initialSearchDir)));
		const Vec3V initialSupportB(b.support(initialSearchDir));
		 
		Vec3V v = V3Sub(initialSupportA, initialSupportB);
		
		register Vec3V A[4] = {initialSupportA, zeroV, zeroV, zeroV}; //ConvexHull a simplex set
		register Vec3V B[4] = {initialSupportB, zeroV, zeroV, zeroV}; //ConvexHull b simplex set
		register Vec3V Q[4] = {v, zeroV, zeroV, zeroV}; //simplex set
		 
		Vec3V supportA = initialSupportA;
		Vec3V supportB = initialSupportB;

		const FloatV _marginA = a.getMargin();
		const FloatV _marginB = b.getMargin();
		const FloatV minMargin = FMin(_marginA, _marginB);

		const FloatV eps1 = FloatV_From_F32(0.001f);
		//const FloatV eps2 = FloatV_From_F32(0.000025f);
		const FloatV eps = FMul(minMargin, FloatV_From_F32(0.001f));
		const FloatV eps2 = FMul(eps, eps);

		Vec3V closA(initialSupportA), closB(initialSupportB);
		FloatV sDist = V3Dot(v, v);
		FloatV minDist = sDist;
		Vec3V tempClosA(initialSupportA), tempClosB(initialSupportB);

		GJKSTATUS status = GJK_DEPENETRATION;

		BoolV bNotTerminated = FIsGrtr(sDist,eps2);
		BoolV bCon = bTrue;

		while(BAllEq(bNotTerminated, bTrue))
		{

			minDist = sDist;
			tempClosA = closA;
			tempClosB = closB;

			supportA=a.support(V3Neg(v));
			supportB=b.support(v);
			//calculate the support point
			const Vec3V support = V3Sub(supportA, supportB);

			PX_ASSERT(size < 4);
			A[size]=supportA;
			B[size]=supportB;
			Q[size++]=support;

			const FloatV vw = V3Dot(v, support);

			const FloatV tmp1 = FSub(sDist, vw);
			const BoolV con = FIsGrtrOrEq(FMul(eps1, sDist), tmp1);

			if(BAllEq(con, bTrue))
			{
				//not intersect
				const FloatV dist = FSqrt(sDist);
				const Vec3V n = V3Scale(v, FRecip(dist));
				contactA = closA;
				contactB = closB;
				penetrationDepth = dist;
				normal = n;//normalise
				size--;
				return GJK_NON_INTERSECT;
			}

			//calculate the closest point between two convex hull
			v = GJKCPairDoSimplex(Q, A, B, size, closA, closB);
			sDist = V3Dot(v, v);

			bCon = FIsGrtr(minDist, sDist);
			bNotTerminated = BAnd(FIsGrtr(sDist, eps2), bCon);

		}

		if(BAllEq(bCon, bFalse)) //must be true otherwise we wouldn't be in here...
		{
			//Reset back to older closest point
			closA = tempClosA;
			closB = tempClosB;
			sDist = minDist;
			const Vec3V v = V3Sub(tempClosA, tempClosB);
			const FloatV dist = V3Length(v);//FSqrt(sDist);
			PX_ASSERT(FAllGrtr(dist, FEps()));
			normal = V3Scale(v, FRecip(dist));//normalise
			contactA = closA;
			contactB = closB;
			penetrationDepth = dist;
			return GJK_CONTACT;
		}

		Vec3V contA = closA;
		Vec3V contB = closB;
		EPA epa;
		if(epa.PenetrationDepth<ConvexA, ConvexB>(a, b, Q, A, B, size, contA, contB))
		{
			const Vec3V v1 = V3Sub(contB, contA);    
			const FloatV dist = V3Length(v1);
			normal = V3Scale(v1, FRecip(dist));
			contactA = contA;
			contactB = contB;
			penetrationDepth = dist;
		}
		else
		{
			const Vec3V v = V3Sub(closA, closB);
			const FloatV dist = V3Length(v);//FSqrt(sDist);
			PX_ASSERT(FAllGrtr(dist, FEps()));
			normal = V3Scale(v, FRecip(dist));//normalise
			contactA = closA;
			contactB = closB;
			penetrationDepth = dist;
		}
		PX_ASSERT(isFiniteVec3V(normal));
		return GJK_CONTACT;
	}


#else
	

	template<typename ConvexA, typename ConvexB>
	GJKSTATUS GJKPenetration(const ConvexA& a, const ConvexB& b, const Ps::aos::FloatVArg contactDist, Ps::aos::Vec3V& contactA, Ps::aos::Vec3V& contactB, Ps::aos::Vec3V& normal, Ps::aos::FloatV& penetrationDepth);

	GJKSTATUS GJKPenetration(const ConvexV& a, const ConvexV& b, Support aSupport, Support bSupport, SupportMargin aSupportMargin, SupportMargin bSupportMargin, const Ps::aos::FloatVArg contactDist, Ps::aos::Vec3V& contactA, Ps::aos::Vec3V& contactB, Ps::aos::Vec3V& normal, Ps::aos::FloatV& penetrationDepth);

	//GJKSTATUS GJKPenetration(const ConvexA& a, const ConvexB& b, Ps::aos::Vec3V* PX_RESTRICT _mA, Ps::aos::Vec3V* PX_RESTRICT _mB, PxU32& _size, Ps::aos::Vec3V& contactA, Ps::aos::Vec3V& contactB, Ps::aos::Vec3V& normal, Ps::aos::FloatV& penetrationDepth);
#endif

}

}

#endif

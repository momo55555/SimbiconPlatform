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


#include "GJK.h"
#include "EPA.h"
#include "GuVecSphere.h"
#include "GuVecBox.h"
#include "GuVecCapsule.h"
#include "GuVecConvexHull.h"
#include "GuVecCylinder.h"
#include "GuVecCone.h"

namespace physx
{
namespace Gu
{


	//PX_FORCE_INLINE Ps::aos::Vec3V closestPtPointTriangle(Ps::aos::Vec3V* PX_RESTRICT Q, Ps::aos::Vec3V* PX_RESTRICT A, Ps::aos::Vec3V* PX_RESTRICT B, PxU32& size, Ps::aos::Vec3V& closestA, Ps::aos::Vec3V& closestB)
	//{
	//	using namespace Ps::aos;

	//	const FloatV zero = FZero();
	//	const BoolV bTrue = BTTTT();
	//	
	//	//const Vec3V zero = V3Zero();
	//	const Vec3V a = Q[0];
	//	const Vec3V b = Q[1];
	//	const Vec3V c = Q[2];

	//	//const FloatV half = FloatV_From_F32(0.5f);
	//	const FloatV eps = FloatV_From_F32(0.002f);//eps should be 0.001f, (2*eps)*(2*eps)

	//	const Vec3V ab = V3Sub(b, a);
	//	const Vec3V ac = V3Sub(c, a);
	//	//const Vec3V bc = V3Sub(c, b);

	//	const Vec3V ap = V3Neg(a);
	//	const Vec3V bp = V3Neg(b);
	//	const Vec3V cp = V3Neg(c);

	//	const FloatV eps2 = FMul(eps, eps);

	//	const FloatV d1 = V3Dot(ab, ap); //  snom
	//	const FloatV d2 = V3Dot(ac, ap); //  tnom
	//	const FloatV d3 = V3Dot(ab, bp); // -sdenom
	//	const FloatV d4 = V3Dot(ac, bp); //  unom = d4 - d3
	//	const FloatV d5 = V3Dot(ab, cp); //  udenom = d5 - d6
	//	const FloatV d6 = V3Dot(ac, cp); // -tdenom
	//	const FloatV unom = FSub(d4, d3);
	//	const FloatV udenom = FSub(d5, d6);

	//	//Detect degenerated case, calculate the triangle area
	//	const FloatV sqAb = V3Dot(ab, ab);
	//	const FloatV sqAc = V3Dot(ac, ac);
	//	const FloatV abac = V3Dot(ab, ac);
	//	const FloatV v = FSub(FMul(sqAb, sqAc), FMul(abac, abac));

	//	const FloatV va = FSub(FMul(d3, d6), FMul(d5, d4));//edge region of BC
	//	const FloatV vb = FSub(FMul(d5, d2), FMul(d1, d6));//edge region of AC
	//	const FloatV vc = FSub(FMul(d1, d4), FMul(d3, d2));//edge region of AB
	//
	//	if(FAllGrtr(eps2, v))
	//	{
	//		PxU32 _size = 2;
	//		Q[1] = Q[2];
	//		A[1] = A[2];
	//		B[1] = B[2];
	//		Vec3V tempClosestA, tempClosestB;
	//		closestPtPointSegment(Q, A, B, _size,tempClosestA, tempClosestB);
	//		closestA = tempClosestA;
	//		closestB = tempClosestB;
	//		size = _size;
	//		return V3Sub(tempClosestA, tempClosestB);
	//	}

	//	
	//	//check if p in vertex region outside a
	//	const BoolV con00 = FIsGrtrOrEq(zero, d1); // snom <= 0
	//	const BoolV con01 = FIsGrtrOrEq(zero, d2); // tnom <= 0
	//	const BoolV con0 = BAnd(con00, con01); // vertex region a
	//	//const Vec3V closestA0 = A[0];
	//	//const Vec3V closestB0 = B[0];

	//	//check if p in vertex region outside b
	//	const BoolV con10 = FIsGrtrOrEq(d3, zero);
	//	const BoolV con11 = FIsGrtrOrEq(d3, d4);
	//	const BoolV con1 = BAnd(con10, con11); // vertex region b

	//	const BoolV con20 = FIsGrtrOrEq(d6, zero);
	//	const BoolV con21 = FIsGrtrOrEq(d6, d5); 
	//	const BoolV con2 = BAnd(con20, con21); // vertex region c

	//	const BoolV bCondition = BOr(con0, BOr(con1, con2));
	//	if(BAllEq(bCondition, bTrue))
	//	{
	//		const Vec3V tempClosestA = V3Sel(con0, A[0], V3Sel(con1, A[1], A[2]));
	//		const Vec3V tempClosestB = V3Sel(con0, B[0], V3Sel(con1, B[1], B[2]));
	//		closestA = tempClosestA;
	//		closestB = tempClosestB;
	//		return V3Sub(tempClosestA, tempClosestB);
	//	}		

	//	//check if p in edge region of AB
	//	const BoolV con30 = FIsGrtrOrEq(zero, vc);
	//	const BoolV con31 = FIsGrtrOrEq(d1, zero);
	//	const BoolV con32 = FIsGrtrOrEq(zero, d3);
	//	const BoolV con3 = BAnd(con30, BAnd(con31, con32));
	//	const FloatV sScale = FDiv(d1, FSub(d1, d3));
	//	//const Vec3V closest3 = V3Add(a, V3Scale(ab, sScale));
	//	if(BAllEq(con3, bTrue))
	//	{
	//		const Vec3V v = V3Sub(A[1], A[0]);
	//		const Vec3V w = V3Sub(B[1], B[0]);
	//		const Vec3V tempClosestA = V3Add(A[0], V3Scale(v, sScale));
	//		const Vec3V tempClosestB = V3Add(B[0], V3Scale(w, sScale));
	//		closestA = tempClosestA;
	//		closestB = tempClosestB;
	//		return V3Sub(tempClosestA, tempClosestB);
	//	}

	//	//check if p in edge region of BC
	//	const BoolV con40 = FIsGrtrOrEq(zero, va);
	//	const BoolV con41 = FIsGrtrOrEq(d4, d3);
	//	const BoolV con42 = FIsGrtrOrEq(d5, d6);
	//	const BoolV con4 = BAnd(con40, BAnd(con41, con42)); 
	//	const FloatV uScale = FDiv(unom, FAdd(unom, udenom));
	//	//const Vec3V closest4 = V3Add(b, V3Scale(bc, uScale));
	//	if(BAllEq(con4, bTrue))
	//	{
	//		const Vec3V v = V3Sub(A[2], A[1]);
	//		const Vec3V w = V3Sub(B[2], B[1]);
	//		const Vec3V tempClosestA = V3Add(A[1], V3Scale(v, uScale));
	//		const Vec3V tempClosestB = V3Add(B[1], V3Scale(w, uScale));
	//		closestA = tempClosestA;
	//		closestB = tempClosestB;
	//		return V3Sub(tempClosestA, tempClosestB);
	//	}

	//	//check if p in edge region of AC
	//	const BoolV con50 = FIsGrtrOrEq(zero, vb);
	//	const BoolV con51 = FIsGrtrOrEq(d2, zero);
	//	const BoolV con52 = FIsGrtrOrEq(zero, d6);
	//	const BoolV con5 = BAnd(con50, BAnd(con51, con52));
	//	const FloatV tScale = FDiv(d2, FSub(d2, d6));
	//	//const Vec3V closest5 = V3Add(a, V3Scale(ac, tScale));
	//	if(BAllEq(con5, bTrue))
	//	{
	//		const Vec3V v = V3Sub(A[2], A[0]);
	//		const Vec3V w = V3Sub(B[2], B[0]);
	//		const Vec3V tempClosestA = V3Add(A[0], V3Scale(v, tScale));
	//		const Vec3V tempClosestB = V3Add(B[0], V3Scale(w, tScale));
	//		closestA = tempClosestA;
	//		closestB = tempClosestB;
	//		return V3Sub(tempClosestA, tempClosestB);
	//	}

	//	//P must project inside face region. Compute Q using Barycentric coordinates
	//	const FloatV denom = FRecip(FAdd(va, FAdd(vb, vc)));
	//
	//	const Vec3V v0 = V3Sub(A[1], A[0]);
	//	const Vec3V v1 = V3Sub(A[2], A[0]);
	//	const Vec3V w0 = V3Sub(B[1], B[0]);
	//	const Vec3V w1 = V3Sub(B[2], B[0]);

	//	const FloatV t = FMul(vb, denom);
	//	const FloatV w = FMul(vc, denom);
	//	const Vec3V vA0 = V3Scale(v0, t);
	//	const Vec3V vA1 = V3Scale(v1, w);
	//	const Vec3V vB0 = V3Scale(w0, t);
	//	const Vec3V vB1 = V3Scale(w1, w);
	//	const Vec3V tempClosestA = V3Add(A[0], V3Add(vA0, vA1));
	//	const Vec3V tempClosestB = V3Add(B[0], V3Add(vB0, vB1));
	//	closestA = tempClosestA;
	//	closestB = tempClosestB;
	//	return V3Sub(tempClosestA, tempClosestB);
	//}

	PX_NOALIAS Ps::aos::Vec3V closestPtPointTriangle(const Ps::aos::Vec3V a, const Ps::aos::Vec3V b, const Ps::aos::Vec3V c, Ps::aos::Vec3V* PX_RESTRICT Q, Ps::aos::Vec3V* PX_RESTRICT A, Ps::aos::Vec3V* PX_RESTRICT B, PxU32& size, Ps::aos::Vec3V& closestA, Ps::aos::Vec3V& closestB)
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
		const FloatV v = FNegMulSub(abac, abac, FMul(sqAb, sqAc));

		const FloatV va = FSub(FMul(d3, d6), FMul(d5, d4));//edge region of BC
		const FloatV vb = FSub(FMul(d5, d2), FMul(d1, d6));//edge region of AC
		const FloatV vc = FSub(FMul(d1, d4), FMul(d3, d2));//edge region of AB

		//const FloatV va = FNegMulSub(d5, d4, FMul(d3, d6));//edge region of BC
		//const FloatV vb = FNegMulSub(d1, d6, FMul(d5, d2));//edge region of AC
		//const FloatV vc = FNegMulSub(d3, d2, FMul(d1, d4));//edge region of AB
	
		if(FAllGrtr(eps2, v))
		{
			size = 2;
			//if(FAllEq(sqAb, zero))
			{
				const BoolV b = FIsEq(sqAb, zero);
				//a and b are the same
				Q[1] = V3Sel(b, Q[2], Q[1]);
				A[1] = V3Sel(b, A[2], A[1]);
				B[1] = V3Sel(b, B[2], B[1]);
			}
			
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


	PX_NOALIAS Ps::aos::Vec3V closestPtPointTriangle(const Ps::aos::Vec3V a, const Ps::aos::Vec3V b, const Ps::aos::Vec3V c, Ps::aos::Vec3V* PX_RESTRICT Q, Ps::aos::Vec3V* PX_RESTRICT A, Ps::aos::Vec3V* PX_RESTRICT B, Ps::aos::Vec3V* PX_RESTRICT Dir, PxU32& size, Ps::aos::Vec3V& closestA, Ps::aos::Vec3V& closestB)
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
		const FloatV v = FNegMulSub(abac, abac, FMul(sqAb, sqAc));

		const FloatV va = FSub(FMul(d3, d6), FMul(d5, d4));//edge region of BC
		const FloatV vb = FSub(FMul(d5, d2), FMul(d1, d6));//edge region of AC
		const FloatV vc = FSub(FMul(d1, d4), FMul(d3, d2));//edge region of AB

		//const FloatV va = FNegMulSub(d5, d4, FMul(d3, d6));//edge region of BC
		//const FloatV vb = FNegMulSub(d1, d6, FMul(d5, d2));//edge region of AC
		//const FloatV vc = FNegMulSub(d3, d2, FMul(d1, d4));//edge region of AB
	
		if(FAllGrtr(eps2, v))
		{
			size = 2;
			//if(FAllEq(sqAb, zero))
			{
				const BoolV b = FIsEq(sqAb, zero);
				//a and b are the same
				Q[1] = V3Sel(b, Q[2], Q[1]);
				A[1] = V3Sel(b, A[2], A[1]);
				B[1] = V3Sel(b, B[2], B[1]);
				Dir[1] = V3Sel(b, Dir[2], Dir[1]);
			}
			
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

	PX_NOALIAS Ps::aos::Vec3V closestPtPointTetrahedron(Ps::aos::Vec3V* PX_RESTRICT Q, Ps::aos::Vec3V* PX_RESTRICT A, Ps::aos::Vec3V* PX_RESTRICT B, PxU32& size, Ps::aos::Vec3V& closestA, Ps::aos::Vec3V& closestB)
	{
		using namespace Ps::aos;
		//const Vec3V origin = V3Zero();
		Vec3V tempClosestA = closestA;
		Vec3V tempClosestB = closestB;
		PxU32 tempSize = size;
		
		const FloatV eps = FloatV_From_F32(0.001f);	
		
		FloatV bestSqDist = FloatV_From_F32(PX_MAX_REAL);
		const Vec3V a = Q[0];
		const Vec3V b = Q[1];
		const Vec3V c = Q[2];
		const Vec3V d = Q[3];
		const BoolV bTrue = BTTTT();
		//const BoolV bFalse = BFFFF();

		Vec3V _Q[] = {Q[0], Q[1], Q[2], Q[3]};
		Vec3V _A[] = {A[0], A[1], A[2], A[3]};
		Vec3V _B[] = {B[0], B[1], B[2], B[3]};

		//test for degenerate case
		const Vec3V db = V3Sub(b, d);
		const Vec3V dc = V3Sub(c, d);
		const Vec3V da = V3Sub(a, d);
		const FloatV volume = FAbs(V3Dot(da, V3Cross(db, dc)));
		if(FAllGrtr(eps, volume))
		{
			
			Vec3V Qt[3] = {_Q[0], _Q[1], _Q[3]};
			Vec3V At[3] = {_A[0], _A[1], _A[3]};
			Vec3V Bt[3] = {_B[0], _B[1], _B[3]};
			PxU32 _size = 3;
			Vec3V closestPtA, closestPtB;
			Vec3V result = closestPtPointTriangle(_Q[0], _Q[1], _Q[3], Qt, At, Bt, _size, closestPtA, closestPtB);

			Q[0] = Qt[0]; Q[1] = Qt[1]; Q[2] = Qt[2];
			A[0] = At[0]; A[1] = At[1]; A[2] = At[2];
			B[0] = Bt[0]; B[1] = Bt[1]; B[2] = Bt[2];
			size = _size;
			closestA = closestPtA;
			closestB = closestPtB;
			return result;
		}

		Vec3V result = V3Zero();

		
		const BoolV bIsOutside4 = PointOutsideOfPlane4(a, b, c, d);

		//if(BAllEq(bIsOutside4, bFalse))
		//{
		//	//All inside
		//	return V3Zero();
		//}


		//if(PointOutsideOfPlane(a, b, c, d))
		//if(bOutside[0])
		if(BAllEq(BGetX(bIsOutside4), bTrue))
		{
			Vec3V Qt[3] = {_Q[0], _Q[1], _Q[2]};
			Vec3V At[3] = {_A[0], _A[1], _A[2]};
			Vec3V Bt[3] = {_B[0], _B[1], _B[2]};
			PxU32 _size = 3;
			Vec3V closestPtA, closestPtB;
			result = closestPtPointTriangle(_Q[0], _Q[1], _Q[2], Qt, At, Bt, _size, closestPtA, closestPtB);

			const FloatV sqDist = V3Dot(result, result);
			//const BoolV con = FIsGrtr(bestSqDist, sqDist);
			bestSqDist = sqDist;
			
			Q[0] = Qt[0]; Q[1] = Qt[1]; Q[2] = Qt[2];
			A[0] = At[0]; A[1] = At[1]; A[2] = At[2];
			B[0] = Bt[0]; B[1] = Bt[1]; B[2] = Bt[2];
			tempSize = _size;
			tempClosestA = closestPtA;
			tempClosestB = closestPtB;
		}

		//if(PointOutsideOfPlane(a, c, d, b))
		//if(bOutside[1])
		if(BAllEq(BGetY(bIsOutside4), bTrue))
		{
			Vec3V Qt[3] = {_Q[0], _Q[2], _Q[3]};
			Vec3V At[3] = {_A[0], _A[2], _A[3]};
			Vec3V Bt[3] = {_B[0], _B[2], _B[3]};
			PxU32 _size = 3;
			
			Vec3V closestPtA, closestPtB;
			const Vec3V q = closestPtPointTriangle(_Q[0], _Q[2], _Q[3], Qt, At, Bt, _size, closestPtA, closestPtB);

			const FloatV sqDist = V3Dot(q, q);
			const BoolV con = FIsGrtr(bestSqDist, sqDist);
			if(BAllEq(con, bTrue))
			{
				result = q;
				bestSqDist = sqDist;
				Q[0] = Qt[0]; Q[1] = Qt[1]; Q[2] = Qt[2];
				A[0] = At[0]; A[1] = At[1]; A[2] = At[2];
				B[0] = Bt[0]; B[1] = Bt[1]; B[2] = Bt[2];
				tempSize = _size;
				tempClosestA = closestPtA;
				tempClosestB = closestPtB;
			}
		}

		//if(PointOutsideOfPlane(a, d, b, c))
		//if(bOutside[2])
		if(BAllEq(BGetZ(bIsOutside4), bTrue))
		{
			Vec3V Qt[3] = {_Q[0], _Q[3], _Q[1]};
			Vec3V At[3] = {_A[0], _A[3], _A[1]};
			Vec3V Bt[3] = {_B[0], _B[3], _B[1]};
			PxU32 _size = 3;
			Vec3V closestPtA, closestPtB;
			const Vec3V q = closestPtPointTriangle(_Q[0], _Q[3], _Q[1], Qt, At, Bt, _size, closestPtA, closestPtB);

			const FloatV sqDist = V3Dot(q, q);
			const BoolV con = FIsGrtr(bestSqDist, sqDist);
			if(BAllEq(con, bTrue))
			{
				result = q;
				bestSqDist = sqDist;
				Q[0] = Qt[0]; Q[1] = Qt[1]; Q[2] = Qt[2];
				A[0] = At[0]; A[1] = At[1]; A[2] = At[2];
				B[0] = Bt[0]; B[1] = Bt[1]; B[2] = Bt[2];
				tempSize = _size;
				tempClosestA = closestPtA;
				tempClosestB = closestPtB;
			}

		}

		//if(PointOutsideOfPlane(b, d, c, a))
		//if(bOutside[3])
		if(BAllEq(BGetW(bIsOutside4), bTrue))
		{
			Vec3V Qt[3] = {_Q[1], _Q[3], _Q[2]};
			Vec3V At[3] = {_A[1], _A[3], _A[2]};
			Vec3V Bt[3] = {_B[1], _B[3], _B[2]};
			PxU32 _size = 3;
			Vec3V closestPtA, closestPtB;
			const Vec3V q = closestPtPointTriangle(_Q[1], _Q[3], _Q[2], Qt, At, Bt, _size, closestPtA, closestPtB);

			const FloatV sqDist = V3Dot(q, q);
			const BoolV con = FIsGrtr(bestSqDist, sqDist);

			if(BAllEq(con, bTrue))
			{
				result = q;
				bestSqDist = sqDist;
				Q[0] = Qt[0]; Q[1] = Qt[1]; Q[2] = Qt[2];
				A[0] = At[0]; A[1] = At[1]; A[2] = At[2];
				B[0] = Bt[0]; B[1] = Bt[1]; B[2] = Bt[2];
				tempSize = _size;
				tempClosestA = closestPtA;
				tempClosestB = closestPtB;
			}
		}

		closestA = tempClosestA;
		closestB = tempClosestB;
		size = tempSize;
		return result;
	}
}

}

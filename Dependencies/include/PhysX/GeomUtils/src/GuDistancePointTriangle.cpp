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


#include "GuDistancePointTriangle.h"
#include "PxVec3.h"

using namespace physx;

// ptchernev:
// There is an extra check for degenerate triangles fDet==0 in the
// case 0 block which is not in the original Magic Software code.
// But I will keep it in since it is probably there for a very good
// reason.

PxReal Gu::distancePointTriangleSquared(	const PxVec3& point, 
										const PxVec3& triangleOrigin, 
										const PxVec3& triangleEdge0, 
										const PxVec3& triangleEdge1,
										PxReal* param0, 
										PxReal* param1)
{
	const PxVec3 kDiff	= triangleOrigin - point;
	const PxReal fA00	= triangleEdge0.magnitudeSquared();
	const PxReal fA01	= triangleEdge0.dot(triangleEdge1);
	const PxReal fA11	= triangleEdge1.magnitudeSquared();
	const PxReal fB0	= kDiff.dot(triangleEdge0);
	const PxReal fB1	= kDiff.dot(triangleEdge1);
	const PxReal fC		= kDiff.magnitudeSquared();
	const PxReal fDet	= PxAbs(fA00*fA11 - fA01*fA01);
	PxReal fS			= fA01*fB1-fA11*fB0;
	PxReal fT			= fA01*fB0-fA00*fB1;
	PxReal fSqrDist;

	if(fS + fT <= fDet)
	{
		if(fS < 0.0f)
		{
			if(fT < 0.0f)  // region 4
			{
				if(fB0 < 0.0f)
				{
					fT = 0.0f;
					if(-fB0 >= fA00)
					{
						fS = 1.0f;
						fSqrDist = fA00+2.0f*fB0+fC;
					}
					else
					{
						fS = -fB0/fA00;
						fSqrDist = fB0*fS+fC;
					}
				}
				else
				{
					fS = 0.0f;
					if(fB1 >= 0.0f)
					{
						fT = 0.0f;
						fSqrDist = fC;
					}
					else if(-fB1 >= fA11)
					{
						fT = 1.0f;
						fSqrDist = fA11+2.0f*fB1+fC;
					}
					else
					{
						fT = -fB1/fA11;
						fSqrDist = fB1*fT+fC;
					}
				}
			}
			else  // region 3
			{
				fS = 0.0f;
				if(fB1 >= 0.0f)
				{
					fT = 0.0f;
					fSqrDist = fC;
				}
				else if(-fB1 >= fA11)
				{
					fT = 1.0f;
					fSqrDist = fA11+2.0f*fB1+fC;
				}
				else
				{
					fT = -fB1/fA11;
					fSqrDist = fB1*fT+fC;
				}
			}
		}
		else if(fT < 0.0f)  // region 5
		{
			fT = 0.0f;
			if(fB0 >= 0.0f)
			{
				fS = 0.0f;
				fSqrDist = fC;
			}
			else if(-fB0 >= fA00)
			{
				fS = 1.0f;
				fSqrDist = fA00+2.0f*fB0+fC;
			}
			else
			{
				fS = -fB0/fA00;
				fSqrDist = fB0*fS+fC;
			}
		}
		else  // region 0
		{
			// minimum at interior PxVec3
			if(fDet==0.0f)
			{
				fS = 0.0f;
				fT = 0.0f;
				fSqrDist = PX_MAX_REAL;
			}
			else
			{
				PxReal fInvDet = 1.0f/fDet;
				fS *= fInvDet;
				fT *= fInvDet;
				fSqrDist = fS*(fA00*fS+fA01*fT+2.0f*fB0) + fT*(fA01*fS+fA11*fT+2.0f*fB1)+fC;
			}
		}
	}
	else
	{
		PxReal fTmp0, fTmp1, fNumer, fDenom;

		if(fS < 0.0f)  // region 2
		{
			fTmp0 = fA01 + fB0;
			fTmp1 = fA11 + fB1;
			if(fTmp1 > fTmp0)
			{
				fNumer = fTmp1 - fTmp0;
				fDenom = fA00-2.0f*fA01+fA11;
				if(fNumer >= fDenom)
				{
					fS = 1.0f;
					fT = 0.0f;
					fSqrDist = fA00+2.0f*fB0+fC;
				}
				else
				{
					fS = fNumer/fDenom;
					fT = 1.0f - fS;
					fSqrDist = fS*(fA00*fS+fA01*fT+2.0f*fB0) + fT*(fA01*fS+fA11*fT+2.0f*fB1)+fC;
				}
			}
			else
			{
				fS = 0.0f;
				if(fTmp1 <= 0.0f)
				{
					fT = 1.0f;
					fSqrDist = fA11+2.0f*fB1+fC;
				}
				else if(fB1 >= 0.0f)
				{
					fT = 0.0f;
					fSqrDist = fC;
				}
				else
				{
					fT = -fB1/fA11;
					fSqrDist = fB1*fT+fC;
				}
			}
		}
		else if(fT < 0.0f)  // region 6
		{
			fTmp0 = fA01 + fB1;
			fTmp1 = fA00 + fB0;
			if(fTmp1 > fTmp0)
			{
				fNumer = fTmp1 - fTmp0;
				fDenom = fA00-2.0f*fA01+fA11;
				if(fNumer >= fDenom)
				{
					fT = 1.0f;
					fS = 0.0f;
					fSqrDist = fA11+2.0f*fB1+fC;
				}
				else
				{
					fT = fNumer/fDenom;
					fS = 1.0f - fT;
					fSqrDist = fS*(fA00*fS+fA01*fT+2.0f*fB0) + fT*(fA01*fS+fA11*fT+2.0f*fB1)+fC;
				}
			}
			else
			{
				fT = 0.0f;
				if(fTmp1 <= 0.0f)
				{
					fS = 1.0f;
					fSqrDist = fA00+2.0f*fB0+fC;
				}
				else if(fB0 >= 0.0f)
				{
					fS = 0.0f;
					fSqrDist = fC;
				}
				else
				{
					fS = -fB0/fA00;
					fSqrDist = fB0*fS+fC;
				}
			}
		}
		else  // region 1
		{
			fNumer = fA11 + fB1 - fA01 - fB0;
			if(fNumer <= 0.0f)
			{
				fS = 0.0f;
				fT = 1.0f;
				fSqrDist = fA11+2.0f*fB1+fC;
			}
			else
			{
				fDenom = fA00-2.0f*fA01+fA11;
				if(fNumer >= fDenom)
				{
					fS = 1.0f;
					fT = 0.0f;
					fSqrDist = fA00+2.0f*fB0+fC;
				}
				else
				{
					fS = fNumer/fDenom;
					fT = 1.0f - fS;
					fSqrDist = fS*(fA00*fS+fA01*fT+2.0f*fB0) + fT*(fA01*fS+fA11*fT+2.0f*fB1)+fC;
				}
			}
		}
	}

	if(param0) *param0 = fS;
	if(param1) *param1 = fT;

	// account for numerical round-off error
	return physx::intrinsics::selectMax(0.0f, fSqrDist);
}


//Ps::aos::FloatV Gu::distancePointTriangleSquared(	const Ps::aos::Vec3VArg p, 
//													const Ps::aos::Vec3VArg a, 
//													const Ps::aos::Vec3VArg b, 
//													const Ps::aos::Vec3VArg c,
//													Ps::aos::FloatV& u,
//													Ps::aos::FloatV& v,
//													Ps::aos::Vec3V& closestP)
//{
//	using namespace Ps::aos;
//
//	const FloatV zero = FZero();
//	const FloatV one = FOne();
//	//const Vec3V zero = V3Zero();
//	const Vec3V ab = V3Sub(b, a);
//	const Vec3V ac = V3Sub(c, a);
//	const Vec3V bc = V3Sub(c, b);
//	const Vec3V ap = V3Sub(p, a);
//	const Vec3V bp = V3Sub(p, b);
//	const Vec3V cp = V3Sub(p, c);
//
//	const FloatV d1 = V3Dot(ab, ap); //  snom
//	const FloatV d2 = V3Dot(ac, ap); //  tnom
//	const FloatV d3 = V3Dot(ab, bp); // -sdenom
//	const FloatV d4 = V3Dot(ac, bp); //  unom = d4 - d3
//	const FloatV d5 = V3Dot(ab, cp); //  udenom = d5 - d6
//	const FloatV d6 = V3Dot(ac, cp); // -tdenom
//	const FloatV unom = FSub(d4, d3);
//	const FloatV udenom = FSub(d5, d6);
//	
//	//check if p in vertex region outside a
//	const BoolV con00 = FIsGrtr(zero, d1); // snom <= 0
//	const BoolV con01 = FIsGrtr(zero, d2); // tnom <= 0
//	const BoolV con0 = BAnd(con00, con01); // vertex region a
//	const FloatV u0 = zero;
//	const FloatV v0 = zero;
//
//	//check if p in vertex region outside b
//	const BoolV con10 = FIsGrtrOrEq(d3, zero);
//	const BoolV con11 = FIsGrtrOrEq(d3, d4);
//	const BoolV con1 = BAnd(con10, con11); // vertex region b
//	const FloatV u1 = one;
//	const FloatV v1 = zero;
//
//	//check if p in vertex region outside c
//	const BoolV con20 = FIsGrtrOrEq(d6, zero);
//	const BoolV con21 = FIsGrtrOrEq(d6, d5); 
//	const BoolV con2 = BAnd(con20, con21); // vertex region c
//	const FloatV u2 = zero;
//	const FloatV v2 = one;
//
//	//check if p in edge region of AB
//	const FloatV vc = FSub(FMul(d1, d4), FMul(d3, d2));
//	
//	const BoolV con30 = FIsGrtr(zero, vc);
//	const BoolV con31 = FIsGrtrOrEq(d1, zero);
//	const BoolV con32 = FIsGrtr(zero, d3);
//	const BoolV con3 = BAnd(con30, BAnd(con31, con32));
//	const FloatV sScale = FDiv(d1, FSub(d1, d3));
//	const Vec3V closest3 = V3Add(a, V3Scale(ab, sScale));
//	const FloatV u3 = sScale;
//	const FloatV v3 = zero;
//
//	//check if p in edge region of BC
//	const FloatV va = FSub(FMul(d3, d6),FMul(d5, d4));
//	const BoolV con40 = FIsGrtr(zero, va);
//	const BoolV con41 = FIsGrtrOrEq(d4, d3);
//	const BoolV con42 = FIsGrtrOrEq(d5, d6);
//	const BoolV con4 = BAnd(con40, BAnd(con41, con42)); 
//	const FloatV uScale = FDiv(unom, FAdd(unom, udenom));
//	const Vec3V closest4 = V3Add(b, V3Scale(bc, uScale));
//	const FloatV u4 = FSub(one, uScale);
//	const FloatV v4 = uScale;
//
//	//check if p in edge region of AC
//	const FloatV vb = FSub(FMul(d5, d2), FMul(d1, d6));
//	const BoolV con50 = FIsGrtr(zero, vb);
//	const BoolV con51 = FIsGrtrOrEq(d2, zero);
//	const BoolV con52 = FIsGrtr(zero, d6);
//	const BoolV con5 = BAnd(con50, BAnd(con51, con52));
//	const FloatV tScale = FDiv(d2, FSub(d2, d6));
//	const Vec3V closest5 = V3Add(a, V3Scale(ac, tScale));
//	const FloatV u5 = zero;
//	const FloatV v5 = tScale;
//
//	//P must project inside face region. Compute Q using Barycentric coordinates
//	const FloatV denom = FRecip(FAdd(va, FAdd(vb, vc)));
//	const FloatV t = FMul(vb, denom);
//	const FloatV w = FMul(vc, denom);
//	const Vec3V bCom = V3Scale(ab, t);
//	const Vec3V cCom = V3Scale(ac, w);
//	const Vec3V closest6 = V3Add(a, V3Add(bCom, cCom));
//	const FloatV u6 = t;
//	const FloatV v6 = w;
//	
//	const Vec3V closest= V3Sel(con0, a, V3Sel(con1, b, V3Sel(con2, c, V3Sel(con3, closest3, V3Sel(con4, closest4, V3Sel(con5, closest5, closest6))))));
//	u = FSel(con0, u0, FSel(con1, u1, FSel(con2, u2, FSel(con3, u3, FSel(con4, u4, FSel(con5, u5, u6))))));
//	v = FSel(con0, v0, FSel(con1, v1, FSel(con2, v2, FSel(con3, v3, FSel(con4, v4, FSel(con5, v5, v6))))));
//	closestP = closest;
//
//	const Vec3V vv = V3Sub(p, closest);
//
//	return V3Dot(vv, vv);
//}


Ps::aos::FloatV Gu::distancePointTriangleSquared(	const Ps::aos::Vec3VArg p, 
													const Ps::aos::Vec3VArg a, 
													const Ps::aos::Vec3VArg b, 
													const Ps::aos::Vec3VArg c,
													Ps::aos::FloatV& u,
													Ps::aos::FloatV& v,
													Ps::aos::Vec3V& closestP)
{
	using namespace Ps::aos;

	const FloatV zero = FZero();
	const FloatV one = FOne();
	//const Vec3V zero = V3Zero();
	const Vec3V ab = V3Sub(b, a);
	const Vec3V ac = V3Sub(c, a);
	const Vec3V bc = V3Sub(c, b);
	const Vec3V ap = V3Sub(p, a);
	const Vec3V bp = V3Sub(p, b);
	const Vec3V cp = V3Sub(p, c);

	const FloatV d1 = V3Dot(ab, ap); //  snom
	const FloatV d2 = V3Dot(ac, ap); //  tnom
	const FloatV d3 = V3Dot(ab, bp); // -sdenom
	const FloatV d4 = V3Dot(ac, bp); //  unom = d4 - d3
	const FloatV d5 = V3Dot(ab, cp); //  udenom = d5 - d6
	const FloatV d6 = V3Dot(ac, cp); // -tdenom
	const FloatV unom = FSub(d4, d3);
	const FloatV udenom = FSub(d5, d6);
	
	//check if p in vertex region outside a
	const BoolV con00 = FIsGrtr(zero, d1); // snom <= 0
	const BoolV con01 = FIsGrtr(zero, d2); // tnom <= 0
	const BoolV con0 = BAnd(con00, con01); // vertex region a

	if(BAllEq(con0, BTTTT()))
	{
		u = zero;
		v = zero;
		const Vec3V vv = V3Sub(p, a);
		closestP = a;
		return V3Dot(vv, vv);
	}

	//check if p in vertex region outside b
	const BoolV con10 = FIsGrtrOrEq(d3, zero);
	const BoolV con11 = FIsGrtrOrEq(d3, d4);
	const BoolV con1 = BAnd(con10, con11); // vertex region b
	if(BAllEq(con1, BTTTT()))
	{
		u = one;
		v = zero;
		const Vec3V vv = V3Sub(p, b);
		closestP = b;
		return V3Dot(vv, vv);
	}

	//check if p in vertex region outside c
	const BoolV con20 = FIsGrtrOrEq(d6, zero);
	const BoolV con21 = FIsGrtrOrEq(d6, d5); 
	const BoolV con2 = BAnd(con20, con21); // vertex region c
	if(BAllEq(con2, BTTTT()))
	{
		u = zero;
		v = one;
		const Vec3V vv = V3Sub(p, c);
		closestP = c;
		return V3Dot(vv, vv);
	}

	//check if p in edge region of AB
	const FloatV vc = FSub(FMul(d1, d4), FMul(d3, d2));
	
	const BoolV con30 = FIsGrtr(zero, vc);
	const BoolV con31 = FIsGrtrOrEq(d1, zero);
	const BoolV con32 = FIsGrtr(zero, d3);
	const BoolV con3 = BAnd(con30, BAnd(con31, con32));
	if(BAllEq(con3, BTTTT()))
	{
		const FloatV sScale = FDiv(d1, FSub(d1, d3));
		const Vec3V closest3 = V3Add(a, V3Scale(ab, sScale));
		u = sScale;
		v = zero;
		const Vec3V vv = V3Sub(p, closest3);
		closestP = closest3;
		return V3Dot(vv, vv);
	}

	//check if p in edge region of BC
	const FloatV va = FSub(FMul(d3, d6),FMul(d5, d4));
	const BoolV con40 = FIsGrtr(zero, va);
	const BoolV con41 = FIsGrtrOrEq(d4, d3);
	const BoolV con42 = FIsGrtrOrEq(d5, d6);
	const BoolV con4 = BAnd(con40, BAnd(con41, con42)); 
	if(BAllEq(con4, BTTTT()))
	{
		const FloatV uScale = FDiv(unom, FAdd(unom, udenom));
		const Vec3V closest4 = V3Add(b, V3Scale(bc, uScale));
		u = FSub(one, uScale);
		v = uScale;
		const Vec3V vv = V3Sub(p, closest4);
		closestP = closest4;
		return V3Dot(vv, vv);
	}

	//check if p in edge region of AC
	const FloatV vb = FSub(FMul(d5, d2), FMul(d1, d6));
	const BoolV con50 = FIsGrtr(zero, vb);
	const BoolV con51 = FIsGrtrOrEq(d2, zero);
	const BoolV con52 = FIsGrtr(zero, d6);
	const BoolV con5 = BAnd(con50, BAnd(con51, con52));
	if(BAllEq(con5, BTTTT()))
	{
		const FloatV tScale = FDiv(d2, FSub(d2, d6));
		const Vec3V closest5 = V3Add(a, V3Scale(ac, tScale));
		u = zero;
		v = tScale;
		const Vec3V vv = V3Sub(p, closest5);
		closestP = closest5;
		return V3Dot(vv, vv);
	}

	//P must project inside face region. Compute Q using Barycentric coordinates
	const FloatV denom = FRecip(FAdd(va, FAdd(vb, vc)));
	const FloatV t = FMul(vb, denom);
	const FloatV w = FMul(vc, denom);
	const Vec3V bCom = V3Scale(ab, t);
	const Vec3V cCom = V3Scale(ac, w);
	const Vec3V closest6 = V3Add(a, V3Add(bCom, cCom));
	u = t;
	v = w;
	closestP = closest6;

	const Vec3V vv = V3Sub(p, closest6);

	return V3Dot(vv, vv);
}
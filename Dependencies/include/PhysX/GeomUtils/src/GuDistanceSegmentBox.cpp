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


#include "GuDistanceSegmentBox.h"
#include "GuDistancePointBox.h"
#include "GuDistanceSegmentSegment.h"
#include "GuDistancePointSegment.h"
#include "GuIntersectionRayBox.h"

using namespace physx;

static void face(int i0, int i1, int i2, PxVec3& rkPnt, const PxVec3& rkDir, const PxVec3& extents, const PxVec3& rkPmE, PxReal* pfLParam, PxReal& rfSqrDistance)
{
	PxVec3 kPpE;
	PxReal fLSqr, fInv, fTmp, fParam, fT, fDelta;

	kPpE[i1] = rkPnt[i1] + extents[i1];
	kPpE[i2] = rkPnt[i2] + extents[i2];
	if(rkDir[i0]*kPpE[i1] >= rkDir[i1]*rkPmE[i0])
	{
		if(rkDir[i0]*kPpE[i2] >= rkDir[i2]*rkPmE[i0])
		{
			// v[i1] >= -e[i1], v[i2] >= -e[i2] (distance = 0)
			if(pfLParam)
			{
				rkPnt[i0] = extents[i0];
				fInv = 1.0f/rkDir[i0];
				rkPnt[i1] -= rkDir[i1]*rkPmE[i0]*fInv;
				rkPnt[i2] -= rkDir[i2]*rkPmE[i0]*fInv;
				*pfLParam = -rkPmE[i0]*fInv;
			}
		}
		else
		{
			// v[i1] >= -e[i1], v[i2] < -e[i2]
			fLSqr = rkDir[i0]*rkDir[i0] + rkDir[i2]*rkDir[i2];
			fTmp = fLSqr*kPpE[i1] - rkDir[i1]*(rkDir[i0]*rkPmE[i0] + rkDir[i2]*kPpE[i2]);
			if(fTmp <= 2.0f*fLSqr*extents[i1])
			{
				fT = fTmp/fLSqr;
				fLSqr += rkDir[i1]*rkDir[i1];
				fTmp = kPpE[i1] - fT;
				fDelta = rkDir[i0]*rkPmE[i0] + rkDir[i1]*fTmp + rkDir[i2]*kPpE[i2];
				fParam = -fDelta/fLSqr;
				rfSqrDistance += rkPmE[i0]*rkPmE[i0] + fTmp*fTmp + kPpE[i2]*kPpE[i2] + fDelta*fParam;

				if(pfLParam)
				{
					*pfLParam = fParam;
					rkPnt[i0] = extents[i0];
					rkPnt[i1] = fT - extents[i1];
					rkPnt[i2] = -extents[i2];
				}
			}
			else
			{
				fLSqr += rkDir[i1]*rkDir[i1];
				fDelta = rkDir[i0]*rkPmE[i0] + rkDir[i1]*rkPmE[i1] + rkDir[i2]*kPpE[i2];
				fParam = -fDelta/fLSqr;
				rfSqrDistance += rkPmE[i0]*rkPmE[i0] + rkPmE[i1]*rkPmE[i1] + kPpE[i2]*kPpE[i2] + fDelta*fParam;

				if(pfLParam)
				{
					*pfLParam = fParam;
					rkPnt[i0] = extents[i0];
					rkPnt[i1] = extents[i1];
					rkPnt[i2] = -extents[i2];
				}
			}
		}
	}
	else
	{
		if ( rkDir[i0]*kPpE[i2] >= rkDir[i2]*rkPmE[i0] )
		{
			// v[i1] < -e[i1], v[i2] >= -e[i2]
			fLSqr = rkDir[i0]*rkDir[i0] + rkDir[i1]*rkDir[i1];
			fTmp = fLSqr*kPpE[i2] - rkDir[i2]*(rkDir[i0]*rkPmE[i0] + rkDir[i1]*kPpE[i1]);
			if(fTmp <= 2.0f*fLSqr*extents[i2])
			{
				fT = fTmp/fLSqr;
				fLSqr += rkDir[i2]*rkDir[i2];
				fTmp = kPpE[i2] - fT;
				fDelta = rkDir[i0]*rkPmE[i0] + rkDir[i1]*kPpE[i1] + rkDir[i2]*fTmp;
				fParam = -fDelta/fLSqr;
				rfSqrDistance += rkPmE[i0]*rkPmE[i0] + kPpE[i1]*kPpE[i1] + fTmp*fTmp + fDelta*fParam;

				if(pfLParam)
				{
					*pfLParam = fParam;
					rkPnt[i0] = extents[i0];
					rkPnt[i1] = -extents[i1];
					rkPnt[i2] = fT - extents[i2];
				}
			}
			else
			{
				fLSqr += rkDir[i2]*rkDir[i2];
				fDelta = rkDir[i0]*rkPmE[i0] + rkDir[i1]*kPpE[i1] + rkDir[i2]*rkPmE[i2];
				fParam = -fDelta/fLSqr;
				rfSqrDistance += rkPmE[i0]*rkPmE[i0] + kPpE[i1]*kPpE[i1] + rkPmE[i2]*rkPmE[i2] + fDelta*fParam;

				if(pfLParam)
				{
					*pfLParam = fParam;
					rkPnt[i0] = extents[i0];
					rkPnt[i1] = -extents[i1];
					rkPnt[i2] = extents[i2];
				}
			}
		}
		else
		{
			// v[i1] < -e[i1], v[i2] < -e[i2]
			fLSqr = rkDir[i0]*rkDir[i0]+rkDir[i2]*rkDir[i2];
			fTmp = fLSqr*kPpE[i1] - rkDir[i1]*(rkDir[i0]*rkPmE[i0] + rkDir[i2]*kPpE[i2]);
			if(fTmp >= 0.0f)
			{
				// v[i1]-edge is closest
				if ( fTmp <= 2.0f*fLSqr*extents[i1] )
				{
					fT = fTmp/fLSqr;
					fLSqr += rkDir[i1]*rkDir[i1];
					fTmp = kPpE[i1] - fT;
					fDelta = rkDir[i0]*rkPmE[i0] + rkDir[i1]*fTmp + rkDir[i2]*kPpE[i2];
					fParam = -fDelta/fLSqr;
					rfSqrDistance += rkPmE[i0]*rkPmE[i0] + fTmp*fTmp + kPpE[i2]*kPpE[i2] + fDelta*fParam;

					if(pfLParam)
					{
						*pfLParam = fParam;
						rkPnt[i0] = extents[i0];
						rkPnt[i1] = fT - extents[i1];
						rkPnt[i2] = -extents[i2];
					}
				}
				else
				{
					fLSqr += rkDir[i1]*rkDir[i1];
					fDelta = rkDir[i0]*rkPmE[i0] + rkDir[i1]*rkPmE[i1] + rkDir[i2]*kPpE[i2];
					fParam = -fDelta/fLSqr;
					rfSqrDistance += rkPmE[i0]*rkPmE[i0] + rkPmE[i1]*rkPmE[i1] + kPpE[i2]*kPpE[i2] + fDelta*fParam;

					if(pfLParam)
					{
						*pfLParam = fParam;
						rkPnt[i0] = extents[i0];
						rkPnt[i1] = extents[i1];
						rkPnt[i2] = -extents[i2];
					}
				}
				return;
			}

			fLSqr = rkDir[i0]*rkDir[i0] + rkDir[i1]*rkDir[i1];
			fTmp = fLSqr*kPpE[i2] - rkDir[i2]*(rkDir[i0]*rkPmE[i0] + rkDir[i1]*kPpE[i1]);
			if(fTmp >= 0.0f)
			{
				// v[i2]-edge is closest
				if(fTmp <= 2.0f*fLSqr*extents[i2])
				{
					fT = fTmp/fLSqr;
					fLSqr += rkDir[i2]*rkDir[i2];
					fTmp = kPpE[i2] - fT;
					fDelta = rkDir[i0]*rkPmE[i0] + rkDir[i1]*kPpE[i1] + rkDir[i2]*fTmp;
					fParam = -fDelta/fLSqr;
					rfSqrDistance += rkPmE[i0]*rkPmE[i0] + kPpE[i1]*kPpE[i1] + fTmp*fTmp + fDelta*fParam;

					if(pfLParam)
					{
						*pfLParam = fParam;
						rkPnt[i0] = extents[i0];
						rkPnt[i1] = -extents[i1];
						rkPnt[i2] = fT - extents[i2];
					}
				}
				else
				{
					fLSqr += rkDir[i2]*rkDir[i2];
					fDelta = rkDir[i0]*rkPmE[i0] + rkDir[i1]*kPpE[i1] + rkDir[i2]*rkPmE[i2];
					fParam = -fDelta/fLSqr;
					rfSqrDistance += rkPmE[i0]*rkPmE[i0] + kPpE[i1]*kPpE[i1] + rkPmE[i2]*rkPmE[i2] + fDelta*fParam;

					if(pfLParam)
					{
						*pfLParam = fParam;
						rkPnt[i0] = extents[i0];
						rkPnt[i1] = -extents[i1];
						rkPnt[i2] = extents[i2];
					}
				}
				return;
			}

			// (v[i1],v[i2])-corner is closest
			fLSqr += rkDir[i2]*rkDir[i2];
			fDelta = rkDir[i0]*rkPmE[i0] + rkDir[i1]*kPpE[i1] + rkDir[i2]*kPpE[i2];
			fParam = -fDelta/fLSqr;
			rfSqrDistance += rkPmE[i0]*rkPmE[i0] + kPpE[i1]*kPpE[i1] + kPpE[i2]*kPpE[i2] + fDelta*fParam;

			if(pfLParam)
			{
				*pfLParam = fParam;
				rkPnt[i0] = extents[i0];
				rkPnt[i1] = -extents[i1];
				rkPnt[i2] = -extents[i2];
			}
		}
	}
}

static void caseNoZeros(PxVec3& rkPnt, const PxVec3& rkDir, const PxVec3& extents, PxReal* pfLParam, PxReal& rfSqrDistance)
{
	PxVec3 kPmE(rkPnt.x - extents.x, rkPnt.y - extents.y, rkPnt.z - extents.z);

	PxReal fProdDxPy, fProdDyPx, fProdDzPx, fProdDxPz, fProdDzPy, fProdDyPz;

	fProdDxPy = rkDir.x*kPmE.y;
	fProdDyPx = rkDir.y*kPmE.x;
	if(fProdDyPx >= fProdDxPy)
	{
		fProdDzPx = rkDir.z*kPmE.x;
		fProdDxPz = rkDir.x*kPmE.z;
		if(fProdDzPx >= fProdDxPz)
		{
			// line intersects x = e0
			face(0, 1, 2, rkPnt, rkDir, extents, kPmE, pfLParam, rfSqrDistance);
		}
		else
		{
			// line intersects z = e2
			face(2, 0, 1, rkPnt, rkDir, extents, kPmE, pfLParam, rfSqrDistance);
		}
	}
	else
	{
		fProdDzPy = rkDir.z*kPmE.y;
		fProdDyPz = rkDir.y*kPmE.z;
		if(fProdDzPy >= fProdDyPz)
		{
			// line intersects y = e1
			face(1, 2, 0, rkPnt, rkDir, extents, kPmE, pfLParam, rfSqrDistance);
		}
		else
		{
			// line intersects z = e2
			face(2, 0, 1, rkPnt, rkDir, extents, kPmE, pfLParam, rfSqrDistance);
		}
	}
}

static void case0(int i0, int i1, int i2, PxVec3& rkPnt, const PxVec3& rkDir, const PxVec3& extents, PxReal* pfLParam, PxReal& rfSqrDistance)
{
	PxReal fPmE0 = rkPnt[i0] - extents[i0];
	PxReal fPmE1 = rkPnt[i1] - extents[i1];
	PxReal fProd0 = rkDir[i1]*fPmE0;
	PxReal fProd1 = rkDir[i0]*fPmE1;
	PxReal fDelta, fInvLSqr, fInv;

	if(fProd0 >= fProd1)
	{
		// line intersects P[i0] = e[i0]
		rkPnt[i0] = extents[i0];

		PxReal fPpE1 = rkPnt[i1] + extents[i1];
		fDelta = fProd0 - rkDir[i0]*fPpE1;
		if(fDelta >= 0.0f)
		{
			fInvLSqr = 1.0f/(rkDir[i0]*rkDir[i0] + rkDir[i1]*rkDir[i1]);
			rfSqrDistance += fDelta*fDelta*fInvLSqr;
			if(pfLParam)
			{
				rkPnt[i1] = -extents[i1];
				*pfLParam = -(rkDir[i0]*fPmE0+rkDir[i1]*fPpE1)*fInvLSqr;
			}
		}
		else
		{
			if(pfLParam)
			{
				fInv = 1.0f/rkDir[i0];
				rkPnt[i1] -= fProd0*fInv;
				*pfLParam = -fPmE0*fInv;
			}
		}
	}
	else
	{
		// line intersects P[i1] = e[i1]
		rkPnt[i1] = extents[i1];

		PxReal fPpE0 = rkPnt[i0] + extents[i0];
		fDelta = fProd1 - rkDir[i1]*fPpE0;
		if(fDelta >= 0.0f)
		{
			fInvLSqr = 1.0f/(rkDir[i0]*rkDir[i0] + rkDir[i1]*rkDir[i1]);
			rfSqrDistance += fDelta*fDelta*fInvLSqr;
			if(pfLParam)
			{
				rkPnt[i0] = -extents[i0];
				*pfLParam = -(rkDir[i0]*fPpE0+rkDir[i1]*fPmE1)*fInvLSqr;
			}
		}
		else
		{
			if(pfLParam)
			{
				fInv = 1.0f/rkDir[i1];
				rkPnt[i0] -= fProd1*fInv;
				*pfLParam = -fPmE1*fInv;
			}
		}
	}

	if(rkPnt[i2] < -extents[i2])
	{
		fDelta = rkPnt[i2] + extents[i2];
		rfSqrDistance += fDelta*fDelta;
		rkPnt[i2] = -extents[i2];
	}
	else if ( rkPnt[i2] > extents[i2] )
	{
		fDelta = rkPnt[i2] - extents[i2];
		rfSqrDistance += fDelta*fDelta;
		rkPnt[i2] = extents[i2];
	}
}

static void case00(int i0, int i1, int i2, PxVec3& rkPnt, const PxVec3& rkDir, const PxVec3& extents, PxReal* pfLParam, PxReal& rfSqrDistance)
{
	PxReal fDelta;

	if(pfLParam)
		*pfLParam = (extents[i0] - rkPnt[i0])/rkDir[i0];

	rkPnt[i0] = extents[i0];

	if(rkPnt[i1] < -extents[i1])
	{
		fDelta = rkPnt[i1] + extents[i1];
		rfSqrDistance += fDelta*fDelta;
		rkPnt[i1] = -extents[i1];
	}
	else if(rkPnt[i1] > extents[i1])
	{
		fDelta = rkPnt[i1] - extents[i1];
		rfSqrDistance += fDelta*fDelta;
		rkPnt[i1] = extents[i1];
	}

	if(rkPnt[i2] < -extents[i2])
	{
		fDelta = rkPnt[i2] + extents[i2];
		rfSqrDistance += fDelta*fDelta;
		rkPnt[i2] = -extents[i2];
	}
	else if(rkPnt[i2] > extents[i2])
	{
		fDelta = rkPnt[i2] - extents[i2];
		rfSqrDistance += fDelta*fDelta;
		rkPnt[i2] = extents[i2];
	}
}

static void case000(PxVec3& rkPnt, const PxVec3& extents, PxReal& rfSqrDistance)
{
	PxReal fDelta;

	if(rkPnt.x < -extents.x)
	{
		fDelta = rkPnt.x + extents.x;
		rfSqrDistance += fDelta*fDelta;
		rkPnt.x = -extents.x;
	}
	else if(rkPnt.x > extents.x)
	{
		fDelta = rkPnt.x - extents.x;
		rfSqrDistance += fDelta*fDelta;
		rkPnt.x = extents.x;
	}

	if(rkPnt.y < -extents.y)
	{
		fDelta = rkPnt.y + extents.y;
		rfSqrDistance += fDelta*fDelta;
		rkPnt.y = -extents.y;
	}
	else if(rkPnt.y > extents.y)
	{
		fDelta = rkPnt.y - extents.y;
		rfSqrDistance += fDelta*fDelta;
		rkPnt.y = extents.y;
	}

	if(rkPnt.z < -extents.z)
	{
		fDelta = rkPnt.z + extents.z;
		rfSqrDistance += fDelta*fDelta;
		rkPnt.z = -extents.z;
	}
	else if(rkPnt.z > extents.z)
	{
		fDelta = rkPnt.z - extents.z;
		rfSqrDistance += fDelta*fDelta;
		rkPnt.z = extents.z;
	}
}

//! Compute the smallest distance from the (infinite) line to the box.
static PxReal distanceLineBoxSquared(const PxVec3& lineOrigin, const PxVec3& lineDirection,
								  const PxVec3& boxOrigin, const PxVec3& boxExtent, const PxMat33& boxBase,
								  PxReal* lineParam,
								  PxVec3* boxParam)
{
	const PxVec3& axis0 = boxBase.column0;
	const PxVec3& axis1 = boxBase.column1;
	const PxVec3& axis2 = boxBase.column2;
	
	// compute coordinates of line in box coordinate system
	PxVec3 diff = lineOrigin - boxOrigin;
	PxVec3 pnt(diff.dot(axis0), diff.dot(axis1), diff.dot(axis2));
	PxVec3 dir(lineDirection.dot(axis0), lineDirection.dot(axis1), lineDirection.dot(axis2));

	// Apply reflections so that direction vector has nonnegative components.
	bool reflect[3];
	for(int i=0;i<3;i++)
	{
		if(dir[i]<0.0f)
		{
			pnt[i] = -pnt[i];
			dir[i] = -dir[i];
			reflect[i] = true;
		}
		else
		{
			reflect[i] = false;
		}
	}

	PxReal sqrDistance = 0.0f;

	if(dir.x>0.0f)
	{
		if(dir.y>0.0f)
		{
			if(dir.z>0.0f)	caseNoZeros(pnt, dir, boxExtent, lineParam, sqrDistance);		// (+,+,+)
			else			case0(0, 1, 2, pnt, dir, boxExtent, lineParam, sqrDistance);	// (+,+,0)
		}
		else
		{
			if(dir.z>0.0f)	case0(0, 2, 1, pnt, dir, boxExtent, lineParam, sqrDistance);	// (+,0,+)
			else			case00(0, 1, 2, pnt, dir, boxExtent, lineParam, sqrDistance);	// (+,0,0)
		}
	}
	else
	{
		if(dir.y>0.0f)
		{
			if(dir.z>0.0f)	case0(1, 2, 0, pnt, dir, boxExtent, lineParam, sqrDistance);	// (0,+,+)
			else			case00(1, 0, 2, pnt, dir, boxExtent, lineParam, sqrDistance);	// (0,+,0)
		}
		else
		{
			if(dir.z>0.0f)	case00(2, 0, 1, pnt, dir, boxExtent, lineParam, sqrDistance);	// (0,0,+)
			else
			{
				case000(pnt, boxExtent, sqrDistance);										// (0,0,0)
				if(lineParam) *lineParam = 0.0f;
			}
		}
	}

	if(boxParam)
	{
		// undo reflections
		for(int i=0;i<3;i++)
		{
			if(reflect[i]) pnt[i] = -pnt[i];
		}

		*boxParam = pnt;
	}

	return sqrDistance;
}

//! Compute the smallest distance from the (finite) line segment to the box.
PxReal Gu::distanceSegmentBoxSquared(	const PxVec3& segmentPoint0, const PxVec3& segmentPoint1,
										const PxVec3& boxOrigin, const PxVec3& boxExtent, const PxMat33& boxBase,
										PxReal* segmentParam,
										PxVec3* boxParam)
{
	// compute coordinates of line in box coordinate system

	PxReal lp;
	PxVec3 bp;
	PxReal sqrDistance = distanceLineBoxSquared(segmentPoint0, segmentPoint1 - segmentPoint0, boxOrigin, boxExtent, boxBase, &lp, &bp);
	if(lp>=0.0f)
	{
		if(lp<=1.0f)
		{
			if(segmentParam) *segmentParam=lp;
			if(boxParam) *boxParam=bp;
			return sqrDistance;
		}
		else
		{
			if(segmentParam) *segmentParam=1.0f;
			return Gu::distancePointBoxSquared(segmentPoint1, boxOrigin, boxExtent, boxBase, boxParam);
		}
	}
	else
	{
		if(segmentParam) *segmentParam = 0.0f;
		return Gu::distancePointBoxSquared(segmentPoint0, boxOrigin, boxExtent, boxBase, boxParam);
	}
}


//Ps::aos::FloatV Gu::distanceSegmentBoxSquared(	const Ps::aos::Vec3VArg segmentPoint0, const Ps::aos::Vec3VArg segmentPoint1,
//												const Ps::aos::Vec3VArg boxOrigin, const Ps::aos::Vec3VArg boxExtent, const Ps::aos::Mat33V& boxBase,
//												Ps::aos::FloatV& segmentParam,
//												Ps::aos::Vec3V& localSpaceClosestP)
//{
//	
//	using namespace Ps::aos;
//	const FloatV zero = FZero();
//	const FloatV one = FOne();
//	const Vec3V eps = V3Eps();
//
//	const Vec3V unitX(V3UnitX());
//	const Vec3V unitY(V3UnitY());
//	const Vec3V unitZ(V3UnitZ());
//	//translate the segment into the box local space
//	const Vec3V saT = V3Sub(segmentPoint0, boxOrigin);
//	const Vec3V sbT = V3Sub(segmentPoint1, boxOrigin);
//	const Vec3V sa = M33TrnspsMulV3(boxBase, saT);
//	const Vec3V sb = M33TrnspsMulV3(boxBase, sbT);
//	const Vec3V sab = V3Sub(sb, sa);
//	const Vec3V sabRecip = V3Recip(sab);
//	const Vec3V nBoxExtent =  V3Neg(boxExtent);
//
//	const Vec3V eX(V3Mul(V3GetX(boxExtent), unitX));
//	const Vec3V eY(V3Mul(V3GetY(boxExtent), unitY));
//	const Vec3V eZ(V3Mul(V3GetZ(boxExtent), unitZ));
//
//	const Vec3V xy = V3Add(eX, eY);
//	const Vec3V xz = V3Add(eX, eZ);
//	const Vec3V yz = V3Add(eY, eZ);
//	const Vec3V a = boxExtent;		//[ ex,  ey,  ez]
//	const Vec3V b = V3Sub(xy, eZ);	//[ ex,  ey, -ez]
//	const Vec3V c = V3Sub(xz, eY);	//[ ex, -ey,  ez]
//	const Vec3V d = V3Sub(eX, yz);	//[ ex, -ey, -ez]
//	const Vec3V e = V3Sub(yz, eX);	//[-ex,  ey,  ez]
//	const Vec3V f = V3Sub(eY, xz);	//[-ex,  ey, -ez]
//	const Vec3V g = V3Sub(eZ, xy);	//[-ex, -ey,  ez]
//	const Vec3V h = nBoxExtent;//[-ex, -ey, -ez]
//
//	const Vec3V ab = FSub(b, a);//ef, cd, gh should be the same
//	const Vec3V ac = FSub(c, a); // bd, eg, fh should be the same
//	const Vec3V ae = FSub(e, a);//bf, cg, dh should be the same
//
//	const Vec3V closestPP0 = V3Clamp(sa, nBoxExtent, boxExtent);
//	const Vec3V closestPP1 = V3Clamp(sb, nBoxExtent, boxExtent);
//	//compute the sqDist to the box
//	FloatV g0, g1;
//	const FloatV sqDistP0 = Gu::distancePointSegmentSquared(sa, sb, closestPP0, g0);
//	const FloatV sqDistP1 = Gu::distancePointSegmentSquared(sa, sb, closestPP1, g1);
//
//	const BoolV conPP = FIsGrtr(sqDistP1, sqDistP0);
//	const FloatV sqDistPP = FSel(conPP, sqDistP0, sqDistP1);
//	const Vec3V closestPP = V3Sel(conPP, closestPP0, closestPP1);
//	const FloatV segPP = FSel(conPP, g0, g1);
//
//	
//	//check for parallel
//	const Vec3V absD = V3Abs(sab);
//
//	const BoolV bParallel = V3IsGrtrOrEq(eps, absD);
//
//	const BoolV bOutOfRange = V3IsGrtr(V3Abs(sa), boxExtent);
//	const BoolV bParallelAndOutOfRange(BAnd(bParallel, bOutOfRange));
//	const BoolV bMiss(BAnyTrue3(bParallelAndOutOfRange));
//
//	const Vec3V _tSlab0 = V3Mul(V3Sub(nBoxExtent, sa), sabRecip);
//	const Vec3V _tSlab1 = V3Mul(V3Sub(boxExtent, sa), sabRecip);
//
//	const Vec3V tSlab0 = V3Min(_tSlab0, _tSlab1);
//	const Vec3V tSlab1 = V3Max(_tSlab0, _tSlab1);
//
//
//	const FloatV tVal = FMax(V3GetX(tSlab0), FMax(V3GetY(tSlab0), V3GetZ(tSlab0)));
//	const FloatV tVal2 = FMin(V3GetX(tSlab1), FMin(V3GetY(tSlab1), V3GetZ(tSlab1)));
//
//	const BoolV bHitSlab(BAnd(BAnd(FIsGrtrOrEq(tVal2, tVal), FIsGrtrOrEq(one, tVal)), FIsGrtrOrEq(tVal2, zero)));
//
//	const BoolV bHitOrbMissParallel(BOr(bHitSlab, bMiss));
//
//	PxU32 tCon;
//	Store_From_BoolV(bHitOrbMissParallel, &tCon);
//	
//	if(tCon)//intersect
//	{
//		localSpaceClosestP = V3Sel(bMiss, closestPP, V3MulAdd(sab, tVal,sa));
//		segmentParam = FSel(bMiss, segPP, tVal);
//		return FSel(bMiss, sqDistPP, zero);
//	}
//
//	
//	Vec4V s0, s1, s2, t0, t1, t2;
//
//	const Vec4V sqDist0 = distanceSegmentSegmentSquared4(sa, sab, a, c, e, g, ab, s0, t0); //ab, cd, ef, gh
//	const Vec4V sqDist1 = distanceSegmentSegmentSquared4(sa, sab, a, b, e, f, ac, s1, t1); //ac, bd, eg, fh
//	const Vec4V sqDist2 = distanceSegmentSegmentSquared4(sa, sab, a, b, c, d, ae, s2, t2); //ae, bf, cg, dh
//
//	Vec3V closest[3];
//	Vec3V sqDist = V3Zero();
//	Vec3V s = V3Zero();
//	{
//		const FloatV sx0 = V4GetX(sqDist0);
//		const FloatV sx1 = V4GetY(sqDist0);
//		const FloatV sx2 = V4GetZ(sqDist0);
//		const FloatV sx3 = V4GetW(sqDist0);
//
//		const Vec3V closest0 = V3MulAdd(ab, V4GetX(t0), a);
//		const Vec3V closest1 = V3MulAdd(ab, V4GetY(t0), c);
//		const Vec3V closest2 = V3MulAdd(ab, V4GetZ(t0), e);
//		const Vec3V closest3 = V3MulAdd(ab, V4GetW(t0), g);
//
//		const BoolV con0 = BAllTrue4(V4IsGrtrOrEq(sqDist0, sx0));
//		const BoolV con1 = BAllTrue4(V4IsGrtrOrEq(sqDist0, sx1));
//		const BoolV con2 = BAllTrue4(V4IsGrtrOrEq(sqDist0, sx2)); 
//
//		closest[0] = V3Sel(con0, closest0, V3Sel(con1, closest1, V3Sel(con2, closest2, closest3)));
//		sqDist = V3SetX(sqDist, FSel(con0, sx0, FSel(con1, sx1, FSel(con2, sx2, sx3))));
//		s = V3SetX(s, FSel(con0, V4GetX(s0), FSel(con1, V4GetY(s0), FSel(con2, V4GetZ(s0), V4GetW(s0)))));
//	}
//
//
//	{
//		
//		const FloatV sx0 = V4GetX(sqDist1);
//		const FloatV sx1 = V4GetY(sqDist1);
//		const FloatV sx2 = V4GetZ(sqDist1);
//		const FloatV sx3 = V4GetW(sqDist1);
//
//		const Vec3V closest0 = V3MulAdd(ac, V4GetX(t1), a);
//		const Vec3V closest1 = V3MulAdd(ac, V4GetY(t1), b);
//		const Vec3V closest2 = V3MulAdd(ac, V4GetZ(t1), e);
//		const Vec3V closest3 = V3MulAdd(ac, V4GetW(t1), f);
//
//		const BoolV con0 = BAllTrue4(V4IsGrtrOrEq(sqDist1, sx0));
//		const BoolV con1 = BAllTrue4(V4IsGrtrOrEq(sqDist1, sx1));
//		const BoolV con2 = BAllTrue4(V4IsGrtrOrEq(sqDist1, sx2));
//
//		closest[1] = V3Sel(con0, closest0, V3Sel(con1, closest1, V3Sel(con2, closest2, closest3)));
//		sqDist = V3SetY(sqDist, FSel(con0, sx0, FSel(con1, sx1, FSel(con2, sx2, sx3))));
//		s = V3SetY(s, FSel(con0, V4GetX(s1), FSel(con1, V4GetY(s1), FSel(con2, V4GetZ(s1), V4GetW(s1)))));
//		
//	}
//
//
//	{
//		
//		const FloatV sx0 = V4GetX(sqDist2);
//		const FloatV sx1 = V4GetY(sqDist2);
//		const FloatV sx2 = V4GetZ(sqDist2);
//		const FloatV sx3 = V4GetW(sqDist2);
//
//		const Vec3V closest0 = V3MulAdd(ae, V4GetX(t2), a);
//		const Vec3V closest1 = V3MulAdd(ae, V4GetY(t2), b);
//		const Vec3V closest2 = V3MulAdd(ae, V4GetZ(t2), c);
//		const Vec3V closest3 = V3MulAdd(ae, V4GetW(t2), d);
//
//		const BoolV con0 = BAllTrue4(V4IsGrtrOrEq(sqDist2, sx0));
//		const BoolV con1 = BAllTrue4(V4IsGrtrOrEq(sqDist2, sx1));
//		const BoolV con2 = BAllTrue4(V4IsGrtrOrEq(sqDist2, sx2));
//
//		closest[2] = V3Sel(con0, closest0, V3Sel(con1, closest1, V3Sel(con2, closest2, closest3)));
//		sqDist = V3SetZ(sqDist, FSel(con0, sx0, FSel(con1, sx1, FSel(con2, sx2, sx3))));
//		s = V3SetZ(s,FSel(con0, V4GetX(s2), FSel(con1, V4GetY(s2), FSel(con2, V4GetZ(s2), V4GetW(s2)))));
//	}
//
//	//compute the closest point of segment to edges
//	const FloatV sq0 = V3GetX(sqDist);
//	const FloatV sq1 = V3GetY(sqDist);
//	const BoolV con0 = BAllTrue3(V3IsGrtrOrEq(sqDist, sq0));
//	const BoolV con1 = BAllTrue3(V3IsGrtrOrEq(sqDist, sq1));
//	const Vec3V closestPE = V3Sel(con0, closest[0], V3Sel(con1, closest[1], closest[2]));
//	const FloatV sqDistPE = FSel(con0, sq0, FSel(con1, sq1, V3GetZ(sqDist)) );
//	const FloatV sPE = FSel(con0, V3GetX(s), FSel(con1, V3GetY(s), V3GetZ(s)) );
//	
//	const BoolV c0 = FIsGrtr(sqDistPP, sqDistPE);
//	const FloatV sdist = FSel(c0, sqDistPE, sqDistPP);
//	segmentParam = FSel(c0, sPE, segPP);
//	localSpaceClosestP = V3Sel(c0, closestPE,closestPP);
//	
//	return sdist;
//}

Ps::aos::FloatV Gu::distanceSegmentBoxSquared(	const Ps::aos::Vec3VArg segmentPoint0, const Ps::aos::Vec3VArg segmentPoint1,
												const Ps::aos::Vec3VArg boxOrigin, const Ps::aos::Vec3VArg boxExtent, const Ps::aos::Mat33V& boxBase,
												Ps::aos::FloatV& segmentParam,
												Ps::aos::Vec3V& localSpaceClosestP)
{
	
	using namespace Ps::aos;
	const FloatV zero = FZero();
	const FloatV one = FOne();
	const FloatV eps = FEps();

	const Vec3V unitX(V3UnitX());
	const Vec3V unitY(V3UnitY());
	const Vec3V unitZ(V3UnitZ());
	//translate the segment into the box local space
	const Vec3V saT = V3Sub(segmentPoint0, boxOrigin);
	const Vec3V sbT = V3Sub(segmentPoint1, boxOrigin);
	const Vec3V sa = M33TrnspsMulV3(boxBase, saT);
	const Vec3V sb = M33TrnspsMulV3(boxBase, sbT);
	const Vec3V sab = V3Sub(sb, sa);
	const Vec3V sabRecip = V3Recip(sab);
	const Vec3V nBoxExtent =  V3Neg(boxExtent);

	const Vec3V eX(V3Mul(V3GetX(boxExtent), unitX));
	const Vec3V eY(V3Mul(V3GetY(boxExtent), unitY));
	const Vec3V eZ(V3Mul(V3GetZ(boxExtent), unitZ));

	const Vec3V xy = V3Add(eX, eY);
	const Vec3V xz = V3Add(eX, eZ);
	const Vec3V yz = V3Add(eY, eZ);
	const Vec3V a = boxExtent;		//[ ex,  ey,  ez]
	const Vec3V b = V3Sub(xy, eZ);	//[ ex,  ey, -ez]
	const Vec3V c = V3Sub(xz, eY);	//[ ex, -ey,  ez]
	const Vec3V d = V3Sub(eX, yz);	//[ ex, -ey, -ez]
	const Vec3V e = V3Sub(yz, eX);	//[-ex,  ey,  ez]
	const Vec3V f = V3Sub(eY, xz);	//[-ex,  ey, -ez]
	const Vec3V g = V3Sub(eZ, xy);	//[-ex, -ey,  ez]
	//const Vec3V h = nBoxExtent;//[-ex, -ey, -ez]

	const Vec3V ab = V3Sub(b, a);//ef, cd, gh should be the same
	const Vec3V ac = V3Sub(c, a); // bd, eg, fh should be the same
	const Vec3V ae = V3Sub(e, a);//bf, cg, dh should be the same

	const Vec3V closestPP0 = V3Clamp(sa, nBoxExtent, boxExtent);
	const Vec3V closestPP1 = V3Clamp(sb, nBoxExtent, boxExtent);
	//compute the sqDist to the box
	/*FloatV g0, g1;
	const FloatV sqDistP0 = Gu::distancePointSegmentSquared(sa, sb, closestPP0, g0);
	const FloatV sqDistP1 = Gu::distancePointSegmentSquared(sa, sb, closestPP1, g1);*/
	const Vec3V v0 = V3Sub(closestPP0, sa);
	const Vec3V v1 = V3Sub(closestPP1, sb);

	const FloatV sqDistP0 = V3Dot(v0, v0);
	const FloatV sqDistP1 = V3Dot(v1, v1);

	const BoolV conPP = FIsGrtr(sqDistP1, sqDistP0);
	const FloatV sqDistPP = FSel(conPP, sqDistP0, sqDistP1);
	const Vec3V closestPP = V3Sel(conPP, closestPP0, closestPP1);
	const FloatV segPP = FSel(conPP, zero, one);

	
	//check for parallel
	const Vec3V absD = V3Abs(sab);
	const BoolV bParallel = V3IsGrtrOrEq(eps, absD);
	const BoolV bOutOfRange = V3IsGrtr(V3Abs(sa), boxExtent);
	const BoolV bParallelAndOutOfRange(BAnd(bParallel, bOutOfRange));

	if(BAllEq(bParallelAndOutOfRange, BTTTT()))
	{
		localSpaceClosestP =  closestPP;
		segmentParam = segPP;
		return sqDistPP;
	}

	//check for intersect
	const Vec3V _tSlab0 = V3Mul(V3Sub(nBoxExtent, sa), sabRecip);
	const Vec3V _tSlab1 = V3Mul(V3Sub(boxExtent, sa), sabRecip);
	const Vec3V tSlab0 = V3Min(_tSlab0, _tSlab1);
	const Vec3V tSlab1 = V3Max(_tSlab0, _tSlab1);

	const FloatV tNear = FMax(V3GetX(tSlab0), FMax(V3GetY(tSlab0), V3GetZ(tSlab0)));
	const FloatV tFar = FMin(V3GetX(tSlab1), FMin(V3GetY(tSlab1), V3GetZ(tSlab1)));

	//const BoolV bHitSlab(BAnd(BAnd(FIsGrtrOrEq(tFar, tNear), FIsGrtrOrEq(one, tNear)), FIsGrtrOrEq(tFar, zero)));
	const BoolV bHitSlab(BAnd(FIsGrtrOrEq(tFar, tNear), FIsGrtrOrEq(tFar, eps)));

	if(BAllEq(bHitSlab, BTTTT()))
	{
		const Vec3V nearP = V3MulAdd(sab, tNear,sa);
		const Vec3V farP = V3MulAdd(sab, tFar, sa);
		const BoolV bNearLessZero = FIsGrtr(zero, tNear);
		const BoolV bNearGrtrOne = FIsGrtr(tNear, one);
		localSpaceClosestP = V3Sel(bNearLessZero, farP, V3Sel(bNearGrtrOne, closestPP1, nearP));
		segmentParam = FSel(bNearLessZero, tFar, FSel(bNearGrtrOne, one, tNear));
		return FSel(bNearGrtrOne, sqDistP1, zero);
	}



	FloatV s0[4], s1[4], s2[4];
	FloatV t0[4], t1[4], t2[4];
	FloatV sqDist0[4], sqDist1[4], sqDist2[4];
	Vec3V closest0[4], closest1[4], closest2[4];
	distanceSegmentSegmentSquared4(sa, sab, a, c, e, g, ab, s0, t0, closest0, sqDist0); //ab, cd, ef, gh
	distanceSegmentSegmentSquared4(sa, sab, a, b, e, f, ac, s1, t1, closest1, sqDist1); //ac, bd, eg, fh
	distanceSegmentSegmentSquared4(sa, sab, a, b, c, d, ae, s2, t2, closest2, sqDist2); //ae, bf, cg, dh

	Vec3V closest[3];
	FloatV sqDist[3];
	FloatV s[3];
	{
		const Vec4V sqDistv = V4Merge(sqDist0); 
		const BoolV con0 = BAllTrue4(V4IsGrtrOrEq(sqDistv, sqDist0[0]));
		const BoolV con1 = BAllTrue4(V4IsGrtrOrEq(sqDistv, sqDist0[1]));
		const BoolV con2 = BAllTrue4(V4IsGrtrOrEq(sqDistv, sqDist0[2])); 

		closest[0] = V3Sel(con0, closest0[0], V3Sel(con1, closest0[1], V3Sel(con2, closest0[2], closest0[3])));
		sqDist[0] =FSel(con0, sqDist0[0], FSel(con1, sqDist0[1], FSel(con2,  sqDist0[2],  sqDist0[3])));
		s[0] = FSel(con0, s0[0], FSel(con1, s0[1], FSel(con2, s0[2], s0[3])));
	}

	{

		const Vec4V sqDistv = V4Merge(sqDist1); 
		const BoolV con0 = BAllTrue4(V4IsGrtrOrEq(sqDistv, sqDist1[0]));
		const BoolV con1 = BAllTrue4(V4IsGrtrOrEq(sqDistv, sqDist1[1]));
		const BoolV con2 = BAllTrue4(V4IsGrtrOrEq(sqDistv, sqDist1[2]));

		closest[1] = V3Sel(con0, closest1[0], V3Sel(con1, closest1[1], V3Sel(con2, closest1[2], closest1[3])));
		sqDist[1] = FSel(con0, sqDist1[0], FSel(con1, sqDist1[1], FSel(con2, sqDist1[2], sqDist1[3])));
		s[1] = FSel(con0, s1[0], FSel(con1, s1[1], FSel(con2, s1[2], s1[3])));
		
	}

	{

		const Vec4V sqDistv = V4Merge(sqDist2); 
		const BoolV con0 = BAllTrue4(V4IsGrtrOrEq(sqDistv, sqDist2[0]));
		const BoolV con1 = BAllTrue4(V4IsGrtrOrEq(sqDistv, sqDist2[1]));
		const BoolV con2 = BAllTrue4(V4IsGrtrOrEq(sqDistv, sqDist2[2]));

		closest[2] = V3Sel(con0, closest2[0], V3Sel(con1, closest2[1], V3Sel(con2, closest2[2], closest2[3])));
		sqDist[2] = FSel(con0, sqDist2[0], FSel(con1, sqDist2[1], FSel(con2, sqDist2[2], sqDist2[3])));
		s[2] = FSel(con0, s2[0], FSel(con1, s2[1], FSel(con2, s2[2], s2[3])));
	}

	//compute the closest point of segment to edges
	const Vec3V dist = V3Merge(sqDist[0], sqDist[1], sqDist[2]);
	const BoolV con0 = BAllTrue3(V3IsGrtrOrEq(dist, sqDist[0]));
	const BoolV con1 = BAllTrue3(V3IsGrtrOrEq(dist, sqDist[1]));
	const Vec3V closestPE = V3Sel(con0, closest[0], V3Sel(con1, closest[1], closest[2]));
	const FloatV sqDistPE = FSel(con0, sqDist[0], FSel(con1, sqDist[1], sqDist[2]) );
	const FloatV sPE = FSel(con0, s[0], FSel(con1, s[1], s[2]) );
	
	const BoolV c0 = FIsGrtr(sqDistPP, sqDistPE);
	const FloatV sdist = FSel(c0, sqDistPE, sqDistPP);
	segmentParam = FSel(c0, sPE, segPP);
	localSpaceClosestP = V3Sel(c0, closestPE,closestPP);
	
	return sdist;
}



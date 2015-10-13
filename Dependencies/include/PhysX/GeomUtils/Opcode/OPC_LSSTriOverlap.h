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


// Following code from Magic-Software (http://www.magic-software.com/)
// A bit modified for Opcode

#include "Opcode.h"

namespace physx
{
namespace Ice
{

static float OPC_PointTriangleSqrDist(const PxVec3& point, const PxVec3& p0, const PxVec3& p1, const PxVec3& p2)
{
	// Hook
	PxVec3 TriEdge0 = p1 - p0;
	PxVec3 TriEdge1 = p2 - p0;

	PxVec3 kDiff	= p0 - point;
	float fA00	= TriEdge0.magnitudeSquared();
	float fA01	= TriEdge0.dot(TriEdge1);
	float fA11	= TriEdge1.magnitudeSquared();
	float fB0	= kDiff.dot(TriEdge0);
	float fB1	= kDiff.dot(TriEdge1);
	float fC	= kDiff.magnitudeSquared();
	float fDet	= PxAbs(fA00*fA11 - fA01*fA01);
	float fS	= fA01*fB1-fA11*fB0;
	float fT	= fA01*fB0-fA00*fB1;
	float fSqrDist;

	if(fS + fT <= fDet)
	{
		if(fS < 0.0f)
		{
			if(fT < 0.0f)  // region 4
			{
				if(fB0 < 0.0f)
				{
					if(-fB0 >= fA00)		fSqrDist = fA00+2.0f*fB0+fC;
					else					fSqrDist = fB0*(-fB0/fA00)+fC;
				}
				else
				{
					if(fB1 >= 0.0f)			fSqrDist = fC;
					else if(-fB1 >= fA11)	fSqrDist = fA11+2.0f*fB1+fC;
					else					fSqrDist = fB1*(-fB1/fA11)+fC;
				}
			}
			else  // region 3
			{
				if(fB1 >= 0.0f)				fSqrDist = fC;
				else if(-fB1 >= fA11)		fSqrDist = fA11+2.0f*fB1+fC;
				else						fSqrDist = fB1*(-fB1/fA11)+fC;
			}
		}
		else if(fT < 0.0f)  // region 5
		{
			if(fB0 >= 0.0f)					fSqrDist = fC;
			else if(-fB0 >= fA00)			fSqrDist = fA00+2.0f*fB0+fC;
			else							fSqrDist = fB0*(-fB0/fA00)+fC;
		}
		else  // region 0
		{
			// minimum at interior point
			if(fDet==0.0f)
			{
				fSqrDist = PX_MAX_F32;
			}
			else
			{
				float fInvDet = 1.0f/fDet;
				fS *= fInvDet;
				fT *= fInvDet;
				fSqrDist = fS*(fA00*fS+fA01*fT+2.0f*fB0) + fT*(fA01*fS+fA11*fT+2.0f*fB1)+fC;
			}
		}
	}
	else
	{
		float fTmp0, fTmp1, fNumer, fDenom;

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
				if(fTmp1 <= 0.0f)		fSqrDist = fA11+2.0f*fB1+fC;
				else if(fB1 >= 0.0f)	fSqrDist = fC;
				else					fSqrDist = fB1*(-fB1/fA11)+fC;
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
				if(fTmp1 <= 0.0f)		fSqrDist = fA00+2.0f*fB0+fC;
				else if(fB0 >= 0.0f)	fSqrDist = fC;
				else					fSqrDist = fB0*(-fB0/fA00)+fC;
			}
		}
		else  // region 1
		{
			fNumer = fA11 + fB1 - fA01 - fB0;
			if(fNumer <= 0.0f)
			{
				fSqrDist = fA11+2.0f*fB1+fC;
			}
			else
			{
				fDenom = fA00-2.0f*fA01+fA11;
				if(fNumer >= fDenom)
				{
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
	// account for numerical round-off error
	return physx::intrinsics::selectMax(0.0f, fSqrDist);
}

static float OPC_SegmentSegmentSqrDist(const Gu::Segment& rkSeg0, const Gu::Segment& rkSeg1)
{
	// Hook
	PxVec3 rkSeg0Direction	= rkSeg0.computeDirection();
	PxVec3 rkSeg1Direction	= rkSeg1.computeDirection();

	PxVec3 kDiff	= rkSeg0.p0 - rkSeg1.p0;
	float fA00	= rkSeg0Direction.magnitudeSquared();
	float fA01	= -rkSeg0Direction.dot(rkSeg1Direction);
	float fA11	= rkSeg1Direction.magnitudeSquared();
	float fB0	= kDiff.dot(rkSeg0Direction);
	float fC	= kDiff.magnitudeSquared();
	float fDet	= PxAbs(fA00*fA11-fA01*fA01);

	float fB1, fS, fT, fSqrDist, fTmp;

	if(fDet>=PX_PARALLEL_TOLERANCE)
	{
		// line segments are not parallel
		fB1 = -kDiff.dot(rkSeg1Direction);
		fS = fA01*fB1-fA11*fB0;
		fT = fA01*fB0-fA00*fB1;

		if(fS >= 0.0f)
		{
			if(fS <= fDet)
			{
				if(fT >= 0.0f)
				{
					if(fT <= fDet)  // region 0 (interior)
					{
						// minimum at two interior points of 3D lines
						float fInvDet = 1.0f/fDet;
						fS *= fInvDet;
						fT *= fInvDet;
						fSqrDist = fS*(fA00*fS+fA01*fT+2.0f*fB0) + fT*(fA01*fS+fA11*fT+2.0f*fB1)+fC;
					}
					else  // region 3 (side)
					{
						fTmp = fA01+fB0;
						if(fTmp>=0.0f)			fSqrDist = fA11+2.0f*fB1+fC;
						else if(-fTmp>=fA00)	fSqrDist = fA00+fA11+fC+2.0f*(fB1+fTmp);
						else					fSqrDist = fTmp*(-fTmp/fA00)+fA11+2.0f*fB1+fC;
					}
				}
				else  // region 7 (side)
				{
					if(fB0>=0.0f)				fSqrDist = fC;
					else if(-fB0>=fA00)			fSqrDist = fA00+2.0f*fB0+fC;
					else						fSqrDist = fB0*(-fB0/fA00)+fC;
				}
			}
			else
			{
				if ( fT >= 0.0 )
				{
					if ( fT <= fDet )  // region 1 (side)
					{
						fTmp = fA01+fB1;
						if(fTmp>=0.0f)			fSqrDist = fA00+2.0f*fB0+fC;
						else if(-fTmp>=fA11)	fSqrDist = fA00+fA11+fC+2.0f*(fB0+fTmp);
						else					fSqrDist = fTmp*(-fTmp/fA11)+fA00+2.0f*fB0+fC;
					}
					else  // region 2 (corner)
					{
						fTmp = fA01+fB0;
						if ( -fTmp <= fA00 )
						{
							if(fTmp>=0.0f)		fSqrDist = fA11+2.0f*fB1+fC;
							else				fSqrDist = fTmp*(-fTmp/fA00)+fA11+2.0f*fB1+fC;
						}
						else
						{
							fTmp = fA01+fB1;
							if(fTmp>=0.0f)			fSqrDist = fA00+2.0f*fB0+fC;
							else if(-fTmp>=fA11)	fSqrDist = fA00+fA11+fC+2.0f*(fB0+fTmp);
							else					fSqrDist = fTmp*(-fTmp/fA11)+fA00+2.0f*fB0+fC;
						}
					}
				}
				else  // region 8 (corner)
				{
					if ( -fB0 < fA00 )
					{
						if(fB0>=0.0f)	fSqrDist = fC;
						else			fSqrDist = fB0*(-fB0/fA00)+fC;
					}
					else
					{
						fTmp = fA01+fB1;
						if(fTmp>=0.0f)			fSqrDist = fA00+2.0f*fB0+fC;
						else if(-fTmp>=fA11)	fSqrDist = fA00+fA11+fC+2.0f*(fB0+fTmp);
						else					fSqrDist = fTmp*(-fTmp/fA11)+fA00+2.0f*fB0+fC;
					}
				}
			}
		}
		else 
		{
			if ( fT >= 0.0f )
			{
				if ( fT <= fDet )  // region 5 (side)
				{
					if(fB1>=0.0f)		fSqrDist = fC;
					else if(-fB1>=fA11)	fSqrDist = fA11+2.0f*fB1+fC;
					else				fSqrDist = fB1*(-fB1/fA11)+fC;
				}
				else  // region 4 (corner)
				{
					fTmp = fA01+fB0;
					if ( fTmp < 0.0f )
					{
						if(-fTmp>=fA00)	fSqrDist = fA00+fA11+fC+2.0f*(fB1+fTmp);
						else			fSqrDist = fTmp*(-fTmp/fA00)+fA11+2.0f*fB1+fC;
					}
					else
					{
						if(fB1>=0.0f)		fSqrDist = fC;
						else if(-fB1>=fA11)	fSqrDist = fA11+2.0f*fB1+fC;
						else				fSqrDist = fB1*(-fB1/fA11)+fC;
					}
				}
			}
			else   // region 6 (corner)
			{
				if ( fB0 < 0.0f )
				{
					if(-fB0>=fA00)	fSqrDist = fA00+2.0f*fB0+fC;
					else			fSqrDist = fB0*(-fB0/fA00)+fC;
				}
				else
				{
					if(fB1>=0.0f)		fSqrDist = fC;
					else if(-fB1>=fA11)	fSqrDist = fA11+2.0f*fB1+fC;
					else				fSqrDist = fB1*(-fB1/fA11)+fC;
				}
			}
		}
	}
	else
	{
		// line segments are parallel
		if ( fA01 > 0.0f )
		{
			// direction vectors form an obtuse angle
			if ( fB0 >= 0.0f )
			{
				fSqrDist = fC;
			}
			else if ( -fB0 <= fA00 )
			{
				fSqrDist = fB0*(-fB0/fA00)+fC;
			}
			else
			{
				fB1 = -kDiff.dot(rkSeg1Direction);
				fTmp = fA00+fB0;
				if ( -fTmp >= fA01 )
				{
					fSqrDist = fA00+fA11+fC+2.0f*(fA01+fB0+fB1);
				}
				else
				{
					fT = -fTmp/fA01;
					fSqrDist = fA00+2.0f*fB0+fC+fT*(fA11*fT+2.0f*(fA01+fB1));
				}
			}
		}
		else
		{
			// direction vectors form an acute angle
			if ( -fB0 >= fA00 )
			{
				fSqrDist = fA00+2.0f*fB0+fC;
			}
			else if ( fB0 <= 0.0f )
			{
				fSqrDist = fB0*(-fB0/fA00)+fC;
			}
			else
			{
				fB1 = -kDiff.dot(rkSeg1Direction);
				if ( fB0 >= -fA01 )
				{
					fSqrDist = fA11+2.0f*fB1+fC;
				}
				else
				{
					fT = -fB0/fA01;
					fSqrDist = fC+fT*(2.0f*fB1+fA11*fT);
				}
			}
		}
	}
	// account for numerical round-off error
	return physx::intrinsics::selectMax(0.0f, fSqrDist);
}

PX_FORCE_INLINE float OPC_SegmentRaySqrDist(const Gu::Segment& rkSeg0, const PxVec3& seg1_orig, const PxVec3& seg1_dir)
{
	return OPC_SegmentSegmentSqrDist(rkSeg0, Gu::Segment(seg1_orig, seg1_orig + seg1_dir));
}

static float OPC_SegmentTriangleSqrDist(const Gu::Segment& segment, const PxVec3& p0, const PxVec3& p1, const PxVec3& p2)
{
	// Hook
	const PxVec3 TriEdge0 = p1 - p0;
	const PxVec3 TriEdge1 = p2 - p0;

	const PxVec3& rkSegOrigin	= segment.getOrigin();
	PxVec3 rkSegDirection		= segment.computeDirection();

	PxVec3 kDiff = p0 - rkSegOrigin;
	float fA00 = rkSegDirection.magnitudeSquared();
	float fA01 = -rkSegDirection.dot(TriEdge0);
	float fA02 = -rkSegDirection.dot(TriEdge1);
	float fA11 = TriEdge0.magnitudeSquared();
	float fA12 = TriEdge0.dot(TriEdge1);
	float fA22 = TriEdge1.dot(TriEdge1);
	float fB0  = -kDiff.dot(rkSegDirection);
	float fB1  = kDiff.dot(TriEdge0);
	float fB2  = kDiff.dot(TriEdge1);
	float fCof00 = fA11*fA22-fA12*fA12;
	float fCof01 = fA02*fA12-fA01*fA22;
	float fCof02 = fA01*fA12-fA02*fA11;
	float fDet = fA00*fCof00+fA01*fCof01+fA02*fCof02;

	PxVec3 kPt;
	float fSqrDist, fSqrDist0;

	// PT: replaced with same fix as in TTP 4021. Seems to fix TTP 5514.
	//	if(fabsf(fDet)>=PX_PARALLEL_TOLERANCE)

	PxVec3 kNormal = TriEdge0.cross(TriEdge1);
	float fDot = kNormal.dot(rkSegDirection);
	if ( fDot*fDot >= 1e-6*rkSegDirection.magnitudeSquared()*kNormal.magnitudeSquared())
	{
		float fCof11 = fA00*fA22-fA02*fA02;
		float fCof12 = fA02*fA01-fA00*fA12;
		float fCof22 = fA00*fA11-fA01*fA01;
		float fInvDet = 1.0f/fDet;
		float fRhs0 = -fB0*fInvDet;
		float fRhs1 = -fB1*fInvDet;
		float fRhs2 = -fB2*fInvDet;

		float fR = fCof00*fRhs0+fCof01*fRhs1+fCof02*fRhs2;
		float fS = fCof01*fRhs0+fCof11*fRhs1+fCof12*fRhs2;
		float fT = fCof02*fRhs0+fCof12*fRhs1+fCof22*fRhs2;

		if ( fR < 0.0f )
		{
			if ( fS+fT <= 1.0f )
			{
				if ( fS < 0.0f )
				{
					if ( fT < 0.0f )  // region 4m
					{
						// minimum on face s=0 or t=0 or r=0
						fSqrDist = OPC_SegmentRaySqrDist(segment, p0, TriEdge1);
						fSqrDist0 = OPC_SegmentRaySqrDist(segment, p0, TriEdge0);
						if(fSqrDist0<fSqrDist)	fSqrDist = fSqrDist0;
						fSqrDist0 = OPC_PointTriangleSqrDist(rkSegOrigin, p0, p1, p2);
						if(fSqrDist0<fSqrDist)	fSqrDist = fSqrDist0;
                    }
                    else  // region 3m
                    {
                        // minimum on face s=0 or r=0
                        fSqrDist = OPC_SegmentRaySqrDist(segment, p0, TriEdge1);
                        fSqrDist0 = OPC_PointTriangleSqrDist(rkSegOrigin, p0, p1, p2);
                        if(fSqrDist0<fSqrDist)	fSqrDist = fSqrDist0;
                    }
                }
                else if ( fT < 0.0f )  // region 5m
                {
                    // minimum on face t=0 or r=0
                    fSqrDist = OPC_SegmentRaySqrDist(segment, p0, TriEdge0);
                    fSqrDist0 = OPC_PointTriangleSqrDist(rkSegOrigin, p0, p1, p2);
                    if(fSqrDist0<fSqrDist)	fSqrDist = fSqrDist0;
                }
                else  // region 0m
                {
                    // minimum on face r=0
                    fSqrDist = OPC_PointTriangleSqrDist(rkSegOrigin, p0, p1, p2);
                }
            }
            else
            {
                if ( fS < 0.0f )  // region 2m
                {
                    // minimum on face s=0 or s+t=1 or r=0
                    fSqrDist = OPC_SegmentRaySqrDist(segment, p0, TriEdge1);
                    fSqrDist0 = OPC_SegmentRaySqrDist(segment, p1, TriEdge1 - TriEdge0);
                    if(fSqrDist0<fSqrDist)	fSqrDist = fSqrDist0;
                    fSqrDist0 = OPC_PointTriangleSqrDist(rkSegOrigin, p0, p1, p2);
                    if(fSqrDist0<fSqrDist)	fSqrDist = fSqrDist0;
                }
                else if ( fT < 0.0f )  // region 6m
                {
                    // minimum on face t=0 or s+t=1 or r=0
                    fSqrDist = OPC_SegmentRaySqrDist(segment, p0, TriEdge0);
                    fSqrDist0 = OPC_SegmentRaySqrDist(segment, p1, TriEdge1 - TriEdge0);
                    if(fSqrDist0<fSqrDist)	fSqrDist = fSqrDist0;
                    fSqrDist0 = OPC_PointTriangleSqrDist(rkSegOrigin, p0, p1, p2);
                    if(fSqrDist0<fSqrDist)	fSqrDist = fSqrDist0;
                }
                else  // region 1m
                {
                    // minimum on face s+t=1 or r=0
                    fSqrDist = OPC_SegmentRaySqrDist(segment, p1, TriEdge1 - TriEdge0);
                    fSqrDist0 = OPC_PointTriangleSqrDist(rkSegOrigin, p0, p1, p2);
                    if(fSqrDist0<fSqrDist)	fSqrDist = fSqrDist0;
                }
            }
        }
        else if ( fR <= 1.0f )
        {
            if ( fS+fT <= 1.0f )
            {
                if ( fS < 0.0f )
                {
                    if ( fT < 0.0f )  // region 4
                    {
                        // minimum on face s=0 or t=0
                        fSqrDist = OPC_SegmentRaySqrDist(segment, p0, TriEdge1);
                        fSqrDist0 = OPC_SegmentRaySqrDist(segment, p0, TriEdge0);
                        if(fSqrDist0<fSqrDist)	fSqrDist = fSqrDist0;
                    }
                    else  // region 3
                    {
                        // minimum on face s=0
                        fSqrDist = OPC_SegmentRaySqrDist(segment, p0, TriEdge1);
                    }
                }
                else if ( fT < 0.0f )  // region 5
                {
                    // minimum on face t=0
                    fSqrDist = OPC_SegmentRaySqrDist(segment, p0, TriEdge0);
                }
                else  // region 0
                {
                    // global minimum is interior, done
                    fSqrDist = fR*(fA00*fR+fA01*fS+fA02*fT+2.0f*fB0)
                          +fS*(fA01*fR+fA11*fS+fA12*fT+2.0f*fB1)
                          +fT*(fA02*fR+fA12*fS+fA22*fT+2.0f*fB2)
                          +kDiff.magnitudeSquared();
                }
            }
            else
            {
                if ( fS < 0.0f )  // region 2
                {
                    // minimum on face s=0 or s+t=1
                    fSqrDist = OPC_SegmentRaySqrDist(segment, p0, TriEdge1);
                    fSqrDist0 = OPC_SegmentRaySqrDist(segment, p1, TriEdge1 - TriEdge0);
                    if(fSqrDist0<fSqrDist)	fSqrDist = fSqrDist0;
                }
                else if ( fT < 0.0f )  // region 6
                {
                    // minimum on face t=0 or s+t=1
                    fSqrDist = OPC_SegmentRaySqrDist(segment, p0, TriEdge0);
                    fSqrDist0 = OPC_SegmentRaySqrDist(segment, p1, TriEdge1 - TriEdge0);
                    if(fSqrDist0<fSqrDist)	fSqrDist = fSqrDist0;
                }
                else  // region 1
                {
                    // minimum on face s+t=1
                    fSqrDist = OPC_SegmentRaySqrDist(segment, p1, TriEdge1 - TriEdge0);
                }
            }
        }
        else  // fR > 1
        {
            if ( fS+fT <= 1.0f )
            {
                if ( fS < 0.0f )
                {
                    if ( fT < 0.0f )  // region 4p
                    {
                        // minimum on face s=0 or t=0 or r=1
                        fSqrDist = OPC_SegmentRaySqrDist(segment, p0, TriEdge1);
                        fSqrDist0 = OPC_SegmentRaySqrDist(segment, p0, TriEdge0);
                        if(fSqrDist0<fSqrDist)	fSqrDist = fSqrDist0;
                        kPt = rkSegOrigin+rkSegDirection;
                        fSqrDist0 = OPC_PointTriangleSqrDist(kPt, p0, p1, p2);
                        if(fSqrDist0<fSqrDist)	fSqrDist = fSqrDist0;
                    }
                    else  // region 3p
                    {
                        // minimum on face s=0 or r=1
                        fSqrDist = OPC_SegmentRaySqrDist(segment, p0, TriEdge1);
                        kPt = rkSegOrigin+rkSegDirection;
                        fSqrDist0 = OPC_PointTriangleSqrDist(kPt, p0, p1, p2);
                        if(fSqrDist0<fSqrDist)	fSqrDist = fSqrDist0;
                    }
                }
                else if ( fT < 0.0f )  // region 5p
                {
                    // minimum on face t=0 or r=1
                    fSqrDist = OPC_SegmentRaySqrDist(segment, p0, TriEdge0);
                    kPt = rkSegOrigin+rkSegDirection;
                    fSqrDist0 = OPC_PointTriangleSqrDist(kPt, p0, p1, p2);
                    if(fSqrDist0<fSqrDist)	fSqrDist = fSqrDist0;
                }
                else  // region 0p
                {
                    // minimum face on r=1
                    kPt = rkSegOrigin+rkSegDirection;
                    fSqrDist = OPC_PointTriangleSqrDist(kPt, p0, p1, p2);
                }
            }
            else
            {
                if ( fS < 0.0f )  // region 2p
                {
                    // minimum on face s=0 or s+t=1 or r=1
                    fSqrDist = OPC_SegmentRaySqrDist(segment, p0, TriEdge1);
                    fSqrDist0 = OPC_SegmentRaySqrDist(segment, p1, TriEdge1 - TriEdge0);
                    if(fSqrDist0<fSqrDist)	fSqrDist = fSqrDist0;
                    kPt = rkSegOrigin+rkSegDirection;
                    fSqrDist0 = OPC_PointTriangleSqrDist(kPt, p0, p1, p2);
                    if(fSqrDist0<fSqrDist)	fSqrDist = fSqrDist0;
                }
                else if ( fT < 0.0f )  // region 6p
                {
                    // minimum on face t=0 or s+t=1 or r=1
                    fSqrDist = OPC_SegmentRaySqrDist(segment, p0, TriEdge0);
                    fSqrDist0 = OPC_SegmentRaySqrDist(segment, p1, TriEdge1 - TriEdge0);
                    if(fSqrDist0<fSqrDist)	fSqrDist = fSqrDist0;
                    kPt = rkSegOrigin+rkSegDirection;
                    fSqrDist0 = OPC_PointTriangleSqrDist(kPt, p0, p1, p2);
                    if(fSqrDist0<fSqrDist)	fSqrDist = fSqrDist0;
                }
                else  // region 1p
                {
                    // minimum on face s+t=1 or r=1
                    fSqrDist = OPC_SegmentRaySqrDist(segment, p1, TriEdge1 - TriEdge0);
                    kPt = rkSegOrigin+rkSegDirection;
                    fSqrDist0 = OPC_PointTriangleSqrDist(kPt, p0, p1, p2);
                    if(fSqrDist0<fSqrDist)	fSqrDist = fSqrDist0;
                }
            }
        }
    }
    else
    {
        // segment and triangle are parallel
        fSqrDist = OPC_SegmentRaySqrDist(segment, p0, TriEdge0);

        fSqrDist0 = OPC_SegmentRaySqrDist(segment, p0, TriEdge1);
        if(fSqrDist0<fSqrDist)	fSqrDist = fSqrDist0;

        fSqrDist0 = OPC_SegmentRaySqrDist(segment, p1, TriEdge1 - TriEdge0);
        if(fSqrDist0<fSqrDist)	fSqrDist = fSqrDist0;

        fSqrDist0 = OPC_PointTriangleSqrDist(rkSegOrigin, p0, p1, p2);
        if(fSqrDist0<fSqrDist)	fSqrDist = fSqrDist0;

        kPt = rkSegOrigin+rkSegDirection;
        fSqrDist0 = OPC_PointTriangleSqrDist(kPt, p0, p1, p2);
        if(fSqrDist0<fSqrDist)	fSqrDist = fSqrDist0;
    }
    return PxAbs(fSqrDist);
}

PX_FORCE_INLINE Ps::IntBool LSSCollider::LSSTriOverlap(const PxVec3& vert0, const PxVec3& vert1, const PxVec3& vert2)
{
//	return LSSTriangleOverlap(Triangle(vert0, vert1, vert2), mSeg.mP0, mSeg.ComputeDirection(), sqrtf(mRadius2));

	const float s2 = OPC_SegmentTriangleSqrDist(mSeg, vert0, vert1, vert2);
	if(s2<mRadius2)	return Ps::IntTrue;
	return Ps::IntFalse;
}

#ifndef OPC_SUPPORT_VMX128

PX_FORCE_INLINE Ps::IntBool LSSCollider::LooseLSSTriOverlap(const PxVec3& vert0, const PxVec3& vert1, const PxVec3& vert2)
{
	const PxVec3 v0 = mOBB.rot.transformTranspose(vert0 - mOBB.center);//should be inverse rotate.
	const PxVec3 v1 = mOBB.rot.transformTranspose(vert1 - mOBB.center);
	const PxVec3 v2 = mOBB.rot.transformTranspose(vert2 - mOBB.center);
// now do a tri-AABB test, with AABB at origin.

	//test tri AABB

	const PxVec3 triMin = PxVec3(PxMin(v0.x, PxMin(v1.x, v2.x)), PxMin(v0.y, PxMin(v1.y, v2.y)), PxMin(v0.z, PxMin(v1.z, v2.z)));
	const PxVec3 triMax = PxVec3(PxMax(v0.x, PxMax(v1.x, v2.x)), PxMax(v0.y, PxMax(v1.y, v2.y)), PxMax(v0.z, PxMax(v1.z, v2.z)));

	if((triMin.x > mOBB.extents.x) || (triMin.y > mOBB.extents.y) || (triMin.z > mOBB.extents.z))
		return Ps::IntFalse;

	if((triMax.x < -mOBB.extents.x) || (triMax.y < -mOBB.extents.y) || (triMax.z < -mOBB.extents.z))
		return Ps::IntFalse;

	// test tri plane.

	PxVec3 normal = (v1 - v0).cross(v2 - v0);
	float dist = normal.dot(v0);

	// find the minimum maximum on normal.
	PxVec3 vMin, vMax;

	if(normal.x > 0) 
	{
		vMin.x = -mOBB.extents.x;
		vMax.x = mOBB.extents.x;
	}
	else
	{
		vMin.x = mOBB.extents.x;
		vMax.x = -mOBB.extents.x;
	}

	if(normal.y > 0) 
	{
		vMin.y = -mOBB.extents.y;
		vMax.y = mOBB.extents.y;
	}
	else
	{
		vMin.y = mOBB.extents.y;
		vMax.y = -mOBB.extents.y;
	}


	if(normal.z > 0) 
	{
		vMin.z = -mOBB.extents.z;
		vMax.z = mOBB.extents.z;
	}
	else
	{
		vMin.z = mOBB.extents.z;
		vMax.z = -mOBB.extents.z;
	}

	// are they disjoint?

	float minDist = vMin.dot(normal);
	float maxDist = vMax.dot(normal);

	if((minDist > dist) || (maxDist < dist))
		return Ps::IntFalse;


	// Test edge axes.

	PxVec3 axis;
	float p0, p1, p2, pMin, pMax, axisRadius;

	PxVec3 edge0 = v1 - v0;
	PxVec3 edge1 = v2 - v1;
	PxVec3 edge2 = v0 - v2;

	PxVec3 radius = mOBB.extents;

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

	

	axis = Ps::cross100(edge0);
	p0 = axis.dot(v0);
	p2 = axis.dot(v2);
	pMin = PxMin(p0, p2);
	pMax = PxMax(p0, p2);
	axisRadius = radius.dot(PxVec3(PxAbs(axis.x), PxAbs(axis.y), PxAbs(axis.z)));
	if((pMin > axisRadius) || (pMax < -axisRadius))
		return Ps::IntFalse;

	// axis == [1,0,0] x e1 == [0, -e1.z, e1.y]
	axis = Ps::cross100(edge1);
	p0 = axis.dot(v0);
	p1 = axis.dot(v1);
	pMin = PxMin(p0, p1);
	pMax = PxMax(p0, p1);
	axisRadius = radius.dot(PxVec3(PxAbs(axis.x), PxAbs(axis.y), PxAbs(axis.z)));
	if((pMin > axisRadius) || (pMax < -axisRadius))
		return Ps::IntFalse;

	// axis == [1,0,0] x e2 == [0, -e2.z, e2.y]
	axis = Ps::cross100(edge2);
	p0 = axis.dot(v0);
	p1 = axis.dot(v1);
	pMin = PxMin(p0, p1);
	pMax = PxMax(p0, p1);
	axisRadius = radius.dot(PxVec3(PxAbs(axis.x), PxAbs(axis.y), PxAbs(axis.z)));
	if((pMin > axisRadius) || (pMax < -axisRadius))
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

	axis = Ps::cross010(edge0);
	p0 = axis.dot(v0);
	p2 = axis.dot(v2);
	pMin = PxMin(p0, p2);
	pMax = PxMax(p0, p2);
	axisRadius = radius.dot(PxVec3(PxAbs(axis.x), PxAbs(axis.y), PxAbs(axis.z)));
	if((pMin > axisRadius) || (pMax < -axisRadius))
		return Ps::IntFalse;


	// axis == [0,1,0] x e1 == [e1.z, 0, -e1.x]
	axis = Ps::cross010(edge1);
	p0 = axis.dot(v0);
	p1 = axis.dot(v1);
	pMin = PxMin(p0, p1);
	pMax = PxMax(p0, p1);
	axisRadius = radius.dot(PxVec3(PxAbs(axis.x), PxAbs(axis.y), PxAbs(axis.z)));
	if((pMin > axisRadius) || (pMax < -axisRadius))
		return Ps::IntFalse;

	// axis == [0, 1, 0] x e2 == [e2.z, 0, -e2.x]
	axis = Ps::cross010(edge2);
	p0 = axis.dot(v0);
	p1 = axis.dot(v1);
	pMin = PxMin(p0, p1);
	pMax = PxMax(p0, p1);
	axisRadius = radius.dot(PxVec3(PxAbs(axis.x), PxAbs(axis.y), PxAbs(axis.z)));
	if((pMin > axisRadius) || (pMax < -axisRadius))
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

	// axis == [0, 1, 0] x e2 == [e2.z, 0, -e2.x]
	axis = Ps::cross001(edge0);
	p0 = axis.dot(v0);
	p2 = axis.dot(v2);
	pMin = PxMin(p0, p2);
	pMax = PxMax(p0, p2);
	axisRadius = radius.dot(PxVec3(PxAbs(axis.x), PxAbs(axis.y), PxAbs(axis.z)));
	if((pMin > axisRadius) || (pMax < -axisRadius))
		return Ps::IntFalse;


	// axis == [0, 0, 1] x e1 == [-e1.y, e1.x, 0]

	axis = Ps::cross001(edge1);
	p0 = axis.dot(v0);
	p1 = axis.dot(v1);
	pMin = PxMin(p0, p1);
	pMax = PxMax(p0, p1);
	axisRadius = radius.dot(PxVec3(PxAbs(axis.x), PxAbs(axis.y), PxAbs(axis.z)));
	if((pMin > axisRadius) || (pMax < -axisRadius))
		return Ps::IntFalse;

	// axis == [0, 0, 1] x e2 == [-e2.y, e2.x, 0]

	axis = Ps::cross001(edge2);
	p0 = axis.dot(v0);
	p1 = axis.dot(v1);
	pMin = PxMin(p0, p1);
	pMax = PxMax(p0, p1);
	axisRadius = radius.dot(PxVec3(PxAbs(axis.x), PxAbs(axis.y), PxAbs(axis.z)));
	if((pMin > axisRadius) || (pMax < -axisRadius))
		return Ps::IntFalse;


	return Ps::IntTrue;
}

#else
PX_FORCE_INLINE Ps::IntBool LSSCollider::LooseLSSTriOverlap(const PxVec3& vert0, const PxVec3& vert1, const PxVec3& vert2)
{
	//transform obb to origin
	Cm::PxSimd::Vector4 v0 = Cm::PxSimd::load(vert0);
	Cm::PxSimd::Vector4 v1 = Cm::PxSimd::load(vert1);
	Cm::PxSimd::Vector4 v2 = Cm::PxSimd::load(vert2);

	Cm::PxSimd::Vector4 rot_0 = Cm::PxSimd::load(mOBB.rot[0]);
	Cm::PxSimd::Vector4 rot_1 = Cm::PxSimd::load(mOBB.rot[1]);
	Cm::PxSimd::Vector4 rot_2 = Cm::PxSimd::load(mOBB.rot[2]);

	Cm::PxSimd::Vector4 center = Cm::PxSimd::load(mOBB.center);

	v0 = Cm::PxSimd::rotateInv(rot_0, rot_1, rot_2, Cm::PxSimd::subtract(v0, center));
	v1 = Cm::PxSimd::rotateInv(rot_0, rot_1, rot_2, Cm::PxSimd::subtract(v1, center));
	v2 = Cm::PxSimd::rotateInv(rot_0, rot_1, rot_2, Cm::PxSimd::subtract(v2, center));

	Cm::PxSimd::Vector4 signMask = Cm::PxSimd::signMask();
	Cm::PxSimd::Vector4 zero = Cm::PxSimd::zero();

	Cm::PxSimd::Vector4 extents = Cm::PxSimd::load(mOBB.extents);
	Cm::PxSimd::Vector4 minusExtents = Cm::PxSimd::xor4(signMask, extents);

// now do a tri-AABB test, with AABB at origin.

// Test triangle AABB
	Cm::PxSimd::Vector4 triMin = Cm::PxSimd::minimum(v0, Cm::PxSimd::minimum(v1, v2));
	Cm::PxSimd::Vector4 triMax = Cm::PxSimd::maximum(v0, Cm::PxSimd::maximum(v1, v2));

	Cm::PxSimd::Vector4 mask = Cm::PxSimd::or4(Cm::PxSimd::greater(triMin, extents), Cm::PxSimd::greater(minusExtents, triMax));
	if(Cm::PxSimd::intNotEqualBool(mask, zero))
		return Ps::IntFalse;

	// test tri plane.
	Cm::PxSimd::Vector4 normal = Cm::PxSimd::cross(Cm::PxSimd::subtract(v1, v0), Cm::PxSimd::subtract(v2, v0));
	Cm::PxSimd::Vector4 dist = Cm::PxSimd::dot(normal, v0);

	
	Cm::PxSimd::Vector4 vMask = Cm::PxSimd::greater(normal, zero);
	Cm::PxSimd::Vector4 vMin = Cm::PxSimd::select(extents, minusExtents, vMask);
	Cm::PxSimd::Vector4 vMax = Cm::PxSimd::select(minusExtents, extents, vMask);

	Cm::PxSimd::Vector4 minDist = Cm::PxSimd::dot(vMin, normal);
	Cm::PxSimd::Vector4 maxDist = Cm::PxSimd::dot(vMax, normal);

	mask = Cm::PxSimd::or4(Cm::PxSimd::greater(minDist, dist), Cm::PxSimd::less(maxDist, dist));
	if(Cm::PxSimd::intNotEqualBool(mask, zero))
		return Ps::IntFalse;

	// Test edge axes.
//////// test edge axis
	//transform the triangle.

	Cm::PxSimd::Vector4 edge0 = Cm::PxSimd::subtract(v1, v0);
	Cm::PxSimd::Vector4 edge1 = Cm::PxSimd::subtract(v2, v1);
	Cm::PxSimd::Vector4 edge2 = Cm::PxSimd::subtract(v0, v2);

	// zero out w
	edge0 = Cm::PxSimd::and4(edge0, Cm::PxSimd::xyzMask());
	edge1 = Cm::PxSimd::and4(edge1, Cm::PxSimd::xyzMask());
	edge2 = Cm::PxSimd::and4(edge2, Cm::PxSimd::xyzMask());
	
	Cm::PxSimd::Vector4 minusEdge0 = Cm::PxSimd::xor4(signMask, edge0);
	Cm::PxSimd::Vector4 minusEdge1 = Cm::PxSimd::xor4(signMask, edge1);
	Cm::PxSimd::Vector4 minusEdge2 = Cm::PxSimd::xor4(signMask, edge2);

	Cm::PxSimd::Vector4 radius = extents;
	Cm::PxSimd::Vector4 minusRadius = Cm::PxSimd::xor4(signMask, radius);


	Cm::PxSimd::Vector4 axis, p0, p1, p2, pMin, pMax, axisRadius, minusAxisRadius;

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

	axis = Cm::PxSimd::permute(edge0, minusEdge0, PERMUTE_0W1Z0Y0W );
	p0 = Cm::PxSimd::dot(v0, axis);
	p2 = Cm::PxSimd::dot(v2, axis);
	pMin = Cm::PxSimd::minimum(p0, p2);
	pMax = Cm::PxSimd::maximum(p0, p2);
	axisRadius = Cm::PxSimd::dot(radius, Cm::PxSimd::abs(axis));
	minusAxisRadius = Cm::PxSimd::xor4(signMask, axisRadius);
	// we test against the sphere radius, not the box radius on the axis...(so this isnt a standard AABB test).
	if(Cm::PxSimd::greaterXBool(pMin, axisRadius) || Cm::PxSimd::lessXBool(pMax, minusAxisRadius))
		return Ps::IntFalse;

	// axis == [1,0,0] x e1 == [0, -e1.z, e1.y]
	axis = Cm::PxSimd::permute(edge1, minusEdge1, PERMUTE_0W1Z0Y0W );
	p0 = Cm::PxSimd::dot(v0, axis);
	p1 = Cm::PxSimd::dot(v1, axis);
	pMin = Cm::PxSimd::minimum(p0, p1);
	pMax = Cm::PxSimd::maximum(p0, p1);
	axisRadius = Cm::PxSimd::dot(radius, Cm::PxSimd::abs(axis));
	minusAxisRadius = Cm::PxSimd::xor4(signMask, axisRadius);
	if(Cm::PxSimd::greaterXBool(pMin, axisRadius) || Cm::PxSimd::lessXBool(pMax, minusAxisRadius))
		return Ps::IntFalse;


	// axis == [1,0,0] x e2 == [0, -e2.z, e2.y]
	axis = Cm::PxSimd::permute(edge2, minusEdge2, PERMUTE_0W1Z0Y0W);
	p0 = Cm::PxSimd::dot(v0, axis);
	p1 = Cm::PxSimd::dot(v1, axis);
	pMin = Cm::PxSimd::minimum(p0, p1);
	pMax = Cm::PxSimd::maximum(p0, p1);
	axisRadius = Cm::PxSimd::dot(radius, Cm::PxSimd::abs(axis));
	minusAxisRadius = Cm::PxSimd::xor4(signMask, axisRadius);
	if(Cm::PxSimd::greaterXBool(pMin, axisRadius) || Cm::PxSimd::lessXBool(pMax, minusAxisRadius))
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
	pMin = Cm::PxSimd::minimum(p0, p2);
	pMax = Cm::PxSimd::maximum(p0, p2);
	axisRadius = Cm::PxSimd::dot(radius, Cm::PxSimd::abs(axis));
	minusAxisRadius = Cm::PxSimd::xor4(signMask, axisRadius);
	if(Cm::PxSimd::greaterXBool(pMin, axisRadius) || Cm::PxSimd::lessXBool(pMax, minusAxisRadius))
		return Ps::IntFalse;

	// axis == [0,1,0] x e1 == [e1.z, 0, -e1.x]
	axis = Cm::PxSimd::permute(edge1, minusEdge1, PERMUTE_0Z0W1X0W);
	p0 = Cm::PxSimd::dot(v0, axis);
	p1 = Cm::PxSimd::dot(v1, axis);
	pMin = Cm::PxSimd::minimum(p0, p1);
	pMax = Cm::PxSimd::maximum(p0, p1);
	axisRadius = Cm::PxSimd::dot(radius, Cm::PxSimd::abs(axis));
	minusAxisRadius = Cm::PxSimd::xor4(signMask, axisRadius);
	if(Cm::PxSimd::greaterXBool(pMin, axisRadius) || Cm::PxSimd::lessXBool(pMax, minusAxisRadius))
		return Ps::IntFalse;

	// axis == [0, 1, 0] x e2 == [e2.z, 0, -e2.x]
	axis = Cm::PxSimd::permute(edge2, minusEdge2, PERMUTE_0Z0W1X0W);
	p0 = Cm::PxSimd::dot(v0, axis);
	p1 = Cm::PxSimd::dot(v1, axis);
	pMin = Cm::PxSimd::minimum(p0, p1);
	pMax = Cm::PxSimd::maximum(p0, p1);
	axisRadius = Cm::PxSimd::dot(radius, Cm::PxSimd::abs(axis));
	minusAxisRadius = Cm::PxSimd::xor4(signMask, axisRadius);
	if(Cm::PxSimd::greaterXBool(pMin, axisRadius) || Cm::PxSimd::lessXBool(pMax, minusAxisRadius))
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
	pMin = Cm::PxSimd::minimum(p0, p2);
	pMax = Cm::PxSimd::maximum(p0, p2);
	axisRadius = Cm::PxSimd::dot(radius, Cm::PxSimd::abs(axis));
	minusAxisRadius = Cm::PxSimd::xor4(signMask, axisRadius);
	if(Cm::PxSimd::greaterXBool(pMin, axisRadius) || Cm::PxSimd::lessXBool(pMax, minusAxisRadius))
		return Ps::IntFalse;

	// axis == [0, 0, 1] x e1 == [-e1.y, e1.x, 0]
	axis = Cm::PxSimd::permute(edge1, minusEdge1, PERMUTE_1Y0X0W0W);
	p0 = Cm::PxSimd::dot(v0, axis);
	p1 = Cm::PxSimd::dot(v1, axis);
	pMin = Cm::PxSimd::minimum(p0, p1);
	pMax = Cm::PxSimd::maximum(p0, p1);
	axisRadius = Cm::PxSimd::dot(radius, Cm::PxSimd::abs(axis));
	minusAxisRadius = Cm::PxSimd::xor4(signMask, axisRadius);
	if(Cm::PxSimd::greaterXBool(pMin, axisRadius) || Cm::PxSimd::lessXBool(pMax, minusAxisRadius))
		return Ps::IntFalse;

	// axis == [0, 0, 1] x e2 == [-e2.y, e2.x, 0]
	axis = Cm::PxSimd::permute(edge2, minusEdge2, PERMUTE_1Y0X0W0W);
	p0 = Cm::PxSimd::dot(v0, axis);
	p1 = Cm::PxSimd::dot(v1, axis);
	pMin = Cm::PxSimd::minimum(p0, p1);
	pMax = Cm::PxSimd::maximum(p0, p1);
	axisRadius = Cm::PxSimd::dot(radius, Cm::PxSimd::abs(axis));
	minusAxisRadius = Cm::PxSimd::xor4(signMask, axisRadius);
	if(Cm::PxSimd::greaterXBool(pMin, axisRadius) || Cm::PxSimd::lessXBool(pMax, minusAxisRadius))
		return Ps::IntFalse;

	return Ps::IntTrue;
}

#endif

} // namespace Ice

}

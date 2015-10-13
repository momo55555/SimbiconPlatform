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

#include "GuDistanceSegmentSegment.h"

using namespace physx;

// ptchernev: 
// The Magic Software code uses a relative error test for parallel case.
// The Novodex code does not presumably as an optimization. 
// Since the Novodex code is working in the trunk I see no reason 
// to reintroduce the relative error test here.

// PT: this might just be because the relative error test has been added
// after we grabbed the code. I don't remember making this change. A good
// idea would be NOT to refactor Magic's code, to easily grab updated
// versions from the website.............................................

//Uses an old Wild Magic function, which suffers from some problems addressed 
//in the new version, see TTP 4617. However, some of the colliders seems to work 
//badly with the new version (e.g. Capsule/Mesh), so we keep the old one around 
//for a while.
PxReal Gu::distanceSegmentSegmentSquaredOLD(	const PxVec3& origin0, const PxVec3& extent0,
											const PxVec3& origin1, const PxVec3& extent1,
											PxReal* s, PxReal* t)
{
	const PxVec3 kDiff	= origin0 - origin1;
	const PxReal fA00	= extent0.magnitudeSquared();
	const PxReal fA01	= -extent0.dot(extent1);
	const PxReal fA11	= extent1.magnitudeSquared();
	const PxReal fB0	= kDiff.dot(extent0);
	const PxReal fC		= kDiff.magnitudeSquared();
	const PxReal fDet	= PxAbs(fA00*fA11-fA01*fA01);

	PxReal fB1, fS, fT, fSqrDist, fTmp;

	if(fDet>=PX_PARALLEL_TOLERANCE)
	{
		// line segments are not parallel
		fB1 = -kDiff.dot(extent1);
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
						PxReal fInvDet = 1.0f/fDet;
						fS *= fInvDet;
						fT *= fInvDet;
						fSqrDist = fS*(fA00*fS+fA01*fT+2.0f*fB0) +
							fT*(fA01*fS+fA11*fT+2.0f*fB1)+fC;
					}
					else  // region 3 (side)
					{
						fT = 1.0f;
						fTmp = fA01+fB0;
						if ( fTmp >= 0.0f )
						{
							fS = 0.0f;
							fSqrDist = fA11+2.0f*fB1+fC;
						}
						else if ( -fTmp >= fA00 )
						{
							fS = 1.0f;
							fSqrDist = fA00+fA11+fC+2.0f*(fB1+fTmp);
						}
						else
						{
							fS = -fTmp/fA00;
							fSqrDist = fTmp*fS+fA11+2.0f*fB1+fC;
						}
					}
				}
				else  // region 7 (side)
				{
					fT = 0.0f;
					if ( fB0 >= 0.0f )
					{
						fS = 0.0f;
						fSqrDist = fC;
					}
					else if ( -fB0 >= fA00 )
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
			}
			else
			{
				if ( fT >= 0.0 )
				{
					if ( fT <= fDet )  // region 1 (side)
					{
						fS = 1.0f;
						fTmp = fA01+fB1;
						if ( fTmp >= 0.0f )
						{
							fT = 0.0f;
							fSqrDist = fA00+2.0f*fB0+fC;
						}
						else if ( -fTmp >= fA11 )
						{
							fT = 1.0f;
							fSqrDist = fA00+fA11+fC+2.0f*(fB0+fTmp);
						}
						else
						{
							fT = -fTmp/fA11;
							fSqrDist = fTmp*fT+fA00+2.0f*fB0+fC;
						}
					}
					else  // region 2 (corner)
					{
						fTmp = fA01+fB0;
						if ( -fTmp <= fA00 )
						{
							fT = 1.0f;
							if ( fTmp >= 0.0f )
							{
								fS = 0.0f;
								fSqrDist = fA11+2.0f*fB1+fC;
							}
							else
							{
								fS = -fTmp/fA00;
								fSqrDist = fTmp*fS+fA11+2.0f*fB1+fC;
							}
						}
						else
						{
							fS = 1.0f;
							fTmp = fA01+fB1;
							if ( fTmp >= 0.0f )
							{
								fT = 0.0f;
								fSqrDist = fA00+2.0f*fB0+fC;
							}
							else if ( -fTmp >= fA11 )
							{
								fT = 1.0f;
								fSqrDist = fA00+fA11+fC+2.0f*(fB0+fTmp);
							}
							else
							{
								fT = -fTmp/fA11;
								fSqrDist = fTmp*fT+fA00+2.0f*fB0+fC;
							}
						}
					}
				}
				else  // region 8 (corner)
				{
					if ( -fB0 < fA00 )
					{
						fT = 0.0f;
						if ( fB0 >= 0.0f )
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
					else
					{
						fS = 1.0f;
						fTmp = fA01+fB1;
						if ( fTmp >= 0.0f )
						{
							fT = 0.0f;
							fSqrDist = fA00+2.0f*fB0+fC;
						}
						else if ( -fTmp >= fA11 )
						{
							fT = 1.0f;
							fSqrDist = fA00+fA11+fC+2.0f*(fB0+fTmp);
						}
						else
						{
							fT = -fTmp/fA11;
							fSqrDist = fTmp*fT+fA00+2.0f*fB0+fC;
						}
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
					fS = 0.0f;
					if ( fB1 >= 0.0f )
					{
						fT = 0.0f;
						fSqrDist = fC;
					}
					else if ( -fB1 >= fA11 )
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
				else  // region 4 (corner)
				{
					fTmp = fA01+fB0;
					if ( fTmp < 0.0f )
					{
						fT = 1.0f;
						if ( -fTmp >= fA00 )
						{
							fS = 1.0f;
							fSqrDist = fA00+fA11+fC+2.0f*(fB1+fTmp);
						}
						else
						{
							fS = -fTmp/fA00;
							fSqrDist = fTmp*fS+fA11+2.0f*fB1+fC;
						}
					}
					else
					{
						fS = 0.0f;
						if ( fB1 >= 0.0f )
						{
							fT = 0.0f;
							fSqrDist = fC;
						}
						else if ( -fB1 >= fA11 )
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
			}
			else   // region 6 (corner)
			{
				if ( fB0 < 0.0f )
				{
					fT = 0.0f;
					if ( -fB0 >= fA00 )
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
					if ( fB1 >= 0.0f )
					{
						fT = 0.0f;
						fSqrDist = fC;
					}
					else if ( -fB1 >= fA11 )
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
				fS = 0.0f;
				fT = 0.0f;
				fSqrDist = fC;
			}
			else if ( -fB0 <= fA00 )
			{
				fS = -fB0/fA00;
				fT = 0.0f;
				fSqrDist = fB0*fS+fC;
			}
			else
			{
				fB1 = -kDiff.dot(extent1);
				fS = 1.0f;
				fTmp = fA00+fB0;
				if ( -fTmp >= fA01 )
				{
					fT = 1.0f;
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
				fS = 1.0f;
				fT = 0.0f;
				fSqrDist = fA00+2.0f*fB0+fC;
			}
			else if ( fB0 <= 0.0f )
			{
				fS = -fB0/fA00;
				fT = 0.0f;
				fSqrDist = fB0*fS+fC;
			}
			else
			{
				fB1 = -kDiff.dot(extent1);
				fS = 0.0f;
				if ( fB0 >= -fA01 )
				{
					fT = 1.0f;
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

	if(s)	*s = fS;
	if(t)	*t = fT;

	// account for numerical round-off error
	return physx::intrinsics::selectMax(0.0f, fSqrDist);
}

static const float ZERO_TOLERANCE = 1e-06f;

// S0 = origin + extent * dir;
// S1 = origin - extent * dir;
static PxReal distanceSegmentSegmentSquaredNEW(const PxVec3& origin0,
										 const PxVec3& dir0,
										 PxReal extent0,

										 const PxVec3& origin1,
										 const PxVec3& dir1,
										 PxReal extent1,

										 PxReal* param0, 
										 PxReal* param1)
{
    const PxVec3 kDiff	= origin0 - origin1;
    const PxReal fA01	= -dir0.dot(dir1);
    const PxReal fB0	= kDiff.dot(dir0);
    const PxReal fB1	= -kDiff.dot(dir1);
	const PxReal fC		= kDiff.magnitudeSquared();
	const PxReal fDet	= PxAbs(1.0f - fA01*fA01);
    PxReal fS0, fS1, fSqrDist, fExtDet0, fExtDet1, fTmpS0, fTmpS1;

    if (fDet >= ZERO_TOLERANCE)
    {
        // segments are not parallel
        fS0 = fA01*fB1-fB0;
        fS1 = fA01*fB0-fB1;
        fExtDet0 = extent0*fDet;
        fExtDet1 = extent1*fDet;

        if (fS0 >= -fExtDet0)
        {
            if (fS0 <= fExtDet0)
            {
                if (fS1 >= -fExtDet1)
                {
                    if (fS1 <= fExtDet1)  // region 0 (interior)
                    {
                        // minimum at two interior points of 3D lines
                        PxReal fInvDet = 1.0f/fDet;
                        fS0 *= fInvDet;
                        fS1 *= fInvDet;
                        fSqrDist = fS0*(fS0+fA01*fS1+2.0f*fB0) +
                            fS1*(fA01*fS0+fS1+2.0f*fB1)+fC;
                    }
                    else  // region 3 (side)
                    {
                        fS1 = extent1;
                        fTmpS0 = -(fA01*fS1+fB0);
                        if (fTmpS0 < -extent0)
                        {
                            fS0 = -extent0;
                            fSqrDist = fS0*(fS0-2.0f*fTmpS0) +
                                fS1*(fS1+2.0f*fB1)+fC;
                        }
                        else if (fTmpS0 <= extent0)
                        {
                            fS0 = fTmpS0;
                            fSqrDist = -fS0*fS0+fS1*(fS1+2.0f*fB1)+fC;
                        }
                        else
                        {
                            fS0 = extent0;
                            fSqrDist = fS0*(fS0-2.0f*fTmpS0) +
                                fS1*(fS1+2.0f*fB1)+fC;
                        }
                    }
                }
                else  // region 7 (side)
                {
                    fS1 = -extent1;
                    fTmpS0 = -(fA01*fS1+fB0);
                    if (fTmpS0 < -extent0)
                    {
                        fS0 = -extent0;
                        fSqrDist = fS0*(fS0-2.0f*fTmpS0) +
                            fS1*(fS1+2.0f*fB1)+fC;
                    }
                    else if (fTmpS0 <= extent0)
                    {
                        fS0 = fTmpS0;
                        fSqrDist = -fS0*fS0+fS1*(fS1+2.0f*fB1)+fC;
                    }
                    else
                    {
                        fS0 = extent0;
                        fSqrDist = fS0*(fS0-2.0f*fTmpS0) +
                            fS1*(fS1+2.0f*fB1)+fC;
                    }
                }
            }
            else
            {
                if (fS1 >= -fExtDet1)
                {
                    if (fS1 <= fExtDet1)  // region 1 (side)
                    {
                        fS0 = extent0;
                        fTmpS1 = -(fA01*fS0+fB1);
                        if (fTmpS1 < -extent1)
                        {
                            fS1 = -extent1;
                            fSqrDist = fS1*(fS1-2.0f*fTmpS1) +
                                fS0*(fS0+2.0f*fB0)+fC;
                        }
                        else if (fTmpS1 <= extent1)
                        {
                            fS1 = fTmpS1;
                            fSqrDist = -fS1*fS1+fS0*(fS0+2.0f*fB0)+fC;
                        }
                        else
                        {
                            fS1 = extent1;
                            fSqrDist = fS1*(fS1-2.0f*fTmpS1) +
                                fS0*(fS0+2.0f*fB0)+fC;
                        }
                    }
                    else  // region 2 (corner)
                    {
                        fS1 = extent1;
                        fTmpS0 = -(fA01*fS1+fB0);
                        if (fTmpS0 < -extent0)
                        {
                            fS0 = -extent0;
                            fSqrDist = fS0*(fS0-2.0f*fTmpS0) +
                                fS1*(fS1+2.0f*fB1)+fC;
                        }
                        else if (fTmpS0 <= extent0)
                        {
                            fS0 = fTmpS0;
                            fSqrDist = -fS0*fS0+fS1*(fS1+2.0f*fB1)+fC;
                        }
                        else
                        {
                            fS0 = extent0;
                            fTmpS1 = -(fA01*fS0+fB1);
                            if (fTmpS1 < -extent1)
                            {
                                fS1 = -extent1;
                                fSqrDist = fS1*(fS1-2.0f*fTmpS1) +
                                    fS0*(fS0+2.0f*fB0)+fC;
                            }
                            else if (fTmpS1 <= extent1)
                            {
                                fS1 = fTmpS1;
                                fSqrDist = -fS1*fS1+fS0*(fS0+2.0f*fB0)
                                    + fC;
                            }
                            else
                            {
                                fS1 = extent1;
                                fSqrDist = fS1*(fS1-2.0f*fTmpS1) +
                                    fS0*(fS0+2.0f*fB0)+fC;
                            }
                        }
                    }
                }
                else  // region 8 (corner)
                {
                    fS1 = -extent1;
                    fTmpS0 = -(fA01*fS1+fB0);
                    if (fTmpS0 < -extent0)
                    {
                        fS0 = -extent0;
                        fSqrDist = fS0*(fS0-2.0f*fTmpS0) +
                            fS1*(fS1+2.0f*fB1)+fC;
                    }
                    else if (fTmpS0 <= extent0)
                    {
                        fS0 = fTmpS0;
                        fSqrDist = -fS0*fS0+fS1*(fS1+2.0f*fB1)+fC;
                    }
                    else
                    {
                        fS0 = extent0;
                        fTmpS1 = -(fA01*fS0+fB1);
                        if (fTmpS1 > extent1)
                        {
                            fS1 = extent1;
                            fSqrDist = fS1*(fS1-2.0f*fTmpS1) +
                                fS0*(fS0+2.0f*fB0)+fC;
                        }
                        else if (fTmpS1 >= -extent1)
                        {
                            fS1 = fTmpS1;
                            fSqrDist = -fS1*fS1+fS0*(fS0+2.0f*fB0)
                                + fC;
                        }
                        else
                        {
                            fS1 = -extent1;
                            fSqrDist = fS1*(fS1-2.0f*fTmpS1) +
                                fS0*(fS0+2.0f*fB0)+fC;
                        }
                    }
                }
            }
        }
        else 
        {
            if (fS1 >= -fExtDet1)
            {
                if (fS1 <= fExtDet1)  // region 5 (side)
                {
                    fS0 = -extent0;
                    fTmpS1 = -(fA01*fS0+fB1);
                    if (fTmpS1 < -extent1)
                    {
                        fS1 = -extent1;
                        fSqrDist = fS1*(fS1-2.0f*fTmpS1) +
                            fS0*(fS0+2.0f*fB0)+fC;
                    }
                    else if (fTmpS1 <= extent1)
                    {
                        fS1 = fTmpS1;
                        fSqrDist = -fS1*fS1+fS0*(fS0+2.0f*fB0)+fC;
                    }
                    else
                    {
                        fS1 = extent1;
                        fSqrDist = fS1*(fS1-2.0f*fTmpS1) +
                            fS0*(fS0+2.0f*fB0)+fC;
                    }
                }
                else  // region 4 (corner)
                {
                    fS1 = extent1;
                    fTmpS0 = -(fA01*fS1+fB0);
                    if (fTmpS0 > extent0)
                    {
                        fS0 = extent0;
                        fSqrDist = fS0*(fS0-2.0f*fTmpS0) +
                            fS1*(fS1+2.0f*fB1)+fC;
                    }
                    else if (fTmpS0 >= -extent0)
                    {
                        fS0 = fTmpS0;
                        fSqrDist = -fS0*fS0+fS1*(fS1+2.0f*fB1)+fC;
                    }
                    else
                    {
                        fS0 = -extent0;
                        fTmpS1 = -(fA01*fS0+fB1);
                        if (fTmpS1 < -extent1)
                        {
                            fS1 = -extent1;
                            fSqrDist = fS1*(fS1-2.0f*fTmpS1) +
                                fS0*(fS0+2.0f*fB0)+fC;
                        }
                        else if (fTmpS1 <= extent1)
                        {
                            fS1 = fTmpS1;
                            fSqrDist = -fS1*fS1+fS0*(fS0+2.0f*fB0)
                                + fC;
                        }
                        else
                        {
                            fS1 = extent1;
                            fSqrDist = fS1*(fS1-2.0f*fTmpS1) +
                                fS0*(fS0+2.0f*fB0)+fC;
                        }
                    }
                }
            }
            else   // region 6 (corner)
            {
                fS1 = -extent1;
                fTmpS0 = -(fA01*fS1+fB0);
                if (fTmpS0 > extent0)
                {
                    fS0 = extent0;
                    fSqrDist = fS0*(fS0-2.0f*fTmpS0) +
                        fS1*(fS1+2.0f*fB1)+fC;
                }
                else if (fTmpS0 >= -extent0)
                {
                    fS0 = fTmpS0;
                    fSqrDist = -fS0*fS0+fS1*(fS1+2.0f*fB1)+fC;
                }
                else
                {
                    fS0 = -extent0;
                    fTmpS1 = -(fA01*fS0+fB1);
                    if (fTmpS1 < -extent1)
                    {
                        fS1 = -extent1;
                        fSqrDist = fS1*(fS1-2.0f*fTmpS1) +
                            fS0*(fS0+2.0f*fB0)+fC;
                    }
                    else if (fTmpS1 <= extent1)
                    {
                        fS1 = fTmpS1;
                        fSqrDist = -fS1*fS1+fS0*(fS0+2.0f*fB0)
                            + fC;
                    }
                    else
                    {
                        fS1 = extent1;
                        fSqrDist = fS1*(fS1-2.0f*fTmpS1) +
                            fS0*(fS0+2.0f*fB0)+fC;
                    }
                }
            }
        }
    }
    else
    {
        // segments are parallel
        PxReal fE0pE1 = extent0 + extent1;
        PxReal fSign = (fA01 > 0.0f ? -1.0f : 1.0f);
        PxReal fLambda = -fB0;
        if (fLambda < -fE0pE1)
        {
            fLambda = -fE0pE1;
        }
        else if (fLambda > fE0pE1)
        {
            fLambda = fE0pE1;
        }

        fS1 = fSign*fB0*extent1/fE0pE1;
        fS0 = fLambda + fSign*fS1;
        fSqrDist = fLambda*(fLambda + 2.0f*fB0) + fC;
    }

	if(param0)	*param0 = fS0;
	if(param1)	*param1 = fS1;

	// account for numerical round-off error
	return physx::intrinsics::selectMax(0.0f, fSqrDist);
}

    // The segment is represented as P+t*D, where P is the segment origin,
    // D is a unit-length direction vector and |t| <= e.  The value e is
    // referred to as the extent of the segment.  The end points of the
    // segment are P-e*D and P+e*D.  The user must ensure that the direction
    // vector is unit-length.  The representation for a segment is analogous
    // to that for an oriented bounding box.  P is the center, D is the
    // axis direction, and e is the extent.

/*    // construction
    Segment3 ();  // uninitialized
    Segment3 (const Vector3<PxReal>& rkOrigin, const Vector3<PxReal>& rkDirection,
        PxReal fExtent);

    // end points
    Vector3<PxReal> GetPosEnd () const;  // P+e*D
    Vector3<PxReal> GetNegEnd () const;  // P-e*D

    Vector3<PxReal> Origin, Direction;
    PxReal Extent;*/


PxReal Gu::distanceSegmentSegmentSquared2(	const Gu::Segment& segment0,
											const Gu::Segment& segment1,
											PxReal* param0, 
											PxReal* param1)
{
	// Some conversion is needed between the old & new code
	// Old:
	// segment (s0, s1)
	// origin = s0
	// extent = s1 - s0
	//
	// New:
	// s0 = origin + extent * dir;
	// s1 = origin - extent * dir;

	// dsequeira: is this really sensible? We use a highly optimized Wild Magic routine, 
	// then use a segment representation that requires an expensive conversion to/from...

	PxVec3 origin0 = segment0.p0;
	PxVec3 origin1 = segment1.p0;
	PxVec3 extent0 = segment0.computeDirection();
	PxVec3 extent1 = segment1.computeDirection();

	const PxVec3 center0 = origin0 + extent0*0.5f;
	PxReal length0 = extent0.magnitude();	//AM: change to make it work for degenerate (zero length) segments.
	const bool b0 = length0 != 0.0f;
	PxReal oneOverLength0(0.0f);
	if (b0)
	{
		oneOverLength0 = 1.0f / length0;
		extent0 *= oneOverLength0;
		length0 *= 0.5f;
	}

	const PxVec3 center1 = origin1 + extent1*0.5f;
	PxReal length1 = extent1.magnitude();
	const bool b1 = length1 != 0.0f;
	PxReal oneOverLength1(0.0f);
	if (b1)
	{
		oneOverLength1 = 1.0f / length1;
		extent1 *= oneOverLength1;
		length1 *= 0.5f;
	}

	// the return param vals have -extent = s0, extent = s1

	PxReal returnValue = distanceSegmentSegmentSquaredNEW(	center0, extent0, length0,
															center1, extent1, length1,
															param0, param1);


	//ML : This is wrong for some reason, I guess it has precision issue
	//// renormalize into the 0 = s0, 1 = s1 range
	//if (param0)
	//	*param0 = b0 ? ((*param0) * oneOverLength0 * 0.5f + 0.5f) : 0.0f;
	//if (param1)
	//	*param1 = b1 ? ((*param1) * oneOverLength1 * 0.5f + 0.5f) : 0.0f;


	if (param0)
		*param0 = b0 ? ((length0 + (*param0))*oneOverLength0) : 0.0f;
	if (param1)
		*param1 = b1 ? ((length1 + (*param1))*oneOverLength1) : 0.0f;

	return returnValue;
}
//
///*
//	S0 = origin + extent * dir;
//	S1 = origin + extent * dir;
//	dir is the vector from start to end point
//	p1 is the start point of segment1
//	q1 is the end point of segment1
//	p2 is the start point of segment2
//	q2 is the end point of segment2
//*/
//
//Ps::aos::FloatV Gu::distanceSegmentSegmentSquared(const Ps::aos::Vec3VArg p1, 
//														const Ps::aos::Vec3VArg q1,
//														const Ps::aos::Vec3VArg p2, 
//														const Ps::aos::Vec3VArg q2,
//														Ps::aos::FloatV& s, 
//														Ps::aos::FloatV& t)
//{
//	using namespace Ps::aos;
//	const FloatV zero = FZero();
//	const FloatV one = FOne();
//	const FloatV eps = FEps();
//	const FloatV parallelTolerance  = FloatV_From_F32(PX_PARALLEL_TOLERANCE);
//
//	const Vec3V d1 = V3Sub(q1, p1); //direction vector of segment1
//	const Vec3V d2 = V3Sub(q2, p2); //direction vector of segment2
//	const Vec3V r = V3Sub(p1, p2);
//	const FloatV a = V3Dot(d1, d1);//squared length of segment1
//	const FloatV e = V3Dot(d2, d2);//squared length of segment2
//	const FloatV aRecip = FRecip(a);
//	const FloatV eRecip = FRecip(e);
//	const FloatV f = V3Dot(d2, r);
//	const FloatV c = V3Dot(d1, r);
//	const FloatV nc = FNeg(c);
//	
//
//	//check if either or both segments degenerate into points
//	const BoolV con0 = FIsGrtr(eps, a);
//	const BoolV con1 = FIsGrtr(eps, e);
//
//	PxU32 v0, v1; 
//	Store_From_BoolV(con0, &v0);
//	Store_From_BoolV(con1, &v1);
//
//	if(v0&v1)
//	{
//		s = zero;
//		t = zero;
//		return V3Dot(r, r);
//	}
//	
//	if(v0)
//	{
//		//First segment degenerates into a point
//		s = zero;
//		const FloatV temp = FMul(f, eRecip);
//		t= FClamp(temp, zero, one);
//	}
//	else
//	{
//		const FloatV d = FMul(nc, aRecip);
//		if(v1)//second segment degenerates into a point
//		{
//			s = zero;
//			t = FClamp(d, zero, one);
//		}
//		else
//		{
//			//the general nondegenerate case
//			const FloatV b = V3Dot(d1, d2);
//			const FloatV denom = FSub(FMul(a, e), FMul(b, b));
//			const FloatV denomRecip = FRecip(denom);
//			const FloatV temp = FSub(FMul(b, f), FMul(c, e));
//			const FloatV s0 = FMul(temp, denomRecip);
//			
//			//if segments not parallell, compute closest point on two segments and clamp to segment1, else pick arbitrary param0
//			const BoolV con2 = FIsGrtrOrEq( parallelTolerance, FAbs(denom));
//			PxU32 v2; 
//			Store_From_BoolV(con2, &v2);
//			if(v2)
//			{
//				//segment is parallel
//				
//				const FloatV a01 = FNeg(V3Dot(d1, d2));
//				const FloatV a01Recip = FRecip(a01);
//
//				const FloatV tmp0 = FNeg(FAdd(a, c));
//				const FloatV tmp1 = FMul(nc, a);
//				const FloatV tmp2 = FMul(nc, aRecip);
//				const FloatV tmp3 = FMul(tmp0, a01Recip);
//				const FloatV tmp4 = FMul(nc, a01Recip);
//
//				const BoolV con00 = FIsGrtr(a01, zero);
//				PxU32 v00; 
//				Store_From_BoolV(con00, &v00);
//
//				if ( v00)
//				{
//					const BoolV con01 = FIsGrtrOrEq(c, zero);
//					const BoolV con02 = FIsGrtr(a, nc);
//					const BoolV con3 = BOr(con01, con02);
//					//const FloatV tmp0 = FNeg(FAdd(a, c));
//					s = FSel(con01, zero, FSel(con02,tmp1, one));
//					//t = FSel(con01, zero, FSel(con02, zero, FSel(FIsGrtrOrEq(tmp0, a01), one, FDiv(tmp0, a01))));
//					t = FSel(con3, zero, FSel(FIsGrtrOrEq(tmp0, a01), one, tmp3));
//
//				}
//				else
//				{
//					const BoolV con03 = FIsGrtrOrEq(nc, a);
//					const BoolV con04 = FIsGrtr(zero, c);
//					const BoolV con5 = BOr(con03, con04);
//
//					s = FSel(con03, one, FSel(con04, tmp2, zero));
//					//t = FSel(con03, zero, FSel(con04, zero, FSel(FIsGrtrOrEq(c, FNeg(a01)), one,  FDiv(nc, a01)) ));
//					t = FSel(con5, zero, FSel(FIsGrtrOrEq(c, FNeg(a01)), one,  tmp4));
//				}
//
//			}
//			else
//			{
//
//				s = FSel(con2, zero, FClamp(s0, zero, one));
//			
//				//compute point on segment2 closest to segement1
//				t = FMul(FAdd(FMul(b, s), f), eRecip);
//
//				//if param1 is in [zero, one], done. otherwise clamp param1 and recompute param0 for the new value
//				const BoolV con3 = FIsGrtrOrEq(zero, t);
//				const BoolV con4 = FIsGrtr(t, one);
//
//				const FloatV par0 = FClamp(d, zero, one);
//				const FloatV par1 = FClamp(FMul(FSub(b, c), aRecip), zero, one);
//
//				t = FSel(con3, zero, FSel(con4, one,  t));
//				s = FSel(con3, par0, FSel(con4, par1, s));
//			}
//
//		}
//	}
//
//	const Vec3V closest1 = V3Add(p1, V3Scale(d1, s));
//	const Vec3V closest2 = V3Add(p2, V3Scale(d2, t));
//	const Vec3V vv = V3Sub(closest1, closest2);
//	return V3Dot(vv, vv);
//
//}



/*
	S0 = origin + extent * dir;
	S1 = origin + extent * dir;
	dir is the vector from start to end point
	p1 is the start point of segment1
	d1 is the direction vector(q1 - p1)
	p2 is the start point of segment2
	d2 is the direction vector(q2 - p2) 
*/

Ps::aos::FloatV Gu::distanceSegmentSegmentSquared(const Ps::aos::Vec3VArg p1, 
														const Ps::aos::Vec3VArg d1,
														const Ps::aos::Vec3VArg p2, 
														const Ps::aos::Vec3VArg d2,
														Ps::aos::FloatV& s, 
														Ps::aos::FloatV& t)
{
	using namespace Ps::aos;
	const FloatV zero = FZero();
	const FloatV one = FOne();
	const FloatV half = FloatV_From_F32(0.5f);
	const FloatV eps = FEps();
	//const FloatV parallelTolerance  = FloatV_From_F32(PX_PARALLEL_TOLERANCE);

	//const Vec3V d1 = V3Sub(q1, p1); //direction vector of segment1
	//const Vec3V d2 = V3Sub(q2, p2); //direction vector of segment2
	const Vec3V r = V3Sub(p1, p2);
	const FloatV a = V3Dot(d1, d1);//squared length of segment1
	const FloatV e = V3Dot(d2, d2);//squared length of segment2
	const FloatV aRecip = FRecip(a);
	const FloatV eRecip = FRecip(e);
	const FloatV b = V3Dot(d1, d2);
	const FloatV c = V3Dot(d1, r);
	const FloatV nc = FNeg(c);
	const FloatV d = FMul(nc, aRecip);//when t = zero
	const FloatV cd = FClamp(d, zero, one);
	const FloatV f = V3Dot(d2, r);
	const FloatV g = FMul(f, eRecip);
	const FloatV cg = FClamp(g, zero, one); //when s = zero
	//const FloatV h = FMul(nc, a);

	/*
		s = (b*f - c*e)/(a*e - b*b);
		t = (a*f - b*c)/(a*e - b*b);

		s = (b*t - c)/a;
		t = (b*s + f)/e;
	*/
	

	//check if either or both segments degenerate into points
	const BoolV con0 = FIsGrtr(eps, a);
	const BoolV con1 = FIsGrtr(eps, e);
		
	//the general nondegenerate case

	//if segments not parallell, compute closest point on two segments and clamp to segment1
	const FloatV denom = FSub(FMul(a, e), FMul(b, b));
	const FloatV denomRecip = FRecip(denom);
	const FloatV temp = FSub(FMul(b, f), FMul(c, e));
	const FloatV s0 = FClamp(FMul(temp, denomRecip), zero, one);
	
	//if segment is parallel
	//const BoolV con00 = FIsGrtr(b, zero);//segment p1q1 and p2q2 point to the same direction
	//const BoolV con01 = FIsGrtrOrEq(c, zero);
	//const BoolV con02 = FIsGrtr(a, nc);

	//const FloatV s01 = FSel(con01, zero, FSel(con02, h, one));
	//const FloatV s02 = FSel(con02, FSel(con01, zero, d), one);
	//const FloatV s1 = FSel(con00, s02, s01);

	
	//const BoolV con2 = FIsGrtrOrEq( parallelTolerance, FAbs(denom));
	const BoolV con2 = FIsEq(denom, zero);
	const FloatV sTmp = FSel(con2, half, s0);
	
	//compute point on segment2 closest to segement1
	const FloatV tTmp = FMul(FAdd(FMul(b, sTmp), f), eRecip);

	//if t is in [zero, one], done. otherwise clamp t
	const FloatV t2 = FClamp(tTmp, zero, one);

	//recompute s for the new value
	const FloatV comp = FMul(FSub(FMul(b,t2), c), aRecip);
	const FloatV s2 = FClamp(comp, zero, one);

	s = FSel(con0, zero, FSel(con1, cd, s2));
	t = FSel(con1, zero, FSel(con0, cg, t2));

	const Vec3V closest1 = V3Add(p1, V3Scale(d1, s));
	const Vec3V closest2 = V3Add(p2, V3Scale(d2, t));
	const Vec3V vv = V3Sub(closest1, closest2);
	return V3Dot(vv, vv);
}


void Gu::distanceSegmentSegmentSquaredNoClamp(const Ps::aos::Vec3VArg p1, 
														const Ps::aos::Vec3VArg d1,
														const Ps::aos::Vec3VArg p2, 
														const Ps::aos::Vec3VArg d2,
														Ps::aos::FloatV& s, 
														Ps::aos::FloatV& t)
{
	using namespace Ps::aos;
	const FloatV zero = FZero();
	const FloatV one = FOne();
	//const FloatV two = FloatV_From_F32(2.f);
	const FloatV eps = FEps();
	const FloatV parallelTolerance  = FloatV_From_F32(PX_PARALLEL_TOLERANCE);

	//const Vec3V d1 = V3Sub(q1, p1); //direction vector of segment1
	//const Vec3V d2 = V3Sub(q2, p2); //direction vector of segment2
	const Vec3V r = V3Sub(p1, p2);
	const FloatV a = V3Dot(d1, d1);//squared length of segment1
	const FloatV e = V3Dot(d2, d2);//squared length of segment2
	const FloatV aRecip = FRecip(a);
	const FloatV eRecip = FRecip(e);
	const FloatV b = V3Dot(d1, d2);
	const FloatV c = V3Dot(d1, r);
	const FloatV nc = FNeg(c);
	const FloatV d = FMul(nc, aRecip);//when t = zero
	const FloatV cd = FClamp(d, zero, one);
	const FloatV f = V3Dot(d2, r);
	const FloatV g = FMul(f, eRecip);
	const FloatV cg = FClamp(g, zero, one); //when s = zero
	const FloatV h = FMul(nc, a);

	/*
		s = (b*f - c*e)/(a*e - b*b);
		t = (a*f - b*c)/(a*e - b*b);

		s = (b*t - c)/a;
		t = (b*s + f)/e;
	*/
	

	//check if either or both segments degenerate into points
	const BoolV con0 = FIsGrtr(eps, a);
	const BoolV con1 = FIsGrtr(eps, e);
		
	//the general nondegenerate case

	//if segments not parallell, compute closest point on two segments and clamp to segment1
	const FloatV denom = FSub(FMul(a, e), FMul(b, b));
	const FloatV denomRecip = FRecip(denom);
	const FloatV temp = FSub(FMul(b, f), FMul(c, e));
	const FloatV s0 = FMul(temp, denomRecip);
	
	//if segment is parallel
	const BoolV con00 = FIsGrtr(b, zero);//segment p1q1 and p2q2 point to the same direction
	const BoolV con01 = FIsGrtrOrEq(c, zero);
	const BoolV con02 = FIsGrtr(a, nc);

	const FloatV s01 = FSel(con01, zero, FSel(con02, h, one));
	const FloatV s02 = FSel(con02, FSel(con01, zero, d), one);
	const FloatV s1 = FSel(con00, s02, s01);

	
	const BoolV con2 = FIsGrtrOrEq( parallelTolerance, FAbs(denom));
	const FloatV s2 = FSel(con2, s1, s0);
	
	//compute point on segment2 closest to segement1
	const FloatV t2 = FMul(FAdd(FMul(b, s2), f), eRecip);

	s = FSel(con0, zero, FSel(con1, cd, s2));
	t = FSel(con1, zero, FSel(con0, cg, t2));
}


/*
	segment p01q01 and segment p02q02
	segment p11q11 and segment p12q12
	segment p21q21 and segment p22q22
	segment p31q31 and segment p32q32
*/
Ps::aos::Vec4V Gu::distanceSegmentSegmentSquared4(		const Ps::aos::Vec3VArg p01, const Ps::aos::Vec3VArg q01, 
														const Ps::aos::Vec3VArg p02, const Ps::aos::Vec3VArg q02, 
														const Ps::aos::Vec3VArg p11, const Ps::aos::Vec3VArg q11,
                                                        const Ps::aos::Vec3VArg p12, const Ps::aos::Vec3VArg q12, 
														const Ps::aos::Vec3VArg p21, const Ps::aos::Vec3VArg q21, 
														const Ps::aos::Vec3VArg p22, const Ps::aos::Vec3VArg q22,
                                                        const Ps::aos::Vec3VArg p31, const Ps::aos::Vec3VArg q31,
                                                        const Ps::aos::Vec3VArg p32, const Ps::aos::Vec3VArg q32,
                                                        Ps::aos::Vec4V& s, Ps::aos::Vec4V& t)
{
      using namespace Ps::aos;
      const Vec4V zero = V4Zero();
      const Vec4V one = V4One();
      const Vec4V eps = V4Eps();
      const Vec4V parallelTolerance  = Vec4V_From_F32(PX_PARALLEL_TOLERANCE);

      const Vec3V d01 = V3Sub(q01, p01); //direction vector of segment1
      const Vec3V d02 = V3Sub(q02, p02); //direction vector of segment2
      const Vec3V r0 = V3Sub(p01, p02);

      const Vec3V d11 = V3Sub(q11, p11); //direction vector of segment1
      const Vec3V d12 = V3Sub(q12, p12); //direction vector of segment2
      const Vec3V r1 = V3Sub(p11, p12);

      const Vec3V d21 = V3Sub(q21, p21); //direction vector of segment1
      const Vec3V d22 = V3Sub(q22, p22); //direction vector of segment2
      const Vec3V r2 = V3Sub(p21, p22);

      const Vec3V d31 = V3Sub(q31, p31); //direction vector of segment1
      const Vec3V d32 = V3Sub(q32, p32); //direction vector of segment2
      const Vec3V r3 = V3Sub(p31, p32);

	  //TODO - store this in a transposed state and avoid so many dot products?
      const FloatV _a[4] = { V3Dot(d01, d01), V3Dot(d11, d11), V3Dot(d21, d21), V3Dot(d31, d31) };
      const FloatV _e[4] = { V3Dot(d02, d02), V3Dot(d12, d12), V3Dot(d22, d22), V3Dot(d32, d32) };

      const FloatV _b[4] = { V3Dot(d01, d02), V3Dot(d11, d12), V3Dot(d21, d22), V3Dot(d31, d32) };
      const FloatV _c[4] = { V3Dot(d01, r0), V3Dot(d11, r1), V3Dot(d21, r2), V3Dot(d31, r3) };
      const FloatV _f[4] = { V3Dot(d02, r0), V3Dot(d12, r1), V3Dot(d22, r2), V3Dot(d32, r3) };


      const Vec4V a(V4Merge(_a));
      const Vec4V e(V4Merge(_e));

      const Vec4V b(V4Merge(_b));
      const Vec4V c(V4Merge(_c));
      const Vec4V f(V4Merge(_f));

      const Vec4V aRecip(V4Recip(a));
      const Vec4V eRecip(V4Recip(e));

      const Vec4V nc(V4Neg(c));
      const Vec4V d(V4Mul(nc, aRecip));
      const Vec4V cd(V4Clamp(d, zero, one));
      
      
      const Vec4V g = V4Mul(f, eRecip);
      const Vec4V cg = V4Clamp(g, zero, one); //when s = zero
      const Vec4V h = V4Mul(nc, a);
      

      //check if either or both segments degenerate into points
      const BoolV con0 = V4IsGrtr(eps, a);
      const BoolV con1 = V4IsGrtr(eps, e);
            
      //the general nondegenerate case

      //if segments not parallell, compute closest point on two segments and clamp to segment1
      const Vec4V denom = V4Sub(V4Mul(a, e), V4Mul(b, b));
      const Vec4V denomRecip = V4Recip(denom);
      const Vec4V temp = V4Sub(V4Mul(b, f), V4Mul(c, e));
      const Vec4V s0 = V4Clamp(V4Mul(temp, denomRecip), zero, one);
      
      //if segment is parallel
      const BoolV con00 = V4IsGrtr(b, zero);//segment p1q1 and p2q2 point to the same direction
      const BoolV con01 = V4IsGrtrOrEq(c, zero);
      const BoolV con02 = V4IsGrtr(a, nc);

      const Vec4V s01 = V4Sel(con01, zero, V4Sel(con02, h, one));
      const Vec4V s02 = V4Sel(con02, V4Sel(con01, zero, d), one);
      const Vec4V s1 = V4Sel(con00, s02, s01);

      
      const BoolV con2 = V4IsGrtrOrEq( parallelTolerance, V4Abs(denom));
      const Vec4V sTmp = V4Sel(con2, s1, s0);
      
      //compute point on segment2 closest to segement1
      const Vec4V tTmp = V4Mul(V4Add(V4Mul(b, sTmp), f), eRecip);

      //if t is in [zero, one], done. otherwise clamp t
      const Vec4V t2 = V4Clamp(tTmp, zero, one);

      //recompute s for the new value
      const Vec4V comp = V4Mul(V4Sub(V4Mul(b,t2), c), aRecip);
      const Vec4V s2 = V4Clamp(comp, zero, one);

      s = V4Sel(con0, zero, V4Sel(con1, cd, s2));
      t = V4Sel(con1, zero, V4Sel(con0, cg, t2));

	  const Vec3V closest01 = V3MulAdd(d01, V4GetX(s), p01);
      const Vec3V closest02 = V3MulAdd(d02, V4GetX(t), p02);
      const Vec3V closest11 = V3MulAdd(d11, V4GetY(s), p11);
      const Vec3V closest12 = V3MulAdd(d12, V4GetY(t), p12);
      const Vec3V closest21 = V3MulAdd(d21, V4GetZ(s), p21);
      const Vec3V closest22 = V3MulAdd(d22, V4GetZ(t), p22);
      const Vec3V closest31 = V3MulAdd(d31, V4GetW(s), p31);
      const Vec3V closest32 = V3MulAdd(d32, V4GetW(t), p32);

      const Vec3V vv0 = V3Sub(closest01, closest02);
      const Vec3V vv1 = V3Sub(closest11, closest12);
      const Vec3V vv2 = V3Sub(closest21, closest22);
      const Vec3V vv3 = V3Sub(closest31, closest32);

      const FloatV vd[4] = {V3Dot(vv0, vv0), V3Dot(vv1, vv1), V3Dot(vv2, vv2), V3Dot(vv3, vv3)};
      return V4Merge(vd);
}



/*
	segment pq and segment p02q02
	segment pq and segment p12q12
	segment pq and segment p22q22
	segment pq and segment p32q32
*/
Ps::aos::Vec4V Gu::distanceSegmentSegmentSquared4(		const Ps::aos::Vec3VArg p, const Ps::aos::Vec3VArg d0, 
														const Ps::aos::Vec3VArg p02, const Ps::aos::Vec3VArg d02, 
                                                        const Ps::aos::Vec3VArg p12, const Ps::aos::Vec3VArg d12, 
														const Ps::aos::Vec3VArg p22, const Ps::aos::Vec3VArg d22,
                                                        const Ps::aos::Vec3VArg p32, const Ps::aos::Vec3VArg d32,
                                                        Ps::aos::Vec4V& s, Ps::aos::Vec4V& t)
{
      using namespace Ps::aos;
      const Vec4V zero = V4Zero();
      const Vec4V one = V4One();
      const Vec4V eps = V4Eps();
      const Vec4V parallelTolerance  = Vec4V_From_F32(PX_PARALLEL_TOLERANCE);

      const Vec3V r0 = V3Sub(p, p02);
      const Vec3V r1 = V3Sub(p, p12);
      const Vec3V r2 = V3Sub(p, p22);
      const Vec3V r3 = V3Sub(p, p32);

	  //TODO - store this in a transposed state and avoid so many dot products?

	  const FloatV dd = V3Dot(d0, d0);


      const FloatV _e[4] = { V3Dot(d02, d02), V3Dot(d12, d12), V3Dot(d22, d22), V3Dot(d32, d32) };

      const FloatV _b[4] = { V3Dot(d0, d02), V3Dot(d0, d12), V3Dot(d0, d22), V3Dot(d0, d32) };
      const FloatV _c[4] = { V3Dot(d0, r0), V3Dot(d0, r1), V3Dot(d0, r2), V3Dot(d0, r3) };
      const FloatV _f[4] = { V3Dot(d02, r0), V3Dot(d12, r1), V3Dot(d22, r2), V3Dot(d32, r3) };


      const Vec4V a(V4Splat(dd));
      const Vec4V e(V4Merge(_e));

      const Vec4V b(V4Merge(_b));
      const Vec4V c(V4Merge(_c));

	  /*const Vec4V b(M44MulV4(m0T, d0));
	  const Vec4V c(M44MulV4(m1T, d0));*/

      const Vec4V f(V4Merge(_f));

      const Vec4V aRecip(V4Recip(a));
      const Vec4V eRecip(V4Recip(e));

      const Vec4V nc(V4Neg(c));
      const Vec4V d(V4Mul(nc, aRecip));
      const Vec4V cd(V4Clamp(d, zero, one));
      
      
      const Vec4V g = V4Mul(f, eRecip);
      const Vec4V cg = V4Clamp(g, zero, one); //when s = zero
      const Vec4V h = V4Mul(nc, a);
      

      //check if either or both segments degenerate into points
      const BoolV con0 = V4IsGrtr(eps, a);
      const BoolV con1 = V4IsGrtr(eps, e);
            
      //the general nondegenerate case

      //if segments not parallell, compute closest point on two segments and clamp to segment1
      const Vec4V denom = V4Sub(V4Mul(a, e), V4Mul(b, b));
      const Vec4V denomRecip = V4Recip(denom);
      const Vec4V temp = V4Sub(V4Mul(b, f), V4Mul(c, e));
      const Vec4V s0 = V4Clamp(V4Mul(temp, denomRecip), zero, one);
      
      //if segment is parallel
      const BoolV con00 = V4IsGrtr(b, zero);//segment p1q1 and p2q2 point to the same direction
      const BoolV con01 = V4IsGrtrOrEq(c, zero);
      const BoolV con02 = V4IsGrtr(a, nc);

      const Vec4V s01 = V4Sel(con01, zero, V4Sel(con02, h, one));
      const Vec4V s02 = V4Sel(con02, V4Sel(con01, zero, d), one);
      const Vec4V s1 = V4Sel(con00, s02, s01);

      
      const BoolV con2 = V4IsGrtrOrEq( parallelTolerance, V4Abs(denom));
      const Vec4V sTmp = V4Sel(con2, s1, s0);
      
      //compute point on segment2 closest to segement1
      const Vec4V tTmp = V4Mul(V4Add(V4Mul(b, sTmp), f), eRecip);

      //if t is in [zero, one], done. otherwise clamp t
      const Vec4V t2 = V4Clamp(tTmp, zero, one);

      //recompute s for the new value
      const Vec4V comp = V4Mul(V4Sub(V4Mul(b,t2), c), aRecip);
      const Vec4V s2 = V4Clamp(comp, zero, one);

      s = V4Sel(con0, zero, V4Sel(con1, cd, s2));
      t = V4Sel(con1, zero, V4Sel(con0, cg, t2));

	  const Vec3V closest01 = V3MulAdd(d0, V4GetX(s), p);
      const Vec3V closest02 = V3MulAdd(d02, V4GetX(t), p02);
      const Vec3V closest11 = V3MulAdd(d0, V4GetY(s), p);
      const Vec3V closest12 = V3MulAdd(d12, V4GetY(t), p12);
      const Vec3V closest21 = V3MulAdd(d0, V4GetZ(s), p);
      const Vec3V closest22 = V3MulAdd(d22, V4GetZ(t), p22);
      const Vec3V closest31 = V3MulAdd(d0, V4GetW(s), p);
      const Vec3V closest32 = V3MulAdd(d32, V4GetW(t), p32);

      const Vec3V vv0 = V3Sub(closest01, closest02);
      const Vec3V vv1 = V3Sub(closest11, closest12);
      const Vec3V vv2 = V3Sub(closest21, closest22);
      const Vec3V vv3 = V3Sub(closest31, closest32);

      const FloatV vd[4] = {V3Dot(vv0, vv0), V3Dot(vv1, vv1), V3Dot(vv2, vv2), V3Dot(vv3, vv3)};
      return V4Merge(vd);
}

/*
	segment p(p+d0) and segment p01(p02 + d1)
	segment p(p+d0) and segment p12(p12 + d1)
	segment p(p+d0) and segment p22(p22 + d1)
	segment p(p+d0) and segment p32(p32 + d1)
*/
void Gu::distanceSegmentSegmentSquared4(		const Ps::aos::Vec3VArg p, const Ps::aos::Vec3VArg d0, 
														const Ps::aos::Vec3VArg p02,  
                                                        const Ps::aos::Vec3VArg p12,  
														const Ps::aos::Vec3VArg p22, 
                                                        const Ps::aos::Vec3VArg p32, 
														const Ps::aos::Vec3VArg d1,
                                                        Ps::aos::FloatV* s, Ps::aos::FloatV* t,
														Ps::aos::Vec3V* closest,
														Ps::aos::FloatV* sqDist)
{
      using namespace Ps::aos;
      const Vec4V zero = V4Zero();
      const Vec4V one = V4One();
      const Vec4V eps = V4Eps();
      const Vec4V parallelTolerance  = Vec4V_From_F32(PX_PARALLEL_TOLERANCE);

      const Vec3V r0 = V3Sub(p, p02);
      const Vec3V r1 = V3Sub(p, p12);
      const Vec3V r2 = V3Sub(p, p22);
      const Vec3V r3 = V3Sub(p, p32);

	  //TODO - store this in a transposed state and avoid so many dot products?

	  const FloatV dd0 = V3Dot(d0, d0);
	  const FloatV dd1 = V3Dot(d1, d1);
	  const FloatV d0d1 = V3Dot(d0, d1);
	
      const FloatV _c[4] = { V3Dot(d0, r0), V3Dot(d0, r1), V3Dot(d0, r2), V3Dot(d0, r3) };
      const FloatV _f[4] = { V3Dot(d1, r0), V3Dot(d1, r1), V3Dot(d1, r2), V3Dot(d1, r3) };


      const Vec4V a(V4Splat(dd0));
      const Vec4V e(V4Splat(dd1));

      const Vec4V b(V4Splat(d0d1));
      const Vec4V c(V4Merge(_c));
      const Vec4V f(V4Merge(_f));

      const Vec4V aRecip(V4Recip(a));
      const Vec4V eRecip(V4Recip(e));

      const Vec4V nc(V4Neg(c));
      const Vec4V d(V4Mul(nc, aRecip));
      const Vec4V cd(V4Clamp(d, zero, one));
      
      
      const Vec4V g = V4Mul(f, eRecip);
      const Vec4V cg = V4Clamp(g, zero, one); //when s = zero
      const Vec4V h = V4Mul(nc, a);
      

      //check if either or both segments degenerate into points
      const BoolV con0 = V4IsGrtr(eps, a);
      const BoolV con1 = V4IsGrtr(eps, e);
            
      //the general nondegenerate case

      //if segments not parallell, compute closest point on two segments and clamp to segment1
      const Vec4V denom = V4Sub(V4Mul(a, e), V4Mul(b, b));
      const Vec4V denomRecip = V4Recip(denom);
      const Vec4V temp = V4Sub(V4Mul(b, f), V4Mul(c, e));
      const Vec4V s0 = V4Clamp(V4Mul(temp, denomRecip), zero, one);
      
      //if segment is parallel
      const BoolV con00 = V4IsGrtr(b, zero);//segment p1q1 and p2q2 point to the same direction
      const BoolV con01 = V4IsGrtrOrEq(c, zero);
      const BoolV con02 = V4IsGrtr(a, nc);

      const Vec4V s01 = V4Sel(con01, zero, V4Sel(con02, h, one));
      const Vec4V s02 = V4Sel(con02, V4Sel(con01, zero, d), one);
      const Vec4V s1 = V4Sel(con00, s02, s01);

      
      const BoolV con2 = V4IsGrtrOrEq( parallelTolerance, V4Abs(denom));
      const Vec4V sTmp = V4Sel(con2, s1, s0);
      
      //compute point on segment2 closest to segement1
      const Vec4V tTmp = V4Mul(V4Add(V4Mul(b, sTmp), f), eRecip);

      //if t is in [zero, one], done. otherwise clamp t
      const Vec4V t2 = V4Clamp(tTmp, zero, one);

      //recompute s for the new value
      const Vec4V comp = V4Mul(V4Sub(V4Mul(b,t2), c), aRecip);
      const Vec4V s2 = V4Clamp(comp, zero, one);

      const Vec4V sv = V4Sel(con0, zero, V4Sel(con1, cd, s2));
      const Vec4V tv = V4Sel(con1, zero, V4Sel(con0, cg, t2));
	  s[0] = V4GetX(sv);
	  s[1] = V4GetY(sv);
	  s[2] = V4GetZ(sv);
	  s[3] = V4GetW(sv);

	  t[0] = V4GetX(tv);
	  t[1] = V4GetY(tv);
	  t[2] = V4GetZ(tv);
	  t[3] = V4GetW(tv);

	  const Vec3V closest01 = V3MulAdd(d0, s[0], p);
      const Vec3V closest11 = V3MulAdd(d0, s[1], p);
	  const Vec3V closest21 = V3MulAdd(d0, s[2], p);
	  const Vec3V closest31 = V3MulAdd(d0, s[3], p);

	  const Vec3V closest02 = V3MulAdd(d1, t[0], p02);
      const Vec3V closest12 = V3MulAdd(d1, t[1], p12);
      const Vec3V closest22 = V3MulAdd(d1, t[2], p22);
      const Vec3V closest32 = V3MulAdd(d1, t[3], p32);

	  closest[0] = closest02;
	  closest[1] = closest12;
	  closest[2] = closest22;
	  closest[3] = closest32;

      const Vec3V vv0 = V3Sub(closest01, closest02);
      const Vec3V vv1 = V3Sub(closest11, closest12);
      const Vec3V vv2 = V3Sub(closest21, closest22);
      const Vec3V vv3 = V3Sub(closest31, closest32);

	  sqDist[0] = V3Dot(vv0, vv0);
	  sqDist[1] = V3Dot(vv1, vv1);
	  sqDist[2] = V3Dot(vv2, vv2);
	  sqDist[3] = V3Dot(vv3, vv3);
}


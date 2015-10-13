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


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef OPC_COMMON_COLLIDER_OVERLAP_H
#define OPC_COMMON_COLLIDER_OVERLAP_H

namespace physx
{
namespace Ice
{

//! This macro quickly finds the minimum & maximum values among 3 variables
#define FINDMINMAX(x0, x1, x2, minimum, maximum)	\
	minimum = maximum = x0;							\
	if(x1<minimum) minimum=x1;						\
	if(x1>maximum) maximum=x1;						\
	if(x2<minimum) minimum=x2;						\
	if(x2>maximum) maximum=x2;

//! TO BE DOCUMENTED
PX_FORCE_INLINE Ps::IntBool planeBoxOverlap(const PxVec3& normal, const float d, const PxVec3& maxbox)
{
	PxVec3 vmin, vmax;
	for(PxU32 q=0;q<=2;q++)
	{
		if(normal[q]>0.0f)	{ vmin[q]=-maxbox[q]; vmax[q]=maxbox[q]; }
		else				{ vmin[q]=maxbox[q]; vmax[q]=-maxbox[q]; }
	}
	if((normal.dot(vmin))+d>0.0f) return Ps::IntFalse;
	if((normal.dot(vmax))+d>=0.0f) return Ps::IntTrue;

	return Ps::IntFalse;
}

//! TO BE DOCUMENTED
#define AXISTEST_X01(a, b, fa, fb)							\
	minimum = a*v0.y - b*v0.z;									\
	maximum = a*v2.y - b*v2.z;									\
	if(minimum>maximum) {const float tmp=maximum; maximum=minimum; minimum=tmp;	}	\
	rad = fa * extents.y + fb * extents.z;					\
	if(minimum>rad || maximum<-rad) return Ps::IntFalse;

//! TO BE DOCUMENTED
#define AXISTEST_X2(a, b, fa, fb)							\
	minimum = a*v0.y - b*v0.z;									\
	maximum = a*v1.y - b*v1.z;									\
	if(minimum>maximum) {const float tmp=maximum; maximum=minimum; minimum=tmp;	}	\
	rad = fa * extents.y + fb * extents.z;					\
	if(minimum>rad || maximum<-rad) return Ps::IntFalse;

//! TO BE DOCUMENTED
#define AXISTEST_Y02(a, b, fa, fb)							\
	minimum = b*v0.z - a*v0.x;									\
	maximum = b*v2.z - a*v2.x;									\
	if(minimum>maximum) {const float tmp=maximum; maximum=minimum; minimum=tmp;	}	\
	rad = fa * extents.x + fb * extents.z;					\
	if(minimum>rad || maximum<-rad) return Ps::IntFalse;

//! TO BE DOCUMENTED
#define AXISTEST_Y1(a, b, fa, fb)							\
	minimum = b*v0.z - a*v0.x;									\
	maximum = b*v1.z - a*v1.x;									\
	if(minimum>maximum) {const float tmp=maximum; maximum=minimum; minimum=tmp;	}	\
	rad = fa * extents.x + fb * extents.z;					\
	if(minimum>rad || maximum<-rad) return Ps::IntFalse;

//! TO BE DOCUMENTED
#define AXISTEST_Z12(a, b, fa, fb)							\
	minimum = a*v1.x - b*v1.y;									\
	maximum = a*v2.x - b*v2.y;									\
	if(minimum>maximum) {const float tmp=maximum; maximum=minimum; minimum=tmp;	}	\
	rad = fa * extents.x + fb * extents.y;					\
	if(minimum>rad || maximum<-rad) return Ps::IntFalse;

//! TO BE DOCUMENTED
#define AXISTEST_Z0(a, b, fa, fb)							\
	minimum = a*v0.x - b*v0.y;									\
	maximum = a*v1.x - b*v1.y;									\
	if(minimum>maximum) {const float tmp=maximum; maximum=minimum; minimum=tmp;	}	\
	rad = fa * extents.x + fb * extents.y;					\
	if(minimum>rad || maximum<-rad) return Ps::IntFalse;

// compute triangle edges
// - edges lazy evaluated to take advantage of early exits
// - fabs precomputed (half less work, possible since extents are always >0)
// - customized macros to take advantage of the NULL component
// - axis vector discarded, possibly saves useless movs
#define IMPLEMENT_CLASS3_TESTS						\
	float rad;										\
	float minimum, maximum;									\
													\
	const float fey0 = PxAbs(e0.y);			\
	const float fez0 = PxAbs(e0.z);			\
	AXISTEST_X01(e0.z, e0.y, fez0, fey0);			\
	const float fex0 = PxAbs(e0.x);			\
	AXISTEST_Y02(e0.z, e0.x, fez0, fex0);			\
	AXISTEST_Z12(e0.y, e0.x, fey0, fex0);			\
													\
	const float fey1 = PxAbs(e1.y);			\
	const float fez1 = PxAbs(e1.z);			\
	AXISTEST_X01(e1.z, e1.y, fez1, fey1);			\
	const float fex1 = PxAbs(e1.x);			\
	AXISTEST_Y02(e1.z, e1.x, fez1, fex1);			\
	AXISTEST_Z0(e1.y, e1.x, fey1, fex1);			\
													\
	const PxVec3 e2 = v0 - v2;						\
	const float fey2 = PxAbs(e2.y);			\
	const float fez2 = PxAbs(e2.z);			\
	AXISTEST_X2(e2.z, e2.y, fez2, fey2);			\
	const float fex2 = PxAbs(e2.x);			\
	AXISTEST_Y1(e2.z, e2.x, fez2, fex2);			\
	AXISTEST_Z12(e2.y, e2.x, fey2, fex2);

} // namespace Ice

}

#endif
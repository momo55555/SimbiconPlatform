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
#include "IceSupport.h"
#include "PxMath.h"
#include "GuGeomUtilsInternal.h"
#include "GuBox.h"
#include "./Ice/IceUtils.h"

using namespace physx;

bool FIFOStack::Pop(PxU32& entry)
{
	PxU32 NbEntries = GetNbEntries();						// Get current number of entries
	if(!NbEntries)	return false;							// Can be NULL when no value has been pushed. This is an invalid pop call.
	entry = GetEntry(mCurIndex++);							// Get oldest entry, move to next one
	if(mCurIndex==NbEntries)	{ Reset(); mCurIndex=0; }	// All values have been poped
	return true;
}

/*
	PX_FORCE_INLINE void closestAxis2(const float v[3], PxU32& j, PxU32& k)
	{
#ifdef _XBOX
		const float delta = v[0] - v[1];

		float max =  physx::intrinsics::fsel(delta, v[0], v[1]);
		float m =  physx::intrinsics::fsel(delta, 1.0f, 2.0f);

		const float delta2 = max - v[2];
//		max =  physx::intrinsics::fsel(delta2, max, absPz);
//		m =  physx::intrinsics::fsel(delta2, m, 2.0f);
		m =  physx::intrinsics::fsel(delta2, m, 0.0f);

		j = PxU32(m);
		k=j+1;
		if(k>2)
			k=0;

//		j = Ps::getNextIndex3(i);
//		k = Ps::getNextIndex3(j);

//		return i;
#else
		PxU32 m = 0;	//x biggest axis
		j = 1;
		k = 2;
		if( v[1] > v[0] && v[1] > v[2])
		{
			//y biggest
			j = 2;
			k = 0;
			m = 1;
		}
		else if(v[2] > v[0])
		{
			//z biggest
			j = 0;
			k = 1;
			m = 2;
		}
//		return m;
#endif
	}
*/

namespace physx
{
	void CreateOBB(Gu::Box& dest, const Gu::Box& box, const PxVec3& dir, float d)
	{
		PxVec3 R1, R2;
		Gu::computeBasis(dir, R1, R2);

			float dd[3];
			dd[0] = PxAbs(box.rot.column0.dot(dir));
			dd[1] = PxAbs(box.rot.column1.dot(dir));
			dd[2] = PxAbs(box.rot.column2.dot(dir));
			float dmax = dd[0];
			int ax0=1;
			int ax1=2;
			if(dd[1]>dmax)
			{
				dmax=dd[1];
				ax0=0;
				ax1=2;
			}
			if(dd[2]>dmax)
			{
				dmax=dd[2];
				ax0=0;
				ax1=1;
			}
			
			// PxU32 ax0;
			// PxU32 ax1;
			// closestAxis2(dd, ax0, ax1);

			if(dd[ax1]<dd[ax0])	TSwap(ax0, ax1);

			R1 = box.rot[ax0];
			R1 -= (R1.dot(dir))*dir;	// Project to plane whose normal is dir
			R1.normalize();
			R2 = dir.cross(R1);

		dest.rot.column0 = dir;
		dest.rot.column1 = R1;
		dest.rot.column2 = R2;

		float Offset[3];
		Offset[0] = d;//*(dir|dir);
		Offset[1] = d*(dir.dot(R1));
		Offset[2] = d*(dir.dot(R2));

		for(PxU32 r=0;r<3;r++)
		{
			const PxVec3& R = dest.rot[r];
			dest.extents[r] = Offset[r]*0.5f + PxAbs(box.rot[0].dot(R))*box.extents.x + PxAbs(box.rot[1].dot(R))*box.extents.y + PxAbs(box.rot[2].dot(R))*box.extents.z;
		}

		dest.center = box.center + dir*d*0.5f;
	}
}

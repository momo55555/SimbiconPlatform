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
#ifndef CTCRAYAABBOVERLAP_H
#define CTCRAYAABBOVERLAP_H

namespace physx
{
namespace Ice
{

	// Inlining those is really faster in some cases...

	PX_INLINE bool SegmentAABB(const Gu::Segment& segment, const PxVec3& minimum, const PxVec3& maximum)
	{
		PxVec3 BoxExtents, Diff, Dir;
		float fAWdU[3];

		Dir.x = (segment.p1.x - segment.p0.x);
		BoxExtents.x = (maximum.x - minimum.x);
		Diff.x = ((segment.p1.x + segment.p0.x) - (maximum.x + minimum.x));
		fAWdU[0] = PxAbs(Dir.x);
		if(PxAbs(Diff.x)>BoxExtents.x + fAWdU[0])	return false;

		Dir.y = (segment.p1.y - segment.p0.y);
		BoxExtents.y = (maximum.y - minimum.y);
		Diff.y = ((segment.p1.y + segment.p0.y) - (maximum.y + minimum.y));
		fAWdU[1] = PxAbs(Dir.y);
		if(PxAbs(Diff.y)>BoxExtents.y + fAWdU[1])	return false;

		Dir.z = (segment.p1.z - segment.p0.z);
		BoxExtents.z = (maximum.z - minimum.z);
		Diff.z = ((segment.p1.z + segment.p0.z) - (maximum.z + minimum.z));
		fAWdU[2] = PxAbs(Dir.z);
		if(PxAbs(Diff.z)>BoxExtents.z + fAWdU[2])	return false;

		float f;
		f = Dir.y * Diff.z - Dir.z * Diff.y;	if(PxAbs(f)>BoxExtents.y*fAWdU[2] + BoxExtents.z*fAWdU[1])	return false;
		f = Dir.z * Diff.x - Dir.x * Diff.z;	if(PxAbs(f)>BoxExtents.x*fAWdU[2] + BoxExtents.z*fAWdU[0])	return false;
		f = Dir.x * Diff.y - Dir.y * Diff.x;	if(PxAbs(f)>BoxExtents.x*fAWdU[1] + BoxExtents.y*fAWdU[0])	return false;

		return true;
	}

#ifndef OPC_SUPPORT_VMX128

	PX_INLINE bool RayAABB(const PxVec3& orig, const PxVec3& dir, const PxVec3& minimum, const PxVec3& maximum)
	{
		PxVec3 BoxExtents, Diff;

		Diff.x = orig.x - ((maximum.x + minimum.x)*0.5f);
		BoxExtents.x = (maximum.x - minimum.x)*0.5f;
		if(PxAbs(Diff.x)>BoxExtents.x && Diff.x*dir.x>=0.0f)	return false;

		Diff.y = orig.y - ((maximum.y + minimum.y)*0.5f);
		BoxExtents.y = (maximum.y - minimum.y)*0.5f;
		if(PxAbs(Diff.y)>BoxExtents.y && Diff.y*dir.y>=0.0f)	return false;

		Diff.z = orig.z - ((maximum.z + minimum.z)*0.5f);
		BoxExtents.z = (maximum.z - minimum.z)*0.5f;
		if(PxAbs(Diff.z)>BoxExtents.z && Diff.z*dir.z>=0.0f)	return false;

		float fAWdU[3];
		fAWdU[0] = PxAbs(dir.x);
		fAWdU[1] = PxAbs(dir.y);
		fAWdU[2] = PxAbs(dir.z);

		float f;
		f = dir.y * Diff.z - dir.z * Diff.y;	if(PxAbs(f)>BoxExtents.y*fAWdU[2] + BoxExtents.z*fAWdU[1])	return false;
		f = dir.z * Diff.x - dir.x * Diff.z;	if(PxAbs(f)>BoxExtents.x*fAWdU[2] + BoxExtents.z*fAWdU[0])	return false;
		f = dir.x * Diff.y - dir.y * Diff.x;	if(PxAbs(f)>BoxExtents.x*fAWdU[1] + BoxExtents.y*fAWdU[0])	return false;
		return true;
	}

#else

	PX_INLINE bool RayAABB(const PxVec3& orig_, const PxVec3& dir_, const PxVec3& minimum, const PxVec3& maximum)
	{

		Cm::PxSimd::Vector4 half = Cm::PxSimd::half(); //opt: preload
		Cm::PxSimd::Vector4 zero = Cm::PxSimd::zero(); //opt: preload

		Cm::PxSimd::Vector4 orig = Cm::PxSimd::load(orig_); //opt: preload
		Cm::PxSimd::Vector4 dir = Cm::PxSimd::load(dir_); //opt: preload

		Cm::PxSimd::Vector4 bbMin = Cm::PxSimd::load(minimum);
		Cm::PxSimd::Vector4 bbMax = Cm::PxSimd::load(maximum);

		Cm::PxSimd::Vector4 boxCen = Cm::PxSimd::multiply(Cm::PxSimd::add(bbMin, bbMax), half);
		Cm::PxSimd::Vector4 boxExtents = Cm::PxSimd::multiply(Cm::PxSimd::subtract(bbMax, bbMin), half);

		/*
		Diff.x = ray.mOrig.x - ((maximum.x + minimum.x)*0.5f);
		BoxExtents.x = (maximum.x - minimum.x)*0.5f;

		if(fabsf(Diff.x)>BoxExtents.x && Diff.x*ray.mDir.x>=0.0f)	return false;

		Diff.y = ray.mOrig.y - ((maximum.y + minimum.y)*0.5f);
		BoxExtents.y = (maximum.y - minimum.y)*0.5f;

		if(fabsf(Diff.y)>BoxExtents.y && Diff.y*ray.mDir.y>=0.0f)	return false;

		Diff.z = ray.mOrig.z - ((maximum.z + minimum.z)*0.5f);
		BoxExtents.z = (maximum.z - minimum.z)*0.5f;

		if(fabsf(Diff.z)>BoxExtents.z && Diff.z*ray.mDir.z>=0.0f)	return false;
		*/


		Cm::PxSimd::Vector4 diff = Cm::PxSimd::subtract(orig, boxCen);
		Cm::PxSimd::Vector4 absDiff = Cm::PxSimd::abs(diff);

		Cm::PxSimd::Vector4 maskA = Cm::PxSimd::greater(absDiff, boxExtents);
		Cm::PxSimd::Vector4 maskB = Cm::PxSimd::greaterEqual(Cm::PxSimd::multiply(diff, dir), zero);
		
		/*float fAWdU[3];

		fAWdU[0] = fabsf(ray.mDir.x);
		fAWdU[1] = fabsf(ray.mDir.y);
		fAWdU[2] = fabsf(ray.mDir.z);
		*/

		Cm::PxSimd::Vector4 absDir = Cm::PxSimd::abs(dir); //opt precompute/load
		Cm::PxSimd::Vector4 absDirYZZ = Cm::PxSimd::permuteYZZ(absDir); //opt precompute/load
		Cm::PxSimd::Vector4 absDirXYX = Cm::PxSimd::permuteXYX(absDir); //opt precompute/load

		Cm::PxSimd::Vector4 diffYZX = Cm::PxSimd::permuteYZX(diff);
		Cm::PxSimd::Vector4 dirYZX = Cm::PxSimd::permuteYZX(dir); //opt precompute/load

		Cm::PxSimd::Vector4 boxExtentsXYX = Cm::PxSimd::permuteXYX(boxExtents);
		Cm::PxSimd::Vector4 boxExtentsYZZ = Cm::PxSimd::permuteYZZ(boxExtents);

		//opt multiply subtract
		Cm::PxSimd::Vector4 f = Cm::PxSimd::subtract(Cm::PxSimd::multiply(dir, diffYZX), Cm::PxSimd::multiply(dirYZX, diff));
		Cm::PxSimd::Vector4 absF = Cm::PxSimd::abs(f);

		Cm::PxSimd::Vector4 b = Cm::PxSimd::multiplyAdd(boxExtentsXYX, absDirYZZ, Cm::PxSimd::multiply(boxExtentsYZZ, absDirXYX));

		Cm::PxSimd::Vector4 maskC = Cm::PxSimd::greater(absF, b);
		/*
		float f;
		f = ray.mDir.x * Diff.y - ray.mDir.y * Diff.x;	if(fabsf(f)>BoxExtents.x*fAWdU[1] + BoxExtents.y*fAWdU[0])	return false;
		f = ray.mDir.y * Diff.z - ray.mDir.z * Diff.y;	if(fabsf(f)>BoxExtents.y*fAWdU[2] + BoxExtents.z*fAWdU[1])	return false;
		f = ray.mDir.z * Diff.x - ray.mDir.x * Diff.z;	if(fabsf(f)>BoxExtents.x*fAWdU[2] + BoxExtents.z*fAWdU[0])	return false;
		*/

		if(Cm::PxSimd::intNotEqualBool(Cm::PxSimd::or4(maskC, Cm::PxSimd::and4(maskA, maskB)), zero))
			return false;
		else
			return true;
	}
#endif

	PX_INLINE bool SegmentAABB(const Gu::Segment& segment, const PxBounds3& aabb)			{ return Ice::SegmentAABB(segment, aabb.minimum, aabb.maximum);	}
	PX_INLINE bool RayAABB(const PxVec3& orig, const PxVec3& dir, const PxBounds3& aabb)	{ return Ice::RayAABB(orig, dir, aabb.minimum, aabb.maximum);	}
}

}

#endif // CTCRAYAABBOVERLAP_H

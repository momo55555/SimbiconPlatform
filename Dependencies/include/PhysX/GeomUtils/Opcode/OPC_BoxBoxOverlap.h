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
/**
 *	OBB-OBB overlap test using the separating axis theorem.
 *	- original code by Gomez / Gamasutra (similar to Gottschalk's one in RAPID)
 *	- optimized for AABB trees by computing the rotation matrix once (SOLID-fashion)
 *	- the fabs matrix is precomputed as well and epsilon-tweaked (RAPID-style, we found this almost mandatory)
 *	- Class III axes can be disabled... (SOLID & Intel fashion)
 *	- ...or enabled to perform some profiling
 *	- CPU comparisons used when appropriate
 *	- lazy evaluation sometimes saves some work in case of early exits (unlike SOLID)
 *
 *	\param		ea	[in] extents from box A
 *	\param		ca	[in] center from box A
 *	\param		eb	[in] extents from box B
 *	\param		cb	[in] center from box B
 *	\return		true if boxes overlap
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef OPC_SUPPORT_VMX128
//! A dedicated version when one box is constant
PX_FORCE_INLINE Ps::IntBool OBBCollider::BoxBoxOverlap(const PxVec3& extents, const PxVec3& center)
{
	// Stats
	mNbVolumeBVTests++;

	float t,t2;

	// Class I : A's basis vectors
	const float Tx = mTBoxToModel.x - center.x;	t = extents.x + mBBx1;	if(GREATER(Tx, t))	return Ps::IntFalse;
	const float Ty = mTBoxToModel.y - center.y;	t = extents.y + mBBy1;	if(GREATER(Ty, t))	return Ps::IntFalse;
	const float Tz = mTBoxToModel.z - center.z;	t = extents.z + mBBz1;	if(GREATER(Tz, t))	return Ps::IntFalse;

	// Class II : B's basis vectors
	t = Tx*mRBoxToModel[0][0] + Ty*mRBoxToModel[0][1] + Tz*mRBoxToModel[0][2];
	t2 = extents.x*mAR[0][0] + extents.y*mAR[0][1] + extents.z*mAR[0][2] + mBoxExtents.x;
	if(GREATER(t, t2))	return Ps::IntFalse;

	t = Tx*mRBoxToModel[1][0] + Ty*mRBoxToModel[1][1] + Tz*mRBoxToModel[1][2];
	t2 = extents.x*mAR[1][0] + extents.y*mAR[1][1] + extents.z*mAR[1][2] + mBoxExtents.y;
	if(GREATER(t, t2))	return Ps::IntFalse;

	t = Tx*mRBoxToModel[2][0] + Ty*mRBoxToModel[2][1] + Tz*mRBoxToModel[2][2];
	t2 = extents.x*mAR[2][0] + extents.y*mAR[2][1] + extents.z*mAR[2][2] + mBoxExtents.z;
	if(GREATER(t, t2))	return Ps::IntFalse;

	// Class III : 9 cross products
	// Cool trick: always perform the full test for first level, regardless of settings.
	// That way pathological cases (such as the pencils scene) are quickly rejected anyway !
	if(mFullBoxBoxTest || mNbVolumeBVTests==1)
	{
		t = Tz*mRBoxToModel[0][1] - Ty*mRBoxToModel[0][2];	t2 = extents.y*mAR[0][2] + extents.z*mAR[0][1] + mBB_1;	if(GREATER(t, t2))	return Ps::IntFalse;	// L = A0 x B0
		t = Tz*mRBoxToModel[1][1] - Ty*mRBoxToModel[1][2];	t2 = extents.y*mAR[1][2] + extents.z*mAR[1][1] + mBB_2;	if(GREATER(t, t2))	return Ps::IntFalse;	// L = A0 x B1
		t = Tz*mRBoxToModel[2][1] - Ty*mRBoxToModel[2][2];	t2 = extents.y*mAR[2][2] + extents.z*mAR[2][1] + mBB_3;	if(GREATER(t, t2))	return Ps::IntFalse;	// L = A0 x B2
		t = Tx*mRBoxToModel[0][2] - Tz*mRBoxToModel[0][0];	t2 = extents.x*mAR[0][2] + extents.z*mAR[0][0] + mBB_4;	if(GREATER(t, t2))	return Ps::IntFalse;	// L = A1 x B0
		t = Tx*mRBoxToModel[1][2] - Tz*mRBoxToModel[1][0];	t2 = extents.x*mAR[1][2] + extents.z*mAR[1][0] + mBB_5;	if(GREATER(t, t2))	return Ps::IntFalse;	// L = A1 x B1
		t = Tx*mRBoxToModel[2][2] - Tz*mRBoxToModel[2][0];	t2 = extents.x*mAR[2][2] + extents.z*mAR[2][0] + mBB_6;	if(GREATER(t, t2))	return Ps::IntFalse;	// L = A1 x B2
		t = Ty*mRBoxToModel[0][0] - Tx*mRBoxToModel[0][1];	t2 = extents.x*mAR[0][1] + extents.y*mAR[0][0] + mBB_7;	if(GREATER(t, t2))	return Ps::IntFalse;	// L = A2 x B0
		t = Ty*mRBoxToModel[1][0] - Tx*mRBoxToModel[1][1];	t2 = extents.x*mAR[1][1] + extents.y*mAR[1][0] + mBB_8;	if(GREATER(t, t2))	return Ps::IntFalse;	// L = A2 x B1
		t = Ty*mRBoxToModel[2][0] - Tx*mRBoxToModel[2][1];	t2 = extents.x*mAR[2][1] + extents.y*mAR[2][0] + mBB_9;	if(GREATER(t, t2))	return Ps::IntFalse;	// L = A2 x B2
	}
	return Ps::IntTrue;
}
#else

PX_FORCE_INLINE Ps::IntBool OBBCollider::BoxBoxOverlap(const PxVec3& extents, const PxVec3& center)
{
	Cm::PxSimd::Vector4 extents4 = Cm::PxSimd::load(extents);
	Cm::PxSimd::Vector4 center4 = Cm::PxSimd::load(center);

	return BoxBoxOverlap(extents4, center4);
}

PX_FORCE_INLINE Ps::IntBool OBBCollider::BoxBoxOverlap(const Cm::PxSimd::Vector4& extents4, const Cm::PxSimd::Vector4& center4)
{
	Cm::PxSimd::Vector4 TBoxToModel = Cm::PxSimd::load(mTBoxToModel);
	Cm::PxSimd::Vector4 BB = mBBxyz1;

	Cm::PxSimd::Vector4 rBoxToModel_0 = Cm::PxSimd::load(mRBoxToModel[0]);
	Cm::PxSimd::Vector4 rBoxToModel_1 = Cm::PxSimd::load(mRBoxToModel[1]);
	Cm::PxSimd::Vector4 rBoxToModel_2 = Cm::PxSimd::load(mRBoxToModel[2]);

	Cm::PxSimd::Vector4 ar_0 = Cm::PxSimd::load(mAR[0]);//would be nice just to use abs, but ar has epsilon added
	Cm::PxSimd::Vector4 ar_1 = Cm::PxSimd::load(mAR[1]);//being able to aligned load would be much better.
	Cm::PxSimd::Vector4 ar_2 = Cm::PxSimd::load(mAR[2]);

	Cm::PxSimd::Vector4 thisExtents = Cm::PxSimd::load(mBoxExtents);

	Cm::PxSimd::Vector4 BB_123 = mBB_123;
	Cm::PxSimd::Vector4 BB_456 = mBB_456;
	Cm::PxSimd::Vector4 BB_789 = mBB_789;

	return BoxBoxOverlap(extents4, center4,
		TBoxToModel, BB,
		rBoxToModel_0, rBoxToModel_1, rBoxToModel_2,
		ar_0, ar_1, ar_2,
		thisExtents,
		BB_123, BB_456, BB_789);
}


//! A dedicated version when one box is constant
// preload constants into registers(passed as params)
PX_FORCE_INLINE Ps::IntBool OBBCollider::BoxBoxOverlap(const Cm::PxSimd::Vector4& extents, const Cm::PxSimd::Vector4& center,
		const Cm::PxSimd::Vector4 &TBoxToModel, const Cm::PxSimd::Vector4 &BB,
		const Cm::PxSimd::Vector4 &rBoxToModel_0, const Cm::PxSimd::Vector4 &rBoxToModel_1, const Cm::PxSimd::Vector4 &rBoxToModel_2,
		const Cm::PxSimd::Vector4 &ar_0, const Cm::PxSimd::Vector4 &ar_1, const Cm::PxSimd::Vector4 &ar_2,
		const Cm::PxSimd::Vector4 &thisExtents,
		const Cm::PxSimd::Vector4 &BB_123,	const Cm::PxSimd::Vector4 &BB_456, const Cm::PxSimd::Vector4 &BB_789
		)
{
	// Stats
	mNbVolumeBVTests++;

	/*
		// Class I : A's basis vectors
	float Tx = mTBoxToModel.x - center.x;	t = extents.x + mBBx1;	if(GREATER(Tx, t))	return Ps::IntFalse;
	float Ty = mTBoxToModel.y - center.y;	t = extents.y + mBBy1;	if(GREATER(Ty, t))	return Ps::IntFalse;
	float Tz = mTBoxToModel.z - center.z;	t = extents.z + mBBz1;	if(GREATER(Tz, t))	return Ps::IntFalse;
*/
	Cm::PxSimd::Vector4 T = Cm::PxSimd::subtract(TBoxToModel, center);
	Cm::PxSimd::Vector4 t = Cm::PxSimd::add(extents, BB);

	if(!Cm::PxSimd::inBounds3Bool(T, t))
		return Ps::IntFalse;

/*
	// Class II : B's basis vectors
	t = Tx*mRBoxToModel.m[0][0] + Ty*mRBoxToModel.m[0][1] + Tz*mRBoxToModel.m[0][2];
	t2 = extents.x*mAR.m[0][0] + extents.y*mAR.m[0][1] + extents.z*mAR.m[0][2] + mBoxExtents.x;
	if(GREATER(t, t2))	return Ps::IntFalse;

	t = Tx*mRBoxToModel.m[1][0] + Ty*mRBoxToModel.m[1][1] + Tz*mRBoxToModel.m[1][2];
	t2 = extents.x*mAR.m[1][0] + extents.y*mAR.m[1][1] + extents.z*mAR.m[1][2] + mBoxExtents.y;
	if(GREATER(t, t2))	return Ps::IntFalse;

	t = Tx*mRBoxToModel.m[2][0] + Ty*mRBoxToModel.m[2][1] + Tz*mRBoxToModel.m[2][2];
	t2 = extents.x*mAR.m[2][0] + extents.y*mAR.m[2][1] + extents.z*mAR.m[2][2] + mBoxExtents.z;
	if(GREATER(t, t2))	return Ps::IntFalse;
	*/

	//OPT: thisExtents is constant and could be factored out... (but would increase register pressure)

	Cm::PxSimd::Vector4 t2;

	Cm::PxSimd::Vector4 t_0 = Cm::PxSimd::dot(T, rBoxToModel_0);
	Cm::PxSimd::Vector4 t_1 = Cm::PxSimd::dot(T, rBoxToModel_1);
	Cm::PxSimd::Vector4 t_2 = Cm::PxSimd::dot(T, rBoxToModel_2);
	Cm::PxSimd::Vector4 t2_0 = Cm::PxSimd::dot(extents, ar_0);
	Cm::PxSimd::Vector4 t2_1 = Cm::PxSimd::dot(extents, ar_1);
	Cm::PxSimd::Vector4 t2_2 = Cm::PxSimd::dot(extents, ar_2);

	t = Cm::PxSimd::mergeXYZ(t_0, t_1, t_2);
	t2 = Cm::PxSimd::mergeXYZ(t2_0, t2_1, t2_2);

	t2 = Cm::PxSimd::add(t2, thisExtents);

	if(!Cm::PxSimd::inBounds3Bool(t, t2))
		return Ps::IntFalse;

	// Class III : 9 cross products
	// Cool trick: always perform the full test for first level, regardless of settings.
	// That way pathological cases (such as the pencils scene) are quickly rejected anyway !
	if(mFullBoxBoxTest || mNbVolumeBVTests==1)
	{
		//OPT: lots of constants could be factored out of the loop, but again increases register pressure.

		Cm::PxSimd::Vector4 extentsX = Cm::PxSimd::splatX(extents);
		Cm::PxSimd::Vector4 extentsY = Cm::PxSimd::splatY(extents);
		Cm::PxSimd::Vector4 extentsZ = Cm::PxSimd::splatZ(extents);
		
		Cm::PxSimd::Vector4 rBoxToModel_colX, rBoxToModel_colY, rBoxToModel_colZ;
		Cm::PxSimd::Vector4 ar_colX, ar_colY, ar_colZ;

		Cm::PxSimd::transpose3x3(rBoxToModel_0, rBoxToModel_1, rBoxToModel_2,
			rBoxToModel_colX, rBoxToModel_colY, rBoxToModel_colZ);

		Cm::PxSimd::transpose3x3(ar_0, ar_1, ar_2,
			ar_colX, ar_colY, ar_colZ);

		Cm::PxSimd::Vector4 Tx = Cm::PxSimd::splatX(T);
		Cm::PxSimd::Vector4 Ty = Cm::PxSimd::splatY(T);
		Cm::PxSimd::Vector4 Tz = Cm::PxSimd::splatZ(T);

		/*
		t = Tz*mRBoxToModel.m[0][1] - Ty*mRBoxToModel.m[0][2];	t2 = extents.y*mAR.m[0][2] + extents.z*mAR.m[0][1] + mBB_1;	if(GREATER(t, t2))	return Ps::IntFalse;	// L = A0 x B0
		t = Tz*mRBoxToModel.m[1][1] - Ty*mRBoxToModel.m[1][2];	t2 = extents.y*mAR.m[1][2] + extents.z*mAR.m[1][1] + mBB_2;	if(GREATER(t, t2))	return Ps::IntFalse;	// L = A0 x B1
		t = Tz*mRBoxToModel.m[2][1] - Ty*mRBoxToModel.m[2][2];	t2 = extents.y*mAR.m[2][2] + extents.z*mAR.m[2][1] + mBB_3;	if(GREATER(t, t2))	return Ps::IntFalse;	// L = A0 x B2*/

		t = Cm::PxSimd::multiply(Tz, rBoxToModel_colY);
		t = Cm::PxSimd::negMultiplySubtract(Ty, rBoxToModel_colZ, t);
		
		t2 = Cm::PxSimd::multiplyAdd(extentsY, ar_colZ, BB_123);
		t2 = Cm::PxSimd::multiplyAdd(extentsZ, ar_colY, t2);

		if(!Cm::PxSimd::inBounds3Bool(t, t2))
			return Ps::IntFalse;

		/*
		t = Tx*mRBoxToModel.m[0][2] - Tz*mRBoxToModel.m[0][0];	t2 = extents.x*mAR.m[0][2] + extents.z*mAR.m[0][0] + mBB_4;	if(GREATER(t, t2))	return Ps::IntFalse;	// L = A1 x B0
		t = Tx*mRBoxToModel.m[1][2] - Tz*mRBoxToModel.m[1][0];	t2 = extents.x*mAR.m[1][2] + extents.z*mAR.m[1][0] + mBB_5;	if(GREATER(t, t2))	return Ps::IntFalse;	// L = A1 x B1
		t = Tx*mRBoxToModel.m[2][2] - Tz*mRBoxToModel.m[2][0];	t2 = extents.x*mAR.m[2][2] + extents.z*mAR.m[2][0] + mBB_6;	if(GREATER(t, t2))	return Ps::IntFalse;	// L = A1 x B2*/

		t = Cm::PxSimd::multiply(Tx, rBoxToModel_colZ);
		t = Cm::PxSimd::negMultiplySubtract(Tz, rBoxToModel_colX, t);
		
		t2 = Cm::PxSimd::multiplyAdd(extentsX, ar_colZ, BB_456);
		t2 = Cm::PxSimd::multiplyAdd(extentsZ, ar_colX, t2);

		if(!Cm::PxSimd::inBounds3Bool(t, t2))
			return Ps::IntFalse;

		/*
		t = Ty*mRBoxToModel.m[0][0] - Tx*mRBoxToModel.m[0][1];	t2 = extents.x*mAR.m[0][1] + extents.y*mAR.m[0][0] + mBB_7;	if(GREATER(t, t2))	return Ps::IntFalse;	// L = A2 x B0
		t = Ty*mRBoxToModel.m[1][0] - Tx*mRBoxToModel.m[1][1];	t2 = extents.x*mAR.m[1][1] + extents.y*mAR.m[1][0] + mBB_8;	if(GREATER(t, t2))	return Ps::IntFalse;	// L = A2 x B1
		t = Ty*mRBoxToModel.m[2][0] - Tx*mRBoxToModel.m[2][1];	t2 = extents.x*mAR.m[2][1] + extents.y*mAR.m[2][0] + mBB_9;	if(GREATER(t, t2))	return Ps::IntFalse;	// L = A2 x B2*/
		
		t = Cm::PxSimd::multiply(Ty, rBoxToModel_colX);
		t = Cm::PxSimd::negMultiplySubtract(Tx, rBoxToModel_colY, t);
		
		t2 = Cm::PxSimd::multiplyAdd(extentsX, ar_colY, BB_789);
		t2 = Cm::PxSimd::multiplyAdd(extentsY, ar_colX, t2);

		if(!Cm::PxSimd::inBounds3Bool(t, t2))
			return Ps::IntFalse;
	}
	

	return Ps::IntTrue;
}
#endif //OPC_SUPPORT_VMX128


//! A special version for 2 axis-aligned boxes
PX_FORCE_INLINE Ps::IntBool AABBCollider::AABBAABBOverlap(const PxVec3& extents, const PxVec3& center)
{
	// Stats
	mNbVolumeBVTests++;
	const float tx = mBox.mCenter.x - center.x;	const float ex = extents.x + mBox.mExtents.x;	if(GREATER(tx, ex))	return Ps::IntFalse;
	const float ty = mBox.mCenter.y - center.y;	const float ey = extents.y + mBox.mExtents.y;	if(GREATER(ty, ey))	return Ps::IntFalse;
	const float tz = mBox.mCenter.z - center.z;	const float ez = extents.z + mBox.mExtents.z;	if(GREATER(tz, ez))	return Ps::IntFalse;

	return Ps::IntTrue;
}

PX_FORCE_INLINE Ps::IntBool AABBCollider::AABBAABBOverlapMinMax(const PxVec3& bbMin, const PxVec3& bbMax)
{
	//Could probably do this faster.
	if((bbMax.x < mMin.x) || (bbMax.y < mMin.y) || (bbMax.z < mMin.z)) return Ps::IntFalse;
	if((bbMin.x > mMax.x) || (bbMin.y > mMax.y) || (bbMin.z > mMax.z)) return Ps::IntFalse;

	return Ps::IntTrue;
}

#ifdef OPC_SUPPORT_VMX128

PX_FORCE_INLINE Ps::IntBool AABBCollider::AABBAABBOverlap(const Cm::PxSimd::Vector4 &extents, const Cm::PxSimd::Vector4 &center)
{
	Cm::PxSimd::Vector4 t = Cm::PxSimd::load(mBox.mCenter) - center;
	Cm::PxSimd::Vector4 e = extents + Cm::PxSimd::load(mBox.mExtents);

	return Cm::PxSimd::inBounds3Bool(t, e);
}

PX_FORCE_INLINE Ps::IntBool AABBCollider::AABBAABBOverlapMinMax(const Cm::PxSimd::Vector4 &bbMin, const Cm::PxSimd::Vector4 &bbMax)
{
	Cm::PxSimd::Vector4 thisMin = Cm::PxSimd::load(mMin);
	Cm::PxSimd::Vector4 thisMax = Cm::PxSimd::load(mMax);

	Cm::PxSimd::Vector4 mask  = Cm::PxSimd::or4(Cm::PxSimd::less(bbMax, thisMin), Cm::PxSimd::greater(bbMin, thisMax));
	return !Cm::PxSimd::anyTrue(mask);
}

#endif

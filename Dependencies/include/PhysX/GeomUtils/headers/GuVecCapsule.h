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


#ifndef PX_PHYSICS_GEOMUTILS_VEC_CAPSULE_H
#define PX_PHYSICS_GEOMUTILS_VEC_CAPSULE_H

/** \addtogroup geomutils
@{
*/

#include "GuVecConvex.h"          

namespace physx
{
namespace Gu
{
	class CapsuleV : public ConvexV
	{
	public:
		/**
		\brief Constructor
		*/

		PX_INLINE CapsuleV():ConvexV(E_CAPSULE)
		{
#ifdef __SPU__
			bMarginIsRadius = true;
#endif
		}

		PX_INLINE CapsuleV(const Ps::aos::Vec3VArg _p0, const Ps::aos::Vec3VArg _p1, const Ps::aos::FloatVArg _radius) : ConvexV(E_CAPSULE)
		{
			using namespace Ps::aos;
			const FloatV half = FloatV_From_F32(0.5f);
			center =V3Scale(V3Add(_p0, _p1), half);
			radius = _radius;
			p0 = _p0;
			p1 = _p1;
			margin = _radius;
			//margin = PxF32_From_FloatV(_radius);
			//const PxF32 _eps = margin * 0.1f;
			//eps =_eps * _eps;
#ifdef __SPU__
			bMarginIsRadius = true;
#endif
		}

		PX_INLINE CapsuleV(const Ps::aos::Vec3VArg _p0, const Ps::aos::Vec3VArg _p1, const Ps::aos::FloatVArg _radius, const PxF32 _margin) : ConvexV(E_CAPSULE)
		{
			using namespace Ps::aos;
			const FloatV half = FloatV_From_F32(0.5f);
			center =V3Scale(V3Add(_p0, _p1), half);
			radius = _radius;
			p0 = _p0;
			p1 = _p1;
			//margin = PxF32_From_FloatV(_radius);
			margin = _radius;
			//const PxF32 _eps = margin * 0.1f;
			//eps = _eps *_eps;
#ifdef __SPU__
			bMarginIsRadius = true;
#endif
		}

		/**
		\brief Constructor

		\param _radius Radius of the capsule.
		*/

		/**
		\brief Destructor
		*/
		PX_INLINE ~CapsuleV()
		{
		}

		PX_INLINE Ps::aos::Vec3V computeDirection() const
		{
			return Ps::aos::V3Sub(p1, p0);
		}

		
		PX_FORCE_INLINE	Ps::aos::FloatV	getRadius()	const
		{
			return radius;
		}


		PX_FORCE_INLINE Ps::aos::Vec3V support(const Ps::aos::Vec3VArg dir)const
		{
			using namespace Ps::aos;
			const Vec3V _dir = V3Normalize(dir);
			const FloatV dist0 = V3Dot(p0, dir);
			const FloatV dist1 = V3Dot(p1, dir);
			const Vec3V p = V3Sel(FIsGrtr(dist0, dist1), p0, p1);
			return V3MulAdd(_dir, radius, p);
		}

		PX_FORCE_INLINE Ps::aos::Vec3V supportMargin(const Ps::aos::Vec3VArg dir, const Ps::aos::FloatVArg _margin)const 
		{
			using namespace Ps::aos;
			/*
			const Vec3V _dir = Ps::aos::V3Normalise(dir);
			const FloatV dist0 = V3Dot(p0, dir);
			const FloatV dist1 = V3Dot(p1, dir);
			const Vec3V p = V3Sel(FIsGrtr(dist0, dist1), p0, p1);
			return V3MulAdd(_dir, V3Sub(radius, _margin), p);*/

			const FloatV dist0 = V3Dot(p0, dir);
			const FloatV dist1 = V3Dot(p1, dir);
			return V3Sel(FIsGrtr(dist0, dist1), p0, p1);
		}

		PX_FORCE_INLINE Ps::aos::Vec3V supportMargin(const Ps::aos::Vec3VArg dir, const Ps::aos::FloatVArg _margin, Ps::aos::Vec3V& support)const 
		{
			using namespace Ps::aos;
			/*
			const Vec3V _dir = Ps::aos::V3Normalise(dir);
			const FloatV dist0 = V3Dot(p0, dir);
			const FloatV dist1 = V3Dot(p1, dir);
			const Vec3V p = V3Sel(FIsGrtr(dist0, dist1), p0, p1);
			return V3MulAdd(_dir, V3Sub(radius, _margin), p);*/

			const FloatV dist0 = V3Dot(p0, dir);
			const FloatV dist1 = V3Dot(p1, dir);
			const Vec3V ret = V3Sel(FIsGrtr(dist0, dist1), p0, p1);
			support = ret;
			return ret;
		}

		PX_FORCE_INLINE bool isMarginEqRadius()const
		{
			return true;
		}

#ifndef __SPU__
		PX_FORCE_INLINE Ps::aos::BoolV isMarginEqRadiusV()const
		{
			return Ps::aos::BTTTT();
		}
#endif

		
		Ps::aos::Vec3V	p0;		//!< Start of segment
		Ps::aos::Vec3V	p1;		//!< End of segment
		Ps::aos::FloatV	radius;
	};
}

}

#endif
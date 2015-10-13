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


#ifndef PX_PHYSICS_GEOMUTILS_SEGMENT
#define PX_PHYSICS_GEOMUTILS_SEGMENT
/** \addtogroup geomutils
@{
*/

#include "PxVec3.h"
#include "Ps.h"
#include "CmPhysXCommon.h"
#include "PsVecMath.h"

namespace physx
{
namespace Gu
{

	/**
	\brief Represents a line segment.

	Line segment geometry
	In some cases this structure will be used to represent the infinite line that passes point0 and point1.
	*/
	class Segment
	{
	public:
		/**
		\brief Constructor
		*/
		PX_INLINE Segment()
		{
		}

		/**
		\brief Constructor
		*/
		PX_INLINE Segment(const PxVec3& _p0, const PxVec3& _p1) : p0(_p0), p1(_p1)
		{
		}

		/**
		\brief Copy constructor
		*/
		PX_INLINE Segment(const Segment& seg) : p0(seg.p0), p1(seg.p1)
		{
		}

		/**
		\brief Destructor
		*/
		PX_INLINE ~Segment()
		{
		}

		//! Assignment operator
		PX_INLINE Segment& operator=(const Segment& other)
		{
			p0 = other.p0; 
			p1 = other.p1; 
			return *this;
		}

		//! Equality operator
		PX_INLINE bool operator==(const Segment& other) const
		{
			return (p0==other.p0 && p1==other.p1);
		}

		//! Inequality operator
		PX_INLINE bool operator!=(const Segment& other) const
		{
			return (p0!=other.p0 || p1!=other.p1);
		}

		PX_INLINE const PxVec3& getOrigin() const
		{
			return p0;
		}

		//! Return the vector from point0 to point1
		PX_INLINE PxVec3 computeDirection() const
		{
			return p1 - p0;
		}

		//! Return the vector from point0 to point1
		PX_INLINE void computeDirection(PxVec3& dir) const
		{
			dir = p1 - p0;
		}

		//! Return the center of the segment segment
		PX_INLINE PxVec3 computeCenter() const
		{
			return (p0 + p1)*0.5f;
		}

		PX_INLINE PxF32 computeLength() const
		{
			return (p1-p0).magnitude();
		}

		PX_INLINE PxF32 computeSquareLength() const
		{
			return (p1-p0).magnitudeSquared();
		}

		// PT: TODO: remove this one
		//! Return the square of the length of vector from point0 to point1
		PX_INLINE PxReal lengthSquared() const
		{
			return ((p1 - p0).magnitudeSquared());
		}

		// PT: TODO: remove this one
		//! Return the length of vector from point0 to point1
		PX_INLINE PxReal length() const
		{
			return ((p1 - p0).magnitude());
		}

		/*		PX_INLINE void setOriginDirection(const PxVec3& origin, const PxVec3& direction)
		{
		p0 = p1 = origin;
		p1 += direction;
		}*/

		/**
		\brief Computes a point on the segment

		\param[out] pt point on segment
		\param[in] t point's parameter [t=0 => pt = mP0, t=1 => pt = mP1]
		*/
		PX_INLINE void computePoint(PxVec3& pt, PxF32 t) const
		{
			pt = p0 + t * (p1 - p0);
		}

		// PT: TODO: remove this one
		//! Return the point at parameter t along the line: point0 + t*(point1-point0)
		PX_INLINE PxVec3 getPointAt(PxReal t) const
		{
			return (p1 - p0)*t + p0;
		}

		PxVec3	p0;		//!< Start of segment
		PxVec3	p1;		//!< End of segment
	};
	PX_COMPILE_TIME_ASSERT(sizeof(Gu::Segment) == 24);


	class SegmentV
	{
	public:
		/**
		\brief Constructor
		*/
		PX_INLINE SegmentV()
		{
		}

		/**
		\brief Constructor
		*/
		PX_INLINE SegmentV(const Ps::aos::Vec3VArg _p0, const Ps::aos::Vec3VArg _p1) : p0(_p0), p1(_p1)
		{
		}

		PX_INLINE SegmentV(const PxVec3& _p0, const PxVec3& _p1)
		{
			p0 = Ps::aos::Vec3V_From_PxVec3(_p0);
			p1 = Ps::aos::Vec3V_From_PxVec3(_p1);
		}

		/**
		\brief Copy constructor
		*/
		PX_INLINE SegmentV(const SegmentV& seg) : p0(seg.p0), p1(seg.p1)
		{
		}

		PX_INLINE SegmentV(const Segment& seg)
		{
			p0 = Ps::aos::Vec3V_From_PxVec3(seg.p0);
			p1 = Ps::aos::Vec3V_From_PxVec3(seg.p1);
		}

		/**
		\brief Destructor
		*/
		PX_INLINE ~SegmentV()
		{
		}

		//! Assignment operator
		PX_INLINE SegmentV& operator=(const SegmentV& other)
		{
			p0 = other.p0; 
			p1 = other.p1; 
			return *this;
		}

		//! Equality operator
		PX_INLINE Ps::aos::BoolV operator==(const SegmentV& other) const
		{
			using namespace Ps::aos;
			return BAnd(V3IsEq(p0, other.p0), V3IsEq(p1, other.p1));
		}

		//! Inequality operator
		PX_INLINE Ps::aos::BoolV operator!=(const SegmentV& other) const
		{
			using namespace Ps::aos;
			return BOr(BNot(V3IsEq(p0, other.p0)), BNot(V3IsEq(p1, other.p1)));
		}

		PX_INLINE const Ps::aos::Vec3V& getOrigin() const
		{
			return p0;
		}

		//! Return the vector from point0 to point1
		PX_INLINE Ps::aos::Vec3V computeDirection() const
		{
			return Ps::aos::V3Sub(p1, p0);
		}

		//! Return the center of the segment segment
		PX_INLINE Ps::aos::Vec3V computeCenter() const
		{
			using namespace Ps::aos;
			const FloatV half = FloatV_From_F32(0.5f);
			return V3Mul(V3Add(p0, p1), half);
		}

		PX_INLINE Ps::aos::FloatV computeLength() const
		{
			return Ps::aos::V3Length(Ps::aos::V3Sub(p1, p0));
		}

		PX_INLINE Ps::aos::FloatV computeSquareLength() const
		{
			
			return Ps::aos::V3LengthSq(Ps::aos::V3Sub(p1, p0));
		}

		/**
		\brief Computes a point on the segment

		\param[out] pt point on segment
		\param[in] t point's parameter [t=0 => pt = mP0, t=1 => pt = mP1]
		*/
		PX_INLINE Ps::aos::Vec3V computePoint(Ps::aos::FloatVArg t) const
		{
			return Ps::aos::V3MulAdd(Ps::aos::V3Sub(p1, p0), t, p0);
		}

		Ps::aos::Vec3V support(const Ps::aos::Vec3VArg dir) const
		{
			using namespace Ps::aos;
			const FloatV dist0 = V3Dot(p0, dir);
			const FloatV dist1 = V3Dot(p1, dir);
			return V3Sel(FIsGrtr(dist0, dist1), p0, p1);
		}

		Ps::aos::Vec3V supportMargin(const Ps::aos::Vec3VArg dir, const Ps::aos::FloatVArg margin)const
		{
			using namespace Ps::aos;
			const FloatV dist0 = V3Dot(p0, dir);
			const FloatV dist1 = V3Dot(p1, dir);
			return V3Sel(FIsGrtr(dist0, dist1), p0, p1);
		}

		Ps::aos::Vec3V	p0;		//!< Start of segment
		Ps::aos::Vec3V	p1;		//!< End of segment
		Ps::aos::FloatV margin;
	};
	//PX_COMPILE_TIME_ASSERT(sizeof(Gu::SegmentV) == 48);
}

}

/** @} */
#endif

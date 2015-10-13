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
#ifndef OPC_COMMON_H
#define OPC_COMMON_H

	#define GREATER(x, y)	PxAbs(x) > (y)

#include "GuBox.h"
#include "PxBounds3.h"
#include "Opcode.h"

namespace physx
{
namespace Ice
{

	class CollisionAABB
	{
		public:
		//! Constructor
		PX_FORCE_INLINE				CollisionAABB()						{}
		//! Constructor
		PX_FORCE_INLINE				CollisionAABB(const PxBounds3& b)
								{
									mCenter = b.getCenter();
									mExtents = b.getExtents();
								}
		//! Destructor
		PX_FORCE_INLINE				~CollisionAABB()					{}

		//! Get minimum point of the box
		PX_FORCE_INLINE	void		GetMin(PxVec3& minimum)		const		{ minimum = mCenter - mExtents;					}
		//! Get maximum point of the box
		PX_FORCE_INLINE	void		GetMax(PxVec3& maximum)		const		{ maximum = mCenter + mExtents;					}

		//! Get component of the box's minimum point along a given axis
		PX_FORCE_INLINE	float		GetMin(PxU32 axis)		const		{ return mCenter[axis] - mExtents[axis];	}
		//! Get component of the box's maximum point along a given axis
		PX_FORCE_INLINE	float		GetMax(PxU32 axis)		const		{ return mCenter[axis] + mExtents[axis];	}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Setups an AABB from minimum & maximum vectors.
		 *	\param		minimum			[in] the minimum point
		 *	\param		maximum			[in] the maximum point
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		PX_FORCE_INLINE	void		SetMinMax(const PxVec3& minimum, const PxVec3& maximum)		{ mCenter = (maximum + minimum)*0.5f; mExtents = (maximum - minimum)*0.5f;		}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Checks a box is inside another box.
		 *	\param		box		[in] the other box
		 *	\return		true if current box is inside input box
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		PX_FORCE_INLINE	Ps::IntBool	IsInside(const CollisionAABB& box) const
								{
									if(box.GetMin(0)>GetMin(0))	return Ps::IntFalse;
									if(box.GetMin(1)>GetMin(1))	return Ps::IntFalse;
									if(box.GetMin(2)>GetMin(2))	return Ps::IntFalse;
									if(box.GetMax(0)<GetMax(0))	return Ps::IntFalse;
									if(box.GetMax(1)<GetMax(1))	return Ps::IntFalse;
									if(box.GetMax(2)<GetMax(2))	return Ps::IntFalse;
									return Ps::IntTrue;
								}

					PxVec3		mCenter;				//!< Box center
					PxVec3		mExtents;				//!< Box extents
	};

	class QuantizedAABB
	{
		public:
		//! Constructor
		PX_FORCE_INLINE				QuantizedAABB()			{}
		//! Destructor
		PX_FORCE_INLINE				~QuantizedAABB()		{}

					PxI16		mCenter[3];				//!< Quantized center
					PxU16		mExtents[3];			//!< Quantized extents
	};

	//! Quickly rotates & translates a vector
	PX_FORCE_INLINE void TransformPoint(PxVec3& dest, const PxVec3& source, const PxMat33& rot, const PxVec3& trans)
	{
		dest = trans + rot.transform(source);
	}

} // namespace Ice

}

#endif // OPC_COMMON_H

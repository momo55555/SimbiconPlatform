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

#ifndef PX_VEHICLE_SUSP_LIMIT_CONSTRAINT_SHADER_H
#define PX_VEHICLE_SUSP_LIMIT_CONSTRAINT_SHADER_H
/** \addtogroup vehicle
  @{
*/

#include "extensions/PxConstraintExt.h"
#include "PxConstraintDesc.h"
#include "PxTransform.h"

#ifndef PX_DOXYGEN
namespace physx
{
#endif

template <PxU32 NUM_ELEMENTS> class PxVehicleConstraintShader : public PxConstraintConnector
{
public:

	virtual void			onComShift(PxU32 actor)	{PX_UNUSED(actor);}

	virtual void*			prepareData()	
	{
		return &mData;
	}

	virtual bool			updatePvdProperties(PVD::PvdDataStream&,
							const PxConstraint*,
							PxPvdUpdateType::Enum) const	 
	{
		return true;
	}

	virtual void			onConstraintRelease()	{}
	virtual void*			getExternalReference(PxU32& typeID) { typeID = PxConstraintExtIDs::eVEHICLE_SUSP_LIMIT; return this; }

	static PxU32 vehicleSuspLimitConstraintSolverPrep(
		Px1DConstraint* constraints,
		PxVec3& body0WorldOffset,
		PxU32 maxConstraints,
		const void* constantBlock,
		const PxTransform& bodyAToWorld,
		const PxTransform& bodyBToWorld
		)
	{
		PX_ASSERT(bodyAToWorld.isValid()); PX_ASSERT(bodyBToWorld.isValid());

		VehicleConstraintData* data = (VehicleConstraintData*)constantBlock;
		PxU32 numActive=0;

		//Susp limit constraints.
		for(PxU32 i=0;i<NUM_ELEMENTS;i++)
		{
			if(data->mSuspLimitData.mActiveFlags[i])
			{
				Px1DConstraint& p=constraints[numActive];
				p.linear0=bodyAToWorld.q.rotate(data->mSuspLimitData.mDirs[i]);
				p.angular0=bodyAToWorld.q.rotate(data->mSuspLimitData.mCMOffsets[i].cross(data->mSuspLimitData.mDirs[i]));
				p.geometricError=data->mSuspLimitData.mErrors[i];
				p.linear1=PxVec3(0);
				p.angular1=PxVec3(0);
				p.minImpulse=-FLT_MAX;
				p.maxImpulse=0;
				p.velocityTarget=0;		
				numActive++;
			}
		}

		//Sticky tyre friction constraints.
		for(PxU32 i=0;i<NUM_ELEMENTS;i++)
		{
			if(data->mStickyTyreData.mActiveFlags[i])
			{
				Px1DConstraint& p=constraints[numActive];
				p.linear0=data->mStickyTyreData.mDirs[i];
				p.angular0=data->mStickyTyreData.mCMOffsets[i].cross(data->mStickyTyreData.mDirs[i]);
				p.geometricError=data->mStickyTyreData.mErrors[i];
				p.linear1=PxVec3(0);
				p.angular1=PxVec3(0);
				p.minImpulse=-FLT_MAX;
				p.maxImpulse=FLT_MAX;
				p.velocityTarget=0;		
				numActive++;
			}
		}

		return numActive;
	}

	static void visualiseConstraint(PxRenderBuffer &out,
		const void* constantBlock,
		const PxTransform& body0Transform,
		const PxTransform& body1Transform,
		PxReal frameScale, 
		PxReal limitScale, 
		PxU32 flags){ PX_ASSERT(body0Transform.isValid()); PX_ASSERT(body1Transform.isValid()); }

public:

	struct SuspLimitConstraintData
	{
		PxVec3 mCMOffsets[NUM_ELEMENTS];
		PxVec3 mDirs[NUM_ELEMENTS];
		PxReal mErrors[NUM_ELEMENTS];
		bool mActiveFlags[NUM_ELEMENTS];
	};
	struct StickyTyreConstraintData
	{
		PxVec3 mCMOffsets[NUM_ELEMENTS];
		PxVec3 mDirs[NUM_ELEMENTS];
		PxReal mErrors[NUM_ELEMENTS];
		bool mActiveFlags[NUM_ELEMENTS];
	};

	struct VehicleConstraintData
	{
		SuspLimitConstraintData mSuspLimitData;
		StickyTyreConstraintData mStickyTyreData;
	};
	VehicleConstraintData mData;

	PxConstraint* mConstraint;
};

#ifndef PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif //PX_VEHICLE_SUSP_LIMIT_CONSTRAINT_SHADER_H

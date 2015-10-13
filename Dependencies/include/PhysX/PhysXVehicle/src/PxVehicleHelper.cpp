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

#include "PxVehicle.h"
#include "PxQuat.h"
#include "PxRigidDynamic.h"
#include "CmPhysXCommon.h"
#include "PsFoundation.h"
#include "PsUtilities.h"

namespace physx
{

bool PxInitVehicles(PxPhysics& physics)
{
	Ps::Foundation::setInstance(static_cast<Ps::Foundation*>(&physics.getFoundation()));
	return true;
}

void PxVehicle4WSetToRestState(PxVehicle4W& vehicle)
{
	//Set analog inputs to zero so the vehicle starts completely at rest.
	for(PxU32 i=0;i<PxVehicleControlInputs::eMAX_NUM_VEHICLE_ANALOG_INPUTS;i++)
	{
		vehicle.mControlAnalogVals[i]=0.0f;
	}
	vehicle.mGearUpPressed=false;
	vehicle.mGearDownPressed=false;

	//Set the vehicle to neutral gear.
	vehicle.mCurrentGear=PxVehicleGearsData::eNEUTRAL;
	vehicle.mTargetGear=PxVehicleGearsData::eNEUTRAL;
	vehicle.mGearSwitchTime=0.0f;
	vehicle.mAutoBoxSwitchTime=0.0f;

	//Set the rigid body to rest and clear all the accumulated forces and impulses.
	vehicle.mActor->setLinearVelocity(PxVec3(0,0,0));
	vehicle.mActor->setAngularVelocity(PxVec3(0,0,0));
	vehicle.mActor->clearForce(PxForceMode::eFORCE);
	vehicle.mActor->clearForce(PxForceMode::eIMPULSE);
	vehicle.mActor->clearTorque(PxForceMode::eFORCE);
	vehicle.mActor->clearTorque(PxForceMode::eIMPULSE);
	//Set internal dynamics to zero so the vehicle starts completely at rest.
	for(PxU32 i=0;i<PxVehicle4WSimulationData::eNUM_WHEELS+1;i++)
	{
		vehicle.mInternalDynamics[i]=0.0f;
		vehicle.mTyreLowForwardSpeedTimers[i]=0.0f;
	}

	//Not really needed for physics but useful for rendering.
	for(PxU32 i=0;i<PxVehicle4WSimulationData::eNUM_WHEELS;i++)
	{
		vehicle.mWheelRotationAngles[i]=0.0f;
	}
}

PxReal PxVehicle4WGetAppliedAccel(const PxVehicle4W& vehicle)
{
	return vehicle.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_ACCEL];
}

PxReal PxVehicle4WGetAppliedBrake(const PxVehicle4W& vehicle)
{
	return vehicle.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_BRAKE];
}

PxReal PxVehicle4WGetAppliedHandbrake(const PxVehicle4W& vehicle)
{
	return vehicle.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_HANDBRAKE];
}

PxReal PxVehicle4WGetAppliedSteer(const PxVehicle4W& vehicle)
{
	return vehicle.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_RIGHT]-vehicle.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_LEFT];
}

bool PxVehicle4WGetAppliedGearup(const PxVehicle4W& vehicle)
{
	return vehicle.mGearUpPressed;
}

bool PxVehicle4WGetAppliedGeardown(const PxVehicle4W& vehicle)
{
	return vehicle.mGearDownPressed;
}

void PxVehicle4WSetUseAutoGears(const bool useAutoGears, PxVehicle4W& vehicle)
{
	vehicle.mUseAutoGears=useAutoGears;
}

bool PxVehicle4WGetUseAutoGears(const PxVehicle4W& vehicle)
{
	return vehicle.mUseAutoGears;
}

PxU32 PxVehicle4WGetCurrentGear(const PxVehicle4W& vehicle)
{
	return vehicle.mCurrentGear;
}

PxU32 PxVehicle4WGetTargetGear(const PxVehicle4W& vehicle)
{
	return vehicle.mTargetGear;
}

void PxVehicle4WStartGearChange(const PxU32 targetGear, PxVehicle4W& vehicle)
{
	vehicle.mTargetGear=targetGear;
}

void PxVehicle4WForceGearChange(const PxU32 targetGear, PxVehicle4W& vehicle)
{
	vehicle.mTargetGear=targetGear;
	vehicle.mCurrentGear=targetGear;
}


void PxVehicle4WSetAnalogInputs(const PxF32 accel, const PxF32 brake, const PxF32 handbrake, const PxF32 steer, const bool gearup, const bool geardown, PxVehicle4W& vehicle)
{
	PX_CHECK_MSG(accel>=0 && accel<=1, "Accel must be in range (0,1)");
	PX_CHECK_MSG(brake>=0 && brake<=1, "brake must be in range (0,1)");
	PX_CHECK_MSG(handbrake>=0 && handbrake<=1, "handbrake must be in range (0,1)");
	PX_CHECK_MSG(steer>=-1 && steer<=1, "steer must be in range (-1,1)");
	PX_CHECK_MSG(!gearup || !geardown, "Impossible to change up gear and change down gear simultaneously");

	vehicle.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_ACCEL]=accel;
	vehicle.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_BRAKE]=brake;
	vehicle.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_HANDBRAKE]=handbrake;
	if(steer>0)
	{
		vehicle.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_RIGHT]=steer;
		vehicle.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_LEFT]=0.0f;
	}
	else
	{
		vehicle.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_RIGHT]=0.0f;
		vehicle.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_LEFT]=-steer;
	}
	vehicle.mGearUpPressed=gearup;
	vehicle.mGearDownPressed=geardown;
}

PxF32 PxVehicle4WGetTypePairFriction
(const PxVehicleDrivableSurfaceToTyreFrictionPairs& pairs, const PxU32 surfaceType, const PxU32 tyreType)
{
	return *(pairs.mPairs + pairs.mNumTyreTypes*surfaceType + tyreType);
}

bool PxVehicle4WIsInAir(const PxVehicle4W& vehicle)
{
	return 
		(
		vehicle.mSuspJounces[PxVehicle4WSimulationData::eFRONT_LEFT_WHEEL]==-vehicle.mVehicleSimData.getSuspensionData(PxVehicle4WSimulationData::eFRONT_LEFT_WHEEL).mMaxDroop && 
		vehicle.mSuspJounces[PxVehicle4WSimulationData::eFRONT_RIGHT_WHEEL]==-vehicle.mVehicleSimData.getSuspensionData(PxVehicle4WSimulationData::eFRONT_RIGHT_WHEEL).mMaxDroop &&
		vehicle.mSuspJounces[PxVehicle4WSimulationData::eREAR_LEFT_WHEEL]==-vehicle.mVehicleSimData.getSuspensionData(PxVehicle4WSimulationData::eREAR_LEFT_WHEEL).mMaxDroop && 
		vehicle.mSuspJounces[PxVehicle4WSimulationData::eREAR_RIGHT_WHEEL]==-vehicle.mVehicleSimData.getSuspensionData(PxVehicle4WSimulationData::eREAR_RIGHT_WHEEL).mMaxDroop
		);
}

PxF32 PxVehicle4WComputeForwardSpeed(const PxVehicle4W& vehicle)
{
	const PxTransform vehicleChassisTrnsfm=vehicle.mActor->getGlobalPose().transform(vehicle.mActor->getCMassLocalPose());
	return vehicle.mActor->getLinearVelocity().dot(vehicleChassisTrnsfm.q.getBasisVector2());
}

PxF32 PxVehicle4WComputeSidewaysSpeed(const PxVehicle4W& vehicle)
{
	const PxTransform vehicleChassisTrnsfm=vehicle.mActor->getGlobalPose().transform(vehicle.mActor->getCMassLocalPose());
	return vehicle.mActor->getLinearVelocity().dot(vehicleChassisTrnsfm.q.getBasisVector1());
}

PxShape* PxVehicle4WGetFrontLeftWheelShape(const PxVehicle4W& vehicle)
{
	PxShape* shapeBuffer[PxVehicle4WSimulationData::eNUM_WHEELS + PX_MAX_NUM_CHASSIS_SHAPES];
	const PxU32 numShapes=vehicle.mActor->getNbShapes();
	PX_ASSERT(numShapes <= (PxVehicle4WSimulationData::eNUM_WHEELS + PX_MAX_NUM_CHASSIS_SHAPES));
	vehicle.mActor->getShapes(shapeBuffer,1);
	return shapeBuffer[0];
}
PxShape* PxVehicle4WGetFrontRightWheelShape(const PxVehicle4W& vehicle)
{
	PxShape* shapeBuffer[PxVehicle4WSimulationData::eNUM_WHEELS + PX_MAX_NUM_CHASSIS_SHAPES];
	const PxU32 numShapes=vehicle.mActor->getNbShapes();
	PX_ASSERT(numShapes <= (PxVehicle4WSimulationData::eNUM_WHEELS + PX_MAX_NUM_CHASSIS_SHAPES));
	vehicle.mActor->getShapes(shapeBuffer,2);
	return shapeBuffer[1];
}
PxShape* PxVehicle4WGetRearLeftWheelShape(const PxVehicle4W& vehicle)
{
	PxShape* shapeBuffer[PxVehicle4WSimulationData::eNUM_WHEELS + PX_MAX_NUM_CHASSIS_SHAPES];
	const PxU32 numShapes=vehicle.mActor->getNbShapes();
	PX_ASSERT(numShapes <= (PxVehicle4WSimulationData::eNUM_WHEELS + PX_MAX_NUM_CHASSIS_SHAPES));
	vehicle.mActor->getShapes(shapeBuffer,3);
	return shapeBuffer[2];
}
PxShape* PxVehicle4WGetRearRightWheelShape(const PxVehicle4W& vehicle)
{
	PxShape* shapeBuffer[PxVehicle4WSimulationData::eNUM_WHEELS + PX_MAX_NUM_CHASSIS_SHAPES];
	const PxU32 numShapes=vehicle.mActor->getNbShapes();
	PX_ASSERT(numShapes <= (PxVehicle4WSimulationData::eNUM_WHEELS + PX_MAX_NUM_CHASSIS_SHAPES));
	vehicle.mActor->getShapes(shapeBuffer,4);
	return shapeBuffer[3];
}
PxU32 PxVehicle4WGetNumChassisShapes(const PxVehicle4W& vehicle)
{
	return (vehicle.mActor->getNbShapes() - PxVehicle4WSimulationData::eNUM_WHEELS);
}
void PxVehicle4WGetChassisShapes(const PxVehicle4W& vehicle, PxShape** chassisShapeBuffer, const PxU32 numShapes)
{
	PxShape* shapeBuffer[PxVehicle4WSimulationData::eNUM_WHEELS + PX_MAX_NUM_CHASSIS_SHAPES];
	PX_ASSERT(numShapes <= PX_MAX_NUM_CHASSIS_SHAPES);
	vehicle.mActor->getShapes(shapeBuffer,PxVehicle4WSimulationData::eNUM_WHEELS+numShapes);
	for(PxU32 i=0;i<numShapes;i++)
	{
		chassisShapeBuffer[i]=shapeBuffer[PxVehicle4WSimulationData::eNUM_WHEELS+i];
	}
}

PxReal PxVehicle4WGetWheelRotationSpeed(const PxVehicle4W& vehicle, const PxU32 wheelIdx)
{
	PX_ASSERT(wheelIdx <= PxVehicle4WSimulationData::eNUM_WHEELS);
	return vehicle.mInternalDynamics[wheelIdx];
}

PxReal PxVehicle4WGetEngineRotationSpeed(const PxVehicle4W& vehicle)
{
	return vehicle.mInternalDynamics[PxVehicle4WSimulationData::eNUM_WHEELS];
}


PxReal PxVehicle4WGetWheelRotationAngle(const PxVehicle4W& vehicle, const PxU32 wheelIdx)
{
	PX_ASSERT(wheelIdx <= PxVehicle4WSimulationData::eNUM_WHEELS);
	return vehicle.mWheelRotationAngles[wheelIdx];
}

PxReal PxVehicle4WGetSuspJounce(const PxVehicle4W& vehicle, const PxU32 suspIdx)
{
	PX_ASSERT(suspIdx <= PxVehicle4WSimulationData::eNUM_WHEELS);
	return vehicle.mSuspJounces[suspIdx];
}

PxReal PxVehicle4WGetTyreLongSlip(const PxVehicle4W& vehicle, const PxU32 tyreIdx)
{
	PX_ASSERT(tyreIdx <= PxVehicle4WSimulationData::eNUM_WHEELS);
	return vehicle.mLongSlips[tyreIdx];
}

PxReal PxVehicle4WGetTyreLatSlip(const PxVehicle4W& vehicle, const PxU32 tyreIdx)
{
	PX_ASSERT(tyreIdx <= PxVehicle4WSimulationData::eNUM_WHEELS);
	return vehicle.mLatSlips[tyreIdx];
}

PxReal PxVehicle4WGetTyreFriction(const PxVehicle4W& vehicle, const PxU32 tyreIdx)
{
	PX_ASSERT(tyreIdx <= PxVehicle4WSimulationData::eNUM_WHEELS);
	return vehicle.mTyreFrictions[tyreIdx];
}

PxU32 PxVehicle4WGetTyreDrivableSurfaceType(const PxVehicle4W& vehicle, const PxU32 tyreIdx)
{
	PX_ASSERT(tyreIdx <= PxVehicle4WSimulationData::eNUM_WHEELS);
	return vehicle.mTyreSurfaceTypes[tyreIdx];
}

void PxVehicle4WGetSuspRaycast(const PxVehicle4W& vehicle, const PxU32 suspIdx, PxVec3& start, PxVec3& dir, PxReal& length)
{
	PX_ASSERT(suspIdx <= PxVehicle4WSimulationData::eNUM_WHEELS);
	start=vehicle.mSuspLineStarts[suspIdx];
	dir=vehicle.mSuspLineDirs[suspIdx];
	length=vehicle.mSuspLineLengths[suspIdx];
}

}

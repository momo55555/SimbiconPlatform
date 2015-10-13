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
#include "PxRigidDynamic.h"
#include "PxConstraint.h"
#include "CmPhysXCommon.h"
#include "PsUtilities.h"
#include "PsFoundation.h"
#ifdef PX_PS3
#include "ps3/PxVehicle4WSuspLimitConstraintShaderSpu.h"
#endif

namespace physx
{

void setupChassis
(const PxVehicle4WSimulationData& desc, 
 const PxFilterData& wheelCollFilterData, const PxFilterData& chassisCollFilterData,
 const PxFilterData& vehQryFilterData, 
 PxRigidDynamic* chassis)
{
	//Set raycast filter data to stop raycasts hitting car chassis and wheel shapes.
	//Set the type and include flags for chassis and wheel shapes.

	//Get the shapes that make up the car.
	PxShape* shapeBuffer[PxVehicle4WSimulationData::eNUM_WHEELS+PX_MAX_NUM_CHASSIS_SHAPES];
	const PxU32 numShapes=chassis->getNbShapes();
	chassis->getShapes(shapeBuffer,numShapes);

	//Make sure cars don't raycast against themselves.
	for(PxU32 i=0;i<numShapes;i++)
	{
		shapeBuffer[i]->setQueryFilterData(vehQryFilterData);
	}

	//Set up the coll type and include flags for chassis and wheels.
	for(PxU32 i=0;i<PxVehicle4WSimulationData::eNUM_WHEELS;i++)
	{
		shapeBuffer[i]->setSimulationFilterData(wheelCollFilterData);
	}
	for(PxU32 i=PxVehicle4WSimulationData::eNUM_WHEELS;i<numShapes;i++)
	{
		shapeBuffer[i]->setSimulationFilterData(chassisCollFilterData);
	}

	//Note: we still need to set up the local pose of the wheels but we'll do this later, just after we've set up 
	//the suspension/wheel/tyre parameters.

	//Modify the mass.
	chassis->setMass(desc.getChassisData().mMass);

	//Modify the moment of inertia.
	chassis->setMassSpaceInertiaTensor(desc.getChassisData().mMOI);

	//Modify the cm pos.
	chassis->setCMassLocalPose(PxTransform(desc.getChassisData().mCMOffset,PxQuat::createIdentity()));
}

} //namespace physx

using namespace physx;

void physx::PxVehicle4WSetup
(const PxVehicle4WSimulationData& data, 
 const PxFilterData& vehQryFilterData, PxRigidDynamic* vehActor,
 const PxGeometry& frontLeftWheelGeom, const PxGeometry& frontRightWheelGeom, const PxGeometry& rearLeftWheelGeom, const PxGeometry& rearRightWheelGeom, PxMaterial* const wheelMaterial, const PxFilterData& wheelCollFilterData,
 const PxGeometry* const* chassisGeoms, const PxTransform* const chassisLocalPoses, const PxU32 numChassisGeoms, PxMaterial* const chassisMaterial, const PxFilterData& chassisCollFilterData,
 PxPhysics* physics, 
 PxVehicle4W* veh)
{
	PX_CHECK_MSG(vehActor, "vehActor is null ptr - you need to instantiate an empty PxRigidDynamic for the vehicle");
	PX_CHECK_MSG(0==vehActor->getNbShapes(), "Actor already has shapes - require an empty actor");
	PX_CHECK_MSG(wheelMaterial, "Null wheel material - must specify a wheel material");
	PX_CHECK_MSG(chassisGeoms, "Null chassisGeoms - must specify an array of chassis geometries");
	PX_CHECK_MSG(chassisLocalPoses, "Null chassisLocalPoses - must specify an array of local poses for the chassis geometries");
	PX_CHECK_MSG(numChassisGeoms>=1, "You need to provide at least a single geometry for the chassis");
	PX_CHECK_MSG(numChassisGeoms<PX_MAX_NUM_CHASSIS_SHAPES, "You are only allowed 16 geometries for the chassis");

	//Add all the shapes to the actor.
	vehActor->createShape(frontLeftWheelGeom, *wheelMaterial);
	vehActor->createShape(frontRightWheelGeom, *wheelMaterial);
	vehActor->createShape(rearLeftWheelGeom, *wheelMaterial);
	vehActor->createShape(rearRightWheelGeom, *wheelMaterial);
	for(PxU32 i=0;i<numChassisGeoms;i++)
	{
		vehActor->createShape(*chassisGeoms[i],*chassisMaterial);
	}

	//Add the actor to the vehicle and set some other ptrs.
	veh->mSqResults=NULL;
	veh->mVehicleSimData=data;
	veh->mActor=vehActor;

	//Set up the chassis.
	setupChassis(data, wheelCollFilterData, chassisCollFilterData, vehQryFilterData, vehActor);

	//Set the local poses for the wheels shapes.
	PxShape* shapeBuffer[PxVehicle4WSimulationData::eNUM_WHEELS + PX_MAX_NUM_CHASSIS_SHAPES];
	vehActor->getShapes(shapeBuffer,PxVehicle4WSimulationData::eNUM_WHEELS + numChassisGeoms);
	for(PxU32 i=0;i<PxVehicle4WSimulationData::eNUM_WHEELS;i++)
	{
		shapeBuffer[i]->setLocalPose(PxTransform(data.getChassisData().mCMOffset+veh->mVehicleSimData.getWheelCentreOffset(i),PxQuat::createIdentity()));
	}
	//Set the local poses for the chassis shapes.
	for(PxU32 i=PxVehicle4WSimulationData::eNUM_WHEELS;i<PxVehicle4WSimulationData::eNUM_WHEELS + numChassisGeoms;i++)
	{
		shapeBuffer[i]->setLocalPose(chassisLocalPoses[i-PxVehicle4WSimulationData::eNUM_WHEELS]);
	}

	//Set up the suspension limits.
	for(PxU32 i=0;i<PxVehicle4WSimulationData::eNUM_WHEELS;i++)
	{
		veh->getVehicletConstraintShader().mData.mSuspLimitData.mCMOffsets[i]=veh->mVehicleSimData.getSuspForceAppPointOffset(i);
		veh->getVehicletConstraintShader().mData.mSuspLimitData.mDirs[i]=veh->mVehicleSimData.getSuspTravelDirection(i);
		veh->getVehicletConstraintShader().mData.mSuspLimitData.mErrors[i]=0.0f;
		veh->getVehicletConstraintShader().mData.mSuspLimitData.mActiveFlags[i]=false;

		veh->getVehicletConstraintShader().mData.mStickyTyreData.mCMOffsets[i]=PxVec3(0,0,0);
		veh->getVehicletConstraintShader().mData.mStickyTyreData.mDirs[i]=PxVec3(0,0,0);
		veh->getVehicletConstraintShader().mData.mStickyTyreData.mErrors[i]=0.0f;
		veh->getVehicletConstraintShader().mData.mStickyTyreData.mActiveFlags[i]=false;

	}

#ifdef PX_PS3
	PxConstraintShaderTable t = { 
		PxVehicleConstraintShader<PxVehicle4WSimulationData::eNUM_WHEELS>::vehicleSuspLimitConstraintSolverPrep,
		PxVehicle4WSuspLimitConstraintShaderSpu,
		PXVEHICLE4WSUSPLIMITCONSTRAINTSHADERSPU_SIZE,
		0,
		PxVehicleConstraintShader<PxVehicle4WSimulationData::eNUM_WHEELS>::visualiseConstraint
	};
#else
	PxConstraintShaderTable t = { 
		PxVehicleConstraintShader<PxVehicle4WSimulationData::eNUM_WHEELS>::vehicleSuspLimitConstraintSolverPrep,
		0,0,0,
		PxVehicleConstraintShader<PxVehicle4WSimulationData::eNUM_WHEELS>::visualiseConstraint
	};
#endif

	veh->getVehicletConstraintShader().mConstraint=physics->createConstraint(veh->mActor, NULL, veh->getVehicletConstraintShader(), t, 
																			 sizeof(PxVehicleConstraintShader<PxVehicle4WSimulationData::eNUM_WHEELS>::VehicleConstraintData));
	veh->getVehicletConstraintShader().mConstraint->markDirty();
}

void PxVehicle4WSimulationData::setChassisData(const PxVehicleChassisData& chassis)
{
	PX_CHECK_MSG(chassis.mMass>0, "Illegal mass, must be greater than zero");
	PX_CHECK_MSG(chassis.mMOI.x>0 && chassis.mMOI.y>0 && chassis.mMOI.z>0, "Illegal moi, each component must be greater than zero");

	mChassis=chassis;
}

void PxVehicle4WSimulationData::setEngineData(const PxVehicleEngineData& engine)
{
	PX_CHECK_MSG(engine.mTorqueCurve.getNumDataPairs()>0, "Engine torque curve must specify at least one entry");
	PX_CHECK_MSG(engine.mPeakTorque>0, "Engine peak torque  must be greater than zero");
	PX_CHECK_MSG(engine.mMaxOmega>0, "Engine max omega must be greater than zero");
	//PX_CHECK_MSG(engine.mInertia>0, "Engine inertia must be greater than zero");
	PX_CHECK_MSG(engine.mEngagedClutchDampingRate>=0, "Engine engaged clutch damping rate must be greater than or equal to zero");
	PX_CHECK_MSG(engine.mDisengagedClutchDampingRate>=0, "Engine disengaged clutch damping rate must be greater than or equal to zero");

	mEngine=engine;
	//mEngine.mRecipInertia=1.0f/engine.mInertia;
	mEngine.mRecipMaxOmega=1.0f/engine.mMaxOmega;
}

void PxVehicle4WSimulationData::setGearsData(const PxVehicleGearsData& gears)
{
	PX_CHECK_MSG(gears.mRatios[PxVehicleGearsData::eREVERSE]<0, "Reverse gear ratio must be negative");
	PX_CHECK_MSG(gears.mRatios[PxVehicleGearsData::eNEUTRAL]==0, "Neutral gear ratio must be zero");
	PX_CHECK_MSG(gears.mRatios[PxVehicleGearsData::eFIRST]>0, "First gear ratio must be positive");
	PX_CHECK_MSG(PxVehicleGearsData::eSECOND>=gears.mNumRatios || (gears.mRatios[PxVehicleGearsData::eSECOND]>0 && gears.mRatios[PxVehicleGearsData::eSECOND] < gears.mRatios[PxVehicleGearsData::eFIRST]), "Second gear ratio must be positive and less than first gear ratio");
	PX_CHECK_MSG(PxVehicleGearsData::eTHIRD>=gears.mNumRatios || (gears.mRatios[PxVehicleGearsData::eTHIRD]>0 && gears.mRatios[PxVehicleGearsData::eTHIRD] < gears.mRatios[PxVehicleGearsData::eSECOND]), "Third gear ratio must be positive and less than second gear ratio");
	PX_CHECK_MSG(PxVehicleGearsData::eFOURTH>=gears.mNumRatios || (gears.mRatios[PxVehicleGearsData::eFOURTH]>0 && gears.mRatios[PxVehicleGearsData::eFOURTH] < gears.mRatios[PxVehicleGearsData::eTHIRD]), "Fourth gear ratio must be positive and less than third gear ratio");
	PX_CHECK_MSG(PxVehicleGearsData::eFIFTH>=gears.mNumRatios || (gears.mRatios[PxVehicleGearsData::eFIFTH]>0 && gears.mRatios[PxVehicleGearsData::eFIFTH] < gears.mRatios[PxVehicleGearsData::eFOURTH]), "Fifth gear ratio must be positive and less than fourth gear ratio");
	PX_CHECK_MSG(PxVehicleGearsData::eSIXTH>=gears.mNumRatios || (gears.mRatios[PxVehicleGearsData::eSIXTH]>0 && gears.mRatios[PxVehicleGearsData::eSIXTH] < gears.mRatios[PxVehicleGearsData::eFIFTH]), "Sixth gear ratio must be positive and less than fifth gear ratio");
	PX_CHECK_MSG(gears.mFinalRatio>0, "Final gear ratio must be greater than zero");
	PX_CHECK_MSG(gears.mNumRatios>=3, "Number of gear ratios must be at least 3 - we need at least reverse, neutral, and a forward gear");

	mGears=gears;
}

void PxVehicle4WSimulationData::setClutchData(const PxVehicleClutchData& clutch)
{
	PX_CHECK_MSG(clutch.mStrength>0, "Clutch strength must be greater than zero");

	mClutch=clutch;
}

void PxVehicle4WSimulationData::setAutoBoxData(const PxVehicleAutoBoxData& autobox)
{
	PX_CHECK_MSG(autobox.mUpRatios[PxVehicleGearsData::eREVERSE]>=0, "Autobox gearup ratio in reverse must be greater than or equal to zero");
	PX_CHECK_MSG(autobox.mUpRatios[PxVehicleGearsData::eNEUTRAL]>=0, "Autobox gearup ratio in neutral must be greater than zero");
	PX_CHECK_MSG(autobox.mUpRatios[PxVehicleGearsData::eFIRST]>=0, "Autobox gearup ratio in first must be greater than or equal to zero");
	PX_CHECK_MSG(autobox.mUpRatios[PxVehicleGearsData::eSECOND]>=0, "Autobox gearup ratio in second must be greater than zero");
	PX_CHECK_MSG(autobox.mUpRatios[PxVehicleGearsData::eTHIRD]>=0, "Autobox gearup ratio in third must be greater than zero");
	PX_CHECK_MSG(autobox.mUpRatios[PxVehicleGearsData::eFOURTH]>=0, "Autobox gearup ratio in fourth must be greater than zero");
	PX_CHECK_MSG(autobox.mUpRatios[PxVehicleGearsData::eFIFTH]>=0, "Autobox gearup ratio in fifth must be greater than zero");
	PX_CHECK_MSG(autobox.mDownRatios[PxVehicleGearsData::eREVERSE]>=0, "Autobox geardown ratio in reverse must be greater than or equal to zero");
	PX_CHECK_MSG(autobox.mDownRatios[PxVehicleGearsData::eNEUTRAL]>=0, "Autobox geardown ratio in neutral must be greater than zero");
	PX_CHECK_MSG(autobox.mDownRatios[PxVehicleGearsData::eFIRST]>=0, "Autobox geardown ratio in first must be greater than or equal to zero");
	PX_CHECK_MSG(autobox.mDownRatios[PxVehicleGearsData::eSECOND]>=0, "Autobox geardown ratio in second must be greater than zero");
	PX_CHECK_MSG(autobox.mDownRatios[PxVehicleGearsData::eTHIRD]>=0, "Autobox geardown ratio in third must be greater than zero");
	PX_CHECK_MSG(autobox.mDownRatios[PxVehicleGearsData::eFOURTH]>=0, "Autobox geardown ratio in fourth must be greater than zero");
	PX_CHECK_MSG(autobox.mDownRatios[PxVehicleGearsData::eFIFTH]>=0, "Autobox geardown ratio in fifth must be greater than zero");
	PX_CHECK_MSG(autobox.mDownRatios[PxVehicleGearsData::eSIXTH]>=0, "Autobox geardown ratio in fifth must be greater than zero");

	mAutoBox=autobox;
}

void PxVehicle4WSimulationData::setDiffData(const PxVehicleDifferential4WData& diff)
{
	PX_CHECK_MSG(diff.mType!=PxVehicleDifferential4WData::eDIFF_TYPE_LS_4WD || (diff.mFrontRearSplit>=0 && diff.mFrontRearSplit<=1.0f), "Diff torque split between front and rear must be in range (0,1)");
	PX_CHECK_MSG(diff.mType!=PxVehicleDifferential4WData::eDIFF_TYPE_LS_4WD || (diff.mCentreBias>=1), "Diff centre bias must be greater than or equal to 1");
	PX_CHECK_MSG((diff.mType!=PxVehicleDifferential4WData::eDIFF_TYPE_LS_4WD && diff.mType!=PxVehicleDifferential4WData::eDIFF_TYPE_LS_FRONTWD) || (diff.mFrontBias>=1), "Diff front bias must be greater than or equal to 1");
	PX_CHECK_MSG((diff.mType!=PxVehicleDifferential4WData::eDIFF_TYPE_LS_4WD && diff.mType!=PxVehicleDifferential4WData::eDIFF_TYPE_LS_REARWD) || (diff.mRearBias>=1), "Diff rear bias must be greater than or equal to 1");
	PX_CHECK_MSG(diff.mType<PxVehicleDifferential4WData::eMAX_NUM_DIFF_TYPES, "Illegal differential type");

	mDiff=diff;
}

void PxVehicle4WSimulationData::setSuspensionData(const PxVehicleSuspensionData& susp, const PxU32 id)
{
	PX_CHECK_MSG(id<eNUM_WHEELS, "Illegal suspension id");
	PX_CHECK_MSG(susp.mSpringStrength>0, "Susp spring strength must be greater than zero");
	PX_CHECK_MSG(susp.mSpringDamperRate>=0, "Susp spring damper rate must be greater than or equal to zero");
	PX_CHECK_MSG(susp.mMaxCompression>0, "Susp max compression must be greater than zero");
	PX_CHECK_MSG(susp.mMaxDroop>0, "Susp max droop must be greater than zero");
	PX_CHECK_MSG(susp.mSprungMass>0, "Susp spring mass must be greater than zero");

	mSuspensions[id]=susp;

	mTyreRestLoads[id]=mWheels[id].mMass+mSuspensions[id].mSprungMass;
	mRecipTyreRestLoads[id]=1.0f/mTyreRestLoads[id];
}

void PxVehicle4WSimulationData::setWheelData(const PxVehicleWheelData& wheel, const PxU32 id)
{
	PX_CHECK_MSG(id<eNUM_WHEELS, "Illegal wheel id");
	PX_CHECK_MSG(wheel.mRadius>0, "Wheel radius must be greater than zero");
	PX_CHECK_MSG(wheel.mMaxBrakeTorque>=0, "Wheel brake torque must be zero or be a positive value");
	PX_CHECK_MSG(wheel.mMaxHandBrakeTorque>=0, "Wheel handbrake torque must be zero or be a positive value");
	PX_CHECK_MSG(wheel.mMaxSteer>=0, "Wheel max steer must be zero or be a positive value");
	PX_CHECK_MSG(wheel.mMaxSteer<PxHalfPi, "Wheel max steer must be in range (0,Pi/2)");
	PX_CHECK_MSG(wheel.mMass>0, "Wheel mass must be greater than zero");
	PX_CHECK_MSG(wheel.mMOI>0, "Wheel moi must be greater than zero");
	PX_CHECK_MSG(wheel.mToeAngle>-PxHalfPi && wheel.mToeAngle<PxHalfPi, "Wheel toe angle must be in range (-Pi/2,Pi/2)");
	PX_CHECK_MSG(wheel.mWidth>0, "Wheel width must be greater than zero");
	PX_CHECK_MSG(wheel.mDampingRate>=0, "Wheel damping rate must be greater than or equal to zero");

	mWheels[id]=wheel;
	mWheels[id].mRecipRadius=1.0f/mWheels[id].mRadius;
	mWheels[id].mRecipMOI=1.0f/mWheels[id].mMOI;

	mTyreRestLoads[id]=mWheels[id].mMass+mSuspensions[id].mSprungMass;
	mRecipTyreRestLoads[id]=1.0f/mTyreRestLoads[id];
}

void PxVehicle4WSimulationData::setTyreData(const PxVehicleTyreData& tyre, const PxU32 id)
{
	PX_CHECK_MSG(id<eNUM_WHEELS, "Illegal tyre id");
	PX_CHECK_MSG(tyre.mLatStiffX>0, "Tyre mLatStiffX must greater than zero");
	PX_CHECK_MSG(tyre.mLatStiffY>0, "Tyre mLatStiffY must greater than zero");
	PX_CHECK_MSG(tyre.mLongitudinalStiffness>0, "Tyre longitudinal stiffness must greater than zero");
	PX_CHECK_MSG(tyre.mCamberStiffness>=0, "Tyre camber stiffness must greater than or equal to zero");
	PX_CHECK_MSG(tyre.mFrictionVsSlipGraph[0][0]==0, "mFrictionVsSlipGraph[0][0] must be zero");
	PX_CHECK_MSG(tyre.mFrictionVsSlipGraph[0][1]>0, "mFrictionVsSlipGraph[0][0] must be greater than zero");
	PX_CHECK_MSG(tyre.mFrictionVsSlipGraph[1][0]>0, "mFrictionVsSlipGraph[1][0] must be greater than zero");
	PX_CHECK_MSG(tyre.mFrictionVsSlipGraph[1][1]>=tyre.mFrictionVsSlipGraph[0][1], "mFrictionVsSlipGraph[1][1] must be greater than mFrictionVsSlipGraph[0][1]");
	PX_CHECK_MSG(tyre.mFrictionVsSlipGraph[2][0]> tyre.mFrictionVsSlipGraph[1][0], "mFrictionVsSlipGraph[2][0] must be greater than mFrictionVsSlipGraph[1][0]");
	PX_CHECK_MSG(tyre.mFrictionVsSlipGraph[2][1]<=tyre.mFrictionVsSlipGraph[1][1], "mFrictionVsSlipGraph[2][1] must be less than or equal to mFrictionVsSlipGraph[1][1]");

	mTyres[id]=tyre;
	mTyres[id].mRecipLongitudinalStiffness=1.0f/mTyres[id].mLongitudinalStiffness;
	mTyres[id].mFrictionVsSlipGraphRecipx1Minusx0=1.0f/(mTyres[id].mFrictionVsSlipGraph[1][0]-mTyres[id].mFrictionVsSlipGraph[0][0]);
	mTyres[id].mFrictionVsSlipGraphRecipx2Minusx1=1.0f/(mTyres[id].mFrictionVsSlipGraph[2][0]-mTyres[id].mFrictionVsSlipGraph[1][0]);
}

void PxVehicle4WSimulationData::setSuspTravelDirection(const PxVec3& dir, const PxU32 id)
{
	PX_CHECK_MSG(id<eNUM_WHEELS, "Illegal suspension id");
	PX_CHECK_MSG(dir.magnitude()>0.999f && dir.magnitude()<1.0001f, "Suspension travel dir must be unit vector");

	mSuspDownwardTravelDirections[id]=dir;
}

void PxVehicle4WSimulationData::setSuspForceAppPointOffset(const PxVec3& offset, const PxU32 id)
{
	PX_CHECK_MSG(id<eNUM_WHEELS, "Illegal suspension id");
	PX_CHECK_MSG(offset.magnitude()>0, "Susp force app point must not be offset from centre of mass");

	mSuspForceAppPointOffsets[id]=offset;
}

void PxVehicle4WSimulationData::setTyreForceAppPointOffset(const PxVec3& offset, const PxU32 id)
{
	PX_CHECK_MSG(id<eNUM_WHEELS, "Illegal tyre id");
	PX_CHECK_MSG(offset.magnitude()>0, "Tyre force app point must not be offset from centre of mass");

	mTyreForceAppPointOffsets[id]=offset;
}

void PxVehicle4WSimulationData::setWheelCentreOffsets(const PxVec3& offsetFL, const PxVec3& offsetFR, const PxVec3& offsetRL, const PxVec3& offsetRR)
{
	PX_CHECK_MSG(offsetFL.magnitude()>0, "Front left wheel centre must be offset from centre of mass");
	PX_CHECK_MSG(offsetFR.magnitude()>0, "Front right wheel centre must be offset from centre of mass");
	PX_CHECK_MSG(offsetRL.magnitude()>0, "Rear left wheel centre must be offset from centre of mass");
	PX_CHECK_MSG(offsetRR.magnitude()>0, "Rear right wheel centre must be offset from centre of mass");

	mWheelCentreOffsets[PxVehicle4WSimulationData::eFRONT_LEFT_WHEEL]=offsetFL;
	mWheelCentreOffsets[PxVehicle4WSimulationData::eFRONT_RIGHT_WHEEL]=offsetFR;
	mWheelCentreOffsets[PxVehicle4WSimulationData::eREAR_LEFT_WHEEL]=offsetRL;
	mWheelCentreOffsets[PxVehicle4WSimulationData::eREAR_RIGHT_WHEEL]=offsetRR;

	mAckermannGeometry.mFrontWidth=mWheelCentreOffsets[PxVehicle4WSimulationData::eFRONT_RIGHT_WHEEL].x-mWheelCentreOffsets[PxVehicle4WSimulationData::eFRONT_LEFT_WHEEL].x;
	mAckermannGeometry.mAxleSeparation=mWheelCentreOffsets[PxVehicle4WSimulationData::eFRONT_LEFT_WHEEL].z-mWheelCentreOffsets[PxVehicle4WSimulationData::eREAR_LEFT_WHEEL].z;	
}

void PxVehicle4WSimulationData::setTyreLoadFilterData(const PxVehicleTyreLoadFilterData& tyreLoadFilter)
{
	PX_CHECK_MSG(tyreLoadFilter.mMaxNormalisedLoad>tyreLoadFilter.mMinNormalisedLoad, "Illegal graph points");
	PX_CHECK_MSG(tyreLoadFilter.mMaxFilteredNormalisedLoad>0, "Max filtered load must be greater than zero");
	mNormalisedLoadFilter=tyreLoadFilter;
	mNormalisedLoadFilter.mDenominator=1.0f/(mNormalisedLoadFilter.mMaxNormalisedLoad-mNormalisedLoadFilter.mMinNormalisedLoad);
}

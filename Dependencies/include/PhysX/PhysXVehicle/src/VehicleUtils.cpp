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

#include "PxVehicleUtils.h"
#include "PxVehicle.h"
#include "PxRigidDynamic.h"
#include "PxConstraint.h"
#include "CmPhysXCommon.h"
#include "PsFoundation.h"
#include "PsUtilities.h"
#include "stdio.h"

namespace physx
{

void setChassisData(const PxVehicle4WSimpleSetup& defaultSetupDesc, PxVehicle4WSimulationData& desc)
{
	PxVehicleChassisData chassisData;

	chassisData.mMass=defaultSetupDesc.mChassisMass;
	chassisData.mMOI=defaultSetupDesc.mChassisMOI;
	chassisData.mCMOffset=defaultSetupDesc.mChassisCMOffset;

	desc.setChassisData(chassisData);
}

void setEngineData(const PxVehicle4WSimpleSetup& defaultSetupDesc, PxVehicle4WSimulationData& desc)
{
	//TODO: engine torque curve data.
	PxVehicleEngineData engineData;

	engineData.mTorqueCurve=defaultSetupDesc.mEngineTorqueCurve;
	engineData.mPeakTorque=defaultSetupDesc.mEnginePeakTorque;
	engineData.mMaxOmega=defaultSetupDesc.mEngineMaxOmega;
	engineData.mEngagedClutchDampingRate=defaultSetupDesc.mEngineEngagedClutchDampingRate;
	engineData.mDisengagedClutchDampingRate=defaultSetupDesc.mEngineDisengagedClutchDampingRate;

	desc.setEngineData(engineData);
}

void setGearData(const PxVehicle4WSimpleSetup& defaultSetupDesc, PxVehicle4WSimulationData& desc)
{
	PxVehicleGearsData gearsData;

	gearsData.mNumRatios=defaultSetupDesc.mNumGearRatios;
	for(PxU32 i=0;i<PxVehicleGearsData::eMAX_NUM_GEAR_RATIOS;i++)
	{
		gearsData.mRatios[i]=defaultSetupDesc.mGearRatios[i];
	}
	gearsData.mFinalRatio=defaultSetupDesc.mGearFinalRatio;
	gearsData.mSwitchTime=defaultSetupDesc.mGearSwitchTime;

	desc.setGearsData(gearsData);
}

void setClutchData(const PxVehicle4WSimpleSetup& defaultSetupDesc, PxVehicle4WSimulationData& desc)
{
	PxVehicleClutchData clutchData;

	clutchData.mStrength=defaultSetupDesc.mClutchStrength;

	desc.setClutchData(clutchData);
}

void setAutoBoxData(const PxVehicle4WSimpleSetup& defaultSetupDesc, PxVehicle4WSimulationData& desc)
{
	PxVehicleAutoBoxData autoboxData;

	for(PxU32 i=0;i<PxVehicleGearsData::eMAX_NUM_GEAR_RATIOS;i++)
	{
		autoboxData.mUpRatios[i]=defaultSetupDesc.mAutoBoxUpRatios[i];
		autoboxData.mDownRatios[i]=defaultSetupDesc.mAutoBoxDownRatios[i];
	}

	desc.setAutoBoxData(autoboxData);
}

void setDiffData(const PxVehicle4WSimpleSetup& defaultSetupDesc, PxVehicle4WSimulationData& desc)
{
	PxVehicleDifferential4WData diffData;

	diffData.mFrontRearSplit=defaultSetupDesc.mDiffFrontRearSplit;
	diffData.mCentreBias=defaultSetupDesc.mDiffCentreBias;
	diffData.mFrontBias=defaultSetupDesc.mDiffFrontBias;
	diffData.mRearBias=defaultSetupDesc.mDiffRearBias;
	diffData.mType=defaultSetupDesc.mDiffType;

	desc.setDiffData(diffData);
}

void setNormalisedLoadFilterData(const PxVehicle4WSimpleSetup& defaultSetupDesc, PxVehicle4WSimulationData& desc)
{
	PxVehicleTyreLoadFilterData tyreLoadFilterData;

	tyreLoadFilterData.mMinNormalisedLoad=defaultSetupDesc.mTyreLoadFilterMinNormalisedLoad;
	tyreLoadFilterData.mMaxNormalisedLoad=defaultSetupDesc.mTyreLoadFilterMaxNormalisedLoad;
	tyreLoadFilterData.mMaxFilteredNormalisedLoad=defaultSetupDesc.mTyreLoadFilterMaxFilteredNormalisedLoad;

	desc.setTyreLoadFilterData(tyreLoadFilterData);
}

void setSuspData(const PxF32 carMassFront, const PxF32 carMassRear, const PxVehicle4WSimpleSetup& defaultSetupDesc, const bool isFront, const PxU32 id, PxVehicle4WSimulationData& desc)
{
	PxVehicleSuspensionData suspData;

	suspData.mSpringStrength =  isFront ? defaultSetupDesc.mFrontSuspensionSpringStrength : defaultSetupDesc.mRearSuspensionSpringStrength;
	suspData.mSpringDamperRate = isFront ? defaultSetupDesc.mFrontSuspensionSpringDampingRate : defaultSetupDesc.mRearSuspensionSpringDampingRate;
	suspData.mMaxCompression =  isFront ? defaultSetupDesc.mFrontSuspensionMaxCompression : defaultSetupDesc.mRearSuspensionMaxCompression;
	suspData.mMaxDroop = isFront ? defaultSetupDesc.mFrontSuspensionMaxDroop : defaultSetupDesc.mRearSuspensionMaxDroop;
	suspData.mSprungMass = isFront ? carMassFront*0.5f : carMassRear*0.5f;

	desc.setSuspensionData(suspData,id);
}

void setWheelData
(const PxVehicle4WSimpleSetup& defaultSetupDesc, const bool isFront, const PxU32 id, PxVehicle4WSimulationData& desc)
{
	PxVehicleWheelData wheelData;

	wheelData.mMaxBrakeTorque = isFront ? defaultSetupDesc.mFrontWheelBrakeTorque : defaultSetupDesc.mRearWheelBrakeTorque;
	wheelData.mMaxHandBrakeTorque = isFront ? 0.0f : defaultSetupDesc.mRearWheelHandbrakeTorque;
	wheelData.mMaxSteer = isFront ? defaultSetupDesc.mFrontWheelMaxSteer : 0.0f;
	wheelData.mRadius = isFront ? defaultSetupDesc.mFrontWheelRadius : defaultSetupDesc.mRearWheelRadius;
	wheelData.mMass = isFront ? defaultSetupDesc.mFrontWheelMass : defaultSetupDesc.mRearWheelMass;
	wheelData.mMOI = wheelData.mMass*wheelData.mRadius*wheelData.mRadius*0.5f;
	wheelData.mDampingRate = isFront ? defaultSetupDesc.mFrontWheelDampingRate : defaultSetupDesc.mRearWheelDampingRate;
	wheelData.mToeAngle = isFront ? defaultSetupDesc.mFrontWheelToeAngle : defaultSetupDesc.mRearWheelToeAngle;
	wheelData.mWidth = isFront ? defaultSetupDesc.mFrontWheelWidth : defaultSetupDesc.mRearWheelWidth;

	desc.setWheelData(wheelData,id);
}

void setTyreData(const PxVehicle4WSimpleSetup& defaultSetupDesc, const bool isFront, const PxU32 id, PxVehicle4WSimulationData& desc)
{
	PxVehicleTyreData tyreData;

	tyreData.mLatStiffX = isFront ? defaultSetupDesc.mFrontTyreLatStiffX : defaultSetupDesc.mRearTyreLatStiffX;
	tyreData.mLatStiffY = isFront ? defaultSetupDesc.mFrontTyreLatStiffY : defaultSetupDesc.mRearTyreLatStiffY;
	tyreData.mLongitudinalStiffness = isFront ? defaultSetupDesc.mFrontTyreLongitudinalStiffness : defaultSetupDesc.mRearTyreLongitudinalStiffness;
	tyreData.mCamberStiffness = isFront ? defaultSetupDesc.mFrontTyreCamberStiffness : defaultSetupDesc.mRearTyreCamberStiffness;
	for(PxU32 i=0;i<3;i++)
	{
		for(PxU32 j=0;j<2;j++)
		{
			tyreData.mFrictionVsSlipGraph[i][j] = isFront ? defaultSetupDesc.mFrontTyreFrictionVsSlipGraph[i][j] : defaultSetupDesc.mRearTyreFrictionVsSlipGraph[i][j];
		}
	}
	tyreData.mType=isFront ? defaultSetupDesc.mFrontTyreType : defaultSetupDesc.mRearTyreType;

	desc.setTyreData(tyreData,id);
}

void setSuspWheelTyreData
(const PxVehicle4WSimpleSetup& defaultSetupDesc, const PxF32 carMassFront, const PxF32 carMassRear, const PxU32 id, PxVehicle4WSimulationData& desc, PxVec3* wheelCentreOffsets)
{
	bool isFront=false;
	switch(id)
	{
	case 0://fl
	case 1://fr
		isFront=true;
		break;
	case 2://rl
	case 3://rr
		isFront=false;
		break;
	default:
		PX_ASSERT(false);
		break;
	}

	desc.setSuspTravelDirection(isFront ? defaultSetupDesc.mFrontSuspensionTravelDir  : defaultSetupDesc.mRearSuspensionTravelDir, id);

	wheelCentreOffsets[id]=defaultSetupDesc.mWheelCentreCMOffsets[id];

	if(isFront)
	{
		desc.setSuspForceAppPointOffset(
			defaultSetupDesc.mWheelCentreCMOffsets[id] - 
				defaultSetupDesc.mFrontSuspensionTravelDir *
					(defaultSetupDesc.mFrontSuspensionTravelDir.dot(defaultSetupDesc.mWheelCentreCMOffsets[id]) - defaultSetupDesc.mFrontSuspForceAppPointVerticalCMOffset),
			id);

		desc.setTyreForceAppPointOffset(
			defaultSetupDesc.mWheelCentreCMOffsets[id] -
				defaultSetupDesc.mFrontSuspensionTravelDir *
					(defaultSetupDesc.mFrontSuspensionTravelDir.dot(defaultSetupDesc.mWheelCentreCMOffsets[id]) - defaultSetupDesc.mFrontTyreForceAppPointVerticalCMOffset),
			id);

	}
	else
	{
		desc.setSuspForceAppPointOffset(
			defaultSetupDesc.mWheelCentreCMOffsets[id] - 
				defaultSetupDesc.mRearSuspensionTravelDir *
					(defaultSetupDesc.mRearSuspensionTravelDir.dot(defaultSetupDesc.mWheelCentreCMOffsets[id]) - defaultSetupDesc.mRearSuspForceAppPointVerticalCMOffset),
			id);

		desc.setTyreForceAppPointOffset(
			defaultSetupDesc.mWheelCentreCMOffsets[id] -
				defaultSetupDesc.mRearSuspensionTravelDir *
				(defaultSetupDesc.mRearSuspensionTravelDir.dot(defaultSetupDesc.mWheelCentreCMOffsets[id]) - defaultSetupDesc.mRearTyreForceAppPointVerticalCMOffset),
			id);
	}

	setSuspData(carMassFront,carMassRear,defaultSetupDesc,isFront,id,desc);
	setWheelData(defaultSetupDesc,isFront,id,desc);
	setTyreData(defaultSetupDesc,isFront,id,desc);
}

} //namespace physx

using namespace physx;

PxVehicle4WSimpleSetup::PxVehicle4WSimpleSetup()
{
	//Chassis and wheel geometry.
	mChassisDims=PxVec3(0,0,0);
	mChassisCMOffset=PxVec3(0,0,0);
	mChassisMass=1500.0f;
	mChassisMOI=PxVec3(3625.0f,3906.25f,1281.25f);//for vehicle with aabb extents 2.5 X 2 * 5
	for(PxU32 i=0;i<PxVehicle4WSimulationData::eNUM_WHEELS;i++)
	{
		mWheelCentreCMOffsets[i]=PxVec3(0,0,0);
	}

	//Front wheel
	mFrontWheelWidth=0;
	mFrontWheelRadius=0;
	mFrontWheelMass=20.0f;
	mFrontWheelBrakeTorque=1500.0f;
	mFrontWheelMaxSteer=PxPi*1.0f/3.0f;
	mFrontWheelToeAngle=0.0f;
	mFrontWheelDampingRate=0.25f;

	//Rear wheel.
	mRearWheelWidth=0.0f;
	mRearWheelRadius=0.0f;
	mRearWheelMass=20.0f;
	mRearWheelBrakeTorque=1500.0f;;
	mRearWheelHandbrakeTorque=4000.0f;
	mRearWheelToeAngle=0.0f;
	mRearWheelDampingRate=0.25f;


	//Front suspension.
	mFrontSuspensionTravelDir=PxVec3(0.0f,0.0f,0.0f);
	mFrontSuspensionMaxCompression=0.3f;
	mFrontSuspensionMaxDroop=0.1f;
	mFrontSuspensionSpringStrength=35000.0f;	
	mFrontSuspensionSpringDampingRate=4500.0f;

	//Rear suspension
	mRearSuspensionTravelDir=PxVec3(0.0f,0.0f,0.0f);
	mRearSuspensionMaxCompression=0.3f;
	mRearSuspensionMaxDroop=0.1f;
	mRearSuspensionSpringStrength=35000.0f;	
	mRearSuspensionSpringDampingRate=4500.0f;

	//Front tyre.
	mFrontTyreLatStiffX=2.0f;//Graph of stiff vs norm load saturates at norm load = 2.0f
	mFrontTyreLatStiffY=0.3125f*(180.0f / PxPi);//Graph of stiff vs norm load has max value Mg*0.3125 per degree
	mFrontTyreLongitudinalStiffness=10000.0f;
	mFrontTyreCamberStiffness=1.0f*(180.0f / PxPi);//specified per degree then multiplied by constant so it is per radian.
	mFrontTyreFrictionVsSlipGraph[0][0]=0.0f;
	mFrontTyreFrictionVsSlipGraph[0][1]=0.5f;
	mFrontTyreFrictionVsSlipGraph[1][0]=0.1f;
	mFrontTyreFrictionVsSlipGraph[1][1]=1.0f;
	mFrontTyreFrictionVsSlipGraph[2][0]=1.0f;
	mFrontTyreFrictionVsSlipGraph[2][1]=0.6f;
	mFrontTyreType=0;

	//Rear tyre.
	mRearTyreLatStiffX=2.0f;//Graph of stiff vs norm load saturates at norm load = 2.0f
	mRearTyreLatStiffY=0.3125f*(180.0f / PxPi);//Graph of stiff vs norm load has max value Mg*0.3125 per degree
	mRearTyreLongitudinalStiffness=10000.0f;
	mRearTyreCamberStiffness=1.0f*(180.0f / PxPi);//specified per degree then multiplied by constant so it is per radian.
	mRearTyreFrictionVsSlipGraph[0][0]=0.0f;
	mRearTyreFrictionVsSlipGraph[0][1]=0.5f;
	mRearTyreFrictionVsSlipGraph[1][0]=0.1f;
	mRearTyreFrictionVsSlipGraph[1][1]=1.0f;
	mRearTyreFrictionVsSlipGraph[2][0]=1.0f;
	mRearTyreFrictionVsSlipGraph[2][1]=0.6f;
	mRearTyreType=0;

	//Tyre load filter
	mTyreLoadFilterMinNormalisedLoad=-0.25f;
	mTyreLoadFilterMaxNormalisedLoad=3.0f;
	mTyreLoadFilterMaxFilteredNormalisedLoad=3.0f;

	//Force app points.
	mFrontSuspForceAppPointVerticalCMOffset=0.3f;
	mRearSuspForceAppPointVerticalCMOffset=0.3f;
	mFrontTyreForceAppPointVerticalCMOffset=0.3f;
	mRearTyreForceAppPointVerticalCMOffset=0.3f;

	//Diff
	mDiffFrontRearSplit=0.45f;
	mDiffCentreBias=1.3f;
	mDiffFrontBias=1.3f;
	mDiffRearBias=1.3f;
	mDiffType=PxVehicleDifferential4WData::eDIFF_TYPE_LS_4WD;

	//Engine
	mEngineTorqueCurve.addPair(0.0f, 0.8f);
	mEngineTorqueCurve.addPair(0.33f, 1.0f);
	mEngineTorqueCurve.addPair(1.0f, 0.8f);
	mEnginePeakTorque=350.0f;
	mEngineMaxOmega=600.0f;//approx 6000 rpm
	mEngineEngagedClutchDampingRate=0.15f;
	mEngineDisengagedClutchDampingRate=0.35f;

	//Gears
	mNumGearRatios=7;
	mGearRatios[PxVehicleGearsData::eREVERSE]=-4.0f;
	mGearRatios[PxVehicleGearsData::eNEUTRAL]=0.0f;
	mGearRatios[PxVehicleGearsData::eFIRST]=4.0f;
	mGearRatios[PxVehicleGearsData::eSECOND]=2.0f;
	mGearRatios[PxVehicleGearsData::eTHIRD]=1.5f;
	mGearRatios[PxVehicleGearsData::eFOURTH]=1.1f;
	mGearRatios[PxVehicleGearsData::eFIFTH]=1.0f;
	mGearFinalRatio=4.0f;
	mGearSwitchTime=0.5f;

	//Autobox
	for(PxU32 i=0;i<PxVehicleGearsData::eMAX_NUM_GEAR_RATIOS;i++)
	{
		mAutoBoxUpRatios[i]=0.65f;
		mAutoBoxDownRatios[i]=0.50f;
	}
	//Not sure how important this is but we want to kick out of neutral very quickly.
	mAutoBoxUpRatios[PxVehicleGearsData::eNEUTRAL]=0.15f;
	//Set the latency time in an unused element of one of the arrays.
	mAutoBoxDownRatios[PxVehicleGearsData::eREVERSE]=2.0f;

	//Clutch
	mClutchStrength=10.0f;

	//Ackermann steer accuracy
	mAckermannAccuracy=1.0f;
}

PxVehicle4WSimulationData physx::PxCreateVehicle4WSimulationData(const PxVehicle4WSimpleSetup& defaultSetupDesc)
{
	PX_CHECK_MSG(defaultSetupDesc.mChassisDims.x>0.0f && defaultSetupDesc.mChassisDims.y>0.0f && defaultSetupDesc.mChassisDims.z>0.0f, "Invalid chassis dimensions - the extent must be non-zero in each direction");
	PX_CHECK_MSG(defaultSetupDesc.mChassisMass>0.0f, "Invalid chassis mass - chassis must have mass greater than zero");
	PX_CHECK_MSG(defaultSetupDesc.mChassisMOI.x>0.0f && defaultSetupDesc.mChassisMOI.y>0.0f && defaultSetupDesc.mChassisMOI.z>0.0f, "Invalid chassis moi - must be non-zero about each axis");
	for(PxU32 i=0;i<PxVehicle4WSimulationData::eNUM_WHEELS;i++)
	{
		PX_CHECK_MSG(defaultSetupDesc.mWheelCentreCMOffsets[i].magnitudeSquared()>0.0f, "Wheel centre must have non-zero offset from rigid body centre of mass");
	}
	PX_CHECK_MSG(defaultSetupDesc.mFrontWheelWidth>0.0f, "Invalid front wheel width - front wheel width must be greater than zero");
	PX_CHECK_MSG(defaultSetupDesc.mFrontWheelRadius>0.0f, "Invalid front wheel radius - front wheel radius must be greater than zero");
	PX_CHECK_MSG(defaultSetupDesc.mRearWheelWidth>0.0f, "Invalid rear wheel width - rear wheel width must be greater than zero");
	PX_CHECK_MSG(defaultSetupDesc.mRearWheelRadius>0.0f, "Invalid rear wheel radius - rear wheel radius must be greater than zero");
	PX_CHECK_MSG(defaultSetupDesc.mFrontWheelMass>0.0f, "Invalid front wheel mass - front wheel mass must be greater than zero");
	PX_CHECK_MSG(defaultSetupDesc.mRearWheelMass>0.0f, "Invalid rear wheel mass - rear wheel mass must be greater than zero");
	PX_CHECK_MSG(defaultSetupDesc.mFrontSuspensionTravelDir.magnitude()>0.999f && defaultSetupDesc.mFrontSuspensionTravelDir.magnitude()<1.001f, "Invalid front suspension travel dir - must be unit vector");
	PX_CHECK_MSG(defaultSetupDesc.mRearSuspensionTravelDir.magnitude()>0.999f && defaultSetupDesc.mRearSuspensionTravelDir.magnitude()<1.001f, "Invalid rear suspension travel dir - must be unit vector");

	//Work out the front/rear mass split from the cm offset.
	//This is a very approximate calculation with lots of assumptions. 
	//massRear*zRear + massFront*zFront = mass*cm		(1)
	//massRear       + massFront        = mass			(2)
	//Rearrange (2)
	//massFront = mass - massRear						(3)
	//Substitute (3) into (1)
	//massRear(zRear - zFront) + mass*zFront = mass*cm	(4)
	//Solve (4) for massRear
	//massRear = mass(cm - zFront)/(zRear-zFront)		(5)
	//Now we also have
	//zFront = (z-cm)/2									(6a)
	//zRear = (-z-cm)/2									(6b)
	//Substituting (6a-b) into (5) gives
	//massRear = 0.5*mass*(z-3cm)/z						(7)
	const PxF32 massRear=0.5f*defaultSetupDesc.mChassisMass*(defaultSetupDesc.mChassisDims.z-3*defaultSetupDesc.mChassisCMOffset.z)/defaultSetupDesc.mChassisDims.z;
	const PxF32 massFront=defaultSetupDesc.mChassisMass-massRear;

	PxVehicle4WSimulationData desc;

	setChassisData(defaultSetupDesc,desc);
	setEngineData(defaultSetupDesc,desc);
	setGearData(defaultSetupDesc,desc);
	setClutchData(defaultSetupDesc,desc);
	setAutoBoxData(defaultSetupDesc,desc);
	setDiffData(defaultSetupDesc,desc);
	setNormalisedLoadFilterData(defaultSetupDesc,desc);

	PxVec3 wheelCentreOffsets[PxVehicle4WSimulationData::eNUM_WHEELS];
	for(PxU32 i=0;i<PxVehicle4WSimulationData::eNUM_WHEELS;i++)
	{
		setSuspWheelTyreData(defaultSetupDesc,massFront,massRear,i,desc,wheelCentreOffsets);
	}
	desc.setWheelCentreOffsets
		(wheelCentreOffsets[PxVehicle4WSimulationData::eFRONT_LEFT_WHEEL],
		 wheelCentreOffsets[PxVehicle4WSimulationData::eFRONT_RIGHT_WHEEL],
		 wheelCentreOffsets[PxVehicle4WSimulationData::eREAR_LEFT_WHEEL],
		 wheelCentreOffsets[PxVehicle4WSimulationData::eREAR_RIGHT_WHEEL]);

	desc.setAckermannAccuracy(defaultSetupDesc.mAckermannAccuracy);

	//Finished
	return desc;
}

namespace physx
{
	void PxVehicle4WSmoothDigitalRawInputsAndSetAnalogInputs
		(const PxVehicleKeySmoothingData& keySmoothing, const PxFixedSizeLookupTable<8>& steerVsForwardSpeedTable,
		const PxVehicleRawInputData& rawInputData,
		const PxF32 timestep,
		PxVehicle4W& focusVehicle);
}

void physx::PxVehicle4WSmoothDigitalRawInputsAndSetAnalogInputs
(const PxVehicleKeySmoothingData& keySmoothing, const PxFixedSizeLookupTable<8>& steerVsForwardSpeedTable,
 const PxVehicleRawInputData& rawInputData, 
 const PxF32 timestep, 
 PxVehicle4W& focusVehicle)
{
	for(PxU32 i=0;i<PxVehicleControlInputs::eMAX_NUM_VEHICLE_ANALOG_INPUTS;i++)
	{
		if(rawInputData.mRawDigitalInputs[i])
		{
			focusVehicle.mControlAnalogVals[i]+=keySmoothing.mRiseRates[i]*timestep;
		}
		else
		{
			focusVehicle.mControlAnalogVals[i]-=keySmoothing.mFallRates[i]*timestep;
		}

		focusVehicle.mControlAnalogVals[i]=PxClamp(focusVehicle.mControlAnalogVals[i],0.0f,1.0f);
	}

	const PxF32 vz=PxVehicle4WComputeForwardSpeed(focusVehicle);
	const PxF32 vzAbs=PxAbs(vz);
	const PxF32 maxSteer=(PxVehicle4WIsInAir(focusVehicle) ? 1.0f :steerVsForwardSpeedTable.getYVal(PxAbs(vz)));
	PxF32 steer=focusVehicle.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_RIGHT]-focusVehicle.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_LEFT];
	steer=PxClamp(steer,-maxSteer,maxSteer);
	if(steer>0)
	{
		focusVehicle.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_RIGHT]=steer;
		focusVehicle.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_LEFT]=0.0f;
	}
	else
	{
		focusVehicle.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_RIGHT]=0.0f;
		focusVehicle.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_LEFT]=-steer;
	}

	//Update digital inputs for focus vehicle.
	focusVehicle.mGearUpPressed=rawInputData.mRawDigitalInputs[PxVehicleControlInputs::eMAX_NUM_VEHICLE_ANALOG_INPUTS+PxVehicleControlInputs::eVEHICLE_DIGITAL_INPUT_GEAR_UP];
	focusVehicle.mGearDownPressed=rawInputData.mRawDigitalInputs[PxVehicleControlInputs::eMAX_NUM_VEHICLE_ANALOG_INPUTS+PxVehicleControlInputs::eVEHICLE_DIGITAL_INPUT_GEAR_DOWN];
}

namespace physx
{
	void PxVehicle4WSmoothAnalogRawInputsAndSetAnalogInputs
		(const PxVehiclePadSmoothingData& padSmoothing, const PxFixedSizeLookupTable<8>& steerVsForwardSpeedTable,
		const PxVehicleRawInputData& rawInputData,
		const PxF32 timestep,
		PxVehicle4W& focusVehicle);
}

void physx::PxVehicle4WSmoothAnalogRawInputsAndSetAnalogInputs
(const PxVehiclePadSmoothingData& padSmoothing, const PxFixedSizeLookupTable<8>& steerVsForwardSpeedTable,
 const PxVehicleRawInputData& rawInputData, 
 const PxF32 timestep, 
 PxVehicle4W& focusVehicle)
{
	//Update analog inputs for focus vehicle.
	const PxF32 vz=PxVehicle4WComputeForwardSpeed(focusVehicle);
	const PxF32 vzAbs=PxAbs(vz);

	//Process the accel.
	{
		PX_ASSERT(PxAbs(rawInputData.mRawAnalogInputs[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_ACCEL])<=1.01f);
		const PxF32 currentVal=focusVehicle.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_ACCEL];
		const PxF32 targetVal=rawInputData.mRawAnalogInputs[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_ACCEL];
		PxF32 val;
		if(currentVal<targetVal)
		{
			val=currentVal + padSmoothing.mRiseRates[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_ACCEL]*timestep;
			val=PxMin(val,targetVal);
		}
		else 
		{
			val=currentVal - padSmoothing.mFallRates[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_ACCEL]*timestep;
			val=PxMax(val,targetVal);
		}
		focusVehicle.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_ACCEL]=val;
	}

	//Process the brake
	{
		PX_ASSERT(PxAbs(rawInputData.mRawAnalogInputs[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_BRAKE])<=1.01f);
		const PxF32 currentVal=focusVehicle.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_BRAKE];
		const PxF32 targetVal=rawInputData.mRawAnalogInputs[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_BRAKE];
		PxF32 val;
		if(currentVal<targetVal)
		{
			val=currentVal + padSmoothing.mRiseRates[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_BRAKE]*timestep;
			val=PxMin(val,targetVal);
		}
		else 
		{
			val=currentVal - padSmoothing.mFallRates[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_BRAKE]*timestep;
			val=PxMax(val,targetVal);
		}	
		focusVehicle.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_BRAKE]=val;
	}

	//Process the handbrake
	{
		PX_ASSERT(PxAbs(rawInputData.mRawAnalogInputs[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_HANDBRAKE])<=1.01f);
		const PxF32 currentVal=focusVehicle.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_HANDBRAKE];
		const PxF32 targetVal=rawInputData.mRawAnalogInputs[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_HANDBRAKE];
		PxF32 val;
		if(currentVal<targetVal)
		{
			val=currentVal + padSmoothing.mRiseRates[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_HANDBRAKE]*timestep;
			val=PxMin(val,targetVal);
		}
		else 
		{
			val=currentVal - padSmoothing.mFallRates[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_HANDBRAKE]*timestep;
			val=PxMax(val,targetVal);
		}	
		focusVehicle.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_HANDBRAKE]=val;
	}

	//Process the steer
	{
		//Work out if the steering wheels are in the air.
		PX_ASSERT(PxAbs(rawInputData.mRawAnalogInputs[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_LEFT])<=1.01f);
		const PxF32 currentVal=focusVehicle.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_RIGHT]-focusVehicle.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_LEFT];
		const PxF32 targetVal=rawInputData.mRawAnalogInputs[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_LEFT]*(PxVehicle4WIsInAir(focusVehicle) ? 1.0f :steerVsForwardSpeedTable.getYVal(PxAbs(vz)));

		PxF32 val=0.0f;	// PT: the following code could leave that variable uninitialized!!!!!
		if(0==targetVal)
		{
			//Drift slowly back to zero 
			if(currentVal>0)
			{
				val=currentVal-padSmoothing.mFallRates[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_LEFT]*timestep;
				val=PxMax(val,0.0f);
			}
			else if(currentVal<0)
			{
				val=currentVal+padSmoothing.mFallRates[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_LEFT]*timestep;
				val=PxMin(val,0.0f);
			}
		}
		else
		{
			if(currentVal < targetVal)
			{
				if(currentVal<0)
				{
					val=currentVal + padSmoothing.mFallRates[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_LEFT]*timestep;
					val=PxMin(val,targetVal);
				}
				else
				{
					val=currentVal + padSmoothing.mRiseRates[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_LEFT]*timestep;
					val=PxMin(val,targetVal);
				}
			}
			else 
			{
				if(currentVal>0)
				{
					val=currentVal - padSmoothing.mFallRates[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_LEFT]*timestep;
					val=PxMax(val,targetVal);
				}
				else
				{
					val=currentVal - padSmoothing.mRiseRates[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_LEFT]*timestep;
					val=PxMax(val,targetVal);
				}
			}	
		}
		if(val>0)
		{
			focusVehicle.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_RIGHT]=val;
			focusVehicle.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_LEFT]=0.0f;
		}
		else
		{
			focusVehicle.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_RIGHT]=0.0f;
			focusVehicle.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_LEFT]=-val;
		}
	}

	focusVehicle.mGearUpPressed=rawInputData.mRawDigitalInputs[PxVehicleControlInputs::eMAX_NUM_VEHICLE_ANALOG_INPUTS+PxVehicleControlInputs::eVEHICLE_DIGITAL_INPUT_GEAR_UP];
	focusVehicle.mGearDownPressed=rawInputData.mRawDigitalInputs[PxVehicleControlInputs::eMAX_NUM_VEHICLE_ANALOG_INPUTS+PxVehicleControlInputs::eVEHICLE_DIGITAL_INPUT_GEAR_DOWN];
}

namespace physx
{
	void PxSetupDrivableShapeQueryFilterData(PxFilterData* qryFilterData)
	{
		PX_CHECK_MSG(0==qryFilterData->word3, "word3 is reserved for filter data for vehicle raycast queries");
		qryFilterData->word3 = PX_DRIVABLE_SURFACE;
	}

	void PxSetupNonDrivableShapeQueryFilterData(PxFilterData* qryFilterData)
	{
		PX_CHECK_MSG(0==qryFilterData->word3, "word3 is reserved for filter data for vehicle raycast queries");
		qryFilterData->word3 = 0;
	}

	void PxSetupVehicleShapeQueryFilterData(const PxU32 vehIndex, PxFilterData* qryFilterData)
	{
		PX_CHECK_MSG(0==qryFilterData->word3, "word3 is reserved for filter data for vehicle raycast queries");
		PX_CHECK_MSG(0== (vehIndex & PX_DRIVABLE_SURFACE), "Max allowed vehIndex is 0xffff");
		PX_CHECK_MSG(vehIndex!=0, "vehIndex must not be zero because zero is already reserved");
		qryFilterData->word3 = vehIndex;
	}
}

#if PX_DEBUG_VEHICLE_ON

PxVehicleGraph::PxVehicleGraph()
{
	mBackgroundMinX=0;
	mBackgroundMaxX=0;
	mBackgroundMinY=0;
	mBackgroundMaxY=0;
	mSampleTide=0;
	mBackgroundColour=PxVec3(255,255,255),
		mBackgroundAlpha=1.0f;
	for(PxU32 i=0;i<eMAX_NUM_CHANNELS;i++)
	{
		mChannelMinY[i]=0;
		mChannelMaxY[i]=0;
		mChannelMidY[i]=0;
		mChannelColourLow[i]=PxVec3(0,0,255);
		mChannelColourHigh[i]=PxVec3(255,0,0);
		memset(mChannelSamples[i], 0, sizeof(PxReal)*eMAX_NUM_SAMPLES);
	}
	mNumChannels = 0;
	PX_COMPILE_TIME_ASSERT((size_t)eMAX_NUM_CHANNELS >= (size_t)eMAX_NUM_ENGINE_CHANNELS && (size_t)eMAX_NUM_CHANNELS >= (size_t)eMAX_NUM_WHEEL_CHANNELS);
}

PxVehicleGraph::~PxVehicleGraph()
{
}

void PxVehicleGraph::setup(const PxVehicleGraphDesc& desc, const eGraphType graphType)
{
	mBackgroundMinX = (desc.mPosX - 0.5f*desc.mSizeX);
	mBackgroundMaxX = (desc.mPosX + 0.5f*desc.mSizeX);
	mBackgroundMinY = (desc.mPosY - 0.5f*desc.mSizeY);
	mBackgroundMaxY = (desc.mPosY + 0.5f*desc.mSizeY);

	mBackgroundColour=desc.mBackgroundColour;
	mBackgroundAlpha=desc.mAlpha;

	mNumChannels = (eGRAPH_TYPE_WHEEL==graphType) ? (PxU32)eMAX_NUM_WHEEL_CHANNELS : (PxU32)eMAX_NUM_ENGINE_CHANNELS;
}

void PxVehicleGraph::setChannel(PxVehicleGraphChannelDesc& desc, const PxU32 channel)
{
	PX_ASSERT(channel<eMAX_NUM_CHANNELS);

	mChannelMinY[channel]=desc.mMinY;
	mChannelMaxY[channel]=desc.mMaxY;
	mChannelMidY[channel]=desc.mMidY;
	PX_CHECK_MSG(mChannelMinY[channel]<=mChannelMidY[channel], "mChannelMinY must be less than or equal to mChannelMidY");
	PX_CHECK_MSG(mChannelMidY[channel]<=mChannelMaxY[channel], "mChannelMidY must be less than or equal to mChannelMaxY");

	mChannelColourLow[channel]=desc.mColourLow;
	mChannelColourHigh[channel]=desc.mColourHigh;

	strcpy(mChannelTitle[channel], desc.mTitle);
}

void PxVehicleGraph::clearRecordedChannelData()
{
	mSampleTide=0;
	for(PxU32 i=0;i<eMAX_NUM_CHANNELS;i++)
	{
		memset(mChannelSamples[i], 0, sizeof(PxReal)*eMAX_NUM_SAMPLES);
	}
}

void PxVehicleGraph::updateTimeSlice(const PxReal* const samples)
{
	mSampleTide++;
	mSampleTide=mSampleTide%eMAX_NUM_SAMPLES;

	for(PxU32 i=0;i<mNumChannels;i++)
	{
		mChannelSamples[i][mSampleTide]=PxClamp(samples[i],mChannelMinY[i],mChannelMaxY[i]);
	}
}

void PxVehicleGraph::computeGraphChannel(const PxU32 channel, PxReal* xy, PxVec3* colours, char* title) const
{
	PX_ASSERT(channel<mNumChannels);
	const PxReal sizeX=mBackgroundMaxX-mBackgroundMinX;
	const PxReal sizeY=mBackgroundMaxY-mBackgroundMinY;

	for(PxU32 i=0;i<PxVehicleGraph::eMAX_NUM_SAMPLES;i++)
	{
		const PxU32 index=(mSampleTide+1+i)%PxVehicleGraph::eMAX_NUM_SAMPLES;
		xy[2*i+0]=mBackgroundMinX+sizeX*i/((PxReal)(PxVehicleGraph::eMAX_NUM_SAMPLES));
		const PxReal y=(mChannelSamples[channel][index]-mChannelMinY[channel])/(mChannelMaxY[channel]-mChannelMinY[channel]);		
		xy[2*i+1]=mBackgroundMinY+sizeY*y;
		colours[i]=mChannelSamples[channel][index]<mChannelMidY[channel] ? mChannelColourLow[channel] : mChannelColourHigh[channel];
	}

	strcpy(title,mChannelTitle[channel]);
}

void PxSetupVehicleEngineGraph
(const PxF32 sizeX, const PxF32 sizeY, const PxF32 posX, const PxF32 posY, 
 const PxVec3& backgoundColour, const PxVec3& lineColourHigh, const PxVec3& lineColourLow, 
 PxVehicleGraph& engineGraph)
{
	PxVehicleGraphDesc desc;
	desc.mSizeX=sizeX;
	desc.mSizeY=sizeY;
	desc.mPosX=posX;
	desc.mPosY=posY;
	desc.mBackgroundColour=backgoundColour;
	desc.mAlpha=0.5f;
	engineGraph.setup(desc,PxVehicleGraph::eGRAPH_TYPE_ENGINE);

	//Engine revs
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColourHigh=lineColourHigh;
		desc2.mColourLow=lineColourLow;
		desc2.mMinY=0.0f;
		desc2.mMaxY=800.0f;
		desc2.mMidY=400.0f;
		char title[64];
		sprintf(title, "engineRevs");
		desc2.mTitle=title;
		engineGraph.setChannel(desc2,PxVehicleGraph::eCHANNEL_ENGINE_REVS);
	}

	//Engine torque
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColourHigh=lineColourHigh;
		desc2.mColourLow=lineColourLow;
		desc2.mMinY=0.0f;
		desc2.mMaxY=1000.0f;
		desc2.mMidY=0.0f;
		char title[64];
		sprintf(title, "engineDriveTorque");
		desc2.mTitle=title;
		engineGraph.setChannel(desc2,PxVehicleGraph::eCHANNEL_ENGINE_DRIVE_TORQUE);
	}

	//Clutch slip
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColourHigh=lineColourHigh;
		desc2.mColourLow=lineColourLow;
		desc2.mMinY=-200.0f;
		desc2.mMaxY=200.0f;
		desc2.mMidY=0.0f;
		char title[64];
		sprintf(title, "clutchSlip");
		desc2.mTitle=title;
		engineGraph.setChannel(desc2,PxVehicleGraph::eCHANNEL_CLUTCH_SLIP);
	}

	//Accel control
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColourHigh=lineColourHigh;
		desc2.mColourLow=lineColourLow;
		desc2.mMinY=0.0f;
		desc2.mMaxY=1.1f;
		desc2.mMidY=0.0f;
		char title[64];
		sprintf(title, "accel");
		desc2.mTitle=title;
		engineGraph.setChannel(desc2,PxVehicleGraph::eCHANNEL_ACCEL_CONTROL);
	}

	//Brake control
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColourHigh=lineColourHigh;
		desc2.mColourLow=lineColourLow;
		desc2.mMinY=0.0f;
		desc2.mMaxY=1.1f;
		desc2.mMidY=0.0f;
		char title[64];
		sprintf(title, "brake");
		desc2.mTitle=title;
		engineGraph.setChannel(desc2,PxVehicleGraph::eCHANNEL_BRAKE_CONTROL);
	}

	//HandBrake control
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColourHigh=lineColourHigh;
		desc2.mColourLow=lineColourLow;
		desc2.mMinY=0.0f;
		desc2.mMaxY=1.1f;
		desc2.mMidY=0.0f;
		char title[64];
		sprintf(title, "handbrake");
		desc2.mTitle=title;
		engineGraph.setChannel(desc2,PxVehicleGraph::eCHANNEL_HANDBRAKE_CONTROL);
	}

	//Steer control
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColourHigh=lineColourHigh;
		desc2.mColourLow=lineColourLow;
		desc2.mMinY=-1.1f;
		desc2.mMaxY=1.1f;
		desc2.mMidY=0.0f;
		char title[64];
		sprintf(title, "steer");
		desc2.mTitle=title;
		engineGraph.setChannel(desc2,PxVehicleGraph::eCHANNEL_STEER_CONTROL);
	}

	//Gear
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColourHigh=lineColourHigh;
		desc2.mColourLow=lineColourLow;
		desc2.mMinY=-4.f;
		desc2.mMaxY=20.f;
		desc2.mMidY=0.0f;
		char title[64];
		sprintf(title, "gearRatio");
		desc2.mTitle=title;
		engineGraph.setChannel(desc2,PxVehicleGraph::eCHANNEL_GEAR_RATIO);
	}
}

void PxSetupVehicleWheelGraph
(const PxF32 sizeX, const PxF32 sizeY, const PxF32 posX, const PxF32 posY, 
 const PxVec3& backgoundColour, const PxVec3& lineColourHigh, const PxVec3& lineColourLow, 
 PxVehicleGraph& wheelGraph)
{
	PxVehicleGraphDesc desc;
	desc.mSizeX=sizeX;
	desc.mSizeY=sizeY;
	desc.mPosX=posX;
	desc.mPosY=posY;
	desc.mBackgroundColour=backgoundColour;
	desc.mAlpha=0.5f;
	wheelGraph.setup(desc,PxVehicleGraph::eGRAPH_TYPE_WHEEL);

	//Jounce data channel
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColourHigh=lineColourHigh;
		desc2.mColourLow=lineColourLow;
		desc2.mMinY=-0.2f;
		desc2.mMaxY=0.4f;
		desc2.mMidY=0.0f;
		char title[64];
		sprintf(title, "suspJounce");
		desc2.mTitle=title;
		wheelGraph.setChannel(desc2,PxVehicleGraph::eCHANNEL_JOUNCE);
	}

	//Jounce susp force channel
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColourHigh=lineColourHigh;
		desc2.mColourLow=lineColourLow;
		desc2.mMinY=0.0f;
		desc2.mMaxY=20000.0f;
		desc2.mMidY=0.0f;
		char title[64];
		sprintf(title, "suspForce");
		desc2.mTitle=title;
		wheelGraph.setChannel(desc2,PxVehicleGraph::eCHANNEL_SUSPFORCE);
	}

	//Tyre load channel.
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColourHigh=lineColourHigh;
		desc2.mColourLow=lineColourLow;
		desc2.mMinY=0.0f;
		desc2.mMaxY=20000.0f;
		desc2.mMidY=0.0f;
		char title[64];
		sprintf(title, "tyreLoad");
		desc2.mTitle=title;
		wheelGraph.setChannel(desc2,PxVehicleGraph::eCHANNEL_TYRELOAD);
	}

	//Normalised tyre load channel.
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColourHigh=lineColourHigh;
		desc2.mColourLow=lineColourLow;
		desc2.mMinY=0.0f;
		desc2.mMaxY=3.0f;
		desc2.mMidY=1.0f;
		char title[64];
		sprintf(title, "normTyreLoad");
		desc2.mTitle=title;
		wheelGraph.setChannel(desc2,PxVehicleGraph::eCHANNEL_NORMALISED_TYRELOAD);
	}

	//Wheel omega channel
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColourHigh=lineColourHigh;
		desc2.mColourLow=lineColourLow;
		desc2.mMinY=-50.0f;
		desc2.mMaxY=250.0f;
		desc2.mMidY=0.0f;
		char title[64];
		sprintf(title, "wheelOmega");
		desc2.mTitle=title;
		wheelGraph.setChannel(desc2,PxVehicleGraph::eCHANNEL_WHEEL_OMEGA);
	}

	//Tyre friction
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColourHigh=lineColourHigh;
		desc2.mColourLow=lineColourLow;
		desc2.mMinY=0.0f;
		desc2.mMaxY=1.1f;
		desc2.mMidY=1.0f;
		char title[64];
		sprintf(title, "friction");
		desc2.mTitle=title;
		wheelGraph.setChannel(desc2,PxVehicleGraph::eCHANNEL_TYRE_FRICTION);
	}


	//Tyre long slip
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColourHigh=lineColourHigh;
		desc2.mColourLow=lineColourLow;
		desc2.mMinY=-0.2f;
		desc2.mMaxY=0.2f;
		desc2.mMidY=0.0f;
		char title[64];
		sprintf(title, "tyreLongSlip");
		desc2.mTitle=title;
		wheelGraph.setChannel(desc2,PxVehicleGraph::eCHANNEL_TYRE_LONG_SLIP);
	}

	//Normalised tyre long force
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColourHigh=lineColourHigh;
		desc2.mColourLow=lineColourLow;
		desc2.mMinY=0.0f;
		desc2.mMaxY=2.0f;
		desc2.mMidY=1.0f;
		char title[64];
		sprintf(title, "normTyreLongForce");
		desc2.mTitle=title;
		wheelGraph.setChannel(desc2,PxVehicleGraph::eCHANNEL_NORM_TYRE_LONG_FORCE);
	}

	//Tyre lat slip
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColourHigh=lineColourHigh;
		desc2.mColourLow=lineColourLow;
		desc2.mMinY=-1.0f;
		desc2.mMaxY=1.0f;
		desc2.mMidY=0.0f;
		char title[64];
		sprintf(title, "tyreLatSlip");
		desc2.mTitle=title;
		wheelGraph.setChannel(desc2,PxVehicleGraph::eCHANNEL_TYRE_LAT_SLIP);
	}

	//Normalised tyre lat force
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColourHigh=lineColourHigh;
		desc2.mColourLow=lineColourLow;
		desc2.mMinY=0.0f;
		desc2.mMaxY=2.0f;
		desc2.mMidY=1.0f;
		char title[64];
		sprintf(title, "normTyreLatForce");
		desc2.mTitle=title;
		wheelGraph.setChannel(desc2,PxVehicleGraph::eCHANNEL_NORM_TYRE_LAT_FORCE);
	}

	//Normalised aligning moment
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColourHigh=lineColourHigh;
		desc2.mColourLow=lineColourLow;
		desc2.mMinY=0.0f;
		desc2.mMaxY=2.0f;
		desc2.mMidY=1.0f;
		char title[64];
		sprintf(title, "normTyreAlignMoment");
		desc2.mTitle=title;
		wheelGraph.setChannel(desc2,PxVehicleGraph::eCHANNEL_NORM_TYRE_ALIGNING_MOMENT);
	}
}

namespace physx
{
	void PxVehicle4WSetupTelemetryData(const PxF32 graphSizeX, const PxF32 graphSizeY,
		const PxF32 engineGraphPosX, const PxF32 engineGraphPosY,
		const PxF32* const wheelGraphPosX, const PxF32* const wheelGraphPosY,
		const PxVec3 backgroundColour, const PxVec3& lineColourHigh, const PxVec3& lineColourLow,
		PxVehicle4WTelemetryData& telemetryData);
}

void physx::PxVehicle4WSetupTelemetryData
(const PxF32 graphSizeX, const PxF32 graphSizeY,
 const PxF32 engineGraphPosX, const PxF32 engineGraphPosY,
 const PxF32* const wheelGraphPosX, const PxF32* const wheelGraphPosY,
 const PxVec3 backgroundColour, const PxVec3& lineColourHigh, const PxVec3& lineColourLow,
 PxVehicle4WTelemetryData& telemetryData)
{
	PxSetupVehicleEngineGraph
		(graphSizeX, graphSizeY, engineGraphPosX, engineGraphPosY, 
		 backgroundColour, lineColourHigh, lineColourLow, 
		 telemetryData.mEngineGraph);

	for(PxU32 i=0;i<PxVehicle4WSimulationData::eNUM_WHEELS;i++)
	{
		PxSetupVehicleWheelGraph
			(graphSizeX, graphSizeY, wheelGraphPosX[i], wheelGraphPosY[i], 
			 backgroundColour, lineColourHigh, lineColourLow, 
			 telemetryData.mWheelGraphs[i]);

		telemetryData.mTyreforceAppPoints[i]=PxVec3(0,0,0);
		telemetryData.mSuspforceAppPoints[i]=PxVec3(0,0,0);
	}
}

namespace physx
{
	void PxVehicle4WClearTelemetryData(PxVehicle4WTelemetryData& telemetryData);
}

void physx::PxVehicle4WClearTelemetryData(PxVehicle4WTelemetryData& telemetryData)
{
	telemetryData.mEngineGraph.clearRecordedChannelData();
	for(PxU32 i=0;i<PxVehicle4WSimulationData::eNUM_WHEELS;i++)
	{
		telemetryData.mWheelGraphs[i].clearRecordedChannelData();
		telemetryData.mTyreforceAppPoints[i]=PxVec3(0,0,0);
		telemetryData.mSuspforceAppPoints[i]=PxVec3(0,0,0);
	}
}

#endif


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
#include "PxVehicleUpdate.h"
#include "PxVehicle.h"
#include "PxVehicleUtils.h"
#include "PxQuat.h"
#include "PxRigidDynamic.h"
#include "PxHeightField.h"
#include "PxTriangleMesh.h"
#include "PxHeightFieldGeometry.h"
#include "PxTriangleMeshGeometry.h"
#include "PxConstraint.h"
#include "PxMaterial.h"
#include "PxRigidBodyExt.h"
#include "PsFoundation.h"
#include "PsUtilities.h"

using namespace physx;

//TODO: lsd - handle case where wheels are spinning in different directions.
//TODO: ackermann - use faster approximate functions for PxTan/PxATan because the results don't need to be too accurate here.
//TODO: tyre lat slip - do we really use PxAbs(vz) as denominator, that's not in the paper?
//TODO: tyre camber angle and camber vs jounce table
//TODO: toe vs jounce table.
//TODO: pneumatic trail.
//TODO: maybe just set the car wheel omega to the blended value?
//TODO: multi-stepping only needs a single pass at working out the jounce.
//TODO: we probably need to have a graphics jounce and a physics jounce and 
//TODO: expose sticky friction values in api.
//TODO: blend the graphics jounce towards the physics jounce to avoid graphical pops at kerbs etc.

namespace physx
{

const PxF32 gThresholdForwardSpeedForWheelAngleIntegration=5.0f;
const PxF32 gRecipThresholdForwardSpeedForWheelAngleIntegration=1.0f/gThresholdForwardSpeedForWheelAngleIntegration;
 
const PxF32 gMinSpeedForTyreModel=1.0f;
const PxF32 gRecipMinSpeedForTyreModel=1.0f/gMinSpeedForTyreModel;

//const PxF32 gMinSpeedForReverseGearAutoBoxPassiveToggle=0.1f;
//const PxF32 gMinSpeedForReverseGearAutoBoxActiveToggle=5.0f;

const PxF32 gStickyTyreFrictionThresholdSpeed=0.2f;
const PxF32 gStickyTyreFrictionDamping=0.01f;
const PxF32 gLowForwardSpeedThresholdTime=1.0f;

#if PX_DEBUG_VEHICLE_ON

//Render data.
PxVec3* gCarTyreForceAppPoints=NULL;
PxVec3* gCarSuspForceAppPoints=NULL;

//Graph data
PxF32* gCarWheelGraphData[PxVehicle4WSimulationData::eNUM_WHEELS]={NULL,NULL,NULL,NULL};
PxF32* gCarEngineGraphData=NULL;

PX_FORCE_INLINE void updateGraphDataInternalDynamics(const PxF32* carInternalDynamics)
{
	if(gCarWheelGraphData[0])
	{
		//Grab the internal rotation speeds for graphing before we update them.
		for(PxU32 j=0;j<PxVehicle4WSimulationData::eNUM_WHEELS;j++)
		{
			gCarWheelGraphData[j][PxVehicleGraph::eCHANNEL_WHEEL_OMEGA]=carInternalDynamics[j];
		}
	}
	if(gCarEngineGraphData)
		gCarEngineGraphData[PxVehicleGraph::eCHANNEL_ENGINE_REVS]=carInternalDynamics[PxVehicle4WSimulationData::eNUM_WHEELS];
}

PX_FORCE_INLINE void updateGraphDataControlInputs(const PxF32 accel, const PxF32 brake, const PxF32 handbrake, const PxF32 steer)
{
	if(gCarEngineGraphData)
	{
		gCarEngineGraphData[PxVehicleGraph::eCHANNEL_ACCEL_CONTROL]=accel;
		gCarEngineGraphData[PxVehicleGraph::eCHANNEL_BRAKE_CONTROL]=brake;
		gCarEngineGraphData[PxVehicleGraph::eCHANNEL_HANDBRAKE_CONTROL]=handbrake;
		gCarEngineGraphData[PxVehicleGraph::eCHANNEL_STEER_CONTROL]=steer;
	}
}
PX_FORCE_INLINE void updateGraphDataGearRatio(const PxF32 G)
{
	if(gCarEngineGraphData)
		gCarEngineGraphData[PxVehicleGraph::eCHANNEL_GEAR_RATIO]=G;
}
PX_FORCE_INLINE void updateGraphDataEngineDriveTorque(const PxF32 engineDriveTorque)
{
	if(gCarEngineGraphData)
		gCarEngineGraphData[PxVehicleGraph::eCHANNEL_ENGINE_DRIVE_TORQUE]=engineDriveTorque;
}
PX_FORCE_INLINE void updateGraphDataClutchSlip(const PxF32* carInternalDynamics, const PxF32* aveWheelSpeedContributions, const PxF32 G)
{
	if(gCarEngineGraphData)
	{
		PxF32 averageWheelSpeed=0;
		for(PxU32 i=0;i<PxVehicle4WSimulationData::eNUM_WHEELS;i++)
		{
			averageWheelSpeed+=carInternalDynamics[i]*aveWheelSpeedContributions[i];
		}
		averageWheelSpeed*=G;
		gCarEngineGraphData[PxVehicleGraph::eCHANNEL_CLUTCH_SLIP]=averageWheelSpeed-carInternalDynamics[PxVehicle4WSimulationData::eNUM_WHEELS];
	}
}

PX_FORCE_INLINE void zeroGraphDataWheels(const PxU32 type)
{
	if(gCarWheelGraphData[0])
	{
		for(PxU32 i=0;i<PxVehicle4WSimulationData::eNUM_WHEELS;i++)
		{
			gCarWheelGraphData[i][type]=0.0f;
		}
	}
}
PX_FORCE_INLINE void updateGraphDataSuspJounce(const PxU32 wheel, const PxF32 jounce)
{
	if(gCarWheelGraphData[0])
		gCarWheelGraphData[wheel][PxVehicleGraph::eCHANNEL_JOUNCE]=jounce;
}
PX_FORCE_INLINE void updateGraphDataSuspForce(const PxU32 wheel, const PxF32 springForce)
{
	if(gCarWheelGraphData[0])
		gCarWheelGraphData[wheel][PxVehicleGraph::eCHANNEL_SUSPFORCE]=springForce;
}
PX_FORCE_INLINE void updateGraphDataTyreLoad(const PxU32 wheel, const PxF32 filteredTyreLoad)
{
	if(gCarWheelGraphData[0])
		gCarWheelGraphData[wheel][PxVehicleGraph::eCHANNEL_TYRELOAD]=filteredTyreLoad;
}
PX_FORCE_INLINE void updateGraphDataNormTyreLoad(const PxU32 wheel, const PxF32 filteredNormalisedTyreLoad)
{
	if(gCarWheelGraphData[0])
		gCarWheelGraphData[wheel][PxVehicleGraph::eCHANNEL_NORMALISED_TYRELOAD]=filteredNormalisedTyreLoad;
}
PX_FORCE_INLINE void updateGraphDataNormLongTyreForce(const PxU32 wheel, const PxF32 normForce)
{
	if(gCarWheelGraphData[0])
		gCarWheelGraphData[wheel][PxVehicleGraph::eCHANNEL_NORM_TYRE_LONG_FORCE]=normForce;
}
PX_FORCE_INLINE void updateGraphDataNormLatTyreForce(const PxU32 wheel, const PxF32 normForce)
{
	if(gCarWheelGraphData[0])
		gCarWheelGraphData[wheel][PxVehicleGraph::eCHANNEL_NORM_TYRE_LAT_FORCE]=normForce;
}
PX_FORCE_INLINE void updateGraphDataLatTyreSlip(const PxU32 wheel, const PxF32 latSlip)
{
	if(gCarWheelGraphData[0])
		gCarWheelGraphData[wheel][PxVehicleGraph::eCHANNEL_TYRE_LAT_SLIP]=latSlip;
}
PX_FORCE_INLINE void updateGraphDataLongTyreSlip(const PxU32 wheel, const PxF32 longSlip)
{
	if(gCarWheelGraphData[0])
		gCarWheelGraphData[wheel][PxVehicleGraph::eCHANNEL_TYRE_LONG_SLIP]=longSlip;
}
PX_FORCE_INLINE void updateGraphDataTyreFriction(const PxU32 wheel, const PxF32 friction)
{
	if(gCarWheelGraphData[0])
		gCarWheelGraphData[wheel][PxVehicleGraph::eCHANNEL_TYRE_FRICTION]=friction;
}
PX_FORCE_INLINE void updateGraphDataNormTyreAligningMoment(const PxU32 wheel, const PxF32 normAlignMoment)
{
	if(gCarWheelGraphData[0])
		gCarWheelGraphData[wheel][PxVehicleGraph::eCHANNEL_NORM_TYRE_ALIGNING_MOMENT]=normAlignMoment;
}

#endif //DEBUG_VEHICLE_ON

PX_FORCE_INLINE void processAutoBox(const PxF32 timestep, PxVehicle4W& car)
{
	PX_ASSERT(car.mUseAutoGears);

	//If still undergoing a gear change triggered by the autobox 
	//then turn off the accelerator pedal.  This happens in autoboxes
	//to stop the driver revving the engine crazily then damaging the 
	//clutch when the clutch re-engages at the end of the gear change.
	const PxU32 currentGear=car.mCurrentGear;
	const PxU32 targetGear=car.mTargetGear;
	if(targetGear!=currentGear && PxVehicleGearsData::eNEUTRAL==currentGear)
	{
		car.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_ACCEL]=0.0f;
	}

	//Only process the autobox if no gear change is underway and the time passed since the last autobox
	//gear change is greater than the autobox latency.
	PxF32 autoBoxSwitchTime=car.mAutoBoxSwitchTime;
	const PxF32 autoBoxLatencyTime=car.mVehicleSimData.getAutoBoxData().mDownRatios[PxVehicleGearsData::eREVERSE];
	if(targetGear==currentGear && autoBoxSwitchTime > autoBoxLatencyTime)
	{
		//Work out if the autobox wants to switch up or down.
		const PxF32 normalisedEngineOmega=car.mInternalDynamics[PxVehicle4WSimulationData::eNUM_WHEELS]*car.mVehicleSimData.getEngineData().getRecipMaxOmega();
		const PxVehicleAutoBoxData& autoBoxData=car.mVehicleSimData.getAutoBoxData();

		bool gearUp=false;
		if(normalisedEngineOmega > autoBoxData.mUpRatios[currentGear] && PxVehicleGearsData::eREVERSE!=currentGear)
		{
			//If revs too high and not in reverse and not undergoing a gear change then switch up. 
			gearUp=true;
		}

		bool gearDown=false;
		if(normalisedEngineOmega < autoBoxData.mDownRatios[currentGear] && currentGear > PxVehicleGearsData::eFIRST)
		{
			//If revs too low and in gear greater than first and not undergoing a gear change then change down.
			gearDown=true;
		}

		//Start the gear change and reset the time since the last autobox gear change.
		if(gearUp || gearDown)
		{
			car.mGearUpPressed=gearUp;
			car.mGearDownPressed=gearDown;
			car.mAutoBoxSwitchTime=0;
		}
	}
	else
	{
		autoBoxSwitchTime+=timestep;
		car.mAutoBoxSwitchTime=autoBoxSwitchTime;
	}
}

void processGears(const PxF32 timestep, PxVehicle4W& car)
{
	const PxVehicleGearsData& gears=car.mVehicleSimData.getGearsData();

	//Process the gears.
	if(car.mGearUpPressed && gears.mNumRatios-1!=car.mCurrentGear && car.mCurrentGear==car.mTargetGear)
	{
		//Car wants to go up a gear and can go up a gear and not already undergoing a gear change.
		if(PxVehicleGearsData::eREVERSE==car.mCurrentGear)
		{
			//In reverse so switch to first through neutral.
			car.mGearSwitchTime=0;
			car.mTargetGear=PxVehicleGearsData::eFIRST;
			car.mCurrentGear=PxVehicleGearsData::eNEUTRAL;
		}
		else if(PxVehicleGearsData::eNEUTRAL==car.mCurrentGear)
		{
			//In neutral so switch to first and stay in neutral.
			car.mGearSwitchTime=0;
			car.mTargetGear=PxVehicleGearsData::eFIRST;
			car.mCurrentGear=PxVehicleGearsData::eNEUTRAL;
		}
		else
		{
			//Switch up a gear through neutral.
			car.mGearSwitchTime=0;
			car.mTargetGear=car.mCurrentGear+1;
			car.mCurrentGear=PxVehicleGearsData::eNEUTRAL;
		}
	}
	if(car.mGearDownPressed && PxVehicleGearsData::eREVERSE!=car.mCurrentGear && car.mCurrentGear==car.mTargetGear)
	{
		//Car wants to go down a gear and can go down a gear and not already undergoing a gear change
		if(PxVehicleGearsData::eFIRST==car.mCurrentGear)
		{
			//In first so switch to reverse through neutral.
			car.mGearSwitchTime=0;
			car.mTargetGear=PxVehicleGearsData::eREVERSE;
			car.mCurrentGear=PxVehicleGearsData::eNEUTRAL;
		}
		else if(PxVehicleGearsData::eNEUTRAL==car.mCurrentGear)
		{
			//In neutral so switch to reverse and stay in neutral.
			car.mGearSwitchTime=0;
			car.mTargetGear=PxVehicleGearsData::eREVERSE;
			car.mCurrentGear=PxVehicleGearsData::eNEUTRAL;
		}
		else
		{
			//Switch down a gear through neutral.
			car.mGearSwitchTime=0;
			car.mTargetGear=car.mCurrentGear-1;			
			car.mCurrentGear=PxVehicleGearsData::eNEUTRAL;
		}
	}
	if(car.mCurrentGear!=car.mTargetGear)
	{
		if(car.mGearSwitchTime>gears.mSwitchTime)
		{
			car.mCurrentGear=car.mTargetGear;
			car.mGearSwitchTime=0;
		}
		else
		{
			car.mGearSwitchTime+=timestep;
		}
	}
}

PX_FORCE_INLINE PxF32 computeSign(const PxF32 f)
{
	return physx::intrinsics::fsel(f, physx::intrinsics::fsel(-f, 0.0f, 1.0f), -1.0f); 
}

PX_FORCE_INLINE PxF32 computeGearRatio(const PxVehicleGearsData& gearsData, const PxU32 currentGear)
{
	const PxF32 gearRatio=gearsData.mRatios[currentGear]*gearsData.mFinalRatio;
	return gearRatio;
}

PX_FORCE_INLINE PxF32 computeEngineDriveTorque(const PxVehicleEngineData& engineData, const PxF32 omega, const PxF32 accel)
{
	const PxF32 engineDriveTorque=accel*engineData.mPeakTorque*engineData.mTorqueCurve.getYVal(omega*engineData.getRecipMaxOmega());
	return engineDriveTorque;
}

PX_FORCE_INLINE PxF32 computeEngineDampingRate(const PxVehicleEngineData& engineData, const PxF32 accel)
{
	//Different damping rates when accel pedal is engaged.
	const PxF32 engineDamping=
		engineData.mDisengagedClutchDampingRate
		+(engineData.mEngagedClutchDampingRate-engineData.mDisengagedClutchDampingRate)*accel;
	return engineDamping;
}

PX_FORCE_INLINE void splitTorque
(const PxF32 w1, const PxF32 w2, const PxF32 diffBias, const PxF32 defaultSplitRatio,
 PxF32* t1, PxF32* t2)
{
	PX_ASSERT(computeSign(w1)==computeSign(w2) && 0.0f!=computeSign(w1));
	const PxF32 w1Abs=PxAbs(w1);
	const PxF32 w2Abs=PxAbs(w2);
	const PxF32 omegaMax=PxMax(w1Abs,w2Abs);
	const PxF32 omegaMin=PxMin(w1Abs,w2Abs);
	const PxF32 delta=omegaMax-diffBias*omegaMin;
	const PxF32 deltaTorque=physx::intrinsics::fsel(delta, delta/omegaMax , 0.0f);
	*t1=physx::intrinsics::fsel(w1Abs-w2Abs, defaultSplitRatio*(1.0f-deltaTorque), defaultSplitRatio*(1.0f+deltaTorque));
	*t2=physx::intrinsics::fsel(w1Abs-w2Abs, (1.0f-defaultSplitRatio)*(1.0f+deltaTorque), (1.0f-defaultSplitRatio)*(1.0f-deltaTorque));
}

PX_FORCE_INLINE void computeDiffTorqueRatios
(const PxVehicleDifferential4WData& diffData, const PxF32 handbrake, const PxF32* PX_RESTRICT wheelOmegas, PxF32* PX_RESTRICT diffTorqueRatios)
{
	//If the handbrake is on only deliver torque to the front wheels.
	PxU32 type=diffData.mType;
	if(handbrake>0)
	{
		switch(diffData.mType)
		{
		case PxVehicleDifferential4WData::eDIFF_TYPE_LS_4WD:
			type=PxVehicleDifferential4WData::eDIFF_TYPE_LS_FRONTWD;
			break;
		case PxVehicleDifferential4WData::eDIFF_TYPE_OPEN_4WD:
			type=PxVehicleDifferential4WData::eDIFF_TYPE_OPEN_FRONTWD;
			break;
		default:
			break;
		}
	}

	//Split a torque of 1 between front and rear.
	//Then split that torque between left and right.
	PxF32 torqueFrontLeft=0;
	PxF32 torqueFrontRight=0;
	PxF32 torqueRearLeft=0;
	PxF32 torqueRearRight=0;
	switch(type)
	{
	case PxVehicleDifferential4WData::eDIFF_TYPE_LS_4WD:
		if(0.0f!=computeSign(wheelOmegas[PxVehicle4WSimulationData::eFRONT_LEFT_WHEEL]) && 
		   computeSign(wheelOmegas[PxVehicle4WSimulationData::eFRONT_LEFT_WHEEL])==computeSign(wheelOmegas[PxVehicle4WSimulationData::eFRONT_RIGHT_WHEEL]) &&
 		   computeSign(wheelOmegas[PxVehicle4WSimulationData::eFRONT_LEFT_WHEEL])==computeSign(wheelOmegas[PxVehicle4WSimulationData::eREAR_LEFT_WHEEL]) &&
		   computeSign(wheelOmegas[PxVehicle4WSimulationData::eFRONT_LEFT_WHEEL])==computeSign(wheelOmegas[PxVehicle4WSimulationData::eREAR_RIGHT_WHEEL]))
		{
			PxF32 torqueFront,torqueRear;
			const PxF32 omegaFront=PxAbs(wheelOmegas[PxVehicle4WSimulationData::eFRONT_LEFT_WHEEL]+wheelOmegas[PxVehicle4WSimulationData::eFRONT_RIGHT_WHEEL]);
			const PxF32 omegaRear=PxAbs(wheelOmegas[PxVehicle4WSimulationData::eREAR_LEFT_WHEEL]+wheelOmegas[PxVehicle4WSimulationData::eREAR_RIGHT_WHEEL]);
			splitTorque(omegaFront,omegaRear,diffData.mCentreBias,diffData.mFrontRearSplit,&torqueFront,&torqueRear);
			splitTorque(wheelOmegas[PxVehicle4WSimulationData::eFRONT_LEFT_WHEEL],wheelOmegas[PxVehicle4WSimulationData::eFRONT_RIGHT_WHEEL],diffData.mFrontBias,0.5f,&torqueFrontLeft,&torqueFrontRight);
			splitTorque(wheelOmegas[PxVehicle4WSimulationData::eREAR_LEFT_WHEEL],wheelOmegas[PxVehicle4WSimulationData::eREAR_RIGHT_WHEEL],diffData.mFrontBias,0.5f,&torqueRearLeft,&torqueRearRight);
		}
		else
		{
			//TODO: need to handle this case.
			torqueFrontLeft=0.25f;
			torqueFrontRight=0.25f;
			torqueRearLeft=0.25f;
			torqueRearRight=0.25f;
		}
		break;

	case PxVehicleDifferential4WData::eDIFF_TYPE_LS_FRONTWD:
		if(0.0f!=computeSign(wheelOmegas[PxVehicle4WSimulationData::eFRONT_LEFT_WHEEL]) && 
			computeSign(wheelOmegas[PxVehicle4WSimulationData::eFRONT_LEFT_WHEEL])==computeSign(wheelOmegas[PxVehicle4WSimulationData::eFRONT_RIGHT_WHEEL]))
		{
			splitTorque(wheelOmegas[PxVehicle4WSimulationData::eFRONT_LEFT_WHEEL],wheelOmegas[PxVehicle4WSimulationData::eFRONT_RIGHT_WHEEL],diffData.mFrontBias,0.5f,&torqueFrontLeft,&torqueFrontRight);
		}
		else
		{
			torqueFrontLeft=0.5f;
			torqueFrontRight=0.5f;
		}
		break;

	case PxVehicleDifferential4WData::eDIFF_TYPE_LS_REARWD:

		if(0.0f!=computeSign(wheelOmegas[PxVehicle4WSimulationData::eREAR_LEFT_WHEEL]) && 
			computeSign(wheelOmegas[PxVehicle4WSimulationData::eREAR_LEFT_WHEEL])==computeSign(wheelOmegas[PxVehicle4WSimulationData::eREAR_RIGHT_WHEEL]))
		{
			splitTorque(wheelOmegas[PxVehicle4WSimulationData::eREAR_LEFT_WHEEL],wheelOmegas[PxVehicle4WSimulationData::eREAR_RIGHT_WHEEL],diffData.mRearBias,0.5f,&torqueRearLeft,&torqueRearRight);
		}
		else
		{
			torqueRearLeft=0.5f;
			torqueRearRight=0.5f;
		}
		break;

	case PxVehicleDifferential4WData::eDIFF_TYPE_OPEN_4WD:
		torqueFrontLeft=diffData.mFrontRearSplit*0.5f;
		torqueFrontRight=torqueFrontLeft;
		torqueRearLeft=(1.0f-diffData.mFrontRearSplit)*0.5f;
		torqueRearRight=torqueRearLeft;
		break;

	case PxVehicleDifferential4WData::eDIFF_TYPE_OPEN_FRONTWD:
		torqueFrontLeft=0.5f;
		torqueFrontRight=0.5f;
		break;

	case PxVehicleDifferential4WData::eDIFF_TYPE_OPEN_REARWD:
		torqueRearLeft=0.5f;
		torqueRearRight=0.5f;
		break;

	default:
		PX_ASSERT(false);
		break;
	}

	diffTorqueRatios[PxVehicle4WSimulationData::eFRONT_LEFT_WHEEL]=torqueFrontLeft;
	diffTorqueRatios[PxVehicle4WSimulationData::eFRONT_RIGHT_WHEEL]=torqueFrontRight;
	diffTorqueRatios[PxVehicle4WSimulationData::eREAR_LEFT_WHEEL]=torqueRearLeft;
	diffTorqueRatios[PxVehicle4WSimulationData::eREAR_RIGHT_WHEEL]=torqueRearRight;
}

PX_FORCE_INLINE void computeDiffAveWheelSpeedContributions
(const PxVehicleDifferential4WData& diffData, const PxF32 handbrake, PxF32* PX_RESTRICT diffAveWheelSpeedContributions)
{
	PxU32 type=diffData.mType;

	//If the handbrake is on only deliver torque to the front wheels.
	if(handbrake>0)
	{
		switch(diffData.mType)
		{
		case PxVehicleDifferential4WData::eDIFF_TYPE_LS_4WD:
			type=PxVehicleDifferential4WData::eDIFF_TYPE_LS_FRONTWD;
			break;
		case PxVehicleDifferential4WData::eDIFF_TYPE_OPEN_4WD:
			type=PxVehicleDifferential4WData::eDIFF_TYPE_OPEN_FRONTWD;
			break;
		default:
			break;
		}
	}

	switch(type)
	{
	case PxVehicleDifferential4WData::eDIFF_TYPE_LS_4WD:
	case PxVehicleDifferential4WData::eDIFF_TYPE_OPEN_4WD:
		diffAveWheelSpeedContributions[PxVehicle4WSimulationData::eFRONT_LEFT_WHEEL]=0.25f;
		diffAveWheelSpeedContributions[PxVehicle4WSimulationData::eFRONT_RIGHT_WHEEL]=0.25f;
		diffAveWheelSpeedContributions[PxVehicle4WSimulationData::eREAR_LEFT_WHEEL]=0.25f;
		diffAveWheelSpeedContributions[PxVehicle4WSimulationData::eREAR_RIGHT_WHEEL]=0.25f;
		break;
	case PxVehicleDifferential4WData::eDIFF_TYPE_LS_FRONTWD:
	case PxVehicleDifferential4WData::eDIFF_TYPE_OPEN_FRONTWD:
		diffAveWheelSpeedContributions[PxVehicle4WSimulationData::eFRONT_LEFT_WHEEL]=0.5f;
		diffAveWheelSpeedContributions[PxVehicle4WSimulationData::eFRONT_RIGHT_WHEEL]=0.5f;
		diffAveWheelSpeedContributions[PxVehicle4WSimulationData::eREAR_LEFT_WHEEL]=0.0f;
		diffAveWheelSpeedContributions[PxVehicle4WSimulationData::eREAR_RIGHT_WHEEL]=0.0f;
		break;
	case PxVehicleDifferential4WData::eDIFF_TYPE_LS_REARWD:
	case PxVehicleDifferential4WData::eDIFF_TYPE_OPEN_REARWD:
		diffAveWheelSpeedContributions[PxVehicle4WSimulationData::eFRONT_LEFT_WHEEL]=0.0f;
		diffAveWheelSpeedContributions[PxVehicle4WSimulationData::eFRONT_RIGHT_WHEEL]=0.0f;
		diffAveWheelSpeedContributions[PxVehicle4WSimulationData::eREAR_LEFT_WHEEL]=0.5f;
		diffAveWheelSpeedContributions[PxVehicle4WSimulationData::eREAR_RIGHT_WHEEL]=0.5f;
		break;
	default:
		PX_ASSERT(false);
		break;
	}

}

PX_FORCE_INLINE void computeBrakeAndHandBrakeTorques
(const PxVehicleWheelData* PX_RESTRICT wheelDatas, const PxF32* PX_RESTRICT wheelOmegas, const PxF32 brake, const PxF32 handbrake, 
 PxF32* PX_RESTRICT brakeTorques, PxF32* brakeTorquesForTyreLongSlip)
{
	for(PxU32 i=0;i<PxVehicle4WSimulationData::eNUM_WHEELS;i++)
	{
		//At zero speed offer no brake torque allowed.
		const PxF32 sign=computeSign(wheelOmegas[i]); 
		brakeTorques[i]=(-brake*sign*wheelDatas[i].mMaxBrakeTorque-handbrake*sign*wheelDatas[i].mMaxHandBrakeTorque);
		brakeTorquesForTyreLongSlip[i]=(brake*wheelDatas[i].mMaxBrakeTorque+handbrake*wheelDatas[i].mMaxHandBrakeTorque);
	}
}

PX_FORCE_INLINE PxF32 computeClutchStrength(const PxVehicleClutchData& clutchData, const PxU32 currentGear)
{
	return ((PxVehicleGearsData::eNEUTRAL!=currentGear) ? clutchData.mStrength : 0.0f);
}

PX_FORCE_INLINE PxF32 computeFilteredNormalisedTyreLoad(const PxVehicleTyreLoadFilterData& filterData, const PxF32 normalisedLoad)
{
	return physx::intrinsics::fsel(filterData.mMinNormalisedLoad-normalisedLoad, 
		0.0f, 
		physx::intrinsics::fsel(normalisedLoad-filterData.mMaxNormalisedLoad, 
		filterData.mMaxNormalisedLoad, 
		filterData.mMaxFilteredNormalisedLoad*(normalisedLoad-filterData.mMinNormalisedLoad)*filterData.getDenominator()));
}

PX_FORCE_INLINE void computeAckermannSteerAngles
(const PxF32 steer, const PxF32 steerGain, const PxVehicleAckermannGeometryData& ackermannGeometry,  
 PxF32* PX_RESTRICT frontLeftSteerAngle, PxF32* PX_RESTRICT frontRightSteerAngle)
{
	PX_ASSERT(steer>=-1.01f && steer<=1.01f);
	PX_ASSERT(steerGain<PxPi);

	const PxF32 steerAngle=steer*steerGain;

	if(0==steerAngle)
	{
		*frontLeftSteerAngle=0;
		*frontRightSteerAngle=0;
		return;
	}

	//Work out the ackermann steer for +ve steer then swap and negate the steer angles if the steer is -ve.
	//TODO: use faster approximate functions for PxTan/PxATan because the results don't need to be too accurate here.
	const PxF32 rightSteerAngle=PxAbs(steerAngle);
	const PxF32 dz=ackermannGeometry.getAxleSeparation();
	const PxF32 dx=ackermannGeometry.getFrontWidth() + dz/PxTan(rightSteerAngle);
	const PxF32 leftSteerAnglePerfect=PxAtan(dz/dx);
	//PX_ASSERT(leftSteerAnglePerfect<=rightSteerAngle);
	const PxF32 leftSteerAngle=rightSteerAngle + ackermannGeometry.mAccuracy*(leftSteerAnglePerfect-rightSteerAngle);
	*frontRightSteerAngle=physx::intrinsics::fsel(steerAngle, rightSteerAngle, -leftSteerAngle);
	*frontLeftSteerAngle=physx::intrinsics::fsel(steerAngle, leftSteerAngle, -rightSteerAngle);
}

#define ONE_TWENTYSEVENTH 0.037037f
#define ONE_THIRD 0.33333f
PX_FORCE_INLINE PxF32 smoothingFunction1(const PxF32 K)
{
	//Equation 20 in CarSimEd manual Appendix F.
	//Looks a bit like a curve of sqrt(x) for 0<x<1 but reaching 1.0 on y-axis at K=3. 
	PX_ASSERT(K>=0.0f);
	return PxMin(1.0f, K - ONE_THIRD*K*K + ONE_TWENTYSEVENTH*K*K*K);
}
PX_FORCE_INLINE PxF32 smoothingFunction2(const PxF32 K)
{
	//Equation 21 in CarSimEd manual Appendix F.
	//Rises to a peak at K=0.75 and falls back to zero by K=3
	PX_ASSERT(K>=0.0f);
	return (K - K*K + ONE_THIRD*K*K*K - ONE_TWENTYSEVENTH*K*K*K*K);
}

void computeTyreForce
(const TyreForceUpdateDescriptor& desc,
 bool* PX_RESTRICT stickyTyreActiveFlag, PxF32* PX_RESTRICT stickyTyreError, PxVec3* PX_RESTRICT stickyTyreDir,
 PxF32* lowForwardSpeedTimer,
 PxF32* PX_RESTRICT forwardSpeed, PxF32* PX_RESTRICT friction, PxF32* PX_RESTRICT slipLong, PxF32* PX_RESTRICT slipLat,
 PxF32* PX_RESTRICT wheelTorque, 
 PxF32* PX_RESTRICT tyreLongForceMag, PxVec3& tyreLongForceDir, PxF32* PX_RESTRICT tyreLatForceMag, PxVec3& tyreLatForceDir, PxF32* PX_RESTRICT tyreAlignMoment)
{
	PX_ASSERT(0==*wheelTorque);
	PX_ASSERT(0==*tyreLongForceMag);
	PX_ASSERT(0==*tyreLatForceMag);

	//Compute the tyre axes in the ground plane.
	const PxVec3 hitNorm=desc.mHitNorm;
	const PxVec3 chassisLatDir=desc.mChassisLatDir;
	PxVec3 tzRaw=chassisLatDir.cross(hitNorm);
	PxVec3 txRaw=hitNorm.cross(tzRaw);
	tzRaw.normalize();
	txRaw.normalize();
	//Rotate the tyre using the steer angle.
	const PxF32 wheelSteer=desc.mWheelSteer;
	const PxF32 cosWheelSteer=PxCos(wheelSteer);
	const PxF32 sinWheelSteer=PxSin(wheelSteer);
	const PxVec3 tz=tzRaw*cosWheelSteer + txRaw*sinWheelSteer;
	const PxVec3 tx=txRaw*cosWheelSteer - tzRaw*sinWheelSteer;
	tyreLongForceDir=tz;
	tyreLatForceDir=tx;

	//Compute the velocity along the tyre axes.
	const PxVec3 wheelBottomVel=desc.mWheelBottomVelocity;
	const PxF32 vz=wheelBottomVel.dot(tz);
	const PxF32 vx=wheelBottomVel.dot(tx);
	const PxF32 vzAbs=PxAbs(vz);
	*forwardSpeed=vz;

	//Get the wheel state.
	const PxF32 wheelOmega=desc.mWheelOmega;
	const PxF32 wheelRadius=desc.mWheelRadius;
	const PxF32 recipWheelRadius=desc.mRecipWheelRadius;

	//Get the tyre data.
	const PxVehicleTyreData& tyreData=desc.mTyreData;

	//Get the normalised tyre load.
	const PxF32 normalisedTyreLoad=desc.mNormalisedTyreLoad;

	//Get the accel input.
	const PxF32 accel=desc.mAccel;

	const PxF32 tyreLoad=desc.mTyreLoad;
	if(tyreLoad<=0)
	{
		//Do nothing (except compute the forward speed because we need this for wheel rotation angle integration).
		return;
	}


	//Compute the lateral slip and stiffness
	PxF32 latSlip;
	PxF32 latStiff;
	{
		//Slip.
		latSlip = PxAtan(vx/(vzAbs+gMinSpeedForTyreModel));//TODO: do we really use PxAbs(vz) as denominator?
		//Stiffness.
		latStiff=desc.mRestTyreLoad*tyreData.mLatStiffY*smoothingFunction1(normalisedTyreLoad*3.0f/tyreData.mLatStiffX);
	}
	*slipLat=latSlip;

	//Compute the longitudinal slip and stiffness
	//Make sure that we never divide by zero.
	PxF32 longSlip;
	PxF32 longStiff;
	PxF32 recipLongStiff;
	PxF32 wheelLinSpeed;
	{
		wheelLinSpeed=wheelOmega*wheelRadius;
		if(desc.mBrakeTorque!=0.0f)
		{
			longSlip = (vzAbs >= PxAbs(wheelLinSpeed)) ? (wheelLinSpeed-vz)/(vzAbs + 1e-5f) : (wheelLinSpeed-vz)/PxAbs(wheelLinSpeed);
		}
		else
		{
			longSlip = (wheelLinSpeed - vz)/(vzAbs + gMinSpeedForTyreModel);
			//Smoothing - want a graph that is smoothly blends near 0 and 1.  This should do: (1-cos(theta))/2
			longSlip *= vzAbs<gMinSpeedForTyreModel ? 0.5f*(1.0f - 0.99f*PxCos(PxPi*vzAbs*gRecipMinSpeedForTyreModel)) : 1.0f;
		}
		//Stiffness.
		longStiff=tyreData.mLongitudinalStiffness;
		recipLongStiff=tyreData.getRecipLongitudinalStiffness();
	}
	*slipLong=longSlip;

	//TODO: add camber angle.
	//Set the camber angle to zero for the time being.
	const PxF32 camber=0;
	const PxF32 camberStiff=tyreData.mCamberStiffness;

	//If long slip/lat slip/camber are all zero than there will be zero tyre force.
	if((0==latSlip)&&(0==longSlip)&&(0==camber))
	{
		return;
	}

	//Setup the sticky friction constraint to bring the vehicle to rest at the tyre contact point.
	//The idea here is to resolve the singularity of the tyre long slip at low vz by replacing the long force with a velocity constraint.
	//Only do this if we can guarantee that the intention is to bring the car to rest (brake applied/no accel applied).
	//Smoothly reduce error to zero to avoid bringing car immediately to rest.  This avoiods graphical glitchiness.
	//We're going to replace the longitudinal tyre force with the sticky friction so set the long slip to zero to ensure zero long force.
	//Apply sticky friction to this tyre if 
	//(1) the wheel is locked (this means the brake/handbrake must be on) and the forward speed at the tyre contact point is small.
	//(2) the wheel is rotating very slowly and the forward speed at the tyre contact point is small and the accel pedal hasn't been pressed.
	if(vzAbs<gStickyTyreFrictionThresholdSpeed && PxAbs(wheelOmega)< gStickyTyreFrictionThresholdSpeed*recipWheelRadius && 0==accel)
	{
		*lowForwardSpeedTimer+=desc.mTimestep;		
	}
	else
	{
		*lowForwardSpeedTimer=0;
	}
	if((vzAbs < gStickyTyreFrictionThresholdSpeed && 0.0f==wheelOmega) || *lowForwardSpeedTimer>gLowForwardSpeedThresholdTime)
	{
		*stickyTyreActiveFlag=true;
		*stickyTyreError=vz*gStickyTyreFrictionDamping;
		*stickyTyreDir=tz;
		longSlip=0;
	}

	//We might have just set longSlip to zero.
	//If long slip/lat slip/camber are all zero than there will be zero tyre force.
	if((0==latSlip)&&(0==longSlip)&&(0==camber))
	{
		return;
	}

	//Compute the friction.
	const PxF32 x0=desc.mTyreData.mFrictionVsSlipGraph[0][0];
	const PxF32 y0=desc.mTyreData.mFrictionVsSlipGraph[0][1];
	const PxF32 x1=desc.mTyreData.mFrictionVsSlipGraph[1][0];
	const PxF32 y1=desc.mTyreData.mFrictionVsSlipGraph[1][1];
	const PxF32 x2=desc.mTyreData.mFrictionVsSlipGraph[2][0];
	const PxF32 y2=desc.mTyreData.mFrictionVsSlipGraph[2][1];
	const PxF32 recipx1Minusx0=desc.mTyreData.getFrictionVsSlipGraphRecipx1Minusx0();
	const PxF32 recipx2Minusx1=desc.mTyreData.getFrictionVsSlipGraphRecipx2Minusx1();
	const PxF32 longSlipAbs=PxAbs(longSlip);
	PxF32 mu;
	if(longSlipAbs<x1)
	{
		mu=y0 + (y1-y0)*(longSlipAbs-x0)*recipx1Minusx0;
	}
	else if(longSlipAbs<x2)
	{
		mu=y1 + (y2-y1)*(longSlipAbs-x1)*recipx2Minusx1;
	}
	else
	{
		mu=y2;
	}
	PX_ASSERT(mu>=0 && mu<=1.0f);
	mu*=desc.mFriction;
	*friction=mu;

	//Carry on and compute the forces.
	const PxF32 TEff = PxTan(latSlip - camber*camberStiff/latStiff);
	const PxF32 K = PxSqrt(latStiff*TEff*latStiff*TEff + longStiff*longSlip*longStiff*longSlip) /(mu*tyreLoad);
	const PxF32 KAbs=PxAbs(K);
	PxF32 FBar = smoothingFunction1(K);//K - ONE_THIRD*PxAbs(K)*K + ONE_TWENTYSEVENTH*K*K*K;
	PxF32 MBar = smoothingFunction2(K); //K - KAbs*K + ONE_THIRD*K*K*K - ONE_TWENTYSEVENTH*KAbs*K*K*K;
	//Mbar = PxMin(Mbar, 1.0f);
	PxF32 nu=1;
	if(K <= 2.0f*PxPi)
	{
		const PxF32 latOverlLong=latStiff*recipLongStiff;
		nu = 0.5f*(1.0f + latOverlLong - (1.0f - latOverlLong)*PxCos(K*0.5f));
	}
	const PxF32 FZero = mu*tyreLoad / (PxSqrt(longSlip*longSlip + nu*TEff*nu*TEff));
	const PxF32 fz = longSlip*FBar*FZero;
	const PxF32 fx = -nu*TEff*FBar*FZero;
	//TODO: pneumatic trail.
	const PxF32 pneumaticTrail=1.0f;
	const PxF32	fMy= nu * pneumaticTrail * TEff * MBar * FZero;

	//We can add the torque to the wheel.
	*wheelTorque=-fz*wheelRadius;
	*tyreLongForceMag=fz;
	*tyreLatForceMag=fx;
	*tyreAlignMoment=fMy;
}

void processSuspTyreWheels
(const PxTransform& carChassisTrnsfm, const PxVec3& carChassisLinVel, const PxVec3& carChassisAngVel,
 const PxVec3& gravity, const PxF32 gravityMagnitude, const PxF32 recipGravityMagnitude, const PxF32 timstestep,
 const PxVehicle4W& car, 
 const PxF32* PX_RESTRICT steerAngles, const PxF32* brakeTorquesForTyreLongSlip,
 const PxVehicleDrivableSurfaceToTyreFrictionPairs* frictionPairs,
 PxVehicleConstraintShader<PxVehicle4WSimulationData::eNUM_WHEELS>::VehicleConstraintData& vehConstraintData,
 PxF32* PX_RESTRICT lowForwardSpeedTimers,
 PxF32* PX_RESTRICT jounces, PxF32* PX_RESTRICT forwardSpeeds, PxF32* frictions, PxF32* PX_RESTRICT longSlips, PxF32* PX_RESTRICT latSlips, PxU32* PX_RESTRICT tyreSurfaceTypes,
 PxF32* PX_RESTRICT tyreTorques, PxVec3& chassisForce, PxVec3& chassisTorque)
{
#if PX_DEBUG_VEHICLE_ON
	zeroGraphDataWheels(PxVehicleGraph::eCHANNEL_JOUNCE);
	zeroGraphDataWheels(PxVehicleGraph::eCHANNEL_SUSPFORCE);
	zeroGraphDataWheels(PxVehicleGraph::eCHANNEL_TYRELOAD);
	zeroGraphDataWheels(PxVehicleGraph::eCHANNEL_NORMALISED_TYRELOAD);
	zeroGraphDataWheels(PxVehicleGraph::eCHANNEL_NORM_TYRE_LONG_FORCE);
	zeroGraphDataWheels(PxVehicleGraph::eCHANNEL_NORM_TYRE_LAT_FORCE);
	zeroGraphDataWheels(PxVehicleGraph::eCHANNEL_TYRE_LONG_SLIP);
	zeroGraphDataWheels(PxVehicleGraph::eCHANNEL_TYRE_LAT_SLIP);
	zeroGraphDataWheels(PxVehicleGraph::eCHANNEL_TYRE_FRICTION);
#endif

	bool* PX_RESTRICT suspLimitActiveFlags=vehConstraintData.mSuspLimitData.mActiveFlags;
	PxF32* PX_RESTRICT suspLimitErrors=vehConstraintData.mSuspLimitData.mErrors;

	bool* PX_RESTRICT stickyTyreActiveFlags=vehConstraintData.mStickyTyreData.mActiveFlags;
	PxVec3* PX_RESTRICT stickyTyreDirs=vehConstraintData.mStickyTyreData.mDirs;
	PxVec3* PX_RESTRICT stickyTyreCMOffsets=vehConstraintData.mStickyTyreData.mCMOffsets;
	PxF32* PX_RESTRICT stickyTyreErrors=vehConstraintData.mStickyTyreData.mErrors;

	//Vehicle data.
	const PxVehicle4WSimulationData& carData=car.mVehicleSimData;
	const PxVehicleTyreLoadFilterData& filterData=carData.getTyreLoadFilterData();
	const PxF32* PX_RESTRICT tyreRestLoads=carData.getTyreRestLoadsArray();
	const PxF32* PX_RESTRICT recipTyreRestLoads=carData.getRecipTyreRestLoadsArray();

	//Compute the right direction for later.
	const PxVec3 latDir=carChassisTrnsfm.rotate(PxVec3(1,0,0));

	PxF32 newLowForwardSpeedTimers[PxVehicle4WSimulationData::eNUM_WHEELS];
	for(PxU32 i=0;i<PxVehicle4WSimulationData::eNUM_WHEELS;i++)
	{
		const PxVehicleWheelData& wheel=carData.getWheelData(i);
		const PxVehicleSuspensionData& susp=carData.getSuspensionData(i);

		newLowForwardSpeedTimers[i]=lowForwardSpeedTimers[i];

#if PX_DEBUG_VEHICLE_ON
		updateGraphDataSuspJounce(i,-susp.mMaxDroop);
#endif

		//If there is no ground intersection then the wheel 
		//will sit at max droop.
		PxF32 jounce=-susp.mMaxDroop;
		jounces[i]=jounce;
		suspLimitActiveFlags[i]=false;
		suspLimitErrors[i]=0.0f;
		stickyTyreActiveFlags[i]=false;
		stickyTyreErrors[i]=0;

		//Find the hit (if any) for each suspension line that results in the most compression.
		PxF32 minDist=PX_MAX_F32;
		PxU32 mink=0xffffffff;
		PxRaycastHit* hits=car.mSqResults[i].hits;
		const PxU32 numHits=car.mSqResults[i].nbHits;
		for(PxU32 k=0;k<numHits;k++)
		{
			PX_ASSERT(&hits[k].shape->getActor()!=car.mActor);
			PX_ASSERT(hits[k].distance>=0);
			const PxF32 dist=hits[k].distance;
			if(dist<minDist)
			{
				minDist=dist;
				mink=k;
			}
		}

		//If there has been a hit then compute the suspension force and tyre load.
		if(mink!=0xffffffff)
		{
			const PxVec3 hitPos=hits[mink].impact;
			const PxVec3 hitNorm=hits[mink].normal;
			PxMaterial* material = hits[mink].shape->getMaterialFromInternalFaceIndex(hits[mink].faceIndex);

			//Get the surface type.
			PxU32 surfaceType;
			if(NULL!=material)
			{
				PX_CHECK_MSG(material->userData, "material->userData null ptr - this must be a pointer to a persistant PxVehicleDrivableSurfaceType instance");
				if(NULL!=material->userData)
				{
					PX_CHECK_MSG(0==((size_t)material->userData & 0x0f), "material->userData must be 16-byte aligned");
					surfaceType=((PxVehicleDrivableSurfaceType*)material->userData)->mType;
				}
				else
				{
					surfaceType=0;
				}
			}
			else
			{
				PX_CHECK_MSG(material, "material is null ptr");
				surfaceType=0;
			}

			//Get the tyre type.
			const PxU32 tyreType=carData.getTyreData(i).mType;
			PX_CHECK_MSG(surfaceType<frictionPairs->mNumSurfaceTypes, "Invalid drivable surface type when looking up frictionPairs table");
			PX_CHECK_MSG(tyreType<frictionPairs->mNumTyreTypes, "Invalid tyre type when looking up frictionPairs table");

			//Get the friction multiplier.
			const PxF32 frictionMultiplier=PxVehicle4WGetTypePairFriction(*frictionPairs,surfaceType,tyreType);
			PX_ASSERT(frictionMultiplier>=0);
			tyreSurfaceTypes[i]=surfaceType;

			//Compute the plane equation for the intersection plane found by the susp line raycast (n.p+d=0)
			const PxF32 hitD=-hitNorm.dot(hitPos);

			//Work out the point on the susp line that touches the intersection plane.
			//n.(v+wt)+d=0 where n,d describe the plane; v,w describe the susp ray; t is the point on the susp line.
			//t=-(n.v + d)/n.w
			const PxVec3& v=car.mSuspLineStarts[i];
			const PxVec3& w=car.mSuspLineDirs[i];
			const PxVec3& n=hitNorm;
			const PxF32 d=hitD;
			const PxF32 T=-(n.dot(v) + d)/(n.dot(w));

			//The rest pos of the susp line is 2*radius + maxBounce.
			const PxF32 restT=2.0f*wheel.mRadius+susp.mMaxCompression;

			//Compute the spring compression ie the difference between T and restT.
			//+ve means that the spring is compressed
			//-ve means that the spring is elongated.
			PxF32 dx=restT-T;

			//If the spring is elongated past its max droop then the wheel isn't touching the ground.
			//In this case the spring offers zero force and provides no support for the chassis/sprung mass.
			//Only carry on computing the spring force if the wheel is touching the ground.
			const PxF32 maxDroop=susp.mMaxDroop;
			PX_ASSERT(maxDroop>0);
			PX_UNUSED(maxDroop);
			if(dx > -susp.mMaxDroop)
			{
				//Clamp the spring compression so that it is never greater than the max bounce.
				suspLimitErrors[i] = dx - susp.mMaxCompression;
				suspLimitActiveFlags[i] = (dx > susp.mMaxCompression);
				jounce=PxMin(dx,susp.mMaxCompression);
				jounces[i]=jounce;
#if PX_DEBUG_VEHICLE_ON
				updateGraphDataSuspJounce(i,jounce);
#endif

				//Compute the speed of the rigid body along the suspension travel dir at the 
				//bottom of the wheel.
				const PxVec3 wheelBottomPos=v+w*jounce;
				const PxVec3 r=wheelBottomPos-carChassisTrnsfm.p;
				PxVec3 wheelBottomVel=carChassisLinVel;
				wheelBottomVel+=carChassisAngVel.cross(r);
				const PxF32 jounceSpeed=wheelBottomVel.dot(w);

				//We've got the cm offset to apply to the sticky tyre friction.  
				//Set it right now;
				stickyTyreCMOffsets[i]=r;

				//Compute the spring force.
				PxF32 springForce=susp.mSprungMass*w.dot(gravity);		//gravity acting along spring direction
				springForce+=susp.mSpringStrength*jounce;				//linear spring
				springForce+=susp.mSpringDamperRate*jounceSpeed;		//damping

#if PX_DEBUG_VEHICLE_ON
				updateGraphDataSuspForce(i,springForce);
#endif

				//Chassis force in opposite direction to spring travel direction.
				springForce*=-1.0f;
				const PxVec3 springForceJ=w*springForce;

				//Torque from spring force.
				const PxVec3 r2=carChassisTrnsfm.rotate(carData.getSuspForceAppPointOffset(i));
				const PxVec3 springTorqueJ=r2.cross(springForceJ);

				//Add the suspension force/torque to the chassis force/torque.
				chassisForce+=springForceJ;
				chassisTorque+=springTorqueJ;

				//Now compute the tyre load.
				//Add on the tyre mass gravity force.
				PxF32 tyreLoad=springForce*w.dot(hitNorm);
				tyreLoad -= wheel.mMass*gravity.dot(hitNorm);

				//Apply the opposite force to the hit object.
				PxRigidDynamic* dynamicHitActor=hits[mink].shape->getActor().isRigidDynamic();
				if(dynamicHitActor)
				{
					const PxVec3 hitForce=hitNorm*(-tyreLoad);
					PxRigidBodyExt::addForceAtPos(*dynamicHitActor,hitForce,hitPos);
				}

				//Normalise the tyre load 
				//Now work out the normalised tyre load.
				const PxF32 normalisedTyreLoad=tyreLoad*recipGravityMagnitude*recipTyreRestLoads[i];
				//Filter the normalised tyre load and compute the filtered tyre load too.
				const PxF32 filteredNormalisedTyreLoad=computeFilteredNormalisedTyreLoad(filterData,normalisedTyreLoad);
				const PxF32 filteredTyreLoad=filteredNormalisedTyreLoad*gravityMagnitude*tyreRestLoads[i];

#if PX_DEBUG_VEHICLE_ON
				updateGraphDataTyreLoad(i,filteredTyreLoad);
				updateGraphDataNormTyreLoad(i,filteredNormalisedTyreLoad);
#endif

				if(filteredTyreLoad>0)
				{
					//Now that we have the tyre load we can compute the tyre force.
					//Fill in data that we've computed and don't want 
					//to compute again when we compute the tyre forces.

					//We're going to compute wheel torque and lat/long forces.
					PxF32 wheelTorque=0;
					PxF32 tyreLongForceMag=0;
					PxVec3 tyreLongForceDir;
					PxF32 tyreLatForceMag=0;
					PxVec3 tyreLatForceDir;
					PxF32 tyreAlignMoment=0;

					//Fill out the desc that describes the inputs to the tyre force calculation.
					TyreForceUpdateDescriptor desc;
					desc.mWheelBottomVelocity=wheelBottomVel;
					desc.mChassisLatDir=latDir;
					desc.mHitNorm=hitNorm;
					desc.mTyreLoad=filteredTyreLoad;
					desc.mNormalisedTyreLoad=filteredNormalisedTyreLoad;
					desc.mRestTyreLoad=gravityMagnitude*tyreRestLoads[i];
					desc.mWheelOmega=car.mInternalDynamics[i];
					desc.mWheelRadius=car.mVehicleSimData.getWheelData(i).mRadius;
					desc.mRecipWheelRadius=car.mVehicleSimData.getWheelData(i).getRecipRadius();
					desc.mWheelSteer=steerAngles[i];
					desc.mTyreData=car.mVehicleSimData.getTyreData(i);
					desc.mBrakeTorque=brakeTorquesForTyreLongSlip[i];
					desc.mFriction=frictionMultiplier;
					desc.mAccel=car.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_ACCEL];
					desc.mTimestep=timstestep;

					//Compute the forces.
					computeTyreForce(desc,
						&stickyTyreActiveFlags[i],&stickyTyreErrors[i],&stickyTyreDirs[i],
						&newLowForwardSpeedTimers[i],
						&forwardSpeeds[i],&frictions[i],&longSlips[i],&latSlips[i],
						&wheelTorque,
						&tyreLongForceMag,tyreLongForceDir,&tyreLatForceMag,tyreLatForceDir,&tyreAlignMoment);

					//Add the tyre torque to the wheel torque.
					tyreTorques[i]=wheelTorque;

					//Tyre lat and long forces.
					const PxVec3 tyreLongForce=tyreLongForceDir*tyreLongForceMag;
					const PxVec3 tyreLatForce=tyreLatForceDir*tyreLatForceMag;
					const PxVec3 tyreForce=tyreLongForce+tyreLatForce;

					//Tyre lat and long torques.
					const PxVec3 r=carChassisTrnsfm.rotate(carData.getTyreForceAppPointOffset(i));
					const PxVec3 tyreTorque=r.cross(tyreForce);

					//Add all the forces/torques together.
					chassisForce+=tyreForce;
					chassisTorque+=tyreTorque;

#if PX_DEBUG_VEHICLE_ON
					if(gCarTyreForceAppPoints)
						gCarTyreForceAppPoints[i]=carChassisTrnsfm.p + carChassisTrnsfm.rotate(carData.getTyreForceAppPointOffset(i));
					if(gCarSuspForceAppPoints)
						gCarSuspForceAppPoints[i]=carChassisTrnsfm.p + carChassisTrnsfm.rotate(carData.getSuspForceAppPointOffset(i));

					if(gCarWheelGraphData[0])
					{
						updateGraphDataNormLongTyreForce(i, PxAbs(tyreLongForceMag)*normalisedTyreLoad/tyreLoad);
						updateGraphDataNormLatTyreForce(i, PxAbs(tyreLatForceMag)*normalisedTyreLoad/tyreLoad);
						updateGraphDataNormTyreAligningMoment(i, tyreAlignMoment*normalisedTyreLoad/tyreLoad);
						updateGraphDataLongTyreSlip(i,longSlips[i]);
						updateGraphDataLatTyreSlip(i,latSlips[i]);
						updateGraphDataTyreFriction(i,frictions[i]);
					}
#endif

				}
			}
		}

		lowForwardSpeedTimers[i]=(newLowForwardSpeedTimers[i]!=lowForwardSpeedTimers[i] ? newLowForwardSpeedTimers[i] : 0.0f);
	}
}

void updateVehicle
(const PxVec3& gravity, const PxF32 gravityMagnitude, const PxF32 recipGravityMagnitude, const PxF32 timestep, const PxVehicleDrivableSurfaceToTyreFrictionPairs& drivableSurfaceToTyreFrictionPairs,
 PxVehicle4W& car)
{
	PX_CHECK_MSG(car.mSqResults, "Need to call PxVehicle4WSuspensionRaycasts before trying to update");

	const PxVehicle4WSimulationData& carData=car.mVehicleSimData;

	//Compute the transform of the centre of mass.
	const PxTransform carChassisTransform=car.mActor->getGlobalPose().transform(car.mActor->getCMassLocalPose());
		
	//Update the autobox and decide whether to change gear up or down.
	if(car.mUseAutoGears)
	{
		processAutoBox(timestep,car);
	}

	//Process gearup/geardown commands.
	processGears(timestep,car);

	//Some constants that we can write down straight away.
	const PxF32 K=computeClutchStrength(carData.getClutchData(), car.mCurrentGear);
	const PxF32 G=computeGearRatio(carData.getGearsData(),car.mCurrentGear);
	const PxF32 KG=K*G;
	const PxF32 KGG=K*G*G;
	const PxF32 accel=car.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_ACCEL];
	const PxF32 brake=car.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_BRAKE];
	const PxF32 handbrake=car.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_HANDBRAKE];
	const PxF32 steer=car.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_RIGHT]-car.mControlAnalogVals[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_LEFT];
	const PxF32 steerGain=carData.getWheelData(0).mMaxSteer;
	PX_ASSERT(carData.getWheelData(0).mMaxSteer==carData.getWheelData(1).mMaxSteer);
#if PX_DEBUG_VEHICLE_ON
	updateGraphDataInternalDynamics(car.mInternalDynamics);
	updateGraphDataControlInputs(accel,brake,handbrake,steer);
	updateGraphDataGearRatio(G);
#endif

	//Contribution of each wheel to average wheel speed at clutch.
	PxF32 aveWheelSpeedContributions[PxVehicle4WSimulationData::eNUM_WHEELS];
	computeDiffAveWheelSpeedContributions(carData.getDiffData(),handbrake,aveWheelSpeedContributions);

	//Ackermann steering angles.
	PxF32 steerAngles[PxVehicle4WSimulationData::eNUM_WHEELS];
	for(PxU32 j=0;j<PxVehicle4WSimulationData::eNUM_WHEELS;j++)
	{
		steerAngles[j]=0.0f;
	}
	PxF32 frontLeftSteer,frontRightSteer;
	computeAckermannSteerAngles(steer,steerGain,carData.getAckermannGeometryData(),&frontLeftSteer,&frontRightSteer);
	steerAngles[PxVehicle4WSimulationData::eFRONT_LEFT_WHEEL]=frontLeftSteer+carData.getWheelData(PxVehicle4WSimulationData::eFRONT_LEFT_WHEEL).mToeAngle;
	steerAngles[PxVehicle4WSimulationData::eFRONT_RIGHT_WHEEL]=frontRightSteer+carData.getWheelData(PxVehicle4WSimulationData::eFRONT_RIGHT_WHEEL).mToeAngle;

	//Ready to do the update.
	PxVec3 carChassisLinVel=car.mActor->getLinearVelocity();
	PxVec3 carChassisAngVel=car.mActor->getAngularVelocity();
	PxF32 jounces[PxVehicle4WSimulationData::eNUM_WHEELS];
	PxF32 diffTorqueRatios[PxVehicle4WSimulationData::eNUM_WHEELS];
	PxF32 brakeTorquesForTyreLongSlip[PxVehicle4WSimulationData::eNUM_WHEELS];
	PxF32 forwardSpeeds[PxVehicle4WSimulationData::eNUM_WHEELS];
	PxF32 tyreFrictions[PxVehicle4WSimulationData::eNUM_WHEELS];
	PxF32 longSlips[PxVehicle4WSimulationData::eNUM_WHEELS];
	PxF32 latSlips[PxVehicle4WSimulationData::eNUM_WHEELS];
	PxU32 tyreSurfaceTypes[PxVehicle4WSimulationData::eNUM_WHEELS];
	PxF32 engineDriveTorque=0.0f;
	const PxU32 numSubSteps=1;
	const PxF32 subTimestep=timestep/(1.0f*numSubSteps);
	for(PxU32 k=0;k<numSubSteps;k++)
	{
		//Bit of a trick here.
		//The sdk will apply gravity*dt completely independent of the tyre forces.
		//Cars will never come to rest this way because even if the tyre model brings the car
		//exactly to rest it will just be immediately perturbed by gravitational acceleration.
		//Maybe we should add gravity here before computing the tyre forces so that the tyre
		//forces act against the gravitational forces that will be later applied.  
		//We don't actually ever apply gravity to the rigid body, we just imagine the tyre/susp 
		//forces that would be needed if gravity had already been applied.  The sdk, therefore, 
		//still needs to apply gravity to the chassis rigid body in its update.
		PX_ASSERT(carChassisLinVel==car.mActor->getLinearVelocity());
		carChassisLinVel+=gravity*timestep;

		//Set the force and torque for the current update to zero.
		PxVec3 chassisForce(0,0,0);
		PxVec3 chassisTorque(0,0,0);

		//Need to solve Aw=b for unknown w (the wheel and engine speeds).
		MatrixNN<PxVehicle4WSimulationData::eNUM_WHEELS+1> A;
		VectorN<PxVehicle4WSimulationData::eNUM_WHEELS+1> b;

		//Diff ratios needed (how we split the torque between the drive wheels).
		computeDiffTorqueRatios(carData.getDiffData(),handbrake,car.mInternalDynamics,diffTorqueRatios);

		//Brake/handbrake/tyre torques for wheels and tyre/susp forces to be applied to rigid body.
		PxF32 tyreTorques[PxVehicle4WSimulationData::eNUM_WHEELS];
		PxF32 brakeTorques[PxVehicle4WSimulationData::eNUM_WHEELS];
		for(PxU32 j=0;j<PxVehicle4WSimulationData::eNUM_WHEELS;j++)
		{
			brakeTorques[j]=0.0f;
			brakeTorquesForTyreLongSlip[j]=0.0f;
			tyreTorques[j]=0.0f;
			jounces[j]=0.0f;
			forwardSpeeds[j]=0.0f;
			tyreFrictions[j]=0.0f;
			longSlips[j]=0.0f;
			latSlips[j]=0.0f;
			tyreSurfaceTypes[j]=PxVehicleDrivableSurfaceType::eSURFACE_TYPE_UNKNOWN;
		}

		//Compute the brake torques.
		computeBrakeAndHandBrakeTorques
			(&carData.getWheelData(0),car.mInternalDynamics,brake,handbrake,
			 brakeTorques,brakeTorquesForTyreLongSlip);


		//Mark the constraints as dirty to force them to be updated in the sdk.
		car.getVehicletConstraintShader().mConstraint->markDirty();

		processSuspTyreWheels
			(carChassisTransform, carChassisLinVel, carChassisAngVel,
			 gravity,gravityMagnitude,recipGravityMagnitude, timestep,
			 car, 
			 steerAngles, brakeTorquesForTyreLongSlip,
			 &drivableSurfaceToTyreFrictionPairs,
			 car.getVehicletConstraintShader().mData, 
			 car.mTyreLowForwardSpeedTimers,
			 jounces, forwardSpeeds, tyreFrictions, longSlips, latSlips, tyreSurfaceTypes,
			 tyreTorques,chassisForce,chassisTorque);

		//Wheels.
		{
			for(PxU32 j=0;j<PxVehicle4WSimulationData::eNUM_WHEELS;j++)
			{
				const PxF32 dt=subTimestep*carData.getWheelData(j).getRecipMOI();
				const PxF32 R=diffTorqueRatios[j];
				const PxF32 dtKGGR=dt*KGG*R;
				for(PxU32 k=0;k<PxVehicle4WSimulationData::eNUM_WHEELS;k++)
				{
					A.set(j,k,dtKGGR*aveWheelSpeedContributions[k]);
				}
				A.set(j,j,1.0f+dtKGGR*aveWheelSpeedContributions[j]+dt*carData.getWheelData(j).mDampingRate);
				A.set(j,PxVehicle4WSimulationData::eNUM_WHEELS,-dt*KG*R);
				b[j] = car.mInternalDynamics[j] + dt*(brakeTorques[j]+tyreTorques[j]);
			}
		}

		//Engine.
		const PxVehicleEngineData& engineData=carData.getEngineData();
		const PxF32 engineOmega=car.mInternalDynamics[PxVehicle4WSimulationData::eNUM_WHEELS];
		engineDriveTorque=computeEngineDriveTorque(engineData,engineOmega,accel);
		{
			const PxF32 dt=subTimestep;
			const PxF32 engineDampRate=computeEngineDampingRate(engineData,accel);
			const PxF32 dtKG=dt*K*G;
			for(PxU32 j=0;j<PxVehicle4WSimulationData::eNUM_WHEELS;j++)
			{
				A.set(PxVehicle4WSimulationData::eNUM_WHEELS,j,-dtKG*aveWheelSpeedContributions[j]);
			}
			A.set(PxVehicle4WSimulationData::eNUM_WHEELS,PxVehicle4WSimulationData::eNUM_WHEELS,1.0f + dt*(K+engineDampRate));
			b[PxVehicle4WSimulationData::eNUM_WHEELS] = engineOmega + dt*engineDriveTorque;
		}

#if PX_DEBUG_VEHICLE_ON
		updateGraphDataEngineDriveTorque(engineDriveTorque);
		updateGraphDataClutchSlip(car.mInternalDynamics,aveWheelSpeedContributions,G);
#endif

		//Solve Aw=b
		VectorN<PxVehicle4WSimulationData::eNUM_WHEELS+1> result;
		MatrixNNLUSolver<PxVehicle4WSimulationData::eNUM_WHEELS+1> solver;
		solver.decomposeLU(A);
		solver.solve(b,result);
		PX_ASSERT(isValid(A,b,result));

		//Check for sanity in the resultant internal rotation speeds.
		//If the brakes are on and the wheels have switched direction then lock them at zero.
		for(PxU32 j=0;j<PxVehicle4WSimulationData::eNUM_WHEELS;j++)
		{
			const PxF32 brakeGain=carData.getWheelData(j).mMaxBrakeTorque;
			const PxF32 handbrakeGain=carData.getWheelData(j).mMaxHandBrakeTorque;
			const PxF32 oldOmega=car.mInternalDynamics[j];
			const PxF32 newOmega=result[j];
			if(((brake*brakeGain + handbrake*handbrakeGain)!=0.0f) && (oldOmega*newOmega <=0))
			{
				result[j]=0.0f;
			}
		}
		//Clamp the engine revs.
		result[PxVehicle4WSimulationData::eNUM_WHEELS]=PxClamp(result[PxVehicle4WSimulationData::eNUM_WHEELS],/*-carData.mCarEngine.mMaxOmega*/0.0f,carData.getEngineData().mMaxOmega);

		//Copy back to the car's internal rotation speeds.
		for(PxU32 j=0;j<PxVehicle4WSimulationData::eNUM_WHEELS+1;j++)
		{
			car.mInternalDynamics[j]=result[j];
		}

		//Integrate the chassis velocity.
		car.mActor->addForce(chassisForce*subTimestep,PxForceMode::eIMPULSE);
		car.mActor->addTorque(chassisTorque*subTimestep,PxForceMode::eIMPULSE);
		carChassisLinVel=car.mActor->getLinearVelocity();
		carChassisAngVel=car.mActor->getAngularVelocity();
	}

	//TODO: we probably need to have a graphics jounce and a physics jounce and 
	//blend the graphics jounce towards the physics jounce to avoid graphical pops at kerbs etc.
	//Integrate the wheel rotation angles.
	//Just use the last value of the wheel rotation speeds if sub-stepping.
	PxShape* shapeBuffer[PxVehicle4WSimulationData::eNUM_WHEELS];
	PX_ASSERT(car.mActor->getNbShapes() >= PxVehicle4WSimulationData::eNUM_WHEELS +1);
	car.mActor->getShapes(shapeBuffer,PxVehicle4WSimulationData::eNUM_WHEELS);
	const PxVec3 cmOffset=car.mActor->getCMassLocalPose().p;
	for(PxU32 j=0;j<PxVehicle4WSimulationData::eNUM_WHEELS;j++)
	{
		//At low vehicle forward speeds we have some numerical difficulties getting the 
		//wheel rotation speeds to be correct due to the tyre model's difficulties at low vz.
		//The solution is to blend between the rolling speed at the wheel and the wheel's actual rotation speed.
		//If the wheel is 
		//(i)   in the air, 
		//(ii)  under braking torque, 
		//(iii) driven by the engine through the gears and diff
		//then always use the wheel's actual rotation speed.
		//Just to be clear, this means we will blend when the wheel
		//(i)   is on the ground
		//(ii)  has no brake applied
		//(iii) has no drive torque applied from the clutch
		//(iv)  is at low forward speed
		PxF32 wheelOmega=car.mInternalDynamics[j];
		if(jounces[j] > -carData.getSuspensionData(j).mMaxDroop &&					//(i)   wheel touching ground
			0.0f==brakeTorquesForTyreLongSlip[j] &&									//(ii)  no brake applied
			0.0f==diffTorqueRatios[j]*KG*engineDriveTorque &&						//(iii) no drive torque applied
			PxAbs(forwardSpeeds[j])<gThresholdForwardSpeedForWheelAngleIntegration)	//(iv)  low speed
		{
			const PxF32 recipWheelRadius=carData.getWheelData(j).getRecipRadius();
			const PxF32 alpha=PxAbs(forwardSpeeds[j])*gRecipThresholdForwardSpeedForWheelAngleIntegration;
			wheelOmega = (forwardSpeeds[j]*recipWheelRadius)*(1.0f-alpha) + wheelOmega*alpha;

			//TODO: maybe just set the car wheel omega to the blended value?
			//Not sure about this bit.  
			car.mInternalDynamics[j]=wheelOmega;
		}

		PxF32 newRotAngle=car.mWheelRotationAngles[j]+wheelOmega*timestep;
		//Clamp the wheel rotation angle to a range (-10*pi,10*pi) to stop it getting crazily big.
		newRotAngle=physx::intrinsics::fsel(newRotAngle-10*PxPi, newRotAngle-10*PxPi, physx::intrinsics::fsel(-newRotAngle-10*PxPi, newRotAngle + 10*PxPi, newRotAngle));
		car.mWheelRotationAngles[j]=newRotAngle;

		//Compute the transform of the wheel shapes. 
		const PxVec3 pos=cmOffset+car.mVehicleSimData.getWheelCentreOffset(j)-car.mVehicleSimData.getSuspTravelDirection(j)*jounces[j];
		const PxQuat quat(steerAngles[j], PxVec3(0,1,0));
		const PxQuat quat2(newRotAngle,quat.getBasisVector0());
		const PxTransform t(pos,quat2*quat);
		shapeBuffer[j]->setLocalPose(t);

		//Store the jounce and slips.
		car.mSuspJounces[j]=jounces[j];
		car.mLongSlips[j]=longSlips[j];
		car.mLatSlips[j]=latSlips[j];
		car.mTyreFrictions[j]=tyreFrictions[j];
		car.mTyreSurfaceTypes[j]=tyreSurfaceTypes[j];
	}

	//Set back to null to prepare for the next raycasts and update.
	car.mSqResults=NULL;
}

}//namespace physx

#if PX_DEBUG_VEHICLE_ON

void physx::PxVehicle4WUpdateSingleVehicleAndStoreTelemetryData(const PxF32 timestep, const PxVec3& gravity, const PxVehicleDrivableSurfaceToTyreFrictionPairs& vehicleDrivableSurfaceToTyreFrictionPairs, PxVehicle4W* focusVehicle, PxVehicle4WTelemetryData& telemetryData)
{
	PX_CHECK_MSG(gravity.magnitude()>0, "gravity vector must have non-zero length");
	PX_CHECK_MSG(timestep>0, "timestep must be greater than zero");
	PX_CHECK_MSG(0==((size_t)vehicleDrivableSurfaceToTyreFrictionPairs.mPairs & 0x0f), "mVehicleDrivableSurfaceToTyreFrictionPairs must be 16-byte aligned");
	PX_CHECK_MSG(0==((size_t)(vehicleDrivableSurfaceToTyreFrictionPairs.mNumSurfaceTypes*vehicleDrivableSurfaceToTyreFrictionPairs.mNumTyreTypes) & 0x03), "mNumSurfaceTypes*mNumTyreTypes must be a multiple of 4");

	const PxF32 gravityMagnitude=gravity.magnitude();
	const PxF32 recipGravityMagnitude=1.0f/gravityMagnitude;
	const PxVehicleDrivableSurfaceToTyreFrictionPairs& drivableSurfaceToTyreFrictionPairs=vehicleDrivableSurfaceToTyreFrictionPairs;

	PxF32 engineGraphData[PxVehicleGraph::eMAX_NUM_ENGINE_CHANNELS];
	PxF32 wheelGraphData[PxVehicle4WSimulationData::eNUM_WHEELS][PxVehicleGraph::eMAX_NUM_WHEEL_CHANNELS];
	PxVec3 suspForceAppPoints[PxVehicle4WSimulationData::eNUM_WHEELS];
	PxVec3 tyreForceAppPoints[PxVehicle4WSimulationData::eNUM_WHEELS];

	gCarEngineGraphData=engineGraphData;
	for(PxU32 i=0;i<PxVehicle4WSimulationData::eNUM_WHEELS;i++)
	{
		gCarWheelGraphData[i]=wheelGraphData[i];
	}
	gCarSuspForceAppPoints=suspForceAppPoints;
	gCarTyreForceAppPoints=tyreForceAppPoints;

	updateVehicle(gravity,gravityMagnitude,recipGravityMagnitude,timestep,drivableSurfaceToTyreFrictionPairs,*focusVehicle);

	for(PxU32 i=0;i<PxVehicle4WSimulationData::eNUM_WHEELS;i++)
	{
		telemetryData.mWheelGraphs[i].updateTimeSlice(wheelGraphData[i]);
	}
	telemetryData.mEngineGraph.updateTimeSlice(engineGraphData);

}

#endif

void physx::PxVehicle4WUpdate(const PxF32 timestep, const PxVec3& gravity, const PxVehicleDrivableSurfaceToTyreFrictionPairs& vehicleDrivableSurfaceToTyreFrictionPairs, const PxU32 numVehicles, PxVehicle4W** vehicles)
{
	PX_CHECK_MSG(gravity.magnitude()>0, "gravity vector must have non-zero length");
	PX_CHECK_MSG(timestep>0, "timestep must be greater than zero");
	PX_CHECK_MSG(0==((size_t)vehicleDrivableSurfaceToTyreFrictionPairs.mPairs & 0x0f), "mVehicleDrivableSurfaceToTyreFrictionPairs must be 16-byte aligned");
	PX_CHECK_MSG(0==((size_t)(vehicleDrivableSurfaceToTyreFrictionPairs.mNumSurfaceTypes*vehicleDrivableSurfaceToTyreFrictionPairs.mNumTyreTypes) & 0x03), "mNumSurfaceTypes*mNumTyreTypes must be a multiple of 4");

	const PxF32 gravityMagnitude=gravity.magnitude();
	const PxF32 recipGravityMagnitude=1.0f/gravityMagnitude;

#if PX_DEBUG_VEHICLE_ON

	gCarEngineGraphData=NULL;
	for(PxU32 j=0;j<PxVehicle4WSimulationData::eNUM_WHEELS;j++)
	{
		gCarWheelGraphData[j]=NULL;
	}
	gCarSuspForceAppPoints=NULL;
	gCarTyreForceAppPoints=NULL;

#endif

	for(PxU32 i=0;i<numVehicles;i++)
	{
		PxVehicle4W& veh=*vehicles[i];
		updateVehicle(gravity,gravityMagnitude,recipGravityMagnitude,timestep,vehicleDrivableSurfaceToTyreFrictionPairs,veh);
	}
}

void physx::PxVehicle4WSuspensionRaycasts(PxBatchQuery* batchQuery, const PxU32 numVehicles, PxRaycastQueryResult* sceneQueryResults, PxVehicle4W** vehicles)
{
	//Work out the rays for the suspension line raycasts and perform all the raycasts.
	for(PxU32 i=0;i<numVehicles;i++)
	{
		//Get the current car.
		PxVehicle4W& veh=*vehicles[i];
		const PxVehicle4WSimulationData& carData=veh.mVehicleSimData;

		//Set the results ptr.
		veh.mSqResults=&sceneQueryResults[i*PxVehicle4WSimulationData::eNUM_WHEELS];

		//Get the filter data.
		PxShape* shapeBuffer[1];
		veh.mActor->getShapes(shapeBuffer,1);
		const PxSceneQueryFilterData carFilterData(shapeBuffer[0]->getQueryFilterData(), PxSceneQueryFilterFlag::eSTATIC|PxSceneQueryFilterFlag::eDYNAMIC|PxSceneQueryFilterFlag::ePREFILTER);

		//Get the transform of the chassis.
		const PxTransform carChassisTrnsfm=veh.mActor->getGlobalPose().transform(veh.mActor->getCMassLocalPose());

		//Add a raycast for each wheel.
		for(PxU32 j=0;j<PxVehicle4WSimulationData::eNUM_WHEELS;j++)
		{
			const PxVehicleSuspensionData& susp=carData.getSuspensionData(j);
			const PxF32 maxDroop=susp.mMaxDroop;
			const PxF32 maxBounce=susp.mMaxCompression;
			const PxVehicleWheelData& wheel=carData.getWheelData(j);
			const PxF32 radius=wheel.mRadius;
			PX_ASSERT(maxBounce>0);
			PX_ASSERT(maxDroop>0);

			//Direction of raycast.
			const PxVec3 downwardSuspensionTravelDir=carChassisTrnsfm.rotate(carData.getSuspTravelDirection(j));

			//Position at top of wheel at maximum compression.
			PxVec3 wheelPosition=carChassisTrnsfm.transform(carData.getWheelCentreOffset(j));
			wheelPosition-=downwardSuspensionTravelDir*(radius+maxBounce);

			//Total length from top of wheel at max compression to bottom of wheel at max droop.
			PxF32 rayLength=radius + maxBounce  + maxDroop + radius;
			//Add another radius on for good measure.
			rayLength+=radius;

			//Store the susp line ray for later use.
			veh.mSuspLineStarts[j]=wheelPosition;
			veh.mSuspLineDirs[j]=downwardSuspensionTravelDir;
			veh.mSuspLineLengths[j]=rayLength;

			//Add the raycast to the scene query.
			batchQuery->raycastSingle(wheelPosition, downwardSuspensionTravelDir, carFilterData, rayLength, PxSceneQueryFlag::eIMPACT|PxSceneQueryFlag::eNORMAL|PxSceneQueryFlag::eDISTANCE|PxSceneQueryFlag::eUV);
		}
	}

	batchQuery->execute();

}

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

#ifndef PX_VEHICLE_EXT_H
#define PX_VEHICLE_EXT_H
/** \addtogroup vehicle
  @{
*/
#include "common/PxPhysXCommon.h"
#include "vehicle/PxVehicle.h"
#include "PxPreprocessor.h"
#include "PxScene.h"
#include "PxSweepCache.h"

#ifndef PX_DOXYGEN
namespace physx
{
#endif

#define PX_DEBUG_VEHICLE_ON 1

class PxRigidDynamic;

struct PxVehicle4WSimpleSetup
{

	PxVehicle4WSimpleSetup();
	~PxVehicle4WSimpleSetup(){}

	/**
	\brief Full extents of chassis aabb (must be set, default value zero)
	*/
	PxVec3	mChassisDims;

	/**
	\brief Offset from actor centre to centre of mass of chassis rigid body (must be set, default value zero)
	*/
	PxVec3	mChassisCMOffset;			

	/**
	\brief Total mass of chassis rigid body (kg) 
	*/
	PxReal	mChassisMass;

	/**
	\brief Moment of inertia of chassis rigid body (kg m^2) (must be set, default value zero)
	*/
	PxVec3	mChassisMOI;

	/**
	\brief Offset from rigid body centre of mass to wheel centres at rest (must be set, default value zero)
	*/
	PxVec3	mWheelCentreCMOffsets[PxVehicle4WSimulationData::eNUM_WHEELS];

	/**
	\brief Front wheel width (must be set, default value zero)
	@see PxVehicleWheelData
	*/
	PxReal	mFrontWheelWidth;					

	/**
	\brief Front wheel radius (must be set, default value zero)
	@see PxVehicleWheelData
	*/
	PxReal	mFrontWheelRadius;					//(must be set, default value zero)

	/**
	\brief Front wheel mass 
	@see PxVehicleWheelData
	*/
	PxReal  mFrontWheelMass;	

	/**
	\brief Front wheel brake torque 
	@see PxVehicleWheelData
	*/
	PxReal	mFrontWheelBrakeTorque;

	/**
	\brief Front wheel maximum steer
	@see PxVehicleWheelData
	*/
	PxReal	mFrontWheelMaxSteer;

	/**
	\brief Front wheel toe angle
	@see PxVehicleWheelData
	*/
	PxReal	mFrontWheelToeAngle;

	/**
	\brief Front wheel damping rate
	@see PxVehicleWheelData
	*/
	PxReal mFrontWheelDampingRate;

	/**
	\brief Rear wheel width (must be set, default value zero)
	@see PxVehicleWheelData
	*/
	PxReal	mRearWheelWidth;					

	/**
	\brief Rear wheel radius (must be set, default value zero)
	@see PxVehicleWheelData
	*/
	PxReal	mRearWheelRadius;					

	/**
	\brief Rear wheel mass 
	@see PxVehicleWheelData
	*/
	PxReal  mRearWheelMass;		

	/**
	\brief Rear wheel brake torque 
	@see PxVehicleWheelData
	*/
	PxReal	mRearWheelBrakeTorque;

	/**
	\brief Rear wheel handbrake torque 
	@see PxVehicleWheelData
	*/
	PxReal	mRearWheelHandbrakeTorque;

	/**
	\brief Rear wheel toe angle
	@see PxVehicleWheelData
	*/
	PxReal	mRearWheelToeAngle;

	/**
	\brief Front wheel damping rate
	@see PxVehicleWheelData
	*/
	PxReal mRearWheelDampingRate;

	/**
	\brief Front suspension travel direction as unit vector pointing downwards (must be set, default value zero)
	@see PxVehicleSuspensionData
	*/
	PxVec3  mFrontSuspensionTravelDir;			

	/**
	\brief Front suspension max compression
	@see PxVehicleSuspensionData
	*/
	PxReal	mFrontSuspensionMaxCompression;

	/**
	\brief Front suspension max droop
	@see PxVehicleSuspensionData
	*/
	PxReal	mFrontSuspensionMaxDroop;

	/**
	\brief Front suspension max droop
	@see PxVehicleSuspensionData
	*/

	/**
	\brief Front suspension spring strength
	@see PxVehicleSuspensionData
	*/
	PxReal	mFrontSuspensionSpringStrength;	

	/**
	\brief Front suspension spring damping rate
	@see PxVehicleSuspensionData
	*/
	PxReal	mFrontSuspensionSpringDampingRate;	

	/**
	\brief Rear suspension travel direction as unit vector pointing downwards (must be set, default value zero)
	@see PxVehicleSuspensionData
	*/
	PxVec3  mRearSuspensionTravelDir;			

	/**
	\brief Rear suspension spring max compression
	@see PxVehicleSuspensionData
	*/
	PxReal	mRearSuspensionMaxCompression;

	/**
	\brief Rear suspension spring max droop
	@see PxVehicleSuspensionData
	*/
	PxReal	mRearSuspensionMaxDroop;

	/**
	\brief Rear suspension spring strength
	@see PxVehicleSuspensionData
	*/
	PxReal	mRearSuspensionSpringStrength;	

	/**
	\brief Rear suspension spring damping rate
	@see PxVehicleSuspensionData
	*/
	PxReal	mRearSuspensionSpringDampingRate;	

	/**
	\brief Minimum normalised load (load/restLoad) that gives a flat lateral stiffness response.
	@see PxVehicleTyre
	*/
	PxReal	mFrontTyreLatStiffX;

	/**
	\brief  Maximum possible lateral stiffness divided by the rest tyre load
	@see PxVehicleTyre
	*/
	PxReal	mFrontTyreLatStiffY;

	/**
	\brief Tyre longitudinal stiffness
	@see PxVehicleTyre
	*/
	PxReal	mFrontTyreLongitudinalStiffness;

	/**
	\brief Tyre camber stiffness
	@see PxVehicleTyre
	*/
	PxReal	mFrontTyreCamberStiffness;

	/**
	\brief Friction vs longitudinal slip graph
	@see PxVehicleTyre
	*/
	PxReal	mFrontTyreFrictionVsSlipGraph[3][2];

	/**
	\brief Front tyre type
	@see PxVehicleTyre
	*/
	PxU32	mFrontTyreType;

	/**
	\brief Minimum normalised load (load/restLoad) that gives a flat lateral stiffness response.
	@see PxVehicleTyre
	*/
	PxReal	mRearTyreLatStiffX;

	/**
	\brief  Maximum possible lateral stiffness divided by the rest tyre load
	@see PxVehicleTyre
	*/
	PxReal	mRearTyreLatStiffY;

	/**
	\brief Tyre longitudinal stiffness
	@see PxVehicleTyre
	*/
	PxReal	mRearTyreLongitudinalStiffness;

	/**
	\brief Tyre camber stiffness
	@see PxVehicleTyre
	*/
	PxReal	mRearTyreCamberStiffness;

	/**
	\brief Friction vs longitudinal slip graph
	@see PxVehicleTyre
	*/
	PxReal	mRearTyreFrictionVsSlipGraph[3][2];

	/**
	\brief Front tyre type
	@see PxVehicleTyre
	*/
	PxU32	mRearTyreType;

	/**
	\brief Graph point of normalised tyre load filter
	@see PxVehicleTyreLoadFilterData
	*/
	PxReal	mTyreLoadFilterMinNormalisedLoad;

	/**
	\brief Graph point of normalised tyre load filter
	@see PxVehicleTyreLoadFilterData
	*/
	PxReal	mTyreLoadFilterMaxNormalisedLoad;

	/**
	\brief Graph point of normalised tyre load filter
	@see PxVehicleTyreLoadFilterData
	*/
	PxReal	mTyreLoadFilterMaxFilteredNormalisedLoad;

	/**
	\brief Offset that specifies the application point of rear suspension forces.
	\brief Force applied at mWheelCentreCMOffset - mSuspensionTravelDir*(mSuspensionTravelDir.mWheelCentreCMOffset - mSuspForceAppPointOffset)
	*/
	PxReal	mFrontSuspForceAppPointVerticalCMOffset;	

	/**
	\brief Offset that specifies the application point of front suspension forces.
	\brief Force applied at mWheelCentreCMOffset - mSuspensionTravelDir*(mSuspensionTravelDir.mWheelCentreCMOffset - mSuspForceAppPointOffset)
	*/

	PxReal	mRearSuspForceAppPointVerticalCMOffset;	

	/**
	\brief Offset that specifies the application point of front tyre forces.
	\brief Force applied at mWheelCentreCMOffset - mSuspensionTravelDir*(mSuspensionTravelDir.mWheelCentreCMOffset - mTyreForceAppPointOffset)
	*/
	PxReal	mFrontTyreForceAppPointVerticalCMOffset;	

	/**
	\brief Offset that specifies the application point of rear tyre forces.
	\brief Force applied at mWheelCentreCMOffset - mSuspensionTravelDir*(mSuspensionTravelDir.mWheelCentreCMOffset - mTyreForceAppPointOffset)
	*/
	PxReal	mRearTyreForceAppPointVerticalCMOffset;	

	/**
	\brief Differential torque between front and rear
	@see PxVehicleDifferential4WData
	*/
	PxReal	mDiffFrontRearSplit;

	/**
	\brief Permitted ration of front and rear wheel rotational speed.
	@see PxVehicleDifferential4WData
	*/
	PxReal	mDiffCentreBias;

	/**
	\brief Permitted ration of front left and front right wheel rotational speed.
	@see PxVehicleDifferential4WData
	*/
	PxReal	mDiffFrontBias;

	/**
	\brief Permitted ration of rear left and ear right wheel rotational speed.
	@see PxVehicleDifferential4WData
	*/
	PxReal	mDiffRearBias;

	/**
	\brief Type of differential 
	@see PxVehicleDifferential4WData
	*/
	PxU32	mDiffType;

	/**
	\brief Maximum torque delivered by engine.
	@see PxVehicleEngineData
	*/
	PxReal	mEnginePeakTorque;

	/**
	\brief Graph of torque against normalised revs (revs/max revs)
	@see PxVehicleEngineData
	*/
	PxFixedSizeLookupTable<PxVehicleEngineData::eMAX_NUM_ENGINE_TORQUE_CURVE_ENTRIES> mEngineTorqueCurve;

	/**
	\brief Maximum engine rotational speed
	@see PxVehicleEngineData
	*/
	PxReal	mEngineMaxOmega;

	/**
	\brief Damping rate of engine when clutch is engaged.
	@see PxVehicleEngineData
	*/
	PxReal	mEngineEngagedClutchDampingRate;

	/**
	\brief Damping rate of engine when clutch is disengaged.
	@see PxVehicleEngineData
	*/
	PxReal	mEngineDisengagedClutchDampingRate;

	/**
	\brief Number of gears.
	@see PxVehicleGearsData
	*/
	PxU32	mNumGearRatios;

	/**
	\brief Ratio of each gear
	@see PxVehicleGearsData
	*/
	PxReal	mGearRatios[PxVehicleGearsData::eMAX_NUM_GEAR_RATIOS];

	/**
	\brief Final gear ratio
	@see PxVehicleGearsData
	*/
	PxReal	mGearFinalRatio;

	/**
	\brief Time taken to complete gear change
	@see PxVehicleGearsData
	*/
	PxReal	mGearSwitchTime;

	/**
	\brief Value of revs/maxRevs required to trigger a gearup change
	@see PxVehicleAutoBoxData
	*/
	PxReal	mAutoBoxUpRatios[PxVehicleGearsData::eMAX_NUM_GEAR_RATIOS];

	/**
	\brief Value of revs/maxRevs required to trigger a geardown change
	@see PxVehicleAutoBoxData
	*/
	PxReal	mAutoBoxDownRatios[PxVehicleGearsData::eMAX_NUM_GEAR_RATIOS];

	/**
	\brief Strength of clutch
	@see PxVehicleClutchData
	*/
	PxReal	mClutchStrength;

	/**
	\brief Ackermann accuracy (1.0f achieves perfect ackermann angle, 0.0f uses raw input steer)
	@see PxVehicleAckermannGeometryData
	*/
	PxReal mAckermannAccuracy;
};

/**
\brief Construct vehicle simulation data from simple setup data.
@see PxSetupVehicle
*/
PxVehicle4WSimulationData  PxCreateVehicle4WSimulationData(const PxVehicle4WSimpleSetup& defaultSetupDesc);

/**
\brief Used to produce smooth analog driving control values from key inputs.
@see PxSmoothDigitalRawInputsAndSetVehicleAnalogInputs, PxSmoothAnalogRawInputsAndSetVehicleAnalogInputs
*/
struct PxVehicleKeySmoothingData
{
public:

	/**
	\brief Rise rate of each analog value if digital value is 1
	*/
	PxReal mRiseRates[PxVehicleControlInputs::eMAX_NUM_VEHICLE_ANALOG_INPUTS];

	/**
	\brief Fall rate of each analog value if digital value is 0
	*/
	PxReal mFallRates[PxVehicleControlInputs::eMAX_NUM_VEHICLE_ANALOG_INPUTS];

	PxReal mPad[2];
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleKeySmoothingData)& 0x0f));

/**
\brief Used to produce smooth analog driving control values from analog inputs.
@see PxSmoothDigitalRawInputsAndSetVehicleAnalogInputs, PxSmoothAnalogRawInputsAndSetVehicleAnalogInputs
*/
struct PxVehiclePadSmoothingData
{
public:

	/**
	\brief Rise rate of each analog value from previous value towards target if target>previous
	*/
	PxReal mRiseRates[PxVehicleControlInputs::eMAX_NUM_VEHICLE_ANALOG_INPUTS];

	/**
	\brief Rise rate of each analog value from previous value towards target if target<previous
	*/
	PxReal mFallRates[PxVehicleControlInputs::eMAX_NUM_VEHICLE_ANALOG_INPUTS];
	PxReal mPad[2];
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehiclePadSmoothingData)& 0x0f));

/**
\brief Used to produce smooth analog driving control values from analog inputs.
@see PxSmoothDigitalRawInputsAndSetVehicleAnalogInputs, PxSmoothAnalogRawInputsAndSetVehicleAnalogInputs
*/
class PxVehicleRawInputData
{
public:

	PxVehicleRawInputData()
	{
		for(PxU32 i=0;i<PxVehicleControlInputs::eMAX_NUM_VEHICLE_ANALOG_INPUTS+PxVehicleControlInputs::eMAX_NUM_VEHICLE_DIGITAL_INPUTS;i++)
		{
			mRawDigitalInputs[i]=false;
		}
		for(PxU32 i=0;i<PxVehicleControlInputs::eMAX_NUM_VEHICLE_ANALOG_INPUTS;i++)
		{
			mRawAnalogInputs[i]=0.0f;
		}
	}

	~PxVehicleRawInputData()
	{
	}

	void setDigitalAccel(const bool accelKeyPressed)			{mRawDigitalInputs[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_ACCEL]=accelKeyPressed;}
	void setDigitalBrake(const bool brakeKeyPressed)			{mRawDigitalInputs[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_BRAKE]=brakeKeyPressed;}
	void setDigitalHandbrake(const bool handbrakeKeyPressed)	{mRawDigitalInputs[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_HANDBRAKE]=handbrakeKeyPressed;}
	void setDigitalSteerLeft(const bool steerLeftKeyPressed)	{mRawDigitalInputs[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_LEFT]=steerLeftKeyPressed;}
	void setDigitalSteerRight(const bool steerRightKeyPressed)	{mRawDigitalInputs[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_RIGHT]=steerRightKeyPressed;}
	void setAnalogAccel(const PxReal accel)						{mRawAnalogInputs[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_ACCEL]=accel;}
	void setAnalogBrake(const PxReal brake)						{mRawAnalogInputs[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_BRAKE]=brake;}
	void setAnalogHandbrake(const PxReal handbrake)				{mRawAnalogInputs[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_HANDBRAKE]=handbrake;}
	void setAnalogSteer(const PxReal steer)						{mRawAnalogInputs[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_LEFT]=steer;}
	void setGearUp(const bool gearUpKeyPressed)					{mRawDigitalInputs[PxVehicleControlInputs::eMAX_NUM_VEHICLE_ANALOG_INPUTS+PxVehicleControlInputs::eVEHICLE_DIGITAL_INPUT_GEAR_UP]=gearUpKeyPressed;}
	void setGearDown(const bool gearDownKeyPressed)				{mRawDigitalInputs[PxVehicleControlInputs::eMAX_NUM_VEHICLE_ANALOG_INPUTS+PxVehicleControlInputs::eVEHICLE_DIGITAL_INPUT_GEAR_DOWN]=gearDownKeyPressed;}

	bool getDigitalAccel() const								{return mRawDigitalInputs[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_ACCEL];}
	bool getDigitalBrake() const								{return mRawDigitalInputs[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_BRAKE];}
	bool getDigitalHandbrake() const							{return mRawDigitalInputs[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_HANDBRAKE];}
	bool getDigitalSteerLeft() const							{return mRawDigitalInputs[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_LEFT];}
	bool getDigitalSteerRight() const							{return mRawDigitalInputs[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_RIGHT];}
	PxReal getAnalogAccel() const								{return mRawAnalogInputs[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_ACCEL];}
	PxReal getAnalogBrake() const								{return mRawAnalogInputs[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_BRAKE];}
	PxReal getAnalogHandbrake() const							{return mRawAnalogInputs[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_HANDBRAKE];}
	PxReal getAnalogSteer() const								{return mRawAnalogInputs[PxVehicleControlInputs::eVEHICLE_ANALOG_INPUT_STEER_LEFT];}
	bool getGearUp() const										{return mRawDigitalInputs[PxVehicleControlInputs::eMAX_NUM_VEHICLE_ANALOG_INPUTS+PxVehicleControlInputs::eVEHICLE_DIGITAL_INPUT_GEAR_UP];}
	bool getGearDown() const									{return mRawDigitalInputs[PxVehicleControlInputs::eMAX_NUM_VEHICLE_ANALOG_INPUTS+PxVehicleControlInputs::eVEHICLE_DIGITAL_INPUT_GEAR_DOWN];}

public:

	bool mRawDigitalInputs[PxVehicleControlInputs::eMAX_NUM_VEHICLE_ANALOG_INPUTS+PxVehicleControlInputs::eMAX_NUM_VEHICLE_DIGITAL_INPUTS];
	PxReal mRawAnalogInputs[PxVehicleControlInputs::eMAX_NUM_VEHICLE_ANALOG_INPUTS];
};

/**
\brief Used to smooth and set analog driving control values (accel,brake,handbrake,steer) from digital inputs (keyboard).
 Also used to set boolean gearup, geardown values.
@see PxSmoothDigitalRawInputsAndSetVehicleAnalogInputs, PxSmoothAnalogRawInputsAndSetVehicleAnalogInputs
*/
void PxVehicle4WSmoothDigitalRawInputsAndSetAnalogInputs
	(const PxVehicleKeySmoothingData& keySmoothing, const PxFixedSizeLookupTable<8>& steerVsForwardSpeedTable,
	const PxVehicleRawInputData& rawInputData, 
	const PxReal timestep, 
	PxVehicle4W& focusVehicle);

/**
\brief Used to smooth and set analog driving control values from analog inputs (gamepad).
Also used to set boolean gearup, geardown values.
@see PxSmoothDigitalRawInputsAndSetVehicleAnalogInputs, PxSmoothAnalogRawInputsAndSetVehicleAnalogInputs
*/
void PxVehicle4WSmoothAnalogRawInputsAndSetAnalogInputs
	(const PxVehiclePadSmoothingData& padSmoothing, const PxFixedSizeLookupTable<8>& steerVsForwardSpeedTable,
	const PxVehicleRawInputData& rawInputData, 
	const PxReal timestep, 
	PxVehicle4W& focusVehicle);

/*
\brief Make sure that suspension raycasts only consider shapes flagged as drivable that don't belong to the owner vehicle.
*/	
enum
{
	PX_DRIVABLE_SURFACE = 0xffff0000
};

static PxSceneQueryHitType::Enum PxWheelRaycastPreFilter(	
	PxFilterData filterData0, 
	PxFilterData filterData1,
	const void* constantBlock, PxU32 constantBlockSize,
	PxSceneQueryFilterFlags& filterFlags)
{
	PX_UNUSED(constantBlock);
	PX_UNUSED(constantBlockSize);
	PX_UNUSED(filterFlags);
	//filterData0 is the vehicle suspension raycast.
	//filterData1 is the shape potentially hit by the raycast.
	return (  ((0 == (filterData1.word3 & PX_DRIVABLE_SURFACE)) || ((filterData0.word3 & ~PX_DRIVABLE_SURFACE) == (filterData1.word3 & ~PX_DRIVABLE_SURFACE))) ? PxSceneQueryHitType::eNONE : PxSceneQueryHitType::eBLOCK);
}


/**
\brief Set up query filter data so that vehicles can drive on shapes with this filter data.
\brief Note that we have reserved word3 of the PxFilterData for vehicle raycast query filtering.
*/
void PxSetupDrivableShapeQueryFilterData(PxFilterData* qryFilterData);

/**
\brief Set up query filter data so that vehicles cannot drive on shapes with this filter data.
\brief Note that we have reserved word3 of the PxFilterData for vehicle raycast query filtering.
*/
void PxSetupNonDrivableShapeQueryFilterData(PxFilterData* qryFilterData);

/**
\brief Set up query filter data for the shapes of a vehicle to ensure that vehicles cannot drive on themselves 
but can drive on the shapes of other vehicles.
\brief vehIndex must be greater than zero and less than 65535.
\brief Note that we have reserved word3 of the PxFilterData for vehicle raycast query filtering.
*/
void PxSetupVehicleShapeQueryFilterData(const PxU32 vehIndex, PxFilterData* qryFilterData);

/**
\brief Data structure for quick setup of scene queries for suspension raycasts.
@see PxVehicle4WSetUpSceneQuery, PxVehicle4WSuspensionRaycasts
*/
template<PxU32 NUM_VEHICLES> class PxVehicle4WSceneQueryData
{
public:

	PxVehicle4WSceneQueryData()
		: mPreFilterShader(PxWheelRaycastPreFilter),
		  mSpuPreFilterShader(NULL),
		  mSpuPreFilterShaderSize(0)
	{
	}

	/**
	\brief One result for each wheel.
	*/
	PxRaycastQueryResult PX_ALIGN(16, mSqResults[NUM_VEHICLES*PxVehicle4WSimulationData::eNUM_WHEELS]);

	/**
	\brief One hit for each wheel.
	*/
	PxRaycastHit PX_ALIGN(16, mSqHitBuffer[NUM_VEHICLES*PxVehicle4WSimulationData::eNUM_WHEELS]);

	/**
	\brief Filter shader used to filter drivable and non-drivable surfaces
	*/
	PxBatchQueryPreFilterShader mPreFilterShader;

	/**
	\brief Ptr to compiled spu filter shader 
	\brief Set this on ps3 for spu raycasts
	*/
	void* mSpuPreFilterShader;

	/**
	\brief Size of compiled spu filter shader 
	\brief Set this on ps3 for spu raycasts.
	*/
	PxU32 mSpuPreFilterShaderSize;
};

/**
\brief Quick setup of scene queries for suspension raycasts.
@see PxVehicle4WSceneQueryData, PxVehicle4WSuspensionRaycasts
*/
template <PxU32 NUM_VEHICLES> PxBatchQuery* PxVehicle4WSetUpSceneQuery(PxScene* scene, PxVehicle4WSceneQueryData<NUM_VEHICLES>& data);


#if PX_DEBUG_VEHICLE_ON 

struct PxVehicleGraphDesc
{
	/**
	\brief x-coord of graph centre.
	<b>Range:</b> (0,1)<br>
	*/	
	PxReal mPosX;

	/**
	\brief y-coord of graph centre.
	<b>Range:</b> (0,1)<br>
	*/	
	PxReal mPosY;

	/**
	\brief x-extents of graph (from mPosX-0.5f*mSizeX to mPosX+0.5f*mSizeX).
	<b>Range:</b> (0,1)<br>
	*/	
	PxReal mSizeX;

	/**
	\brief y-extents of graph (from mPosY-0.5f*mSizeY to mPosY+0.5f*mSizeY).
	<b>Range:</b> (0,1)<br>
	*/	
	PxReal mSizeY;

	/**
	\brief Background colour of graph.
	*/	
	PxVec3 mBackgroundColour;

	/**
	\brief Alpha value of background colour.
	*/	
	PxReal mAlpha;
};

struct PxVehicleGraphChannelDesc
{
	/**
	\brief Data values less than mMinY will be clamped at mMinY.
	*/	
	PxReal mMinY;

	/**
	\brief Data values greater than mMaxY will be clamped at mMaxY.
	*/	
	PxReal mMaxY;

	/**
	\brief Data values greater than mMidY will be drawn with colour mColourHigh.
	\brief Data values less than mMidY will be drawn with colour mColourLow.
	*/	
	PxReal mMidY;

	/**
	\brief Colour used to render data values lower than mMidY.
	*/	
	PxVec3 mColourLow;

	/**
	\brief Colour used to render data values greater than mMidY.
	*/	
	PxVec3 mColourHigh;

	/**
	\brief String to describe data channel.
	*/	
	char* mTitle;
};

class PxVehicleGraph
{
public:

	enum
	{
		eMAX_NUM_SAMPLES=256
	};

	enum
	{
		eMAX_NUM_TITLE_CHARS=256
	};

	enum
	{
		eCHANNEL_JOUNCE=0,
		eCHANNEL_SUSPFORCE,
		eCHANNEL_TYRELOAD,
		eCHANNEL_NORMALISED_TYRELOAD,
		eCHANNEL_WHEEL_OMEGA, 
		eCHANNEL_TYRE_FRICTION,
		eCHANNEL_TYRE_LONG_SLIP,
		eCHANNEL_NORM_TYRE_LONG_FORCE,
		eCHANNEL_TYRE_LAT_SLIP,
		eCHANNEL_NORM_TYRE_LAT_FORCE,
		eCHANNEL_NORM_TYRE_ALIGNING_MOMENT,
		eMAX_NUM_WHEEL_CHANNELS
	};

	enum
	{
		eCHANNEL_ENGINE_REVS=0,
		eCHANNEL_ENGINE_DRIVE_TORQUE,
		eCHANNEL_CLUTCH_SLIP,
		eCHANNEL_ACCEL_CONTROL,
		eCHANNEL_BRAKE_CONTROL,
		eCHANNEL_HANDBRAKE_CONTROL,
		eCHANNEL_STEER_CONTROL,
		eCHANNEL_GEAR_RATIO,
		eMAX_NUM_ENGINE_CHANNELS
	};

	enum
	{
		eMAX_NUM_CHANNELS=12
	};

	enum eGraphType
	{
		eGRAPH_TYPE_WHEEL=0,
		eGRAPH_TYPE_ENGINE
	};

	PxVehicleGraph();
	~PxVehicleGraph();

	void setup(const PxVehicleGraphDesc& desc, const eGraphType graphType);

	void setChannel(PxVehicleGraphChannelDesc& desc, const PxU32 channel);

	void clearRecordedChannelData();

	void updateTimeSlice(const PxReal* const samples);

	const PxVec3& getBackgroundColor() const {return mBackgroundColour;}
	PxReal getBackgroundAlpha() const {return mBackgroundAlpha;}
	void getBackgroundCoords(PxReal& xMin, PxReal& yMin, PxReal& xMax, PxReal& yMax) const {xMin = mBackgroundMinX;xMax = mBackgroundMaxX;yMin = mBackgroundMinY;yMax = mBackgroundMaxY;}

	void computeGraphChannel(const PxU32 channel, PxReal* xy, PxVec3* colours, char* title) const;

private:

	//Background colour,alpha,coords
	PxVec3 mBackgroundColour;
	PxReal mBackgroundAlpha;
	PxReal mBackgroundMinX;
	PxReal mBackgroundMaxX;
	PxReal mBackgroundMinY;
	PxReal mBackgroundMaxY;

	PxU32 mSampleTide;

	PxU32 mNumChannels;
	PxReal mPad;

	//Min and max of each sample.
	PxReal mChannelMinY[eMAX_NUM_CHANNELS];
	PxReal mChannelMaxY[eMAX_NUM_CHANNELS];
	//Discriminate between high and low values with different colours.
	PxReal mChannelMidY[eMAX_NUM_CHANNELS];
	//Different colours for values than midY and less than midY.
	PxVec3 mChannelColourLow[eMAX_NUM_CHANNELS];
	PxVec3 mChannelColourHigh[eMAX_NUM_CHANNELS];
	//Title of graph
	char mChannelTitle[eMAX_NUM_CHANNELS][eMAX_NUM_TITLE_CHARS];
	//Graph data.
	PxReal mChannelSamples[eMAX_NUM_CHANNELS][eMAX_NUM_SAMPLES];
};
PX_COMPILE_TIME_ASSERT(PxU32(PxVehicleGraph::eMAX_NUM_CHANNELS) >= PxU32(PxVehicleGraph::eMAX_NUM_WHEEL_CHANNELS) && PxU32(PxVehicleGraph::eMAX_NUM_CHANNELS) >= PxU32(PxVehicleGraph::eMAX_NUM_ENGINE_CHANNELS));

struct PxVehicle4WTelemetryData
{
	/**
	\brief Graph data for each wheel.
	\brief Used for storing single timeslices of debug data for wheel graphs.
	@see PxVehicleGraph
	*/
	PxVehicleGraph mWheelGraphs[PxVehicle4WSimulationData::eNUM_WHEELS];

	/**
	\brief Graph data for engine.
	\brief Used for storing single timeslices of debug data for engine graph.
	@see PxVehicleGraph
	*/
	PxVehicleGraph mEngineGraph;

	/**
	\brief Application point of tyre forces.
	*/
	PxVec3 mTyreforceAppPoints[PxVehicle4WSimulationData::eNUM_WHEELS];

	/**
	\brief Application point of susp forces.
	*/
	PxVec3 mSuspforceAppPoints[PxVehicle4WSimulationData::eNUM_WHEELS];
};

/**
\brief Set up all the graphs so that they are ready to record data.
*/
void PxVehicle4WSetupTelemetryData
	(const PxReal graphSizeX, const PxReal graphSizeY,
	 const PxReal engineGraphPosX, const PxReal engineGraphPosY,
	 const PxReal* const wheelGraphPosX, const PxReal* const wheelGraphPosY,
	 const PxVec3 backGroundColour, const PxVec3& lineColourHigh, const PxVec3& lineColourLow,
	 PxVehicle4WTelemetryData& telemetryData);

/**
\brief Clear the graphs of recorded data.
*/
void PxVehicle4WClearTelemetryData(PxVehicle4WTelemetryData& telemetryData);

/**
\brief Update the focus vehicle and also store key debug data for the specified focus vehicle.
*/
void PxVehicle4WUpdateSingleVehicleAndStoreTelemetryData(const PxReal timestep, const PxVec3& gravity, const PxVehicleDrivableSurfaceToTyreFrictionPairs& vehicleDrivableSurfaceToTyreFrictionPairsconst, PxVehicle4W* focusVehicle, PxVehicle4WTelemetryData& telemetryData);

#endif

template <PxU32 NUM_VEHICLES> PxBatchQuery* PxVehicle4WSetUpSceneQuery(PxScene* scene, PxVehicle4WSceneQueryData<NUM_VEHICLES>& data)
{
	PxBatchQueryDesc sqDesc;
	sqDesc.userRaycastResultBuffer = data.mSqResults;
	sqDesc.userRaycastHitBuffer = data.mSqHitBuffer;
	sqDesc.raycastHitBufferSize = NUM_VEHICLES*PxVehicle4WSimulationData::eNUM_WHEELS;
	sqDesc.preFilterShader = data.mPreFilterShader;
	sqDesc.spuPreFilterShader = data.mSpuPreFilterShader;
	sqDesc.spuPreFilterShaderSize = data.mSpuPreFilterShaderSize;
	return scene->createBatchQuery(sqDesc);
}

#ifndef PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif //PX_VEHICLE_EXT_H

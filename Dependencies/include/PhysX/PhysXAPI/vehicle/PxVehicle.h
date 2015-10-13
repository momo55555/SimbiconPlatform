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
#ifndef PX_VEHICLE_H
#define PX_VEHICLE_H
/** \addtogroup vehicle
  @{
*/

#include "common/PxPhysXCommon.h"
#include "common/PxCoreUtilityTypes.h"
#include "PxVehicleSuspLimitConstraintShader.h"
#include "PxVec3.h"
#include "PxTransform.h"
#include "PxFiltering.h"
#include "PxBatchQuery.h"
#include "PxSceneQueryReport.h"
#include "PxBatchQueryDesc.h"
#include "PxConstraintDesc.h"
#include "extensions/PxConstraintExt.h"

#ifndef PX_DOXYGEN
namespace physx
{
#endif

class PxRigidDynamic;
class PxPhysics;

class PxVehicleChassisData
{
public:
	PxVec3 mMOI;
	PxReal mMass;
	PxVec3 mCMOffset;
private:
	PxReal pad;
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleChassisData)& 0x0f));

class PxVehicleEngineData
{
public:

	friend class PxVehicle4WSimulationData;

	/**
	\brief Graph of normalised torque (torque/maxTorque) against normalised engine revs (revs/maxRevs).
	*/
	enum
	{
		eMAX_NUM_ENGINE_TORQUE_CURVE_ENTRIES = 8
	};
	PxFixedSizeLookupTable<eMAX_NUM_ENGINE_TORQUE_CURVE_ENTRIES> mTorqueCurve;

	/**
	\brief Maximum torque available to apply to the engine, specified in Nm.
	<b>Range:</b> (0,inf)<br>
	*/
	PxReal mPeakTorque;

	/**
	\brief Maximum rotation speed of the engine, specified in radians per second.
	<b>Range:</b> (0,inf)<br>
	*/
	PxReal mMaxOmega;

	/**
	\brief Damping rate of engine when the clutch is engaged, specified in s^-1.
	<b>Range:</b> (0,inf)<br>
	*/
	PxReal mEngagedClutchDampingRate;

	/**
	\brief Damping rate of engine when the clutch is not engaged, specified in s^-1.
	<b>Range:</b> (0,inf)<br>
	*/
	PxReal mDisengagedClutchDampingRate;


	PX_FORCE_INLINE PxReal getRecipMaxOmega() const {return mRecipMaxOmega;}

private:

	/**
	\brief Reciprocal of the maximum rotation speed of the engine.
	Not necessary to set this value.
	<b>Range:</b> (0,inf)<br>
	*/
	PxReal mRecipMaxOmega;

	PxReal mPad[3];
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleEngineData)& 0x0f));

class PxVehicleWheelData
{
public:

	friend class PxVehicle4WSimulationData;

	/**
	\brief Radius of unit that includes metal wheel plus rubber tyre, specified in m.
	<b>Range:</b> (0,inf)<br>
	*/
	PxReal mRadius;

	/**
	\brief Maximum width of unit that includes wheel plus tyre, specified in m.
	<b>Range:</b> (0,inf)<br>
	*/
	PxReal mWidth;

	/**
	\brief Mass of unit that includes wheel plus tyre, specified in kg.
	<b>Range:</b> (0,inf)<br>
	*/
	PxReal mMass;

	/**
	\brief Moment of inertia of unit that includes wheel plus tyre about single allowed axis of rotation, specified in kg m^2.
	<b>Range:</b> (0,inf)<br>
	*/
	PxReal mMOI;

	/**
	\brief Damping rate applied to wheel.
	*/
	PxReal mDampingRate;

	/**
	\brief Max brake torque that can be applied to wheel, specified in Nm.
	<b>Range:</b> (0,inf)<br>
	*/
	PxReal mMaxBrakeTorque;

	/**
	\brief Max handbrake torque that can be applied to wheel, specified in Nm
	<b>Range:</b> (0,inf)<br>
	*/
	PxReal mMaxHandBrakeTorque;

	/**
	\brief Max steer angle that can be achieved by the wheel, specified in radians.
	<b>Range:</b> (0,inf)<br>
	*/
	PxReal mMaxSteer;

	/**
	\brief Wheel toe angle, specified in radians.
	<b>Range:</b> (0,Pi/2)<br>
	*/
	PxReal mToeAngle;//in radians


	PX_FORCE_INLINE PxReal getRecipRadius() const {return mRecipRadius;}
	PX_FORCE_INLINE PxReal getRecipMOI() const {return mRecipMOI;}

private:

	/**
	\brief Reciprocal of radius of unit that includes metal wheel plus rubber tyre.
	Not necessary to set this value.
	<b>Range:</b> (0,inf)<br>
	*/
	PxReal mRecipRadius;

	/**
	\brief Reciprocal of moment of inertia of unit that includes wheel plus tyre about single allowed axis of rotation.
	Not necessary to set this value.
	<b>Range:</b> (0,inf)<br>
	*/
	PxReal mRecipMOI;

	PxReal mPad[1];
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleWheelData)& 0x0f));

class PxVehicleSuspensionData
{
public:

	/**
	\brief Spring strength of suspension unit, specified in N m^-1.
	<b>Range:</b> (0,inf)<br>
	*/
	PxReal mSpringStrength;

	/**
	\brief Spring damper rate of suspension unit, specified in s^-1.
	<b>Range:</b> (0,inf)<br>
	*/
	PxReal mSpringDamperRate;

	/**
	\brief Maximum compression allowed by suspension spring, specified in m.
	<b>Range:</b> (0,inf)<br>
	*/
	PxReal mMaxCompression;

	/**
	\brief Maximum elongation allowed by suspension spring, specified in m.
	<b>Range:</b> (0,inf)<br>
	*/
	PxReal mMaxDroop;

	/**
	\brief Mass of vehicle that is supported by suspension spring, specified in kg.
	<b>Range:</b> (0,inf)<br>
	*/
	PxReal mSprungMass;

private:

	PxReal mPad[3];
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleSuspensionData)& 0x0f));

class PxVehicleTyreData
{
public:

	friend class PxVehicle4WSimulationData;

	/**
	\brief Tyre lateral stiffness is typically a graph of tyre load that has linear behaviour near zero load and 
	flattens at large loads.  mLatStiffX describes the minimum normalised load (load/restLoad) 
	that gives a flat lateral stiffness response.
	<b>Range:</b> (0,inf)<br>
	*/
	PxReal mLatStiffX;

	/**
	\brief Tyre lateral stiffness is a graph of tyre load that has linear behaviour near zero load and 
	flattens at large loads.  mLatStiffY describes the maximum possible lateral stiffness 
	divided by the rest tyre load, specified in "per radian"
	<b>Range:</b> (0,inf)<br>
	*/
	PxReal mLatStiffY;

	/**
	\brief Tyre Longitudinal stiffness of the tyre, specified in N per radian.
	<b>Range:</b> (0,inf)<br>
	*/
	PxReal mLongitudinalStiffness;

	/**
	\brief Camber stiffness, specified in N per radian.
	<b>Range:</b> (0,inf)<br>
	*/
	PxReal mCamberStiffness;

	/**
	\brief Graph of friction vs longitudinal slip with 3 points. 
	\brief point[0][0] is always zero.
	\brief point[0][1] is the friction available at zero longitudinal slip.
	\brief point[1][0] is the value of longitudinal slip with maximum friction.
	\brief point[1][1] is the maximum friction.
	\brief point[2][0] is the end point of the graph.
	\brief point[2][1] is the value of friction for slips greater than point[2][0]
	<b>Range:</b> (0,inf)<br>
	*/
	PxReal mFrictionVsSlipGraph[3][2];

	/**
	\brief Tyre type denoting slicks, wets, snow, winter, summer, all-terrain, mud etc.
	<b>Range:</b> (0,inf)<br>
	*/
	PxU32 mType;

	PX_FORCE_INLINE PxReal getRecipLongitudinalStiffness() const {return mRecipLongitudinalStiffness;}
	PX_FORCE_INLINE PxReal getFrictionVsSlipGraphRecipx1Minusx0() const {return mFrictionVsSlipGraphRecipx1Minusx0;}
	PX_FORCE_INLINE PxReal getFrictionVsSlipGraphRecipx2Minusx1() const {return mFrictionVsSlipGraphRecipx2Minusx1;}

private:

	PxReal mRecipLongitudinalStiffness;
	PxReal mFrictionVsSlipGraphRecipx1Minusx0;
	PxReal mFrictionVsSlipGraphRecipx2Minusx1;

	PxReal mPad[2];
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleTyreData)& 0x0f));

/**
\brief Tyre load can be strongly dependent on the timestep so it is a good idea to filter it 
to give less jerky handling behaviour.  The filtered tyre load is used as an input to the tyre model.
\brief Two points on graph with normalised tyre load on x-axis and filtered normalised tyre load on y-axis.
\brief Loads less than mMinNormalisedLoad have filtered normalised load = zero.
\brief Loads greater than mMaxNormalisedLoad have filtered normalised load = mMaxFilteredNormalisedLoad.
\brief Loads in-between are linearly interpolated between 0 and mMaxFilteredNormalisedLoad.
\brief The two graphs points that we specify are (mMinNormalisedLoad,0) and (mMaxNormalisedLoad,mMaxFilteredNormalisedLoad).
*/
class PxVehicleTyreLoadFilterData
{
public:

	friend class PxVehicle4WSimulationData;

	/**
	\brief Graph point (mMinNormalisedLoad,0)
	*/
	PxReal mMinNormalisedLoad; 

	/**
	\brief Graph point (mMaxNormalisedLoad,mMaxFilteredNormalisedLoad)
	*/
	PxReal mMaxNormalisedLoad;
		
	/**
	\brief Graph point (mMaxNormalisedLoad,mMaxFilteredNormalisedLoad)
	*/
	PxReal mMaxFilteredNormalisedLoad;

	PX_FORCE_INLINE PxReal getDenominator() const {return mDenominator;}

private:

	/**
	\brief Not necessary to set this value.
	*/
	//1.0f/(mMaxNormalisedLoad-mMinNormalisedLoad) for quick calculations
	PxReal mDenominator;
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleTyreLoadFilterData)& 0x0f));

class PxVehicleGearsData
{
public:

	enum
	{
		eREVERSE=0,
		eNEUTRAL,
		eFIRST,
		eSECOND,
		eTHIRD,
		eFOURTH,
		eFIFTH,
		eSIXTH,
		eMAX_NUM_GEAR_RATIOS
	};

	/**
	\brief Gear ratios 
	<b>Range:</b> (0,inf)<br>
	*/
	PxReal mRatios[eMAX_NUM_GEAR_RATIOS];

	/**
	\brief Gear ratio applied is mRatios[currentGear]*finalRatio
	<b>Range:</b> (0,inf)<br>
	*/
	PxReal mFinalRatio;

	/**
	\brief Number of gears (including reverse and neutral).
	<b>Range:</b> (0,MAX_NUM_GEAR_RATIOS)<br>
	*/
	PxU32 mNumRatios;
	
	/**
	\brief Time it takes to switch gear, specified in s.
	<b>Range:</b> (0,MAX_NUM_GEAR_RATIOS)<br>
	*/
	PxReal mSwitchTime;

private:

	PxReal mPad;
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleGearsData)& 0x0f));

class PxVehicleClutchData
{
public:

	/**
	\brief Strength of clutch
	<b>Range:</b> (0,MAX_NUM_GEAR_RATIOS)<br>
	*/
	PxReal mStrength;

private:

	PxReal mPad[3];
};

PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleClutchData)& 0x0f));

class PxVehicleAutoBoxData
{
public:
	/**
	\brief Value of engineRevs/maxEngineRevs that is high enough to increment gear.
	<b>Range:</b> (0,1)<br>
	*/
	PxReal mUpRatios[PxVehicleGearsData::eMAX_NUM_GEAR_RATIOS];

	/**
	\brief Value of engineRevs/maxEngineRevs that is low enough to decrement gear.
	<b>Range:</b> (0,1)<br>
	*/
	PxReal mDownRatios[PxVehicleGearsData::eMAX_NUM_GEAR_RATIOS];

	/**
	\brief Latency time of gearbox stored in mDownRatios[REVERSE], specified in s.
	*/
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleAutoBoxData)& 0x0f));

class PxVehicleDifferential4WData
{
public:

	enum
	{
		eDIFF_TYPE_LS_4WD,
		eDIFF_TYPE_LS_FRONTWD,
		eDIFF_TYPE_LS_REARWD,
		eDIFF_TYPE_OPEN_4WD,
		eDIFF_TYPE_OPEN_FRONTWD,
		eDIFF_TYPE_OPEN_REARWD,
		eMAX_NUM_DIFF_TYPES
	};

	/**
	\brief Ratio of torque split between front and rear (>0.5 means more to front, <0.5 means more to rear).
	\brief Only applied to DIFF_TYPE_LS_4WD and eDIFF_TYPE_OPEN_4WD
	<b>Range:</b> (0,1)<br>
	*/
	PxReal mFrontRearSplit;

	/**
	\brief Maximum allowed ratio of average front wheel rotation speed and rear wheel rotation speeds 
	\brief Only applied to DIFF_TYPE_LS_4WD
	<b>Range:</b> (1,inf)<br>
	*/
	PxReal mCentreBias;

	/**
	\brief Maximum allowed ratio of front-left and front-right wheel rotation speeds.
	\brief Only applied to DIFF_TYPE_LS_4WD and DIFF_TYPE_LS_FRONTWD
	<b>Range:</b> (1,inf)<br>
	*/
	PxReal mFrontBias;

	/**
	\brief Maximum allowed ratio of rear-left and rear-right wheel rotation speeds.
	\brief Only applied to DIFF_TYPE_LS_4WD and DIFF_TYPE_LS_REARWD
	<b>Range:</b> (1,inf)<br>
	*/
	PxReal mRearBias;

	/**
	\brief Type of differential.
	<b>Range:</b> (DIFF_TYPE_LS_4WD,DIFF_TYPE_OPEN_FRONTWD)<br>
	*/
	PxU32 mType;

private:

	PxReal mPad[3];
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleDifferential4WData)& 0x0f));

class PxVehicleAckermannGeometryData
{
public:

	friend class PxVehicle4WSimulationData;

	/**
	\brief Accuracy of Ackermann steer calculation.
	<b>Range:</b> (0,1)<br>
	*/		
	PxReal mAccuracy;

	PX_FORCE_INLINE PxReal getFrontWidth() const {return mFrontWidth;}
	PX_FORCE_INLINE PxReal getAxleSeparation() const {return mAxleSeparation;}

private:

	/**
	\brief Distance between centre-point of the two front wheels, specified in m.
	<b>Range:</b> (0,inf)<br>
	*/
	PxReal mFrontWidth;		//Distance between two front wheels

	/**
	\brief Distance between centre of front axle and centre of rear axle, specified in m.
	<b>Range:</b> (0,inf)<br>
	*/
	PxReal mAxleSeparation;	//Distance between front and rear axles

	PxReal mPad;
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleAckermannGeometryData)& 0x0f));

class PxVehicle4WSimulationData
{
public:

	enum
	{
		eFRONT_LEFT_WHEEL=0,
		eFRONT_RIGHT_WHEEL,
		eREAR_LEFT_WHEEL,
		eREAR_RIGHT_WHEEL,
		eNUM_WHEELS
	};

	PX_FORCE_INLINE const PxVehicleChassisData&			getChassisData()							const {return mChassis;}
	PX_FORCE_INLINE const PxVehicleEngineData&			getEngineData()								const {return mEngine;}
	PX_FORCE_INLINE const PxVehicleGearsData&			getGearsData()								const {return mGears;}
	PX_FORCE_INLINE const PxVehicleClutchData&			getClutchData()								const {return mClutch;}
	PX_FORCE_INLINE const PxVehicleAutoBoxData&			getAutoBoxData()							const {return mAutoBox;}
	PX_FORCE_INLINE const PxVehicleDifferential4WData&	getDiffData()								const {return mDiff;}

	PX_FORCE_INLINE const PxVehicleSuspensionData&		getSuspensionData(const PxU32 id)			const {return mSuspensions[id];}
	PX_FORCE_INLINE const PxVehicleWheelData&			getWheelData(const PxU32 id)				const {return mWheels[id];}
	PX_FORCE_INLINE const PxVehicleTyreData&			getTyreData(const PxU32 id)					const {return mTyres[id];}
	PX_FORCE_INLINE const PxVec3&						getSuspTravelDirection(const PxU32 id)		const {return mSuspDownwardTravelDirections[id];}
	PX_FORCE_INLINE const PxVec3&						getSuspForceAppPointOffset(const PxU32 id)	const {return mSuspForceAppPointOffsets[id];}
	PX_FORCE_INLINE const PxVec3&						getTyreForceAppPointOffset(const PxU32 id)	const {return mTyreForceAppPointOffsets[id];}
	PX_FORCE_INLINE const PxVec3&						getWheelCentreOffset(const PxU32 id)		const {return mWheelCentreOffsets[id];}

	PX_FORCE_INLINE const PxVehicleTyreLoadFilterData&	getTyreLoadFilterData()						const {return mNormalisedLoadFilter;}

	PX_FORCE_INLINE const PxReal*						getTyreRestLoadsArray()						const {return mTyreRestLoads;}
	PX_FORCE_INLINE const PxReal*						getRecipTyreRestLoadsArray()					const {return mRecipTyreRestLoads;}

	PX_FORCE_INLINE const PxVehicleAckermannGeometryData& getAckermannGeometryData()				const {return mAckermannGeometry;}

	void setChassisData				(const PxVehicleChassisData& chassis);
	void setEngineData				(const PxVehicleEngineData& engine);
	void setGearsData				(const PxVehicleGearsData& gears);
	void setClutchData				(const PxVehicleClutchData& clutch);
	void setAutoBoxData				(const PxVehicleAutoBoxData& autobox);
	void setDiffData				(const PxVehicleDifferential4WData& diff);
	void setSuspensionData			(const PxVehicleSuspensionData& susp, const PxU32 id);
	void setWheelData				(const PxVehicleWheelData& susp, const PxU32 id);
	void setTyreData				(const PxVehicleTyreData& tyre, const PxU32 id);
	void setSuspTravelDirection		(const PxVec3& dir, const PxU32 id);
	void setSuspForceAppPointOffset	(const PxVec3& offset, const PxU32 id);
	void setTyreForceAppPointOffset	(const PxVec3& offset, const PxU32 id);
	void setWheelCentreOffsets		(const PxVec3& offsetFrontLeft, const PxVec3& offsetFrontRight, const PxVec3& offsetRearLeft, const PxVec3& offsetRearRight);
	void setTyreLoadFilterData		(const PxVehicleTyreLoadFilterData& tyreLoadFilter);
	void setAckermannAccuracy		(const PxReal ackermannAccuracy) {mAckermannGeometry.mAccuracy=ackermannAccuracy;}

private:

	/*
	\brief Chassis simulation data
	@see setChassisData, getChassisData
	*/
	PxVehicleChassisData			mChassis;

	/*
	\brief Engine simulation data
	@see setEngineData, getEngineData
	*/
	PxVehicleEngineData				mEngine;

	/*
	\brief Gear simulation data
	@see setGearsData, getGearsData
	*/
	PxVehicleGearsData				mGears;

	/*
	\brief Clutch simulation data
	@see setClutchData, getClutchData
	*/
	PxVehicleClutchData				mClutch;

	/*
	\brief Autobox simulation data
	@see setAutoboxData, getAutoboxData
	*/
	PxVehicleAutoBoxData			mAutoBox;

	/**
	\brief Differential simulation data
	@see setDiffData, getDiffData
	*/
	PxVehicleDifferential4WData		mDiff;

	/**
	\brief Suspension simulation data
	@see setSuspensionData, getSuspensionData
	*/
	PxVehicleSuspensionData			mSuspensions[eNUM_WHEELS];

	/**
	\brief Wheel simulation data
	@see setWheelData, getWheelData
	*/
	PxVehicleWheelData				mWheels[eNUM_WHEELS];

	/**
	\brief Tyre simulation data
	@see setTyreData, getTyreData
	*/
	PxVehicleTyreData				mTyres[eNUM_WHEELS];

	/**
	\brief Direction of suspension travel, pointing downwards.
	*/
	PxVec3							mSuspDownwardTravelDirections[eNUM_WHEELS];

	/**
	\brief Application point of suspension force specified as an offset from the rigid body centre of mass.
	*/
	PxVec3							mSuspForceAppPointOffsets[eNUM_WHEELS];	//Offset from cm

	/**
	\brief Application point of tyre forces specified as an offset from the rigid body centre of mass.
	*/
	PxVec3							mTyreForceAppPointOffsets[eNUM_WHEELS];	//Offset from cm

	/**
	\brief Position of wheel centre specified as an offset from the rigid body centre of mass.
	*/
	PxVec3							mWheelCentreOffsets[eNUM_WHEELS];		//Offset from cm

	/**
	\brief Graph to filter normalised load
	*/
	PxVehicleTyreLoadFilterData		mNormalisedLoadFilter;

	/** 
	\brief Normalised tyre load on each tyre (load/rest load) at zero suspension jounce under gravity.
	*/
	PxReal							mTyreRestLoads[eNUM_WHEELS];	

	/** 
	\brief Reciprocal normalised tyre load on each tyre at zero suspension jounce under gravity.
	*/
	PxReal							mRecipTyreRestLoads[eNUM_WHEELS];	

	//Data for ackermann steer angle computation.
	PxVehicleAckermannGeometryData	mAckermannGeometry;
};

class PxVehicleControlInputs
{
public:
	enum
	{
		eVEHICLE_ANALOG_INPUT_ACCEL=0,
		eVEHICLE_ANALOG_INPUT_BRAKE,
		eVEHICLE_ANALOG_INPUT_HANDBRAKE,
		eVEHICLE_ANALOG_INPUT_STEER_LEFT,
		eVEHICLE_ANALOG_INPUT_STEER_RIGHT,
		eMAX_NUM_VEHICLE_ANALOG_INPUTS
	};
	enum
	{
		eVEHICLE_DIGITAL_INPUT_GEAR_UP=0,
		eVEHICLE_DIGITAL_INPUT_GEAR_DOWN,
		eMAX_NUM_VEHICLE_DIGITAL_INPUTS
	};
};

class PxVehicle4W
{
public:

	PxVehicle4W()
		: mUseAutoGears(false),
		  mGearUpPressed(false),
		  mGearDownPressed(false),
		  mCurrentGear(PxVehicleGearsData::eNEUTRAL),
		  mTargetGear(PxVehicleGearsData::eNEUTRAL),
		  mActor(NULL),
		  mSqResults(NULL),
		  mGearSwitchTime(0.0f),
		  mAutoBoxSwitchTime(0.0f)
	{
	}

	/**
	\brief Simulation data that models vehicle components
	@see PxVehicle4WSetup
	*/
	PxVehicle4WSimulationData mVehicleSimData;	

	/**
	\brief Analog control values used by vehicle simulation.
	@see PxVehicle4WSetAnalogInputs,PxVehicle4WGetAppliedAccel,PxVehicle4WGetAppliedBrake,PxVehicle4WGetAppliedHandbrake,PxVehicle4WGetAppliedSteer.
	*/
	PxReal mControlAnalogVals[PxVehicleControlInputs::eMAX_NUM_VEHICLE_ANALOG_INPUTS];

	/**
	\brief Autogear flag used by vehicle simulation
	@see PxVehicle4WSetUseAutoGears, PxVehicle4WGetUseAutoGears
	*/
	bool mUseAutoGears;

	/**
	\brief Gearup digital control value used by vehicle simulation
	@see PxVehicle4WSetToRestState, PxVehicle4WGetAppliedGearup, PxVehicle4WGetAppliedGeardown
	*/
	bool mGearUpPressed;

	/**
	\brief Geardown digital control value used by vehicle simulation
	@see PxVehicle4WSetToRestState, PxVehicle4WGetAppliedGearup, PxVehicle4WGetAppliedGeardown
	*/
	bool mGearDownPressed;

	/**
	\brief Current gear 
	@see PxVehicle4WSetToRestState, PxVehicle4WGetCurrentGear, PxVehicle4WGetTargetGear, PxVehicle4WStartGearChange, PxVehicle4WForceGearChange
	*/
	PxU32 mCurrentGear;

	/**
	\brief Target gear (different from current gear if a gear change is underway) 
	@see PxVehicle4WSetToRestState, PxVehicle4WGetCurrentGear, PxVehicle4WGetTargetGear, PxVehicle4WStartGearChange, PxVehicle4WForceGearChange
	*/

	PxU32 mTargetGear;

	/**
	\brief Rigid body actor representing chassis rigid body and composite bound of chassis+wheels shapes used by vehicle simulation.
	@see PxVehicle4WSetup, PxVehicle4WSetToRestState,
	PxVehicle4WComputeForwardSpeed, PxVehicle4WComputeSidewaysSpeed,
	PxVehicle4WGetFrontLeftWheelShape,PxVehicle4WGetFrontRightWheelShape, PxVehicle4WGetRearLeftWheelShape, PxVehicle4WGetRearRightWheelShape
	PxVehicle4WGetRearRightWheelShape, PxVehicle4WGetChassisShapes
	*/
	PxRigidDynamic* mActor;		

	/**
	\brief Set by PxVehicle4WSuspensionRaycasts
	@see PxVehicle4WSuspensionRaycasts
	*/
	const PxRaycastQueryResult* mSqResults;

	/**
	\brief Rotation speeds of wheels and engine
	@see PxVehicle4WSetToRestState, PxVehicle4WGetWheelRotationSpeed, PxVehicle4WGetEngineRotationSpeed
	*/	
	PxReal mInternalDynamics[PxVehicle4WSimulationData::eNUM_WHEELS+1];

	/**
	\brief Timers used to trigger sticky friction to hold the car perfectly at rest. 
	\brief Used only internally.
	*/
	PxReal mTyreLowForwardSpeedTimers[PxVehicle4WSimulationData::eNUM_WHEELS];

	/**
	\brief Reported rotation angle about rolling axis.
	@see PxVehicle4WSetToRestState, PxVehicle4WGetWheelRotationAngle
	*/	
	PxReal mWheelRotationAngles[PxVehicle4WSimulationData::eNUM_WHEELS];

	/**
	\brief Reported compression of each suspension spring
	@see PxVehicle4WGetSuspJounce
	*/	
	PxReal mSuspJounces[PxVehicle4WSimulationData::eNUM_WHEELS];

	/**
	\brief Reported longitudinal slip of each tyre
	@see PxVehicle4WGetTyreLongSlip
	*/	
	PxReal mLongSlips[PxVehicle4WSimulationData::eNUM_WHEELS];

	/**
	\brief Reported lateral slip of each tyre
	@see PxVehicle4WGetTyreLatSlip
	*/	
	PxReal mLatSlips[PxVehicle4WSimulationData::eNUM_WHEELS];

	/**
	\brief Reported friction experienced by each tyre
	@see PxVehicle4WGetTyreFriction
	*/	
	PxReal mTyreFrictions[PxVehicle4WSimulationData::eNUM_WHEELS];

	/**
	\brief Reported surface type experienced by each tyre.
	@see PxVehicle4WGetTyreDrivableSurfaceType
	*/	
	PxU32 mTyreSurfaceTypes[PxVehicle4WSimulationData::eNUM_WHEELS];

	/**
	\brief Reported start point of suspension line raycasts used in more recent scene query.
	@see PxVehicle4WSuspensionRaycasts, PxVehicle4WGetSuspRaycast
	*/
	PxVec3 mSuspLineStarts[PxVehicle4WSimulationData::eNUM_WHEELS];

	/**
	\brief Reported directions of suspension line raycasts used in more recent scene query.
	@see PxVehicle4WSuspensionRaycasts, PxVehicle4WGetSuspRaycast
	*/
	PxVec3 mSuspLineDirs[PxVehicle4WSimulationData::eNUM_WHEELS];

	/**
	\brief Reported lengths of suspension line raycasts used in more recent scene query.
	@see PxVehicle4WSuspensionRaycasts, PxVehicle4WGetSuspRaycast
	*/
	PxReal mSuspLineLengths[PxVehicle4WSimulationData::eNUM_WHEELS];

	/**
	\brief Reported time that has passed since gear change started.
	@see PxVehicle4WSetToRestState
	*/
	PxReal mGearSwitchTime;

	/**
	\brief Reported time that has passed since last autobox gearup/geardown decision.
	@see PxVehicle4WSetToRestState
	*/
	PxReal mAutoBoxSwitchTime;

	/**
	\brief Used only internally.
	*/
	PxVehicleConstraintShader<PxVehicle4WSimulationData::eNUM_WHEELS>& getVehicletConstraintShader() const {return mVehicleConstraints;} 

private:

	//Susp limits and sticky tyre friction for all wheels.
	mutable PxVehicleConstraintShader<PxVehicle4WSimulationData::eNUM_WHEELS> mVehicleConstraints;
};

/**
\brief Driving surface type 
\brief One-to-one correspondence between PxMaterial and PxVehicleDrivableSurfaceType
\brief Store ptr to PxVehicleDrivableSurfaceType instance as PxMaterial::mUserData
\brief Must be 16-byte aligned.
@see PxMaterial
*/	
struct PxVehicleDrivableSurfaceType
{
	enum
	{
		eSURFACE_TYPE_UNKNOWN=0xffffffff
	};
	PxU32 mType;
	PxU32 mPad[3];
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleDrivableSurfaceType)& 0x0f));


/**
\brief Friction for each combination of driving surface type and tyre type.
*/
struct PxVehicleDrivableSurfaceToTyreFrictionPairs
{
	/**
	\brief Ptr to base address of a 2d PxReal array with dimensions [mNumSurfaceTypes][mNumTyreTypes]
	\brief Each element of the array describes the maximum friction provided by a surface type-tyre type combination.
	\brief Ptr must be 16-byte aligned and mNumSurfaceTypes*mNumTyreTypes must be a multiple of 4.
	*/
	PxReal* mPairs;					

	/**
	\brief Total number of different driving surface types.
	\brief Driving surface types stored in PxVehicleDrivableSurfaceType
	*/	
	PxU32 mNumSurfaceTypes;			

	/**
	\brief Total number of different tyre types.
	\brief Tyre types stored in PxVehicleTyreDesc
	*/	
	PxU32 mNumTyreTypes;			//must be a multiple of 16.
};

/**
\brief Return the friction for a specified pair of tyre/drivable surface type pair.
*/
PxReal PxVehicle4WGetTypePairFriction(const PxVehicleDrivableSurfaceToTyreFrictionPairs& frictionPairs, const PxU32 surfaceType, const PxU32 tyreType);

/**
\brief Call this before using any of the vehicle functions.
*/
bool PxInitVehicles(PxPhysics& physics);

/**
\brief Set up a vehicle from data.
*/
#define PX_MAX_NUM_CHASSIS_SHAPES 16

void PxVehicle4WSetup
	(const PxVehicle4WSimulationData& data, 
	 const PxFilterData& vehQryFilterData, PxRigidDynamic* vehActor,
	 const PxGeometry& frontLeftWheelGeom, const PxGeometry& frontRightWheelGeom, const PxGeometry& rearLeftWheelGeom, const PxGeometry& rearRightWheelGeom, PxMaterial* const wheelMaterial, const PxFilterData& wheelCollFilterData,
	 const PxGeometry* const* chassisGeoms, const PxTransform* const chassisLocalPoses, const PxU32 numChassisGeoms, PxMaterial* const chassisMaterial, const PxFilterData& chassisCollFilterData,
	 PxPhysics* physics, 
	 PxVehicle4W* veh);



/**
\brief Start raycasts of all suspension lines.
\brief sceneQueryResults array must have dimensions [numVehicles * PxVehicle4WSimulationData::eNUM_WHEELS] or greater.
*/
void PxVehicle4WSuspensionRaycasts(PxBatchQuery* batchQuery, const PxU32 numVehicles, PxRaycastQueryResult* sceneQueryResults, PxVehicle4W** vehicles);

/**
\brief Update an array of vehicles.
*/
void PxVehicle4WUpdate(const PxReal timestep, const PxVec3& gravity, const PxVehicleDrivableSurfaceToTyreFrictionPairs& vehicleDrivableSurfaceToTyreFrictionPairs, const PxU32 numVehicles, PxVehicle4W** vehicles);

/**
\brief Set analog and digital inputs directly.
\brief accel,brake,handbrake must be in range(0,1)
\brief steer must be in range (-1,1)
\brief Either gearup or gear down must be false
@see PxSmoothDigitalRawInputsAndSetVehicleAnalogInputs, PxSmoothAnalogRawInputsAndSetVehicleAnalogInputs
*/
void PxVehicle4WSetAnalogInputs(const PxReal accel, const PxReal brake, const PxReal handbrake, const PxReal steer, const bool gearup, const bool geardown, PxVehicle4W& vehicle);

/**
\brief Return the accel applied to a vehicle
*/
PxReal PxVehicle4WGetAppliedAccel(const PxVehicle4W& vehicle);

/**
\brief Return the brake applied to a vehicle
*/
PxReal PxVehicle4WGetAppliedBrake(const PxVehicle4W& vehicle);

/**
\brief Return the handbrake applied to a vehicle
*/
PxReal PxVehicle4WGetAppliedHandbrake(const PxVehicle4W& vehicle);

/**
\brief Return the steer applied to a vehicle
*/
PxReal PxVehicle4WGetAppliedSteer(const PxVehicle4W& vehicle);

/**
\brief Return the gearup applied to a vehicle
*/
bool PxVehicle4WGetAppliedGearup(const PxVehicle4W& vehicle);

/**
\brief Return the geardown applied to a vehicle
*/
bool PxVehicle4WGetAppliedGeardown(const PxVehicle4W& vehicle);

/**
\brief Set the flag that will be used to select auto-gears
*/
void PxVehicle4WSetUseAutoGears(const bool useAutoGears, PxVehicle4W& vehicle);

/**
\brief Get the flag status that is used to select auto-gears
*/
bool PxVehicle4WGetUseAutoGears(const PxVehicle4W& vehicle);

/**
\brief Get the current gear
*/
PxU32 PxVehicle4WGetCurrentGear(const PxVehicle4W& vehicle);

/**
\brief Get the target gear
*/
PxU32 PxVehicle4WGetTargetGear(const PxVehicle4W& vehicle);

/**
\brief Start a gear change to a target gear
*/
void PxVehicle4WStartGearChange(const PxU32 targetGear, PxVehicle4W& vehicle);

/**
\brief Force an immediate gear change to a target gear
*/
void PxVehicle4WForceGearChange(const PxU32 targetGear, PxVehicle4W& vehicle);

/**
\brief Set a vehicle to its rest state.
\brief Call after PxVehicle4WSetup
*/
void PxVehicle4WSetToRestState(PxVehicle4W& vehicle);

/**
\brief Test if all wheels are off the ground.
*/
bool PxVehicle4WIsInAir(const PxVehicle4W& vehicle);

/**
\brief Compute the rigid body velocity component along the forward vector.
*/
PxReal PxVehicle4WComputeForwardSpeed(const PxVehicle4W& vehicle);

/**
\brief Compute the rigid body velocity component along the right vector.
*/
PxReal PxVehicle4WComputeSidewaysSpeed(const PxVehicle4W& vehicle);

/**
\brief Return the collision shape for the front left wheel.
*/
PxShape* PxVehicle4WGetFrontLeftWheelShape(const PxVehicle4W& vehicle);

/**
\brief Return the collision shape for the front right wheel.
*/
PxShape* PxVehicle4WGetFrontRightWheelShape(const PxVehicle4W& vehicle);

/**
\brief Return the collision shape for the rear left wheel
*/
PxShape* PxVehicle4WGetRearLeftWheelShape(const PxVehicle4W& vehicle);

/**
\brief Return the collision shape for the rear right wheel
*/
PxShape* PxVehicle4WGetRearRightWheelShape(const PxVehicle4W& vehicle);

/**
\brief Return the number of chassis collision shapes.
*/
PxU32 PxVehicle4WGetNumChassisShapes(const PxVehicle4W& vehicle);

/**
\brief Fill a supplied buffer with the required number of chassis collision shapes.
*/
void PxVehicle4WGetChassisShapes(const PxVehicle4W& vehicle, PxShape** shapeBuffer, const PxU32 numShapes);

/**
\brief Return the rotation speed of a specified wheel.
*/
PxReal PxVehicle4WGetWheelRotationSpeed(const PxVehicle4W& vehicle, const PxU32 wheelIdx);

/**
\brief Return the rotation speed of the engine.
*/
PxReal PxVehicle4WGetEngineRotationSpeed(const PxVehicle4W& vehicle);

/**
\brief Return the rotation angle of the specified wheel about the axis of rolling rotation.
*/
PxReal PxVehicle4WGetWheelRotationAngle(const PxVehicle4W& vehicle, const PxU32 wheelIdx);

/**
\brief Return the susp jounce of the specified suspension.
*/
PxReal PxVehicle4WGetSuspJounce(const PxVehicle4W& vehicle, const PxU32 suspIdx);

/**
\brief Return the longitudinal slip of the specified tyre
*/
PxReal PxVehicle4WGetTyreLongSlip(const PxVehicle4W& vehicle, const PxU32 tyreIdx);

/**
\brief Return the lateral slip of the specified tyre
*/
PxReal PxVehicle4WGetTyreLatSlip(const PxVehicle4W& vehicle, const PxU32 tyreIdx);

/**
\brief Return the friction applied to the specified tyre
*/
PxReal PxVehicle4WGetTyreFriction(const PxVehicle4W& vehicle, const PxU32 tyreIdx);

/**
\brief Return the drivable surface type underneath the specified tyre
*/
PxU32 PxVehicle4WGetTyreDrivableSurfaceType(const PxVehicle4W& vehicle, const PxU32 tyreIdx);

/**
\brief Return the raycast start, direction, and length of the specified suspension line.
*/
void PxVehicle4WGetSuspRaycast(const PxVehicle4W& vehicle, const PxU32 suspIdx, PxVec3& start, PxVec3& dir, PxReal& length);

#ifndef PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif //PX_VEHICLE_H

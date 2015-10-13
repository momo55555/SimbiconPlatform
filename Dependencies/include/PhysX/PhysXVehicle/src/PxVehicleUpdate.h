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
#ifndef PX_VEHICLE_UPDATE_H
#define PX_VEHICLE_UPDATE_H

#include "PxAssert.h"
#include "PxPhysXCommon.h"
#include "PxFiltering.h"
#include "PxSceneQueryReport.h"
#include "PxBatchQuery.h"
#include "PxVehicle.h"

class PxRigidDynamic;

namespace physx
{

template <PxU32 NUM_ELEMENTS> class VectorN
{
public:

	VectorN()
	{
	}
	~VectorN()
	{
	}

	VectorN(const VectorN& src)
	{
		for(PxU32 i=0;i<NUM_ELEMENTS;i++)
		{
			mValues[i]=src.mValues[i];
		}
	}

	VectorN& operator=(const VectorN& src)
	{
		for(PxU32 i=0;i<NUM_ELEMENTS;i++)
		{
			mValues[i]=src.mValues[i];
		}
		return *this;
	}

	PX_FORCE_INLINE PxF32& operator[] (const PxU32 i)
	{
		PX_ASSERT(i<NUM_ELEMENTS);
		return (mValues[i]);
	}

	PX_FORCE_INLINE const PxF32& operator[] (const PxU32 i) const
	{
		PX_ASSERT(i<NUM_ELEMENTS);
		return (mValues[i]);
	}

public:

	PxF32 mValues[NUM_ELEMENTS];
};

template <PxU32 NUM_ELEMENTS> class MatrixNN
{
public:

	MatrixNN()
	{
	}
	MatrixNN(const MatrixNN& src)
	{
		for(PxU32 i=0;i<NUM_ELEMENTS;i++)
		{
			for(PxU32 j=0;j<NUM_ELEMENTS;j++)
			{
				mValues[i][j]=src.mValues[i][j];
			}
		}
	}
	~MatrixNN()
	{
	}

	MatrixNN& operator=(const MatrixNN& src)
	{
		for(PxU32 i=0;i<NUM_ELEMENTS;i++)
		{
			for(PxU32 j=0;j<NUM_ELEMENTS;j++)
			{
				mValues[i][j]=src.mValues[i][j];
			}
		}
		return *this;
	}

	PX_FORCE_INLINE PxF32 get(const PxU32 i, const PxU32 j) const
	{
		PX_ASSERT(i<NUM_ELEMENTS);
		PX_ASSERT(j<NUM_ELEMENTS);
		return mValues[i][j];
	}
	PX_FORCE_INLINE void set(const PxU32 i, const PxU32 j, const PxF32 val)
	{
		PX_ASSERT(i<NUM_ELEMENTS);
		PX_ASSERT(j<NUM_ELEMENTS);
		mValues[i][j]=val;
	}

public:

	PxF32 mValues[NUM_ELEMENTS][NUM_ELEMENTS];
};

template <PxU32 NUM_ELEMENTS> bool isValid(const MatrixNN<NUM_ELEMENTS>& A, const VectorN<NUM_ELEMENTS>& b, const VectorN<NUM_ELEMENTS>& result) 
{
	//r=A*result-b
	VectorN<NUM_ELEMENTS> r;
	for(PxU32 i=0;i<NUM_ELEMENTS;i++)
	{
		r[i]=-b[i];
		for(PxU32 j=0;j<NUM_ELEMENTS;j++)
		{
			r[i]+=A.get(i,j)*result[j];
		}
	}

	PxF32 rLength=0;
	PxF32 bLength=0;
	for(PxU32 i=0;i<NUM_ELEMENTS;i++)
	{
		rLength+=r[i]*r[i];
		bLength+=b[i]*b[i];
	}
	const PxF32 error=PxSqrt(rLength/(bLength+1e-5f));
	return (error<1e-5f);
}

template <PxU32 NUM_ELEMENTS> class MatrixNNLUSolver
{
private:
	
	PxU32 mIndex[NUM_ELEMENTS];
	MatrixNN<NUM_ELEMENTS> mLU;

public:

	MatrixNNLUSolver(){}
	~MatrixNNLUSolver(){}

	//Algorithm taken from Numerical Recipes in Fortran 77, page 38

	void decomposeLU(const MatrixNN<NUM_ELEMENTS>& a)
	{

#define TINY (1.0e-20f)

		//Copy m into result then work exclusively on the result matrix.
		mLU=a;

		//Initialise index swapping values.
		for(PxU32 i=0;i<NUM_ELEMENTS;i++)
		{
			mIndex[i]=0xffffffff;
		}

		PxU32 imax=0;
		PxF32 big,dum,sum;
		PxF32 vv[NUM_ELEMENTS];
		PxF32 d=1.0f;

		for(PxU32 i=0;i<=(NUM_ELEMENTS-1);i++)
		{
			big=0.0f;
			for(PxU32 j=0;j<=(NUM_ELEMENTS-1);j++)
			{
				const PxF32 temp=PxAbs(mLU.get(i,j));
				big = temp>big ? temp : big;
			}
			PX_ASSERT(big!=0.0f);
			vv[i]=1.0f/big;
		}

		for(PxU32 j=0;j<=(NUM_ELEMENTS-1);j++)
		{
			for(PxU32 i=0;i<j;i++)
			{
				PxF32 sum=mLU.get(i,j);
				for(PxU32 k=0;k<i;k++)
				{
					sum-=mLU.get(i,k)*mLU.get(k,j);
				}
				mLU.set(i,j,sum);
			}

			big=0.0f;
			for(PxU32 i=j;i<=(NUM_ELEMENTS-1);i++)
			{
				sum=mLU.get(i,j);
				for(PxU32 k=0;k<j;k++)
				{
					sum-=mLU.get(i,k)*mLU.get(k,j);
				}
				mLU.set(i,j,sum);
				dum=vv[i]*PxAbs(sum);
				if(dum>=big)
				{
					big=dum;
					imax=i;
				}
			}

			if(j!=imax)
			{
				for(PxU32 k=0;k<NUM_ELEMENTS;k++)
				{
					dum=mLU.get(imax,k);
					mLU.set(imax,k,mLU.get(j,k));
					mLU.set(j,k,dum);
				}
				d=-d;
				vv[imax]=vv[j];
			}
			mIndex[j]=imax;

			if(mLU.get(j,j)==0)
			{
				mLU.set(j,j,TINY);
			}

			if(j!=(NUM_ELEMENTS-1))
			{
				dum=1.0f/mLU.get(j,j);
				for(PxU32 i=j+1;i<=(NUM_ELEMENTS-1);i++)
				{
					mLU.set(i,j,mLU.get(i,j)*dum);
				}
			}
		}
	}

	void solve(const VectorN<NUM_ELEMENTS>& input, VectorN<NUM_ELEMENTS>& result) const
	{
		result=input;

		PxU32 ip;
		PxU32 ii=0xffffffff;
		PxF32 sum;

		for(PxU32 i=0;i<NUM_ELEMENTS;i++)
		{
			ip=mIndex[i];
			sum=result[ip];
			result[ip]=result[i];
			if(ii!=-1)
			{
				for(PxU32 j=ii;j<=(i-1);j++)
				{
					sum-=mLU.get(i,j)*result[j];
				}
			}
			else if(sum!=0)
			{
				ii=i;
			}
			result[i]=sum;
		}
		for(PxI32 i=NUM_ELEMENTS-1;i>=0;i--)
		{
			sum=result[i];
			for(PxU32 j=i+1;j<=(NUM_ELEMENTS-1);j++)
			{
				sum-=mLU.get(i,j)*result[j];
			}
			result[i]=sum/mLU.get(i,i);
		}
	}
};

struct TyreForceUpdateDescriptor
{
	//Velocity of rigid body at ground contact point on wheel/tyre.
	PxVec3 mWheelBottomVelocity;

	//Direction of right vector in world space
	PxVec3 mChassisLatDir;

	//Normal of ground plane.
	PxVec3 mHitNorm;

	//Tyre load and normalised tyre load.
	PxF32 mTyreLoad;
	PxF32 mNormalisedTyreLoad;
	PxF32 mRestTyreLoad;//normLoad=load/MG

	//Wheel radius, angular speed, and steer angle.
	PxF32 mWheelRadius;
	PxF32 mRecipWheelRadius;
	PxF32 mWheelOmega;
	PxF32 mWheelSteer;

	//Tyre data.
	PxVehicleTyreData mTyreData;
	PxF32 mFriction;

	//Brake torque applied to wheel.
	PxF32 mBrakeTorque;

	//Accel value.
	PxF32 mAccel;

	//timestep for timers
	PxF32 mTimestep;
};

} //namespace physx

#endif //PX_VEHICLE_UPDATE_H

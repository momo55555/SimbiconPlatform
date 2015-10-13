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


#ifndef PX_FOUNDATION_PSTIME_H
#define PX_FOUNDATION_PSTIME_H

#include "Ps.h"

#if defined PX_LINUX
#include <time.h>
#endif

namespace physx
{
namespace shdfnd3
{

	struct CounterFrequencyToTensOfNanos
	{
		PxU64 mNumerator;
		PxU64 mDenominator;
		CounterFrequencyToTensOfNanos( PxU64 inNum, PxU64 inDenom )
			: mNumerator( inNum )
			, mDenominator( inDenom )
		{
		}

		//quite slow.
		PxU64 toTensOfNanos( PxU64 inCounter ) const
		{
			return ( inCounter * mNumerator ) / mDenominator;
		}
	};

	class Time
	{
	public:
		typedef double Second;
		static const PxU64 sNumTensOfNanoSecondsInASecond = 100000000;
		//This is supposedly guaranteed to not change after system boot
		//regardless of processors, speedstep, etc.
		static const CounterFrequencyToTensOfNanos sCounterFreq;

		static CounterFrequencyToTensOfNanos getCounterFrequency();

		static PxU64 getCurrentCounterValue();

		//SLOW!!
		//Thar be a 64 bit divide in thar!
		static PxU64 getCurrentTimeInTensOfNanoSeconds()
		{
			PxU64 ticks = getCurrentCounterValue();
			return sCounterFreq.toTensOfNanos( ticks );
		}

		Time();
		Second getElapsedSeconds();
		Second peekElapsedSeconds();
		Second getLastTime() const { return mLastTime; }
	private:

		Second  mLastTime;
	};
} // namespace shdfnd3
} // namespace physx

#endif

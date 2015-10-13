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


#ifndef PXC_MEMFETCH_H
#define PXC_MEMFETCH_H

#ifdef __SPU__

#include "../../../ps3/include/CellUtil.h"

#else

namespace physx
{

#ifndef PX_X64
typedef unsigned int PxMemFetchPtr;
#else
typedef PxU64 PxMemFetchPtr;
#endif

struct PxMemFetchSmallBuffer
{
};

struct PxMemFetchSmallBuffer16
{
};

inline void pxMemFetchWait(unsigned int)
{
}

inline void pxMemFetchWaitMask(unsigned int)
{
}

template<typename T> inline T pxMemFetch(PxMemFetchPtr ea, unsigned int channel)
{
	return *(T*)ea;
}

template<typename T> inline T* pxMemFetchAsync(PxMemFetchPtr ea, unsigned int channel, PxMemFetchSmallBuffer&)
{
	return (T*)ea;
}

template<typename T> inline T* pxMemFetchAsync(PxMemFetchPtr ea, unsigned int channel, PxMemFetchSmallBuffer16&)
{
	return (T*)ea;
}

template<typename T> inline T* pxMemFetchAsync(void* ls, PxMemFetchPtr ea, unsigned int size, unsigned int channel)
{
	return (T*)ea;
}

inline void pxMemFetchAlignedAsync(PxMemFetchPtr target, PxMemFetchPtr ea, unsigned int size, unsigned int channel)
{
	PX_ASSERT((target & 0xF) == 0);
	PX_ASSERT((size & 0xF) == 0);
	PX_ASSERT((ea & 0xF) == 0);
	memcpy((void*)target, (const void*)ea, size);
}

}

#endif

#endif
